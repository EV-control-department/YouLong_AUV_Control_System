"""前视YOLO居中、向前接近、一次机体+x BMOVE顶碰；不使用棍子或孔位。"""
import math
import threading
import time

import rclpy
from uv_msgs.msg import DetectionArray
from uv_msgs.action import BasicMotion


class _TargetLost(RuntimeError):
    """仅视觉目标丢失可转入盘心兜底；取消、导航失败等不能混用。"""


class SimpleTurntableTask:
    def __init__(self, node, params):
        self.node, self.params = node, params
        self.log = node.get_logger()
        self._lock = threading.Lock()
        self._box = None
        self._last_stamp = -1
        self._started_stamp = node.get_clock().now().nanoseconds
        self._sub = node.create_subscription(DetectionArray,
            '/perception/detection/front_left', self._on_boxes, 10)

    def destroy(self):
        self.node.destroy_subscription(self._sub)

    def _on_boxes(self, msg):
        stamp = int(msg.header.stamp.sec)*1_000_000_000 + int(msg.header.stamp.nanosec)
        now = self.node.get_clock().now().nanoseconds
        age = float(self.params.get('box_max_age_s', .8))
        with self._lock:
            if (stamp <= self._started_stamp or stamp <= self._last_stamp
                    or not 0 <= (now-stamp)*1e-9 <= age):
                return
            self._last_stamp = stamp
            candidates = []
            width = float(self.params.get('image_width', 640))
            height = float(self.params.get('image_height', 480))
            for detection in msg.detections:
                values = (detection.bbox_x1, detection.bbox_y1,
                          detection.bbox_x2, detection.bbox_y2, detection.confidence)
                if (int(detection.class_id) != int(self.params.get('disk_class_id', 3))
                        or not all(math.isfinite(v) for v in values)
                        or detection.confidence < float(self.params.get('min_confidence', .35))):
                    continue
                x1, y1, x2, y2 = values[:4]
                if not (0 <= x1 < x2 <= width and 0 <= y1 < y2 <= height):
                    continue
                candidates.append(detection)
            best = max(candidates, key=lambda d: d.confidence, default=None)
            self._box = ((stamp, (best.bbox_x1+best.bbox_x2)/2/width,
                          (best.bbox_y1+best.bbox_y2)/2/height,
                          (best.bbox_x2-best.bbox_x1)/width) if best else None)

    def _latest_box(self):
        with self._lock:
            box = self._box
        if box and 0 <= (self.node.get_clock().now().nanoseconds-box[0])*1e-9 <= self.params.get('box_max_age_s', .8):
            return box
        return None

    def _check_running(self):
        if self.node.stopped or not rclpy.ok():
            raise RuntimeError('任务被停止')
        if time.monotonic() >= self._deadline:
            raise RuntimeError('简单转盘总超时')

    def _measured_pose(self):
        with self.node._perception_lock:
            pose, received = self.node._robot_pose, self.node._robot_pose_received
        if (pose is None or len(pose) != 6 or not all(map(math.isfinite, pose))
                or time.monotonic()-received > 1.):
            raise RuntimeError('顶碰所需定位缺失/过期')
        return pose

    def _bump_forward(self):
        """沿body+x MOVE一次；独立核对XY，避免短行程落在全局10cm容差内空完成。"""
        self._check_running()
        distance = float(self.params.get('bump_forward_m', .08))
        tolerance = float(self.params.get('bump_position_tolerance_m', .02))
        pose = self._measured_pose()
        angle = math.radians(pose[5])
        target_x, target_y = pose[0]+distance*math.cos(angle), pose[1]+distance*math.sin(angle)
        end = min(self._deadline, time.monotonic()+float(self.params.get('bump_move_timeout_s', 10.)))
        self.node._publish_body_velocity()  # 结束视觉速度控制，再交由BasicMotion位置闭环。
        self.log.info(f'简单转盘：前向BMOVE顶碰一次，body+x={distance:.3f}m；保持深度/航向，不上移、不转yaw')
        self._check_running()
        remaining = end-time.monotonic()
        if remaining <= 0:
            raise RuntimeError('前向顶碰MOVE预算已耗尽')
        ok, message = self.node._send_action_goal(BasicMotion.Goal.BMOVE,
            [distance, 0., 0., 0.], 'x', timeout=remaining,
            task_context='turntable_simple:前向MOVE顶碰一次')
        if not ok:
            raise RuntimeError(f'前向顶碰MOVE失败：{message}')
        while time.monotonic() < end:
            self._check_running()
            current = self._measured_pose()
            if math.hypot(current[0]-target_x, current[1]-target_y) <= tolerance:
                return
            time.sleep(.05)
        raise RuntimeError('前向顶碰实测位移等待超时；action成功不等于短行程已执行')

    def _preposition(self):
        """Optional disk center in START odom; stop outside it before vision."""
        nominal = self.params.get('disk_pose_odom', [])
        if not nominal:
            return
        if len(nominal) != 4:
            raise ValueError('disk_pose_odom必须为[x,y,z,yaw度]或空列表')
        x, y, z, yaw = map(float, nominal)
        camera = list(map(float, self.params.get(
            'front_camera_center_body', [.230, 0., .076])))
        standoff = float(self.params.get('front_standoff_m', .55))
        motion_timeout = float(self.params.get('motion_timeout_s', 90.))
        if (len(camera) != 3 or not all(math.isfinite(v) for v in [x, y, z, yaw, *camera,
                                                                  standoff, motion_timeout])
                or z < 0 or standoff <= 0 or motion_timeout <= 0):
            raise ValueError('预输入盘心/相机外参须为有限数值，观察距离和运动超时须为正')
        angle = math.radians(yaw)
        ca, sa = math.cos(angle), math.sin(angle)
        target = [x-(standoff+camera[0])*ca+camera[1]*sa,
                  y-(standoff+camera[0])*sa-camera[1]*ca,
                  z-camera[2], yaw]
        if target[2] < 0:
            raise ValueError('预输入盘心换算后观察位z小于0，请检查深度基准')
        self.log.info(f'简单转盘预定位：盘心={nominal}，艇体观察目标={target}')
        if self.params.get('preposition_mode', 'face_path') == 'fixed_yaw':
            self._check_running()
            if not self.node._follow_world_waypoints([target], '转盘观察位微调',
                    min(motion_timeout, self._deadline-time.monotonic()),
                    float(self.params.get('max_world_step_m', .4)), 'fixed_yaw'):
                raise RuntimeError('固定航向转盘观察位定位失败')
        else:
            for command, label in ((BasicMotion.Goal.WTRAVEL, '预输入盘心观察位'),
                                   (BasicMotion.Goal.SET, '观察位归向')):
                self._check_running()
                timeout = min(motion_timeout, self._deadline-time.monotonic())
                ok, message = self.node._send_action_goal(command, target, 'xyzrz', timeout=timeout,
                                                           task_context=f'turntable_simple:{label}')
                if not ok:
                    raise RuntimeError(f'{label}失败：{message}')
        # Never approach using frames captured before/during prepositioning.
        with self._lock:
            self._started_stamp = self.node.get_clock().now().nanoseconds
            self._last_stamp = -1
            self._box = None

    def _validate(self):
        p = self.params
        if p.get('preposition_mode', 'face_path') not in ('face_path', 'fixed_yaw'):
            raise ValueError('preposition_mode仅支持fixed_yaw或face_path')
        for key, default in (('timeout', 45.), ('observation_timeout_s', 8.),
                             ('approach_timeout_s', 25.), ('box_max_age_s', .8),
                             ('target_lost_timeout_s', 1.), ('image_width', 640),
                             ('image_height', 480), ('approach_speed_mps', .08),
                             ('bump_forward_m', .08), ('bump_move_timeout_s', 10.),
                             ('bump_position_tolerance_m', .02),
                             ('align_yaw_gain', 12.), ('align_vertical_gain', .12),
                             ('max_align_yaw_rate_deg_s', 6.), ('max_vertical_speed_mps', .04)):
            if not math.isfinite(float(p.get(key, default))) or float(p.get(key, default)) <= 0:
                raise ValueError(f'{key}必须为有限正数')
        for key, default in (('min_confidence', .35), ('center_tolerance_fraction', .06),
                             ('bump_box_width_fraction', .45)):
            if not math.isfinite(float(p.get(key, default))) or not 0 < float(p.get(key, default)) < 1:
                raise ValueError(f'{key}必须在(0,1)内')
        if float(p.get('bump_position_tolerance_m', .02)) >= float(p.get('bump_forward_m', .08)):
            raise ValueError('顶碰实测容差须小于前向行程，避免未移动就判完成')
        if int(p.get('disk_class_id', 3)) < 0:
            raise ValueError('disk_class_id必须非负')

    def _fallback_to_disk_center(self):
        """目标丢失后按已知盘心移动；不声称视觉确认或180度转动成功。"""
        nominal = self.params.get('disk_pose_odom', [])
        if len(nominal) != 4:
            self.log.error('简单转盘兜底不可用：必须填写disk_pose_odom=[x,y,z,yaw度]')
            return False
        target = list(map(float, nominal))
        if not all(math.isfinite(v) for v in target) or target[2] < 0:
            self.log.error('简单转盘兜底盘心坐标无效')
            return False
        self.node._publish_body_velocity()
        self.log.info(f'简单转盘目标丢失：改用盘心坐标兜底移动{target}；'
                      '不等待视觉，不验证接触或实际转角')
        for command, label in ((BasicMotion.Goal.WTRAVEL, '盘心兜底移动'),
                               (BasicMotion.Goal.SET, '盘心兜底归向')):
            self._check_running()
            ok, message = self.node._send_action_goal(command, target, 'xyzrz',
                timeout=min(float(self.params.get('motion_timeout_s', 90.)),
                            self._deadline-time.monotonic()),
                task_context=f'turntable_simple:{label}')
            if not ok:
                self.log.error(f'简单转盘：{label}失败：{message}')
                return False
        self.log.info('简单转盘兜底移动已执行，未确认180度旋转')
        return True

    def execute(self):
        p = self.params
        try:
            self._validate()
            self._deadline = time.monotonic() + float(p.get('timeout', 45.))
            self._preposition()
            first_deadline = time.monotonic() + float(p.get('observation_timeout_s', 8.))
            approach_deadline = None
            last_seen = None
            self.log.info('简单转盘：YOLO框居中→向前接近→前向MOVE顶碰一次；不使用棍子/孔位，不追加yaw')
            while True:
                self._check_running()
                now = time.monotonic()
                box = self._latest_box()
                if box is None:
                    self.node._publish_body_velocity()
                    if last_seen is None and now >= first_deadline:
                        raise _TargetLost('未识别到新鲜转盘YOLO框')
                    if last_seen is not None and now-last_seen >= float(p.get('target_lost_timeout_s', 1.)):
                        raise _TargetLost('接近时转盘框丢失')
                else:
                    last_seen = now
                    if approach_deadline is None:
                        approach_deadline = now + float(p.get('approach_timeout_s', 25.))
                    _, cx, cy, box_width = box
                    sign = -1. if p.get('front_camera_upside_down', True) else 1.
                    ex, ey = sign*(cx-.5), sign*(cy-.5)
                    tolerance = float(p.get('center_tolerance_fraction', .06))
                    centered = abs(ex) <= tolerance and abs(ey) <= tolerance
                    if centered and box_width >= float(p.get('bump_box_width_fraction', .45)):
                        break
                    yaw_limit = float(p.get('max_align_yaw_rate_deg_s', 6.))
                    z_limit = float(p.get('max_vertical_speed_mps', .04))
                    yaw = max(-yaw_limit, min(yaw_limit, ex*float(p.get('align_yaw_gain', 12.))))
                    vertical = max(-z_limit, min(z_limit, ey*float(p.get('align_vertical_gain', .12))))
                    self.node._publish_body_velocity(
                        forward_mps=float(p.get('approach_speed_mps', .08)) if centered else 0.,
                        vertical_mps=vertical, yaw_rate_deg_s=yaw)
                if approach_deadline is not None and now >= approach_deadline:
                    raise RuntimeError('YOLO框接近超时')
                time.sleep(.05)
            self._bump_forward()
            self.log.info('简单转盘动作完成：前向MOVE实测到位，不验证触碰或盘体转角')
            return True
        except _TargetLost as error:
            self.log.info(f'简单转盘：{error}；尝试已知盘心兜底')
            try:
                return self._fallback_to_disk_center()
            except (ValueError, RuntimeError, TypeError) as fallback_error:
                self.log.error(f'简单转盘兜底停止：{fallback_error}')
                return False
        except (ValueError, RuntimeError, TypeError) as error:
            self.log.error(f'简单转盘停止：{error}')
            return False
        finally:
            self.node._publish_body_velocity()
