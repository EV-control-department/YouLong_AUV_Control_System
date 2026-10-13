"""海参抓取真机联调任务；通过 config/tasks/grab_sea_cucumber.yaml 配置。

必填：模型类别、实际左目尺寸、抓取区位姿（含扫描深度）、下压速度/时长、收集区位姿、
支持舵机1圆盘爪与舵机2前下方夹爪，通过gripper.servo_id选择。
任务已注册到 task_runner，但不加入默认 robocup_26 任务链。
"""

from __future__ import annotations

from importlib import import_module
import math
import statistics
import time
from types import SimpleNamespace

from uv_msgs.action import BasicMotion
from uv_msgs.srv import CorrectOdomXY
from uv_camera.down_calibration import load_real_down_json, real_down_calibration_path


RB26GrabBallTask = import_module('uv_task.26rb_grab_ball').RB26GrabBallTask


class GrabSeaCucumberTask(RB26GrabBallTask):
    """复用抓球视觉伺服和下压动作，以新鲜下视帧的数量变化复检。"""

    def __init__(self, node, params: dict):
        # 明确要求现场模型和投放机构参数；不能用抓球模型的默认类别误抓。
        required = (
            'sea_cucumber_class_id', 'image_width', 'image_height',
            'descent_speed_mps', 'descent_duration_seconds',
            'drop_pose', 'search_pose')
        if int(params.get('gripper_servo_id', 1)) == 1:
            required += ('gripper_offset_x_m', 'gripper_offset_y_m')
        missing = [key for key in required if key not in params]
        if missing:
            raise ValueError(
                f'抓海参缺少必填参数：{", ".join(missing)}；'
                '请在任务 YAML 或 params 中填写现场标定值')
        speed = float(params['descent_speed_mps'])
        duration = float(params['descent_duration_seconds'])
        # 不恢复旧的0.3m/s和0.5m行程限幅；现场决定速度、持续时间。
        # 仍拒绝NaN、负速度或零时长，避免反向运动和无法终止的指令。
        if not math.isfinite(speed) or not math.isfinite(duration) or speed <= 0 or duration <= 0:
            raise ValueError('下压速度和时长必须是有限正数')
        super().__init__(node, params)
        self._class_id = int(params['sea_cucumber_class_id'])
        if self._class_id < 0:
            raise ValueError('sea_cucumber_class_id 必须非负')
        self._color = '海参'
        self._search_pose = tuple(float(v) for v in params['search_pose'])
        if (len(self._search_pose) != 4
                or not all(map(math.isfinite, self._search_pose))):
            raise ValueError('search_pose 必须是有限数值 [x, y, z, yaw_deg]，使用 START 后的 odom 坐标')
        if self._search_pose[2] < 0:
            raise ValueError('search_pose 中的 z 必须是非负扫描目标深度，单位米、向下为正')
        self._search_travel_timeout = float(params.get('search_travel_timeout_seconds', 90.0))
        if not math.isfinite(self._search_travel_timeout) or self._search_travel_timeout <= 0:
            raise ValueError('search_travel_timeout_seconds 必须是有限的正数')
        self._drop_pose = tuple(float(v) for v in params['drop_pose'])
        if len(self._drop_pose) != 4 or not all(map(math.isfinite, self._drop_pose)):
            raise ValueError('drop_pose 必须是有限数值 [x, y, z, yaw_deg]')
        self._gripper_servo_id = int(params.get('gripper_servo_id', 1))
        if self._gripper_servo_id not in (1, 2):
            raise ValueError('gripper.servo_id只能选择1或2')
        pickup_deg = float(params.get('pickup_servo_angle_deg',
                                      math.degrees(float(params.get('pickup_servo_angle_rad', math.radians(80.0))))))
        release_deg = float(params.get('release_servo_angle_deg',
                                       math.degrees(float(params.get('release_servo_angle_rad', math.pi)))))
        if self._gripper_servo_id == 2:
            pickup_deg = float(params.get('servo2_close_angle_deg', 150.0))
            release_deg = float(params.get('servo2_open_angle_deg', 270.0))
            if (not all(map(math.isfinite, (pickup_deg, release_deg)))
                    or not 0 <= pickup_deg < release_deg <= 270):
                raise ValueError('舵机2闭合/张开角度须满足0≤close<open≤270°')
        elif (not all(map(math.isfinite, (pickup_deg, release_deg)))
              or not all(0 <= angle <= 180 for angle in (pickup_deg, release_deg))):
            raise ValueError('舵机1抓放角度须为0~180°的有限数值')
        # 配置使用角度；任务内部沿用弧度接口，set_servo出口再转为真机角度协议。
        self._release_angle = math.radians(release_deg)
        self._pickup_angle = math.radians(pickup_deg)
        # 度配置优先；旧弧度配置仅在未提供度时使用，不再拒绝角度冲突。
        self._close_wait = float(params.get('gripper_close_wait_seconds', 0.5))
        if not math.isfinite(self._close_wait) or self._close_wait < 0:
            raise ValueError('夹爪闭合等待时间须为非负秒数')
        self._servo2_contact_z = None
        self._contact_body_xyz = tuple(float(v) for v in params.get(
            'gripper_contact_body_xyz', (-.430, 0., .290)))
        if (len(self._contact_body_xyz) != 3
                or not all(map(math.isfinite, self._contact_body_xyz))):
            raise ValueError('gripper_contact_body_xyz 必须为有限机体系[x,y,z]米')
        if self._gripper_servo_id == 2:
            front = tuple(float(v) for v in params.get('servo2_front_camera_body_xyz', (.230, 0., .076)))
            down = tuple(float(v) for v in params.get('servo2_down_camera_body_xyz', (-.130, .030, .0645)))
            relative = tuple(float(v) for v in params.get('servo2_gripper_from_front_xyz', (0., 0., .10)))
            if any(len(v) != 3 or not all(map(math.isfinite, v)) for v in (front, down, relative)):
                raise ValueError('舵机2安装坐标必须为有限[x,y,z]，机体系米，z向下')
            gripper = tuple(front[i] + relative[i] for i in range(3))
            # 对准后目标位于下视左光心正下；艇体需移动camera_xy-gripper_xy。
            # 这几个位置已经是物理安装后的body坐标，不能再旋转180°。
            self._gripper_offset_x = down[0] - gripper[0]
            self._gripper_offset_y = down[1] - gripper[1]
            self._servo2_contact_z = gripper[2]
            self._contact_body_xyz = gripper
            self._logger.info(f'抓海参：选择舵机2，张开={release_deg:g}°、闭合={pickup_deg:g}°；'
                              f'前下方抓取点body={gripper}m，水平补偿='
                              f'({self._gripper_offset_x:+.3f},{self._gripper_offset_y:+.3f})m')
        else:
            self._logger.info(f'抓海参：选择舵机1圆盘爪，抓{pickup_deg:g}°、放{release_deg:g}°，使用原水平补偿')
        self._total_timeout = float(params.get('total_timeout_seconds', 600.0))
        self._target_count = int(params.get('expected_count', 5))
        self._count_frames = int(params.get('count_frames', 3))
        self._count_timeout = float(params.get('count_timeout_seconds', 12.0))
        self._min_confidence = float(params.get('min_confidence', 0.35))
        self._drop_timeout = float(params.get('drop_timeout_seconds', 90.0))
        self._release_wait = float(params.get('release_wait_seconds', 2.0))
        self._ascent_speed = float(params.get('ascent_speed_mps', 0.01))
        self._ascent_period = float(params.get('ascent_publish_period', 0.05))
        self._ascent_tolerance = float(params.get('ascent_tolerance_m', 0.01))
        self._ascent_gain = float(params.get('ascent_slowdown_gain', .1))
        self._ascent_min_speed = float(params.get('ascent_min_speed_mps', .001))
        if (not all(math.isfinite(v) and v > 0 for v in (
                self._ascent_speed, self._ascent_gain, self._ascent_min_speed))
                or self._ascent_min_speed > self._ascent_speed):
            raise ValueError('上浮速度/减速增益必须为有限正数，最低速度不得超过上限')
        self._collection_release_offset = tuple(map(float, params.get(
            'collection_release_offset_body_xy', (0., 0.))))
        if (len(self._collection_release_offset) != 2
                or not all(map(math.isfinite, self._collection_release_offset))):
            raise ValueError('collection_release_offset_body_xy必须为有限机体系[x,y]米')
        self._max_failed_attempts = int(params.get('max_failed_attempts', 3))
        self._pixel_tolerance = float(params.get('pixel_tolerance_fraction', 0.035))
        self._target_match_px = float(params.get('target_match_radius_px', 70.0))
        self._target_lost_seconds = float(params.get('target_lost_wait_seconds', 2.0))
        self._servo_xy_tolerance = float(params.get('visual_position_tolerance_m', 0.01))
        self._servo_xy_noise_tolerance = float(params.get('visual_small_step_tolerance_m', 0.015))
        self._servo_settle_seconds = float(params.get('visual_motion_settle_seconds', 0.3))
        if (not all(math.isfinite(v) and v > 0 for v in (
                self._target_match_px, self._target_lost_seconds,
                self._servo_xy_tolerance, self._servo_xy_noise_tolerance,
                self._servo_settle_seconds))):
            raise ValueError('视觉匹配、丢失等待、微调容差和稳定时间必须为有限正数')
        width = float(params.get('image_width', 640.0))
        height = float(params.get('image_height', 480.0))
        if (width <= 0 or height <= 0 or self._target_count <= 0
                or self._count_frames < 2 or self._count_timeout <= 0
                or not math.isfinite(self._total_timeout)
                or self._total_timeout <= 0 or self._drop_timeout <= 0
                or self._max_failed_attempts < 0 or not 0 < self._min_confidence <= 1):
            raise ValueError('抓海参视觉、计数或超时参数无效')
        # 真机左目像素伺服必须使用与当前采集模式一致的实测 K，不再沿用
        # 抓球任务由名义 HFOV 推算的焦距。
        cal_width, cal_height, left_k, _, right_k, _, _, _ = \
            load_real_down_json(real_down_calibration_path())
        if (width, height) != (cal_width, cal_height):
            raise ValueError(
                f'抓海参图像应为每目 {cal_width}x{cal_height}，收到 {width}x{height}')
        self._FX, self._FY = float(left_k[0, 0]), float(left_k[1, 1])
        self._CX, self._CY = float(left_k[0, 2]), float(left_k[1, 2])
        self._right_intrinsics = tuple(float(right_k[i, j]) for i, j in ((0, 0), (1, 1), (0, 2), (1, 2)))
        # camera发布的掩膜恢复到原始标定像素，不是监控窗口旋转后的像素。
        # 下视整体物理绕机体yaw倒装180°，原像素误差对应的body x/y均须反向。
        self._camera_mount_yaw = float(params.get('down_camera_mount_yaw_deg', 180.0))
        if not math.isfinite(self._camera_mount_yaw):
            raise ValueError('down_camera_mount_yaw_deg 必须为有限数值')
        self._logger.info(
            f'抓海参：下视安装yaw补偿={self._camera_mount_yaw:g}°；'
            '输入为camera还原后的标定像素，夹爪偏置保持机体系定义')
        self.confirmed_removed = 0
        self.delivery_commands = 0
        self._scan_received_after = None
        self._scan_capture_after_ns = None
        self._near_floor_enabled = bool(params.get('near_floor_open_loop', False))
        self._floor_z = float(params.get('floor_z_m', -1.0))
        self._bottom_offset = float(params.get('bottom_reference_offset_z_m', 0.0))
        self._force_clearance = float(params.get('open_loop_clearance_m', 0.20))
        self._press_thrust = float(params.get('press_thrust', 0.0))
        self._lift_thrust = float(params.get('lift_thrust', 0.0))
        self._press_seconds = float(params.get('press_thrust_seconds', 1.0))
        self._pre_press_pose = None
        self._restore_press_xy = bool(params.get('restore_pre_press_odom_xy', False))
        self._collection_align = bool(params.get('collection_visual_align', True))
        self._collection_class = int(params.get('collection_class_id', 4))
        self._collection_depth = float(params.get('collection_projection_depth_m', self._projection_depth))
        self._collection_pixel_tolerance = float(params.get('collection_pixel_tolerance_fraction', .08))
        self._collection_hold_seconds = float(params.get('collection_hold_seconds', 0.0))
        self._collection_stereo_mode = False
        if (not math.isfinite(self._collection_pixel_tolerance)
                or not 0 < self._collection_pixel_tolerance <= .3
                or not math.isfinite(self._collection_hold_seconds)
                or self._collection_hold_seconds < 0):
            raise ValueError('收集框居中容差或保持时间无效')
        self._correction_max = float(params.get('odom_correction_max_m', 1.0))
        if self._near_floor_enabled and (not all(map(math.isfinite, (
                self._floor_z, self._bottom_offset, self._force_clearance,
                self._press_thrust, self._lift_thrust, self._press_seconds))) or self._floor_z <= 0
                or self._force_clearance <= 0 or not 0 < self._press_thrust <= 1
                or not -1 <= self._lift_thrust < 0
                or self._press_seconds <= 0):
            raise ValueError('近底开环请先标定floor_z_m、press_thrust(0,1]、lift_thrust[-1,0)，不是速度')
        if (self._collection_class < 0 or not math.isfinite(self._collection_depth)
                or self._collection_depth <= 0 or not math.isfinite(self._correction_max)
                or self._correction_max <= 0):
            raise ValueError('收集框类别、伺服参考距离或校正最大偏移无效')
        obsolete = ('collection_release_margin_m', 'collection_correct_odom_xy',
                    'collection_center_odom_xy', 'collection_camera_body_xy',
                    'max_press_distance_m', 'ascent_step_m', 'ascent_step_timeout_seconds',
                    'ascent_pause_seconds')
        for key in obsolete:
            if key in params:
                self._logger.warning(f'抓海参：旧参数{key}不再使用，请从配置删除')

    def _horizontal_step(self, detection):
        """像素 → 名义机体水平修正 → 安装yaw补偿 → odom修正。"""
        pose, dx, dy, _, _, du, dv = super()._horizontal_step(detection)
        body_dx, body_dy = self._body_to_world(dx, dy, self._camera_mount_yaw)
        world_dx, world_dy = self._body_to_world(body_dx, body_dy, pose[5])
        return pose, body_dx, body_dy, world_dx, world_dy, du, dv

    def _floor_clearance(self, pose):
        """最低参考点离底高度；考虑爪子安装位置以及roll/pitch。"""
        offset = self._bottom_offset
        contact = getattr(self, '_contact_body_xyz', None)
        if contact is not None:
            x, y, z = contact
            roll, pitch = math.radians(pose[3]), math.radians(pose[4])
            contact_down = (-math.sin(pitch)*x + math.cos(pitch)*math.sin(roll)*y
                            + math.cos(pitch)*math.cos(roll)*z)
            offset = max(offset, contact_down)
        return self._floor_z - pose[2] - offset

    def _servo_horizontally(self):
        """水平视觉伺服期间始终闭环保持同一z/yaw；近底下压仍独立开环。"""
        pose = self._measured_pose()
        if pose is None:
            self._logger.error('抓海参：水平对准缺少新鲜位姿')
            return None
        self._horizontal_hold_z_yaw = (pose[2], pose[5])
        # 每轮抓取/收集框对准重新锁定；轮内丢失不能跳选另一个目标。
        self._locked_pixel = None
        self._locked_pose = None
        self._association_stamp_ns = -1
        self._association_active = True
        self._logger.info(
            f'抓海参：{self._color}水平对准固定z={pose[2]:.3f}m、'
            f'yaw={pose[5]:.1f}°，SET axes=xyzrz；'
            '下视像素竖向误差只修正水平位置，不用于升沉；z正方向向下')
        # 即使首帧已居中，也必须退出之前的速度/推力模式，进入位置保持。
        started = time.monotonic()
        timeout = min(self._command_timeout, self._servo_timeout)
        if timeout <= 0 or self._node.stopped:
            self._association_active = False
            return None
        success, message = self._node._send_action_goal(
            BasicMotion.Goal.SET, [pose[0], pose[1], pose[2], pose[5]], 'xyzrz',
            timeout=timeout, quiet=True,
            task_context=self._node._format_motion_context(f'{self._color}对准启用深度航向闭环'))
        if not success:
            self._logger.warning(f'抓海参：启用位置保持失败：{message}')
            self._association_active = False
            return None
        self._node._cmd_z, self._node._cmd_yaw = self._horizontal_hold_z_yaw
        try:
            deadline = started + self._servo_timeout
            if not self._wait_visual_motion(
                    [pose[0], pose[1], pose[2], pose[5]], deadline):
                return None
            return self._locked_visual_loop(deadline)
        finally:
            self._association_active = False

    def _wait_visual_motion(self, target, deadline, xy_tolerance=None):
        """独立检查实测反馈，不把BasicMotion的10cm到位当作微调完成。"""
        tolerance = self._servo_xy_tolerance if xy_tolerance is None else xy_tolerance
        end = min(deadline, time.monotonic() + self._command_timeout)
        stable_since, last_log = None, float('-inf')
        while not self._node.stopped and time.monotonic() < end:
            pose = self._measured_pose()
            now = time.monotonic()
            if pose is not None:
                xy_error = math.hypot(pose[0]-target[0], pose[1]-target[1])
                yaw_error = abs((pose[5]-target[3]+180.) % 360.-180.)
                reached = (xy_error <= tolerance and abs(pose[2]-target[2]) <= .05
                           and yaw_error <= 5.)
                if reached:
                    stable_since = now if stable_since is None else stable_since
                    if now-stable_since >= self._servo_settle_seconds:
                        # 动作响应后才设采集截止，下一步只用稳定后的新图像。
                        self._scan_capture_after_ns = self._node.get_clock().now().nanoseconds
                        self._scan_received_after = now
                        return True
                else:
                    stable_since = None
                if now-last_log >= self._log_period:
                    self._logger.info(f'抓海参：等待视觉微调实测到位，XY误差={xy_error:.3f}m/'
                                      f'{tolerance:.3f}m，z误差={pose[2]-target[2]:+.3f}m，'
                                      f'yaw误差={yaw_error:.1f}°')
                    last_log = now
            else:
                stable_since = None
            time.sleep(min(.05, max(0., end-time.monotonic())))
        self._logger.warning('抓海参：视觉微调未实测到位，超时；不连续追加修正')
        return False

    def _locked_visual_loop(self, deadline):
        """一帧一决策：锁定同一目标→微移→实际到位→新采集帧。"""
        last_stamp, hold_since, lost_since = -1, None, None
        last_log = float('-inf')
        while not self._node.stopped and time.monotonic() < deadline:
            now = time.monotonic()
            detection = self._best_left_detection()
            if detection is None:
                reason = getattr(self, '_detection_reject_reason', '新帧无匹配目标')
                # 等待/重复/旧采集帧不是一次新的反证，不能清除已居中状态。
                missing = reason in ('新帧无匹配目标', '匹配距离超过门限', '等待同一双目对')
                if missing:
                    hold_since = None
                    lost_since = now if lost_since is None else lost_since
                if now-last_log >= self._log_period:
                    self._logger.info(f'抓海参：等待锁定目标；原因={reason}，不切换其他目标')
                    last_log = now
                if (self._locked_pixel is not None
                        and missing and lost_since is not None
                        and now-lost_since >= self._target_lost_seconds):
                    self._logger.warning('抓海参：锁定目标丢失等待超时，结束本轮对准')
                    return None
            elif detection.stamp_ns > last_stamp:
                last_stamp, lost_since = detection.stamp_ns, None
                pose, dx, dy, wx, wy, du, dv = self._horizontal_step(detection)
                if now-last_log >= self._log_period:
                    self._logger.info(
                        f'抓海参：锁定目标像素=({detection.pixel_x:.1f},{detection.pixel_y:.1f})，'
                        f'误差=({du:+.4f},{dv:+.4f})，机体步长=({dx:+.3f},{dy:+.3f})m，'
                        f'世界步长=({wx:+.3f},{wy:+.3f})m')
                    last_log = now
                centered = abs(du) <= self._pixel_tolerance and abs(dv) <= self._pixel_tolerance
                if centered:
                    hold_since = now if hold_since is None else hold_since
                    if now-hold_since >= self._hold_seconds:
                        measured = self._measured_pose()
                        if measured is not None:
                            self._logger.info(f'抓海参：{self._color}视觉居中通过，误差='
                                              f'({du:+.4f},{dv:+.4f})，容差={self._pixel_tolerance:g}')
                            return [measured[0], measured[1], measured[2], measured[5]]
                else:
                    hold_since = None
                    target = [pose[0]+wx, pose[1]+wy, *self._horizontal_hold_z_yaw]
                    success, message = self._node._send_action_goal(
                        BasicMotion.Goal.SET, target, 'xyzrz',
                        timeout=min(self._command_timeout, max(.01, deadline-time.monotonic())),
                        quiet=True, task_context=self._node._format_motion_context(
                            f'{self._color}锁定目标视觉微调'))
                    if not success:
                        self._logger.warning(f'抓海参：视觉微调动作失败：{message}')
                        return None
                    self._node._cmd_x, self._node._cmd_y, self._node._cmd_z, self._node._cmd_yaw = target
                    # 小步不再一律收紧到半步：允许厘米级反馈噪声。
                    # 仍限制为步长的80%，静止在出发点不能通过；最终抓取由新帧视觉确认。
                    step = math.hypot(wx, wy)
                    tolerance = min(self._servo_xy_tolerance, step*.8,
                                    max(getattr(self, '_servo_xy_noise_tolerance', .015), step*.5))
                    if not self._wait_visual_motion(target, deadline, max(1e-5, tolerance)):
                        return None
            time.sleep(min(self._servo_period, max(0., deadline-time.monotonic())))
        self._logger.warning('抓海参：锁定目标水平视觉伺服超时')
        return None

    def _apply_gripper_offset(self):
        """按实测XY计算绝对偏置目标，并继续保持本次视觉对准的z/yaw。"""
        pose = self._measured_pose()
        if pose is None:
            self._logger.error('抓海参：夹爪偏置缺少新鲜位姿')
            return False
        hold = getattr(self, '_horizontal_hold_z_yaw', (pose[2], pose[5]))
        dx, dy = self._body_to_world(self._gripper_offset_x, self._gripper_offset_y, pose[5])
        target = [pose[0] + dx, pose[1] + dy, hold[0], hold[1]]
        self._logger.info(
            f'抓海参：夹爪偏置目标XY=({target[0]:.3f},{target[1]:.3f})m，'
            f'保持z={hold[0]:.3f}m、yaw={hold[1]:.1f}°；SET axes=xyzrz')
        success, message = self._node._send_action_goal(
            BasicMotion.Goal.SET, target, 'xyzrz', timeout=self._command_timeout,
            task_context=self._node._format_motion_context(f'{self._color}夹爪偏置并保持深度航向'))
        if not success:
            self._logger.error(f'抓海参：夹爪偏置失败：{message}')
            return False
        self._node._cmd_x, self._node._cmd_y, self._node._cmd_z, self._node._cmd_yaw = target
        return True

    def _segmented_detections(self, message):
        return [d for d in getattr(message, 'detections', ())
                if int(getattr(d, 'class_id', -1)) == self._class_id
                and float(getattr(d, 'confidence', 0.0)) >= self._min_confidence
                and len(getattr(d, 'mask_x', ())) >= 3
                and len(getattr(d, 'mask_x', ())) == len(getattr(d, 'mask_y', ()))]

    def _best_left_detection(self):
        """选择最大可信掩膜，并用掩膜中心而非 bbox 中心压准目标。"""
        if getattr(self, '_collection_stereo_mode', False):
            return self._collection_stereo_detection()
        self._detection_reject_reason = '等待新帧'
        with self._node._perception_lock:
            entry = self._node._down_detections.get('down_left')
        if entry is None or time.monotonic() - entry[0] > self._detection_timeout:
            self._detection_reject_reason = '检测消息缺失或过期'
            return None
        # 前往抓取区或返回后的扫描，只采用扫描开始后收到且拍摄的图像。
        received_after = getattr(self, '_scan_received_after', None)
        if received_after is not None and entry[0] <= received_after:
            self._detection_reject_reason = '等待动作稳定后收到的新帧'
            return None
        capture_after_ns = getattr(self, '_scan_capture_after_ns', None)
        if capture_after_ns is not None:
            stamp = getattr(getattr(entry[1], 'header', None), 'stamp', None)
            stamp_ns = (int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
                        if stamp is not None else 0)
            if stamp_ns <= capture_after_ns:
                self._detection_reject_reason = '采集时间早于动作稳定时间'
                return None
        stamp_ns = (int(entry[1].header.stamp.sec)*1_000_000_000
                    + int(entry[1].header.stamp.nanosec))
        if (getattr(self, '_association_active', False)
                and stamp_ns <= getattr(self, '_association_stamp_ns', -1)):
            self._detection_reject_reason = '重复采集帧'
            return None
        if getattr(self, '_association_active', False):
            # 空帧也只算一次，不把同一张空图反复累计为连续丢失。
            self._association_stamp_ns = stamp_ns
        candidates = self._segmented_detections(entry[1])
        if not candidates:
            self._detection_reject_reason = '新帧无匹配目标'
            return None
        locked = getattr(self, '_locked_pixel', None)
        if getattr(self, '_association_active', False) and locked is not None:
            # 用实际艇体位移预测目标的新像素，避免较大微移后仍围绕旧像素匹配。
            previous_pose = getattr(self, '_locked_pose', None)
            current_pose = self._measured_pose()
            if previous_pose is not None and current_pose is not None:
                du, dv = (locked[0]-self._CX)/self._FX, (locked[1]-self._CY)/self._FY
                bx, by = self._body_to_world(-dv*self._projection_depth,
                                            du*self._projection_depth, self._camera_mount_yaw)
                wx, wy = self._body_to_world(bx, by, previous_pose[5])
                bx, by = self._body_to_world(previous_pose[0]+wx-current_pose[0],
                                            previous_pose[1]+wy-current_pose[1], -current_pose[5])
                bx, by = self._body_to_world(bx, by, -self._camera_mount_yaw)
                locked = (self._CX+by/self._projection_depth*self._FX,
                          self._CY-bx/self._projection_depth*self._FY)
            target = min(candidates, key=lambda d: math.hypot(
                statistics.median(d.mask_x)-locked[0], statistics.median(d.mask_y)-locked[1]))
            if math.hypot(statistics.median(target.mask_x)-locked[0],
                          statistics.median(target.mask_y)-locked[1]) > self._target_match_px:
                self._detection_reject_reason = '匹配距离超过门限'
                return None
        else:
            target = max(candidates, key=lambda d: (float(d.confidence), len(d.mask_x)))
        if getattr(self, '_association_active', False):
            self._locked_pixel = (statistics.median(target.mask_x), statistics.median(target.mask_y))
            self._locked_pose = self._measured_pose()
            self._association_stamp_ns = stamp_ns
        return SimpleNamespace(
            pixel_x=statistics.median(target.mask_x),
            pixel_y=statistics.median(target.mask_y),
            confidence=target.confidence,
            stamp_ns=stamp_ns,
        )

    def _collection_stereo_detection(self):
        """同一对双目框中心的平均归一化误差；不读取掩膜或推算爪子落点。"""
        self._detection_reject_reason = '新帧无匹配目标'
        with self._node._perception_lock:
            entries = [self._node._down_detections.get(side)
                       for side in ('down_left', 'down_right')]
        now = time.monotonic()
        stamps, pair_ids, centers = [], [], []
        for entry in entries:
            if entry is None or not 0 <= now-entry[0] <= self._detection_timeout:
                return self._collection_single_eye_detection()
            if entry[0] <= (getattr(self, '_scan_received_after', None) or float('-inf')):
                self._detection_reject_reason = '等待动作稳定后收到的新帧'
                return self._collection_single_eye_detection()
            msg = entry[1]
            stamp = msg.header.stamp
            stamp_ns = int(stamp.sec)*1_000_000_000 + int(stamp.nanosec)
            if stamp_ns <= (getattr(self, '_scan_capture_after_ns', None) or 0):
                self._detection_reject_reason = '采集时间早于动作稳定时间'
                return self._collection_single_eye_detection()
            age = (self._node.get_clock().now().nanoseconds-stamp_ns)/1e9
            if not 0 <= age <= self._detection_timeout:
                return self._collection_single_eye_detection()
            candidates = []
            for d in msg.detections:
                if int(d.class_id) != self._class_id:
                    continue
                values = tuple(float(v) for v in
                               (d.bbox_x1, d.bbox_y1, d.bbox_x2, d.bbox_y2, d.confidence))
                if not all(map(math.isfinite, values)):
                    continue
                x1, y1, x2, y2, confidence = values
                if (confidence >= self._min_confidence and
                        0 <= x1 < x2 <= self._IMAGE_WIDTH and
                        0 <= y1 < y2 <= self._IMAGE_HEIGHT):
                    candidates.append(((x1+x2)/2., (y1+y2)/2.))
            # 现场预期一个收集框，歧义时不混配两个不同框。
            if len(candidates) != 1:
                return self._collection_single_eye_detection()
            centers.append(candidates[0])
            stamps.append(stamp_ns)
            pair_ids.append(int(getattr(msg, 'stereo_pair_id', 0)))
        if any(pair_ids):
            if not pair_ids[0] or pair_ids[0] != pair_ids[1]:
                self._detection_reject_reason = '等待同一双目对'
                return self._collection_single_eye_detection()
        elif abs(stamps[0]-stamps[1]) > 50_000_000:
            self._detection_reject_reason = '等待同一双目对'
            return self._collection_single_eye_detection()
        # 两目都必须是动作稳定后的新采集帧，不能复用右目配后续左目。
        if min(stamps) <= getattr(self, '_collection_last_stamp_ns', -1):
            self._detection_reject_reason = '重复采集帧'
            return self._collection_single_eye_detection()
        self._collection_last_stamp_ns = max(stamps)
        fx, fy, cx, cy = self._right_intrinsics
        du = ((centers[0][0]-self._CX)/self._FX + (centers[1][0]-cx)/fx)/2.
        dv = ((centers[0][1]-self._CY)/self._FY + (centers[1][1]-cy)/fy)/2.
        # 转成等效左目像素以复用已标定的水平步进、安装yaw和实测到位等待。
        return SimpleNamespace(pixel_x=self._CX+du*self._FX, pixel_y=self._CY+dv*self._FY,
                               stamp_ns=max(stamps))

    def _collection_single_eye_detection(self):
        """双目不完整/不匹配时用一目新鲜框；多框选靠近光心的可信框，不混配。"""
        with self._node._perception_lock:
            entries = [self._node._down_detections.get(s) for s in ('down_left', 'down_right')]
        now, ros_ns = time.monotonic(), self._node.get_clock().now().nanoseconds
        for side, entry in enumerate(entries):
            if entry is None or not 0 <= now-entry[0] <= self._detection_timeout:
                continue
            if entry[0] <= (getattr(self, '_scan_received_after', None) or float('-inf')):
                continue
            msg = entry[1]
            stamp = int(msg.header.stamp.sec)*1_000_000_000+int(msg.header.stamp.nanosec)
            if (stamp <= max(getattr(self, '_scan_capture_after_ns', 0) or 0,
                             getattr(self, '_collection_last_stamp_ns', -1))
                    or not 0 <= (ros_ns-stamp)/1e9 <= self._detection_timeout):
                continue
            fx, fy, cx, cy = ((self._FX, self._FY, self._CX, self._CY) if side == 0
                              else self._right_intrinsics)
            candidates = []
            for d in msg.detections:
                values = (d.bbox_x1, d.bbox_y1, d.bbox_x2, d.bbox_y2, d.confidence)
                if int(d.class_id) != self._class_id or not all(map(math.isfinite, values)):
                    continue
                x1, y1, x2, y2, confidence = values
                if (confidence >= self._min_confidence and 0 <= x1 < x2 <= self._IMAGE_WIDTH
                        and 0 <= y1 < y2 <= self._IMAGE_HEIGHT):
                    du, dv = ((x1+x2)/2-cx)/fx, ((y1+y2)/2-cy)/fy
                    candidates.append((math.hypot(du, dv), -confidence, du, dv))
            if candidates:
                _, _, du, dv = min(candidates)
                self._collection_last_stamp_ns = stamp
                self._logger.warning(f'收集框降级：使用{"左" if side == 0 else "右"}目新鲜框，'
                                     f'候选={len(candidates)}，不要求双目配对')
                return SimpleNamespace(pixel_x=self._CX+du*self._FX, pixel_y=self._CY+dv*self._FY,
                                       stamp_ns=stamp)
        if getattr(self, '_detection_reject_reason', '') != '重复采集帧':
            self._detection_reject_reason = '等待新鲜单目收集框'
        return None

    def _count_visible(self, since: float, capture_after_ns: int,
                       deadline: float, conservative: str):
        """只数下视左目中不同的、动作完成后的新鲜分割帧。"""
        counts = []
        last_stamp = since
        end = min(deadline, time.monotonic() + self._count_timeout)
        while not self._node.stopped and time.monotonic() < end:
            with self._node._perception_lock:
                entry = self._node._down_detections.get('down_left')
            if entry is not None and entry[0] > last_stamp:
                last_stamp = entry[0]
                stamp = getattr(getattr(entry[1], 'header', None), 'stamp', None)
                stamp_ns = (int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
                            if stamp is not None else 0)
                if (stamp_ns > capture_after_ns
                        and time.monotonic() - last_stamp <= self._detection_timeout):
                    counts.append(len(self._segmented_detections(entry[1])))
                    if len(counts) >= self._count_frames:
                        # 抓前取最小、抓后取最大：偶发漏检不会假报抓取成功。
                        result = min(counts) if conservative == 'before' else max(counts)
                        self._logger.info(
                            f'抓海参：{conservative}计数帧={counts}，保守结果={result}')
                        return result
            time.sleep(min(0.1, max(0.0, end - time.monotonic())))
        self._logger.warning(f'抓海参：{conservative}计数缺少新鲜帧，已收{len(counts)}帧')
        # 帧率波动时不要求必须收满默认3帧；至少2帧仍保留多帧复核。
        # 不接受旧帧、不用单帧或历史数量猜测当前是否抓到。
        if not self._node.stopped and time.monotonic() < deadline and len(counts) >= 2:
            result = min(counts) if conservative == 'before' else max(counts)
            self._logger.warning(
                f'抓海参：降级使用{len(counts)}个新鲜计数帧={counts}，保守结果={result}')
            return result
        return None

    def _retry_perception(self, reason, failures, deadline):
        """视觉失败只结束本轮，不立刻结束整项任务；仍受重试与总超时约束。"""
        if self._node.stopped:
            self._logger.warning(f'抓海参：{reason}；节点已停止，无法重试')
            return False
        if time.monotonic() >= deadline:
            self._logger.error(f'抓海参：{reason}；整项任务总预算已耗尽，无法重试'
                               '（单轮视觉超时与总预算独立）')
            return False
        if failures > self._max_failed_attempts:
            self._logger.error(f'抓海参：{reason}；连续视觉失败已超过重试上限{self._max_failed_attempts}')
            return False
        self._logger.warning(
            f'抓海参：{reason}；不盲目下压，等待新观测后重试 '
            f'{failures}/{self._max_failed_attempts}')
        end = min(deadline, time.monotonic() + 1.0)
        while not self._node.stopped and time.monotonic() < end:
            time.sleep(min(0.05, max(0.0, end-time.monotonic())))
        return not self._node.stopped and time.monotonic() < deadline

    def _travel(self, pose, label, deadline, timeout):
        motion_deadline = min(deadline, time.monotonic() + timeout)
        self._logger.info(f'抓海参：{label}时间预算：局部上限={timeout:.1f}s，'
                          f'剩余总预算={max(0., deadline-time.monotonic()):.1f}s，'
                          f'本段有效预算={max(0., motion_deadline-time.monotonic()):.1f}s')
        measured = self._measured_pose()
        if measured is None:
            self._logger.error(f'抓海参：{label}缺少实测位姿，拒绝移动')
            return False
        target = list(pose)
        # 深目标必须真的下沉到位；浅目标沿用速度上浮，不下发向下纠偏。
        descending = float(target[2]) > measured[2]
        if float(target[2]) < measured[2] - self._ascent_tolerance:
            if not self._ascend_to_depth(float(target[2]), motion_deadline):
                return False
            measured = self._measured_pose()
            if measured is None:
                return False
            target[2] = measured[2]  # 上浮过冲也不让WTRAVEL再下沉回补。
        if float(target[2]) < measured[2]:
            # WTRAVEL 不再发出向上的 z 目标；保留上浮后实测深度。
            target[2] = measured[2]
        remaining = motion_deadline - time.monotonic()
        if remaining <= 0 or self._node.stopped:
            return False
        success, message = self._node._send_action_goal(
            BasicMotion.Goal.WTRAVEL, target, 'xyzrz',
            timeout=remaining,
            task_context=self._node._format_motion_context(label))
        if not success:
            self._logger.error(f'抓海参：{label}失败：{message}')
            return False
        # WTRAVEL 水平前进结束时保持行进航向，而非配置中的 rz。
        # 显式水平定位并归向，不能只修改 _cmd_yaw 就当作机器人已归位。
        remaining = motion_deadline - time.monotonic()
        measured = self._measured_pose()
        if remaining <= 0 or measured is None or self._node.stopped:
            return False
        if not descending:
            target[2] = measured[2]  # 不用位置环拉升或纠正速度上浮的过冲。
        self._logger.info(f'抓海参：{label}运输到位，开始水平定位及归向：'
                          f'x={target[0]:.3f}，y={target[1]:.3f}，z={target[2]:.3f}m，'
                          f'yaw={target[3]:.1f}°')
        success, message = self._node._send_action_goal(
            BasicMotion.Goal.SET, target, 'xyzrz', timeout=remaining,
            task_context=self._node._format_motion_context(label + '到位归向'))
        if not success:
            self._logger.error(f'抓海参：{label}到位归向失败：{message}')
            return False
        # 动作结果和实测位姿同时确认；旧指令位姿不能作为返回扫描区的证据。
        while not self._node.stopped and time.monotonic() < motion_deadline:
            measured = self._measured_pose()
            if measured is not None:
                yaw_error = abs((measured[5] - target[3] + 180.0) % 360.0 - 180.0)
                if (abs(measured[0] - target[0]) <= 0.1
                        and abs(measured[1] - target[1]) <= 0.1
                        and abs(measured[2] - target[2]) <= min(self._ascent_tolerance, .05)
                        and yaw_error <= 5.0):
                    break
            time.sleep(0.05)
        else:
            self._logger.error(f'抓海参：{label}实测位置/深度/航向未归位，禁止释放或重新扫描')
            return False
        self._node._cmd_x, self._node._cmd_y, self._node._cmd_z, self._node._cmd_yaw = target
        self._logger.info(f'抓海参：{label}位置、深度及航向已确认，实测z={measured[2]:.3f}m')
        return True

    def _measured_pose(self):
        """上浮只能依据实测位姿，不能使用任务节点的指令位姿兜底。"""
        with self._node._perception_lock:
            pose = self._node._robot_pose
            received = getattr(self._node, '_robot_pose_received', time.monotonic())
        if time.monotonic() - received > 1.0:
            return None
        if pose is None or len(pose) < 6 or not all(math.isfinite(float(v)) for v in pose):
            return None
        return tuple(float(v) for v in pose)

    def _ascend_to_depth(self, target_z, deadline):
        """绕过位置步进环，直接持续发布负body-z速度；不发送向下修正。"""
        pose = self._measured_pose()
        if pose is None:
            self._logger.error('抓海参：没有实测位姿，拒绝盲目上浮')
            return False
        self._logger.info(
            f'抓海参：直接速度上浮，当前z={pose[2]:.3f}m，目标z={target_z:.3f}m，'
            f'速度上限={self._ascent_speed:.3f}m/s；不使用BMOVE位置拉升')
        last_log = float('-inf')
        force_mode = (getattr(self, '_near_floor_enabled', False)
                      and self._floor_clearance(pose)
                      <= self._force_clearance + 0.03)
        try:
            while not self._node.stopped and time.monotonic() < deadline:
                pose = self._measured_pose()
                if pose is None:
                    self._logger.error('抓海参：速度上浮期间丢失实测位姿')
                    return False
                remaining = pose[2] - target_z
                if remaining <= self._ascent_tolerance:
                    self._node._cmd_z = pose[2]
                    self._logger.info(f'抓海参：速度上浮到位，实测z={pose[2]:.3f}m；不向下纠偏')
                    return True
                if force_mode:
                    if self._floor_clearance(pose) <= self._force_clearance + 0.03:
                        self._node._publish_body_thrust(self._lift_thrust)
                        if time.monotonic() - last_log >= 1.0:
                            last_log = time.monotonic()
                            self._logger.info(f'抓海参：近底纯推力离底，z={pose[2]:.3f}，'
                                              f'推力={self._lift_thrust:g}；速度不作保证')
                        time.sleep(min(self._ascent_period, max(0., deadline-time.monotonic())))
                        continue
                    self._node._publish_body_thrust(0.0)
                    force_mode = False
                    self._logger.info('抓海参：离地超过开环高度+3cm，恢复慢速上浮速度环')
                # 距目标越近，指令越缓；不突然从下压切到大幅上浮速度。
                speed = min(self._ascent_speed, max(
                    getattr(self, '_ascent_min_speed', .001),
                    remaining * getattr(self, '_ascent_gain', .1)))
                self._node._publish_body_velocity(vertical_mps=-speed)
                if time.monotonic() - last_log >= 1.0:
                    last_log = time.monotonic()
                    self._logger.info(f'抓海参：上浮z={pose[2]:.3f}m，剩余={remaining:.3f}m，vz={-speed:.4f}m/s')
                time.sleep(min(self._ascent_period, max(0., deadline-time.monotonic())))
            self._logger.error('抓海参：上浮超时或任务被停止')
            return False
        finally:
            # 零速度仍是MCU速度环，不等于原始推进器零推力。
            if force_mode:
                self._node._publish_body_thrust(0.0)
            else:
                self._node._publish_body_velocity()

    def _descend(self):
        """近底阶段一次性切入纯推力；DVL跳变不能把模式切回速度环。"""
        self._pre_press_pose = self._measured_pose()
        if self._pre_press_pose is None:
            return False
        self._logger.info(f'抓海参：记录夹爪补偿完成后的下压前XY={self._pre_press_pose[:2]}；'
                          '离底校正使用此锚点，不使用相机居中观察点')
        if getattr(self, '_near_floor_enabled', False):
            deadline = min(getattr(self, '_mission_deadline', float('inf')),
                           time.monotonic() + self._descent_duration)
            force_mode = False
            try:
                while not self._node.stopped and time.monotonic() < deadline:
                    pose = self._measured_pose()
                    if pose is None:
                        self._logger.error('抓海参：缺少新鲜深度，不能判断离地高度')
                        return False
                    clearance = self._floor_clearance(pose)
                    if clearance <= self._force_clearance:
                        force_mode = True
                        self._logger.info(
                            f'抓海参：离地{clearance:.3f}m，锁定纯开环下压；'
                            f'推力={self._press_thrust:g}，持续={self._press_seconds:g}s；不使用DVL XY闭环')
                        end = min(deadline, time.monotonic() + self._press_seconds)
                        while not self._node.stopped and time.monotonic() < end:
                            self._node._publish_body_thrust(self._press_thrust)
                            time.sleep(min(self._descent_period, max(0., end-time.monotonic())))
                        return not self._node.stopped
                    self._node._publish_body_velocity(vertical_mps=self._descent_speed)
                    time.sleep(min(self._descent_period, max(0., deadline-time.monotonic())))
                self._logger.error('抓海参：下压超时，尚未进入近底开环阶段')
                return False
            finally:
                if force_mode:
                    self._node._publish_body_thrust(0.0)
                else:
                    self._node._publish_body_velocity()
        self._logger.info(f'抓海参：连续下压，vz={self._descent_speed:g}m/s，'
                          f'持续={self._descent_duration:g}s；行程由现场配置决定')
        deadline = time.monotonic() + self._descent_duration
        try:
            while not self._node.stopped and time.monotonic() < deadline:
                self._node._publish_body_velocity(vertical_mps=self._descent_speed)
                time.sleep(min(self._descent_period, max(0., deadline-time.monotonic())))
            return not self._node.stopped
        finally:
            self._node._publish_body_velocity()

    def _return_to_recorded_pose(self, recorded_pose):
        """先直接低速上浮，再做不含 z 的水平回位。"""
        if len(recorded_pose) != 4 or not all(math.isfinite(float(v)) for v in recorded_pose):
            self._logger.error('抓海参：记录位姿无效，拒绝上浮')
            return False
        deadline = min(getattr(self, '_mission_deadline', float('inf')),
                       time.monotonic() + self._return_timeout)
        if not self._ascend_to_depth(float(recorded_pose[2]), deadline):
            return False

        before = getattr(self, '_pre_press_pose', None)
        after = self._measured_pose()
        if before is not None and after is not None:
            self._logger.info(f'抓海参：离底后XY变化=({after[0]-before[0]:+.3f},'
                              f'{after[1]-before[1]:+.3f})m；'
                              f'下压锚点校正开关={getattr(self, "_restore_press_xy", False)}')
            if getattr(self, '_restore_press_xy', False):
                # 只有确认压抓中无真实水平漂移时才启用这一近似。
                if not self._correct_xy(before[:2], deadline, '下压前XY锚点（假定无真实水平漂移）'):
                    return False

        # SET 只控制水平位置和航向，z 保持当前实测值，不再一次性拉升。
        pose = self._measured_pose()
        if pose is None or self._node.stopped or time.monotonic() >= deadline:
            return False
        target = [float(recorded_pose[0]), float(recorded_pose[1]),
                  pose[2], float(recorded_pose[3])]
        success, message = self._node._send_action_goal(
            BasicMotion.Goal.SET, target, 'xyrz',
            timeout=max(0.1, deadline - time.monotonic()),
            task_context=self._node._format_motion_context('抓海参后水平回位'))
        if not success:
            self._logger.error(f'抓海参：上浮后水平回位失败：{message}')
            return False
        self._node._cmd_x, self._node._cmd_y = target[:2]
        self._node._cmd_z, self._node._cmd_yaw = target[2:]
        return True

    def _deliver(self, search_pose, deadline, return_to_search):
        if not self._travel(self._drop_pose, '海参运往收集区', deadline, self._drop_timeout):
            return False
        aligned = False
        position_confirmed = True
        if getattr(self, '_collection_align', False):
            # 视觉阶段不能用尽总预算：留出部分时间回到配置投放点并释放。
            remaining = max(0., deadline-time.monotonic())
            release_budget = self._release_wait + self._pre_descent_settle_seconds
            reserve = min(self._drop_timeout, max(0., remaining-release_budget)*.5)
            align_deadline = deadline-reserve-release_budget
            aligned = (align_deadline > time.monotonic()
                       and self._align_collection(align_deadline))
            if not aligned:
                if self._node.stopped:
                    self._logger.warning('抓海参：对准失败且外部停止，不执行释放')
                    return False
                self._logger.warning(
                    f'抓海参：收集框对准失败，启用投放兜底；重新前往配置位姿'
                    f'{self._drop_pose}并恢复航向，到位后执行释放前偏置再释放，不再等待视觉')
                # 对准可能已使艇体偏离，不能在当前位置直接张爪。
                if (time.monotonic() >= deadline or not self._travel(
                        self._drop_pose, '收集框对准失败：配置投放点兜底', deadline, self._drop_timeout)):
                    position_confirmed = False
                    self._logger.warning('抓海参：兜底投放移动失败或总预算耗尽，仍尝试释放；'
                                         '当前释放区域未确认，不计为有效投放证明')
        # 视觉只做相机居中；所有投放路径统一追加一次偏置并等待稳定。
        if position_confirmed and not self._move_collection_release_offset(deadline):
            self._logger.warning('抓海参：释放前偏置/稳定未到位，仍尝试释放；位置未精确确认')
        if self._node.stopped:
            return False
        self._command_gripper(self._release_angle, '海参收集区释放')
        self.delivery_commands += 1  # 已发释放指令，不能因后续超时丢掉这条记录。
        end = min(deadline, time.monotonic() + self._release_wait)
        while not self._node.stopped and time.monotonic() < end:
            time.sleep(min(0.05, end - time.monotonic()))
        if self._node.stopped or time.monotonic() >= deadline:
            return False
        self._logger.info('抓海参：已发释放指令；实际投放仍须靠视觉/现场验证')
        self._logger.info(f'抓海参：释放等待完成，开始上浮至搜索深度z={search_pose[2]:.3f}m')
        if not self._ascend_to_depth(float(search_pose[2]), deadline):
            return False
        if not return_to_search:
            return True
        self._logger.info('抓海参：投放结束，返回配置扫描点；返回成功前不启动扫描')
        if not self._travel(search_pose, '返回海参搜索区', deadline,
                            self._search_travel_timeout):
            return False
        self._logger.info('抓海参：已返回扫描点并恢复扫描航向，允许开始下一轮识别')
        return True

    def _correct_xy(self, target_xy, deadline, reason):
        """只平移BasicMotion的任务坐标变换，不改MCU原点/深度/航向。"""
        pose = self._measured_pose()
        if pose is None:
            return False
        delta = (float(target_xy[0])-pose[0], float(target_xy[1])-pose[1])
        if math.hypot(*delta) > self._correction_max:
            self._logger.error(f'抓海参：{reason}校正幅度{delta}超过配置，检查坐标系/识别')
            return False
        client = self._node._odom_xy_client
        if not client.wait_for_service(timeout_sec=min(1.0, max(0., deadline-time.monotonic()))):
            self._logger.error('抓海参：BasicMotion XY校正接口不可用，请重编uv_msgs/uv_control')
            return False
        request = CorrectOdomXY.Request()
        request.expected_x, request.expected_y = pose[:2]
        request.corrected_x, request.corrected_y = map(float, target_xy)
        future = client.call_async(request)
        end = min(deadline, time.monotonic()+2.0)
        while not future.done() and not self._node.stopped and time.monotonic() < end:
            time.sleep(0.02)
        if not future.done():
            future.cancel()
            self._logger.error('抓海参：XY校正响应超时，结果未知，停止后续移动')
            return False
        result = future.result()
        if not result.success:
            self._logger.error(f'抓海参：{result.message}')
            return False
        # 指令坐标也随反馈平移；既有绝对任务点仍保持原定义。
        self._node._cmd_x += delta[0]
        self._node._cmd_y += delta[1]
        self._logger.warning(f'抓海参：{reason}：{result.message}')
        while not self._node.stopped and time.monotonic() < end:
            current = self._measured_pose()
            if current and math.hypot(current[0]-target_xy[0], current[1]-target_xy[1]) < 0.03:
                return True
            time.sleep(0.02)
        return False

    def _move_collection_release_offset(self, deadline):
        offset = getattr(self, '_collection_release_offset', (0., 0.))
        pose = self._measured_pose()
        if pose is None or self._node.stopped or time.monotonic() >= deadline:
            return False
        wx, wy = self._body_to_world(*offset, pose[5])
        target = [pose[0]+wx, pose[1]+wy, pose[2], pose[5]]
        if math.hypot(*offset) >= 1e-9:
            self._logger.info(f'抓海参：收集框释放前爪子偏置body={offset}m，保持深度与航向')
            ok, message = self._node._send_action_goal(
                BasicMotion.Goal.SET, target, 'xyzrz',
                timeout=min(self._command_timeout, deadline-time.monotonic()),
                task_context=self._node._format_motion_context('收集框释放前爪子偏置'))
            if not ok:
                self._logger.error(f'抓海参：收集框爪子偏置失败，不释放：{message}')
                return False
            self._node._cmd_x, self._node._cmd_y, self._node._cmd_z, self._node._cmd_yaw = target
        if not self._wait_visual_motion(target, deadline):
            return False
        settle = self._pre_descent_settle_seconds
        self._logger.info(f'抓海参：收集框偏置实测到位，稳定等待{settle:g}s后释放')
        end = time.monotonic()+settle
        while not self._node.stopped and time.monotonic() < min(end, deadline):
            time.sleep(min(.05, max(0., min(end, deadline)-time.monotonic())))
        return not self._node.stopped and time.monotonic() >= end and time.monotonic() < deadline

    def _align_collection(self, deadline):
        """下视双目框中心伺服；偏置和释放由deliver统一执行。"""
        previous = (self._class_id, self._color, self._projection_depth, self._servo_timeout,
                    self._pixel_tolerance, self._hold_seconds)
        previous_stereo = getattr(self, '_collection_stereo_mode', False)
        try:
            self._collection_stereo_mode = True
            self._collection_last_stamp_ns = -1
            self._class_id, self._color = self._collection_class, '收集框双目'
            self._projection_depth = self._collection_depth
            self._pixel_tolerance = getattr(self, '_collection_pixel_tolerance', .08)
            self._hold_seconds = getattr(self, '_collection_hold_seconds', 0.0)
            self._logger.info('抓海参：收集框使用下视双目检测框中心水平伺服；'
                              '居中后追加爪子偏置、稳定后释放')
            self._servo_timeout = min(self._servo_timeout, max(0., deadline-time.monotonic()))
            self._scan_received_after = time.monotonic()
            self._scan_capture_after_ns = self._node.get_clock().now().nanoseconds
            aligned = self._servo_horizontally()
            if aligned is None or self._node.stopped or time.monotonic() >= deadline:
                self._logger.warning('抓海参：收集框阶段失败：双目视觉未通过、停止或预算耗尽')
                return False
            return True
        finally:
            self._collection_stereo_mode = previous_stereo
            (self._class_id, self._color, self._projection_depth, self._servo_timeout,
             self._pixel_tolerance, self._hold_seconds) = previous

    def _command_gripper(self, angle, label):
        """所有海参抓放统一路由至选定舵机；内部弧度，线协议度。"""
        if getattr(self, '_gripper_servo_id', 1) == 1:
            self._node.set_servo(angle, label)
        else:
            self._node.set_servo(angle, label, servo_id=2)

    def _close_after_press(self, deadline):
        if getattr(self, '_gripper_servo_id', 1) != 2:
            return True  # 原圆盘爪下压卡住海参，没有额外闭合动作。
        if self._node.stopped or time.monotonic() >= deadline:
            return False
        self._command_gripper(self._pickup_angle, '下压完成，舵机2闭合夹住海参')
        end = min(deadline, time.monotonic()+self._close_wait)
        while not self._node.stopped and time.monotonic() < end:
            time.sleep(min(.05, max(0., end-time.monotonic())))
        return not self._node.stopped and time.monotonic() < deadline

    def execute(self) -> bool:
        deadline = time.monotonic() + self._total_timeout
        self._mission_deadline = deadline
        pose = self._measured_pose()
        if pose is None:
            self._logger.error('抓海参：前往抓取区缺少实测位姿，停止任务')
            return False
        # 四个分量均来自配置；x/y/z 已是 START 后 odom 系目标，不能再加上当前位姿。
        search_pose = self._search_pose
        self._logger.info(
            f'抓海参：先前往抓取区 ({search_pose[0]:.3f}, {search_pose[1]:.3f})m，'
            f'扫描目标深度={search_pose[2]:.3f}m，配置航向={search_pose[3]:.1f}°')
        if not self._travel(search_pose, '前往海参抓取区并调整扫描深度',
                            deadline, self._search_travel_timeout):
            self._logger.error('抓海参：未到达抓取区，停止扫描和抓取')
            return False
        failed_attempts = 0
        perception_failures = 0
        while not self._node.stopped and time.monotonic() < deadline:
            if self.confirmed_removed >= self._target_count:
                self._logger.info(f'抓海参：已确认减少 {self.confirmed_removed} 只')
                return True
            self._scan_received_after = time.monotonic()
            self._scan_capture_after_ns = self._node.get_clock().now().nanoseconds
            prepare_angle = (self._release_angle if getattr(self, '_gripper_servo_id', 1) == 2
                             else self._pickup_angle)
            self._command_gripper(prepare_angle, '海参抓取准备')
            self._servo_timeout = min(
                self._servo_timeout, max(0.1, deadline - time.monotonic()))
            recorded_pose = self._servo_horizontally()
            if recorded_pose is None:
                perception_failures += 1
                if self._retry_perception('水平对准本轮未完成', perception_failures, deadline):
                    continue
                break
            before = self._count_visible(
                time.monotonic(), self._node.get_clock().now().nanoseconds,
                deadline, 'before')
            if before is None or before == 0:
                perception_failures += 1
                if self._retry_perception('抓前计数缺失或暂未看到目标', perception_failures, deadline):
                    continue
                break
            if not self._apply_gripper_offset() or not self._wait_pre_descent_settle():
                break
            if time.monotonic() >= deadline:
                break
            self._return_timeout = min(
                self._return_timeout, max(0.1, deadline - time.monotonic()))
            self._descent_duration = min(self._descent_duration, deadline - time.monotonic())
            if (not self._descend() or not self._close_after_press(deadline)
                    or not self._return_to_recorded_pose(recorded_pose)):
                break
            after = self._count_visible(
                time.monotonic(), self._node.get_clock().now().nanoseconds,
                deadline, 'after')
            if after is None:
                self._logger.warning(
                    '抓海参：抓后计数不确定，但已完成下压和回位；'
                    '先去收集区释放可能携带的海参，不增加确认数量，然后返回复查')
                # 无法确认时不能再带着可能已有的海参重复下压；先清空夹爪。
                if not self._deliver(search_pose, deadline, return_to_search=True):
                    break
                perception_failures += 1
                if self._retry_perception('投放后重新确认目标数量', perception_failures, deadline):
                    continue
                break
            perception_failures = 0
            removed = max(0, before - after)
            if not removed:
                failed_attempts += 1
                self._logger.warning(
                    f'抓海参：数量未减少（{before}→{after}），'
                    f'重试 {failed_attempts}/{self._max_failed_attempts}')
                if failed_attempts > self._max_failed_attempts:
                    break
                continue
            failed_attempts = 0
            self.confirmed_removed += removed
            self._logger.info(f'抓海参：观察位数量 {before}→{after}，本次可能抓起 {removed} 只')
            if not self._deliver(
                    search_pose, deadline,
                    return_to_search=self.confirmed_removed < self._target_count):
                break
            if self.confirmed_removed >= self._target_count:
                self._logger.info(
                    f'抓海参：视觉确认减少 {self.confirmed_removed} 只，'
                    f'已发投放指令 {self.delivery_commands} 次')
                return True
        self._logger.error(
            f'抓海参：任务停止；视觉确认减少={self.confirmed_removed}/'
            f'{self._target_count}，已发投放指令={self.delivery_commands}')
        return False
