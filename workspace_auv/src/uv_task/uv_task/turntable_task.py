"""真机转盘任务：视觉锁定盘心/盘轴后，三次退出式 yaw 推动。

坐标约定：里程计和机体系 z 向下，机体 x 向前、y 向右；转盘轴
位于水平面。盘面法向与轴心由前视双目视觉给出。
图像角度 0° 向右、90° 向上，与真实机械角度的对应必须现场标定。

本任务不在任何仿真 mission 中自动启动。没有几何标定、可辨识盘面相位、
有效里程计或者显式 allow_contact_motion 时，只给出诊断，不会接触转盘。
"""

from __future__ import annotations

import json
import math
import threading
import time

from std_msgs.msg import String

from uv_msgs.action import BasicMotion
from uv_msgs.msg import PoseInfo


def _wrap(angle):
    return (float(angle) + 180.0) % 360.0 - 180.0


def _rotation_delta(start, end):
    """只比较一次短行程；跨过 ±180° 时仍保留正确差值。"""
    return _wrap(end - start)


def _tip_world(robot_xyz, yaw_deg, tip_body):
    yaw = math.radians(yaw_deg)
    x, y, z = robot_xyz
    tx, ty, tz = tip_body
    return (x + math.cos(yaw) * tx - math.sin(yaw) * ty,
            y + math.sin(yaw) * tx + math.cos(yaw) * ty, z + tz)


def _robot_for_tip(tip_xyz, yaw_deg, tip_body):
    """逆运动学：已知希望的棍端位置，求 AUV 机体原点。"""
    yaw = math.radians(yaw_deg)
    tx, ty, tz = tip_body
    return (tip_xyz[0] - math.cos(yaw) * tx + math.sin(yaw) * ty,
            tip_xyz[1] - math.sin(yaw) * tx - math.cos(yaw) * ty,
            tip_xyz[2] - tz)


def _hole_world(center, axis_yaw_deg, radius, hole_angle_deg, axial_offset):
    """盘面孔位：角度 0° 沿盘面水平切向，90° 沿竖直向上。"""
    axis = math.radians(axis_yaw_deg)
    hole = math.radians(hole_angle_deg)
    nx, ny = math.cos(axis), math.sin(axis)
    tangent = (-ny, nx)
    return (center[0] + axial_offset * nx + radius * math.cos(hole) * tangent[0],
            center[1] + axial_offset * ny + radius * math.cos(hole) * tangent[1],
            center[2] - radius * math.sin(hole))


def _required_float(params, name, positive=False):
    value = float(params.get(name, float('nan')))
    if not math.isfinite(value) or (positive and value <= 0):
        raise ValueError(f'{name} 尚未标定')
    return value


class TurntableTask:
    """只消费 camera 发布的小型 JSON 观测，绝不订阅图像 topic。"""

    def __init__(self, node, params):
        self.node = node
        self.params = params
        self.log = node.get_logger()
        self._lock = threading.RLock()
        self._observation = None
        self._pose = None
        self._obs_sub = node.create_subscription(
            String, '/perception/turntable/observation', self._on_observation, 10)
        self._pose_sub = node.create_subscription(
            PoseInfo, '/basic_motion/pose_info', self._on_pose, 10)

    def destroy(self):
        self.node.destroy_subscription(self._obs_sub)
        self.node.destroy_subscription(self._pose_sub)

    def _on_observation(self, msg):
        try:
            data = json.loads(msg.data)
            stamp = int(data['capture_stamp_ns'])
            center = data.get('disk_center_px', ())
            world = data.get('disk_center_world', ())
            radius = float(data.get('disk_radius_px', float('nan')))
            axis = float(data.get('disk_axis_yaw_deg', float('nan')))
            if (not data.get('valid') or stamp <= 0 or not math.isfinite(axis)
                    or not math.isfinite(radius) or radius <= 0
                    or len(center) != 2 or not all(math.isfinite(float(v)) for v in center)
                    or len(world) != 3 or not all(math.isfinite(float(v)) for v in world)):
                return
        except (ValueError, KeyError, TypeError, OverflowError):
            return
        with self._lock:
            # 推理可能积压；只保留采集时间更新的一帧，禁止旧帧倒灌。
            if self._observation is None or stamp > self._observation[1]['capture_stamp_ns']:
                self._observation = (time.monotonic(), data)

    def _on_pose(self, msg):
        pose = (float(msg.robot_x), float(msg.robot_y), float(msg.robot_z),
                float(msg.robot_yaw))
        if all(math.isfinite(v) for v in pose):
            with self._lock:
                self._pose = (time.monotonic(), pose)

    def _latest(self, newer_than_stamp=0):
        with self._lock:
            entry = self._observation
        if entry is None or time.monotonic() - entry[0] > 1.0:
            raise RuntimeError('前视转盘观测超时；检查分割模型、推流及相机节点')
        observation = entry[1]
        if observation['capture_stamp_ns'] <= newer_than_stamp:
            raise RuntimeError('转盘观测没有更新；拒绝使用接触前的旧图像')
        # 到达任务节点的时间不能代表图像新鲜度。真机 camera 与任务节点
        # 使用相同 ROS 时钟；采集时间相对当前 ROS 时间过旧即拒绝。
        capture_age_s = (
            self.node.get_clock().now().nanoseconds - observation['capture_stamp_ns']) / 1e9
        if not -0.2 <= capture_age_s <= 1.0:
            raise RuntimeError(f'转盘图像采集时间延迟 {capture_age_s:.2f}s')
        return observation

    def _measured_pose(self):
        with self._lock:
            entry = self._pose
        if entry is None or time.monotonic() - entry[0] > 1.0:
            raise RuntimeError('缺少新鲜的实测里程计；不使用任务指令位姿代替')
        return entry[1]

    def _wait_new_observation(self, last_stamp, timeout=3.0):
        deadline = time.monotonic() + timeout
        while not self.node.stopped and time.monotonic() < deadline:
            try:
                return self._latest(last_stamp)
            except RuntimeError:
                time.sleep(0.05)
        raise RuntimeError('动作后没有新的转盘视觉观测')

    def _after_motion_observation(self, last_stamp):
        # 不能把动作执行途中采集的帧误当作到位后的闭环测量。
        cutoff = max(last_stamp, self.node.get_clock().now().nanoseconds)
        return self._wait_new_observation(cutoff)

    def _calibration(self):
        p = self.params
        camera = tuple(_required_float(p, f'front_camera_center_{axis}') for axis in 'xyz')
        root = (0.0, -0.09, 0.0)
        tip = (camera[0] + root[0] + 0.16,
               camera[1] + root[1], camera[2] + root[2])
        inner = _required_float(p, 'inner_radius_m', positive=True)
        outer = _required_float(p, 'outer_radius_m', positive=True)
        radius = _required_float(p, 'contact_radius_m', positive=True)
        diameter = _required_float(p, 'disk_diameter_m', positive=True)
        spoke_width = _required_float(p, 'spoke_width_m', positive=True)
        rod_radius = _required_float(p, 'rod_radius_m', positive=True)
        if not 0 < inner < radius < outer < diameter/2 or abs(diameter-0.230) > 0.005:
            raise ValueError('棍端接触半径必须在内外圆环之间')
        if min(radius - inner, outer - radius) < rod_radius + 0.01:
            raise ValueError('内外圆环净空不足以容纳细棍及 1cm 安全裕量')
        # 四根等间隔辐条；孔位按相邻辐条的角平分线标定。
        # 这里只验证静态间隙，yaw 扫掠中的棍端轨迹仍须空载验证。
        if radius * math.sin(math.pi / 4) - spoke_width / 2 < rod_radius + 0.01:
            raise ValueError('孔位与 20mm 条幅的切向净空不足')
        _required_float(p, 'label_to_hole_deg')
        if int(p.get('image_angle_to_disk_sign', 0)) not in (-1, 1):
            raise ValueError('image_angle_to_disk_sign 必须现场标定为 +1 或 -1')
        _required_float(p, 'approach_standoff_m', positive=True)
        _required_float(p, 'insert_depth_m', positive=True)
        return tip, radius

    def _vision_geometry(self, observation, require_front=True):
        center = tuple(float(v) for v in observation['disk_center_world'])
        axis_yaw = float(observation['disk_axis_yaw_deg'])
        ratio = float(observation.get('axis_ratio', 0.0))
        residual = float(observation.get('plane_residual_m', float('inf')))
        if ratio < (0.88 if require_front else 0.70) or residual > 0.02:
            raise RuntimeError(f'盘面尚未正视或平面质量不足：长短轴比={ratio:.2f}，残差={residual:.3f}m')
        if abs(_wrap(axis_yaw-self._measured_pose()[3])) > (15.0 if require_front else 45.0):
            raise RuntimeError('前视盘轴与机体朝向差异过大；需重新接近')
        return center, axis_yaw

    @staticmethod
    def _require_phase(observation):
        if not observation.get('phase_valid') or not math.isfinite(
                float(observation.get('angle_deg', float('nan')))):
            raise RuntimeError('未找到盘面黄色标记；圆形轮廓不能确定绝对相位，禁止插杆')

    def _check_hole_alignment(self, observation, tip_body, radius, standoff):
        self._require_phase(observation)
        center, axis_yaw = self._vision_geometry(observation)
        phase = (int(self.params['image_angle_to_disk_sign'])*observation['angle_deg']+
                 float(self.params['label_to_hole_deg']))
        holes = [(phase+90.0*i) % 360.0 for i in range(4)]
        hole = max(holes, key=lambda value: abs(math.sin(math.radians(value))))
        desired = _hole_world(center, axis_yaw, radius, hole, -standoff)
        pose = self._measured_pose()
        actual = _tip_world(pose[:3], pose[3], tip_body)
        error = math.dist(actual, desired)
        if error > 0.025:
            raise RuntimeError(f'插杆前视觉重测孔位误差 {error:.3f}m >2.5cm；停止接触')

    @staticmethod
    def _check_yaw_sweep(pre_robot, pre_tip, axis_yaw, tip, stroke_yaw, direction):
        swept = _tip_world(pre_robot, axis_yaw+direction*stroke_yaw, tip)
        axis = math.radians(axis_yaw)
        axial_error = abs((swept[0]-pre_tip[0])*math.cos(axis)+
                          (swept[1]-pre_tip[1])*math.sin(axis))
        if axial_error > 0.01:
            raise RuntimeError(f'yaw 行程预计轴向拖曳 {axial_error:.3f}m >1cm')

    def _motion(self, command, target, axes, context, timeout=30.0):
        if self.node.stopped:
            raise RuntimeError('任务被停止')
        before = self._measured_pose()
        ok, message = self.node._send_action_goal(
            command, [float(v) for v in target], axes, timeout,
            task_context=f'turntable:{context}')
        if not ok:
            raise RuntimeError(f'{context} 执行失败：{message}')
        # BasicMotion 的位置容差约 0.1m，小行程可能立刻返回 SUCCESS。
        # 因此任务再检查实测里程计是否真的产生了相应位移。
        if command == BasicMotion.Goal.BMOVE and axes == 'x':
            delta = float(target[0])
            cy, sy = math.cos(math.radians(before[3])), math.sin(math.radians(before[3]))
            expected = (before[0] + cy * delta, before[1] + sy * delta)
            deadline = time.monotonic() + min(timeout, 8.0)
            while time.monotonic() < deadline and not self.node.stopped:
                pose = self._measured_pose()
                if math.hypot(pose[0] - expected[0], pose[1] - expected[1]) < 0.015:
                    return
                time.sleep(0.05)
            raise RuntimeError(f'{context} 未产生预期位移；禁止继续接触')
        if command == BasicMotion.Goal.BMOVE and axes == 'rz':
            expected = _wrap(before[3] + target[3])
            deadline = time.monotonic() + min(timeout, 8.0)
            while time.monotonic() < deadline and not self.node.stopped:
                if abs(_wrap(self._measured_pose()[3] - expected)) < 1.0:
                    return
                time.sleep(0.05)
            raise RuntimeError(f'{context} 未产生预期偏航；禁止继续接触')
        if command == BasicMotion.Goal.WTRAVEL:
            deadline = time.monotonic() + 10.0
            while time.monotonic() < deadline and not self.node.stopped:
                pose = self._measured_pose()
                position_error = math.dist(pose[:3], target[:3])
                yaw_error = abs(_wrap(pose[3] - target[3]))
                if position_error < 0.025 and yaw_error < 2.0:
                    return
                time.sleep(0.05)
            raise RuntimeError(f'{context} 位姿未达到接触精度 2.5cm/2°')

    def _check_disk(self, reference, last_stamp):
        current = self._after_motion_observation(last_stamp)
        # 世界盘心比图像像素更适合比较机器人运动前后的盘体位移。
        center0, center1 = reference['disk_center_world'], current['disk_center_world']
        shift = math.dist(center0, center1)
        if shift > float(self.params.get('max_disk_world_shift_m', 0.05)):
            raise RuntimeError(f'盘心世界坐标位移 {shift:.3f}m 超限；疑似拉动转盘')
        return current

    def execute(self):
        try:
            tip, radius = self._calibration()
            observation = self._wait_new_observation(0, timeout=8.0)
            self._measured_pose()
            center, axis_yaw = self._vision_geometry(observation, require_front=False)
            p = self.params
            stroke_count = int(p.get('stroke_count', 3))
            stroke_yaw = _required_float(p, 'stroke_yaw_deg', positive=True)
            yaw_step = _required_float(p, 'yaw_step_deg', positive=True)
            direction = int(p.get('drive_yaw_sign', 0))
            if (stroke_count != 3 or not 0 < yaw_step <= 3.0
                    or not 0 < stroke_yaw <= 12.0
                    or direction not in (-1, 1)):
                raise ValueError('仅支持三次行程、每步≤3°、每程≤12°，且必须标定 yaw 方向')
            standoff = float(p['approach_standoff_m'])
            depth = float(p['insert_depth_m'])
            if standoff > 0.3 or depth > 0.06:
                raise ValueError('接近距离或插入深度超过任务保守上限')

            self.log.info(f'转盘初始视觉：盘心={center}，盘轴={axis_yaw:.1f}°，'
                          f'长短轴比={observation["axis_ratio"]:.2f}，'
                          f'有效深度点={observation.get("depth_points", 0)}')
            if not bool(p.get('allow_contact_motion', False)):
                raise RuntimeError('仅完成视觉定位；allow_contact_motion=false，不发送运动命令')
            if not bool(p.get('force_limited_control_confirmed', False)):
                raise RuntimeError('尚未确认真机控制器低速/限推力；禁止自动接触')

            # 第一阶段只做非接触正视对准。盘轴来自 SGBM 平面法向，
            # 而非任务文件中的名义朝向；盘心也不取手填坐标。
            camera_body = tuple(_required_float(p, f'front_camera_center_{a}') for a in 'xyz')
            axis = math.radians(axis_yaw)
            align_camera = (center[0]-0.55*math.cos(axis),
                            center[1]-0.55*math.sin(axis), center[2])
            align_robot = _robot_for_tip(align_camera, axis_yaw, camera_body)
            self._motion(BasicMotion.Goal.WTRAVEL,
                         [*align_robot, axis_yaw], 'xyzrz', '视觉正视对准', 90.0)
            observation = self._after_motion_observation(observation['capture_stamp_ns'])
            center, axis_yaw = self._vision_geometry(observation)
            self._check_hole_alignment(observation, tip, radius, standoff)
            # 黄色标签角度 + 标定偏角确定四个孔位。yaw 使棍端水平
            # 摆动，只有靠近盘的顶部/底部，水平力才有足够的切向分量。
            # 因此选择最靠近竖直方向的孔。安装相位需实测。
            image_sign = int(p['image_angle_to_disk_sign'])
            holes = [(image_sign * observation['angle_deg'] + float(p['label_to_hole_deg'])
                      + 90.0 * index) % 360.0 for index in range(4)]
            hole = max(holes, key=lambda value: abs(math.sin(math.radians(value))))
            pre_tip = _hole_world(center, axis_yaw, radius, hole, -standoff)
            pre_robot = _robot_for_tip(pre_tip, axis_yaw, tip)
            self.log.info(
                f'转盘视觉：盘心={center}，盘轴={axis_yaw:.1f}°，'
                f'长短轴比={observation["axis_ratio"]:.2f}，'
                f'深度点={observation.get("depth_points", 0)}；'
                f'黄标={observation["angle_deg"]:.1f}° 孔位={hole:.1f}° '
                f'预插入机体位姿=({pre_robot[0]:.3f},{pre_robot[1]:.3f},'
                f'{pre_robot[2]:.3f},{axis_yaw:.1f}°)，'
                f'三次 yaw 行程 {direction * stroke_yaw:.1f}°')
            # yaw 不是真正的圆周轨迹：棍端会沿盘轴产生附带位移。
            # 若最初一个行程的几何预测超过保守限值，任何接触都不执行。
            self._check_yaw_sweep(pre_robot, pre_tip, axis_yaw, tip,
                                  stroke_yaw, direction)

            # 接近位姿本身由现场测量的中心/轴线/棍端外参确定；没有这些
            # 数值时上方校验已失败。接近后需要重新确认黄标可见。
            self._motion(BasicMotion.Goal.WTRAVEL,
                         [*pre_robot, axis_yaw], 'xyzrz', '接近转盘', 90.0)
            observation = self._after_motion_observation(observation['capture_stamp_ns'])
            # 初始位置只是视觉粗接近；到位后按新图像重新计算真正孔位。
            center, axis_yaw = self._vision_geometry(observation)
            self._require_phase(observation)
            holes = [(image_sign*observation['angle_deg']+float(p['label_to_hole_deg'])
                      +90.0*index) % 360.0 for index in range(4)]
            hole = max(holes, key=lambda value: abs(math.sin(math.radians(value))))
            pre_tip = _hole_world(center, axis_yaw, radius, hole, -standoff)
            pre_robot = _robot_for_tip(pre_tip, axis_yaw, tip)
            self._check_yaw_sweep(pre_robot, pre_tip, axis_yaw, tip,
                                  stroke_yaw, direction)
            self._motion(BasicMotion.Goal.WTRAVEL,
                         [*pre_robot, axis_yaw], 'xyzrz', '视觉精对孔', 60.0)
            observation = self._after_motion_observation(observation['capture_stamp_ns'])
            center, axis_yaw = self._vision_geometry(observation)
            self._require_phase(observation)
            total_rotation = 0.0
            last_direction = None
            insertion = standoff + depth
            for stroke in range(stroke_count):
                if self.node.stopped:
                    raise RuntimeError('任务被停止')
                before = observation
                # 小步插入，避免一次性撞到四根辐条。此处控制的是位置，
                # 并非推力；真机必须另行完成低速控制器/限推力标定。
                remaining = insertion
                while remaining > 1e-5:
                    step = min(0.02, remaining)
                    self._motion(BasicMotion.Goal.BMOVE,
                                 [step, 0, 0, 0], 'x', f'第{stroke+1}次插入')
                    remaining -= step
                observation = self._check_disk(before, before['capture_stamp_ns'])

                # 细棍保持插入后用短 yaw 行程向同一方向推。每步等待实测
                # yaw 到位以及新视觉观测；行程较短以控制轴向拖曳。
                remaining = stroke_yaw
                before = observation
                while remaining > 1e-5:
                    step = min(yaw_step, remaining) * direction
                    self._motion(BasicMotion.Goal.BMOVE,
                                 [0, 0, 0, step], 'rz', f'第{stroke+1}次推盘')
                    observation = self._check_disk(observation, observation['capture_stamp_ns'])
                    remaining -= abs(step)

                # 严格按“先退出，再复位”顺序；带棍回摆会把转盘倒转。
                remaining = insertion
                while remaining > 1e-5:
                    step = min(0.02, remaining)
                    self._motion(BasicMotion.Goal.BMOVE,
                                 [-step, 0, 0, 0], 'x', f'第{stroke+1}次退出')
                    remaining -= step
                after = self._after_motion_observation(observation['capture_stamp_ns'])
                self._require_phase(after)
                delta = _rotation_delta(before['angle_deg'], after['angle_deg'])
                if abs(delta) < float(p.get('min_progress_deg', 2.0)):
                    raise RuntimeError(f'第{stroke+1}次黄标仅变化 {delta:.1f}°；未确认转动')
                sign = 1 if delta > 0 else -1
                if last_direction is not None and sign != last_direction:
                    raise RuntimeError('黄标转向前后不一致；停止后续行程')
                last_direction = sign
                total_rotation += abs(delta)
                self.log.info(
                    f'第{stroke+1}/3次推盘完成：黄标变化={delta:+.1f}°，'
                    f'累计={total_rotation:.1f}°')
                # 退到盘前再复位 yaw；重新计算下一孔位需要实际图像和
                # 轴线标定。当前只支持同一孔的短角度往复验证。
                if stroke < stroke_count - 1:
                    self._motion(BasicMotion.Goal.BMOVE,
                                 [0, 0, 0, -direction * stroke_yaw], 'rz', '盘外复位')
                    observation = self._after_motion_observation(after['capture_stamp_ns'])
                    # 标签已转动，孔位也随之移动；不能盲目再次插入。
                    # 新孔位偏差超过杆孔几何余量则需要重新接近定位。
                    center, axis_yaw = self._vision_geometry(observation)
                    self._require_phase(observation)
                    holes = [(image_sign*observation['angle_deg']+
                              float(p['label_to_hole_deg'])+90.0*index) % 360.0
                             for index in range(4)]
                    new_hole = max(holes, key=lambda value: abs(math.sin(math.radians(value))))
                    hole = new_hole
                    target_tip = _hole_world(center, axis_yaw, radius, new_hole, -standoff)
                    target_robot = _robot_for_tip(target_tip, axis_yaw, tip)
                    self._check_yaw_sweep(target_robot, target_tip, axis_yaw,
                                          tip, stroke_yaw, direction)
                    self._motion(BasicMotion.Goal.WTRAVEL,
                                 [*target_robot, axis_yaw], 'xyzrz', '盘外重新对孔', 60.0)
                    observation = self._after_motion_observation(observation['capture_stamp_ns'])
                    self._check_hole_alignment(observation, tip, radius, standoff)
            self.log.info(
                f'转盘任务完成：三次有效推动，累计黄标转角 {total_rotation:.1f}°；'
                '未指定绝对目标角')
            return True
        except (ValueError, RuntimeError, KeyError, TypeError) as exc:
            self.log.error(f'转盘任务停止：{exc}')
            return False
