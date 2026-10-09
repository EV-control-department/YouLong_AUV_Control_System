"""真机转盘任务：左下孔单次插入，上浮、yaw推动、下沉、退杆。

坐标约定：里程计和机体系 z 向下，机体 x 向前、y 向右；转盘轴
位于水平面。盘面法向与轴心由前视双目视觉给出。
图像角度 0° 向右、90° 向上；黄色条幅视为四根辐条之一。

本任务不在任何仿真 mission 中自动启动。每次插入前只用视觉定位孔位，
不根据盘面转角判断任务完成；没有有效观测、里程计或显式放行则禁动。
"""

from __future__ import annotations

import json
import math
import threading
import time

from std_msgs.msg import String

from uv_msgs.action import BasicMotion
from uv_msgs.msg import PoseInfo


# 已知盘体尺寸和名义棍端外参；接触前必须在实物上复核左右镜像与棍端位置。
DISK_DIAMETER_M = 0.230
HOLE_INNER_RADIUS_M = 0.0175
HOLE_OUTER_RADIUS_M = 0.100
SPOKE_WIDTH_M = 0.020
CONTACT_RADIUS_M = (HOLE_INNER_RADIUS_M + HOLE_OUTER_RADIUS_M) / 2
FRONT_CAMERA_CENTER_BODY = (0.230, 0.0, 0.076)
ROD_TIP_BODY = (FRONT_CAMERA_CENTER_BODY[0] + 0.160,
                FRONT_CAMERA_CENTER_BODY[1] - 0.090,
                FRONT_CAMERA_CENTER_BODY[2])
STROKE_COUNT = 1
YAW_STEP_DEG = 2.0
DRIVE_YAW_SIGN = 1


def _wrap(angle):
    return (float(angle) + 180.0) % 360.0 - 180.0


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
        self._last_observation_error = None
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
            if not data.get('valid'):
                with self._lock:
                    self._last_observation_error = (
                        time.monotonic(), str(data.get('reason', '视觉观测无效')))
                return
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
                self._last_observation_error = None

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
        with self._lock:
            error = self._last_observation_error
        stage = '初始等待' if last_stamp == 0 else '动作后等待'
        reason = f'；最近无效原因={error[1]}' if error and time.monotonic() - error[0] < 5.0 else ''
        raise RuntimeError(f'{stage}超时：没有新的有效转盘视觉观测{reason}')

    def _after_motion_observation(self, last_stamp):
        # 不能把动作执行途中采集的帧误当作到位后的闭环测量。
        cutoff = max(last_stamp, self.node.get_clock().now().nanoseconds)
        return self._wait_new_observation(cutoff)

    def _calibration(self):
        p = self.params
        rod_radius = _required_float(p, 'rod_radius_m', positive=True)
        if min(CONTACT_RADIUS_M - HOLE_INNER_RADIUS_M,
               HOLE_OUTER_RADIUS_M - CONTACT_RADIUS_M) < rod_radius + 0.01:
            raise ValueError('内外圆环净空不足以容纳细棍及 1cm 安全裕量')
        if CONTACT_RADIUS_M * math.sin(math.pi / 4) - SPOKE_WIDTH_M / 2 < rod_radius + 0.01:
            raise ValueError('孔位与 20mm 条幅的切向净空不足')
        _required_float(p, 'approach_standoff_m', positive=True)
        _required_float(p, 'insert_depth_m', positive=True)
        return ROD_TIP_BODY, CONTACT_RADIUS_M

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
            raise RuntimeError('未找到黄色条幅；无法确定孔位，禁止插杆')

    @classmethod
    def _hole_angle(cls, observation):
        cls._require_phase(observation)
        # 黄色条幅是一根辐条；相邻辐条间的孔中心在其 45° 处。
        # 前视 optical x 向右、y 向下与 _hole_world 的角度定义一致。
        holes = [(float(observation['angle_deg']) + 45.0 + 90.0 * i) % 360.0
                 for i in range(4)]
        # 左下方向为225°；选择距该方向最近的真实孔中心，不能硬插辐条。
        return min(holes, key=lambda angle: abs(_wrap(angle - 225.0)))

    def _check_hole_alignment(self, observation, tip_body, radius, standoff):
        self._require_phase(observation)
        center, axis_yaw = self._vision_geometry(observation)
        hole = self._hole_angle(observation)
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
        if command == BasicMotion.Goal.WTRAVEL:
            # WTRAVEL保持行进方向，必须显式恢复正视航向及最终位置。
            ok, message = self.node._send_action_goal(
                BasicMotion.Goal.SET, [float(v) for v in target], axes, timeout,
                task_context=f'turntable:{context}:最终定位归向')
            if not ok:
                raise RuntimeError(f'{context} 最终定位归向失败：{message}')
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
        if command == BasicMotion.Goal.BMOVE and axes == 'z':
            expected = before[2] + float(target[2])
            deadline = time.monotonic() + min(timeout, 8.0)
            while time.monotonic() < deadline and not self.node.stopped:
                if abs(self._measured_pose()[2] - expected) < 0.005:
                    return
                time.sleep(0.05)
            raise RuntimeError(f'{context} 未产生预期升沉；禁止继续接触')
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

    def _step_axis(self, distance, axes, context):
        """单次动作内部小步执行，不代表重复插杆；z向下为正。"""
        index, limit = {'x': (0, 0.02), 'z': (2, 0.01), 'rz': (3, YAW_STEP_DEG)}[axes]
        remaining = abs(distance)
        while remaining > 1e-5:
            step = min(limit, remaining) * (1 if distance > 0 else -1)
            target = [0., 0., 0., 0.]
            target[index] = step
            self._motion(BasicMotion.Goal.BMOVE, target, axes, context)
            remaining -= abs(step)

    def _coarse_position(self):
        """可选盘心名义位姿只用于盘外接近；不可代替视觉插孔坐标。"""
        nominal = self.params.get('disk_pose_odom', [])
        if not nominal:
            return
        if len(nominal) != 4:
            raise ValueError('disk_pose_odom必须为[x, y, z, yaw_deg]或空列表')
        x, y, z, yaw = (float(v) for v in nominal)
        if not all(math.isfinite(v) for v in (x, y, z, yaw)) or z < 0:
            raise ValueError('disk_pose_odom必须为有限数值，z向下且非负')
        if not (self.params.get('allow_contact_motion', False)
                and self.params.get('force_limited_control_confirmed', False)):
            self.log.info('转盘名义odom位姿已填写，但运动未放行，不执行粗定位')
            return
        pose_deadline = time.monotonic() + 3.0
        while True:
            try:
                self._measured_pose()
                break
            except RuntimeError:
                if self.node.stopped or time.monotonic() >= pose_deadline:
                    raise
                time.sleep(.05)
        axis = math.radians(yaw)
        camera = (x - .55 * math.cos(axis), y - .55 * math.sin(axis), z)
        robot = _robot_for_tip(camera, yaw, FRONT_CAMERA_CENTER_BODY)
        self.log.info(f'转盘odom粗定位：盘心={(x, y, z)}，正视航向={yaw:.1f}°，'
                      f'AUV目标={robot}；相机距盘面0.55m，随后视觉重测')
        self._motion(BasicMotion.Goal.WTRAVEL, [*robot, yaw], 'xyzrz',
                     '盘心odom粗定位', 90.0)
        # 清除移动前/移动中的观测，后续只接收新的视觉测量。
        with self._lock:
            self._observation = None
        self._coarse_capture_cutoff = self.node.get_clock().now().nanoseconds

    def execute(self):
        try:
            tip, radius = self._calibration()
            self._coarse_position()
            observation = self._wait_new_observation(
                getattr(self, '_coarse_capture_cutoff', 0), timeout=8.0)
            self._measured_pose()
            center, axis_yaw = self._vision_geometry(observation, require_front=False)
            p = self.params
            stroke_yaw = float(p.get('stroke_yaw_deg', 6.0))
            if not math.isfinite(stroke_yaw) or not 0 < stroke_yaw <= 12.0:
                raise ValueError('单次 yaw 行程必须在 (0, 12]°')
            ascent = _required_float(p, 'contact_ascent_m', positive=True)
            descent = _required_float(p, 'contact_descent_m', positive=True)
            if ascent > 0.05 or descent > 0.05:
                raise ValueError('接触升沉单段行程不得超过5cm；需先实测机构轨迹')
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
            axis = math.radians(axis_yaw)
            align_camera = (center[0]-0.55*math.cos(axis),
                            center[1]-0.55*math.sin(axis), center[2])
            align_robot = _robot_for_tip(align_camera, axis_yaw, FRONT_CAMERA_CENTER_BODY)
            self._motion(BasicMotion.Goal.WTRAVEL,
                         [*align_robot, axis_yaw], 'xyzrz', '视觉正视对准', 90.0)
            observation = self._after_motion_observation(observation['capture_stamp_ns'])
            insertion = standoff + depth
            center, axis_yaw = self._vision_geometry(observation)
            hole = self._hole_angle(observation)
            pre_tip = _hole_world(center, axis_yaw, radius, hole, -standoff)
            pre_robot = _robot_for_tip(pre_tip, axis_yaw, tip)
            self._check_yaw_sweep(pre_robot, pre_tip, axis_yaw, tip,
                                  stroke_yaw, DRIVE_YAW_SIGN)
            self.log.info(f'单次左下孔动作：孔角={hole:.1f}°，上浮={ascent:.3f}m，'
                          f'yaw={stroke_yaw:.1f}°，下沉={descent:.3f}m；待实测圆盘转角')
            self._motion(BasicMotion.Goal.WTRAVEL,
                         [*pre_robot, axis_yaw], 'xyzrz', '左下孔盘外对准', 90.0)
            observation = self._after_motion_observation(observation['capture_stamp_ns'])
            self._check_hole_alignment(observation, tip, radius, standoff)
            # 仅一次插入；升沉/yaw小步是同一次接触动作，不重新插杆。
            self._step_axis(insertion, 'x', '单次插入')
            self._step_axis(-ascent, 'z', '插杆后上浮')
            self._step_axis(DRIVE_YAW_SIGN * stroke_yaw, 'rz', '上浮后yaw推盘')
            self._step_axis(descent, 'z', 'yaw后下沉')
            self._step_axis(-insertion, 'x', '单次退出')
            self.log.info('转盘单次上浮-yaw-下沉动作已完成；未测量或保证实际180°转角')
            return True
        except (ValueError, RuntimeError, KeyError, TypeError) as exc:
            self.log.error(f'转盘任务停止：{exc}')
            return False
