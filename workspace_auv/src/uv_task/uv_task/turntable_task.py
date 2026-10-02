"""真机转盘任务：观测黄标角度，三次退出式 yaw 推动。

坐标约定：里程计和机体系 z 向下，机体 x 向前、y 向右；转盘轴
位于水平面。转盘正面朝向 AUV 的法向由 disk_axis_yaw_deg 给出。
图像角度 0° 向右、90° 向上，与真实机械角度的对应必须现场标定。

本任务不在任何仿真 mission 中自动启动。没有几何标定、可辨识黄标、
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
            angle = float(data.get('angle_deg', float('nan')))
            center = data.get('disk_center_px', ())
            radius = float(data.get('disk_radius_px', float('nan')))
            if (not data.get('valid') or stamp <= 0 or not math.isfinite(angle)
                    or not math.isfinite(radius) or radius <= 0
                    or len(center) != 2 or not all(math.isfinite(float(v)) for v in center)):
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

    def _calibration(self):
        p = self.params
        center = tuple(_required_float(p, f'disk_center_{axis}') for axis in 'xyz')
        tip = tuple(_required_float(p, f'rod_tip_{axis}') for axis in 'xyz')
        yaw = _required_float(p, 'disk_axis_yaw_deg')
        inner = _required_float(p, 'inner_radius_m', positive=True)
        outer = _required_float(p, 'outer_radius_m', positive=True)
        radius = _required_float(p, 'contact_radius_m', positive=True)
        if not inner < radius < outer:
            raise ValueError('棍端接触半径必须在内外圆环之间')
        if min(radius - inner, outer - radius) < _required_float(p, 'rod_radius_m', True) + 0.01:
            raise ValueError('内外圆环净空不足以容纳细棍及 1cm 安全裕量')
        _required_float(p, 'label_to_hole_deg')
        if int(p.get('image_angle_to_disk_sign', 0)) not in (-1, 1):
            raise ValueError('image_angle_to_disk_sign 必须现场标定为 +1 或 -1')
        _required_float(p, 'approach_standoff_m', positive=True)
        _required_float(p, 'insert_depth_m', positive=True)
        return center, tip, yaw, radius

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
        current = self._wait_new_observation(last_stamp)
        # 只在同一观察位姿下比较圆心/视半径；移动后的图像必须用相机
        # 外参重投影，不能直接套用静止阈值。因此这里只对动作结束后
        # 的画面做保守的突变检查，不把它当完整的盘轴平移估计。
        center0, center1 = reference['disk_center_px'], current['disk_center_px']
        radius0, radius1 = reference['disk_radius_px'], current['disk_radius_px']
        shift = math.hypot(center1[0] - center0[0], center1[1] - center0[1])
        if shift > float(self.params.get('max_disk_image_shift_px', 120.0)):
            raise RuntimeError(f'盘心图像位移 {shift:.1f}px 超限；疑似拉动转盘')
        if abs(radius1 / radius0 - 1.0) > 0.2:
            raise RuntimeError('转盘视半径突变；疑似盘体位移或测量错误')
        return current

    def execute(self):
        try:
            center, tip, axis_yaw, radius = self._calibration()
            observation = self._wait_new_observation(0, timeout=8.0)
            self._measured_pose()
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
                f'转盘计划：黄标={observation["angle_deg"]:.1f}° 孔位={hole:.1f}° '
                f'预插入机体位姿=({pre_robot[0]:.3f},{pre_robot[1]:.3f},'
                f'{pre_robot[2]:.3f},{axis_yaw:.1f}°)，'
                f'三次 yaw 行程 {direction * stroke_yaw:.1f}°')
            if not bool(p.get('allow_contact_motion', False)):
                raise RuntimeError('仅完成观测和动作计划；allow_contact_motion=false，不接触转盘')
            if not bool(p.get('force_limited_control_confirmed', False)):
                raise RuntimeError('尚未确认真机控制器的低速/限推力能力；禁止自动接触')

            # yaw 不是真正的圆周轨迹：棍端会沿盘轴产生附带位移。
            # 若最初一个行程的几何预测超过保守限值，任何接触都不执行。
            swept = _tip_world(pre_robot, axis_yaw + direction * stroke_yaw, tip)
            axis = math.radians(axis_yaw)
            axial_error = abs((swept[0] - pre_tip[0]) * math.cos(axis)
                              + (swept[1] - pre_tip[1]) * math.sin(axis))
            if axial_error > 0.01:
                raise RuntimeError(
                    f'yaw 行程预计产生 {axial_error:.3f}m 轴向拖曳，超过 1cm 上限')

            # 接近位姿本身由现场测量的中心/轴线/棍端外参确定；没有这些
            # 数值时上方校验已失败。接近后需要重新确认黄标可见。
            self._motion(BasicMotion.Goal.WTRAVEL,
                         [*pre_robot, axis_yaw], 'xyzrz', '接近转盘', 90.0)
            observation = self._wait_new_observation(observation['capture_stamp_ns'])
            self._measured_pose()
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
                after = self._wait_new_observation(observation['capture_stamp_ns'])
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
                    observation = self._wait_new_observation(after['capture_stamp_ns'])
                    # 标签已转动，孔位也随之移动；不能盲目再次插入。
                    # 新孔位偏差超过杆孔几何余量则需要重新接近定位。
                    new_hole = (hole + image_sign * sign * total_rotation) % 360.0
                    target_tip = _hole_world(center, axis_yaw, radius, new_hole, -standoff)
                    target_robot = _robot_for_tip(target_tip, axis_yaw, tip)
                    self._motion(BasicMotion.Goal.WTRAVEL,
                                 [*target_robot, axis_yaw], 'xyzrz', '盘外重新对孔', 60.0)
                    observation = self._wait_new_observation(observation['capture_stamp_ns'])
            self.log.info(
                f'转盘任务完成：三次有效推动，累计黄标转角 {total_rotation:.1f}°；'
                '未指定绝对目标角')
            return True
        except (ValueError, RuntimeError, KeyError, TypeError) as exc:
            self.log.error(f'转盘任务停止：{exc}')
            return False
