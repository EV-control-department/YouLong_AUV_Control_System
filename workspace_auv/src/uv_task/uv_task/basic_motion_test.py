"""Small, supervised water tests of the BasicMotion action boundary.

The ROS driver checks live feedback while waiting for actions.  This is a
software cancellation aid; neutral velocity does not disarm the MCU.
"""

from __future__ import annotations

from dataclasses import dataclass, fields
import math
import threading
import time

from uv_task.task_outcome import TaskOutcome


class MotionTestError(RuntimeError):
    def __init__(self, stage, message):
        super().__init__(message)
        self.stage = stage


@dataclass
class Settings:
    stage: str = 'hold'
    reset_origin: bool = False
    test_depth: bool = False
    distance_m: float = 0.25
    depth_delta_m: float = 0.15
    yaw_delta_deg: float = 10.0
    speed_mps: float = 0.05
    yaw_rate_deg_s: float = 3.0
    pulse_seconds: float = 1.0
    action_timeout: float = 20.0
    settle_seconds: float = 2.0
    feedback_timeout: float = 1.0
    horizontal_limit_m: float = 1.0
    vertical_limit_m: float = 0.25
    min_battery_voltage: float = 21.0
    external_battery_voltage: float = 0.0
    max_speed_mps: float = 0.20

    @classmethod
    def load(cls, params):
        if not isinstance(params, dict):
            raise MotionTestError('config', '测试参数必须为映射')
        unknown = set(params) - {field.name for field in fields(cls)}
        if unknown:
            raise MotionTestError('config', f'未知测试参数：{sorted(unknown)}')
        value = cls(**params)
        if not isinstance(value.stage, str) or value.stage not in {'hold', 'set', 'bmove', 'wmove', 'travel', 'velocity', 'all'}:
            raise MotionTestError('config', 'stage 应为 hold/set/bmove/wmove/travel/velocity/all')
        if not isinstance(value.reset_origin, bool) or not isinstance(value.test_depth, bool):
            raise MotionTestError('config', 'reset_origin/test_depth 必须为布尔值')
        limits = {
            'distance_m': (0.15, 0.5), 'depth_delta_m': (0.15, 0.3),
            'yaw_delta_deg': (6.0, 15.0), 'speed_mps': (0.01, 0.1),
            'yaw_rate_deg_s': (1.0, 5.0), 'pulse_seconds': (0.5, 2.0),
            'action_timeout': (5.0, 30.0), 'settle_seconds': (0.5, 5.0),
            'feedback_timeout': (0.2, 2.0), 'horizontal_limit_m': (0.5, 2.0),
            'vertical_limit_m': (0.2, 1.0), 'min_battery_voltage': (1.0, 100.0),
            'external_battery_voltage': (0.0, 100.0),
            'max_speed_mps': (0.1, 0.5),
        }
        for key, (low, high) in limits.items():
            number = getattr(value, key)
            if (isinstance(number, bool) or not isinstance(number, (int, float))
                    or not math.isfinite(number) or not low <= number <= high):
                raise MotionTestError('config', f'{key} 必须为 {low}～{high} 内的有限数值')
        if value.horizontal_limit_m < 2 * value.distance_m:
            raise MotionTestError('config', 'horizontal_limit_m 至少应为 distance_m 的两倍')
        if value.vertical_limit_m < value.depth_delta_m + 0.1:
            raise MotionTestError('config', 'vertical_limit_m 至少应比 depth_delta_m 大 0.1m')
        return value


def wrap_yaw(value):
    return (value + 180.0) % 360.0 - 180.0


def expected_target(command, target, pose):
    """Predict [x,y,z,yaw_deg] using the measured pose at command dispatch."""
    if command in (1, 3):
        return list(target)
    if command == 4:
        result = list(target)
        dx, dy = target[0] - pose[0], target[1] - pose[1]
        if math.hypot(dx, dy) > 0.01:
            result[3] = math.degrees(math.atan2(dy, dx))
        return result
    dx, dy, dz, dyaw = target
    if command in (2, 5):
        heading = math.radians(pose[3])
        dx, dy = math.cos(heading) * dx - math.sin(heading) * dy, math.sin(heading) * dx + math.cos(heading) * dy
    yaw = wrap_yaw(pose[3] + dyaw)
    if command == 5 and math.hypot(dx, dy) > 0.01:
        yaw = math.degrees(math.atan2(dy, dx))
    return [pose[0] + dx, pose[1] + dy, pose[2] + dz, yaw]


def build_steps(settings, reference):
    """Every excursion is followed by an absolute return to the entry pose."""
    steps = [('motion', 3, list(reference), '保持进入测试时的位姿')]
    stages = ('set', 'bmove', 'wmove', 'travel', 'velocity') if settings.stage == 'all' else (settings.stage,)
    for stage in stages:
        if stage in ('set', 'bmove', 'wmove'):
            command = {'set': 3, 'bmove': 2, 'wmove': 1}[stage]
            for axis, delta in ((2, settings.depth_delta_m), (3, settings.yaw_delta_deg), (0, settings.distance_m), (1, settings.distance_m)):
                if axis == 2 and not settings.test_depth:
                    continue
                target = list(reference) if command in (1, 3) else [0.0] * 4
                target[axis] += delta
                steps.append(('motion', command, target, f'{stage} 单轴 {axis}'))
                steps.append(('motion', 3, list(reference), '回到测试进入位姿'))
        elif stage == 'travel':
            heading = math.radians(reference[3])
            steps.extend([
                ('motion', 4, [reference[0] + settings.distance_m * math.cos(heading), reference[1] + settings.distance_m * math.sin(heading), reference[2], reference[3]], 'WTRAVEL 沿进入航向前进'),
                ('motion', 3, list(reference), '回到测试进入位姿'),
                ('motion', 5, [settings.distance_m, 0.0, 0.0, 0.0], 'BTRAVEL 前进'),
                ('motion', 3, list(reference), '回到测试进入位姿'),
            ])
        elif stage == 'velocity':
            for axis, speed in ((0, settings.speed_mps), (1, settings.speed_mps), (2, settings.speed_mps), (3, settings.yaw_rate_deg_s)):
                if axis == 2 and not settings.test_depth:
                    continue
                target = [0.0] * 4
                target[axis] = speed
                steps.append(('pulse', 7, target, f'BODY_VELOCITY 单轴 {axis}'))
                steps.append(('motion', 3, list(reference), '回到测试进入位姿'))
            steps.extend([
                ('lease', 7, [settings.speed_mps, 0.0, 0.0, 0.0], '单次速度租约；等待看门狗发零速度'),
                ('motion', 3, list(reference), '回到测试进入位姿'),
            ])
    return steps


def run_test(port, params):
    dispatched = False
    outcome = TaskOutcome.ok()
    try:
        settings = Settings.load(params)
        port.configure(settings)
        port.ready()
        if settings.reset_origin:
            dispatched = True
            port.send(7, [0.0] * 4, 2.0)
            reset_time = port.now()
            port.send(6, [0.0] * 4, 3.0)
            port.wait_reset(reset_time)
        reference = port.pose()
        port.set_reference(reference)
        steps = build_steps(settings, reference)
        for index, (kind, command, target, label) in enumerate(steps, 1):
            port.check()
            port.log(f'basic_motion_test [{index}/{len(steps)}] {label}: {target}')
            dispatched = True
            if kind == 'motion':
                expected = expected_target(command, target, port.pose())
                port.send(command, target, settings.action_timeout)
                port.pause(settings.settle_seconds)
                port.verify_pose(expected)
            elif kind == 'pulse':
                deadline = port.now() + settings.pulse_seconds
                while port.now() < deadline:
                    port.send(7, target, 2.0)
                    port.pause(min(0.1, max(0.0, deadline - port.now())))
                port.send(7, [0.0] * 4, 2.0)
                port.pause(settings.settle_seconds)
            else:
                sent_at = port.now()
                port.send(7, target, 2.0)
                port.pause(0.8)
                port.verify_watchdog(sent_at)
        port.log(f'basic_motion_test PASS stage={settings.stage}')
    except MotionTestError as exc:
        outcome = TaskOutcome.failed(f'basic_motion_test.{exc.stage}', str(exc))
    except Exception as exc:
        outcome = TaskOutcome.failed('basic_motion_test.exception', str(exc))
    finally:
        if dispatched:
            try:
                port.neutral()
            except Exception as exc:
                message = f'{outcome.message}; 零速度确认失败：{exc}'.strip('; ')
                outcome = TaskOutcome.failed('basic_motion_test.stop', message)
    return outcome


class RosMotionTest:
    """Adapter for the existing runner executor; never spins a second executor."""

    def __init__(self, node):
        from geometry_msgs.msg import TwistWithCovarianceStamped
        from zit6_interfaces.msg import ZitSetpoint, ZitStatus
        from uv_msgs.msg import PoseInfo, SensorHealth
        from uv_msgs.action import BasicMotion
        from auv_protocol.topics import STATE_ODOM, STATE_TWIST, STATE_HEALTH, ZIT6_STATUS, ZIT6_SETPOINT

        self.node = node
        self.action_type = BasicMotion
        self.feedback = {}
        self.lock = threading.Lock()
        self.reference = None
        self.settings = Settings()
        self.subscriptions = []
        for key, msg, topic in (
                ('pose', PoseInfo, STATE_ODOM),
                ('twist', TwistWithCovarianceStamped, STATE_TWIST),
                ('health', SensorHealth, STATE_HEALTH),
                ('status', ZitStatus, ZIT6_STATUS),
                ('setpoint', ZitSetpoint, ZIT6_SETPOINT)):
            self.subscriptions.append(node.create_subscription(
                msg, topic, lambda message, name=key: self.receive(name, message), 10))

    @staticmethod
    def now():
        return time.monotonic()

    def receive(self, name, message):
        if name == 'health' and message.sensor_name != 'localization':
            return
        with self.lock:
            self.feedback[name] = (self.now(), message)

    def snapshot(self):
        with self.lock:
            return dict(self.feedback)

    def configure(self, settings):
        self.settings = settings
        self.reference = None

    def log(self, text):
        self.node.get_logger().info(text)

    def pose(self):
        message = self.snapshot()['pose'][1]
        return [float(message.robot_x), float(message.robot_y), float(message.robot_z), float(message.robot_yaw)]

    def set_reference(self, pose):
        self.reference = list(pose)

    def check(self):
        if self.node.stopped:
            raise MotionTestError('stopped', '收到停止请求')
        feedback = self.snapshot()
        for name in ('pose', 'twist', 'health', 'status'):
            if name not in feedback or self.now() - feedback[name][0] > self.settings.feedback_timeout:
                raise MotionTestError('feedback', f'{name} 反馈缺失或超时')
        status = feedback['status'][1]
        if not feedback['health'][1].available:
            raise MotionTestError('feedback', 'localization available=false；原始定位反馈不健康')
        if not status.is_armed or not status.navigation_ready or status.error_flags:
            raise MotionTestError('hardware', f'MCU 未满足测试条件：armed={status.is_armed}, navigation_ready={status.navigation_ready}, error_flags={status.error_flags}')
        voltage = status.battery_voltage
        if voltage == 0.0:
            voltage = self.settings.external_battery_voltage
            if voltage == 0.0:
                raise MotionTestError('hardware', 'MCU 电压未上报；本轮必须填写现场实测 external_battery_voltage，不能自动监控电压')
        if not math.isfinite(voltage) or voltage < self.settings.min_battery_voltage:
            raise MotionTestError('hardware', '电压未达到配置的测试下限')
        message = feedback['pose'][1]
        pose = [message.robot_x, message.robot_y, message.robot_z, message.robot_yaw]
        if not all(math.isfinite(value) for value in (*pose, message.robot_roll, message.robot_pitch)):
            raise MotionTestError('feedback', '位姿包含非有限数值')
        if max(abs(message.robot_roll), abs(message.robot_pitch)) > 20.0:
            raise MotionTestError('envelope', '横滚/俯仰超过 20°')
        twist = feedback['twist'][1].twist.twist
        speed = math.sqrt(twist.linear.x ** 2 + twist.linear.y ** 2 + twist.linear.z ** 2)
        angular = (twist.angular.x, twist.angular.y, twist.angular.z)
        if not all(math.isfinite(value) for value in angular):
            raise MotionTestError('feedback', '角速度包含非有限数值')
        if not math.isfinite(speed) or speed > self.settings.max_speed_mps:
            raise MotionTestError('envelope', f'实测线速度超出测试范围：{speed:.3f}m/s')
        if self.reference is not None:
            ref = self.reference
            if (math.hypot(pose[0] - ref[0], pose[1] - ref[1]) > self.settings.horizontal_limit_m
                    or abs(pose[2] - ref[2]) > self.settings.vertical_limit_m
                    or abs(wrap_yaw(pose[3] - ref[3])) > 25.0):
                raise MotionTestError('envelope', '位姿超出本轮测试的水平/深度/航向范围')

    def ready(self):
        deadline = self.now() + 5.0
        while True:
            try:
                self.check()
                if self.snapshot()['status'][1].battery_voltage == 0.0:
                    self.log(f'电压遥测缺失；使用本轮外部实测 {self.settings.external_battery_voltage:.2f}V，仅用于开始放行，现场须独立监控带载电压')
                return
            except MotionTestError:
                if self.node.stopped or self.now() >= deadline:
                    raise
                time.sleep(0.05)

    def pause(self, seconds):
        deadline = self.now() + seconds
        while self.now() < deadline:
            self.check()
            time.sleep(min(0.02, max(0.0, deadline - self.now())))
        self.check()

    def wait_reset(self, sent_at):
        deadline = self.now() + 3.0
        while self.now() < deadline:
            self.check()
            pose = self.pose()
            if (self.snapshot()['pose'][0] > sent_at + 0.15
                    and math.sqrt(sum(value * value for value in pose[:3])) < 0.05
                    and abs(wrap_yaw(pose[3])) < 2.0):
                return
            self.pause(0.05)
        raise MotionTestError('feedback', 'START 后未确认 odom 原点重置')

    def verify_pose(self, target):
        self.check()
        pose = self.pose()
        error = math.sqrt(sum((pose[index] - target[index]) ** 2 for index in range(3)))
        yaw_error = abs(wrap_yaw(pose[3] - target[3]))
        self.log(f'实测={pose}, 位置误差={error:.3f}m, 航向误差={yaw_error:.1f}°')
        if error > 0.15 or yaw_error > 6.0:
            raise MotionTestError('accuracy', 'Action 成功但稳定后的实测误差超过 0.15m/6°')

    def verify_watchdog(self, sent_at):
        item = self.snapshot().get('setpoint')
        if item is None:
            raise MotionTestError('watchdog', '未收到 BasicMotion setpoint 输出')
        received, message = item
        if (received <= sent_at + 0.2 or message.control_key != 0x11
                or any(abs(value) > 1e-6 for value in (message.x, message.y, message.z, message.yaw))):
            raise MotionTestError('watchdog', '速度租约结束后未观测到零速度 setpoint')

    @staticmethod
    def cancel_late_goal(future):
        handle = future.result()
        if handle is not None and handle.accepted:
            handle.cancel_goal_async()

    def send(self, command, target, timeout, guarded=True):
        if not guarded and (command != 7 or any(target)):
            raise MotionTestError('config', '仅零速度允许绕过反馈检查')
        if guarded:
            self.check()
        client = self.node._action_client
        if not client.wait_for_server(timeout_sec=1.0):
            raise MotionTestError('motion', 'BasicMotion action server 不可用')
        if guarded:
            self.check()
        goal = self.action_type.Goal()
        goal.cmd_type, goal.axes = command, 'xyzrz'
        goal.target, goal.timeout = list(target), float(timeout)
        goal.velocity_lease = 0.25
        goal.task_context = f'basic_motion_test/{self.settings.stage}'
        send_future = client.send_goal_async(goal)
        handle = None
        try:
            self.wait_future(send_future, 3.0, guarded)
            handle = send_future.result()
            if handle is None or not handle.accepted:
                raise MotionTestError('motion', 'BasicMotion 拒绝目标')
            self.node._active_goal_handle = handle
            if guarded:
                self.check()
            result_future = handle.get_result_async()
            self.wait_future(result_future, timeout + 2.0, guarded)
            result = result_future.result()
            if result is None or result.status != 4 or not result.result.success:
                raise MotionTestError('motion', '动作失败：' + (result.result.message if result is not None else '无结果'))
        except Exception:
            if handle is not None and handle.accepted:
                handle.cancel_goal_async()
            elif not send_future.done():
                send_future.add_done_callback(self.cancel_late_goal)
            elif send_future.result() is not None and send_future.result().accepted:
                send_future.result().cancel_goal_async()
            raise
        finally:
            if self.node._active_goal_handle is handle:
                self.node._active_goal_handle = None

    def wait_future(self, future, timeout, guarded):
        deadline = self.now() + timeout
        while not future.done():
            if guarded:
                self.check()
            if self.now() >= deadline:
                raise MotionTestError('timeout', '等待 Action 应答/结果超时')
            time.sleep(0.01)

    def neutral(self):
        self.send(7, [0.0] * 4, 2.0, guarded=False)
        self.log('测试结束：已确认零速度 Action；MCU 仍可能处于解锁状态')

    def run(self, params):
        # Manual stages keep the voltage/range settings loaded by the first
        # YAML run. An empty request always holds, and does not reset odom.
        merged = {field.name: getattr(self.settings, field.name) for field in fields(Settings)}
        merged.update(stage='hold', reset_origin=False, test_depth=False,
                      external_battery_voltage=0.0)
        if not isinstance(params, dict):
            return TaskOutcome.failed('basic_motion_test.config', '测试参数必须为映射')
        merged.update(params)
        return run_test(self, merged)
