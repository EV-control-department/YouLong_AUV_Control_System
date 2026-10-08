"""抓球流程的偏置、回位复检和重试测试。"""

from importlib import import_module
import threading
import time
import math
import ast
from pathlib import Path
import pytest
from types import SimpleNamespace


_grab = import_module('uv_task.26rb_grab_ball')
GrabBallTask = _grab.RB26GrabBallTask
GrabSeaCucumberTask = import_module('uv_task.grab_sea_cucumber').GrabSeaCucumberTask


class _Logger:
    def info(self, _message):
        pass

    def warning(self, _message):
        pass

    def error(self, _message):
        pass


def test_failed_start_prevents_grab_and_movement():
    source = Path(__file__).parents[1] / 'uv_task' / 'task_runner.py'
    cls = next(n for n in ast.parse(source.read_text()).body
               if isinstance(n, ast.ClassDef) and n.name == 'TaskRunnerNode')
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'run_task_list')
    scope = {}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), 'exec'), scope)
    calls = []
    logger = SimpleNamespace(info=lambda msg: None, warn=lambda msg: None, error=lambda msg: None)
    node = SimpleNamespace(tasks=[{'name': 'start'}, {'name': 'grab_sea_cucumber'}],
                           _cmd_x=0., _cmd_y=0., _cmd_z=0., _cmd_yaw=0.,
                           get_logger=lambda: logger,
                           _execute_task=lambda name, params: calls.append(name) or False)
    scope['run_task_list'](node)
    assert calls == ['start']
    assert node.stopped and not node.running


def test_negative_sea_descent_rejected_before_initialization():
    params = dict(sea_cucumber_class_id=2, image_width=640, image_height=480,
                  gripper_offset_x_m=.3, gripper_offset_y_m=0.,
                  descent_speed_mps=-.5, descent_duration_seconds=4.,
                  max_press_distance_m=1.3, drop_pose=[0,0,.3,0], search_pose=[1,1,.3,0])
    with pytest.raises(ValueError, match='有限正数'):
        GrabSeaCucumberTask(None, params)


def test_sea_degree_config_preserves_internal_radians_without_old_descent_cap():
    from uv_task.config_loader import load_task
    params = load_task(Path(__file__).parents[1] / 'config/tasks/grab_sea_cucumber.yaml')[0]['params']
    params.update(descent_speed_mps=.5, descent_duration_seconds=3.,
                  max_press_distance_m=1.3, pickup_servo_angle_deg=0., release_servo_angle_deg=90.)
    node = SimpleNamespace(get_logger=lambda: _Logger())
    task = GrabSeaCucumberTask(node, params)
    assert task._pickup_angle == 0.
    assert task._release_angle == pytest.approx(math.pi/2)
    assert task._descent_speed == .5 and task._descent_duration == 3.
    assert task._ascent_speed == .01


def test_sea_descent_exception_still_sends_zero_velocity():
    task = _sea_flow_fake()
    del task._descend
    task._descent_speed, task._descent_duration, task._descent_period = .5, 3., .05
    packets = []
    def publish(vertical_mps=0.):
        packets.append(vertical_mps)
        if vertical_mps:
            raise RuntimeError('publish failed')
    task._node._publish_body_velocity = publish
    with pytest.raises(RuntimeError, match='publish failed'):
        task._descend()
    assert packets == [.5,0.]


def test_sea_rejects_stale_pose_before_direct_velocity_control():
    task = _sea_flow_fake()
    del task._measured_pose  # 检查真实方法，不使用流程测试替身。
    task._node._perception_lock = threading.Lock()
    task._node._robot_pose = (0,0,.8,0,0,0)
    task._node._robot_pose_received = time.monotonic() - 2.
    assert task._measured_pose() is None


@pytest.mark.parametrize('robot_yaw', [0., 90., -45.])
@pytest.mark.parametrize('pixel', [(420.,240.), (320.,340.), (420.,340.)])
def test_sea_camera_mount_180_reverses_body_and_world_correction(robot_yaw, pixel):
    task = GrabSeaCucumberTask.__new__(GrabSeaCucumberTask)
    task._node = SimpleNamespace(_latest_robot_pose=lambda: (1,2,.8,0,0,robot_yaw))
    task._CX, task._CY, task._FX, task._FY = 320.,240.,500.,500.
    task._projection_depth, task._servo_gain, task._max_xy_step = .8,.8,.08
    detection = SimpleNamespace(pixel_x=pixel[0], pixel_y=pixel[1])
    task._camera_mount_yaw = 0.
    nominal = task._horizontal_step(detection)
    task._camera_mount_yaw = 180.
    corrected = task._horizontal_step(detection)
    assert corrected[0] == nominal[0]
    assert corrected[1:5] == pytest.approx(tuple(-v for v in nominal[1:5]))
    assert corrected[5:] == nominal[5:]
    assert math.hypot(corrected[1], corrected[2]) <= .08 + 1e-9


def test_sea_cucumber_retries_and_delivers_until_five_removed():
    task = GrabSeaCucumberTask.__new__(GrabSeaCucumberTask)
    calls = []
    task._node = SimpleNamespace(
        stopped=False, set_servo=lambda angle, label: calls.append(('servo', angle)),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1)))
    task._logger = _Logger()
    task._total_timeout = 10
    task._search_pose = (3, 1, .8, 0)
    task._search_travel_timeout = 2
    task._target_count = 5
    task._max_failed_attempts = 3
    task._pickup_angle = 0
    task._servo_timeout = task._return_timeout = 1
    task._descent_duration = .1
    task.confirmed_removed = task.delivery_commands = 0
    task._measured_pose = lambda: (0,0,.8,0,0,0)
    task._travel = lambda pose, *args: calls.append(('travel', pose)) or True
    task._servo_horizontally = lambda: (.1,.2,.8,0)
    counts = iter((5,5,5,3,3,0))
    task._count_visible = lambda *args: next(counts)
    task._apply_gripper_offset = task._wait_pre_descent_settle = task._descend = lambda: True
    task._return_to_recorded_pose = lambda pose: True
    task._deliver = lambda pose, deadline, return_to_search: calls.append(('deliver', return_to_search)) or True
    assert task.execute()
    assert calls[0] == ('travel', task._search_pose)
    assert task.confirmed_removed == 5
    assert [c for c in calls if c[0] == 'deliver'] == [('deliver', True), ('deliver', False)]
    assert len([c for c in calls if c[0] == 'servo']) == 3


def test_sea_cucumber_scan_rejects_old_capture_and_accepts_fresh_frame():
    task = GrabSeaCucumberTask.__new__(GrabSeaCucumberTask)
    task._class_id, task._min_confidence, task._detection_timeout = 2, .35, 1
    now = time.monotonic()
    task._scan_received_after, task._scan_capture_after_ns = now-.2, 100
    stamp = SimpleNamespace(sec=0, nanosec=99)
    detection = SimpleNamespace(class_id=2, confidence=.9, mask_x=[1,2,3], mask_y=[4,5,6])
    task._node = SimpleNamespace(_perception_lock=threading.RLock(), _down_detections={
        'down_left': (now, SimpleNamespace(header=SimpleNamespace(stamp=stamp), detections=[detection]))})
    assert task._best_left_detection() is None
    stamp.nanosec = 101
    assert task._best_left_detection().pixel_x == 2


def _sea_flow_fake():
    task = GrabSeaCucumberTask.__new__(GrabSeaCucumberTask)
    task._node = SimpleNamespace(stopped=False, set_servo=lambda *a: None,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1)))
    task._logger = _Logger()
    task._total_timeout = 10
    task._search_pose = (0,0,.8,0)
    task._search_travel_timeout = task._servo_timeout = task._return_timeout = 1
    task._target_count, task._max_failed_attempts = 5, 3
    task._pickup_angle, task._descent_duration = 0, .1
    task.confirmed_removed = task.delivery_commands = 0
    task._measured_pose = lambda: (0,0,.8,0,0,0)
    task._travel = lambda *a: True
    task._servo_horizontally = lambda: (0,0,.8,0)
    task._apply_gripper_offset = task._wait_pre_descent_settle = task._descend = lambda: True
    task._return_to_recorded_pose = lambda p: True
    task._retry_perception = lambda reason, failures, deadline: failures <= 3
    return task


@pytest.mark.parametrize('failure', ['alignment', 'count_missing', 'count_zero'])
def test_sea_visual_failure_retries_instead_of_stopping(failure):
    task = _sea_flow_fake()
    deliveries = []
    if failure == 'alignment':
        positions = iter((None, (0,0,.8,0)))
        task._servo_horizontally = lambda: next(positions)
        counts = iter((5,0))
    else:
        counts = iter((None if failure == 'count_missing' else 0, 5, 0))
    task._count_visible = lambda *a: next(counts)
    task._deliver = lambda *a, **kw: deliveries.append(True) or True
    assert task.execute()
    assert task.confirmed_removed == 5 and len(deliveries) == 1


def test_sea_unknown_postgrab_count_delivers_without_claiming_success():
    task = _sea_flow_fake()
    counts = iter((5,None,5,0))
    deliveries = []
    task._count_visible = lambda *a: next(counts)
    task._deliver = lambda *a, **kw: deliveries.append(
        (task.confirmed_removed, kw['return_to_search'])) or True
    assert task.execute()
    assert deliveries == [(0,True), (5,False)]


def test_sea_partial_count_accepts_two_but_not_one_fresh_frames(monkeypatch):
    module = import_module('uv_task.grab_sea_cucumber')
    for number in (1,2):
        clock = SimpleNamespace(value=0.)
        task = _sea_flow_fake()
        task._count_frames, task._count_timeout, task._detection_timeout = 3, .25, 1
        task._node._perception_lock = threading.Lock()
        task._node._down_detections = {'down_left': (0., SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace(sec=0,nanosec=1))))}
        task._segmented_detections = lambda msg: [object()] * 5
        def sleep(dt):
            clock.value += dt
            if number == 2 and clock.value <= .1:
                task._node._down_detections['down_left'] = (.1, SimpleNamespace(
                    header=SimpleNamespace(stamp=SimpleNamespace(sec=0,nanosec=2))))
        monkeypatch.setattr(module, 'time', SimpleNamespace(monotonic=lambda: clock.value, sleep=sleep))
        result = task._count_visible(-1., 0, 10., 'before')
        assert result == (5 if number == 2 else None)


def test_task_runner_servo_protocol_and_start_reset_without_ros_node():
    # 仅提取方法执行：无需创建节点，测试不会向DDS发布或触发硬件。
    import ast
    from zit6_interfaces.msg import ZitServo
    file = Path(__file__).parents[1] / 'uv_task' / 'task_runner.py'
    tree = ast.parse(file.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'TaskRunnerNode')
    methods = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in ('set_servo', '_do_start')]
    namespace = {'math': math, 'ZitServo': ZitServo, 'BasicMotion': _grab.BasicMotion}
    exec(compile(ast.Module(body=methods, type_ignores=[]), str(file), 'exec'), namespace)
    messages, events = [], []
    fake = SimpleNamespace(pub_servo=SimpleNamespace(publish=messages.append),
                           get_logger=lambda: _Logger(), ANGLE_INIT=0,
                           light_off=lambda: events.append('light'),
                           _send_action_goal=lambda *args, **kw: (events.append('start') or True, 'ok'))
    fake.set_servo = lambda angle, label: namespace['set_servo'](fake, angle, label)
    fake.set_servo(math.pi/2, '放')
    assert messages[-1].servo_id == 1
    assert messages[-1].angle == pytest.approx(90.)
    fake.set_servo(math.pi, '180度')
    assert messages[-1].angle == pytest.approx(180.)
    assert namespace['_do_start'](fake)
    assert messages[-1].angle == 0
    assert messages[-1].servo_id == 1
    assert events == ['light', 'start']
    with pytest.raises(ValueError):
        fake.set_servo(90, '错误度数')


def test_execute_retries_when_ball_remains_after_return():
    events = []
    verification_results = iter((False, True))
    task = GrabBallTask.__new__(GrabBallTask)
    task._logger = _Logger()
    task._node = SimpleNamespace(stopped=False)
    task._max_grab_retries = 1
    task._color = 'red'
    task._class_id = 7
    task._descent_speed = 0.4
    task._descent_duration = 10.0

    def servo():
        events.append('servo')
        return [1.0, 2.0, -0.3, 15.0]

    def offset():
        events.append('offset')
        return True

    def settle():
        events.append('settle')
        return True

    def descend():
        events.append('descend')
        return True

    def return_pose(_pose):
        events.append('return')
        return True

    def verify():
        events.append('verify')
        return next(verification_results)

    task._servo_horizontally = servo
    task._apply_gripper_offset = offset
    task._wait_pre_descent_settle = settle
    task._descend = descend
    task._return_to_recorded_pose = return_pose
    task._verify_ball_removed = verify

    assert task.execute()
    assert events == [
        'servo', 'offset', 'settle', 'descend', 'return', 'verify',
        'servo', 'offset', 'settle', 'descend', 'return', 'verify',
    ]


def test_sea_cucumber_ascent_uses_direct_slow_velocity():
    commands = []
    node = SimpleNamespace(
        stopped=False,
        _perception_lock=threading.RLock(),
        _robot_pose=(1.0, 2.0, 0.50, 0.0, 0.0, 0.0),
        _format_motion_context=lambda label: label,
        _cmd_x=1.0, _cmd_y=2.0, _cmd_z=0.50, _cmd_yaw=0.0,
    )

    def send_goal(command, target, axes, **_kwargs):
        commands.append((command, list(target), axes))
        if command == _grab.BasicMotion.Goal.BMOVE:
            pose = list(node._robot_pose)
            pose[2] += target[2]
            node._robot_pose = tuple(pose)
        return True, 'ok'

    node._send_action_goal = send_goal
    velocities = []
    def publish(vertical_mps=0.):
        velocities.append(vertical_mps)
        if vertical_mps < 0:
            pose = list(node._robot_pose)
            pose[2] -= .03  # 模拟艇体到达下一次测量深度。
            node._robot_pose = tuple(pose)
    node._publish_body_velocity = publish
    task = GrabSeaCucumberTask.__new__(GrabSeaCucumberTask)
    task._node = node
    task._logger = _Logger()
    task._return_timeout = 5.0
    task._ascent_step = 0.03
    task._ascent_step_timeout = 0.5
    task._ascent_pause = 0.0
    task._ascent_tolerance = 0.015
    task._ascent_speed, task._ascent_period = .01, .001

    assert task._return_to_recorded_pose((1.1, 2.1, 0.40, 5.0))
    assert len(velocities) >= 4
    assert all(-.01 <= v <= 0 for v in velocities)
    assert velocities[-1] == 0.
    assert len(commands) == 1  # 上浮不再发送任何BMOVE。
    assert commands[-1][0] == _grab.BasicMotion.Goal.SET
    assert commands[-1][2] == 'xyrz'
    assert commands[-1][1][2] == node._robot_pose[2]


def test_sea_cucumber_ascent_stops_if_measured_depth_does_not_change():
    commands = []
    node = SimpleNamespace(
        stopped=False,
        _perception_lock=threading.RLock(),
        _robot_pose=(1.0, 2.0, 0.50, 0.0, 0.0, 0.0),
        _format_motion_context=lambda label: label,
        _send_action_goal=lambda command, target, axes, **_kwargs:
            (commands.append((command, target, axes)) or True, 'ok'),
    )
    task = GrabSeaCucumberTask.__new__(GrabSeaCucumberTask)
    task._node = node
    task._logger = _Logger()
    task._return_timeout = 0.2
    task._ascent_step = 0.03
    task._ascent_step_timeout = 0.05
    task._ascent_pause = 0.0
    task._ascent_tolerance = 0.015
    task._ascent_speed, task._ascent_period = .01, .02
    velocities = []
    node._publish_body_velocity = lambda vertical_mps=0.: velocities.append(vertical_mps)

    assert not task._return_to_recorded_pose((1.0, 2.0, 0.40, 0.0))
    assert commands == []
    assert velocities and velocities[-1] == 0.
    assert all(-.01 <= v <= 0 for v in velocities)


def test_sea_cucumber_delivery_ascent_precedes_horizontal_travel():
    commands = []
    node = SimpleNamespace(
        stopped=False,
        _perception_lock=threading.RLock(),
        _robot_pose=(1.0, 2.0, 0.50, 0.0, 0.0, 0.0),
        _format_motion_context=lambda label: label,
        _cmd_x=1.0, _cmd_y=2.0, _cmd_z=0.50, _cmd_yaw=0.0,
    )

    def send_goal(command, target, axes, **_kwargs):
        commands.append((command, list(target), axes))
        if command == _grab.BasicMotion.Goal.BMOVE:
            pose = list(node._robot_pose)
            pose[2] += target[2]
            node._robot_pose = tuple(pose)
        return True, 'ok'

    node._send_action_goal = send_goal
    velocities = []
    def publish(vertical_mps=0.):
        velocities.append(vertical_mps)
        if vertical_mps < 0:
            pose = list(node._robot_pose)
            pose[2] -= .03
            node._robot_pose = tuple(pose)
    node._publish_body_velocity = publish
    task = GrabSeaCucumberTask.__new__(GrabSeaCucumberTask)
    task._node = node
    task._logger = _Logger()
    task._ascent_step = 0.03
    task._ascent_step_timeout = 0.5
    task._ascent_pause = 0.0
    task._ascent_tolerance = 0.015
    task._ascent_speed, task._ascent_period = .01, .001

    assert task._travel((3.0, 4.0, 0.40, 10.0), '投放',
                        time.monotonic() + 5.0, 5.0)
    assert len(commands) == 1
    assert velocities and velocities[-1] == 0.
    assert commands[-1][0] == _grab.BasicMotion.Goal.WTRAVEL
    assert commands[-1][1][2] >= node._robot_pose[2]
