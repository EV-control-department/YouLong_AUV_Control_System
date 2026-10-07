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


def test_unsafe_sea_descent_reports_distance_before_initialization():
    params = dict(sea_cucumber_class_id=2, image_width=640, image_height=480,
                  gripper_offset_x_m=.3, gripper_offset_y_m=0.,
                  descent_speed_mps=.5, descent_duration_seconds=4.,
                  max_press_distance_m=1.3, drop_pose=[0,0,.3,0], search_pose=[1,1,.3,0])
    with pytest.raises(ValueError, match='预计行程=2m'):
        GrabSeaCucumberTask(None, params)


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
    assert messages[-1].angle == pytest.approx(math.pi/2)
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


def test_sea_cucumber_ascent_uses_measured_small_bmove_steps():
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
    task = GrabSeaCucumberTask.__new__(GrabSeaCucumberTask)
    task._node = node
    task._logger = _Logger()
    task._return_timeout = 5.0
    task._ascent_step = 0.03
    task._ascent_step_timeout = 0.5
    task._ascent_pause = 0.0
    task._ascent_tolerance = 0.015

    assert task._return_to_recorded_pose((1.1, 2.1, 0.40, 5.0))
    climbs = [target[2] for command, target, _ in commands
              if command == _grab.BasicMotion.Goal.BMOVE]
    assert len(climbs) >= 3
    assert all(-0.031 <= dz < 0 for dz in climbs)
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

    assert not task._return_to_recorded_pose((1.0, 2.0, 0.40, 0.0))
    assert len(commands) == 1
    assert commands[0][0] == _grab.BasicMotion.Goal.BMOVE


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
    task = GrabSeaCucumberTask.__new__(GrabSeaCucumberTask)
    task._node = node
    task._logger = _Logger()
    task._ascent_step = 0.03
    task._ascent_step_timeout = 0.5
    task._ascent_pause = 0.0
    task._ascent_tolerance = 0.015

    assert task._travel((3.0, 4.0, 0.40, 10.0), '投放',
                        time.monotonic() + 5.0, 5.0)
    assert all(command == _grab.BasicMotion.Goal.BMOVE
               for command, _, _ in commands[:-1])
    assert commands[-1][0] == _grab.BasicMotion.Goal.WTRAVEL
    assert commands[-1][1][2] >= node._robot_pose[2]
