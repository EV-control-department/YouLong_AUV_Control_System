"""抓球流程的偏置、回位复检和重试测试。"""

from importlib import import_module
import threading
import time
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
