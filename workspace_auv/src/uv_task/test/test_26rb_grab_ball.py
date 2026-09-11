"""抓球流程的偏置、回位复检和重试测试。"""

from importlib import import_module
from types import SimpleNamespace


_grab = import_module('uv_task.26rb_grab_ball')
GrabBallTask = _grab.RB26GrabBallTask


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

