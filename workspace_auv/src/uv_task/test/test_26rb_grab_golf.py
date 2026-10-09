"""抓高尔夫球流程的偏置、回位复检和重试测试。"""

from importlib import import_module
from types import SimpleNamespace
from uv_task.task_outcome import TaskOutcome


_grab = import_module('uv_task.26rb_grab_golf')
GrabGolfTask = _grab.RB26GrabGolfTask


class _Logger:
    def info(self, _message):
        pass

    def warning(self, _message):
        pass

    def error(self, _message):
        pass


def test_execute_retries_when_golf_remains_after_return(monkeypatch):
    events = []
    verification_results = iter((False, True))
    node = SimpleNamespace(
        stopped=False, LIGHT_YELLOW=3, get_logger=lambda: _Logger(),
        _set_task_phase_light=lambda *args: None,
        _model_mapping=SimpleNamespace(model_class_id=lambda name, **kwargs:
            1 if name == 'collection_frame_down' else 7))
    task = GrabGolfTask(node, {'max_grab_retries': 1})
    monkeypatch.setattr(_grab, 'CollectionFrameSearch', lambda *args:
        SimpleNamespace(execute=lambda: TaskOutcome.ok()))
    task._wait_for_detection = lambda *args: SimpleNamespace()
    task._flash_green = lambda *args: True
    task._prepare_claw = lambda: True

    def servo(class_id, label):
        events.append('collection_servo' if class_id == 1 else 'servo')
        return [1.0, 2.0, -0.3, 15.0]

    def offset():
        events.append('offset')
        return [1.0, 2.0, -0.3, 15.0]

    def settle():
        events.append('settle')
        return True

    def descend():
        events.append('descend')
        return True

    def ascend():
        events.append('ascend')
        return True

    def return_pose(_pose):
        events.append('return')
        return True

    def verify():
        events.append('verify')
        return next(verification_results)

    task._servo_horizontally = servo
    task._apply_camera_gripper_offset = offset
    task._wait_pre_descent_settle = settle
    task._descend = descend
    task._ascend = ascend
    task._return_to_recorded_pose = return_pose
    task._verify_golf_removed = verify

    assert task.execute()
    assert events == [
        'collection_servo',
        'servo', 'offset', 'settle', 'descend', 'ascend', 'return', 'verify',
        'servo', 'offset', 'settle', 'descend', 'ascend', 'return', 'verify',
    ]
