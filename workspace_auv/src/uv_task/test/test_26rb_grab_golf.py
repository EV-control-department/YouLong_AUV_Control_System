"""抓高尔夫球流程的偏置、回位复检和重试测试。"""

import pytest

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


@pytest.mark.parametrize('early_ball', [False, True])
def test_execute_retries_when_golf_remains_after_return(monkeypatch, early_ball):
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
    observations = []
    flashes = []
    def observe(class_id, *args):
        observations.append(class_id)
        return SimpleNamespace()
    task._wait_for_detection = observe
    task._flash_green = lambda count, *args: flashes.append(count) or True
    task._prepare_claw = lambda: True

    def servo(class_id, label):
        events.append('collection_servo' if class_id == 1 else 'servo')
        if class_id == 1 and early_ball:
            task._pending_golf_priority = object()
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
    assert observations == ([1] if early_ball else [1, 7])
    assert flashes == ([1, 3, 3] if early_ball else [1, 2, 3, 3])
    assert events == [
        'collection_servo',
        'servo', 'offset', 'settle', 'descend', 'ascend', 'return', 'verify',
        'servo', 'offset', 'settle', 'descend', 'ascend', 'return', 'verify',
    ]


@pytest.mark.parametrize('camera', ['down_left', 'down_right'])
def test_frame_servo_hands_fresh_ball_camera_to_ball_servo(monkeypatch, camera):
    commands = []
    clock = [100.0]
    monkeypatch.setattr(_grab.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(_grab.time, 'sleep', lambda seconds: clock.__setitem__(0, clock[0]+seconds))
    detection = SimpleNamespace(pixel_x=30., pixel_y=40., confidence=.9)
    priorities = []
    class Priority:
        def __init__(self, node, class_id, **kwargs):
            self.class_id = class_id
            self.generation = 1
            self.updates = 0
            priorities.append(self)
        def update(self):
            self.updates += 1
            if self.class_id == 7 and self.updates < 2:
                return None, None
            return camera, detection
    monkeypatch.setattr(_grab, 'DownCameraPriority', Priority)
    node = SimpleNamespace(
        stopped=False, LIGHT_YELLOW=3, get_logger=lambda: _Logger(),
        _cmd_x=0., _cmd_y=0., _cmd_z=.2, _cmd_yaw=0.,
        _ensure_camera_extrinsics=lambda: True,
        _set_task_phase_light=lambda *args: None,
        _format_motion_context=lambda text: text,
        _send_action_goal=lambda *args, **kwargs: commands.append(args) or (True, ''),
        _model_mapping=SimpleNamespace(model_class_id=lambda name, **kwargs:
            1 if name == 'collection_frame_down' else 7))
    task = GrabGolfTask(node, {'horizontal_hold_seconds': 0.0})
    task._horizontal_step = lambda *args: ([0., 0., .2, 0., 0., 0.], .05, 0., .05, 0., .1, 0.)
    assert task._servo_horizontally(1, 'collection_frame') is not None
    assert len(commands) == 1  # Stop frame corrections as soon as the ball appears.
    assert task._aligned_camera is None
    assert task._pending_golf_priority is priorities[1]
    task._horizontal_step = lambda *args: ([0., 0., .2, 0., 0., 0.], 0., 0., 0., 0., 0., 0.)
    assert task._servo_horizontally(7, 'pink_golf') is not None
    assert len(priorities) == 2  # Reuse the fresh ball lease rather than discard its detection.
    assert task._pending_golf_priority is None
    assert task._aligned_camera == camera
