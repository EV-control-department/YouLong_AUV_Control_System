"""抓高尔夫球流程的偏置、回位复检和重试测试。"""

import pytest
import numpy as np

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
    velocities = []
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
        _latest_robot_pose=lambda: [0., 0., .2, 0., 0., 0.],
        _send_body_velocity=lambda *args, **kwargs: velocities.append(args) or (True, ''),
        _ensure_camera_extrinsics=lambda: True,
        _set_task_phase_light=lambda *args: None,
        _format_motion_context=lambda text: text,
        _send_action_goal=lambda *args, **kwargs: commands.append(args) or (True, ''),
        _model_mapping=SimpleNamespace(model_class_id=lambda name, **kwargs:
            1 if name == 'collection_frame_down' else 7))
    task = GrabGolfTask(node, {'horizontal_hold_seconds': 0.0})
    task._horizontal_velocity = lambda *args: ([0., 0., .2, 0., 0., 0.], np.array([.05, 0.]), .1, 0.)
    assert task._servo_horizontally(1, 'collection_frame') is not None
    assert len(velocities) == 2 and velocities[-1] == ()
    assert len(commands) == 1 and commands[0][2] == 'xyzrz'  # Only final position hold.
    assert task._aligned_camera is None
    assert task._pending_golf_priority is priorities[1]
    task._horizontal_velocity = lambda *args: ([0., 0., .2, 0., 0., 0.], np.zeros(2), 0., 0.)
    assert task._servo_horizontally(7, 'pink_golf') is not None
    assert len(priorities) == 2  # Reuse the fresh ball lease rather than discard its detection.
    assert task._pending_golf_priority is None
    assert task._aligned_camera == camera


@pytest.fixture
def velocity_rig(monkeypatch):
    clock = [100.0]
    pose = np.array([0., 0., .2, 17., -12., 37.])
    velocity = np.zeros(3)
    sends, holds = [], []
    detection = SimpleNamespace(pixel_x=30., pixel_y=40., confidence=.9)
    node = SimpleNamespace(
        stopped=False, LIGHT_YELLOW=3, get_logger=lambda: _Logger(),
        _cmd_x=5., _cmd_y=6., _cmd_z=.8, _cmd_yaw=90.,
        _latest_robot_pose=lambda: tuple(pose),
        _ensure_camera_extrinsics=lambda: True,
        _set_task_phase_light=lambda *args: None,
        _model_mapping=SimpleNamespace(model_class_id=lambda name, **kwargs:
            1 if name == 'collection_frame_down' else 7))
    state = SimpleNamespace(detected=True, failure=None, hook=lambda: None)
    class Priority:
        generation = 1
        def __init__(self, *args, **kwargs):
            pass
        def update(self):
            return ('down_left', detection) if state.detected else (None, None)
    monkeypatch.setattr(_grab, 'DownCameraPriority', Priority)
    monkeypatch.setattr(_grab.time, 'monotonic', lambda: clock[0])
    def sleep(dt):
        pose[:3] += _grab.body_to_world_rotation(pose) @ velocity*dt
        clock[0] += dt
        state.hook()
    monkeypatch.setattr(_grab.time, 'sleep', sleep)
    def send(*args, **kwargs):
        velocity[:] = args if args else [0., 0., 0.]
        world = _grab.body_to_world_rotation(pose) @ velocity
        sends.append((clock[0], world.copy(), kwargs))
        if args and state.failure is not None:
            if isinstance(state.failure, Exception):
                raise state.failure
            return False, state.failure
        return True, ''
    node._send_body_velocity = send
    def hold(command, target, axes, **kwargs):
        holds.append((command, list(target), axes))
        pose[:3], pose[5] = target[:3], target[3]
        return True, ''
    node._send_action_goal = hold
    task = GrabGolfTask(node, {'horizontal_hold_seconds': .3})
    return SimpleNamespace(task=task, node=node, pose=pose, clock=clock,
                           sends=sends, holds=holds, state=state)


def test_xy_velocity_converges_while_rejecting_depth_disturbance(velocity_rig):
    rig = velocity_rig
    target = np.array([.12, -.08])
    disturbed = []
    def disturb():
        if not disturbed and rig.clock[0] >= 100.3:
            rig.pose[2] += .06
            disturbed.append(True)
    rig.state.hook = disturb
    def correction(*args):
        error = target-rig.pose[:2]
        velocity = error*.8
        norm = np.linalg.norm(velocity)
        if norm > .08:
            velocity *= .08/norm
        return tuple(rig.pose), velocity, error[0], error[1]
    rig.task._horizontal_velocity = correction
    recorded = rig.task._servo_horizontally(7, 'pink_golf')
    assert recorded is not None
    assert np.max(np.abs(target-np.array(recorded[:2]))) <= .02
    assert recorded[2] == .2  # Lock measured depth, not the stale command depth (.8).
    assert rig.task._servo_depth == .2
    assert any(world[2] < -.001 for _, world, _ in rig.sends)
    assert all(np.linalg.norm(world[:2]) <= .08+1e-9 for _, world, _ in rig.sends)
    assert np.allclose(rig.sends[-1][1], 0.)
    assert len(rig.holds) == 1 and rig.holds[0][2] == 'xyzrz'
    assert recorded[:2] != [5., 6.]  # Record actual motion instead of an old XY command.


@pytest.mark.parametrize('camera', ['down_left', 'down_right'])
def test_tilted_xy_servo_has_no_world_vertical_motion_at_locked_depth(velocity_rig, monkeypatch, camera):
    rig = velocity_rig
    rig.task._servo_depth = rig.pose[2]
    rig.node.camera_extrinsics = {camera: SimpleNamespace(optical_to_body=np.eye(3))}
    monkeypatch.setattr(_grab, 'normalized_image_error', lambda *args: (.4, -.3))
    _, horizontal, _, _ = rig.task._horizontal_velocity(camera, SimpleNamespace())
    rig.task._send_horizontal_velocity(horizontal, rig.clock[0]+5.)
    world = rig.sends[-1][1]
    assert np.linalg.norm(world[:2]) > 0.
    assert world[2] == pytest.approx(0., abs=1e-12)
    assert np.linalg.norm(world[:2]) <= .08+1e-9


def test_centered_ball_waits_for_depth_recovery(velocity_rig):
    rig = velocity_rig
    rig.task._servo_depth = .2
    rig.pose[2] = .28
    rig.task._horizontal_velocity = lambda *args: (tuple(rig.pose), np.zeros(2), 0., 0.)
    depths = []
    rig.state.hook = lambda: depths.append(rig.pose[2])
    assert rig.task._servo_horizontally(7, 'pink_golf') is not None
    assert len(depths) > 3
    assert abs(depths[-1]-.2) <= rig.task._depth_hold_tolerance
    assert all(np.allclose(world[:2], 0.) for _, world, _ in rig.sends)


def test_lost_detection_stops_xy_and_keeps_depth_feedback(velocity_rig):
    rig = velocity_rig
    rig.task._servo_timeout = 1.
    rig.task._servo_depth = .2
    rig.pose[2] = .26
    rig.task._horizontal_velocity = lambda *args: (tuple(rig.pose), np.array([.05, 0.]), .1, 0.)
    rig.state.hook = lambda: setattr(rig.state, 'detected', False)
    assert rig.task._servo_horizontally(7, 'pink_golf') is None
    assert rig.sends[0][1][0] > 0.
    assert all(np.allclose(world[:2], 0.) for _, world, _ in rig.sends[1:])
    assert any(world[2] < 0. for _, world, _ in rig.sends[1:-1])
    assert np.allclose(rig.sends[-1][1], 0.)
    assert rig.holds[-1][1][2] == .2


@pytest.mark.parametrize('failure', ['发送动作目标超时', RuntimeError('模拟动作异常'), 'cancel'])
def test_velocity_failure_or_cancel_always_stops_lease(velocity_rig, failure):
    rig = velocity_rig
    rig.task._horizontal_velocity = lambda *args: (tuple(rig.pose), np.array([.05, 0.]), .1, 0.)
    if failure == 'cancel':
        rig.state.hook = lambda: setattr(rig.node, 'stopped', True)
    else:
        rig.state.failure = failure
    assert rig.task._servo_horizontally(7, 'pink_golf') is None
    assert np.allclose(rig.sends[-1][1], 0.)
    assert rig.task._aligned_camera is None
    if failure == 'cancel':
        assert rig.holds == []


def test_yaml_exposes_velocity_and_depth_parameters():
    from pathlib import Path
    from uv_task.config_loader import load_task
    import yaml
    for name in ('26rb_grab_golf.yaml', '26rb_grab_ball.yaml'):
        path = Path(__file__).parents[1]/'config/tasks'/name
        params = load_task(path)[0]['params']
        configured = yaml.safe_load(path.read_text())['params']['servo']
        assert params['horizontal_max_speed_mps'] == configured['max_speed_mps']
        assert params['depth_hold_gain'] == configured['depth_gain']
        assert params['depth_hold_max_speed_mps'] == configured['max_vertical_speed_mps']
        assert params['depth_hold_tolerance_m'] == configured['depth_tolerance_m']
        assert 'max_horizontal_step_m' not in params
