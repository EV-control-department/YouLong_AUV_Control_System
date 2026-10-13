"""Drop servo with virtual odometry, action acknowledgements and servo publisher.

No ROS executor or physical actuator is started.
"""
from importlib import import_module
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest
import yaml

from uv_task.config_loader import ConfigError, _validate_params, load_task
from uv_task.task_outcome import TaskOutcome
from uv_task.task_runner import TaskRunnerNode

mod = import_module('uv_task.26rb_drop_ball_target_rack')


@pytest.fixture
def rig(monkeypatch):
    clock = [100.0]
    pose = np.array([0., 0., .2, 17., -12., 37.])
    body_velocity = np.zeros(3)
    yaw_rate = [0.]
    calls, lights, messages, logs = [], [], [], []
    state = NS(camera='down_left', detected=True, generation=1,
               failure=None, hook=lambda: None)
    optical = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
    camera = {eye: NS(optical_to_body=optical,
                     translation=np.array([-.13, offset, .0645]))
              for eye, offset in [('down_left', -.030586), ('down_right', .030586)]}
    claw = NS(translation=NS(x=-.38, y=0., z=.29))
    hairpin = NS(translation=NS(x=.08, y=0., z=.13))
    def lookup(base, child):
        return {'disc_claw_link': claw, 'hairpin_claw_link': hairpin}[child]
    logger = NS(info=logs.append, warning=logs.append, error=logs.append)
    node = NS(
        stopped=False, LIGHT_YELLOW=3, LIGHT_GREEN=2, LIGHT_OFF=0, _light_state=0,
        _cmd_x=5., _cmd_y=6., _cmd_z=.8, _cmd_yaw=90.,
        _target_rack_down_class_id=7, camera_extrinsics=camera,
        camera_extrinsics_provider=NS(base_frame='base_link', lookup_transform=lookup),
        _ensure_camera_extrinsics=lambda: True, _latest_robot_pose=lambda: tuple(pose),
        _format_motion_context=lambda label: label, get_logger=lambda: logger,
        pub_servo=NS(publish=messages.append))
    def publish_servo(message):
        messages.append(message)
        calls.append(('servo', clock[0], message.servo_id, message.angle))
    node.pub_servo.publish = publish_servo
    node._set_task_phase_light = lambda color, *args, **kwargs: setattr(node, '_light_state', color)
    node.light_off = lambda: setattr(node, '_light_state', 0)
    node.set_servo = lambda angle, label, servo_id=1: TaskRunnerNode.set_servo(
        node, angle, label, servo_id=servo_id)
    node._fallback_failure_outcome = lambda task, phase, *args: TaskOutcome.failed(
        task+'.'+phase, getattr(node, '_last_motion_failure_message', 'failed'))
    detection = NS(pixel_x=50., pixel_y=60., confidence=.9)
    class Priority:
        def __init__(self, *args, **kwargs):
            pass
        @property
        def generation(self):
            return state.generation
        def update(self):
            return (state.camera, detection) if state.detected else (None, None)
    monkeypatch.setattr(mod, 'DownCameraPriority', Priority)
    monkeypatch.setattr(mod.time, 'monotonic', lambda: clock[0])
    def sleep(dt):
        pose[:3] += mod.body_to_world_rotation(pose) @ body_velocity*dt
        pose[5] += yaw_rate[0]*dt
        clock[0] += dt
        state.hook()
    monkeypatch.setattr(mod.time, 'sleep', sleep)
    def velocity(*args, **kwargs):
        body_velocity[:] = args if args else [0., 0., 0.]
        yaw_rate[0] = kwargs.get('yaw_rate_deg_s', 0.)
        calls.append(('velocity', clock[0],
                      (mod.body_to_world_rotation(pose) @ body_velocity).copy(), kwargs))
        if args and state.failure is not None:
            if isinstance(state.failure, Exception):
                raise state.failure
            return False, state.failure
        return True, ''
    node._send_body_velocity = velocity
    def action(command, target, axes, **kwargs):
        calls.append(('action', command, list(target), axes))
        body_velocity[:] = 0.
        yaw_rate[0] = 0.
        pose[:3], pose[5] = target[:3], target[3]
        return True, ''
    node._send_action_goal = action
    task = mod.RB26DropBallTargetRackTask(node, {'depth_task_depth_m': .2, 'down_visual_servo_stable_seconds': .2})
    monkeypatch.setattr(mod, 'normalized_image_error', lambda *args: (.12, -.08))
    return NS(task=task, node=node, pose=pose, state=state, calls=calls,
              clock=clock, messages=messages, logs=logs)


@pytest.mark.parametrize('camera', ['down_left', 'down_right'])
def test_tilted_horizontal_velocity_does_not_descend(rig, camera):
    rig.task._servo_depth = rig.pose[2]
    horizontal, _, _ = rig.task._rack_velocity(camera, NS(), tuple(rig.pose))
    rig.task._send_rack_velocity(horizontal, rig.clock[0]+5.)
    world = rig.calls[-1][2]
    assert np.linalg.norm(world[:2]) > 0.
    assert np.linalg.norm(world[:2]) <= rig.task._max_speed+1e-12
    assert world[2] == pytest.approx(0., abs=1e-12)


def test_closed_loop_centers_rack_and_recovers_depth(rig, monkeypatch):
    target = np.array([.12, -.08])
    depths = []
    disturbed = []
    def hook():
        if not disturbed and rig.clock[0] >= 100.3:
            rig.pose[2] += .06
            disturbed.append(True)
        depths.append(rig.pose[2])
    rig.state.hook = hook
    def image_error(node, camera, detection):
        delta = np.array([*(target-rig.pose[:2]), 0.])
        body = mod.body_to_world_rotation(rig.pose).T @ delta
        optical = node.camera_extrinsics[camera].optical_to_body.T @ body
        return optical[0]/rig.task._projection_depth, optical[1]/rig.task._projection_depth
    monkeypatch.setattr(mod, 'normalized_image_error', image_error)
    assert rig.task._servo_rack()
    assert max(map(abs, image_error(rig.node, rig.state.camera, NS()))) <= rig.task._pixel_tolerance
    assert abs(depths[-1]-.2) <= rig.task._depth_tolerance
    assert rig.task._servo_depth == .2 and rig.node._cmd_z == .2
    velocities = [call[2] for call in rig.calls if call[0] == 'velocity']
    assert any(world[2] < -.001 for world in velocities)
    assert all(np.linalg.norm(world[:2]) <= .08+1e-9 for world in velocities)
    assert rig.calls[-1][0] == 'action'  # SET takes over velocity control.
    holds = [call for call in rig.calls if call[0] == 'action']
    assert len(holds) == 1 and holds[0][3] == 'xyzrz'
    assert holds[0][2][2] == .2  # Configured depth, not stale command Z=.8.


def test_centered_rack_still_waits_for_depth_recovery(rig, monkeypatch):
    rig.task._servo_depth = .2
    rig.pose[2] = .28
    monkeypatch.setattr(mod, 'normalized_image_error', lambda *args: (0., 0.))
    depth_before_hold = []
    rig.state.hook = lambda: depth_before_hold.append(rig.pose[2])
    assert rig.task._servo_rack()
    assert len(depth_before_hold) > 1
    assert abs(depth_before_hold[0]-.2) > rig.task._depth_tolerance
    assert rig.clock[0]-100. >= rig.task._stable_seconds
    assert abs(depth_before_hold[-1]-.2) <= rig.task._depth_tolerance
    assert all(np.allclose(call[2][:2], 0.) for call in rig.calls if call[0] == 'velocity')


def test_lost_rack_stops_xy_but_retains_upward_depth_correction(rig):
    rig.task._servo_depth = .2
    rig.pose[2] = .26
    rig.task._servo_timeout = 1.
    rig.state.hook = lambda: setattr(rig.state, 'detected', False)
    assert rig.task._servo_rack()  # Detection timeout still permits the drop flow.
    velocities = [call[2] for call in rig.calls if call[0] == 'velocity']
    assert np.linalg.norm(velocities[0][:2]) > 0.
    assert all(np.allclose(world[:2], 0.) for world in velocities[1:])
    assert any(world[2] < 0. for world in velocities[1:])
    assert rig.calls[-1][0] == 'action'  # SET takes over velocity control.
    assert rig.task._aligned_camera == rig.state.camera
    assert rig.messages == []


@pytest.mark.parametrize('failure', ['动作确认超时', RuntimeError('模拟动作异常'), 'cancel'])
def test_error_or_cancel_prevents_release(rig, monkeypatch, failure):
    monkeypatch.setattr(mod, 'TargetRackSearch', lambda *args: NS(execute=lambda: TaskOutcome.ok()))
    rig.task._flash_green = lambda *args, **kwargs: True
    if failure == 'cancel':
        rig.state.hook = lambda: setattr(rig.node, 'stopped', True)
    else:
        rig.state.failure = failure
    assert not rig.task.execute()
    velocities = [call for call in rig.calls if call[0] == 'velocity']
    if failure != 'cancel':
        assert rig.calls[-1][0] == 'action'  # Hold with SET after a motion error.
    assert rig.task._aligned_camera is None
    assert rig.messages == []
    if failure == 'cancel':
        assert not any(call[0] == 'action' for call in rig.calls)


@pytest.mark.parametrize('camera', ['down_left', 'down_right'])
def test_disc_claw_alignment_preserves_locked_depth_and_eye_offset(rig, camera):
    rig.task._aligned_camera = camera
    rig.task._servo_depth = .2
    rig.pose[2] = .24
    start = rig.pose.copy()
    assert rig.task._align_disc_claw()
    action = rig.calls[-1]
    assert action[0] == 'action' and action[3] == 'xyzrz'
    assert action[2][2] == .2
    yaw = np.deg2rad(start[5])
    offset = rig.node.camera_extrinsics[camera].translation[:2]-[-.38, 0.]
    expected = start[:2]+np.array([[np.cos(yaw), -np.sin(yaw)],
                                 [np.sin(yaw), np.cos(yaw)]]) @ offset
    assert np.linalg.norm(np.asarray(action[2][:2])-expected) <= .02
    assert len(rig.calls) == 1 and action[1] == mod.SET_COMMAND


def test_release_publishes_180_degrees_without_radian_conversion(rig):
    assert rig.task._release_ball()
    assert len(rig.messages) == 3
    assert all(message.servo_id == 1 and message.angle == 180. for message in rig.messages)
    assert not any('180.00 rad' in line for line in rig.logs)


def test_full_drop_sequence_holds_depth_through_release(rig, monkeypatch):
    monkeypatch.setattr(mod, 'TargetRackSearch', lambda *args: NS(execute=lambda: TaskOutcome.ok()))
    monkeypatch.setattr(mod, 'normalized_image_error', lambda *args: (0., 0.))
    rig.task._flash_green = lambda *args, **kwargs: True
    assert rig.task.execute()
    assert rig.task._servo_depth == .2
    assert rig.pose[2] == .2 and rig.node._cmd_z == .2
    assert [(msg.servo_id, msg.angle) for msg in rig.messages] == [(1, 180.)]*3+[(2, 270.)]*3
    holds = [call for call in rig.calls if call[0] == 'action']
    assert len(holds) == 3
    # Release the ball before moving to the hairpin target, then release the ring.
    events = [call[0] for call in rig.calls if call[0] in ('action', 'servo')]
    assert events == ['action', 'action', 'servo', 'servo', 'servo',
                      'action', 'servo', 'servo', 'servo']
    assert all(call[3] == 'xyzrz' and call[2][2] == .2 for call in holds)


def test_velocity_and_depth_parameters_load_from_yaml():
    path = Path(__file__).parents[1]/'config/tasks/26rb_drop_ball_target_rack.yaml'
    params = load_task(path)[0]['params']
    configured = yaml.safe_load(path.read_text())['params']['visual_servo']
    assert params['down_visual_servo_max_speed_mps'] == configured['max_speed_mps']
    assert params['down_depth_hold_gain'] == configured['depth_gain']
    assert params['down_depth_hold_max_speed_mps'] == configured['max_vertical_speed_mps']
    assert params['down_depth_hold_tolerance_m'] == configured['depth_tolerance_m']
    assert 'down_visual_servo_max_step_m' not in params


@pytest.mark.parametrize('camera', ['down_left', 'down_right'])
def test_hairpin_alignment_uses_original_rack_anchor_and_holds_depth(rig, camera):
    rig.task._aligned_camera = camera
    rig.task._servo_depth = .2
    camera_pose = rig.pose.copy()
    assert rig.task._align_disc_claw()
    disc_pose = rig.pose.copy()
    # A second camera-to-claw offset from the current disc-aligned pose would be wrong.
    rig.pose[:2] += [.3, -.1]
    rig.pose[2] = .26
    rig.pose[5] = -45.
    rig.task._alignment_timeout = 20.
    assert rig.task._align_hairpin_claw()
    yaw = np.deg2rad(camera_pose[5])
    rotation = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
    rack_xy = camera_pose[:2]+rotation @ rig.node.camera_extrinsics[camera].translation[:2]
    assert np.linalg.norm(disc_pose[:2]+rotation @ np.array([-.38, 0.])-rack_xy) <= .02
    assert np.linalg.norm(rig.pose[:2]+rotation @ np.array([.08, 0.])-rack_xy) <= .02
    assert rig.pose[2] == .2 and abs(rig.pose[5]-camera_pose[5]) <= 3.
    assert rig.messages == []


def test_hairpin_alignment_without_saved_anchor_does_not_move(rig):
    rig.task._aligned_camera = 'down_left'
    assert not rig.task._align_hairpin_claw()
    assert rig.calls == [] and rig.messages == []


def test_release_ring_publishes_configurable_degrees_on_servo_two(rig):
    params = {'ring_release_angle_deg': 15., 'ring_release_repeat_count': 2,
              'ring_release_repeat_period': .2, 'ring_release_settle_seconds': .5}
    task = mod.RB26DropBallTargetRackTask(rig.node, params)
    started = rig.clock[0]
    assert task._release_ring()
    assert [(msg.servo_id, msg.angle) for msg in rig.messages] == [(2, 15.)]*2
    assert rig.clock[0]-started == pytest.approx(.7)


@pytest.mark.parametrize('failure', ['missing_tf', 'alignment_motion', 'cancel_ball', 'cancel_ring'])
def test_failed_ring_alignment_or_cancellation_prevents_further_release(rig, monkeypatch, failure):
    monkeypatch.setattr(mod, 'TargetRackSearch', lambda *args: NS(execute=TaskOutcome.ok))
    monkeypatch.setattr(mod, 'normalized_image_error', lambda *args: (0., 0.))
    rig.task._flash_green = lambda *args, **kwargs: True
    lookup = rig.node.camera_extrinsics_provider.lookup_transform
    if failure == 'missing_tf':
        def missing(base, child):
            if child == 'hairpin_claw_link':
                raise RuntimeError('missing hairpin TF')
            return lookup(base, child)
        rig.node.camera_extrinsics_provider.lookup_transform = missing
    elif failure == 'alignment_motion':
        action = rig.node._send_action_goal
        def fail_hairpin(*args, **kwargs):
            completed_actions = sum(call[0] == 'action' for call in rig.calls)
            if completed_actions >= 2:
                return False, '模拟发夹爪定位失败'
            return action(*args, **kwargs)
        rig.node._send_action_goal = fail_hairpin
    else:
        servo_id = 1 if failure == 'cancel_ball' else 2
        def cancel_on_servo():
            if any(msg.servo_id == servo_id for msg in rig.messages):
                rig.node.stopped = True
        rig.state.hook = cancel_on_servo
    result = rig.task.execute()
    assert not result
    if failure in ('missing_tf', 'alignment_motion'):
        assert result.failure_code.endswith('.ring_alignment')
        assert [(msg.servo_id, msg.angle) for msg in rig.messages] == [(1, 180.)]*3
    elif failure == 'cancel_ball':
        assert [msg.servo_id for msg in rig.messages] == [1]
    else:
        assert [msg.servo_id for msg in rig.messages] == [1, 1, 1, 2]
    assert rig.node._light_state == 0


@pytest.mark.parametrize('params', [{'ring_release_angle_deg': float('nan')},
    {'ring_release_angle_deg': 271.}, {'ring_release_angle_deg': -1.}, {'release_angle_deg': 271.}, {'release_angle_deg': -1.}, {'ring_release_repeat_count': 0},
    {'ring_release_repeat_count': True}, {'ring_release_repeat_period': -.1},
    {'ring_release_settle_seconds': float('inf')}])
def test_invalid_ring_release_settings_rejected_at_runtime_and_yaml(rig, params):
    with pytest.raises(ValueError):
        mod.RB26DropBallTargetRackTask(rig.node, params)
    with pytest.raises(ConfigError):
        _validate_params('26rb_drop_ball_target_rack', params)


def test_ring_release_yaml_defaults():
    path = Path(__file__).parents[1]/'config/tasks/26rb_drop_ball_target_rack.yaml'
    params = load_task(path)[0]['params']
    assert params['release_angle_deg'] == 180.
    assert params['ring_release_angle_deg'] == 210.
    assert params['ring_release_repeat_count'] == 3
    assert params['ring_release_repeat_period'] == .1
    assert params['ring_release_settle_seconds'] == 1.


@pytest.mark.parametrize('key', ['release_angle_deg', 'ring_release_angle_deg'])
@pytest.mark.parametrize('angle', [0., 180., 181., 270.])
def test_release_servos_full_angle_range(rig, key, angle):
    task = mod.RB26DropBallTargetRackTask(rig.node, {key: angle})
    field = '_release_angle_deg' if key == 'release_angle_deg' else '_ring_release_angle_deg'
    assert getattr(task, field) == angle
    _validate_params('26rb_drop_ball_target_rack', {key: angle})


@pytest.mark.parametrize('servo_id', [1, 2])
def test_servo_270_reaches_publisher(rig, servo_id):
    task = mod.RB26DropBallTargetRackTask(rig.node, {
        'release_angle_deg': 270., 'ring_release_angle_deg': 270.})
    release = task._release_ball if servo_id == 1 else task._release_ring
    assert release()
    assert [(msg.servo_id, msg.angle) for msg in rig.messages] == [(servo_id, 270.)]*3


@pytest.mark.parametrize('residual', [(0.0033, 0.0799), (-0.0047, 0.0591)])
def test_logged_rack_residual_can_finish_with_relaxed_conditions(rig, monkeypatch, residual):
    rig.task = mod.RB26DropBallTargetRackTask(rig.node, {'depth_task_depth_m': .2})
    rig.pose[2] = .24
    monkeypatch.setattr(mod, 'normalized_image_error', lambda *args: residual)
    started = rig.clock[0]
    assert rig.task._servo_rack()
    assert not rig.task._visual_servo_timed_out
    assert .5 <= rig.clock[0]-started < 1.
    assert all(np.allclose(call[2][:2], 0.) for call in rig.calls if call[0] == 'velocity')


def test_residual_outside_relaxed_tolerance_still_uses_timeout_fallback(rig, monkeypatch):
    monkeypatch.setattr(mod, 'normalized_image_error', lambda *args: (0., .081))
    rig.task._servo_timeout = 1.
    assert rig.task._servo_rack()
    assert rig.task._visual_servo_timed_out
    assert any('超时' in line for line in rig.logs)


def test_timeout_drop_flow_does_not_announce_visual_servo_success(rig, monkeypatch):
    monkeypatch.setattr(mod, 'TargetRackSearch', lambda *args: NS(execute=TaskOutcome.ok))
    rig.task._servo_timeout = 1.
    indications = []
    def flash(count, label, **kwargs):
        indications.append(label)
        return True
    rig.task._flash_green = flash
    assert rig.task.execute()
    assert rig.task._visual_servo_timed_out
    assert indications == ['放球完成', '放环完成']
    assert [msg.servo_id for msg in rig.messages] == [1, 1, 1, 2, 2, 2]


def test_completed_visual_servo_retains_success_indication(rig, monkeypatch):
    monkeypatch.setattr(mod, 'TargetRackSearch', lambda *args: NS(execute=TaskOutcome.ok))
    monkeypatch.setattr(mod, 'normalized_image_error', lambda *args: (0., 0.))
    indications = []
    rig.task._flash_green = lambda count, label, **kwargs: indications.append(label) or True
    assert rig.task.execute()
    assert not rig.task._visual_servo_timed_out
    assert indications == ['rack视觉伺服成功', '放球完成', '放环完成']


def test_last_velocity_ack_can_finish_after_visual_deadline_and_continue_drop(rig, monkeypatch):
    monkeypatch.setattr(mod, 'TargetRackSearch', lambda *args: NS(execute=TaskOutcome.ok))
    rig.task._servo_timeout = 1.
    started = rig.clock[0]
    visual_deadline = started+1.
    original = rig.node._send_body_velocity
    acknowledgements = []
    def delayed_ack(*args, **kwargs):
        sent_at = rig.clock[0]
        reply_at = sent_at+.12
        wait_deadline = kwargs['wait_deadline']
        acknowledgements.append((sent_at, reply_at, wait_deadline))
        if reply_at > wait_deadline:
            rig.clock[0] = wait_deadline
            return False, '等待动作结果超时'
        rig.clock[0] = reply_at
        return original(*args, **kwargs)
    rig.node._send_body_velocity = delayed_ack
    rig.task._flash_green = lambda *args, **kwargs: True
    assert rig.task.execute()
    assert rig.task._visual_servo_timed_out
    assert acknowledgements[-1][0] < visual_deadline < acknowledgements[-1][1]
    assert all(sent_at < visual_deadline for sent_at, _, _ in acknowledgements)
    assert all(deadline-sent_at == pytest.approx(2.) for sent_at, _, deadline in acknowledgements)
    assert [message.servo_id for message in rig.messages] == [1, 1, 1, 2, 2, 2]
    assert not any('rack速度指令失败' in line for line in rig.logs)


def test_acknowledgement_beyond_independent_budget_still_prevents_drop(rig, monkeypatch):
    monkeypatch.setattr(mod, 'TargetRackSearch', lambda *args: NS(execute=TaskOutcome.ok))
    rig.task._flash_green = lambda *args, **kwargs: True
    budgets = []
    def missing_reply(*args, **kwargs):
        budgets.append(kwargs['wait_deadline']-rig.clock[0])
        rig.clock[0] = kwargs['wait_deadline']
        return False, '等待动作结果超时'
    rig.node._send_body_velocity = missing_reply
    assert not rig.task.execute()
    assert budgets == pytest.approx([2.])
    assert rig.messages == []
    assert any('rack速度指令失败' in line for line in rig.logs)
    assert rig.calls[-1][0] == 'action'  # Attempt the normal SET hold cleanup.


@pytest.mark.parametrize('expires_during_geometry', [False, True])
def test_expired_visual_window_never_sends_another_velocity_goal(rig, expires_during_geometry):
    deadline = rig.clock[0]+(.05 if expires_during_geometry else 0.)
    if expires_during_geometry:
        def slow_pose():
            rig.clock[0] += .1
            return tuple(rig.pose)
        rig.task._servo_pose = slow_pose
    assert not rig.task._send_rack_velocity(np.zeros(2), deadline)
    assert rig.calls == []
