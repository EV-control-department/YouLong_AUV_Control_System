"""Three-target and legacy acceptance checks with virtual cameras/actuators."""
from itertools import permutations
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

from test_26rb_grab_ball_ring import Node, mod
from test_pickup_windows import detection, stream
from uv_task.config_loader import ConfigError, _validate_params, load_task
from uv_task.task_outcome import TaskOutcome


@pytest.fixture
def three(monkeypatch):
    node = Node()
    ids = {'collection_frame_down': 0, 'pink_golf': 7, 'red_ring': 8, 'yellow_golf': 11}
    node._model_mapping = NS(model_class_id=lambda name, **kwargs: ids.get(name))
    monkeypatch.setattr(mod.time, 'monotonic', lambda: node.clock)
    monkeypatch.setattr(mod.time, 'sleep', node.sleep)
    monkeypatch.setattr(mod, 'CollectionFrameSearch', lambda *args: NS(execute=TaskOutcome.ok))
    params = load_task(Path(__file__).parents[1]/'config/tasks/26rb_grab_ball_ring.yaml')[0]['params']
    params.update(work_depth_m=.2, golf_grab_depth_m=.44, ring_grab_depth_m=.44,
                  ring_target_lock_enabled=False, ring_confirmation='class_absence')
    task = mod.RB26GrabBallRingTask(node, params)
    node.hook = lambda: stream(node, tuple(cid for cid in (0, 7, 8, 11) if cid not in node.removed))
    return node, task


@pytest.mark.parametrize('order', list(permutations(('ring', 'yellow', 'red'))))
def test_three_orders_validate(order):
    assert _validate_params('26rb_grab_ball_ring', {'pickup': {'order': list(order)}})['pickup_order'] == list(order)


@pytest.mark.parametrize('order', [[], ['ring', 'ring'], ['ring', 'yellow'], ['ring', 'golf', 'red'],
                                  ['ring', 'yellow', 'yellow'], ['ring', 'yellow', 'pink_golf'],
                                  ['ring', 'yellow', 'red', 'golf'], ['ring', 'yellow', {}]])
def test_bad_orders_rejected_at_load_and_runtime(order):
    with pytest.raises(ConfigError):
        _validate_params('26rb_grab_ball_ring', {'pickup_order': order})
    with pytest.raises(ValueError):
        mod.validate_combined_params({'pickup_order': order})


@pytest.mark.parametrize('order', [('golf', 'ring'), ('ring', 'golf'), None])
def test_legacy_routes_keep_single_ball_and_default_angle(three, order):
    node, _ = three
    params = {'collection_frame_position': [0., 0., .5], 'golf_color': 'yellow_golf'}
    if order is not None:
        params['pickup_order'] = list(order)
    task = mod.RB26GrabBallRingTask(node, params)
    assert task.pickup_order == (order or ('golf', 'ring'))
    assert not task.colored_pickup and set(task.controllers) == {'golf', 'ring'}
    assert task.ball._golf_class_id == 11 and task.ball._claw_prepare_angle == 0.
    task._update_target_progress('golf', attempts=2, status='success')
    assert task.state.pickup.golf_attempts == 2
    assert task.state.pickup.golf_status == 'success'
    assert task.state.pickup.yellow_status == task.state.pickup.red_status == 'pending'


def test_model_ids_are_resolved_and_controllers_do_not_share_pid(three):
    node, task = three
    ids = {'collection_frame_down': 42, 'yellow_golf': 103, 'pink_golf': 205, 'red_ring': 307}
    node._model_mapping = NS(model_class_id=lambda name, **kwargs: ids.get(name))
    task = mod.RB26GrabBallRingTask(node, task.p)
    assert [task.controllers[k]._golf_class_id for k in task.pickup_order] == [307, 103, 205]
    yellow, red = task.controllers['yellow'], task.controllers['red']
    yellow._target_pid_integral[:] = [.2, -.1]
    assert np.allclose(red._target_pid_integral, 0.)
    assert yellow._gripper_frame == red._gripper_frame == 'disc_claw_link'
    assert task.ring._gripper_frame == 'hairpin_claw_link'


def test_full_three_pickup_with_both_colors_visible_and_real_descent(three):
    node, task = three
    mechanical = []
    descend = task._descend_to_depth
    def down(kind, controller):
        mechanical.append((kind, controller._golf_class_id))
        reached = descend(kind, controller)
        assert reached
        if kind != 'ring':
            node.removed.add(controller._golf_class_id)
        return reached
    servo = node.set_servo
    def send(angle, label, servo_id=1):
        servo(angle, label, servo_id)
        if servo_id == 2 and angle == 50.:
            node.removed.add(8)
    node.set_servo = send
    task._descend_to_depth = down
    outcome = task.execute()
    assert outcome, outcome.message
    assert mechanical == [('ring', 8), ('yellow', 11), ('red', 7)]
    progress = task.state.pickup
    assert [getattr(progress, k+'_attempts') for k in task.pickup_order] == [1, 1, 1]
    assert [getattr(progress, k+'_status') for k in task.pickup_order] == ['success']*3
    assert progress.golf_attempts == 2 and progress.golf_status == 'success'
    assert [angle for _, sid, angle in node.servos if sid == 1] == [65.]*6
    assert [angle for _, sid, angle in node.servos if sid == 2] == [210.]*3+[50.]*3
    assert np.allclose(node.velocity, 0.)


@pytest.mark.parametrize('failed,code', [
    (set(), ''), ({'yellow'}, 'golf_exhausted'), ({'red'}, 'golf_exhausted'),
    ({'ring'}, 'ring_exhausted'), ({'yellow', 'red'}, 'both_exhausted'),
    ({'ring', 'red'}, 'both_exhausted'), ({'ring', 'yellow', 'red'}, 'both_exhausted')])
def test_normal_absence_continues_and_preserves_failure_codes(three, failed, code):
    node, task = three
    seen, entered = [], []
    def align(kind, controller, started, **kwargs):
        entered.append((kind, started))
        if kind in failed:
            return 'unobserved'
        return 'absent' if kwargs.get('checking') else 'ready'
    task._align_target = align
    task._mechanical_attempt = lambda kind, controller: seen.append(kind) or True
    task._return_frame = lambda controller: setattr(node, 'clock', node.clock+2.)
    outcome = task.execute()
    assert bool(outcome) == (not failed)
    assert [k for k in task.pickup_order if k not in failed] == seen
    assert list(dict.fromkeys(k for k, _ in entered)) == list(task.pickup_order)
    if failed:
        assert outcome.failure_code == '26rb_grab_ball_ring.'+code
        assert all(k in outcome.message for k in failed)
    progress = task.state.pickup
    assert progress.golf_status == ('failed' if failed & {'yellow', 'red'} else 'success')


def test_attempt_limits_and_retry_depth_are_independent(three):
    node, task = three
    calls = []
    def align(kind, controller, started, **kwargs):
        return 'exhausted' if kwargs.get('exhausted') else 'ready'
    def grab(kind, controller):
        calls.append((kind, task._grab_target_depth(kind)))
        return True
    task._align_target, task._mechanical_attempt = align, grab
    task._return_frame = lambda controller: None
    assert not task.execute()
    assert [kind for kind, _ in calls] == ['ring']*3+['yellow']*3+['red']*3
    for kind in task.pickup_order:
        assert [depth for k, depth in calls if k == kind] == pytest.approx([.44, .49, .54])
        assert getattr(task.state.pickup, kind+'_attempts') == 3
    assert task.state.pickup.golf_attempts == 6


def test_hard_recovery_failure_does_not_enter_red_stage(three):
    node, task = three
    calls = []
    task._align_target = lambda kind, controller, started, **kw: 'absent' if kw.get('checking') else 'ready'
    task._mechanical_attempt = lambda kind, controller: calls.append(kind) or True
    def recover(controller):
        if controller is task.controllers['yellow']:
            raise mod.PickupFailure('yellow 回位失败')
    task._return_frame = recover
    outcome = task.execute()
    assert outcome.failure_code == '26rb_grab_ball_ring.recovery'
    assert calls == ['ring', 'yellow']
    assert task.state.pickup.red_attempts == 0


def test_colored_observation_poses_and_world_targets_do_not_overwrite(three):
    node, task = three
    for kind, cid, point, pose in (
        ('yellow', 11, (1., 2.), (.5, .2, .2, 0., 0., 10.)),
        ('red', 7, (3., 4.), (1.5, .8, .2, 0., 0., -20.))):
        node.pose[:] = pose
        task._record_target(kind, task.controllers[kind], 'down_left', detection(node, cid),
                            timed_out=False, world_target=point)
    assert task._targets['yellow'][0] == (1., 2.) and task._targets['red'][0] == (3., 4.)
    yellow_pose, red_pose = task.state.pickup.yellow_camera_pose, task.state.pickup.red_camera_pose
    assert yellow_pose != red_pose
    for kind, expected in (('yellow', yellow_pose), ('red', red_pose)):
        task._return_frame(task.controllers[kind])
        assert tuple(node.pose[[0, 1, 2, 5]]) == pytest.approx(expected)
    assert task.state.pickup.golf_camera_pose == red_pose
    snapshot = task.state.pickup
    task.state.reset_pickup()
    assert snapshot.yellow_camera_pose == yellow_pose
    assert task.state.pickup.yellow_camera_pose is None
    assert task.state.pickup.yellow_attempts == task.state.pickup.red_attempts == 0


@pytest.mark.parametrize('kind,cid', [('ring', 8), ('yellow', 11), ('red', 7)])
def test_blind_grab_consumes_same_attempt_pool_and_cannot_repeat(three, kind, cid):
    node, task = three
    controller = task.controllers[kind]
    node.hook = lambda: stream(node, (cid,), error=(.15, -.1))
    node.hook()
    grabs = []
    def grab(actual, current):
        assert actual == kind and current is controller
        assert kind in task._blind_grabs_used
        grabs.append(getattr(task.state.pickup, kind+'_attempts'))
        return True
    task._mechanical_attempt = grab
    task._return_frame = lambda controller: None
    task._run_target(kind, controller, started=node.clock)
    assert grabs == [1]
    assert getattr(task.state.pickup, kind+'_attempts') == 1
    assert getattr(task.state.pickup, kind+'_status') == 'failed'
    assert task._blind_grabs_used == {kind}
    other = 'red' if kind != 'red' else 'yellow'
    assert getattr(task.state.pickup, other+'_attempts') == 0


def test_each_target_can_blind_grab_once_and_only_uses_current_color(three):
    node, task = three
    task.servo_timeout = 2.
    for kind in task.pickup_order:
        controller = task.controllers[kind]
        node.hook = lambda: stream(node, (7, 8, 11), error=(.15, -.1))
        node.hook()
        controller._target_pid_integral[:] = [.2, -.1]
        controller._pending_golf_priority = object()
        assert task._align_target(kind, controller, node.clock) == 'ready'
        assert task._targets[kind][2] in ('down_left', 'down_right')
        assert np.allclose(controller._target_pid_integral, 0.)
        assert controller._pending_golf_priority is None
    assert task._blind_grabs_used == set(task.pickup_order)


@pytest.mark.parametrize('kind,cid', [('ring', 8), ('yellow', 11), ('red', 7)])
def test_normal_and_blind_grabs_share_three_attempts(three, kind, cid):
    node, task = three
    task.servo_timeout = 2.
    controller = task.controllers[kind]
    normal = task._align_target
    entry = [0]
    def align(actual, current, started, **kwargs):
        entry[0] += 1
        if entry[0] <= 2:
            return 'ready'
        node.hook = lambda: stream(node, (cid,), error=(.15, -.1))
        node.hook()
        return normal(actual, current, started, **kwargs)
    attempts = []
    task._align_target = align
    task._mechanical_attempt = lambda k, c: attempts.append((
        getattr(task.state.pickup, k+'_attempts'), k in task._blind_grabs_used)) or True
    task._return_frame = lambda c: None
    task._run_target(kind, controller, started=node.clock)
    assert attempts == [(1, False), (2, False), (3, True)]
    assert getattr(task.state.pickup, kind+'_attempts') == 3


@pytest.mark.parametrize('kind,cid', [('yellow', 11), ('red', 7)])
@pytest.mark.parametrize('reason', ['outside', 'projection', 'exhausted'])
def test_colored_timeout_does_not_blind_without_eligibility(three, monkeypatch, kind, cid, reason):
    node, task = three
    task.servo_timeout = 2.
    error = (.35, 0.) if reason == 'outside' else (.15, -.1)
    node.hook = lambda: stream(node, (cid,), error=error)
    if reason == 'projection':
        def invalid(*args):
            raise ValueError('invalid projection')
        monkeypatch.setattr(mod, 'target_world_xy', invalid)
    node.hook()
    result = task._align_target(kind, task.controllers[kind], node.clock, exhausted=reason == 'exhausted')
    assert result in ('timeout', 'exhausted')
    assert kind not in task._blind_grabs_used and node.servos == []


@pytest.mark.parametrize('kind', ['yellow', 'red'])
def test_blind_claw_alignment_uses_saved_point_and_disc_tf(three, kind):
    node, task = three
    controller = task.controllers[kind]
    saved = (2., -1.)
    task._targets[kind] = (saved, tuple(node.pose), 'down_left')
    task._blind_targets[kind] = None
    node.pose[:2] = [4., 3.]
    controller._aligned_camera = 'down_left'
    task._update_target_progress(kind, attempts=1)
    controller._wait_pre_descent_settle = lambda: True
    task._descend_to_depth = lambda k, c: True
    task._mechanical_attempt(kind, controller)
    # yaw=0: disc mount X=-0.32, Y=0; current position/pixels cannot replace saved target.
    action = [c for c in node.calls if c[0] == 'action'][-1]
    assert action[2][:2] == pytest.approx([2.32, -1.])
    assert [(sid, angle) for _, sid, angle in node.servos] == [(1, 65.)]*3


@pytest.mark.parametrize('kind,cid', [('yellow', 11), ('red', 7)])
def test_colored_blind_grab_freezes_observed_world_point_during_drift(three, monkeypatch, kind, cid):
    node, task = three
    task.servo_timeout = 3.
    started, points = node.clock, []
    project = mod.target_world_xy
    def save(*args):
        point = project(*args)
        points.append(point)
        return point
    monkeypatch.setattr(mod, 'target_world_xy', save)
    def images():
        if node.clock-started > .3:
            node.pose[:2] = [3., -2.]
        stream(node, (cid,) if node.clock-started < .3 else (), error=(.15, -.1))
    node.hook = images
    images()
    assert task._align_target(kind, task.controllers[kind], started) == 'ready'
    assert task._targets[kind][0] == pytest.approx(points[-1])
    assert np.linalg.norm(np.asarray(task._targets[kind][1][:2])-node.pose[:2]) > 1.


@pytest.mark.parametrize('task_name', ['26rb_grab_golf', '26rb_grab_ball_ring'])
@pytest.mark.parametrize('settings,value', [
    ({'claw': {'prepare_angle_deg': 65.}}, 65.),
    ({'claw': {'prepare_angle_rad': 1.2}}, 1.2),
    ({'claw_prepare_angle_deg': 65.}, 65.),
    ({'claw_prepare_angle_rad': 0.}, 0.)])
def test_prepare_angle_alias_preserves_raw_numeric_value(three, task_name, settings, value):
    node, _ = three
    normalized = _validate_params(task_name, settings)
    assert normalized == {'claw_prepare_angle_rad': value}
    controller = mod.PickupController(node, normalized)
    assert controller._prepare_claw()
    assert [angle for _, sid, angle in node.servos if sid == 1] == [value]*3


@pytest.mark.parametrize('settings', [
    {'claw': {'prepare_angle_deg': 65., 'prepare_angle_rad': 0.}},
    {'claw': {'prepare_angle_deg': 65.}, 'claw_prepare_angle_rad': 0.},
    {'claw_prepare_angle_deg': 65., 'claw_prepare_angle_rad': 0.}])
def test_duplicate_prepare_angle_aliases_rejected(settings):
    with pytest.raises(ConfigError, match='重复'):
        _validate_params('26rb_grab_ball_ring', settings)


@pytest.mark.parametrize('missing', ['yellow_golf', 'pink_golf', 'red_ring'])
def test_three_mode_requires_model_classes_at_load(missing):
    registry = NS(entries=[NS(name=name) for name in ('yellow_golf', 'pink_golf', 'red_ring') if name != missing])
    with pytest.raises(ConfigError, match=missing):
        _validate_params('26rb_grab_ball_ring', {'pickup_order': ['ring', 'yellow', 'red']}, registry)


@pytest.mark.parametrize('field,value', [('yellow_attempts', -1), ('red_attempts', True),
                                       ('yellow_status', 'bad'), ('red_camera_pose', [0., 0., float('nan'), 0.])])
def test_new_state_fields_are_validated(three, field, value):
    with pytest.raises(ValueError):
        three[0].state.update_pickup(**{field: value})


def test_field_yaml_uses_three_stages_and_confirmed_angles():
    directory = Path(__file__).parents[1]/'config/tasks'
    pickup = load_task(directory/'26rb_grab_ball_ring.yaml')[0]['params']
    drop = load_task(directory/'26rb_drop_ball_target_rack.yaml')[0]['params']
    assert pickup['pickup_order'] == ['ring', 'yellow', 'red']
    assert pickup['claw_prepare_angle_rad'] == 65.
    assert pickup['golf_max_attempts'] == pickup['ring_max_attempts'] == 3
    assert pickup['ring_open_angle_deg'] == 210. and pickup['ring_close_angle_deg'] == 50.
    assert drop['release_angle_deg'] == 180. and drop['ring_release_angle_deg'] == 210.
    assert drop['completion_wait_seconds'] == 2.
