"""Micro-servo timeout fallback and fixed-column logs using virtual devices."""
import math
import re

import numpy as np
import pytest

from test_26rb_grab_ball_ring import rig, mod
from test_pickup_windows import detection, stream


@pytest.mark.parametrize('kind,cid', [('golf', 7), ('ring', 8)])
@pytest.mark.parametrize('seconds', [20., 30.])
def test_micro_entry_timeout_permits_one_blind_grab_per_target(rig, kind, cid, seconds):
    node, task = rig
    task.servo_timeout = seconds
    controller = task.ball if kind == 'golf' else task.ring
    node.hook = lambda: stream(node, (cid,), error=(.15, -.1))
    node.hook()
    started = node.clock
    assert task._align_target(kind, controller, started) == 'ready'
    assert node.clock-started == pytest.approx(seconds)
    assert kind in task._blind_targets and task._blind_grabs_used == {kind}
    assert np.allclose(node.velocity, 0.)
    assert node.servos == []
    # Eligibility and the last pixel alone cannot cause repeated blind grabs.
    started = node.clock
    assert task._align_target(kind, controller, started) == 'timeout'
    assert node.clock-started == pytest.approx(seconds)
    assert kind not in task._blind_targets and task._blind_grabs_used == {kind}


@pytest.mark.parametrize('kind,cid', [('golf', 7), ('ring', 8)])
def test_target_outside_micro_zone_is_skipped_at_timeout(rig, kind, cid):
    node, task = rig
    controller = task.ball if kind == 'golf' else task.ring
    node.hook = lambda: stream(node, (cid,), error=(.35, .0))
    node.hook()
    assert task._align_target(kind, controller, node.clock) == 'timeout'
    assert task._blind_targets == {} and task._blind_grabs_used == set()
    assert task._targets == {} and node.servos == []


@pytest.mark.parametrize('kind,cid', [('golf', 7), ('ring', 8)])
def test_equal_center_error_prefers_latest_position_and_retains_it_after_loss_and_drift(rig, monkeypatch, kind, cid):
    node, task = rig
    controller = task.ball if kind == 'golf' else task.ring
    started = node.clock
    observations = []
    project = mod.target_world_xy
    def record_projection(*args):
        point = project(*args)
        observations.append((point, args[3]))
        return point
    monkeypatch.setattr(mod, 'target_world_xy', record_projection)
    drifted = False
    drift_position = None
    def images():
        nonlocal drifted, drift_position
        age = node.clock-started
        if age > 1.2:
            if not drifted:
                drift_position = node.pose[:2]+[1., -.5]
                drifted = True
            # Recovery now heads back to the target observation point. Keep
            # the external disturbance active to test frozen-world geometry
            # while the measured vehicle pose still differs at timeout.
            node.pose[:2] = drift_position
            node.pose[5] = 25.
        stream(node, (cid,) if age < .3 else (), error=(.15, -.1))
    node.hook = images
    images()
    assert task._align_target(kind, controller, started) == 'ready'
    assert drifted and observations
    target, observed_pose, camera = task._targets[kind]
    assert target == pytest.approx(observations[-1][0])
    assert observed_pose == pytest.approx(observations[-1][1])
    assert getattr(task.state.pickup, kind+'_camera_pose')[:2] == pytest.approx(observed_pose[:2])
    assert np.linalg.norm(np.asarray(node.pose[:2])-observed_pose[:2]) > .05
    assert kind in task._blind_targets


def test_micro_entry_without_valid_world_projection_cannot_blind_grab(rig, monkeypatch):
    node, task = rig
    node.hook = lambda: stream(node, (7,), error=(.15, -.1))
    def invalid_projection(*args):
        raise ValueError('invalid projection')
    monkeypatch.setattr(mod, 'target_world_xy', invalid_projection)
    node.hook()
    assert task._align_target('golf', task.ball, node.clock) == 'timeout'
    assert task._blind_targets == {} and task._blind_grabs_used == set()


@pytest.mark.parametrize('kind,cid', [('golf', 7), ('ring', 8)])
def test_run_target_calls_mechanical_attempt_once_and_then_checks(rig, kind, cid):
    node, task = rig
    controller = task.ball if kind == 'golf' else task.ring
    node.hook = lambda: stream(node, (cid,), error=(.15, -.1))
    node.hook()
    started = node.clock
    calls = []
    def grab(actual_kind, actual_controller):
        assert actual_kind == kind and actual_controller is controller
        assert kind in task._blind_targets
        assert node.clock-started == pytest.approx(task.servo_timeout)
        assert np.allclose(node.velocity, 0.)
        calls.append(kind)
        node.hook = lambda: stream(node)
        return True
    task._mechanical_attempt = grab
    task._run_target(kind, controller, started=started)
    assert calls == [kind]
    assert getattr(task.state.pickup, kind+'_attempts') == 1
    assert getattr(task.state.pickup, kind+'_status') == 'success'


@pytest.mark.parametrize('camera', ['down_left', 'down_right'])
@pytest.mark.parametrize('attitude', [(0., 0., 0.), (5., -7., 25.)])
def test_ring_blind_uses_frozen_world_axis_without_two_second_wait(rig, camera, attitude):
    node, task = rig
    node.pose[3:] = attitude
    ring = detection(node, 8, camera, error=(.15, -.1))
    ring.orientation_valid = True
    ring.orientation_quality = .95
    ring.orientation_axis_deg = 35.
    task._sample_axis(camera, node.clock, type('Message', (), {'detections': [ring]})(), collect=False)
    assert task._last_ring_axis is not None
    history = task._last_ring_axis
    task._blind_targets['ring'] = history
    target = (1., 2.)
    task._targets['ring'] = (target, tuple(node.pose), camera)
    task._collect_ring_axis = lambda: pytest.fail('blind grab must not start a new yaw window')
    # Later samples must not replace the direction frozen at the timeout.
    task._last_ring_axis = (node.clock+1., (history[1]+30.) % 180.)
    xy = node.pose[:2].copy()
    started = node.clock
    assert task._orient_and_align_ring()
    assert node.clock == started
    axis = np.array([math.cos(math.radians(history[1])), math.sin(math.radians(history[1]))])
    heading = np.array([math.cos(math.radians(node.pose[5])), math.sin(math.radians(node.pose[5]))])
    assert abs(float(axis @ heading)) == pytest.approx(1., abs=1e-8)
    assert float(heading @ (xy-np.asarray(task.state.pickup.start_xy))) >= -1e-9
    tf = node.camera_extrinsics_provider.lookup_transform('base_link', 'hairpin_claw_link').translation
    offset = mod.body_to_world_rotation(node.pose) @ [tf.x, tf.y, tf.z]
    assert node.pose[:2]+offset[:2] == pytest.approx(target)
    actions = [call for call in node.calls if call[0] == 'action']
    assert len(actions) == 1 and actions[0][3] == 'xyzrz'


def test_ring_blind_without_any_history_keeps_yaw(rig):
    node, task = rig
    node.pose[5] = 17.
    task._blind_targets['ring'] = None
    task._targets['ring'] = ((1., 2.), tuple(node.pose), 'down_left')
    task._collect_ring_axis = lambda: pytest.fail('blind grab must not wait for yaw samples')
    assert task._orient_and_align_ring()
    assert node.pose[5] == pytest.approx(17.)
    assert any('没有历史有效方向' in line for line in node.logs)


def test_fixed_columns_cover_missing_data_eye_change_loss_and_blind(rig):
    node, task = rig
    started = node.clock
    def images():
        age = node.clock-started
        if age < .3:
            stream(node)
        elif age < 1.3:
            stream(node, (7,), error=(.15, -.1))
        elif age < 3.:
            node.feed('down_left', [])
            node.feed('down_right', [detection(node, 7, 'down_right', error=(-.17, .09))])
        else:
            stream(node)
    node.hook = images
    assert task._align_target('golf', task.ball, started) == 'ready'
    rows = [line for line in node.logs if line.startswith('[PICKUP] obj=')]
    keys = ('mode=', 'eye=', 'src=', 'tol_phase=', 'ex_n=', 'ey_n=', 'tol_n=',
            'elapsed_s=', 'remain_s=', 'hold_s=', 'I_x=', 'I_y=', 'vx_mps=')
    columns = [rows[0].index(key) for key in keys]
    assert all([row.index(key) for key in keys] == columns for row in rows)
    assert len({len(row) for row in rows}) == 1
    assert any('src=none ' in row and 'ex_n=       --' in row for row in rows)
    assert any('ex_n=  +0.1500' in row for row in rows)
    assert any('ex_n=  -0.1700' in row and 'eye=down_right' in row for row in rows)
    assert 'mode=blind' in rows[-1] and 'src=best ' in rows[-1]
    assert 'tol_n=  0.1000' in rows[-1] and 'remain_s=    0.00' in rows[-1]
    tolerances = [float(re.search(r'tol_n=\s*([0-9.]+)', row).group(1)) for row in rows]
    assert tolerances[0] == pytest.approx(.04) and tolerances[-1] == pytest.approx(.10)
    assert all(first <= second for first, second in zip(tolerances, tolerances[1:]))


@pytest.mark.parametrize('kind,cid', [('golf', 7), ('ring', 8)])
def test_blind_grab_runs_real_mechanical_sequence_on_virtual_actuators(rig, kind, cid):
    node, task = rig
    controller = task.ball if kind == 'golf' else task.ring
    def images():
        for camera in ('down_left', 'down_right'):
            target = detection(node, cid, camera, error=(.15, -.1))
            target.orientation_valid = kind == 'ring'
            target.orientation_quality = .95
            target.orientation_axis_deg = 90.
            node.feed(camera, [target])
    node.hook = images
    images()
    assert task._align_target(kind, controller, node.clock) == 'ready'
    target_xy = task._targets[kind][0]
    history = task._blind_targets[kind]
    node.hook = lambda: stream(node)
    task._collect_ring_axis = lambda: pytest.fail('blind sequence must use historical yaw directly')
    assert task._mechanical_attempt(kind, controller)
    assert node.pose[2] == pytest.approx(task.grab_depths[kind], abs=1e-6)
    tf = node.camera_extrinsics_provider.lookup_transform('base_link', controller._gripper_frame).translation
    offset = mod.body_to_world_rotation(node.pose) @ [tf.x, tf.y, tf.z]
    assert node.pose[:2]+offset[:2] == pytest.approx(target_xy)
    if kind == 'ring':
        assert history is not None
        assert abs(math.cos(math.radians(node.pose[5]-history[1]))) == pytest.approx(1., abs=1e-8)
        angles = [angle for stamp, servo_id, angle in node.servos if servo_id == 2]
        assert angles == [0.]*3+[90.]*3
    else:
        assert len([entry for entry in node.servos if entry[1] == 1]) == 3
    assert np.allclose(node.velocity, 0.)


def test_each_target_has_independent_blind_eligibility(rig):
    node, task = rig
    for kind, cid, controller in (('golf', 7, task.ball), ('ring', 8, task.ring)):
        node.hook = lambda: stream(node, (cid,), error=(.15, -.1))
        node.hook()
        assert task._align_target(kind, controller, node.clock) == 'ready'
        assert kind in task._blind_targets
    assert task._blind_grabs_used == {'golf', 'ring'}


@pytest.mark.parametrize('kind,cid', [('golf', 7), ('ring', 8)])
def test_prior_micro_entry_does_not_arm_a_later_attempt(rig, kind, cid):
    node, task = rig
    node.pose[2] = task.work_depth
    controller = task.ball if kind == 'golf' else task.ring
    node.hook = lambda: stream(node, (cid,))
    node.hook()
    assert task._align_target(kind, controller, node.clock) == 'ready'
    assert kind not in task._blind_grabs_used
    node.hook = lambda: stream(node, (cid,), error=(.35, 0.))
    node.hook()
    assert task._align_target(kind, controller, node.clock) == 'timeout'
    assert kind not in task._blind_targets and kind not in task._blind_grabs_used


def test_failed_parking_prevents_mechanical_blind_attempt(rig):
    node, task = rig
    node.hook = lambda: stream(node, (7,), error=(.15, -.1))
    node.hook()
    task.ball._stop_horizontal_velocity = lambda **kwargs: None
    task._mechanical_attempt = lambda *args: pytest.fail('blind motion requires completed parking')
    with pytest.raises(mod.PickupFailure, match='停车保持失败'):
        task._run_target('golf', task.ball, started=node.clock)
    assert task.state.pickup.golf_attempts == 0 and node.servos == []


def test_exhausted_target_does_not_get_an_extra_blind_attempt(rig):
    node, task = rig
    node.hook = lambda: stream(node, (7,), error=(.15, -.1))
    node.hook()
    assert task._align_target('golf', task.ball, node.clock, exhausted=True) == 'exhausted'
    assert task._blind_targets == {} and task._blind_grabs_used == set()


@pytest.mark.parametrize('kind,cid', [('golf', 7), ('ring', 8)])
def test_timeout_grabs_best_centered_historical_world_point_instead_of_last(rig, monkeypatch, kind, cid):
    node, task = rig
    controller = task.ball if kind == 'golf' else task.ring
    started = node.clock
    records = []
    project = mod.target_world_xy
    def remember(node, camera, det, pose, depth):
        world = project(node, camera, det, pose, depth)
        error = mod.normalized_image_error(node, camera, det)
        records.append((math.hypot(*error), node.clock, world, tuple(pose), camera))
        return world
    monkeypatch.setattr(mod, 'target_world_xy', remember)
    def images():
        age = node.clock-started
        error = (.11, .04) if .3 <= age < .6 else (.22, .10)
        stream(node, (cid,) if age < 1. else (), error=error)
    node.hook = images
    images()
    assert task._align_target(kind, controller, started) == 'ready'
    nearest = [entry for entry in records if entry[0] < .15][-1]
    target, observed_pose, camera = task._targets[kind]
    assert target == pytest.approx(nearest[2])
    assert observed_pose == pytest.approx(nearest[3])
    assert camera == nearest[4]
    assert np.linalg.norm(np.asarray(target)-records[-1][2]) > .02
    assert getattr(task.state.pickup, kind+'_camera_pose')[:2] == pytest.approx(nearest[3][:2])
    blind_row = [line for line in node.logs if 'mode=blind' in line][-1]
    assert 'src=best ' in blind_row and 'ex_n=  +0.1100' in blind_row
    task._align_claw(kind, controller, observed_pose[5])
    tf = node.camera_extrinsics_provider.lookup_transform('base_link', controller._gripper_frame).translation
    offset = mod.body_to_world_rotation(node.pose) @ [tf.x, tf.y, tf.z]
    assert node.pose[:2]+offset[:2] == pytest.approx(target)


@pytest.mark.parametrize('kind,cid', [('golf', 7), ('ring', 8)])
def test_invalid_better_projection_does_not_replace_valid_blind_record(rig, monkeypatch, kind, cid):
    node, task = rig
    controller = task.ball if kind == 'golf' else task.ring
    started = node.clock
    valid_records = []
    invalid_records = []
    project = mod.target_world_xy
    def checked_projection(node, camera, det, pose, depth):
        error = mod.normalized_image_error(node, camera, det)
        score = math.hypot(*error)
        if score < .15:
            invalid_records.append(node.clock)
            raise ValueError('投影无效的更居中观测')
        world = project(node, camera, det, pose, depth)
        valid_records.append((score, world))
        return world
    monkeypatch.setattr(mod, 'target_world_xy', checked_projection)
    def images():
        age = node.clock-started
        error = (.11, .04) if .3 <= age < .6 else ((.18, .04) if age < .3 else (.24, .10))
        stream(node, (cid,) if age < 1. else (), error=error)
    node.hook = images
    images()
    assert task._align_target(kind, controller, started) == 'ready'
    assert invalid_records
    best_valid = [entry for entry in valid_records if entry[0] < .20][-1]
    assert task._targets[kind][0] == pytest.approx(best_valid[1])


@pytest.mark.parametrize('kind,cid', [('golf', 7), ('ring', 8)])
def test_best_history_is_reset_between_servo_attempts(rig, kind, cid):
    node, task = rig
    controller = task.ball if kind == 'golf' else task.ring
    node.pose[2] = task.work_depth
    node.hook = lambda: stream(node, (cid,))
    node.hook()
    assert task._align_target(kind, controller, node.clock) == 'ready'
    old_target = np.asarray(task._targets[kind][0])
    node.pose[:2] += [2., -1.]
    node.hook = lambda: stream(node, (cid,), error=(.15, -.1))
    node.hook()
    assert task._align_target(kind, controller, node.clock) == 'ready'
    assert kind in task._blind_targets
    assert np.linalg.norm(np.asarray(task._targets[kind][0])-old_target) > 1.



@pytest.mark.parametrize('kind,cid', [('golf', 7), ('ring', 8)])
@pytest.mark.parametrize('claw_timeout', [30., 60.])
def test_relaxed_claw_alignment_budget_and_tolerances_do_not_change_other_set_steps(rig, kind, cid, claw_timeout):
    node, task = rig
    task = mod.RB26GrabBallRingTask(node, dict(task.p, claw_alignment_timeout=claw_timeout))
    controller = task.ball if kind == 'golf' else task.ring
    task._targets[kind] = ((1., 2.), tuple(node.pose), 'down_left')
    task._align_claw(kind, controller, 90.)
    action = [call for call in node.calls if call[0] == 'action'][-1]
    assert action[4]['timeout'] == claw_timeout
    assert action[4]['position_tolerance'] == .15
    assert action[4]['yaw_tolerance'] == 10.
    assert action[2][2] == task.work_depth
    task._set_pose([node.pose[0], node.pose[1], task.work_depth, node.pose[5]], '普通SET', timeout=10.)
    plain = [call for call in node.calls if call[0] == 'action'][-1]
    assert plain[4]['timeout'] == 10.
    assert 'position_tolerance' not in plain[4] and 'yaw_tolerance' not in plain[4]


@pytest.mark.parametrize('params', [
    {'claw_alignment': {'timeout': 0.}},
    {'claw_alignment': {'position_tolerance_m': float('nan')}},
    {'claw_alignment': {'yaw_tolerance_deg': -1.}},
    {'claw_alignment': {'yaw_tolerance_deg': 181.}},
])
def test_invalid_claw_alignment_configuration_is_rejected(params):
    from uv_task.config_loader import ConfigError, _validate_params
    with pytest.raises(ConfigError):
        _validate_params('26rb_grab_ball_ring', params)
