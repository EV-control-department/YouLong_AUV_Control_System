"""Deterministic state-machine acceptance checks; no actuator is connected."""
from importlib import import_module
from types import SimpleNamespace as NS
import math
import numpy as np
import pytest
from test_26rb_grab_ball_ring import rig, mod
from uv_task.config_loader import ConfigError, _validate_params
from uv_task.pickup_alignment import PickupWindow


def detection(node, cid, camera='down_left', error=(0., 0.)):
    side = node.camera_configs['down'].side(camera.split('_')[1])
    k = side.matrix
    return NS(class_id=cid, confidence=.95, pixel_x=k[0, 2]+error[0]*k[0, 0],
              pixel_y=k[1, 2]+error[1]*k[1, 1], orientation_valid=False)


def stream(node, ids=(), error=(0., 0.)):
    for camera in ('down_left', 'down_right'):
        node.feed(camera, [detection(node, cid, camera, error) for cid in ids])


@pytest.mark.parametrize('key,default', [('find', 15.), ('check', 5.), ('servo', 30.), ('loss', 5.)])
def test_timeout_aliases_and_validation(key, default):
    canonical = 'horizontal_servo_timeout' if key == 'servo' else key+'_timeout'
    assert _validate_params('26rb_grab_ball_ring', {key: {'timeout': default}}) == {canonical: default}
    for invalid in (0., -1., float('nan'), float('inf'), True):
        with pytest.raises(ConfigError):
            _validate_params('26rb_grab_ball_ring', {key: {'timeout': invalid}})


def test_loss_transitions_and_reappearance_do_not_reset_deadline():
    w = PickupWindow(100., 15., 30., 5.)
    w.seen = True
    assert w.transition(100., True, False)
    w.transition(101., False, False)
    assert w.mode == 'target'
    w.transition(105.99, False, False)
    assert w.mode == 'target'
    w.transition(106., False, False)
    assert w.mode == 'frame'
    w.transition(111., False, False)
    assert w.mode == 'return'
    w.transition(112., True, False)
    assert w.mode == 'target' and w.deadline == 130.


def test_find_skips_after_fifteen_seconds_without_target(rig):
    node, task = rig
    node.hook = lambda: stream(node, (0,))
    start = node.clock
    assert task._align_target('golf', task.ball, start) == 'unobserved'
    assert node.clock-start == pytest.approx(15.)
    assert node.servos == []
    assert np.allclose(node.velocity, 0.)


def test_target_has_priority_without_waiting_for_frame(rig):
    node, task = rig
    node.hook = lambda: stream(node, (7,))
    assert task._align_target('golf', task.ball, node.clock) == 'ready'
    assert node.clock < 101.
    assert task.state.pickup.golf_camera_pose is not None
    assert not any('frame' in call[4].get('task_context', '') for call in node.calls if call[0] == 'action')


def test_seen_once_survives_find_expiry_and_loss_return(rig):
    node, task = rig
    start = node.clock
    def images():
        stream(node, (7,) if node.clock-start < 1. else (), error=(.2, 0.))
    node.hook = images
    assert task._align_target('golf', task.ball, start) == 'unobserved'
    assert node.clock-start == pytest.approx(30.)
    assert any('进入return' in line for line in node.logs)


@pytest.mark.parametrize('recorded', [None, (-1., 1., .2, 0.)])
def test_lost_frame_uses_recorded_or_configured_closed_loop_position(rig, recorded):
    node, task = rig
    task.state.update_pickup(last_servo_pose=recorded)
    task.p['collection_frame_position'] = [-2., 2., .6]
    node.hook = lambda: stream(node)
    task._align_target('golf', task.ball, node.clock)
    moves = [c for c in node.calls if c[0] == 'velocity' and np.linalg.norm(c[2][:2]) > 0]
    assert moves and moves[0][1] >= 105.
    assert moves[0][2][0] < 0 and moves[0][2][1] > 0
    assert max(np.linalg.norm(c[2][:2]) for c in moves) <= task.ball._max_xy_speed+1e-9


def test_target_interrupts_closed_loop_return(rig):
    node, task = rig
    start = node.clock
    node.hook = lambda: stream(node, (7,) if node.clock-start > 6. else ())
    assert task._align_target('golf', task.ball, start) == 'ready'
    assert node.clock-start < 8.
    assert any('进入return' in line for line in node.logs)


def test_timeout_keeps_visible_target_residual_for_claw_alignment(rig):
    node, task = rig
    node.hook = lambda: stream(node, (7,), error=(.15, -.1))
    start = node.clock
    assert task._align_target('golf', task.ball, start) == 'ready'
    assert node.clock-start == pytest.approx(30.)
    target, pose, camera = task._targets['golf']
    centred = np.array(pose[:2])+node.camera_extrinsics[camera].translation[:2]
    assert np.linalg.norm(np.array(target)-centred) > .05
    task._align_claw('golf', task.ball, pose[5])
    claw = node.camera_extrinsics_provider.lookup_transform('base_link', 'disc_claw_link').translation
    assert node.pose[:2]+[claw.x, claw.y] == pytest.approx(target)


@pytest.mark.parametrize('mode', ['empty', 'no_frame', 'missing_right', 'gap', 'brief', 'overflow'])
def test_check_requires_new_continuous_dual_eye_observations(rig, mode):
    node, task = rig
    # Cached pre-return messages must never start a regrasp.
    stream(node, (7,))
    start = node.clock
    def images():
        if mode == 'gap' and 1. < node.clock-start < 2.:
            return
        for camera in ('down_left', 'down_right'):
            if mode == 'missing_right' and camera == 'down_right':
                continue
            if mode == 'brief' and node.clock-start < .3:
                node.feed(camera, [detection(node, 7, camera)])
            if mode == 'overflow':
                for _ in range(260):
                    node.feed(camera, [])
            node.feed(camera, [] if mode == 'no_frame' else [detection(node, 0, camera)])
    node.hook = images
    result = task._align_target('golf', task.ball, start, checking=True)
    assert result == ('absent' if mode in ('empty', 'no_frame') else 'unobserved')
    assert node.clock-start == pytest.approx(30. if mode == 'brief' else 5.)


def test_check_reacquires_current_target_without_frame_centering(rig):
    node, task = rig
    node.hook = lambda: stream(node, (7,))
    assert task._align_target('golf', task.ball, node.clock, checking=True) == 'ready'
    assert node.clock < 101.


def test_return_uses_target_camera_pose_without_visual_servo(rig):
    node, task = rig
    anchor = (.8, -.4, .2, 35.)
    task.state.update_pickup(golf_camera_pose=anchor)
    task.ball._servo_horizontally = lambda *a, **k: pytest.fail('return must not visually servo')
    node.pose[2] = .44
    task._return_frame(task.ball)
    assert node.pose[[0, 1, 2, 5]] == pytest.approx(anchor)
    assert [c[3] for c in node.calls if c[0] == 'action'] == ['z', 'xyrz']


@pytest.mark.parametrize('axis', [None, 60.])
def test_ring_turn_and_claw_alignment_are_one_action_without_reservo(rig, axis):
    node, task = rig
    node.pose[5] = 17.
    task._targets['ring'] = ((1., 2.), tuple(node.pose), 'down_left')
    task._ring_axis_at_alignment = axis
    task.ring._servo_horizontally = lambda *a, **k: pytest.fail('no ring reservo')
    assert task._orient_and_align_ring()
    actions = [c for c in node.calls if c[0] == 'action']
    assert len(actions) == 1 and actions[0][3] == 'xyzrz'
    assert node.pose[5] == pytest.approx(17. if axis is None else 60.)
    tf = node.camera_extrinsics_provider.lookup_transform('base_link', 'hairpin_claw_link').translation
    claw = mod.body_to_world_rotation(node.pose) @ [tf.x, tf.y, tf.z]
    assert node.pose[:2]+claw[:2] == pytest.approx([1., 2.])


def test_find_uses_bline_timestamp_and_ring_starts_on_entry(rig, monkeypatch):
    node, task = rig
    monkeypatch.setattr(mod, 'CollectionFrameSearch', lambda *a: NS(first_down_seen_at=98., execute=mod.TaskOutcome.ok))
    starts=[]
    def align(kind, controller, started, **kwargs):
        starts.append((kind, started))
        node.clock += 1.
        return 'unobserved'
    task._align_target=align
    task.execute()
    assert starts == [('golf', 98.), ('ring', 101.)]
    assert task.state.pickup.first_frame_seen_at == 98.


def test_check_starts_after_return_and_regrasp_preserves_attempt_limit(rig):
    node, task = rig
    starts=[]
    results=iter(['ready', 'ready', 'absent'])
    def align(kind, controller, started, **kwargs):
        starts.append((started, kwargs['checking']))
        task.state.update_pickup(golf_camera_pose=(.5, 0., .2, 0.))
        return next(results)
    task._align_target=align
    task._mechanical_attempt=lambda *a: True
    task._return_frame=lambda *a: setattr(node, 'clock', node.clock+2.)
    task._run_target('golf', task.ball)
    assert starts == [(100., False), (102., True), (104., True)]
    assert task.state.pickup.golf_attempts == 2 and task.state.pickup.golf_status == 'success'


def test_motion_inhibition_is_not_a_timeout_grab(rig):
    node, task = rig
    node.hook=lambda: stream(node, (7,), error=(.2, 0.))
    node._send_body_velocity=lambda *a, **k: (False, 'motion inhibited')
    with pytest.raises((RuntimeError, mod.PickupFailure)):
        task._align_target('golf', task.ball, node.clock)
    assert node.servos == [] and task._targets == {}


def test_stage_lights_do_not_consume_simulated_motion_time(rig):
    node, task = rig
    flashes=[]
    node._flash_task_light=lambda color,count,label,**k: flashes.append((node.clock,count)) or True
    start=node.clock
    node.hook=lambda: stream(node, (7,) if node.clock-start > .5 else (0,))
    assert task._align_target('golf', task.ball, start)=='ready'
    task._align_claw=lambda *a: None
    task._descend_to_depth=lambda *a: True
    task.ball._prepare_claw=lambda: True
    task.ball._wait_pre_descent_settle=lambda: True
    task._mechanical_attempt('golf',task.ball)
    assert [count for stamp,count in flashes] == [1,2,3]
    assert node.clock-start < 2.


def test_first_down_frame_timestamp_captured_only_during_bline():
    from test_runtime_compatibility import TaskNode
    from uv_task.collection_frame_search import CollectionFrameSearch
    node = TaskNode()
    task = CollectionFrameSearch(node, {})
    now = [100.]
    task._now = lambda: now[0]
    message = NS(camera_name='down_left', detections=[NS(class_id=7, confidence=.9, pixel_x=320., pixel_y=240.)],
                 header=NS(stamp=NS(sec=100, nanosec=0)), capture_id=1)
    task._detection_cb(message)
    assert task.first_down_seen_at is None
    task._bline_active = True
    now[0] = 100.1
    message.capture_id = 2
    task._detection_cb(message)
    assert task.first_down_seen_at == 100.1
    now[0] = 100.2
    message.capture_id = 3
    task._detection_cb(message)
    assert task.first_down_seen_at == 100.1


def test_real_async_light_pattern_replaces_and_cancels_without_waiting(monkeypatch):
    import threading
    import time
    from types import MethodType
    from uv_task.task_runner import TaskRunnerNode
    from uv_task import task_runner
    monkeypatch.setattr(task_runner.rclpy, 'ok', lambda: True)
    messages = []
    node = NS(stopped=False, LIGHT_OFF=0, _light_lock=threading.RLock(),
              _light_animation_cancel=None, _light_phase_color=3, _light_state=3,
              pub_light=NS(publish=lambda msg: messages.append(msg.data)),
              get_logger=lambda: NS(info=lambda *a: None, warning=lambda *a: None))
    for name in ('_publish_light_locked', '_cancel_light_animation_locked',
                 '_cancel_task_light_animation', '_flash_task_light'):
        setattr(node, name, MethodType(getattr(TaskRunnerNode, name), node))
    started = time.monotonic()
    assert node._flash_task_light(2, 2, 'target', pulse_seconds=10.)
    first = node._light_animation_cancel
    assert node._flash_task_light(2, 3, 'claw', pulse_seconds=10.)
    second = node._light_animation_cancel
    assert first.is_set() and not second.is_set()
    node._cancel_task_light_animation(restore=False)
    assert second.is_set() and node._light_animation_cancel is None
    assert time.monotonic()-started < 1.
    assert messages == [2, 2]
