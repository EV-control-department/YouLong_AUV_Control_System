"""Three trips and same-class rings, using virtual cameras/actuators only."""
from importlib import import_module
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

from test_26rb_grab_ball_ring import Node, mod
from test_pickup_windows import detection
from uv_task.config_loader import ConfigError, _validate_params, load_mission_or_task, load_task
from uv_task.ring_observation import RingTargetLock, StableRingCounts, compare_ring_counts
from uv_task.task_outcome import TaskOutcome


@pytest.fixture
def ring(monkeypatch):
    node = Node()
    monkeypatch.setattr(mod.time, 'monotonic', lambda: node.clock)
    monkeypatch.setattr(mod.time, 'sleep', node.sleep)
    monkeypatch.setattr(mod, 'CollectionFrameSearch', lambda *args: NS(execute=TaskOutcome.ok))
    params = load_task(Path(__file__).parents[1]/'config/tasks/26rb_grab_ball_ring.yaml')[0]['params']
    params.update(pickup_order=['ring'], ring_confirmation='count_decrease',
                  work_depth_m=.2, ring_grab_depth_m=.44)
    task = mod.RB26GrabBallRingTask(node, params)
    task._ring_lock = RingTargetLock(8, 2., task._ring_observation)
    return node, task


def boxed(node, camera, error=(0., 0.), confidence=.9, axis=0.):
    det = detection(node, 8, camera, error)
    det.confidence = confidence
    det.bbox_x1, det.bbox_x2 = det.pixel_x-2., det.pixel_x+2.
    det.bbox_y1, det.bbox_y2 = det.pixel_y-2., det.pixel_y+2.
    det.orientation_valid, det.orientation_quality, det.orientation_axis_deg = True, .95, axis
    return det


def images(node, errors=((0., 0.),), *, capture=None, include_frame=True):
    for camera in ('down_left', 'down_right'):
        dets = [boxed(node, camera, error) for error in errors]
        if include_frame:
            frame = detection(node, 0, camera)
            cfg = node.camera_configs['down']
            frame.bbox_x1 = frame.bbox_y1 = 10.
            frame.bbox_x2, frame.bbox_y2 = cfg.width-10., cfg.height-10.
            dets.insert(0, frame)
        node.feed(camera, dets)
        if capture is not None:
            node._down_detections[camera][1].capture_id = capture


def selected(task):
    return task._fresh_detection(task.ring._golf_class_id, task.ring)


def test_ring_only_never_requests_ball_classes(ring):
    node, task = ring
    requested = []
    def lookup(name, **kwargs):
        requested.append(name)
        assert name not in ('yellow_golf', 'pink_golf')
        return {'red_ring': 8, 'collection_frame_down': 0}.get(name)
    node._model_mapping = NS(model_class_id=lookup)
    task = mod.RB26GrabBallRingTask(node, task.p)
    assert task.controllers == {'ring': task.ring}
    assert task.ball is task.ring and not task.colored_pickup
    assert requested == ['red_ring', 'collection_frame_down']
    assert task.ring._detection_timeout == 2.


def test_nearest_bbox_ignores_confidence_and_does_not_require_overlap(ring):
    node, task = ring
    first = boxed(node, 'down_left', (.02, 0.))
    node.feed('down_left', [first, boxed(node, 'down_left', (.4, 0.), confidence=1.)])
    assert selected(task)[1] is first
    node.clock += .1
    near = boxed(node, 'down_left', (.12, 0.), confidence=.1)
    far = boxed(node, 'down_left', (.3, 0.), confidence=1.)
    assert near.bbox_x1 > first.bbox_x2
    node.feed('down_left', [far, near])
    assert selected(task)[1] is near


def test_empty_eye_waits_exactly_two_seconds_then_requires_new_frames(ring):
    node, task = ring
    images(node, ((.1, 0.),))
    assert selected(task)[0] == 'down_left'
    node.clock = 100.1
    node.feed('down_left', [])
    node.feed('down_right', [boxed(node, 'down_right')])
    assert selected(task) == (None, None, None)
    node.clock = 101.99
    node.pose_received_at = node.clock
    node.feed('down_right', [boxed(node, 'down_right')])
    assert selected(task)[0] is None
    assert task._ring_lock.active == 'down_left'
    node.clock = 102.
    assert selected(task)[0] is None
    assert task._ring_lock.active is None
    node.clock = 102.01
    node.pose_received_at = node.clock
    node.feed('down_right', [boxed(node, 'down_right')])
    assert selected(task)[0] == 'down_right'
    assert task._ring_lock.generation == 2


def test_detection_resumes_within_two_seconds_and_restarts_loss_clock(ring):
    node, task = ring
    images(node, ((.1, 0.),))
    selected(task)
    node.clock = 100.1
    node.feed('down_left', [])
    node.feed('down_right', [boxed(node, 'down_right')])
    assert selected(task)[0] is None
    node.clock = 101.
    recovered = boxed(node, 'down_left', (.12, 0.))
    node.feed('down_left', [recovered])
    assert selected(task)[1] is recovered
    node.clock = 102.9
    node.feed('down_left', [])
    assert selected(task)[0] is None and task._ring_lock.active == 'down_left'
    node.clock = 103.
    selected(task)
    assert task._ring_lock.active is None


@pytest.mark.parametrize('source_identity', ['capture', 'stamp'])
def test_replayed_source_frames_cannot_refresh_lock_or_world_pose(ring, source_identity):
    node, task = ring
    images(node, capture=12 if source_identity == 'capture' else None)
    if source_identity == 'stamp':
        for _, msg in node._down_detections.values():
            msg.header = NS(stamp=NS(sec=100, nanosec=1))
    selected(task)
    saved = task._ring_lock._eyes['down_left']['visible']
    node.clock = 101.
    node.pose[0] += 2.
    images(node, capture=12 if source_identity == 'capture' else None)
    if source_identity == 'stamp':
        for _, msg in node._down_detections.values():
            msg.header = NS(stamp=NS(sec=100, nanosec=1))
    selected(task)
    assert task._ring_lock._eyes['down_left']['visible'] is saved
    assert task._ring_lock.new_observations == []
    node.clock = 102.
    selected(task)
    assert task._ring_lock.active is None


def test_direction_collection_uses_selected_ring_not_highest_confidence(ring, monkeypatch):
    node, task = ring
    def feed():
        node.feed('down_left', [boxed(node, 'down_left', (.01, 0.), confidence=.2, axis=15.),
                                boxed(node, 'down_left', (.4, 0.), confidence=.99, axis=90.)])
    node.hook = feed
    feed()
    monkeypatch.setattr(task, '_ring_world_axis', lambda camera, det, pose: det.orientation_axis_deg)
    selected(task)
    assert task._collect_ring_axis() == pytest.approx(15.)
    assert all(angle == 15. for _, angle in task.samples)


def test_blind_uses_best_observation_from_either_eye_with_its_pose_and_axis(ring, monkeypatch):
    node, task = ring
    task.ring._hold_seconds = 1000.
    saved = []
    make = task._ring_observation
    def observe(*args):
        obs = make(*args)
        if obs.camera == 'down_right' and abs(obs.error[0]-.01) < 1e-6:
            saved.append(obs)
        return obs
    monkeypatch.setattr(task, '_ring_observation', observe)
    start = node.clock
    node.feed('down_left', [boxed(node, 'down_left', (.2, 0.), axis=0.)])
    def feed():
        node.feed('down_left', [boxed(node, 'down_left', (.2, 0.), axis=0.)])
        node.feed('down_right', [boxed(node, 'down_right',
                  (.01 if node.clock-start < 1. else .2, 0.), axis=90.)])
    node.hook = feed
    assert task._align_target('ring', task.ring, start) == 'ready'
    assert node.clock-start == pytest.approx(30.)
    assert saved
    best = saved[-1]
    world, pose, camera = task._targets['ring']
    assert camera == 'down_right' and world == best.world and pose == best.pose
    assert task._blind_targets['ring'] == best.axis_sample
    assert 'ring' in task._blind_grabs_used


def test_relock_resets_micro_eligibility_without_extending_servo_budget(ring):
    node, task = ring
    task.ring._hold_seconds = 1000.
    start = node.clock
    def feed():
        node.feed('down_left', [boxed(node, 'down_left', (.2, 0.))]
                  if node.clock-start < .5 else [])
        node.feed('down_right', [boxed(node, 'down_right', (.6, 0.))])
    node.hook = feed
    assert task._align_target('ring', task.ring, start) == 'timeout'
    assert node.clock-start == pytest.approx(30.)
    assert task._ring_lock.generation == 2
    assert 'ring' not in task._blind_grabs_used
    missing = [call for call in node.calls if call[0] == 'velocity' and 100.5 < call[1] < 102.4]
    assert missing and all(np.allclose(call[2][:2], 0.) for call in missing)


@pytest.mark.parametrize('before,after,expected', [
    ((2, 2), (1, 1), 'success'), ((1, 1), (0, 0), 'success'),
    ((2, 1), (1, 0), 'success'), ((1, 0), (0, 0), 'success'),
    ((2, 2), (2, 2), 'retry'), ((1, 1), (1, 1), 'retry'),
    ((2, 2), (1, 2), 'unknown'), ((2, 2), (0, 0), 'unknown'),
    ((1, 0), (0, 1), 'unknown'), ((0, 0), (0, 0), 'unknown')])
def test_per_eye_count_confirmation(before, after, expected):
    eyes = ('down_left', 'down_right')
    assert compare_ring_counts(dict(zip(eyes, before)), dict(zip(eyes, after))) == expected


def test_duplicate_frames_do_not_confirm_stable_counts():
    counts = StableRingCounts(.5, 2.)
    msg = NS(capture_id=8)
    for camera in ('down_left', 'down_right'):
        counts.observe(camera, 100., msg, 2)
        counts.observe(camera, 101., msg, 2)
    assert counts.result(101.) is None
    for camera in ('down_left', 'down_right'):
        counts.observe(camera, 101.1, NS(capture_id=9), 2)
    assert counts.result(101.1) == {'down_left': 2, 'down_right': 2}
    assert counts.result(103.2) is None


def test_counts_require_visible_frame_and_same_pose(ring):
    node, task = ring
    images(node, ((0., 0.), (.1, 0.)))
    msg = node._down_detections['down_left'][1]
    assert task._ring_count('down_left', msg) == 2
    msg.detections[0].bbox_x1 = 0.
    assert task._ring_count('down_left', msg) is None
    node.hook = lambda: images(node, ((0., 0.),))
    assert task._observe_ring_counts(1., (10., 10., .2, 0.)) is None


def test_one_removed_of_two_stops_after_one_real_mechanical_attempt(ring):
    node, task = ring
    remaining = [2]
    node.hook = lambda: images(node, ((0., 0.), (.15, 0.))[:remaining[0]])
    servo = node.set_servo
    closed = [False]
    def send(angle, label, servo_id=1):
        servo(angle, label, servo_id)
        if servo_id == 2 and angle == 50. and not closed[0]:
            remaining[0] -= 1
            closed[0] = True
    node.set_servo = send
    outcome = task.execute()
    assert outcome, outcome.message
    assert task.state.pickup.ring_attempts == 1 and task.state.pickup.ring_status == 'success'
    assert remaining[0] == 1
    assert [angle for _, sid, angle in node.servos if sid == 2] == [210.]*3+[50.]*3
    assert not any(sid == 1 for _, sid, _ in node.servos)


@pytest.mark.parametrize('confirmation,attempts,expected_status', [
    ('unknown', 1, 'pending'), ('retry', 3, 'failed'), ('success', 1, 'success')])
def test_count_result_controls_retry_and_unconfirmed_keeps_claw_closed(ring, confirmation, attempts, expected_status):
    node, task = ring
    calls = []
    task._align_target = lambda *a, **kw: 'ready'
    task._observe_ring_counts = lambda *a, **kw: {'down_left': 2, 'down_right': 2}
    task._mechanical_attempt = lambda *a: calls.append(a[0]) or True
    task._return_frame = lambda *a: None
    task._check_ring_grab = lambda *a: confirmation
    task._run_target('ring', task.ring)
    assert calls == ['ring']*attempts
    assert task.state.pickup.ring_attempts == attempts
    assert task.state.pickup.ring_status == expected_status
    assert ('ring' in task._unconfirmed_targets) == (confirmation == 'unknown')


def test_first_trip_defers_ring_and_continues_both_balls(ring):
    node, previous = ring
    params = dict(previous.p, pickup_order=['ring', 'yellow', 'red'],
                  ring_confirmation='deferred', ring_max_attempts=1)
    ids = {'red_ring': 8, 'collection_frame_down': 0, 'yellow_golf': 11, 'pink_golf': 7}
    node._model_mapping = NS(model_class_id=lambda name, **kw: ids.get(name))
    task = mod.RB26GrabBallRingTask(node, params)
    entered = []
    task._align_target = lambda kind, controller, started, **kw: 'absent' if kw.get('checking') else 'ready'
    task._mechanical_attempt = lambda kind, controller: entered.append(kind) or True
    task._return_frame = lambda *args: None
    outcome = task.execute()
    assert outcome, outcome.message
    assert entered == ['ring', 'yellow', 'red']
    assert task.state.pickup.ring_attempts == 1 and task.state.pickup.ring_status == 'pending'
    assert task.state.pickup.yellow_status == task.state.pickup.red_status == 'success'
    assert '未确认' in outcome.message


def test_healthy_empty_ring_trip_completes_without_attempt(ring):
    node, task = ring
    node.hook = lambda: images(node, ())
    outcome = task.execute()
    assert outcome, outcome.message
    assert task.state.pickup.ring_attempts == 0 and node.servos == []
    assert task.state.pickup.ring_status == 'pending'


@pytest.mark.parametrize('params', [
    {'ring_confirmation': 'bad'}, {'ring_confirmation': 'deferred', 'ring_max_attempts': 2},
    {'ring_relock_timeout': 0.}, {'ring_relock_timeout': float('nan')},
    {'ring_target_lock_enabled': 1}])
def test_new_config_rejects_invalid_modes_and_timing(params):
    with pytest.raises(ConfigError):
        _validate_params('26rb_grab_ball_ring', params)


@pytest.mark.parametrize('name', ['robocup_26.yaml', 'robocup_26 copy.yaml'])
def test_both_missions_have_fixed_three_trips_and_direct_gate_exit(name):
    root = Path(__file__).parents[1]
    tasks = load_mission_or_task(root/'config/missions'/name)
    names = [t['name'] for t in tasks]
    assert names == ['start', '26rb_hit_balls']+['26rb_grab_ball_ring', '26rb_drop_ball_target_rack']*3+['26rb_gate_task', 'return_origin']
    grabs = [t for t in tasks if t['name'] == '26rb_grab_ball_ring']
    assert [t['params']['pickup_order'] for t in grabs] == [['ring', 'yellow', 'red'], ['ring'], ['ring']]
    assert [t['params']['ring_max_attempts'] for t in grabs] == [1, 3, 3]
    assert [t['params']['ring_confirmation'] for t in grabs] == ['deferred', 'count_decrease', 'count_decrease']
    assert all(t['params']['ring_relock_timeout'] == 2. for t in grabs)
    assert all(not t.get('initial_pose') for t in grabs[1:])
    assert all(not profile.get('skip_next') for t in grabs for profile in t.get('on_failure', {}).values())
    drops = [t for t in tasks if t['name'] == '26rb_drop_ball_target_rack']
    assert all(t['params']['release_at_preset_on_no_frame'] for t in drops)
    gate = tasks[-2]
    initial = gate['initial_pose']
    assert len(initial) == 1 and initial[0]['target'] == [0., 3., .1, 0.]


def test_frame_freshness_remains_point_eight_seconds(ring):
    node, task = ring
    images(node)
    selected(task)
    node.clock += .9
    assert selected(task)[1] is not None
    assert task._fresh_detection(0, task.ring)[1] is None


def test_post_servo_direction_relock_cannot_replace_frozen_target_axis(ring, monkeypatch):
    node, task = ring
    monkeypatch.setattr(task, '_ring_world_axis', lambda camera, det, pose: det.orientation_axis_deg)
    node.feed('down_left', [boxed(node, 'down_left', axis=15.)])
    selected(task)
    task._last_ring_axis = (100., 15.)
    task.ring_orientation_seconds = 3.
    def images_after_loss():
        node.feed('down_left', [])
        node.feed('down_right', [boxed(node, 'down_right', (.1, 0.), axis=90.)])
    node.hook = images_after_loss
    assert task._collect_ring_axis() == pytest.approx(15.)
    assert task._ring_lock.generation == 2
    assert not task.samples


def test_recovery_failure_pauses_new_multi_trip_mode(ring):
    node, task = ring
    task._align_target = lambda *a, **kw: 'ready'
    task._observe_ring_counts = lambda *a, **kw: None
    task._mechanical_attempt = lambda *a: True
    def failed(*a):
        raise mod.PickupFailure('回位失败')
    task._return_frame = failed
    outcome = task.execute()
    assert not outcome and outcome.failure_code.endswith('.recovery')
    assert node.stopped


@pytest.mark.parametrize('failed_rounds', [set(), {0}, {2}, {0, 1, 2}])
def test_production_scheduler_keeps_all_three_trips_on_normal_ring_absence(failed_rounds):
    from types import MethodType
    from uv_task.task_runner import TaskRunnerNode
    tasks = load_mission_or_task(Path(__file__).parents[1]/'config/missions/robocup_26.yaml')
    entered = []
    ring_index = [0]
    log = NS(info=lambda *a: None, warn=lambda *a: None, error=lambda *a: None)
    node = NS(tasks=tasks, _cmd_x=0., _cmd_y=0., _cmd_z=.2, _cmd_yaw=0., get_logger=lambda: log)
    def execute(name, params, **kw):
        entered.append(name)
        if name == '26rb_grab_ball_ring':
            index = ring_index[0]
            ring_index[0] += 1
            if index in failed_rounds:
                return TaskOutcome.failed('26rb_grab_ball_ring.no_frame', '普通未发现')
        return TaskOutcome.ok()
    node._execute_task = execute
    node._select_failure_override = MethodType(TaskRunnerNode._select_failure_override, node)
    TaskRunnerNode.run_task_list(node)
    assert entered == [t['name'] for t in tasks]
    assert entered.count('26rb_drop_ball_target_rack') == 3 and not node.stopped


def test_failed_relock_retains_saved_world_point_for_one_blind_grab(ring):
    node, task = ring
    task.ring._hold_seconds = 1000.
    start = node.clock
    def feed():
        images(node, ((.2, 0.),) if node.clock-start < .5 else ())
    node.hook = feed
    assert task._align_target('ring', task.ring, start) == 'ready'
    assert node.clock-start == pytest.approx(30.)
    assert task._ring_lock.active is None and task._ring_lock.generation == 1
    assert 'ring' in task._blind_grabs_used
    observed_pose = task._targets['ring'][1]
    assert observed_pose != task._pose()
    start = node.clock
    assert task._align_target('ring', task.ring, start) == 'timeout'
    assert task._blind_grabs_used == {'ring'}


def test_new_ring_observations_require_fresh_measured_odometry(ring):
    node, task = ring
    node.clock += 1.1
    images(node)
    with pytest.raises(ValueError, match='odom'):
        selected(task)


def test_position_hold_pause_does_not_expire_an_eye_with_new_valid_data(ring):
    node, task = ring
    node.feed('down_left', [boxed(node, 'down_left', (.1, 0.))])
    selected(task)
    node.clock += 3.
    node.pose_received_at = node.clock
    node.feed('down_left', [boxed(node, 'down_left', (.12, 0.))])
    node.feed('down_right', [boxed(node, 'down_right')])
    assert selected(task)[0] == 'down_left'
    assert task._ring_lock.generation == 1
