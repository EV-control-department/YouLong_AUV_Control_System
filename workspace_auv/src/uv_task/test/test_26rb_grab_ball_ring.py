"""Virtual camera/odom/actuator acceptance checks; no hardware is started."""
from collections import deque
from dataclasses import FrozenInstanceError
from importlib import import_module
from pathlib import Path
import math
import threading
from types import SimpleNamespace as NS
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from uv_camera.camera_config import load_camera_config
from uv_task.config_loader import ConfigError, _validate_params, load_task
from uv_task.down_camera_servo import body_to_world_rotation
from uv_task.task_outcome import TaskOutcome
from uv_task.task_state import TaskState

mod = import_module('uv_task.26rb_grab_ball_ring')


class Node:
    LIGHT_OFF, LIGHT_RED, LIGHT_GREEN, LIGHT_YELLOW = 0, 1, 2, 3
    stopped = False

    def __init__(self):
        self.state = TaskState()
        self.pose = np.array([.5, 0., .2, 0., 0., 0.])
        self.clock = 100.
        self.velocity = np.zeros(4)
        self._cmd_x, self._cmd_y, self._cmd_z, self._cmd_yaw = .5, 0., .2, 0.
        self._perception_lock = threading.RLock()
        self._down_detections = {}
        self._down_detection_sequence = 0
        self._down_detection_events = deque(maxlen=256)
        self.calls, self.servos, self.logs = [], [], []
        self._model_mapping = NS(model_class_id=lambda name, **kwargs:
            {'collection_frame_down': 0, 'pink_golf': 7, 'red_ring': 8}.get(name))
        self.camera_configs = {'down': load_camera_config('down', 'sim')}
        self.camera_extrinsics = {camera: NS(
            translation=np.array([-.13, offset, .0645]),
            optical_to_body=np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]]))
            for camera, offset in [('down_left', -.030586), ('down_right', .030586)]}
        source = Path(__file__).parents[2]/'auv_description/urdf/auv.urdf'
        transforms = {joint.find('child').get('link'): list(map(float, joint.find('origin').get('xyz').split()))
                      for joint in ET.parse(source).getroot().findall('joint') if joint.find('origin') is not None}
        def lookup(base, child):
            xyz = transforms[child]
            return NS(translation=NS(x=xyz[0], y=xyz[1], z=xyz[2]))
        self.camera_extrinsics_provider = NS(base_frame='base_link', lookup_transform=lookup)
        self.removed = set()
        self.hook = self.scene

    def get_logger(self):
        return NS(info=self.logs.append, warning=self.logs.append, error=self.logs.append)

    def _latest_robot_pose(self, require_measured=False):
        return tuple(self.pose)

    def _ensure_camera_extrinsics(self):
        return True

    def _format_motion_context(self, label):
        return label

    def set_light(self, *args, **kwargs):
        pass

    def light_off(self):
        pass

    def _set_task_phase_light(self, *args, **kwargs):
        pass

    def set_servo(self, angle, label, servo_id=1):
        self.servos.append((self.clock, servo_id, angle))
        if servo_id == 2 and angle == 90.:
            self.removed.add(8)

    def _send_body_velocity(self, *args, **kwargs):
        self.velocity[:3] = args if args else [0., 0., kwargs.get('vertical_mps', 0.)]
        self.velocity[3] = kwargs.get('yaw_rate_deg_s', 0.)
        self.calls.append(('velocity', self.clock, self.velocity.copy(), kwargs))
        return True, ''

    def _send_action_goal(self, command, target, axes, **kwargs):
        self.velocity[:] = 0.
        self.calls.append(('action', command, list(target), axes, kwargs))
        if command == mod.BasicMotion.Goal.SET:
            linear_axes = axes.replace('rz', '') if axes else 'xyz'
            for index, axis in enumerate('xyz'):
                if axis in linear_axes:
                    self.pose[index] = target[index]
            if not axes or 'rz' in axes:
                self.pose[5] = target[3]
        else:
            self.pose[:3] += body_to_world_rotation(self.pose) @ np.array(target[:3])
        return True, ''

    def sleep(self, dt):
        self.pose[:3] += body_to_world_rotation(self.pose) @ self.velocity[:3]*dt
        self.pose[5] = mod.wrap(self.pose[5]+self.velocity[3]*dt)
        self.clock += dt
        self.hook()

    def feed(self, camera, detections, stamp=None):
        received = self.clock if stamp is None else stamp
        message = NS(detections=detections)
        self._down_detections[camera] = (received, message)
        self._down_detection_sequence += 1
        self._down_detection_events.append((self._down_detection_sequence, received, camera, message))

    def scene(self):
        objects = {0: [.5, 0., .6], 7: [.55, -.08, .6], 8: [.45, .08, .6]}
        for camera, extrinsic in self.camera_extrinsics.items():
            calibration = self.camera_configs['down'].side('left' if camera.endswith('left') else 'right')
            rotation = body_to_world_rotation(self.pose)
            origin = self.pose[:3]+rotation @ extrinsic.translation
            def pixel(point):
                ray = extrinsic.optical_to_body.T @ rotation.T @ (np.asarray(point)-origin)
                raw = calibration.matrix @ ray
                return raw[:2]/raw[2]
            detections = []
            for cid, point in objects.items():
                if cid in self.removed:
                    continue
                px, py = pixel(point)
                axis = np.array([.025, math.sqrt(3.)*.025, 0.])
                difference = pixel(np.asarray(point)+axis)-pixel(np.asarray(point)-axis)
                detections.append(NS(class_id=cid, confidence=.95, pixel_x=px, pixel_y=py,
                    bbox_x1=px-80, bbox_y1=py-80, bbox_x2=px+80, bbox_y2=py+80,
                    orientation_valid=cid == 8,
                    orientation_axis_deg=math.degrees(math.atan2(difference[1], difference[0])) % 180.,
                    orientation_quality=.95))
            self.feed(camera, detections)


@pytest.fixture
def rig(monkeypatch):
    node = Node()
    monkeypatch.setattr(mod.time, 'monotonic', lambda: node.clock)
    monkeypatch.setattr(mod.time, 'sleep', node.sleep)
    monkeypatch.setattr(mod, 'CollectionFrameSearch', lambda *args: NS(execute=TaskOutcome.ok))
    config = Path(__file__).parents[1]/'config/tasks/26rb_grab_ball_ring.yaml'
    params = load_task(config)[0]['params']
    params.update(golf_max_attempts=2, ring_max_attempts=3)
    return node, mod.RB26GrabBallRingTask(node, params)


@pytest.mark.parametrize('axis,xy,start,current,expected', [
    (0, (2, 0), (0, 0), 80, 0), (0, (-2, 0), (0, 0), 0, -180),
    (90, (0, 2), (0, 0), 0, 90), (90, (0, -2), (0, 0), 0, -90),
    (45, (1, 1), (1, 1), -140, -135), (0, (2, 5), (4, 5), 0, -180),
])
def test_heading_is_parallel_to_ring_plane_and_faces_away(axis, xy, start, current, expected):
    assert mod.choose_away_heading(axis, xy, start, current) == pytest.approx(expected)


@pytest.mark.parametrize('golf,ring,code', [
    ([False, False], [True], 'golf_exhausted'),
    ([True], [False, False, False], 'ring_exhausted'),
    ([False, False], [False, False, False], 'both_exhausted'),
    ([False, True], [False, True], ''),
])
def test_separate_attempt_limits_and_frame_verification_order(rig, golf, ring, code):
    node, task = rig
    events = []
    outcomes = {'golf': iter(golf), 'ring': iter(ring)}
    def attempt(kind, controller):
        events.append(('attempt', kind))
        return True
    task._mechanical_attempt = attempt
    task._return_frame = lambda controller: events.append(('frame', 'golf' if controller is task.ball else 'ring'))
    def verify(class_id):
        kind = 'golf' if class_id == 7 else 'ring'
        events.append(('verify', kind))
        return next(outcomes[kind])
    task._verify = verify
    result = task.execute()
    assert bool(result) is (not code)
    if code:
        assert result.failure_code == '26rb_grab_ball_ring.'+code
    assert node.state.pickup.golf_attempts == len(golf)
    assert node.state.pickup.ring_attempts == len(ring)
    assert events == [event for kind, values in (('golf', golf), ('ring', ring))
                      for _ in values for event in (('attempt', kind), ('frame', kind), ('verify', kind))]


def test_frame_return_failure_stops_further_attempts(rig):
    node, task = rig
    task._mechanical_attempt = lambda *args: True
    def failed_return(*args):
        raise mod.PickupFailure('模拟frame回位失败')
    task._return_frame = failed_return
    result = task.execute()
    assert not result
    assert result.failure_code.endswith('.recovery')
    assert node.state.pickup.golf_attempts == 1
    assert node.state.pickup.ring_attempts == 0


@pytest.mark.parametrize('mode', ['absent', 'left_present', 'right_present', 'missing_right', 'stale', 'late_present', 'between_polls', 'overflow', 'no_frame'])
def test_frame_verification_requires_both_fresh_eyes_for_two_seconds(rig, mode):
    node, task = rig
    frame = NS(class_id=0, confidence=.9, pixel_x=0., pixel_y=0.)
    ball = NS(class_id=7, confidence=.9, pixel_x=0., pixel_y=0.)
    started = node.clock
    def stream():
        for camera in ('down_left', 'down_right'):
            if mode == 'missing_right' and camera.endswith('right'):
                continue
            items = [] if mode == 'no_frame' else [frame]
            if (mode == 'left_present' and camera.endswith('left')) or (mode == 'right_present' and camera.endswith('right')) or (mode == 'late_present' and node.clock-started >= 1.9):
                items = items+[ball]
            if mode == 'between_polls':
                node.feed(camera, items+[ball])
            if mode == 'overflow':
                for _ in range(260):
                    node.feed(camera, items)
            node.feed(camera, items, stamp=started-.1 if mode == 'stale' else None)
    node.hook = stream
    assert task._verify(7) is (mode == 'absent')
    assert node.clock-started == pytest.approx(2.)


def test_return_frame_disables_target_handoff(rig):
    node, task = rig
    # Deliberately stale anchor Z and a below-work-depth starting pose.
    node.state.update_pickup(frame_pose=(.5, 0., .7, 0.), depth=.2)
    node.pose[2] = .44
    task.ball._servo_depth = .2
    # Both targets remain visible throughout frame centering.
    task._return_frame(task.ball)
    assert task.ball._pending_golf_priority is None
    assert task.ball._aligned_camera in ('down_left', 'down_right')
    assert node.state.pickup.frame_pose[0] > .5
    actions = [call for call in node.calls if call[0] == 'action']
    assert actions[0][3] == 'z'
    assert actions[0][2] == pytest.approx([.5, 0., .2, 0.])
    assert actions[1][3] == 'xyrz'
    assert actions[1][2][2] == .2
    assert node.pose[2] == .2


def test_orientation_averages_modulo_180_and_rejects_stale_or_unstable_samples(rig):
    node, task = rig
    task.samples.extend([(node.clock-.3, 179.), (node.clock-.15, 1.), (node.clock, 0.)])
    assert min(task._axis_estimate(), 180-task._axis_estimate()) < .1
    node.clock += 1.
    assert task._axis_estimate() is None
    task.samples.clear()
    task.samples.extend([(node.clock, value) for value in (0., 30., 60.)])
    assert task._axis_estimate() is None


def test_missing_orientation_does_not_descend_or_close_claw(rig):
    node, task = rig
    task.ring._aligned_camera = 'down_left'
    task.state.update_pickup(depth=.2)
    ring = NS(class_id=8, confidence=.9, pixel_x=100., pixel_y=100.)
    node.hook = lambda: node.feed('down_left', [ring])
    assert not task._orient_and_align_ring()
    assert [(servo, angle) for _, servo, angle in node.servos] == [(2, 0.)]*3
    assert not any(call[0] == 'velocity' and call[2][2] != 0 for call in node.calls)


@pytest.mark.parametrize('work_depth', [.2, .18])
def test_full_combined_pickup_with_projected_cameras_and_gripper_tf(rig, monkeypatch, work_depth):
    node, task = rig
    task.work_depth = work_depth
    task.p['work_depth_m'] = work_depth
    node.pose[2] = .27
    searches = []
    def search(node, params):
        searches.append(params['search_cruise_depth_m'])
        return NS(execute=TaskOutcome.ok)
    monkeypatch.setattr(mod, 'CollectionFrameSearch', search)
    stages = []
    descend, restore = task._descend_to_depth, task._restore_work_depth
    def down(kind, controller):
        started = node.clock
        success = descend(kind, controller)
        assert success and node.pose[2] == pytest.approx(.44, abs=1e-6)
        assert node.clock-started < 15.
        if kind == 'golf':
            node.removed.add(7)
        else:
            stages.append(('down_complete', node.clock))
        return success
    def recover(label):
        if stages and label == '组合抓取：抓取后恢复作业深度':
            stages.append(('up_start', node.clock))
        return restore(label)
    task._descend_to_depth, task._restore_work_depth = down, recover
    result = task.execute()
    assert result, result.message
    progress = node.state.pickup
    assert progress.golf_status == progress.ring_status == 'success'
    assert progress.golf_attempts == progress.ring_attempts == 1
    assert progress.frame_pose and progress.ring_camera_pose and progress.golf_camera_pose
    assert searches == [work_depth]
    assert progress.depth == work_depth
    assert task.ball._servo_depth == task.ring._servo_depth == work_depth
    assert node.pose[2] == work_depth
    closes = [stamp for stamp, servo, angle in node.servos if servo == 2 and angle == 90.]
    assert len(closes) == 3
    assert stages[0][1] <= closes[0] <= closes[-1] <= stages[1][1]
    ring_hold = next(call for call in node.calls if call[0] == 'action' and
                     call[4].get('task_context') == '抓环：根据成功下视相机与发夹爪TF对准')
    assert ring_hold[2][2] == work_depth
    assert ring_hold[2][3] == pytest.approx(60., abs=.1)
    assert ring_hold[2][:2] == pytest.approx([.45-.08*math.cos(math.pi/3), .08-.08*math.sin(math.pi/3)], abs=.02)
    assert np.allclose(node.velocity, 0.)


def test_shared_pickup_snapshots_reset_without_changing_bias():
    state = TaskState()
    state.set_bias([1., 2., 3., 4.])
    state.update_pickup(frame_pose=(1., 2., .2, 30.), golf_attempts=2, golf_status='success')
    snapshot = state.pickup
    with pytest.raises(FrozenInstanceError):
        snapshot.golf_attempts = 3
    state.reset_pickup(start_xy=(0., 0.))
    assert snapshot.golf_attempts == 2
    assert state.pickup.golf_attempts == state.pickup.ring_attempts == 0
    assert state.pickup.frame_pose is None
    assert state.bias == (1., 2., 3., 4.)


@pytest.mark.parametrize('params', [{'golf_max_attempts': 0}, {'ring_max_attempts': 1.5},
    {'max_grab_retries': 3}, {'work_depth_m': -0.1}, {'work_depth_m': True},
    {'work_depth_m': float('nan')}, {'work_depth_m': float('inf')}, {'ring_open_angle_deg': 271.}, {'ring_open_angle_deg': -1.}, {'ring_close_angle_deg': 271.}, {'ring_close_angle_deg': -1.}, {'ring_orientation_timeout': float('nan')}, {'verification_timeout': float('inf')}])
def test_invalid_combined_parameters_rejected_at_load_and_runtime(params):
    with pytest.raises(ValueError):
        mod.validate_combined_params(params)
    with pytest.raises(ConfigError):
        _validate_params('26rb_grab_ball_ring', params)


def test_combined_yaml_and_mission_configuration():
    path = Path(__file__).parents[1]/'config/tasks/26rb_grab_ball_ring.yaml'
    task = load_task(path)[0]
    assert task['name'] == '26rb_grab_ball_ring'
    assert task['params']['golf_max_attempts'] == task['params']['ring_max_attempts'] == 3
    assert task['params']['ring_open_angle_deg'] == 0.
    assert task['params']['ring_close_angle_deg'] == 90.
    assert task['params']['work_depth_m'] == .2
    assert 'search_cruise_depth_m' not in task['params']
    assert 'ascent_duration_seconds' not in task['params']
    assert 'descent_duration_seconds' not in task['params']
    assert task['params']['golf_grab_depth_m'] == task['params']['ring_grab_depth_m'] == .44
    assert task['params']['descent_timeout'] == 15.
    assert task['params']['retry_depth_step_m'] == .05


def test_ring_yaw_feedback_and_depth_hold_share_one_velocity_command(rig):
    node, task = rig
    node.pose[2:] = [.25, 8., 10., -170.]
    task.ring._servo_depth = .2
    task.ring._servo_yaw_target = 170.
    task.ring._send_horizontal_velocity(np.array([.02, -.01]), node.clock+1.)
    velocity = node.velocity.copy()
    assert body_to_world_rotation(node.pose) @ velocity[:3] == pytest.approx([.02, -.01, -.04])
    assert velocity[3] == -10.


def test_failed_ring_descent_still_restores_depth_without_closing(rig):
    node, task = rig
    task.state.update_pickup(frame_pose=(.5, 0., .2, 0.), depth=.2)
    task.ring._servo_horizontally = lambda *args, **kwargs: [.5, 0., .2, 0.]
    task.ring._flash_green = lambda *args: True
    task.ring._wait_pre_descent_settle = lambda: True
    task._orient_and_align_ring = lambda: True
    def failed_descent(kind, controller):
        node.pose[2] = .44
        return False
    task._descend_to_depth = failed_descent
    task.max_attempts['ring'] = 1
    task._verify = lambda *args: False
    task._run_target('ring', task.ring)
    assert node.pose[2] == .2
    assert task.state.pickup.ring_status == 'failed'
    recoveries = [call for call in node.calls if call[0] == 'action' and
                  call[4].get('task_context') == '组合抓取：抓取后恢复作业深度']
    assert len(recoveries) == 1 and recoveries[0][3] == 'z'
    assert not node.servos


def test_tf_lookup_error_rejects_grasp_before_descending(rig):
    node, task = rig
    task.ring._servo_depth = .2
    task.state.update_pickup(depth=.2)
    task.ring._aligned_camera = 'down_left'
    class TransformError(Exception):
        pass
    def missing_tf(*args):
        raise TransformError('missing hairpin TF')
    node.camera_extrinsics_provider.lookup_transform = missing_tf
    with pytest.raises(mod.PickupFailure, match='读取发夹爪外参失败'):
        task._orient_and_align_ring()
    assert not any(servo == 2 and angle == 90. for _, servo, angle in node.servos)


@pytest.mark.parametrize('legacy', [{'search': {'cruise_depth_m': .31}},
                                     {'search_cruise_depth_m': .31}])
def test_old_depth_spelling_maps_to_the_single_work_depth(legacy):
    params = _validate_params('26rb_grab_ball_ring', legacy)
    assert params == {'work_depth_m': .31}
    with pytest.raises(ConfigError, match='重复'):
        _validate_params('26rb_grab_ball_ring', {**legacy, 'work_depth_m': .2})


def test_failed_depth_restore_prevents_horizontal_recovery(rig):
    node, task = rig
    task.state.update_pickup(frame_pose=(.6, .1, .2, 20.), depth=.2)
    node.pose[2] = .44
    def fail_depth(command, target, axes, **kwargs):
        node.calls.append(('failed_action', axes))
        return False, '模拟定深失败'
    node._send_action_goal = fail_depth
    with pytest.raises(mod.PickupFailure, match='定深失败'):
        task._return_frame(task.ball)
    assert node.calls == [('failed_action', 'z')]
    assert node.pose[2] == .44


@pytest.mark.parametrize('kind,attempt,expected', [('golf', 1, .44), ('golf', 2, .49),
    ('golf', 3, .54), ('ring', 1, .44), ('ring', 2, .49), ('ring', 3, .54)])
def test_descent_stops_at_measured_retry_depth_before_timeout(rig, kind, attempt, expected):
    node, task = rig
    # Nonzero roll/pitch must not turn vertical descent into horizontal drift.
    node.pose[3:] = [12., -8., 35.]
    before = node.pose.copy()
    task.state.update_pickup(**{kind+'_attempts': attempt})
    started = node.clock
    assert task._descend_to_depth(kind, task.ball if kind == 'golf' else task.ring)
    assert node.pose[2] == pytest.approx(expected, abs=1e-6)
    assert node.pose[:2] == pytest.approx(before[:2], abs=1e-9)
    assert node.clock-started < 15.
    assert np.allclose(node.velocity, 0.)


def test_ball_and_ring_retry_depths_increment_independently_and_reset(rig):
    node, task = rig
    task.grab_depths = {'golf': .40, 'ring': .46}
    task.state.update_pickup(golf_attempts=3, ring_attempts=1)
    assert task._grab_target_depth('golf') == pytest.approx(.50)
    assert task._grab_target_depth('ring') == pytest.approx(.46)
    task.state.update_pickup(ring_attempts=2)
    assert task._grab_target_depth('ring') == pytest.approx(.51)
    task.retry_depth_step = 0.
    assert task._grab_target_depth('golf') == pytest.approx(.40)
    task.state.reset_pickup()
    task.retry_depth_step = .05
    assert task._grab_target_depth('golf') == pytest.approx(.40)
    assert task._grab_target_depth('ring') == pytest.approx(.46)


def test_real_descents_get_deeper_after_failed_verification(rig):
    node, task = rig
    task.state.update_pickup(frame_pose=(.5, 0., .2, 0.), depth=.2)
    task.ball._servo_depth = .2
    task.ring._servo_depth = .2
    task.ball._servo_horizontally = task.ring._servo_horizontally = lambda *args, **kwargs: [.5, 0., .2, 0.]
    reached = []
    def attempt(kind, controller):
        ok = task._descend_to_depth(kind, controller)
        reached.append((kind, node.pose[2]))
        return ok
    task._mechanical_attempt = attempt
    task._verify = lambda *args: False
    task._run_target('golf', task.ball)
    task._run_target('ring', task.ring)
    assert [kind for kind, _ in reached] == ['golf', 'golf', 'ring', 'ring', 'ring']
    assert [depth for _, depth in reached] == pytest.approx([.44, .49, .44, .49, .54], abs=1e-6)
    assert node.pose[2] == .2


@pytest.mark.parametrize('kind', ['golf', 'ring'])
def test_descent_timeout_at_fifteen_seconds_releases_velocity_and_recovers(rig, kind):
    node, task = rig
    task.state.update_pickup(frame_pose=(.5, 0., .2, 0.), depth=.2)
    controller = task.ball if kind == 'golf' else task.ring
    controller._servo_depth = .2
    controller._servo_horizontally = lambda *args, **kwargs: [.5, 0., .2, 0.]
    controller._wait_for_detection = lambda *args: object()
    controller._flash_green = lambda *args: True
    controller._prepare_claw = lambda: True
    controller._apply_camera_gripper_offset = lambda: [.5, 0., .2, 0.]
    controller._wait_pre_descent_settle = lambda: True
    task._orient_and_align_ring = lambda: True
    task.max_attempts[kind] = 1
    task._verify = lambda *args: False
    # Simulate a blocked vehicle: time and detections advance, measured Z does not.
    def blocked_sleep(dt):
        node.clock += dt
        node.hook()
    mod.time.sleep = blocked_sleep
    started = node.clock
    task._run_target(kind, controller)
    assert node.clock-started == pytest.approx(15.)
    assert np.allclose(node.velocity, 0.)
    assert getattr(task.state.pickup, kind+'_status') == 'failed'
    assert not any(servo == 2 and angle == 90. for _, servo, angle in node.servos)
    stops = [call for call in node.calls if call[0] == 'velocity' and '结束目标深度下潜' in call[3].get('task_context', '')]
    assert stops[-1][3]['wait_deadline'] > node.clock
    recovery = [call for call in node.calls if call[0] == 'action' and call[4].get('task_context') == '组合抓取：抓取后恢复作业深度']
    assert recovery and recovery[0][3] == 'z' and recovery[0][2][2] == .2


@pytest.mark.parametrize('failure', ['velocity', 'pose', 'neutral', 'cancel'])
def test_descent_failure_always_neutralizes_and_reports_failure(rig, failure):
    node, task = rig
    original_velocity = node._send_body_velocity
    if failure in ('velocity', 'neutral'):
        def fail_velocity(*args, **kwargs):
            original_velocity(*args, **kwargs)
            moving = any(abs(value) > 1e-9 for value in node.velocity[:3])
            if (failure == 'velocity' and moving) or (failure == 'neutral' and not moving):
                return False, '模拟速度或停车失败'
            return True, ''
        node._send_body_velocity = fail_velocity
    elif failure == 'pose':
        node.pose[2] = float('nan')
    else:
        node.hook = lambda: setattr(node, 'stopped', True)
    if failure == 'pose':
        with pytest.raises((mod.PickupFailure, ValueError)):
            task._descend_to_depth('ring', task.ring)
    else:
        assert not task._descend_to_depth('ring', task.ring)
    assert node.calls[-1][0] == 'velocity'
    assert np.allclose(node.velocity, 0.)


@pytest.mark.parametrize('params', [{'golf_grab_depth_m': .1}, {'ring_grab_depth_m': float('nan')},
    {'ring_grab_depth_m': True}, {'retry_depth_step_m': -.05}, {'retry_depth_step_m': float('inf')},
    {'descent_timeout': 0.}, {'descent_timeout': float('nan')}, {'descent_speed_mps': 0.},
    {'vertical_publish_period': 0.}])
def test_invalid_target_depth_and_descent_settings_rejected(params):
    with pytest.raises(ValueError):
        mod.validate_combined_params(params)
    with pytest.raises(ConfigError):
        _validate_params('26rb_grab_ball_ring', params, complete=True)


def test_old_descent_duration_is_a_timeout_alias_not_a_second_timer():
    assert _validate_params('26rb_grab_ball_ring', {'descent': {'duration_seconds': 8.}}) == {'descent_timeout': 8.}
    with pytest.raises(ConfigError, match='重复'):
        _validate_params('26rb_grab_ball_ring', {'descent': {'duration_seconds': 8., 'timeout': 15.}})


def test_reaching_depth_after_the_deadline_is_not_success(rig):
    node, task = rig
    original_velocity = node._send_body_velocity
    def late_velocity(*args, **kwargs):
        ok = original_velocity(*args, **kwargs)
        if any(abs(value) > 1e-9 for value in node.velocity[:3]):
            node.pose[2] = .44
            node.clock += 15.1
        return ok
    node._send_body_velocity = late_velocity
    assert not task._descend_to_depth('ring', task.ring)
    assert np.allclose(node.velocity, 0.)


def test_partial_work_depth_override_validates_against_merged_grab_depths():
    override = _validate_params('26rb_grab_ball_ring', {'work_depth_m': .6})
    params = _validate_params('26rb_grab_ball_ring',
        {'golf_grab_depth_m': .8, 'ring_grab_depth_m': .85, **override}, complete=True)
    assert params['work_depth_m'] == .6
    with pytest.raises(ConfigError, match='不小于work_depth_m'):
        _validate_params('26rb_grab_ball_ring', override, complete=True)


@pytest.mark.parametrize('key', ['ring_open_angle_deg', 'ring_close_angle_deg'])
@pytest.mark.parametrize('angle', [0., 180., 181., 270.])
def test_ring_servo_full_angle_range(key, angle):
    mod.validate_combined_params({key: angle})
    _validate_params('26rb_grab_ball_ring', {key: angle})
