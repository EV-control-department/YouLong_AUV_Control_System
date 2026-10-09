"""Gate acceptance tests with a virtual clock, odom and action server.

No ROS executor, camera hardware or thrusters are started.
"""
from importlib import import_module
import math
from types import SimpleNamespace as NS

import numpy as np
import pytest

from uv_camera.camera_config import load_camera_config
from uv_task.gate_config import validate_gate_params
from uv_task.config_loader import ConfigError, _validate_params, load_task
from pathlib import Path

mod = import_module('uv_task.26rb_gate_task')
Task = mod.RB26GateTask
OPTICAL = np.array([[0., 0., 1.], [1., 0., 0.], [0., 1., 0.]])


class Clock:
    def __init__(self):
        self.time = 100.0
        self.hook = lambda: None
        self.node = None

    def now(self):
        return self.time

    def sleep(self, dt):
        self.time += dt
        node = self.node
        if node is not None:
            body = np.array(node.velocity[:3])
            world = mod.rotation(node.pose) @ body
            node.pose[:3] += world*dt
            if node.integrate_yaw:
                node.pose[5] = mod.wrap_degrees(node.pose[5]+node.velocity[3]*dt)
        self.hook()


class Node:
    LIGHT_YELLOW = 1
    LIGHT_GREEN = 2
    LIGHT_RED = 3
    LIGHT_OFF = 0
    stopped = False

    def __init__(self, clock):
        self.clock = clock
        clock.node = self
        self.camera_configs = {'front': load_camera_config('front', 'sim')}
        self.camera_extrinsics = {f'front_{e}': NS(
            translation=np.array([.19, offset, .176]), optical_to_body=OPTICAL)
            for e, offset in [('left', -.05), ('right', .05)]}
        self._model_mapping = NS(model_class_id=lambda *a, **k: 7)
        self.pose = np.array([0., 0., .1, 0., 0., 0.])
        self.velocity = [0., 0., 0., 0.]
        self.integrate_yaw = True
        self.calls = []
        self.lights = []
        self.logs = []
        self._last_motion_final_target = None
        self._task_failure_light_handled = False
        self._active_goal_handle = None

    def get_clock(self):
        return NS(now=lambda: NS(nanoseconds=int(self.clock.time*1e9)))

    def get_logger(self):
        return NS(info=self.logs.append, warn=self.logs.append)

    def create_subscription(self, *args):
        return args

    def destroy_subscription(self, *args):
        pass

    def _latest_robot_pose(self):
        return tuple(self.pose)

    def _set_task_phase_light(self, color, label, **kwargs):
        self.lights.append(color)

    def _send_body_velocity(self, *args, yaw_rate_deg_s=0., **kwargs):
        body = list(args or [0., 0., 0.])
        self.velocity = [*body, yaw_rate_deg_s]
        self.calls.append(('velocity', self.velocity.copy(), kwargs))
        return True, ''

    def _send_action_goal(self, command, target, axes, **kwargs):
        self.calls.append(('action', command, list(target), axes, kwargs))
        self.velocity = [0., 0., 0., 0.]
        if command == mod.BasicMotion.Goal.SET:
            self.pose[:3] = target[:3]
            if 'rz' in axes:
                self.pose[5] = target[3]
        else:
            yaw = math.radians(self.pose[5])
            self.pose[:3] += [target[0]*math.cos(yaw), target[0]*math.sin(yaw), target[2]]
            self._last_motion_final_target = [*self.pose[:3], self.pose[5]]
        return True, ''


@pytest.fixture
def rig(monkeypatch):
    monkeypatch.setattr(mod.rclpy, 'ok', lambda: True)
    clock = Clock()
    node = Node(clock)
    task = Task(node, {})
    task._now, task._sleep = clock.now, clock.sleep
    task._deadline = clock.time+720
    task._accept_frames = True
    return task, node, clock


def frame(task, eye='left', center=None, area=20., pair=0, empty=False, stamp=None, boxes=None):
    w, h = task.width, task.height
    if center is None:
        center = [task.k[eye][0, 2], task.k[eye][1, 2]]
    side = math.sqrt(area/100)
    box = [center[0]-w*side/2, center[1]-h*side/2,
           center[0]+w*side/2, center[1]+h*side/2]
    boxes = [] if empty else ([box] if boxes is None else boxes)
    timestamp = task._now() if stamp is None else stamp
    detections = [NS(class_id=7, confidence=.8,
                     bbox_x1=b[0], bbox_y1=b[1], bbox_x2=b[2], bbox_y2=b[3]) for b in boxes]
    msg = NS(camera_name='front_'+eye, stereo_pair_id=pair,
             header=NS(stamp=NS(sec=int(timestamp), nanosec=int((timestamp%1)*1e9))),
             detections=detections)
    task._detection_cb(msg)


def project(task, eye, point):
    pose = task._pose()
    extrinsic = task.extrinsics[eye]
    origin = np.asarray(pose[:3])+mod.rotation(pose)@extrinsic.translation
    vector = extrinsic.optical_to_body.T@mod.rotation(pose).T@(np.asarray(point)-origin)
    # sim calibration distortion is zero.
    return [task.k[eye][0, 0]*vector[0]/vector[2]+task.k[eye][0, 2],
            task.k[eye][1, 1]*vector[1]/vector[2]+task.k[eye][1, 2]]


@pytest.mark.parametrize('first', ['left', 'right'])
def test_first_eye_wins_and_other_keeps_data(rig, first):
    task, _, _ = rig
    frame(task, first)
    frame(task, 'right' if first == 'left' else 'left')
    assert task.owner == first
    assert all(task._latest.values())


@pytest.mark.parametrize('winner', ['left', 'right'])
def test_loss_release_and_new_frame_race(rig, winner):
    task, _, clock = rig
    frame(task)
    frame(task, 'right')
    frame(task, empty=True)
    clock.time += 1.9
    assert task._owner_frame() is None
    assert task.owner == 'left'
    clock.time += .11
    assert task._owner_frame() is None
    assert task.owner is None  # cached other eye cannot acquire ownership
    frame(task, winner)
    assert task.owner == winner
    assert task._owner_frame() is not None


def test_lost_owner_stops_horizontal_even_when_other_eye_has_box(rig):
    task, node, clock = rig
    frame(task)
    frame(task, 'right')
    frame(task, empty=True)
    node.pose[2] = 0.
    clock.hook = lambda: setattr(node, 'stopped', True)
    with pytest.raises(mod.GateFailure):
        task._lateral(.5)
    world = mod.rotation(node.pose)@node.calls[0][1][:3]
    assert np.allclose(world[:2], 0)
    assert world[2] > 0


def test_stale_timestamp_does_not_win(rig):
    task, _, clock = rig
    frame(task, stamp=clock.time-2)
    assert task.owner is None
    frame(task, 'right')
    assert task.owner == 'right'


def test_clipped_bbox_area_and_identity_rejects_neighbour(rig):
    task, _, _ = rig
    frame(task, boxes=[[-100, -100, task.width/2, task.height]])
    assert task._owner_frame().area_percent == pytest.approx(50.)
    original = task._owner_frame()
    frame(task, boxes=[[task.width*.85, 0, task.width, task.height*.2]])
    assert task._owner_frame() is None
    assert task._reference is original


def test_depth_velocity_uses_full_measured_rotation(rig):
    task, node, _ = rig
    node.pose[2:5] = [0., 22., -30.]
    task._velocity(horizontal=(.05, -.03))
    body = np.array(node.velocity[:3])
    assert np.allclose(mod.rotation(node.pose)@body, [.05, -.03, .08])


def test_lateral_handover_preserves_measured_path(rig):
    task, node, clock = rig
    frame(task)
    switch_start = None
    def update():
        nonlocal switch_start
        if node.pose[1] >= .05 and switch_start is None:
            switch_start = clock.time
        if switch_start is None:
            frame(task)
        elif clock.time-switch_start <= 2.1:
            frame(task, empty=True)
            frame(task, 'right')
        else:
            frame(task, 'right')
    clock.hook = update
    task._lateral(.15)
    assert task.owner == 'right'
    assert node.pose[1] == pytest.approx(.15, abs=.021)
    assert abs(node.pose[0]) < .02
    assert clock.time < 115


def test_recorded_ray_and_stability_survive_handover(rig):
    task, node, clock = rig
    frame(task, area=80)
    task._record_ray(task._owner_frame())
    ray = task._recorded_ray
    node.pose[:2] = ray.origin[:2]
    old_generation = task.generation
    def update():
        if clock.time < 100.3:
            frame(task, area=80)
        elif clock.time < 102.4:
            frame(task, empty=True)
            frame(task, 'right', area=80)
        else:
            frame(task, 'right', area=80)
    clock.hook = update
    task._fore_aft()
    assert task._recorded_ray is ray
    assert task.generation > old_generation
    assert clock.time >= 102.9-1e-9
    assert task.owner == 'right'


def test_area_unreachable_stops_at_travel_limit(rig):
    task, node, clock = rig
    task.p['fore_aft_max_travel_m'] = .05
    frame(task, area=20)
    task._record_ray(task._owner_frame())
    node.pose[:2] = task._recorded_ray.origin[:2]
    clock.hook = lambda: frame(task, area=20)
    with pytest.raises(mod.GateFailure, match='位移') as error:
        task._fore_aft()
    assert error.value.kind == 'area_unreachable'


@pytest.mark.parametrize('pair', [11, 0])
def test_stereo_center_removes_single_camera_offset(rig, pair):
    task, node, _ = rig
    point = np.array([2., 0., node.pose[2]])
    for eye in ('left', 'right'):
        frame(task, eye, project(task, eye, point), area=10, pair=pair)
    assert np.allclose(task._stereo_center(task._now()), point, atol=1e-5)
    error, source = task._yaw_error(task._owner_frame(), stereo=True)
    assert error == pytest.approx(0., abs=.0001)
    assert source == 'stereo'
    single, source = task._yaw_error(task._owner_frame())
    assert abs(single) > 1


def test_bad_pair_geometry_falls_back_to_owner(rig):
    task, _, _ = rig
    frame(task, pair=12)
    frame(task, 'right', pair=13)
    assert task._stereo_center(task._now()) is None
    assert task._yaw_error(task._owner_frame(), stereo=True)[1] == 'left'
    frame(task, 'right', pair=12)
    # Identical parallel rays cannot triangulate a finite center.
    assert task._stereo_center(task._now()) is None


def test_final_yaw_earliest_one_second_no_horizontal_speed(rig):
    task, node, clock = rig
    frame(task)
    clock.hook = lambda: frame(task)
    task._final_yaw()
    assert clock.time-100 == pytest.approx(1.0, abs=.06)
    assert node.LIGHT_GREEN in node.lights
    assert all(np.allclose(c[1][:2], 0) for c in node.calls if c[0] == 'velocity')


def test_final_yaw_three_seconds_proceeds_with_error(rig):
    task, node, clock = rig
    node.integrate_yaw = False
    center = [task.k['left'][0, 2]+task.k['left'][0, 0]*math.tan(math.radians(10)), task.k['left'][1, 2]]
    frame(task, center=center)
    clock.hook = lambda: frame(task, center=center)
    task._final_yaw()
    assert clock.time-100 == pytest.approx(3, abs=.06)
    assert node.LIGHT_GREEN not in node.lights
    assert any('按当前实测航向' in log for log in node.logs)


def test_final_yaw_lost_at_end_never_sends_line(rig):
    task, node, clock = rig
    frame(task)
    clock.hook = lambda: frame(task, empty=True)
    with pytest.raises(mod.GateFailure, match='不发送 BLINE'):
        task._final_yaw()
    assert not any(c[0] == 'action' for c in node.calls)


def test_final_switch_does_not_extend_three_second_budget(rig):
    task, node, clock = rig
    node.integrate_yaw = False
    center = [task.k['left'][0, 2]+task.k['left'][0, 0]*.2, task.k['left'][1, 2]]
    frame(task, center=center)
    def update():
        frame(task, empty=True)
        frame(task, 'right', center=center)
    clock.hook = update
    task._final_yaw()
    assert task.owner == 'right'
    assert clock.time-100 < 3.1


def test_bline_neutral_before_line_and_final_target_sync(rig):
    task, node, _ = rig
    node.pose[2] = .15
    frame(task)
    task._velocity(yaw_rate=5.)
    task._pass()
    line_index = next(i for i, c in enumerate(node.calls) if c[0] == 'action')
    assert node.calls[line_index-1][0] == 'velocity'
    assert node.calls[line_index-1][1] == [0., 0., 0., 0.]
    line = node.calls[line_index]
    assert line[1] == mod.BasicMotion.Goal.BLINE
    assert np.allclose(line[2], [1.6, 0., -.05, 0.])
    assert line[3] == 'xyz'
    assert line[4]['cruise_speed'] == .15
    assert not any(c[0] == 'velocity' for c in node.calls[line_index+1:])
    assert [node._cmd_x, node._cmd_y, node._cmd_z, node._cmd_yaw] == node._last_motion_final_target


def test_full_sequence_four_depths_and_no_old_servo_after_bline(rig):
    task, node, clock = rig
    # All gates initially fill 80% of image and are centered on each owning eye.
    clock.hook = lambda: frame(task, area=80)
    result = task.execute()
    assert result, result.message
    lines = [c for c in node.calls if c[0] == 'action' and c[1] == mod.BasicMotion.Goal.BLINE]
    assert len(lines) == 4
    assert node._cmd_x == pytest.approx(6.4)
    assert node._cmd_z == .1


def test_search_failure_two_red_flashes_preserved(rig):
    task, node, _ = rig
    task.p['search_timeout'] = .2
    result = task.execute()
    assert not result
    assert node._task_failure_light_handled
    # initial no-detection blink plus the final two failure blinks
    assert node.lights.count(node.LIGHT_RED) == 3
    assert not any(c[0] == 'action' and c[1] == mod.BasicMotion.Goal.BLINE for c in node.calls)


@pytest.mark.parametrize('cancel', [True, False])
def test_cancel_and_timeout_cleanup_neutral(rig, cancel):
    task, node, clock = rig
    def update():
        frame(task, area=20)
        if cancel:
            node.stopped = True
    clock.hook = update
    if not cancel:
        task.p['timeout'] = .2
    result = task.execute()
    assert not result
    assert result.failure_code.endswith('cancelled' if cancel else 'timeout')
    assert node.velocity == [0., 0., 0., 0.]


@pytest.mark.parametrize('params', [
    {'depth_front': [.1]}, {'lateral_front': [0, 0, 0]},
    {'fore_aft_target_area_percent': 101}, {'pass_speed_mps': .18},
    {'pass_yaw_servo_min_seconds': 4}, {'timeout': math.nan},
    {'depth_front': [.1, .1, .1, -.1]},
])
def test_invalid_configuration_rejected_by_runtime_and_loader(params):
    with pytest.raises(ValueError):
        validate_gate_params(params)
    with pytest.raises(ConfigError):
        _validate_params('26rb_gate_task', params)


def test_five_yaml_groups_load():
    source = Path(__file__).parents[1]/'config/tasks/26rb_gate_task.yaml'
    params = load_task(source)[0]['params']
    assert params['depth_front'] == [.1]*4
    assert params['lateral_front'] == [0]*4
    assert params['fore_aft_target_area_percent'] == 80
    assert params['pass_timeout'] == 60
    assert 'bbox_ratio_target' not in params


def test_observation_window_is_full_two_seconds(rig):
    task, _, clock = rig
    clock.hook = lambda: frame(task, 'right')
    task._search()
    assert task.owner == 'right'
    # Two seconds observing, then the green pulse + gap; discovery does not
    # prematurely end the initial observation window.
    assert clock.time >= 102.6-1e-8


def test_scan_stops_on_first_right_eye_frame(rig):
    task, node, clock = rig
    def update():
        if clock.time > 103.0:
            frame(task, 'right')
    clock.hook = update
    task._search()
    assert task.owner == 'right'
    assert node.LIGHT_GREEN in node.lights
    assert clock.time < 104
    assert node.velocity == [0., 0., 0., 0.]


def test_scan_unwraps_yaw_across_180_degrees(rig):
    task, node, clock = rig
    node.pose[5] = 170
    task.p['search_observe_seconds'] = .01
    task.p['search_start_offset_deg'] = 0
    task.p['search_sweep_degrees'] = [60]
    with pytest.raises(mod.GateFailure, match='扫视结束'):
        task._search()
    # Must traverse all 60°, not stop at wrapped -180° after only 10°.
    assert node.pose[5] == pytest.approx(-130., abs=1.)
    assert clock.time > 106


def test_negative_lateral_distance(rig):
    task, node, clock = rig
    frame(task)
    clock.hook = lambda: frame(task)
    task._lateral(-.1)
    assert node.pose[1] == pytest.approx(-.1, abs=.021)


def test_fore_aft_reverse_and_cross_track_correction(rig):
    task, node, clock = rig
    frame(task, area=95)
    task._record_ray(task._owner_frame())
    node.pose[:2] = task._recorded_ray.origin[:2]+[0., .1]
    def update():
        node.stopped = True
    clock.hook = update
    with pytest.raises(mod.GateFailure):
        task._fore_aft()
    world = mod.rotation(node.pose)@node.calls[0][1][:3]
    assert world[0] < 0 and world[1] < 0
    assert abs(world[0]) <= .08
    assert abs(world[1]) <= .03


def test_recording_requires_a_post_alignment_frame(rig):
    task, _, clock = rig
    frame(task)
    clock.hook = lambda: frame(task)
    floor = task._align(.1)
    previous = task._owner_frame()
    fresh = task._fresh_after(floor)
    assert fresh.sequence > previous.sequence
    assert fresh.sequence > floor


def test_loss_during_neutral_handoff_prevents_line(rig):
    task, node, clock = rig
    frame(task)
    clock.hook = lambda: frame(task)
    original = node._send_body_velocity
    def send(*args, **kwargs):
        result = original(*args, **kwargs)
        if not args:
            frame(task, empty=True)
        return result
    node._send_body_velocity = send
    with pytest.raises(mod.GateFailure, match='不发送 BLINE'):
        task._final_yaw()
    assert not any(c[0] == 'action' for c in node.calls)


def test_stereo_invalid_epipolar_geometry_is_rejected(rig):
    task, node, _ = rig
    point = np.array([2., 0., node.pose[2]])
    frame(task, 'left', project(task, 'left', point), pair=31)
    wrong = project(task, 'right', point)
    wrong[1] += task.height*.25
    frame(task, 'right', wrong, pair=31)
    assert task._stereo_center(task._now()) is None


def test_stereo_timestamp_slop_rejects_unpaired_frames(rig):
    task, node, clock = rig
    point = np.array([2., 0., node.pose[2]])
    frame(task, 'left', project(task, 'left', point))
    clock.time += .3
    frame(task, 'right', project(task, 'right', point))
    assert task._stereo_center(task._now()) is None


def test_velocity_action_wait_uses_phase_deadline(rig):
    task, node, clock = rig
    task._phase_deadline = clock.time+.1
    task._velocity()
    assert node.calls[-1][2]['wait_deadline'] == clock.time+.1


def test_next_gate_clears_identity_and_owner(rig):
    task, _, _ = rig
    frame(task, 'right')
    task._record_ray(task._owner_frame())
    task._reset_gate(.2)
    assert task.owner is None
    assert task._reference is None
    assert task._recorded_ray is None
    assert not any(task._history.values())
    assert not any(task._identity.values())


@pytest.mark.parametrize('first_warn', [False, True])
def test_logging_can_alternate_info_and_warning(rig, monkeypatch, first_warn):
    from rclpy.logging import get_logger, LoggingSeverity

    task, node, _ = rig
    logger = get_logger(f'gate_logging_alternation_{first_warn}')
    logger.set_level(LoggingSeverity.INFO)
    monkeypatch.setattr(node, 'get_logger', lambda: logger)
    for warn in (first_warn, not first_warn, first_warn, not first_warn):
        task._log('Gate logging regression', warn=warn)


def test_detection_callback_releases_lost_camera_with_ros_logger(rig, monkeypatch):
    from rclpy.logging import get_logger, LoggingSeverity

    task, node, clock = rig
    logger = get_logger('gate_logging_camera_release')
    logger.set_level(LoggingSeverity.INFO)
    monkeypatch.setattr(node, 'get_logger', lambda: logger)
    frame(task, 'left')
    assert task.owner == 'left'
    generation = task.generation
    clock.time += task.p['search_priority_release_seconds'] + .1
    frame(task, 'right')
    assert task.owner == 'right'
    assert task.generation > generation
    assert task._latest['right'] is not None
