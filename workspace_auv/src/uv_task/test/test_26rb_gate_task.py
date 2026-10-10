"""Gate acceptance tests with a virtual clock, odom and action server.

No ROS executor, camera hardware or thrusters are started.
"""
from importlib import import_module
import math
from types import SimpleNamespace as NS

import numpy as np
import pytest
import yaml

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

    def _latest_robot_pose(self, require_measured=False):
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
            self.pose[:3] += [target[0]*math.cos(yaw)-target[1]*math.sin(yaw),
                              target[0]*math.sin(yaw)+target[1]*math.cos(yaw), target[2]]
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


def frame(task, eye='left', center=None, height=70., width=40., pair=0,
          empty=False, stamp=None, boxes=None, lock=True, count=3, capture=0):
    w, h = task.width, task.height
    if center is None:
        center = [task.k[eye][0, 2], task.k[eye][1, 2]]
    box = [center[0]-w*width/200, center[1]-h*height/200,
           center[0]+w*width/200, center[1]+h*height/200]
    boxes = [] if empty else ([box] if boxes is None else boxes)
    timestamp = task._now() if stamp is None else stamp
    detections = [NS(class_id=7, confidence=.8, bbox_x1=b[0], bbox_y1=b[1],
                     bbox_x2=b[2], bbox_y2=b[3]) for b in boxes]
    msg = NS(camera_name='front_'+eye, stereo_pair_id=pair, capture_id=capture,
             header=NS(stamp=NS(sec=int(timestamp), nanosec=int((timestamp%1)*1e9))),
             detections=detections)
    for _ in range(count):
        task._detection_cb(msg)
    if lock and task._target_id is None:
        track = task._tracks.highest(task._now(), True)
        if track is not None:
            task._select_track(track, fresh=True)


def project(task, eye, point):
    pose = task._pose()
    ext = task.extrinsics[eye]
    origin = np.asarray(pose[:3])+mod.rotation(pose)@ext.translation
    vector = ext.optical_to_body.T@mod.rotation(pose).T@(np.asarray(point)-origin)
    return [task.k[eye][0, 0]*vector[0]/vector[2]+task.k[eye][0, 2],
            task.k[eye][1, 1]*vector[1]/vector[2]+task.k[eye][1, 2]]


def record_ray(task, node):
    task._record_ray(task._owner_frame())
    node.pose[:2] = task._recorded_ray.origin[:2]


@pytest.mark.parametrize('eye', ['left', 'right'])
def test_three_frames_confirm_before_selection(rig, eye):
    task, _, _ = rig
    frame(task, eye, count=2)
    assert task.owner is None
    frame(task, eye, count=1)
    assert task.owner == eye
    assert task._target_id is not None


def test_capture_duplicate_and_stale_frames_do_not_confirm(rig):
    task, _, clock = rig
    frame(task, capture=1)
    assert task.owner is None
    frame(task, capture=2, stamp=clock.time-2)
    assert task.owner is None
    frame(task, capture=3, count=1)
    frame(task, capture=4, count=1)
    assert task.owner == 'left'


def test_select_tallest_not_largest_area(rig):
    task, _, _ = rig
    w, h = task.width, task.height
    narrow_tall = [w*.2, h*.1, w*.3, h*.9]
    wide_short = [w*.4, h*.2, w*.95, h*.8]
    frame(task, boxes=[wide_short, narrow_tall])
    assert task._owner_frame().height_percent == pytest.approx(80)
    assert task._owner_frame().area_percent == pytest.approx(8)


@pytest.mark.parametrize('width', [20, 40, 80])
def test_height_percent_is_single_eye_and_independent_of_width(rig, width):
    task, _, _ = rig
    frame(task, height=70, width=width)
    assert task._owner_frame().height_percent == pytest.approx(70)


def test_bbox_clipped_to_eye_height(rig):
    task, _, _ = rig
    frame(task, boxes=[[-100, -100, task.width/2, task.height*2]])
    assert task._owner_frame().height_percent == pytest.approx(100)
    assert task._owner_frame().area_percent == pytest.approx(50)


@pytest.mark.parametrize('height,sign', [(60, 1), (80, -1), (69.5, 0), (70, 0), (70.5, 0)])
def test_height_servo_direction_and_depth_hold(rig, height, sign):
    task, node, clock = rig
    frame(task, height=height)
    record_ray(task, node)
    node.pose[2] = 0
    clock.hook = lambda: setattr(node, 'stopped', True)
    with pytest.raises(mod.GateFailure):
        task._fore_aft()
    world = mod.rotation(node.pose)@node.calls[0][1][:3]
    assert np.sign(world[0]) == sign
    assert world[2] > 0
    assert abs(world[0]) <= (.1 if sign >= 0 else .08)
    assert '高度伺服' in node.logs[-1]


def test_height_servo_stability_and_fixed_ray(rig):
    task, node, clock = rig
    frame(task)
    record_ray(task, node)
    ray = task._recorded_ray
    clock.hook = lambda: frame(task)
    task._fore_aft()
    assert clock.time >= 100.5
    assert task._recorded_ray is ray
    assert node.velocity == [0, 0, 0, 0]
    assert node.pose[2] == pytest.approx(.2, abs=.03)


def test_height_unreachable_stops_at_travel_limit(rig):
    task, node, clock = rig
    task.p['fore_aft_max_travel_m'] = .05
    frame(task, height=20)
    record_ray(task, node)
    clock.hook = lambda: frame(task, height=20)
    with pytest.raises(mod.GateFailure, match='位移') as error:
        task._fore_aft()
    assert error.value.kind == 'height_unreachable'


def test_reverse_and_cross_track_correction(rig):
    task, node, clock = rig
    frame(task, height=95)
    record_ray(task, node)
    node.pose[1] += .1
    clock.hook = lambda: setattr(node, 'stopped', True)
    with pytest.raises(mod.GateFailure):
        task._fore_aft()
    world = mod.rotation(node.pose)@node.calls[0][1][:3]
    assert -.08 <= world[0] < 0
    assert -.03 <= world[1] < 0


def test_lost_target_never_switches_to_far_gate_and_holds_depth(rig):
    task, node, clock = rig
    frame(task)
    selected = task._target_id
    frame(task, height=25)
    assert task._owner_frame() is None
    assert task._target_id == selected
    node.pose[2] = 0
    clock.hook = lambda: setattr(node, 'stopped', True)
    with pytest.raises(mod.GateFailure):
        task._lateral(.5)
    world = mod.rotation(node.pose)@node.calls[0][1][:3]
    assert np.allclose(world[:2], 0)
    assert world[2] > 0


def test_ambiguous_neighbours_freeze_then_reacquire_same_id(rig):
    task, _, _ = rig
    frame(task)
    selected = task._target_id
    cx, cy = task._owner_frame().center
    w, h = task.width, task.height
    boxes = [[cx-w*.2+dx, cy-h*.35, cx+w*.2+dx, cy+h*.35] for dx in [-w*.02, w*.02]]
    frame(task, boxes=boxes)
    assert task._owner_frame() is None
    assert selected in task._tracks.ambiguous
    frame(task)
    assert task._owner_frame() is not None
    assert task._target_id == selected


def test_detection_order_and_larger_gate_do_not_change_target(rig):
    task, _, _ = rig
    w, h = task.width, task.height
    original = [w*.35, h*.15, w*.65, h*.85]
    neighbour = [w*.01, h*.02, w*.20, h*.98]
    frame(task, boxes=[original])
    selected = task._target_id
    for boxes in ([neighbour, original], [original, neighbour]):
        frame(task, boxes=boxes)
        assert task._owner_frame().height_percent == pytest.approx(70)
        assert task._target_id == selected


def test_continuous_approach_size_change_keeps_target(rig):
    task, _, clock = rig
    frame(task, height=40, width=20)
    selected = task._target_id
    for height in [42, 46, 50, 55, 60, 66, 70]:
        clock.time += .1
        frame(task, height=height, width=height/2)
        assert task._owner_frame() is not None
        assert task._target_id == selected


def test_edge_clipping_does_not_shrink_hidden_extent(rig):
    task, _, _ = rig
    w, h = task.width, task.height
    frame(task, boxes=[[0, h*.15, w*.6, h*.85]])
    selected = task._target_id
    frame(task, boxes=[[-w*.05, h*.14, w*.59, h*.86]])
    assert task._owner_frame() is not None
    assert task._target_id == selected


def test_reacquire_timeout_preserves_target_id(rig):
    task, _, clock = rig
    frame(task)
    selected = task._target_id
    assert task._owner_frame() is not None
    frame(task, empty=True)
    assert task._owner_frame() is None
    clock.time += 10.1
    with pytest.raises(mod.GateFailure, match='重获超时'):
        task._owner_frame()
    assert task._target_id == selected


def test_camera_handover_requires_three_unique_confirmations_and_geometry(rig):
    task, node, clock = rig
    point = np.array([2., 0., .2])
    frame(task, center=project(task, 'left', point), pair=11)
    target_id = task._target_id
    frame(task, 'right', center=project(task, 'right', point), pair=11, count=2)
    assert task._latest['right'] is None
    frame(task, 'right', center=project(task, 'right', point), pair=11, count=1)
    assert task._latest['right'] is not None
    assert np.allclose(task._stereo_center(clock.time), point, atol=1e-5)
    clock.time += 2.1
    frame(task, 'right', center=project(task, 'right', point), pair=12)
    assert task.owner == 'right'
    assert task._target_id == target_id


def test_bad_simultaneous_stereo_never_acquires_other_eye(rig):
    task, _, _ = rig
    frame(task, pair=11)
    frame(task, 'right', pair=11)
    assert task._latest['right'] is None
    assert task._stereo_center(task._now()) is None


@pytest.mark.parametrize('yaw', [0, 90, -90, 179])
def test_absolute_observation_btravel_and_set(rig, yaw):
    task, node, _ = rig
    node.pose[:] = [2, 3, .5, 0, 0, yaw]
    task._reach_observation()
    actions = [c for c in node.calls if c[0] == 'action']
    assert [c[1] for c in actions] == [mod.BasicMotion.Goal.BTRAVEL, mod.BasicMotion.Goal.SET]
    assert np.allclose(node.pose[[0, 1, 2, 5]], [1.5, -2, .2, 0])
    assert actions[1][2] == [1.5, -2, .2, 0]
    assert node.lights[-1] == node.LIGHT_GREEN


def test_observe_full_two_seconds_and_two_green_pulses(rig):
    task, node, clock = rig
    node.pose[2] = task.depth
    clock.hook = lambda: frame(task, lock=False, count=1)
    task._search()
    assert clock.time >= 103.5-1e-8
    assert node.lights.count(node.LIGHT_GREEN) == 2
    assert node.lights[0] == node.LIGHT_OFF
    assert task._owner_frame() is not None


def test_entire_right_return_left_scan_even_when_seen_early(rig):
    task, node, clock = rig
    node.pose[[0, 1, 2, 5]] = task._observation_pose
    point = np.array([4., -1., .2])
    def update():
        if task._stage == '右扫 90°' and 10 < node.pose[5] < 30:
            frame(task, center=project(task, 'left', point), lock=False, count=1)
        elif task._stage == '扫描目标转向与重捕获':
            frame(task, center=project(task, 'left', point), count=1)
    clock.hook = update
    task._search()
    rates = [c[1][3] for c in node.calls if c[0] == 'velocity']
    assert 10 in rates and -10 in rates
    actions = [c for c in node.calls if c[0] == 'action']
    assert actions[0][2] == task._observation_pose
    assert clock.time > 120
    assert node.lights.count(node.LIGHT_RED) == 1
    assert node.lights.count(node.LIGHT_GREEN) == 2
    assert task._owner_frame().sequence > task._race_floor


@pytest.mark.parametrize('yaw,sign,expected', [(170, 1, -100), (-170, -1, 100)])
def test_scan_unwraps_measured_yaw(rig, yaw, sign, expected):
    task, node, clock = rig
    node.pose[5] = yaw
    task._scan(sign, clock.time+20)
    assert node.pose[5] == pytest.approx(expected, abs=1)
    assert clock.time >= 109-1e-8


def test_search_failure_red_counts_and_no_line(rig):
    task, node, _ = rig
    result = task.execute()
    assert not result
    assert result.failure_code == '26rb_gate_task.search'
    assert node.lights.count(node.LIGHT_RED) == 3
    assert node._task_failure_light_handled
    assert not any(c[0] == 'action' and c[1] == mod.BasicMotion.Goal.BLINE for c in node.calls)


def test_scan_timeout_is_failure_not_incomplete_search_success(rig):
    task, node, _ = rig
    task.p['search_timeout'] = .2
    result = task.execute()
    assert not result
    assert result.failure_code == '26rb_gate_task.timeout'
    assert node.velocity == [0, 0, 0, 0]


def test_final_yaw_stable_minimum_one_second(rig):
    task, node, clock = rig
    node.pose[2] = task.depth
    frame(task)
    clock.hook = lambda: frame(task)
    task._final_yaw()
    assert clock.time-100 == pytest.approx(1, abs=.06)
    assert node.LIGHT_GREEN in node.lights
    assert all(np.allclose(c[1][:2], 0) for c in node.calls if c[0] == 'velocity')


def test_final_yaw_unstable_times_out_and_never_passes(rig):
    task, node, clock = rig
    node.integrate_yaw = False
    center = [task.k['left'][0, 2]+task.k['left'][0, 0]*math.tan(math.radians(10)), task.k['left'][1, 2]]
    frame(task, center=center)
    clock.hook = lambda: frame(task, center=center)
    with pytest.raises(mod.GateFailure, match='不发送 BLINE'):
        task._final_yaw()
    assert clock.time-100 == pytest.approx(3, abs=.06)
    assert not any(c[0] == 'action' for c in node.calls)


def test_final_yaw_lost_prevents_line(rig):
    task, node, clock = rig
    frame(task)
    clock.hook = lambda: frame(task, empty=True)
    with pytest.raises(mod.GateFailure, match='不发送 BLINE'):
        task._final_yaw()


def test_bline_neutral_handoff_and_final_target(rig):
    task, node, _ = rig
    node.pose[2] = .15
    frame(task)
    task._velocity(yaw_rate=5)
    assert task._pass()
    i = next(i for i, c in enumerate(node.calls) if c[0] == 'action')
    assert node.calls[i-1][1] == [0, 0, 0, 0]
    line = node.calls[i]
    assert line[1] == mod.BasicMotion.Goal.BLINE
    assert np.allclose(line[2], [1.6, 0, .05, 0])
    assert line[4]['cruise_speed'] == .15
    assert not any(c[0] == 'velocity' for c in node.calls[i+1:])
    assert [node._cmd_x, node._cmd_y, node._cmd_z, node._cmd_yaw] == node._last_motion_final_target


def test_bline_rejects_height_drift_or_loss_during_neutral(rig):
    task, node, _ = rig
    frame(task, height=60)
    task._velocity()
    assert task._pass() is False
    assert not any(c[0] == 'action' for c in node.calls)
    original = node._send_body_velocity
    def send(*args, **kwargs):
        result = original(*args, **kwargs)
        if not args:
            frame(task, empty=True)
        return result
    node._send_body_velocity = send
    task._velocity_active = True
    with pytest.raises(mod.GateFailure, match='交接前'):
        task._pass()


def test_complete_one_gate_sequence(rig):
    task, node, clock = rig
    clock.hook = lambda: frame(task, lock=False, count=1)
    outcome = task.execute()
    assert outcome, outcome.message
    actions = [c for c in node.calls if c[0] == 'action']
    assert actions[0][1] == mod.BasicMotion.Goal.BTRAVEL
    assert actions[1][2] == [1.5, -2, .2, 0]
    assert actions[-1][1] == mod.BasicMotion.Goal.BLINE
    assert node.pose[2] == pytest.approx(.2)
    assert '通过 1，跳过 0/1' in outcome.message


@pytest.mark.parametrize('budget,failures,success,visited', [(0, [0], False, 1), (1, [0], True, 3), (1, [0, 1], False, 2), (1, [2], False, 3)])
def test_per_gate_failure_budget_and_next_navigation(rig, monkeypatch, budget, failures, success, visited):
    task, node, clock = rig
    task.p.update(gate_count=3, max_failures=budget,
                  observation_poses=[[1, 0, .2, 0], [2, 1, .3, 90], [3, 2, .4, -90]], timeout=5.)
    starts = []
    def run():
        starts.append((clock.time, task._deadline))
        assert task._target_id is None
        task._reach_observation()
        clock.sleep(4)
        if task._gate_index in failures:
            raise mod.GateFailure('timeout', 'injected gate stage timeout')
    monkeypatch.setattr(task, '_run_gate', run)
    outcome = task.execute()
    assert bool(outcome) == success
    travels = [c for c in node.calls if c[0] == 'action' and c[1] == mod.BasicMotion.Goal.BTRAVEL]
    assert len(travels) == visited
    assert all(deadline-start == pytest.approx(5) for start, deadline in starts)
    assert starts[-1][0] >= 100+4*(visited-1)


@pytest.mark.parametrize('kind', ['cancelled', 'odom'])
def test_terminal_failure_never_skips(rig, monkeypatch, kind):
    task, node, _ = rig
    task.p.update(gate_count=2, observation_poses=[[1, 0, .2, 0], [2, 0, .2, 0]])
    def run():
        raise mod.GateFailure(kind, kind)
    monkeypatch.setattr(task, '_run_gate', run)
    result = task.execute()
    assert not result
    assert task._gate_index == 0


def test_unconfirmed_cancel_prevents_next_gate(rig, monkeypatch):
    task, node, _ = rig
    task.p.update(gate_count=2, observation_poses=[[1, 0, .2, 0], [2, 0, .2, 0]])
    def run():
        task._cleanup_unconfirmed = True
        raise mod.GateFailure('timeout', 'pending late acceptance')
    monkeypatch.setattr(task, '_run_gate', run)
    result = task.execute()
    assert not result
    assert '禁止进入下一门' in result.message
    assert task._gate_index == 0
    assert not any(c[0] == 'action' for c in node.calls)


def test_expired_deadline_cleanup_uses_independent_budget(rig):
    task, node, clock = rig
    task._velocity()
    task._deadline = clock.time-.1
    assert task._park()
    assert node.velocity == [0, 0, 0, 0]
    assert all(c[-1]['wait_deadline'] > clock.time for c in node.calls[-2:])


def test_missing_measured_pose_is_terminal(rig):
    task, node, _ = rig
    node._latest_robot_pose = lambda **_: None
    outcome = task.execute()
    assert outcome.failure_code == '26rb_gate_task.odom'
    assert not any(c[0] == 'action' for c in node.calls)


def test_new_gate_clears_bank_identity_and_rejects_navigation_frames(rig):
    task, _, clock = rig
    frame(task)
    task._record_ray(task._owner_frame())
    task._reset_gate(.4)
    assert not task._tracks.tracks and task._target_id is None
    assert task._reference is None and task._recorded_ray is None
    assert not any(task._history.values())
    frame(task)
    assert not task._tracks.tracks
    task._accept_frames = True
    task._capture_floor = clock.time
    frame(task, stamp=clock.time-.1)
    assert not task._tracks.tracks


@pytest.mark.parametrize('params', [
    {'depth_front': [.1]}, {'lateral_front': [0, 0, 0]},
    {'fore_aft_target_height_percent': 101}, {'pass_speed_mps': .18},
    {'pass_yaw_servo_min_seconds': 4}, {'timeout': math.nan},
    {'depth_front': [.1, .1, .1, -.1]}, {'max_failures': -1}, {'max_failures': True},
    {'tracking_confirm_frames': 1.5}, {'observation_poses': [[1, 2, .2]]},
    {'observation_poses': [[1, 2, -.2, 0]]}, {'observation_poses': [[1, 2, math.inf, 0]]},
    {'gate_count': 2, 'observation_poses': [[1, 2, .2, 0]]},
    {'depth_front': [.1]*4, 'observation_poses': [[1, 2, .2, 0]]},
])
def test_invalid_configuration_rejected(params):
    with pytest.raises(ValueError):
        validate_gate_params(params)
    with pytest.raises(ConfigError):
        _validate_params('26rb_gate_task', params)


def test_runtime_rejects_gate_count_without_required_poses(rig):
    _, node, _ = rig
    with pytest.raises(ValueError, match='数量'):
        Task(node, {'gate_count': 2})


def test_nested_pose_yaml_and_height_defaults():
    source = Path(__file__).parents[1]/'config/tasks/26rb_gate_task.yaml'
    params = load_task(source)[0]['params']
    assert params['observation_poses'] == [[1.5, -2, .2, 0]]
    assert params['fore_aft_target_height_percent'] == 70
    assert params['max_failures'] == 1
    assert validate_gate_params({})['fore_aft_target_height_percent'] == 70
    assert _validate_params('26rb_gate_task', {'observation': {'poses': [[1, 2, .3, 90]]}})['observation_poses'] == [[1, 2, .3, 90]]


def test_legacy_area_inputs_warn_and_do_not_change_height_target(rig):
    _, node, _ = rig
    task = Task(node, {'fore_aft_target_area_percent': 90, 'fore_aft_area_kp': .9})
    assert task.p['fore_aft_target_height_percent'] == 70
    assert task.p['fore_aft_height_kp'] == .4
    assert sum('已废弃' in log for log in node.logs) == 2


@pytest.mark.parametrize('ack_seconds', [.08, .35])
def test_flash_ack_budget_and_initial_off(rig, monkeypatch, ack_seconds):
    task, node, clock = rig
    send = node._send_body_velocity
    phase = task._phase_deadline
    def delayed(*args, **kwargs):
        clock.sleep(ack_seconds)
        if clock.time >= kwargs['wait_deadline']:
            return False, '发送动作目标超时'
        return send(*args, **kwargs)
    monkeypatch.setattr(node, '_send_body_velocity', delayed)
    task._flash(node.LIGHT_GREEN, 2)
    assert node.lights == [0, 2, 0, 2, 0]
    assert node.velocity == [0, 0, 0, 0]
    assert task._phase_deadline == phase


def test_depth_velocity_uses_full_rotation(rig):
    task, node, _ = rig
    node.pose[2:5] = [0, 22, -30]
    task._velocity(horizontal=(.05, -.03))
    assert np.allclose(mod.rotation(node.pose)@node.velocity[:3], [.05, -.03, .12])


@pytest.mark.parametrize('distance', [.15, -.1])
def test_lateral_measured_path(rig, distance):
    task, node, clock = rig
    node.pose[2] = task.depth
    frame(task)
    clock.hook = lambda: frame(task)
    task._lateral(distance)
    assert node.pose[1] == pytest.approx(distance, abs=.021)


@pytest.mark.parametrize('first_warn', [False, True])
def test_logging_info_and_warning_compatible_with_foxy(rig, monkeypatch, first_warn):
    from rclpy.logging import get_logger, LoggingSeverity
    task, node, _ = rig
    logger = get_logger('gate_logging_'+str(first_warn))
    logger.set_level(LoggingSeverity.INFO)
    monkeypatch.setattr(node, 'get_logger', lambda: logger)
    for warn in (first_warn, not first_warn, first_warn):
        task._log('Gate logging regression', warn=warn)


def test_locked_track_never_takes_second_choice_in_global_assignment(rig, monkeypatch):
    task, _, _ = rig
    w, h = task.width, task.height
    boxes = [[w*.35, h*.15, w*.65, h*.85], [w*.72, h*.2, w*.92, h*.8]]
    frame(task, boxes=boxes)
    original_id = task._target_id
    def cost(predicted, measured):
        original_track = (predicted[0]+predicted[2])/2 < w*.65
        original_detection = (measured[0]+measured[2])/2 < w*.65
        return (.3 if original_detection else .55) if original_track else (.05 if original_detection else None)
    monkeypatch.setattr(task._tracks, 'cost', cost)
    frame(task, boxes=boxes, count=1)
    assert task._owner_frame() is None
    assert task._target_id == original_id


def test_gyro_rotation_prediction_keeps_fixed_world_gate(rig):
    task, node, clock = rig
    point = np.array([3., 0., .2])
    frame(task, center=project(task, 'left', point), height=30, width=20)
    original_id = task._target_id
    for yaw in [5, 10, 15, 20]:
        node.pose[5] = yaw
        clock.time += .1
        frame(task, center=project(task, 'left', point), height=30, width=20)
        assert task._owner_frame() is not None
        assert task._target_id == original_id


def test_initial_observation_velocity_preserves_arrival_green(rig):
    task, node, clock = rig
    clock.hook = lambda: frame(task, lock=False, count=1)
    task._search()
    first_hold = next(c for c in node.calls if c[0] == 'velocity')
    assert first_hold[2]['light_color'] == node.LIGHT_GREEN


def test_missing_poses_rejected_by_standalone_loader(tmp_path):
    source = tmp_path/'gate.yaml'
    source.write_text('task: 26rb_gate_task\nparams:\n  gate_count: 2\n')
    with pytest.raises(ConfigError, match='数量'):
        load_task(source)


def test_scan_chooses_tallest_candidate_from_all_scan_angles(rig):
    task, node, clock = rig
    node.pose[[0, 1, 2, 5]] = task._observation_pose
    near = np.array([4., -1., .2])
    far = np.array([4., -3., .2])
    def update():
        if task._stage == '右扫 90°' and 10 < node.pose[5] < 30:
            frame(task, center=project(task, 'left', near), height=75, lock=False, count=1)
        elif task._stage == '左扫 90°' and -30 < node.pose[5] < -10:
            frame(task, center=project(task, 'left', far), height=35, lock=False, count=1)
        elif task._stage == '扫描目标转向与重捕获':
            frame(task, center=project(task, 'left', near), height=75, count=1)
    clock.hook = update
    task._search()
    assert task._owner_frame().height_percent == pytest.approx(75)
    assert task._tracks.tracks[task._target_id].peak_height == pytest.approx(75)


def test_bline_height_drift_returns_to_height_servo(rig, monkeypatch):
    task, _, clock = rig
    clock.hook = lambda: frame(task, lock=False, count=1)
    passes = []
    servo_calls = []
    original_servo = task._fore_aft
    original_pass = task._pass
    def servo():
        servo_calls.append(True)
        original_servo()
    def pass_with_drift():
        passes.append(True)
        if len(passes) == 1:
            return False
        return original_pass()
    monkeypatch.setattr(task, '_fore_aft', servo)
    monkeypatch.setattr(task, '_pass', pass_with_drift)
    assert task.execute()
    assert len(servo_calls) == len(passes) == 2


def test_reacquire_after_silence_requires_three_new_frames(rig):
    task, _, clock = rig
    frame(task)
    target = task._target_id
    clock.time += 2.1
    frame(task, count=1)
    assert task._owner_frame() is None
    frame(task, count=1)
    assert task._owner_frame() is None
    frame(task, count=1)
    assert task._owner_frame() is not None
    assert task._target_id == target


def test_scan_selection_does_not_reuse_prior_confirmation(rig):
    task, _, _ = rig
    frame(task, lock=False)
    selected = task._tracks.highest(task._now(), fresh_only=False)
    task._select_track(selected, fresh=False)
    frame(task, count=2)
    assert task._owner_frame() is None
    frame(task, count=1)
    assert task._owner_frame() is not None


def test_cleanup_can_confirm_previous_timeout_before_next_gate(rig, monkeypatch):
    task, node, clock = rig
    task.p.update(gate_count=2, observation_poses=[[1, 0, .2, 0], [2, 0, .2, 0]])
    cancelled = []
    result = NS(done=lambda: clock.time >= 100.3,
                result=lambda: NS(result=NS(success=False)))
    handle = NS(get_result_async=lambda: result,
                cancel_goal_async=lambda: cancelled.append(clock.time))
    def run():
        if task._gate_index == 0:
            task._cleanup_unconfirmed = True
            node._active_goal_handle = handle
            raise mod.GateFailure('timeout', 'late cancel result')
        assert clock.time >= 100.3
        task._reach_observation()
    monkeypatch.setattr(task, '_run_gate', run)
    outcome = task.execute()
    assert outcome
    assert cancelled
    assert task._gate_index == 1
    assert '跳过 1/1' in outcome.message


@pytest.mark.parametrize('height', [40., 85.])
def test_height_stage_timeout_with_same_gate_aligns_and_passes(rig, monkeypatch, height):
    task, node, clock = rig
    task.p['fore_aft_timeout'] = .2
    clock.hook = lambda: frame(task, height=height, lock=False, count=1)
    sequence = []
    labels = []
    original_servo, original_yaw = task._fore_aft, task._final_yaw
    original_pass, original_flash = task._pass, task._flash
    def servo():
        sequence.append('height')
        original_servo()
    def yaw():
        sequence.append('yaw')
        original_yaw()
    def passed(require_height=True):
        sequence.append(('bline', require_height))
        return original_pass(require_height=require_height)
    def flash(color, count, label='门框观察闪灯'):
        labels.append(label)
        original_flash(color, count, label)
    monkeypatch.setattr(task, '_fore_aft', servo)
    monkeypatch.setattr(task, '_final_yaw', yaw)
    monkeypatch.setattr(task, '_pass', passed)
    monkeypatch.setattr(task, '_flash', flash)
    outcome = task.execute()
    assert outcome, outcome.message
    assert sequence == ['height', 'yaw', ('bline', False)]
    assert '前后对正完成' not in labels
    assert task._owner_frame().height_percent == pytest.approx(height)
    assert '通过 1，跳过 0/1' in outcome.message
    assert any('高度伺服超时，但固定门框' in log for log in node.logs)
    line_index = next(i for i, c in enumerate(node.calls)
                      if c[0] == 'action' and c[1] == mod.BasicMotion.Goal.BLINE)
    assert node.calls[line_index-1][1] == [0., 0., 0., 0.]
    assert node.pose[2] == pytest.approx(.2)
    assert not any(c[0] == 'velocity' for c in node.calls[line_index+1:])


def test_height_timeout_without_visible_gate_still_fails(rig, monkeypatch):
    task, node, clock = rig
    task.p['fore_aft_timeout'] = .2
    active = [False]
    original = task._fore_aft
    def servo():
        active[0] = True
        original()
    monkeypatch.setattr(task, '_fore_aft', servo)
    clock.hook = lambda: frame(task, height=40, empty=active[0], lock=False, count=1)
    outcome = task.execute()
    assert not outcome
    assert not any('高度伺服超时，但固定门框' in log for log in node.logs)
    assert not any(c[0] == 'action' and c[1] == mod.BasicMotion.Goal.BLINE for c in node.calls)
    assert node.velocity == [0., 0., 0., 0.]


def test_height_timeout_does_not_bypass_final_yaw_failure(rig, monkeypatch):
    task, node, clock = rig
    task.p['fore_aft_timeout'] = .2
    clock.hook = lambda: frame(task, height=40, lock=False, count=1)
    attempted = []
    def yaw():
        attempted.append(True)
        raise mod.GateFailure('timeout', '最终 yaw 未稳定')
    monkeypatch.setattr(task, '_final_yaw', yaw)
    outcome = task.execute()
    assert not outcome
    assert attempted == [True]
    assert outcome.failure_code == '26rb_gate_task.timeout'
    assert not any(c[0] == 'action' and c[1] == mod.BasicMotion.Goal.BLINE for c in node.calls)


def test_overall_deadline_during_height_servo_does_not_force_pass(rig, monkeypatch):
    task, node, clock = rig
    clock.hook = lambda: frame(task, height=40, lock=False, count=1)
    original = task._fore_aft
    def servo():
        task._deadline = clock.time+.2
        original()
    monkeypatch.setattr(task, '_fore_aft', servo)
    outcome = task.execute()
    assert not outcome
    assert outcome.failure_code == '26rb_gate_task.timeout'
    assert not any('高度伺服超时，但固定门框' in log for log in node.logs)
    assert not any(c[0] == 'action' and c[1] == mod.BasicMotion.Goal.BLINE for c in node.calls)


@pytest.mark.parametrize('kind', ['motion', 'height_unreachable', 'observation_lost', 'cancelled', 'timeout'])
def test_other_height_stage_errors_do_not_relax_height_threshold(rig, monkeypatch, kind):
    task, node, clock = rig
    clock.hook = lambda: frame(task, height=40, lock=False, count=1)
    def servo():
        task._phase_deadline = clock.time+90
        raise mod.GateFailure(kind, 'injected transport/limit failure')
    monkeypatch.setattr(task, '_fore_aft', servo)
    outcome = task.execute()
    assert not outcome
    assert outcome.failure_code == '26rb_gate_task.'+kind
    assert not any('高度伺服超时，但固定门框' in log for log in node.logs)
    assert not any(c[0] == 'action' and c[1] == mod.BasicMotion.Goal.BLINE for c in node.calls)


def test_unconfirmed_motion_at_height_deadline_does_not_force_pass(rig, monkeypatch):
    task, node, clock = rig
    clock.hook = lambda: frame(task, height=40, lock=False, count=1)
    def servo():
        task._phase_deadline = clock.time
        task._cleanup_unconfirmed = True
        raise mod.GateFailure('timeout', '动作取消未确认')
    monkeypatch.setattr(task, '_fore_aft', servo)
    outcome = task.execute()
    assert not outcome
    assert not any('高度伺服超时，但固定门框' in log for log in node.logs)
    assert not any(c[0] == 'action' and c[1] == mod.BasicMotion.Goal.BLINE for c in node.calls)


def test_height_timeout_fallback_is_local_to_current_gate(rig, monkeypatch):
    task, _, clock = rig
    task.p.update(gate_count=2, observation_poses=[[1, 0, .2, 0], [2, 0, .3, 0]],
                  fore_aft_timeout=3.)
    clock.hook = lambda: frame(task, height=40 if task._gate_index == 0 else 70,
                               lock=False, count=1)
    required = []
    original = task._pass
    def passed(require_height=True):
        required.append(require_height)
        return original(require_height=require_height)
    monkeypatch.setattr(task, '_pass', passed)
    outcome = task.execute()
    assert outcome, outcome.message
    assert required == [False, True]
    assert '通过 2，跳过 0/1' in outcome.message


def test_fallback_still_checks_target_after_neutral_handoff(rig):
    task, node, _ = rig
    frame(task, height=40)
    task._velocity()
    original = node._send_body_velocity
    def send(*args, **kwargs):
        result = original(*args, **kwargs)
        if not args:
            frame(task, empty=True)
        return result
    node._send_body_velocity = send
    with pytest.raises(mod.GateFailure, match='交接前'):
        task._pass(require_height=False)
    assert not any(c[0] == 'action' for c in node.calls)


def test_corrected_gate_ray_and_tracking_box_do_not_undistort_twice(rig):
    from sensor_msgs.msg import CameraInfo
    from uv_camera.image_geometry import EyeUndistorter, normalized_pixel
    task,node,clock=rig
    source=CameraInfo(width=task.width,height=task.height,
        k=task.k['left'].reshape(-1).tolist(),d=[-.3,.1,.002,0.,0.])
    correction=EyeUndistorter(source,4)
    center=[450.,300.]
    origin,ray=task._camera_ray('left',center,node.pose,correction.image_info)
    xy=normalized_pixel(source.k,[],*center)
    expected=mod.rotation(node.pose)@task.extrinsics['left'].optical_to_body@np.array([*xy,1.])
    expected/=np.linalg.norm(expected)
    assert ray==pytest.approx(expected)
    bbox=(400.,250.,500.,350.)
    value=mod.GateFrame('left',1,clock.time,clock.time,1,bbox,tuple(node.pose),origin,ray,1.,1.,1)
    assert task._tracking_box(value)==pytest.approx(bbox)
    raw=mod.GateFrame('left',1,clock.time,clock.time,1,bbox,tuple(node.pose),origin,ray,1.,1.)
    task.distortion['left']=np.array(source.d)
    assert task._tracking_box(raw)!=pytest.approx(bbox)
