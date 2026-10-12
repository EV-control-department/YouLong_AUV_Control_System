"""Preset gate stages against a virtual odom/action server; no ROS or hardware."""
import __future__
import ast
from dataclasses import dataclass
import math
from pathlib import Path
import threading
import time
from types import SimpleNamespace as NS

import numpy as np
import pytest

from uv_task.config_loader import ConfigError, _validate_params, load_task
from uv_task.gate_config import gate_uses_vision, validate_gate_params
from uv_task.task_outcome import TaskOutcome

ROOT = Path(__file__).parents[1]
GOAL = NS(SET=3, BLINE=8)


def production_task():
    """Load the actual class while excluding ROS-only imports."""
    path = ROOT/'uv_task/26rb_gate_task.py'
    tree = ast.parse(path.read_text())
    namespace = dict(__name__=__name__, dataclass=dataclass, math=math, np=np,
                     threading=threading, time=time, rclpy=NS(ok=lambda: True),
                     BasicMotion=NS(Goal=GOAL), validate_gate_params=validate_gate_params,
                     TaskOutcome=TaskOutcome, DetectionArray=object,
                     PERCEPTION_DETECTIONS='detections', QoSProfile=lambda **kw: kw,
                     ReliabilityPolicy=NS(BEST_EFFORT=1))
    declarations = [item for item in tree.body if isinstance(item, (ast.ClassDef, ast.FunctionDef))]
    exec(compile(ast.Module(body=declarations, type_ignores=[]), str(path), 'exec',
                 flags=__future__.annotations.compiler_flag, dont_inherit=True), namespace)
    return namespace['RB26GateTask'], namespace['GateFailure']


Task, GateFailure = production_task()
POSES = [[0., 0., .2, 90.], [0., 2., .5, -60.], [2., 2., .3, 15.],
         [2., 4., .7, 160.], [4., 4., .4, -85.]]


class Clock:
    def __init__(self):
        self.t = 100.
        self.node = None

    def sleep(self, dt):
        self.t += dt
        node = self.node
        if node.depth_goal is not None and not node.freeze_depth:
            error = node.depth_goal-node.pose[2]
            node.pose[2] += float(np.clip(error, -.1*dt, .1*dt))


class Node:
    LIGHT_OFF, LIGHT_YELLOW, LIGHT_GREEN, LIGHT_RED = range(4)
    stopped = False

    def __init__(self, clock, vision=False):
        clock.node = self
        self.clock = clock
        self.pose = np.array([-2., -3., .1, 0., 0., 50.])
        self.actions, self.events, self.logs, self.velocity_calls = [], [], [], []
        self._last_motion_cleanup_confirmed = True
        self._last_motion_cancelled_by_condition = False
        self._last_motion_final_target = None
        self._active_goal_handle = None
        self.condition_cleanup = True
        self.condition_success = True
        self.natural_completion_race = False
        self.yaw_offset = 0.
        self.depth_gap = 0.
        self.depth_goal = None
        self.freeze_depth = False
        if vision:
            side = NS(matrix=np.eye(3), distortion=[])
            self.camera_configs = {'front': NS(eye_resolution=(640, 480), side=lambda _: side)}
            self.camera_extrinsics = dict(front_left=None, front_right=None)
            self._model_mapping = NS(model_class_id=lambda *a, **kw: 7)

    def get_logger(self):
        return NS(info=self.logs.append, warn=self.logs.append)

    def get_clock(self):
        return NS(now=lambda: NS(nanoseconds=int(self.clock.t*1e9)))

    def _latest_robot_pose(self, require_measured=False):
        assert require_measured
        return tuple(self.pose)

    def create_subscription(self, *args):
        self.events.append(('subscribe',))
        return object()

    def destroy_subscription(self, _):
        self.events.append(('destroy_subscription',))

    def _flash_task_light(self, color, count, *args, **kw):
        self.events.append(('flash', color, count))

    def _set_task_phase_light(self, *args, **kw):
        pass

    def _send_body_velocity(self, *args, **kw):
        self.velocity_calls.append((args, kw))
        return True, ''

    def _send_action_goal(self, command, target, axes, **kw):
        call = NS(command=command, target=list(target), axes=axes, kwargs=kw,
                  start=self.pose.copy(), t=self.clock.t)
        self.actions.append(call)
        self.events.append(('action', command, axes))
        self._last_motion_cleanup_confirmed = True
        self._last_motion_cancelled_by_condition = False
        if command == GOAL.SET:
            linear = axes.replace('rz', '')
            for i, axis in enumerate('xyz'):
                if axis in linear:
                    self.pose[i] = target[i]
            if 'rz' in axes:
                self.pose[5] = target[3]+(self.yaw_offset if axes == 'rz' else 0.)
            if axes == 'z' and self.depth_gap:
                self.pose[2] = target[2]-self.depth_gap
                self.depth_goal = target[2]
            else:
                self.depth_goal = None
            self.clock.t += .01
            return True, ''
        assert command == GOAL.BLINE
        yaw = math.radians(self.pose[5])
        delta = np.array([target[0]*math.cos(yaw)-target[1]*math.sin(yaw),
                          target[0]*math.sin(yaw)+target[1]*math.cos(yaw), target[2]])
        start, end = self.pose[:3].copy(), self.pose[:3]+delta
        self.depth_goal = None
        steps = max(1, math.ceil(float(np.linalg.norm(delta))/(kw['cruise_speed']*.05)))
        predicate = kw.get('cancel_predicate')
        for i in range(steps+1):
            self.pose[:3] = start+delta*(i/steps)
            if predicate is not None and not self.natural_completion_race and predicate():
                self.events.append(('cancel_requested',))
                self.clock.t += .1  # terminal confirmation is separate from the request
                self._last_motion_cleanup_confirmed = self.condition_cleanup
                self._last_motion_cancelled_by_condition = True
                if self.condition_cleanup:
                    self.events.append(('cancel_terminal',))
                return self.condition_success, 'LINE interrupted by cancellation'
            if self.clock.t >= kw['wait_deadline']:
                return False, 'LINE total timeout'
            self.clock.sleep(.05)
        self._last_motion_final_target = [*end, self.pose[5]]
        self.events.append(('line_terminal',))
        return True, ''


def rig(code='1111', count=4, poses=None, **overrides):
    params = dict(fixed_pass_code=code, gate_count=count,
                  observation_poses=POSES if poses is None else poses, pass_speed_mps=.2)
    params.update(overrides)
    clock = Clock()
    node = Node(clock, vision=gate_uses_vision(params))
    task = Task(node, params)
    task._now = lambda: clock.t
    task._sleep = clock.sleep
    return task, node, clock


def lines(node):
    return [call for call in node.actions if call.command == GOAL.BLINE]


def test_yaml_defaults_and_binary_type():
    params = load_task(ROOT/'config/tasks/26rb_gate_task.yaml')[0]['params']
    assert params['fixed_pass_code'] == '0000'
    assert params['observation_xy_radius_m'] == [.6]*5
    assert params['observation_travel_timeout'] == 60.
    assert params['pass_speed_mps'] == .2
    with pytest.raises(ConfigError):
        _validate_params('26rb_gate_task', {'fixed_pass': {'code': 1111}})


@pytest.mark.parametrize('code', ['', '000', '00000', '0012', 1111, True, None])
def test_invalid_codes(code):
    with pytest.raises(ValueError):
        validate_gate_params({'fixed_pass_code': code})


@pytest.mark.parametrize('radii', [[], [0.], [-.1], [math.nan], [math.inf], [True], [[.6]], [.6]*6])
def test_invalid_radii(radii):
    with pytest.raises(ValueError):
        validate_gate_params({'observation_xy_radius_m': radii})


def test_exit_and_radius_validation_and_partial_overrides():
    params = dict(gate_count=4, fixed_pass_code='0001', observation_poses=POSES[:4])
    with pytest.raises(ValueError, match='第 5 条'):
        validate_gate_params(params, complete=True)
    params['observation_poses'] = POSES
    assert validate_gate_params(params, complete=True)['observation_poses'] == POSES
    with pytest.raises(ValueError, match='半径|xy_radius'):
        validate_gate_params({**params, 'observation_xy_radius_m': [.6]*4}, complete=True)
    assert validate_gate_params({'fixed_pass_code': '1111'})['fixed_pass_code'] == '1111'
    assert gate_uses_vision({**params, 'gate_count': 3})
    assert validate_gate_params({**params, 'depth_front': [pose[2] for pose in POSES[:4]]},
                                complete=True)['observation_poses'][-1] == POSES[-1]


def test_default_dispatch_keeps_visual_stages_and_active_prefix():
    task, node, _ = rig(code='0000')
    visited = []
    task._run_gate = lambda: visited.append((task._gate_index, task._observe_after_pass))
    task._run_fixed_gate = lambda: pytest.fail('default mode entered preset branch')
    assert task.execute()
    assert visited == [(0, False), (1, True), (2, True), (3, True)]
    assert not node.actions


def test_visual_flow_still_observes_then_aligns_and_passes():
    task, _, _ = rig(code='0000')
    task._observe_after_pass = True
    visited = []
    frame = object()
    task._observe_next_gate_here = lambda: frame
    task._reach_observation = lambda: pytest.fail('already observed next gate')
    task._search = lambda: pytest.fail('already have current bbox')
    task._align = lambda selected: visited.append(('align', selected))
    task._fore_aft = lambda: visited.append('fore_aft')
    task._flash = lambda *args: visited.append('flash')
    task._final_yaw = lambda: visited.append('final_yaw')
    task._pass = lambda: visited.append('pass')
    task._run_gate()
    assert visited == [('align', frame), 'fore_aft', 'flash', 'final_yaw', 'pass']


def test_all_fixed_routes_initialize_first_pose_and_finish_at_exit():
    radii = [.01, .4, .5, .6, .2]
    task, node, _ = rig(observation_xy_radius_m=radii)
    task._largest_frame = lambda: pytest.fail('preset branch read a bbox')
    assert not task.uses_vision and task._sub is None
    outcome = task.execute()
    assert outcome and '通过 4' in outcome.message
    full_sets = [call for call in node.actions if call.command == GOAL.SET and call.axes == 'xyzrz']
    assert [call.target for call in full_sets] == POSES
    assert node.events[0] == ('action', GOAL.SET, 'xyzrz')
    assert node.events[1] == ('flash', node.LIGHT_GREEN, 1)
    assert [event for event in node.events if event[0] == 'flash'] == [('flash', node.LIGHT_GREEN, 1)]*4
    depth_sets = [call for call in node.actions if call.command == GOAL.SET and call.axes == 'z']
    assert len(lines(node)) == 8
    for index, call in enumerate(depth_sets):
        assert math.dist(call.start[:2], POSES[index+1][:2]) <= radii[index+1]+1e-9
        assert call.target[2] == POSES[index+1][2]
        assert lines(node)[2*index].target[2] == pytest.approx(0.)
        assert lines(node)[2*index+1].start[2] == POSES[index+1][2]
    for call in lines(node):
        assert call.kwargs['timeout'] == pytest.approx(60.)
        assert call.kwargs['cruise_speed'] == .2
        assert call.kwargs['cancel_wait_timeout'] == 2.
    assert node.events.count(('cancel_terminal',)) == 4
    for i, event in enumerate(node.events):
        if event == ('action', GOAL.SET, 'z'):
            assert node.events[i-1] == ('cancel_terminal',)
    assert list(node.pose[[0, 1, 2, 5]]) == POSES[-1]
    assert [node._cmd_x, node._cmd_y, node._cmd_z, node._cmd_yaw] == POSES[-1]
    assert not node.velocity_calls
    task.destroy()


def test_body_displacement_uses_measured_yaw_after_aim():
    task, node, _ = rig(code='1000', count=1)
    node.yaw_offset = 3.
    assert task.execute()
    line = lines(node)[0]
    assert line.start[5] == 93.
    assert abs(line.target[1]) > .01
    yaw = math.radians(line.start[5])
    xy = [line.start[0]+line.target[0]*math.cos(yaw)-line.target[1]*math.sin(yaw),
          line.start[1]+line.target[0]*math.sin(yaw)+line.target[1]*math.cos(yaw)]
    assert xy == pytest.approx(POSES[1][:2])


@pytest.mark.parametrize('distance, expected_lines', [(0., 0), (.3, 1), (.6, 1)])
def test_already_inside_radius_ignores_z_and_skips_first_line(distance, expected_lines):
    poses = [[9., 9., .2, 0.], [8., 8., .2, 0.], [0., 0., 1.5, 20.]]
    task, node, _ = rig(code='1100', count=2, poses=poses)
    task._gate_index = 1
    node.pose = np.array([distance, 0., .1, 0., 0., 0.])
    task._run_fixed_gate()
    assert len(lines(node)) == expected_lines
    assert not any(call.kwargs.get('cancel_predicate') for call in lines(node))
    assert not any(call.axes == 'xyzrz' and call.target == poses[1] for call in node.actions)
    assert list(node.pose[[0, 1, 2, 5]]) == poses[2]


def test_terminal_race_at_endpoint_still_changes_depth():
    task, node, _ = rig(code='1000', count=1)
    node.natural_completion_race = True
    assert task.execute()
    assert len(lines(node)) == 1
    assert not node._last_motion_cancelled_by_condition
    assert list(node.pose[[0, 1, 2, 5]]) == POSES[1]


def test_depth_hold_waits_for_measured_tolerance_without_velocity():
    task, node, _ = rig(code='1000', count=1)
    node.depth_gap = .08
    assert task.execute()
    depth_set = next(call for call in node.actions if call.axes == 'z')
    remaining_line = lines(node)[1]
    assert remaining_line.t-depth_set.t >= .5
    assert abs(remaining_line.start[2]-POSES[1][2]) <= task.p['depth_tolerance_m']
    assert not node.velocity_calls


def test_depth_timeout_does_not_start_remaining_line_or_count_success():
    task, node, _ = rig(code='1000', count=1, depth_set_timeout=.3)
    node.depth_gap, node.freeze_depth = .08, True
    task._park = lambda: True
    outcome = task.execute()
    assert not outcome and outcome.failure_code.endswith('.timeout')
    assert '通过 0' in outcome.message
    assert len(lines(node)) == 1
    assert node.actions[-1].axes == 'z'


@pytest.mark.parametrize('action_success', [False, True])
def test_unconfirmed_radius_cancellation_blocks_depth_set(action_success):
    task, node, _ = rig(code='1000', count=1)
    node.condition_cleanup, node.condition_success = False, action_success
    task._park = lambda: False
    outcome = task.execute()
    assert not outcome
    assert task._cleanup_unconfirmed
    assert '禁止进入下一门' in outcome.message
    assert not any(call.axes == 'z' for call in node.actions)
    assert len(lines(node)) == 1


def test_mixed_modes_use_next_pose_and_preserve_after_pass_observation():
    task, node, _ = rig(code='0101')
    visited = []
    def visual():
        visited.append((task._gate_index, task._observe_after_pass))
        node.pose[:3] = [task._gate_index, task._gate_index+1., .2]
    task._run_gate = visual
    outcome = task.execute()
    assert outcome and '通过 4' in outcome.message
    assert visited == [(0, False), (2, True)]
    full_sets = [call.target for call in node.actions if call.axes == 'xyzrz']
    assert full_sets == [POSES[2], POSES[4]]


def test_runner_skips_gate_camera_check_only_when_active_stages_are_fixed():
    path = ROOT/'uv_task/task_runner.py'
    tree = ast.parse(path.read_text())
    cls = next(item for item in tree.body if isinstance(item, ast.ClassDef) and item.name == 'TaskRunnerNode')
    method = next(item for item in cls.body if isinstance(item, ast.FunctionDef) and item.name == '_task_pass_gates')
    destroyed = []
    namespace = dict(TaskOutcome=TaskOutcome, gate_uses_vision=gate_uses_vision,
                     RB26GateTask=lambda *args: NS(execute=lambda: TaskOutcome.ok(),
                                                   destroy=lambda: destroyed.append(True)))
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec',
                 flags=__future__.annotations.compiler_flag, dont_inherit=True), namespace)
    run = namespace['_task_pass_gates']
    node = NS(_ensure_camera_extrinsics=lambda: pytest.fail('fixed task requested camera TF'))
    assert run(node, dict(fixed_pass_code='1000', gate_count=1, observation_poses=POSES[:2]))
    assert destroyed == [True]
    node._ensure_camera_extrinsics = lambda: False
    assert not run(node, dict(fixed_pass_code='0000'))


@pytest.mark.parametrize('exit_pose', [[1., 2., .3], [1., 2., -.3, 0.],
                                       [math.nan, 2., .3, 0.], [[1.], 2., .3, 0.]])
def test_invalid_fifth_pose_is_rejected_before_motion(exit_pose):
    with pytest.raises(ValueError):
        validate_gate_params(dict(gate_count=4, fixed_pass_code='1111',
                                  observation_poses=POSES[:4]+[exit_pose]), complete=True)


def test_fixed_final_set_failure_does_not_count_as_passed():
    task, node, _ = rig(code='1000', count=1)
    send = node._send_action_goal
    def fail_final(command, target, axes, **kw):
        if command == GOAL.SET and axes == 'xyzrz' and list(target) == POSES[1]:
            return False, 'final SET failed'
        return send(command, target, axes, **kw)
    node._send_action_goal = fail_final
    task._park = lambda: True
    outcome = task.execute()
    assert not outcome and '通过 0' in outcome.message
    assert outcome.failure_code.endswith('.motion')


def test_confirmed_fixed_failure_keeps_skip_budget_and_advances_stage():
    task, node, _ = rig(code='1100', count=2, poses=POSES[:3], max_failures=1)
    send = node._send_action_goal
    failed = []
    def fail_first_line(command, target, axes, **kw):
        if command == GOAL.BLINE and not failed:
            failed.append(True)
            return False, 'LINE total timeout'
        return send(command, target, axes, **kw)
    node._send_action_goal = fail_first_line
    task._park = lambda: True
    outcome = task.execute()
    assert outcome and '通过 1' in outcome.message and '跳过 1/1' in outcome.message
    full_sets = [call.target for call in node.actions if call.axes == 'xyzrz']
    assert full_sets == [POSES[0], POSES[2]]
