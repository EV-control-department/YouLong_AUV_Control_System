"""Position-holding descent acceptance checks without starting ROS or hardware."""
import math
from types import MethodType, SimpleNamespace as NS

import numpy as np
import pytest

from uv_task.config_loader import ConfigError, _validate_params
from uv_task.down_camera_servo import body_to_world_rotation
from test_26rb_grab_ball_ring import mod, rig
from test_gate_action_handoff import Future, method


HOLD_DEFAULTS = {
    'xy_hold_gain': .8, 'xy_hold_max_speed_mps': .05,
    'yaw_hold_gain': 1.2, 'yaw_hold_max_rate_deg_s': 10.,
}


def test_descent_hold_yaml_aliases_and_defaults(rig):
    _, task = rig
    expected = {'descent_'+key: value for key, value in HOLD_DEFAULTS.items()}
    assert _validate_params('26rb_grab_ball_ring', {'descent': HOLD_DEFAULTS}) == expected
    for key, value in expected.items():
        assert task.p[key] == value
    assert task.descent_speed == .03
    assert task.descent_period == .05
    assert task.descent_timeout == 15.
    with pytest.raises(ConfigError, match='重复'):
        _validate_params('26rb_grab_ball_ring', {
            'descent': {'xy_hold_gain': .8}, 'descent_xy_hold_gain': 1.})


@pytest.mark.parametrize('key', list(HOLD_DEFAULTS))
@pytest.mark.parametrize('value', [0., -1., float('nan'), float('inf'), True, 'bad'])
def test_descent_hold_parameters_reject_invalid_values(key, value):
    with pytest.raises(ValueError):
        mod.validate_combined_params({'descent_'+key: value})
    with pytest.raises(ConfigError):
        _validate_params('26rb_grab_ball_ring', {'descent': {key: value}})


def test_descent_corrects_fixed_world_xy_and_wrapped_yaw_at_nonzero_tilt(rig):
    node, task = rig
    node.pose[:] = [.9, -.3, .2, 12., -8., 170.]
    node._cmd_x, node._cmd_y, node._cmd_yaw = .5, 0., -170.
    task.grab_depths['golf'] = .23
    initial_error = np.linalg.norm(node.pose[:2]-[.5, 0.])
    samples = []
    original = node._send_body_velocity

    def record(*args, **kwargs):
        if args:
            samples.append((body_to_world_rotation(node.pose) @ np.asarray(args),
                            kwargs['yaw_rate_deg_s']))
        return original(*args, **kwargs)

    node._send_body_velocity = record
    assert task._descend_to_depth('golf', task.ball)
    assert task._descent_anchor == (.5, 0., -170.)
    assert samples[0][0][:2] == pytest.approx(
        np.array([-.4, .3])/.5*task.descent_xy_max_speed)
    assert samples[0][1] == 10.  # +20 degrees across the wrap, not -340.
    for world, yaw_rate in samples:
        assert world[2] == pytest.approx(.03, abs=1e-12)
        assert np.linalg.norm(world[:2]) <= .05+1e-12
        assert abs(yaw_rate) <= 10.
    assert np.linalg.norm(node.pose[:2]-[.5, 0.]) < initial_error
    assert np.allclose(node.velocity, 0.)


def test_descent_timeout_stops_with_independent_two_second_budget(rig):
    node, task = rig
    task.grab_depths['golf'] = 2.
    started = node.clock
    assert not task._descend_to_depth('golf', task.ball)
    assert node.clock-started == pytest.approx(15.)
    moving = [call for call in node.calls if np.any(call[2])]
    assert all(0. < call[3]['lease_s'] <= .25 for call in moving)
    assert np.diff([call[1] for call in moving]) == pytest.approx(
        np.full(len(moving)-1, .05))
    neutral = node.calls[-1]
    assert np.allclose(neutral[2], 0.)
    assert neutral[3]['wait_deadline']-neutral[1] == pytest.approx(2.)
    assert neutral[3]['cancel_wait_timeout'] == 0.
    assert all(call[3]['light_color'] == node.LIGHT_OFF for call in node.calls)
    assert not node.stopped


@pytest.mark.parametrize('depth', [.44, .50])
def test_already_at_or_below_depth_never_commands_further_descent(rig, depth):
    node, task = rig
    node.pose[2] = depth
    assert task._descend_to_depth('golf', task.ball)
    assert len(node.calls) == 1 and np.allclose(node.calls[0][2], 0.)


def test_final_reply_at_descent_deadline_with_confirmed_cleanup_is_normal_timeout(rig):
    node, task = rig
    task.descent_timeout = .15
    original = node._send_body_velocity

    def final_reply(*args, **kwargs):
        result = original(*args, **kwargs)
        if args:
            node.clock = kwargs['wait_deadline']
            node.pose_received_at = node.clock
            node._last_motion_failure_kind = 'timeout'
            return False, '等待动作结果超时'
        return result

    node._send_body_velocity = final_reply
    assert not task._descend_to_depth('ring', task.ring)
    assert not node.stopped and np.allclose(node.velocity, 0.)


def prepare_mechanical_attempt(node, task, kind):
    controller = task.ball if kind == 'golf' else task.ring
    node._cmd_x, node._cmd_y, node._cmd_yaw = .55, -.05, 90.
    node.pose[5] = 90.
    task._align_claw = lambda *args: None
    task._orient_and_align_ring = lambda: None
    task.ball._prepare_claw = lambda: True
    controller._wait_pre_descent_settle = lambda: True
    task.state.update_pickup(**{kind+'_camera_pose': (.6, .1, .2, -45.)})
    return controller


@pytest.mark.parametrize('kind', ['golf', 'ring'])
@pytest.mark.parametrize('timed_out', [False, True])
def test_reached_and_timed_out_attempts_close_then_ascend_once_at_fixed_anchor(rig, kind, timed_out):
    node, task = rig
    controller = prepare_mechanical_attempt(node, task, kind)
    task.grab_depths[kind] = 2. if timed_out else .203
    if timed_out:
        task.descent_timeout = .15
    assert task._mechanical_attempt(kind, controller)
    stopped_depth = node.pose[2]
    assert stopped_depth > .2
    if timed_out:
        assert stopped_depth < 2.
    closes = [stamp for stamp, servo, angle in node.servos if servo == 2 and angle == 90.]
    actions = [call for call in node.calls if call[0] == 'action']
    if kind == 'ring':
        assert len(closes) == 3
        assert len(actions) == 1
        assert actions[0][2] == pytest.approx([.55, -.05, stopped_depth, 90.])
        assert actions[0][3] == 'xyzrz'
        assert np.allclose(node.velocity, 0.)
    else:
        assert not closes and not actions
    # Recovery must keep the frozen command even if the measurement drifts.
    node.pose[:2] += [.02, -.03]
    node.pose[5] = 80.
    task._return_frame(controller)
    actions = [call for call in node.calls if call[0] == 'action']
    ascent, camera_return = actions[-2:]
    assert ascent[2] == pytest.approx([.55, -.05, .2, 90.])
    assert ascent[3] == 'xyzrz' and ascent[4]['timeout'] == 10.
    assert camera_return[2] == [.6, .1, .2, -45.]
    assert camera_return[3] == 'xyrz'
    assert sum(call[4]['task_context'] == '组合抓取：抓取后恢复作业深度' for call in actions) == 1
    assert all(call[4]['light_color'] == node.LIGHT_OFF for call in actions)
    assert task._descent_anchor is None


@pytest.mark.parametrize('kind', ['golf', 'ring'])
def test_normal_timeout_still_enters_check_and_can_confirm_grab(rig, kind):
    node, task = rig
    controller = prepare_mechanical_attempt(node, task, kind)
    task.grab_depths[kind] = 2.
    task.descent_timeout = .15
    checking_calls = []

    def align(*args, checking=False, **kwargs):
        checking_calls.append(checking)
        return 'absent' if checking else 'ready'

    task._align_target = align
    task._run_target(kind, controller)
    assert checking_calls == [False, True]
    assert getattr(task.state.pickup, kind+'_status') == 'success'
    assert getattr(task.state.pickup, kind+'_attempts') == 1


@pytest.mark.parametrize('failure', ['stale', 'invalid', 'cancel', 'unconfirmed'])
def test_descent_fault_neutralizes_without_closing_or_ascending(rig, failure):
    node, task = rig
    controller = prepare_mechanical_attempt(node, task, 'ring')
    if failure == 'stale':
        node.update_pose_stamp = False
    elif failure == 'invalid':
        node.hook = lambda: node.pose.__setitem__(0, float('nan'))
    elif failure == 'cancel':
        original = node._send_body_velocity

        def cancel_during_descent(*args, **kwargs):
            result = original(*args, **kwargs)
            if args:
                node.stopped = True
            return result

        node._send_body_velocity = cancel_during_descent
    else:
        original = node._send_body_velocity

        def unconfirmed(*args, **kwargs):
            result = original(*args, **kwargs)
            if args:
                node._last_motion_cleanup_confirmed = False
                return False, '模拟下压动作终态未确认'
            return result

        node._send_body_velocity = unconfirmed
    with pytest.raises((mod.PickupFailure, ValueError)):
        task._mechanical_attempt('ring', controller)
    assert node.calls[-1][0] == 'velocity' and np.allclose(node.velocity, 0.)
    assert not any(servo == 2 and angle == 90. for _, servo, angle in node.servos)
    assert not any(call[0] == 'action' for call in node.calls)
    if failure == 'unconfirmed':
        assert node.stopped  # The later successful zero must not mask the old goal.


def test_ascent_waits_for_measured_depth_before_horizontal_camera_return(rig):
    node, task = rig
    task._descent_anchor = (.55, -.05, 90.)
    task.state.update_pickup(golf_camera_pose=(.6, .1, .2, -45.))
    original = node._send_action_goal

    def early_arrival(command, target, axes, **kwargs):
        result = original(command, target, axes, **kwargs)
        if axes == 'xyzrz':
            node.pose[2] = .30  # Generic SET arrival is too loose for recovery.
        return result

    node._send_action_goal = early_arrival
    node.hook = lambda: node.pose.__setitem__(2, .2)
    task._return_frame(task.ball)
    assert node.pose[2] == .2
    assert node.clock > 100.
    assert node.calls[-1][3] == 'xyrz'


def test_ascent_depth_confirmation_timeout_blocks_horizontal_return(rig):
    node, task = rig
    task._descent_anchor = (.55, -.05, 90.)
    task.state.update_pickup(golf_camera_pose=(.6, .1, .2, -45.))
    original = node._send_action_goal

    def stalled_ascent(*args, **kwargs):
        result = original(*args, **kwargs)
        node.pose[2] = .30
        return result

    node._send_action_goal = stalled_ascent
    with pytest.raises(mod.PickupFailure, match='深度确认超时'):
        task._return_frame(task.ball)
    assert len(node.calls) == 1 and node.calls[0][3] == 'xyzrz'


def test_runner_fresh_pose_option_and_callback_keep_legacy_callers_compatible():
    now = [0.]
    namespace = {'math': math, 'time': NS(monotonic=lambda: now[0])}
    get_pose, callback = (method(name, namespace)
                          for name in ('_latest_robot_pose', '_pose_cb'))
    import threading
    node = NS(_perception_lock=threading.RLock(), _robot_pose=None,
              _robot_pose_received_at=None, _cmd_x=1., _cmd_y=2., _cmd_z=.2, _cmd_yaw=90.)
    assert get_pose(node) == (1., 2., .2, 0., 0., 90.)
    with pytest.raises(ValueError, match='odom'):
        get_pose(node, require_measured=True, max_age=1.)
    callback(node, NS(robot_x=.5, robot_y=0., robot_z=.2,
                      robot_roll=12., robot_pitch=-8., robot_yaw=35.))
    assert node._robot_pose_received_at == 0.
    now[0] = 1.
    assert get_pose(node, require_measured=True, max_age=1.) == node._robot_pose
    now[0] += .001
    with pytest.raises(ValueError, match='odom'):
        get_pose(node, require_measured=True, max_age=1.)
    assert get_pose(node, require_measured=True) == node._robot_pose
    callback(node, NS(robot_x=float('nan'), robot_y=0., robot_z=.2,
                      robot_roll=0., robot_pitch=0., robot_yaw=0.))
    with pytest.raises(ValueError, match='odom'):
        get_pose(node, require_measured=True, max_age=1.)


def test_real_runner_late_velocity_acceptance_cannot_handoff_to_ascent(rig):
    node, task = rig
    namespace = {'BasicMotion': NS(Goal=type('Goal', (), {
        'START': 6, 'SET': 3, 'BODY_VELOCITY': 7})),
        'rclpy': NS(ok=lambda: True), 'time': mod.time}
    node._send_action_goal = MethodType(method('_send_action_goal', namespace), node)
    node._send_body_velocity = MethodType(method('_send_body_velocity', namespace), node)
    node._debug_timeout = 0.
    node._active_goal_handle = node._pending_goal_send_future = None
    pending, cancelled, sent = Future(), [], []
    result = NS(result=NS(success=True, final_target=[], message=''))
    handle = NS(accepted=True, get_result_async=lambda: Future(result),
                cancel_goal_async=lambda: cancelled.append(True))

    def send(goal):
        sent.append(goal)
        return pending if any(goal.target) else Future(handle)

    node._action_client = NS(wait_for_server=lambda **kwargs: True, send_goal_async=send)
    with pytest.raises(mod.PickupFailure, match='结束未确认'):
        task._descend_to_depth('golf', task.ball)
    assert node.stopped
    assert len(sent) == 2 and not any(sent[-1].target)
    assert node._pending_goal_send_future is pending
    pending.resolve(handle)
    assert cancelled == [True]
