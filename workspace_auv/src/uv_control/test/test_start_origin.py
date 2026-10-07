"""START must acknowledge the MCU origin before ARM or any motion output."""

from __future__ import annotations

import ast
import asyncio
from pathlib import Path
import threading
import time
from types import SimpleNamespace as NS

import pytest

from uv_control.coordinate import Coordinate


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class Goal:
    def __init__(self, cmd_type=6, target=None, timeout=0.0):
        self.request = NS(cmd_type=cmd_type, target=target or [0.0] * 4,
                          timeout=timeout, axes='', task_context='', velocity_lease=0.25)
        self.is_cancel_requested = False
        self.terminal = None

    def succeed(self):
        self.terminal = 'succeeded'

    def abort(self):
        self.terminal = 'aborted'

    def canceled(self):
        self.terminal = 'canceled'


@pytest.fixture
def node():
    """Run production methods without needing generated ROS messages or DDS."""
    source = Path(__file__).parents[1] / 'uv_control/basic_motion.py'
    tree = ast.parse(source.read_text())
    cls = next(item for item in tree.body if isinstance(item, ast.ClassDef)
               and item.name == 'BasicMotionNode')
    namespace = {
        'Node': object, 'Coordinate': Coordinate, 'threading': threading,
        'time': time, 'math': __import__('math'),
        'wrap_deg': lambda value: (value + 180.0) % 360.0 - 180.0,
        'BasicMotion': NS(Goal=NS(START=6, BODY_VELOCITY=7), Result=NS),
        'GoalResponse': NS(ACCEPT='accept', REJECT='reject'),
        'Future': lambda executor=None: asyncio.Future(),
        'UInt32': NS, 'StateResetRequest': NS, 'StateResetResult': NS, 'ZitSetpoint': NS,
        'DEFAULT_VELOCITY_LEASE': 0.25, 'CK_POS': 0, 'CK_VEL_BODY': 0x11,
        'TOL_X': 0.1, 'TOL_Y': 0.1, 'TOL_Z': 0.1, 'TOL_RZ': 5.0,
    }
    exec(compile(ast.Module(body=[cls], type_ignores=[]), str(source), 'exec'), namespace)
    instance = namespace['BasicMotionNode'].__new__(namespace['BasicMotionNode'])
    logger = NS(info=lambda *_: None, error=lambda *_: None, warning=lambda *_: None)
    values = dict(
        _state_lock=threading.Lock(), _velocity_lock=threading.Lock(),
        _shutdown_requested=False, _safe_stop_latched=False,
        _start_in_progress=False, _motion_reserved=False, _started=False,
        _heartbeat_enabled=False, _active_origin_generation=None,
        _status_received_at=float('-inf'), _odom_received_at=float('-inf'),
        _nav_sample_at=float('-inf'), _nav_sample_key=None, _state_timeout=1.0,
        _start_timeout=0.15, _arm_mode=1, _origin_initialized=False,
        _nav_valid=True, _origin_generation=0, _nav_timestamp_ms=0,
        _reset_request_id=500, _pending_reset_id=None, _reset_result=None, _reset_invalidated=False,
        _start_waiter=None, _velocity_active=False, _velocity_deadline=0.0,
        _action_goal_handle=None, _action_target=None, executor=None,
        status=NS(is_armed=False), pose=Coordinate(), _target=Coordinate(),
        pub_setpoint=Publisher(), pub_arm_heartbeat=Publisher(),
        pub_state_reset=Publisher(), _timers=[], get_logger=lambda: logger,
    )
    instance.__dict__.update(values)
    return instance


def refresh(node, *, armed=None, origin=None, generation=None):
    now = time.monotonic()
    node._status_received_at = node._odom_received_at = node._nav_sample_at = now
    if armed is not None:
        node.status.is_armed = armed
    if origin is not None:
        node._origin_initialized = origin
    if generation is not None:
        node._origin_generation = generation


async def tick(node):
    node._start_wait_tick()
    await asyncio.sleep(0)


async def begin_reset(node, goal):
    node._start_in_progress = True
    task = asyncio.create_task(node._execute_start(goal))
    await asyncio.sleep(0)
    refresh(node, armed=False)
    await tick(node)
    await tick(node)
    assert len(node.pub_state_reset.messages) == 1
    return task, node.pub_state_reset.messages[-1].request_id


def test_start_requires_correlated_reset_and_matching_fresh_odom_before_heartbeat(node):
    async def scenario():
        goal = Goal()
        task, request_id = await begin_reset(node, goal)
        node._state_reset_result_cb(NS(request_id=request_id - 1, success=True,
                                      origin_generation=8, message='old'))
        await tick(node)
        assert not node._heartbeat_enabled and not task.done()
        node._state_reset_result_cb(NS(request_id=request_id, success=True,
                                      origin_generation=8, message=''))
        await tick(node)
        assert not node._heartbeat_enabled
        refresh(node, origin=True, generation=7)
        await tick(node)
        assert not node._heartbeat_enabled
        refresh(node, origin=True, generation=8)
        await tick(node)
        assert node._heartbeat_enabled and not task.done()
        assert not node._motion_ready()
        node._heartbeat_cb()
        assert node.pub_arm_heartbeat.messages[-1].data == 1
        refresh(node, armed=True)
        await tick(node)
        result = await task
        assert result.success and goal.terminal == 'succeeded'
        assert node._motion_ready()
    asyncio.run(scenario())


@pytest.mark.parametrize('failure', ['rejected_reset', 'cancel', 'timeout', 'missing_odom'])
def test_failed_or_cancelled_start_never_leaves_arm_heartbeat_enabled(node, failure):
    async def scenario():
        goal = Goal(timeout=0.015)
        task, request_id = await begin_reset(node, goal)
        if failure == 'rejected_reset':
            node._state_reset_result_cb(NS(request_id=request_id, success=False,
                                          origin_generation=0, message='nav invalid'))
        elif failure == 'cancel':
            goal.is_cancel_requested = True
        elif failure == 'missing_odom':
            node._state_reset_result_cb(NS(request_id=request_id, success=True,
                                          origin_generation=9, message=''))
        while not task.done():
            await tick(node)
            await asyncio.sleep(0.001)
        result = await task
        assert not result.success and not node._heartbeat_enabled
        assert not node._started and node._pending_reset_id is None
        assert goal.terminal == ('canceled' if failure == 'cancel' else 'aborted')
    asyncio.run(scenario())


def test_start_cancellation_after_origin_before_armed_stops_heartbeat(node):
    async def scenario():
        goal = Goal()
        task, request_id = await begin_reset(node, goal)
        node._state_reset_result_cb(NS(request_id=request_id, success=True,
                                      origin_generation=8, message=''))
        await tick(node)
        refresh(node, origin=True, generation=8)
        await tick(node)
        assert node._heartbeat_enabled
        goal.is_cancel_requested = True
        await tick(node)
        assert not (await task).success
        assert not node._heartbeat_enabled
    asyncio.run(scenario())


def test_start_waits_for_disarm_and_never_resets_armed_vehicle(node):
    async def scenario():
        goal = Goal(timeout=0.015)
        node._start_in_progress = True
        refresh(node, armed=True, origin=True, generation=3)
        node._started = node._heartbeat_enabled = True
        task = asyncio.create_task(node._execute_start(goal))
        await asyncio.sleep(0)
        assert not node._heartbeat_enabled
        while not task.done():
            refresh(node, armed=True)
            await tick(node)
            await asyncio.sleep(0.001)
        assert not (await task).success
        assert not node.pub_state_reset.messages
    asyncio.run(scenario())


@pytest.mark.parametrize('ready', [False, True])
def test_position_targets_keep_mcu_odom_values_without_second_origin_transform(node, ready):
    refresh(node, armed=True, origin=True, generation=12)
    node._started = node._heartbeat_enabled = ready
    node._active_origin_generation = 12
    if not ready:
        with pytest.raises(RuntimeError, match='motion inhibited'):
            node.set_world(4.0, -2.0, 1.5, 90.0)
        assert not node.pub_setpoint.messages
        return
    node.set_world(4.0, -2.0, 1.5, 90.0)
    message = node.pub_setpoint.messages[-1]
    assert [message.x, message.y, message.z, message.yaw] == pytest.approx(
        [4.0, -2.0, 1.5, __import__('math').pi / 2])
    assert [node._target.x, node._target.y, node._target.z, node._target.rz] == [4, -2, 1.5, 90]


def test_nonzero_velocity_requires_ready_but_internal_neutral_still_works(node):
    with pytest.raises(RuntimeError, match='motion inhibited'):
        node._publish_body_velocity(0.2, 0.0, 0.0, 0.0)
    assert not node.pub_setpoint.messages
    node._publish_body_velocity()
    assert node.pub_setpoint.messages[-1].x == 0.0
    goal = Goal(cmd_type=7, target=[0.2, 0.0, 0.0, 0.0])
    result = asyncio.run(node._action_execute_cb(goal))
    assert not result.success and goal.terminal == 'aborted'


@pytest.mark.parametrize('fault', ['stale_status', 'stale_nav', 'nav_invalid', 'new_generation', 'disarmed'])
def test_heartbeat_stops_when_started_state_is_lost(node, fault):
    refresh(node, armed=True, origin=True, generation=4)
    node._started = node._heartbeat_enabled = True
    node._active_origin_generation = 4
    if fault == 'stale_status':
        node._status_received_at -= 2.0
    elif fault == 'stale_nav':
        node._nav_sample_at -= 2.0
    elif fault == 'nav_invalid':
        node._nav_valid = False
    elif fault == 'new_generation':
        node._origin_generation = 5
    else:
        node.status.is_armed = False
    node._heartbeat_cb()
    assert not node._heartbeat_enabled and not node._motion_ready()
    assert not node.pub_arm_heartbeat.messages


def test_republishing_old_nav_sample_does_not_refresh_nav_age(node):
    message = NS(robot_x=0, robot_y=0, robot_z=0, robot_roll=0,
                 robot_pitch=0, robot_yaw=0, origin_initialized=True,
                 nav_valid=True, origin_generation=4, nav_timestamp_ms=17, stamp=NS())
    node._state_odom_cb(message)
    node._nav_sample_at -= 2.0
    stale_sample_at = node._nav_sample_at
    node._state_odom_cb(message)
    assert node._nav_sample_at == stale_sample_at
    message.nav_timestamp_ms += 1
    node._state_odom_cb(message)
    assert node._nav_sample_at > stale_sample_at


def test_start_and_other_position_goals_cannot_overlap(node):
    assert node._action_goal_cb(Goal().request) == 'accept'
    assert node._action_goal_cb(Goal().request) == 'reject'
    assert node._action_goal_cb(Goal(cmd_type=3).request) == 'reject'
    assert node._action_goal_cb(Goal(cmd_type=7).request) == 'reject'


def test_velocity_goal_preserves_active_position_cancel_handle(node):
    refresh(node, armed=True, origin=True, generation=4)
    node._started = node._heartbeat_enabled = True
    node._active_origin_generation = 4
    active = NS(is_cancel_requested=True)
    node._action_goal_handle = active
    result = asyncio.run(node._action_execute_cb(Goal(cmd_type=7)))
    assert result.success
    assert node._action_goal_handle is active


def test_body_targets_use_current_pose_instead_of_previous_goal(node):
    refresh(node, armed=True, origin=True, generation=12)
    node._started = node._heartbeat_enabled = True
    node._active_origin_generation = 12
    node.pose = Coordinate(x=4.0, y=3.0, z=2.0, rz=90.0)
    node._target = Coordinate(x=99.0, y=99.0, z=99.0, rz=0.0)
    node.set_body(1.0, 0.0, 0.5, 10.0)
    message = node.pub_setpoint.messages[-1]
    assert [message.x, message.y, message.z] == pytest.approx([4.0, 4.0, 2.5])
    assert message.yaw == pytest.approx(__import__('math').radians(100.0))


def test_start_preempts_previous_position_motion_and_waits_before_reset(node):
    async def scenario():
        refresh(node, armed=True, origin=True, generation=2)
        node._started = node._heartbeat_enabled = True
        node._active_origin_generation = 2
        node._motion_reserved = True
        old_goal = Goal(cmd_type=3)
        node._action_goal_handle = old_goal
        start_goal = Goal()
        assert node._action_goal_cb(start_goal.request) == 'accept'
        assert node._is_cancelled()
        task = asyncio.create_task(node._execute_start(start_goal))
        await asyncio.sleep(0)
        await tick(node)
        assert not node.pub_state_reset.messages
        assert node._action_goal_handle is old_goal
        node._motion_reserved = False
        await tick(node)
        assert node._action_goal_handle is start_goal
        refresh(node, armed=False)
        await tick(node)
        assert len(node.pub_state_reset.messages) == 1
        start_goal.is_cancel_requested = True
        await tick(node)
        assert not (await task).success
    asyncio.run(scenario())


def test_previous_action_cleanup_does_not_clear_new_start_handle(node):
    old_goal = Goal(cmd_type=3)
    start_goal = Goal()
    node._motion_reserved = True
    def interrupted_motion(_goal):
        node._action_goal_handle = start_goal
        node._action_target = {'start': True}
        return NS(success=False)
    node._execute_motion = interrupted_motion
    asyncio.run(node._action_execute_cb(old_goal))
    assert node._action_goal_handle is start_goal
    assert node._action_target == {'start': True}
    assert not node._motion_reserved


@pytest.mark.parametrize('had_origin', [False, True])
def test_mcu_restart_invalidates_pending_reset_and_discards_late_success(node, had_origin):
    node._origin_initialized = had_origin
    node._nav_timestamp_ms = 1000
    node._nav_sample_key = (4, 1000)
    node._pending_reset_id = 501
    node._start_in_progress = True
    message = NS(robot_x=0, robot_y=0, robot_z=0, robot_roll=0,
                 robot_pitch=0, robot_yaw=0, origin_initialized=False,
                 nav_valid=True, origin_generation=0, nav_timestamp_ms=1, stamp=NS())
    node._state_odom_cb(message)
    assert node._reset_invalidated and not node._reset_result.success
    node._state_reset_result_cb(NS(request_id=501, success=True, origin_generation=9))
    assert not node._reset_result.success
    assert not node._heartbeat_enabled


def test_body_odom_helpers_preserve_current_pose_translation_and_yaw(node):
    node.pose = Coordinate(x=4.0, y=3.0, z=2.0, rz=90.0)
    body = Coordinate(x=1.0, y=0.0, z=0.5, rz=10.0)
    odom = node._body_to_odom(body)
    assert [odom.x, odom.y, odom.z, odom.rz] == pytest.approx([4.0, 4.0, 2.5, 100.0])
    restored = node._odom_to_body(odom)
    assert [restored.x, restored.y, restored.z, restored.rz] == pytest.approx([1.0, 0.0, 0.5, 10.0])


def test_reboot_after_reset_reply_before_armed_aborts_start(node):
    async def scenario():
        goal = Goal()
        task, request_id = await begin_reset(node, goal)
        node._state_reset_result_cb(NS(request_id=request_id, success=True,
                                      origin_generation=8, message=''))
        await tick(node)
        refresh(node, origin=True, generation=8)
        await tick(node)
        assert node._heartbeat_enabled
        node._nav_sample_key = (8, 1000)
        node._nav_timestamp_ms = 1000
        node._state_odom_cb(NS(robot_x=0, robot_y=0, robot_z=0,
                             robot_roll=0, robot_pitch=0, robot_yaw=0,
                             origin_initialized=False, nav_valid=True,
                             origin_generation=0, nav_timestamp_ms=1, stamp=NS()))
        await tick(node)
        result = await task
        assert not result.success and 'restarted' in result.message
        assert not node._heartbeat_enabled
    asyncio.run(scenario())
