"""Run the production handoff against ZIT6's mode/nav acceptance rules."""
import asyncio
from types import SimpleNamespace as NS

import pytest

from test_start_origin import Goal, node  # noqa: F401
from uv_control.coordinate import Coordinate


class MCUClock:
    def __init__(self, node, *, nav=True, feedback=True, fault=None):
        self.now = 100.0
        self.node = node
        self.nav = nav
        self.feedback = feedback
        self.fault = fault
        self.mode = 2
        self.velocity = [.04, .02, 0., 0.]
        self.thrust = [.2, 0., 0., 0.]
        self.position = None
        self.position_reported_at = None
        self.position_resets = 0
        self.commands = []
        node._started = node._heartbeat_enabled = node._origin_initialized = True
        node._active_origin_generation = node._origin_generation = 4
        node._nav_valid = True  # Simulates force_nav_valid masking native failure.
        node.status = NS(is_armed=True, navigation_ready=nav, control_level=2)
        node._status_received_at = node._odom_received_at = node._nav_sample_at = self.now
        node.pose = Coordinate(x=1.2, y=3.4, z=.15, rz=-110.)
        node._target = Coordinate(x=9., y=8., z=.8, rz=90.)
        node._velocity_active, node._velocity_deadline = True, self.now+.8
        # The host has forgotten the MCU's persistent additive thrust.
        node._body_thrust_active = False
        original_publish = node.pub_setpoint.publish

        def publish(message):
            original_publish(message)
            values = [message.x, message.y, message.z, message.yaw]
            self.commands.append((self.now, message.control_key, values))
            if message.control_key == 0x12:
                assert message.type_mask == 0
                self.mode, self.thrust = 3, values
            elif self.nav:
                mode = 2 if message.control_key == 0x11 else 1
                self.position_resets += int(mode == 1 and self.mode != 1)
                self.mode = mode
                if mode == 2:
                    self.velocity = values
                else:
                    self.position = values

        node.pub_setpoint.publish = publish
        node._execute_motion.__globals__['time'] = self

    def monotonic(self):
        return self.now

    def sleep(self, dt):
        self.now += dt
        self.node._odom_received_at = self.node._nav_sample_at = self.now
        if self.feedback:
            self.node.status.navigation_ready = self.nav
            self.node.status.control_level = self.mode
            self.node._status_received_at = self.now
            if self.mode == 1 and self.position_reported_at is None:
                self.position_reported_at = self.now
        if self.fault and self.now >= 100.8:
            self.fault()


def run(node, clock, goal=None):
    goal = goal or Goal(12, timeout=5.)
    goal.request.velocity_lease = 2.
    assert node._action_goal_cb(goal.request) == 'accept'
    return goal, asyncio.run(node._action_execute_cb(goal))


def test_clears_forgotten_outputs_and_dwells_after_native_position_confirmation(node):
    clock = MCUClock(node)
    goal, result = run(node, clock)
    assert result.success and goal.terminal == 'succeeded'
    assert clock.now-clock.position_reported_at >= 2.
    assert clock.now-clock.position_reported_at <= 2.06
    assert [key for _, key, _ in clock.commands[:3]] == [0x12, 0x11, 0]
    assert clock.thrust == clock.velocity == [0.]*4
    assert clock.mode == 1 and clock.position_resets == 1
    assert result.final_target == pytest.approx([1.2, 3.4, .15, -110.])
    assert not node._velocity_active and node._velocity_deadline == 0.
    assert not node._body_thrust_active and not node._motion_reserved
    assert not node._handoff_reserved and node._action_goal_handle is None
    assert node._origin_generation == node._active_origin_generation == 4
    assert node._heartbeat_enabled and node.status.is_armed
    assert not node.pub_state_reset.messages and not node.pub_arm_heartbeat.messages
    # A subsequent initial SET can acquire motion ownership immediately.
    assert node._action_goal_cb(Goal(3, [5., 2., .2, -110.]).request) == 'accept'


def test_raw_nav_failure_cannot_be_hidden_by_host_override(node):
    clock = MCUClock(node, nav=False)
    goal, result = run(node, clock)
    assert not result.success and goal.terminal == 'aborted'
    assert 'confirmation timed out' in result.message
    assert clock.now <= 103.06
    assert clock.mode == 3 and clock.thrust == [0.]*4
    assert clock.position is None and not node._velocity_active


def test_stale_position_feedback_does_not_confirm_new_hold(node):
    clock = MCUClock(node, feedback=False)
    node.status.control_level = 1
    node._state_timeout = 10.
    _, result = run(node, clock)
    assert not result.success and 'confirmation timed out' in result.message
    assert clock.thrust == clock.velocity == [0.]*4
    assert clock.mode == 2


@pytest.mark.parametrize('fault', ['cancel', 'zero', 'nav', 'mode', 'origin', 'stale'])
def test_interrupted_dwell_never_completes_or_leaves_old_output(node, fault):
    goal = Goal(12, timeout=5.)
    clock = MCUClock(node)
    def interrupt():
        if fault == 'cancel':
            goal.is_cancel_requested = True
        elif fault == 'zero':
            node._execute_motion(Goal(7))
        elif fault == 'nav':
            clock.nav = node.status.navigation_ready = False
        elif fault == 'mode':
            node.status.control_level = 2
        elif fault == 'origin':
            node._origin_generation = 5
        else:
            node._status_received_at = clock.now-2.
    clock.fault = interrupt
    _, result = run(node, clock, goal)
    assert not result.success
    assert goal.terminal == ('canceled' if fault == 'cancel' else 'aborted')
    assert clock.now < 102.
    assert clock.thrust == [0.]*4 and not node._velocity_active
    assert clock.mode == (3 if fault == 'nav' else 2)
    assert not node._handoff_reserved and not node._motion_reserved


def test_handoff_rejects_competing_velocity_in_acceptance_and_execution(node):
    clock = MCUClock(node)
    goal = Goal(12, timeout=5.)
    goal.request.velocity_lease = 2.
    assert node._action_goal_cb(goal.request) == 'accept'
    assert node._action_goal_cb(Goal(7, [.08, 0., 0., 0.]).request) == 'reject'
    with pytest.raises(RuntimeError, match='owns motion output'):
        node._execute_motion(Goal(7, [.08, 0., 0., 0.]))
    assert node._action_goal_cb(Goal(7).request) == 'accept'


def test_controller_stall_past_total_deadline_cannot_report_success(node):
    clock = MCUClock(node)
    def stall():
        clock.now += 10.
        node._status_received_at = node._odom_received_at = node._nav_sample_at = clock.now
    clock.fault = stall
    _, result = run(node, clock)
    assert not result.success and result.message == 'STOP_AND_HOLD total timeout'
    assert clock.thrust == clock.velocity == [0.]*4 and clock.mode == 2


@pytest.mark.parametrize('field,value', [
    ('velocity_lease', 0.), ('velocity_lease', float('nan')),
    ('timeout', 2.), ('timeout', float('inf')), ('target', [1., 0., 0., 0.]),
    ('axes', 'xy'),
])
def test_invalid_handoff_rejected_before_reserving_or_sending(node, field, value):
    clock = MCUClock(node)
    goal = Goal(12, timeout=5.)
    goal.request.velocity_lease = 2.
    setattr(goal.request, field, value)
    assert node._action_goal_cb(goal.request) == 'reject'
    assert not node._motion_reserved and not clock.commands
