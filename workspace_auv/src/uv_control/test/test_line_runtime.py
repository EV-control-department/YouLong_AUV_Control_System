"""Exercise production BLINE callbacks with a virtual clock and vehicle."""
from types import SimpleNamespace as NS

import pytest

from test_start_origin import Goal, node  # noqa: F401 (shared production fixture)
from uv_control.coordinate import Coordinate
from uv_control.line_guidance import rotate_body_to_world, rotate_world_to_body


class VehicleClock:
    def __init__(self, instance, goal, fault=None):
        self.now = 0.0
        self.instance = instance
        self.goal = goal
        self.fault = fault
        self.ticks = 0
        self.hold_at = None

    def monotonic(self):
        return self.now

    def sleep(self, dt):
        self.now += dt
        self.ticks += 1
        n = self.instance
        if n.pub_setpoint.messages:
            command = n.pub_setpoint.messages[-1]
            pose = n.pose
            if command.control_key == 0x11:
                world = rotate_body_to_world((command.x, command.y, command.z),
                                             pose.rx, pose.ry, pose.rz)
                yaw = pose.rz + __import__('math').degrees(command.yaw) * dt
            else:
                world = tuple(max(-.3, min(.3, .8 * (b - p)))
                              for b, p in zip((command.x, command.y, command.z),
                                              (pose.x, pose.y, pose.z)))
                yaw = __import__('math').degrees(command.yaw)
                if n._line_feedback and n._line_feedback[0] == 'HOLD':
                    self.hold_at = self.now if self.hold_at is None else self.hold_at
            n.pose = Coordinate(x=pose.x + world[0] * dt,
                                y=pose.y + world[1] * dt,
                                z=pose.z + world[2] * dt, rz=yaw)
            body = rotate_world_to_body(world, 0, 0, yaw)
            n.vel_body = dict(zip(('x', 'y', 'z'), body))
        n._status_received_at = n._odom_received_at = n._nav_sample_at = self.now
        n._twist_received_at = self.now
        if self.ticks >= 20:
            if self.fault == 'cancel':
                self.goal.is_cancel_requested = True
            elif self.fault == 'zero':
                n._execute_motion(Goal(7))
            elif self.fault == 'twist':
                n._twist_received_at = self.now - 2
            elif self.fault == 'odom':
                n._odom_received_at = self.now - 2
            elif self.fault == 'origin':
                n._origin_generation = 5
            elif self.fault == 'lease':
                n._velocity_active = True
                n._velocity_deadline = self.now - 1
                n._velocity_watchdog_cb()


def prepare(instance, goal, fault=None):
    clock = VehicleClock(instance, goal, fault)
    instance._execute_body_line.__func__.__globals__['time'] = clock
    instance._started = instance._heartbeat_enabled = True
    instance._origin_initialized = True
    instance._active_origin_generation = instance._origin_generation = 4
    instance.status.is_armed = True
    instance._status_received_at = instance._odom_received_at = 0
    instance._nav_sample_at = instance._twist_received_at = 0
    assert instance._action_goal_cb(goal.request) == 'accept'
    return clock


def test_success_keeps_position_mode_and_waits_in_hold(node):
    goal = Goal(8, [0, 1, .2, 0])
    clock = prepare(node, goal)
    result = node._execute_body_line(goal)
    assert result.success and goal.terminal == 'succeeded'
    assert result.final_target == pytest.approx([0, 1, .2, 0])
    assert clock.hold_at is not None and clock.now - clock.hold_at >= node._line_config.stable_seconds - .051
    assert node.pub_setpoint.messages[-1].control_key == 0
    assert not node._velocity_active


@pytest.mark.parametrize('fault', ['cancel', 'zero', 'twist', 'odom', 'origin', 'lease'])
def test_failure_ends_lease_and_holds_only_with_valid_state(node, fault):
    goal = Goal(8, [2, 0, 0, 0])
    prepare(node, goal, fault)
    result = node._execute_body_line(goal)
    assert not result.success and not node._velocity_active
    assert goal.terminal == ('canceled' if fault == 'cancel' else 'aborted')
    last = node.pub_setpoint.messages[-1]
    if fault in ('odom', 'origin'):
        assert last.control_key == 0x11 and (last.x, last.y, last.z, last.yaw) == (0, 0, 0, 0)
    else:
        assert last.control_key == 0
    velocities = [m for m in node.pub_setpoint.messages if m.control_key == 0x11]
    assert (velocities[-1].x, velocities[-1].y, velocities[-1].z, velocities[-1].yaw) == (0, 0, 0, 0)


def test_total_timeout_includes_tracking(node):
    goal = Goal(8, [2, 0, 0, 0], timeout=.5)
    clock = prepare(node, goal)
    result = node._execute_body_line(goal)
    assert not result.success and 'timeout' in result.message
    assert clock.now <= .55 and not node._velocity_active


def test_reserved_line_rejects_nonzero_and_accepts_zero_velocity(node):
    goal = Goal(8, [1, 0, 0, 0])
    prepare(node, goal)
    assert node._action_goal_cb(Goal(7, [.1, 0, 0, 0]).request) == 'reject'
    assert node._action_goal_cb(Goal(7).request) == 'accept'
    # Also reject a competing goal accepted before the line reservation.
    with pytest.raises(RuntimeError, match='owns motion'):
        node._execute_motion(Goal(7, [.1, 0, 0, 0]))
    assert node._execute_motion(Goal(7)).success
    assert node._line_stop_requested.is_set()


@pytest.mark.parametrize('speed', [.28, .32, 1.0])
def test_overspeed_action_is_accepted_and_completes(node, speed):
    goal = Goal(8, [0, 1, .2, 0])
    goal.request.cruise_speed = speed
    prepare(node, goal)
    result = node._execute_body_line(goal)
    assert result.success and goal.terminal == 'succeeded'


@pytest.mark.parametrize('length,speed,requested,expected', [
    (.5, .15, 0., 15.),
    (3., .15, 0., 30.),
    (.5, .15, 7., 7.),
])
def test_timeout_uses_new_auto_formula_or_explicit_override(node, length, speed, requested, expected):
    goal = Goal(8, [length, 0, 0, 0], timeout=requested)
    goal.request.cruise_speed = speed
    clock = prepare(node, goal)
    advance = clock.sleep
    def frozen_vehicle(dt):
        pose = node.pose
        advance(dt)
        node.pose = pose
        node.vel_body = {'x': 0., 'y': 0., 'z': 0.}
    clock.sleep = frozen_vehicle
    result = node._execute_body_line(goal)
    assert not result.success and goal.terminal == 'aborted'
    assert result.message == 'LINE total timeout'
    assert clock.now == pytest.approx(expected, abs=.1)
    assert not node._velocity_active


@pytest.mark.parametrize('command,target,expected', [
    (8, [1,0,0,90], [1,0,0,90]),
    (8, [0,1,.2,-90], [0,1,.2,-90]),
    (9, [1,0,.2,135], [1,0,.2,135]),
    (9, [0,0,0,90], [0,0,0,90]),
    (8, [0,0,0,90], [0,0,0,90]),
])
def test_both_line_commands_turn_only_after_arriving(node, command, target, expected):
    goal = Goal(command, target, timeout=60.)
    clock = prepare(node, goal)
    sent = []
    publish = node._send_setpoint
    def record(*args, **kwargs):
        sent.append((clock.now, node.pose, args))
        return publish(*args, **kwargs)
    node._send_setpoint = record
    result = node._execute_motion(goal)
    assert result.success and goal.terminal == 'succeeded'
    assert result.final_target == pytest.approx(expected)
    assert abs((node.pose.rz-expected[3]+180)%360-180) <= 5
    # First command holds the start facing travel direction; the later
    # position command contains final heading and is sent only at the endpoint.
    final_commands = [(pose,args) for _,pose,args in sent
                      if args[0] == 0 and abs(args[2]-expected[0]) < 1e-6
                      and abs(args[3]-expected[1]) < 1e-6
                      and abs(args[4]-expected[2]) < 1e-6
                      and abs(__import__('math').degrees(args[5])-expected[3]) < 1e-6]
    assert final_commands
    for pose, _ in final_commands:
        assert max(abs(p-t) for p,t in zip((pose.x,pose.y,pose.z),expected[:3])) <= .1
    assert clock.hold_at is not None
    assert not node._velocity_active


def test_body_line_final_yaw_is_relative_and_world_line_is_absolute(node):
    node.pose = Coordinate(x=2., y=-3., z=.5, rz=90.)
    goal = Goal(8, [1,0,0,45], timeout=60.)
    prepare(node, goal)
    result = node._execute_motion(goal)
    assert result.success
    assert result.final_target == pytest.approx([2.,-2.,.5,135.])


def test_world_line_can_return_to_world_origin_from_nonzero_start(node):
    node.pose = Coordinate(x=.5, y=.3, z=.2, rz=35.)
    goal = Goal(9, [0,0,0,-45], timeout=60.)
    prepare(node, goal)
    result = node._execute_motion(goal)
    assert result.success
    assert result.final_target == pytest.approx([0,0,0,-45])


def test_final_turn_cannot_succeed_before_heading_is_reached(node):
    goal = Goal(9, [.5,0,0,90], timeout=18.)
    clock = prepare(node, goal)
    advance = clock.sleep
    def jam_final_turn(dt):
        advance(dt)
        if node.pub_setpoint.messages and clock.now > 1:
            msg = node.pub_setpoint.messages[-1]
            if msg.control_key == 0 and abs(msg.x-.5) < 1e-6:
                node.pose = Coordinate(x=node.pose.x, y=node.pose.y, z=node.pose.z, rz=0.)
    clock.sleep = jam_final_turn
    result = node._execute_motion(goal)
    assert not result.success and result.message == 'LINE total timeout'
    assert not node._velocity_active


@pytest.mark.parametrize('fault', ['cancel', 'zero', 'twist', 'odom', 'origin', 'lease'])
def test_world_line_retains_fault_stop_guards(node, fault):
    goal = Goal(9, [1.,.5,.2,90.], timeout=60.)
    prepare(node, goal, fault)
    result = node._execute_motion(goal)
    assert not result.success
    assert not node._velocity_active
