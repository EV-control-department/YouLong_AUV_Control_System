"""Exercise the real ROS SIL origin/control chain with the native firmware core.

Run after building zit6_interfaces, uv_msgs and zit6_control_core, with the
uv_control, uv_localization, uv_sim_bridge and auv_protocol packages on PYTHONPATH.
Sensor inputs are synthetic DVL/gyro streams; no ground-truth pose is supplied.
"""

import math
import threading
import time
from collections import deque
from types import SimpleNamespace as NS

import pytest

rclpy = pytest.importorskip('rclpy')
core_module = pytest.importorskip('zit6_control_core')
msg_module = pytest.importorskip('uv_msgs.msg')
zit_msg_module = pytest.importorskip('zit6_interfaces.msg')
pytest.importorskip('uv_sim_bridge')
pytest.importorskip('uv_localization')
if (not hasattr(msg_module, 'StateResetRequest')
        or not hasattr(zit_msg_module, 'ZitOdom')
        or not hasattr(core_module.Zit6Controller, 'try_set_origin')):
    pytest.skip('rebuilt versioned origin interfaces/native core required', allow_module_level=True)

from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import UInt32
from uv_msgs.action import BasicMotion
from uv_msgs.msg import DvlVelocity, PoseInfo
from zit6_interfaces.msg import ZitOdom, ZitSetpoint
from zit6_interfaces.srv import SetOrigin
from auv_protocol.topics import (
    BASIC_MOTION, DVL_VELOCITY, IMU, STATE_ODOM,
    ZIT6_ARM_HEARTBEAT, ZIT6_ODOM, ZIT6_SETPOINT, ZIT6_SET_ORIGIN,
)
from uv_control.basic_motion import BasicMotionNode
from uv_localization.estimator import EstimatorNode
from uv_sim_bridge.sim_bridge import SimBridgeNode


def wait_for(predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.005)
    assert predicate(), 'SIL chain did not reach the expected state before timeout'


def wait_future(future, timeout=5.0):
    wait_for(future.done, timeout)
    return future.result()


@pytest.fixture
def chain(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros-log'))
    # Isolate the test from actual vehicle command endpoints on the default domain.
    rclpy.init(args=['--ros-args', '-p', 'navigation_timeout:=0.25'], domain_id=183)
    nodes = []
    executor = MultiThreadedExecutor(num_threads=4)
    thread = None
    try:
        bridge = SimBridgeNode()
        nodes.append(bridge)
        localization = EstimatorNode()
        nodes.append(localization)
        motion = BasicMotionNode()
        nodes.append(motion)
        probe = Node('sil_origin_integration_probe')
        nodes.append(probe)
        state = NS(bridge=bridge, localization=localization, motion=motion,
                   probe=probe, sensors_active=True,
                   velocity=(0.4, 0.2, 0.15), yaw_rate=0.3,
                   odom=deque(maxlen=300), pose=deque(maxlen=300),
                   setpoints=deque(maxlen=300), heartbeats=deque(maxlen=300),
                   commits=[], native_setpoints=[])
        velocity_pub = probe.create_publisher(DvlVelocity, DVL_VELOCITY, 10)
        imu_pub = probe.create_publisher(Imu, IMU, 10)

        def sensors():
            if not state.sensors_active:
                return
            dvl = DvlVelocity()
            dvl.header.stamp = probe.get_clock().now().to_msg()
            dvl.valid = True
            dvl.velocity.x, dvl.velocity.y, dvl.velocity.z = state.velocity
            imu = Imu()
            imu.header.stamp = dvl.header.stamp
            imu.angular_velocity.z = state.yaw_rate
            velocity_pub.publish(dvl)
            imu_pub.publish(imu)

        probe.create_timer(0.02, sensors)
        probe.create_subscription(ZitOdom, ZIT6_ODOM, lambda msg: state.odom.append(msg), 10)
        probe.create_subscription(PoseInfo, STATE_ODOM, lambda msg: state.pose.append(msg), 10)
        probe.create_subscription(ZitSetpoint, ZIT6_SETPOINT, lambda msg: state.setpoints.append(msg), 10)
        probe.create_subscription(
            UInt32, ZIT6_ARM_HEARTBEAT,
            lambda msg: state.heartbeats.append((time.monotonic(), msg.data)), 10)
        state.action = ActionClient(probe, BasicMotion, BASIC_MOTION)

        # Spies observe calls while delegating every operation to the real C++ core.
        native_update = bridge._core.update_setpoint
        native_origin = bridge._core.try_set_origin

        def observe_setpoint(mode, values, mask, body, incremental):
            state.native_setpoints.append((mode, tuple(values), mask, body, incremental))
            return native_update(mode, values, mask, body, incremental)

        def observe_origin(*args):
            result = native_origin(*args)
            state.commits.append(dict(result))
            return result

        bridge._core.update_setpoint = observe_setpoint
        bridge._core.try_set_origin = observe_origin
        for node in nodes:
            executor.add_node(node)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        wait_for(lambda: state.action.server_is_ready() and motion._nav_valid
                 and motion._status_received_at > 0, 5.0)
        state.velocity = (0.0, 0.0, 0.0)
        state.yaw_rate = 0.0
        # Let the last moving sensor samples leave the transport/control queues.
        time.sleep(0.08)
        yield state
    finally:
        for node in nodes:
            shutdown = getattr(node, '_on_context_shutdown', None)
            if shutdown is not None:
                shutdown()
        executor.shutdown(timeout_sec=3.0)
        if thread is not None:
            thread.join(timeout=3.0)
        for node in reversed(nodes):
            node.destroy_node()
        rclpy.try_shutdown()


def send_goal(chain, command, target=None, timeout=5.0):
    goal = BasicMotion.Goal()
    goal.cmd_type = command
    goal.target = list(target or [0.0] * 4)
    goal.timeout = timeout
    handle = wait_future(chain.action.send_goal_async(goal))
    assert handle.accepted
    return handle


def start(chain):
    handle = send_goal(chain, BasicMotion.Goal.START)
    wrapped = wait_future(handle.get_result_async(), 6.0)
    assert wrapped.status == GoalStatus.STATUS_SUCCEEDED
    assert wrapped.result.success, wrapped.result.message
    return wrapped.result


def assert_one_odom_frame(chain, generation):
    wait_for(lambda: chain.motion._origin_generation == generation
             and chain.pose and chain.pose[-1].origin_generation == generation
             and chain.odom and chain.odom[-1].origin_generation == generation)
    # Stationary sensors make all samples identical despite different publish rates.
    native = chain.bridge._core.get_odom_snapshot()
    wire = chain.odom[-1]
    pose = chain.pose[-1]
    motion = chain.motion.pose
    expected = native['pose_odom']
    assert wire.pose_odom == pytest.approx(expected, abs=1e-4)
    assert [pose.robot_x, pose.robot_y, pose.robot_z] == pytest.approx(expected[:3], abs=1e-4)
    assert [pose.robot_roll, pose.robot_pitch, pose.robot_yaw] == pytest.approx(
        [math.degrees(value) for value in expected[3:]], abs=1e-4)
    assert [motion.x, motion.y, motion.z, motion.rz] == pytest.approx(
        [expected[0], expected[1], expected[2], math.degrees(expected[5])], abs=1e-4)
    assert [pose.origin_x, pose.origin_y, pose.origin_z, pose.origin_yaw] == [0.0] * 4
    assert expected == pytest.approx((0.0,) * 6, abs=1e-3)
    assert pose.origin_initialized and pose.nav_valid and chain.motion._motion_ready()


def test_real_sil_start_repeat_motion_cancel_and_failure_chain(chain):
    native = chain.bridge._core.get_odom_snapshot()
    raw_before = chain.bridge._navigation.snapshot(time.monotonic()).position
    assert all(abs(raw_before[index]) > 0.05 for index in (0, 1, 2, 5))
    assert not native['origin_initialized'] and not chain.bridge._lifecycle.armed
    assert not chain.heartbeats

    start(chain)
    first = chain.commits[-1]
    assert first['success'] and first['origin_generation'] == 1
    assert first['origin_nav'] == pytest.approx(raw_before, abs=0.03)
    assert_one_odom_frame(chain, 1)
    assert len(chain.heartbeats) >= 10
    # Bridge qualification itself is exact at 1.0s; topic callback timestamps
    # can compress slightly under scheduler load, so leave 100ms observation
    # tolerance here and keep the lifecycle boundary covered by unit tests.
    assert chain.heartbeats[-1][0] - chain.heartbeats[0][0] >= 0.90

    # Observe the position target delivered to both ROS and the real native core.
    target = [1.25, -0.4, 0.3, 45.0]
    handle = send_goal(chain, BasicMotion.Goal.SET, target)
    wait_for(lambda: any(call[0] == 0 for call in chain.native_setpoints))
    position_calls = [call for call in chain.native_setpoints if call[0] == 0]
    assert position_calls[-1][1] == pytest.approx(
        [target[0], target[1], target[2], 0.0, 0.0, math.radians(target[3])])
    assert position_calls[-1][2:] == (0, False, False)
    position_messages = [message for message in chain.setpoints if message.control_key == 0]
    assert [position_messages[-1].x, position_messages[-1].y,
            position_messages[-1].z, position_messages[-1].yaw] == pytest.approx(
        [target[0], target[1], target[2], math.radians(target[3])])
    wait_future(handle.cancel_goal_async())
    stopped = wait_future(handle.get_result_async())
    assert stopped.status == GoalStatus.STATUS_CANCELED and not stopped.result.success
    assert chain.motion._heartbeat_enabled

    # Move the continuous raw nav again, then reset at that new nonzero pose.
    chain.velocity = (0.25, -0.12, 0.08)
    chain.yaw_rate = -0.2
    time.sleep(0.35)
    chain.velocity = (0.0, 0.0, 0.0)
    chain.yaw_rate = 0.0
    time.sleep(0.08)
    raw_second = chain.bridge._navigation.snapshot(time.monotonic()).position
    assert abs(raw_second[0] - raw_before[0]) > 0.04
    start(chain)
    second = chain.commits[-1]
    assert second['origin_generation'] == 2
    assert second['origin_nav'] == pytest.approx(raw_second, abs=0.02)
    assert chain.bridge._navigation.snapshot(time.monotonic()).position == pytest.approx(raw_second, abs=0.02)
    assert_one_odom_frame(chain, 2)

    # Cancel START while it is stopping the previous ARM heartbeat.
    before_cancel_commits = len(chain.commits)
    handle = send_goal(chain, BasicMotion.Goal.START)
    wait_for(lambda: not chain.motion._heartbeat_enabled)
    wait_future(handle.cancel_goal_async())
    canceled = wait_future(handle.get_result_async())
    assert canceled.status == GoalStatus.STATUS_CANCELED and not canceled.result.success
    assert not chain.motion._heartbeat_enabled
    time.sleep(0.08)
    heartbeat_count = len(chain.heartbeats)
    wait_for(lambda: not chain.bridge._lifecycle.armed, 2.0)
    assert len(chain.heartbeats) == heartbeat_count
    assert len(chain.commits) == before_cancel_commits
    assert chain.bridge._core.get_odom_snapshot()['origin_generation'] == 2
    assert chain.bridge._core.get_odom_snapshot()['origin_initialized']

    # A sensor outage cannot initialize/arm the vehicle from republished old state.
    chain.sensors_active = False
    wait_for(lambda: not chain.motion._nav_valid, 2.0)
    handle = send_goal(chain, BasicMotion.Goal.START, timeout=0.15)
    failed = wait_future(handle.get_result_async())
    assert not failed.result.success and not chain.motion._heartbeat_enabled
    assert len(chain.heartbeats) == heartbeat_count
    assert len(chain.commits) == before_cancel_commits

    # Resume fresh nav but remove the backend service: the async failure returns
    # through localization and BasicMotion instead of enabling ARM or deadlocking.
    chain.sensors_active = True
    wait_for(lambda: chain.motion._nav_valid)
    service = next(service for service in chain.bridge.services
                   if service.srv_name == ZIT6_SET_ORIGIN)
    chain.bridge.destroy_service(service)
    wait_for(lambda: not chain.localization._origin_client.service_is_ready())
    handle = send_goal(chain, BasicMotion.Goal.START)
    unavailable = wait_future(handle.get_result_async())
    assert not unavailable.result.success
    assert 'unavailable' in unavailable.result.message
    assert not chain.motion._heartbeat_enabled and not chain.motion._motion_ready()
    assert len(chain.heartbeats) == heartbeat_count
