"""Exercise real SIL node callbacks against the native firmware core."""

import time

import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('zit6_interfaces.msg')
pytest.importorskip('uv_msgs.msg')
pytest.importorskip('zit6_control_core')

from rclpy.duration import Duration
from std_msgs.msg import UInt32
from zit6_interfaces.srv import SetOrigin
from uv_sim_bridge.sim_bridge import SimBridgeNode


@pytest.fixture
def bridge():
    rclpy.init(domain_id=185)
    node = SimBridgeNode()
    node._control_stop.set()
    node._control_thread.join(timeout=2)
    node._ins_boot_time = node.get_clock().now() - Duration(seconds=2)
    now = time.monotonic()
    node._navigation.update_velocity((0, 0, 0), now)
    node._navigation.update_imu((0, 0, 0), now)
    node._control_tick()
    yield node
    node.destroy_node()
    rclpy.try_shutdown()


def heartbeat(node, value):
    message = UInt32()
    message.data = value
    node._agxhbt_cb(message)


def test_sil_origin_service_and_arm_preserve_the_same_mcu_frame(bridge):
    assert not bridge._lifecycle.armed
    heartbeat(bridge, 3)
    assert not bridge._lifecycle.armed
    response = bridge._set_origin_cb(SetOrigin.Request(), SetOrigin.Response())
    assert response.success
    assert response.origin_generation == 1
    # Exercise the firmware's ten-heartbeat + one-second qualification.
    for _ in range(10):
        heartbeat(bridge, 1)
    assert not bridge._lifecycle.armed
    bridge._lifecycle.arm_start_s = time.monotonic() - 1.01
    bridge._control_tick()
    assert bridge._lifecycle.armed
    denied = bridge._set_origin_cb(SetOrigin.Request(), SetOrigin.Response())
    assert not denied.success
    assert bridge._core.get_odom_snapshot()['origin_generation'] == 1
    bridge._lifecycle.last_heartbeat_s = time.monotonic() - 1.1
    bridge._control_tick()
    assert not bridge._lifecycle.armed
    assert bridge._core.get_odom_snapshot()['origin_initialized']
    repeated = bridge._set_origin_cb(SetOrigin.Request(), SetOrigin.Response())
    assert repeated.success and repeated.origin_generation == 2


def test_sil_setorigin_rejects_stale_navigation(bridge):
    bridge._core.update_nav((1, 2, 3, 0, 0, 0), (0,) * 6,
                            timestamp_ms=(bridge._mcu_tick_ms() - 201) & 0xFFFFFFFF)
    response = bridge._set_origin_cb(SetOrigin.Request(), SetOrigin.Response())
    assert not response.success
    assert not bridge._core.get_odom_snapshot()['origin_initialized']


def test_sil_ros_service_and_odom_roundtrip(bridge):
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from zit6_interfaces.msg import ZitOdom
    from auv_protocol.topics import ZIT6_ODOM, ZIT6_SET_ORIGIN

    probe = Node('origin_service_probe')
    client = probe.create_client(SetOrigin, ZIT6_SET_ORIGIN)
    received = []
    probe.create_subscription(ZitOdom, ZIT6_ODOM, received.append, 10)
    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    executor.add_node(probe)
    try:
        assert client.wait_for_service(timeout_sec=2)
        bridge._control_tick()
        future = client.call_async(SetOrigin.Request())
        executor.spin_until_future_complete(future, timeout_sec=2)
        response = future.result()
        assert response.success and response.origin_generation == 1
        bridge._publish_odom()
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline and not any(
                msg.origin_generation == response.origin_generation for msg in received):
            executor.spin_once(timeout_sec=0.02)
        matching = [msg for msg in received
                    if msg.origin_generation == response.origin_generation]
        assert matching
        assert matching[-1].pose_odom == pytest.approx((0,) * 6)
        assert matching[-1].origin_initialized
    finally:
        executor.remove_node(bridge)
        executor.remove_node(probe)
        executor.shutdown()
        probe.destroy_node()


def test_hil_ros_routes_continuous_rawnav_and_reconfigures_after_mcu_reboot():
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from sensor_msgs.msg import Imu
    from std_msgs.msg import Float32MultiArray
    from zit6_interfaces.msg import ZitStatus
    from zit6_interfaces.srv import UpdateParams
    from uv_msgs.msg import DvlVelocity
    from uv_hm.hw_manager import HwManagerNode
    from auv_protocol.topics import (
        DVL_VELOCITY, IMU, LEGACY_ZIT6_HEARTBEAT_STATE,
        LEGACY_ZIT6_SIM_NAV, LEGACY_ZIT6_STATUS, LEGACY_ZIT6_UPDATE_PARAMS,
    )

    rclpy.init(args=['--ros-args', '-p', 'hil_mode:=true'], domain_id=186)
    bridge = SimBridgeNode()
    manager = HwManagerNode()
    mcu = Node('hil_mcu_probe')
    config_requests = []
    nav_messages = []

    def update_config(request, response):
        config_requests.append(dict(zip(request.paths, request.values)))
        response.success = True
        response.message = 'ok'
        return response

    mcu.create_service(UpdateParams, LEGACY_ZIT6_UPDATE_PARAMS, update_config)
    mcu.create_subscription(Float32MultiArray, LEGACY_ZIT6_SIM_NAV,
                            lambda message: nav_messages.append(list(message.data)), 10)
    status_pub = mcu.create_publisher(ZitStatus, LEGACY_ZIT6_STATUS, 10)
    heartbeat_pub = mcu.create_publisher(UInt32, LEGACY_ZIT6_HEARTBEAT_STATE, 10)
    dvl_pub = mcu.create_publisher(DvlVelocity, DVL_VELOCITY, 10)
    imu_pub = mcu.create_publisher(Imu, IMU, 10)

    def publish_inputs():
        status = ZitStatus()
        status.is_armed = False
        status_pub.publish(status)
        dvl = DvlVelocity()
        dvl.valid = True
        dvl.velocity.x = 1.0
        dvl_pub.publish(dvl)
        imu_pub.publish(Imu())

    mcu.create_timer(0.02, publish_inputs)
    executor = SingleThreadedExecutor()
    for node in (bridge, manager, mcu):
        executor.add_node(node)
    try:
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline and len(nav_messages) < 6:
            executor.spin_once(timeout_sec=0.02)
        assert config_requests
        assert config_requests[0] == {
            'simulation.hitl_enabled': 'false', 'simulation.sitl_enabled': 'true'}
        assert len(nav_messages) >= 6
        assert len(nav_messages[-1]) == 12
        assert nav_messages[-1][0] > nav_messages[0][0] + 0.05
        assert nav_messages[-1][6] == pytest.approx(1.0)
        heartbeat = UInt32()
        heartbeat.data = 5000
        heartbeat_pub.publish(heartbeat)
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline and bridge._hil_last_heartbeat_tick != 5000:
            executor.spin_once(timeout_sec=0.01)
        assert bridge._hil_last_heartbeat_tick == 5000
        heartbeat.data = 1000
        heartbeat_pub.publish(heartbeat)
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline and bridge._hil_last_heartbeat_tick != 1000:
            executor.spin_once(timeout_sec=0.01)
        assert bridge._hil_last_heartbeat_tick == 1000
        assert not bridge._hil_navigation_configured
        before_restart_frames = len(nav_messages)
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline and (
                len(config_requests) < 2 or len(nav_messages) < before_restart_frames + 3):
            executor.spin_once(timeout_sec=0.02)
        assert len(config_requests) >= 2
        assert bridge._hil_navigation_configured
        assert len(nav_messages) >= before_restart_frames + 3
        assert nav_messages[-1][0] > nav_messages[0][0] + 0.1
    finally:
        for node in (bridge, manager, mcu):
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        rclpy.try_shutdown()


def test_sil_rejects_reserved_setpoint_mode(bridge):
    from zit6_interfaces.msg import ZitSetpoint
    response = bridge._set_origin_cb(SetOrigin.Request(), SetOrigin.Response())
    assert response.success
    for _ in range(10):
        heartbeat(bridge, 1)
    bridge._lifecycle.arm_start_s = time.monotonic() - 1.01
    bridge._control_tick()
    assert bridge._lifecycle.armed
    command = ZitSetpoint()
    command.control_key = 3
    bridge._setpoint_cb(command)
    assert bridge._core.control_level == 0


def test_sil_actuator_mode_matches_force_arm_without_navigation(bridge):
    from zit6_interfaces.msg import ZitSetpoint
    assert bridge._set_origin_cb(SetOrigin.Request(), SetOrigin.Response()).success
    for _ in range(10):
        heartbeat(bridge, 3)
    bridge._lifecycle.arm_start_s = time.monotonic() - 1.01
    bridge._control_tick()
    assert bridge._lifecycle.armed
    command = ZitSetpoint()
    command.control_key = 0x12  # body actuator force
    command.x = 0.4
    bridge._setpoint_cb(command)
    old = time.monotonic() - 2.1
    bridge._navigation.update_velocity((0, 0, 0), old)
    bridge._navigation.update_imu((0, 0, 0), old)
    bridge._control_tick()
    assert not bridge._core.get_odom_snapshot()['nav_valid']
    assert bridge.force_6dof[0] == pytest.approx(0.4)


def test_setorigin_discards_pre_reset_heartbeat_arming_qualification(bridge):
    assert bridge._set_origin_cb(SetOrigin.Request(), SetOrigin.Response()).success
    for _ in range(10):
        heartbeat(bridge, 1)
    assert not bridge._lifecycle.armed
    bridge._lifecycle.arm_start_s = time.monotonic() - 1.01
    # A pending control tick would otherwise ARM from old heartbeats here.
    response = bridge._set_origin_cb(SetOrigin.Request(), SetOrigin.Response())
    assert response.success and response.origin_generation == 2
    bridge._control_tick()
    assert not bridge._lifecycle.armed
    assert bridge._lifecycle.heartbeat_count == 0
    for _ in range(10):
        heartbeat(bridge, 1)
    assert not bridge._lifecycle.armed
    bridge._lifecycle.arm_start_s = time.monotonic() - 1.01
    bridge._control_tick()
    assert bridge._lifecycle.armed
