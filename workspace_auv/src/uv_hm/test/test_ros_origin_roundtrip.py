"""A real ROS executor roundtrip verifies the callbacks remain responsive."""

import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.task import Future
from std_msgs.msg import UInt32
from uv_msgs.msg import PoseInfo, StateResetRequest, StateResetResult
from zit6_interfaces.msg import ZitOdom
from zit6_interfaces.srv import GetParams, SetOrigin, UpdateParams
from auv_protocol.topics import (
    LEGACY_ZIT6_GET_PARAMS, LEGACY_ZIT6_HEARTBEAT, LEGACY_ZIT6_ODOM,
    LEGACY_ZIT6_SET_ORIGIN, LEGACY_ZIT6_UPDATE_PARAMS, STATE_ODOM,
    STATE_RESET, STATE_RESET_RESULT, ZIT6_ARM_HEARTBEAT, ZIT6_GET_PARAMS,
    ZIT6_UPDATE_PARAMS,
)
from uv_hm.hw_manager import HwManagerNode
from uv_localization.estimator import EstimatorNode


def test_reset_proxy_waits_for_reply_and_odom_while_heartbeat_stays_responsive():
    # Keep fake MCU services/commands away from the vehicle DDS domain.
    rclpy.init(domain_id=184)
    executor = SingleThreadedExecutor()
    hardware = HwManagerNode()
    estimator = EstimatorNode()
    probe = rclpy.create_node('origin_roundtrip_probe')
    nodes = (hardware, estimator, probe)
    for node in nodes:
        executor.add_node(node)
    held = {}
    frames = []
    resets = []
    heartbeats = []
    nav = {'generation': 0, 'timestamp': 1000}

    def spin_until(predicate, timeout=3.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.01)
            if predicate():
                return
        raise AssertionError('ROS roundtrip timeout')

    async def set_origin(_request, response):
        nav['generation'] += 1
        response.success = True
        response.message = 'origin set'
        response.origin_nav = [30.0, 40.0, 2.0, 0.0, 0.0, 0.5]
        response.origin_generation = nav['generation']
        response.nav_timestamp_ms = nav['timestamp']
        held['response'] = response
        held['future'] = Future(executor=executor)
        return await held['future']

    def get_params(_request, response):
        response.success = True
        response.config_json = '{"simulation":{"sitl_enabled":true}}'
        return response

    def update_params(request, response):
        response.success = True
        response.message = request.json
        return response

    group = ReentrantCallbackGroup()
    probe.create_service(SetOrigin, LEGACY_ZIT6_SET_ORIGIN, set_origin, callback_group=group)
    probe.create_service(GetParams, LEGACY_ZIT6_GET_PARAMS, get_params, callback_group=group)
    probe.create_service(UpdateParams, LEGACY_ZIT6_UPDATE_PARAMS, update_params, callback_group=group)
    odom_pub = probe.create_publisher(ZitOdom, LEGACY_ZIT6_ODOM, 10)

    def publish_nav():
        nav['timestamp'] += 10
        message = ZitOdom(
            pose_odom=[2.0, -1.0, 0.5, 0.0, 0.0, 0.3], twist_body=[0.0] * 6,
            nav_timestamp_ms=nav['timestamp'], nav_valid=True,
            origin_initialized=nav['generation'] != 0,
            origin_generation=nav['generation'])
        odom_pub.publish(message)

    probe.create_timer(0.02, publish_nav)
    probe.create_subscription(PoseInfo, STATE_ODOM, frames.append, 10)
    probe.create_subscription(StateResetResult, STATE_RESET_RESULT, resets.append, 10)
    probe.create_subscription(UInt32, LEGACY_ZIT6_HEARTBEAT, heartbeats.append, 10)
    reset_pub = probe.create_publisher(StateResetRequest, STATE_RESET, 10)
    heartbeat_pub = probe.create_publisher(UInt32, ZIT6_ARM_HEARTBEAT, 10)
    get_client = probe.create_client(GetParams, ZIT6_GET_PARAMS)
    update_client = probe.create_client(UpdateParams, ZIT6_UPDATE_PARAMS)
    try:
        spin_until(lambda: hardware._origin_client.service_is_ready()
                   and estimator._origin_client.service_is_ready()
                   and get_client.service_is_ready() and update_client.service_is_ready()
                   and frames and frames[-1].nav_valid)
        assert not heartbeats  # Adapter never manufactures an arm heartbeat.
        reset_pub.publish(StateResetRequest(request_id=77))
        spin_until(lambda: 'future' in held and frames[-1].origin_generation == 1)
        assert not resets  # New-generation odom alone cannot complete reset.
        heartbeat_pub.publish(UInt32(data=0))
        spin_until(lambda: bool(heartbeats))
        assert heartbeats[-1].data == 0
        assert not resets  # Client wait did not block forwarding on one executor.
        held['future'].set_result(held['response'])
        spin_until(lambda: bool(resets))
        assert resets[-1].request_id == 77 and resets[-1].success
        assert resets[-1].origin_generation == 1
        assert frames[-1].robot_x == 2.0 and frames[-1].origin_x == 0.0
        get_future = get_client.call_async(GetParams.Request())
        spin_until(get_future.done)
        assert get_future.result().success and 'sitl_enabled' in get_future.result().config_json
        update_future = update_client.call_async(UpdateParams.Request(json='{"sitl":true}'))
        spin_until(update_future.done)
        assert update_future.result().success
        assert update_future.result().message == '{"sitl":true}'
    finally:
        executor.shutdown()
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()
