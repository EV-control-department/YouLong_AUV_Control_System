"""Real ROS action exchange against an isolated synthetic kinematic backend."""
import math
import threading
import time

import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('zit6_interfaces.msg')
pytest.importorskip('uv_msgs.action')

from geometry_msgs.msg import TwistWithCovarianceStamped
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from uv_msgs.action import BasicMotion
from uv_msgs.msg import PoseInfo
from zit6_interfaces.msg import ZitSetpoint, ZitStatus
from auv_protocol.topics import BASIC_MOTION, STATE_ODOM, STATE_TWIST, ZIT6_SETPOINT, ZIT6_STATUS
from uv_control.basic_motion import BasicMotionNode
from uv_control.line_guidance import rotate_body_to_world, rotate_world_to_body

if not hasattr(BasicMotion.Goal, 'WLINE'):
    pytest.skip('rebuilt WLINE action required', allow_module_level=True)


def wait(predicate, seconds=15.0):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.01)
    assert predicate(), 'ROS action/backend did not finish before test timeout'


@pytest.mark.parametrize('command', [BasicMotion.Goal.BLINE, BasicMotion.Goal.WLINE])
def test_real_action_success_and_zero_speed_preemption(tmp_path, command):
    # Never connect a synthetic control test to the vehicle's default domain.
    rclpy.init(domain_id=184)
    motion = BasicMotionNode()
    backend = Node('bline_synthetic_backend')
    executor = MultiThreadedExecutor(num_threads=6)
    thread = None
    messages = []
    pose = [0.0, 0.0, 0.0, 35.0]
    mode = [None]
    status_pub = backend.create_publisher(ZitStatus, ZIT6_STATUS, 10)
    odom_pub = backend.create_publisher(PoseInfo, STATE_ODOM, 10)
    twist_pub = backend.create_publisher(TwistWithCovarianceStamped, STATE_TWIST, 10)

    def receive(command):
        mode[0] = command
        messages.append(command)

    backend.create_subscription(ZitSetpoint, ZIT6_SETPOINT, receive, 10)

    def tick():
        command = mode[0]
        world = (0.0, 0.0, 0.0)
        if command is not None:
            if command.control_key == 0x11:
                world = rotate_body_to_world((command.x, command.y, command.z), 0, 0, pose[3])
                pose[3] += math.degrees(command.yaw) * .02
            else:
                world = tuple(max(-.3, min(.3, .8 * (b - p)))
                              for b, p in zip((command.x, command.y, command.z), pose[:3]))
                pose[3] = math.degrees(command.yaw)
        pose[:3] = [p + v * .02 for p, v in zip(pose[:3], world)]
        status = ZitStatus()
        status.is_armed = True
        status_pub.publish(status)
        odom = PoseInfo()
        odom.stamp = backend.get_clock().now().to_msg()
        odom.robot_x, odom.robot_y, odom.robot_z, odom.robot_yaw = pose
        odom.origin_initialized = odom.nav_valid = True
        odom.origin_generation = 1
        odom.nav_timestamp_ms = int(time.monotonic() * 1000) & 0xFFFFFFFF
        odom_pub.publish(odom)
        twist = TwistWithCovarianceStamped()
        twist.header.stamp = odom.stamp
        twist.header.frame_id = 'base_link'
        body = rotate_world_to_body(world, 0, 0, pose[3])
        twist.twist.twist.linear.x, twist.twist.twist.linear.y, twist.twist.twist.linear.z = body
        twist_pub.publish(twist)

    backend.create_timer(.02, tick)
    client = ActionClient(backend, BasicMotion, BASIC_MOTION)
    try:
        for instance in (motion, backend):
            executor.add_node(instance)
        # Represent an already completed START; START handshakes are covered
        # by test_start_origin and test_sil_origin_integration.
        with motion._state_lock:
            motion._started = motion._heartbeat_enabled = True
            motion._active_origin_generation = 1
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        assert client.wait_for_server(timeout_sec=5)
        wait(motion._motion_ready)
        goal = BasicMotion.Goal()
        goal.cmd_type = command
        goal.axes = 'xyz'
        goal.target = [0.0, .25, .1, -45.0]
        goal.cruise_speed = .17
        goal.timeout = 15.0
        feedback = []
        send = client.send_goal_async(goal, feedback_callback=lambda msg: feedback.append(msg.feedback))
        wait(send.done)
        handle = send.result()
        assert handle.accepted
        result = handle.get_result_async()
        wait(result.done)
        assert result.result().result.success
        target = result.result().result.final_target
        expected = ([-.25 * math.sin(math.radians(35)), .25 * math.cos(math.radians(35)), .1, -10.]
                    if command == BasicMotion.Goal.BLINE else [0., .25, .1, -45.])
        assert target == pytest.approx(expected, abs=1e-4)
        assert abs((pose[3]-expected[3]+180)%360-180) <= 5.
        assert messages[-1].control_key == 0
        assert any(item.phase == 'HOLD' for item in feedback)
        assert not motion._velocity_active

        goal.target = [2.0, 0.0, 0.0, 0.0]
        send = client.send_goal_async(goal)
        wait(send.done)
        handle = send.result()
        assert handle.accepted
        result = handle.get_result_async()
        wait(lambda: motion._velocity_active)
        stop = BasicMotion.Goal()
        stop.cmd_type = BasicMotion.Goal.BODY_VELOCITY
        stop.target = [0.0] * 4
        stop_send = client.send_goal_async(stop)
        wait(stop_send.done)
        assert stop_send.result().accepted
        wait(result.done)
        assert not result.result().result.success and not motion._velocity_active
        wait(lambda: messages[-1].control_key == 0)
        velocities = [m for m in messages if m.control_key == 0x11]
        assert (velocities[-1].x, velocities[-1].y, velocities[-1].z, velocities[-1].yaw) == (0, 0, 0, 0)
    finally:
        executor.shutdown(timeout_sec=5)
        if thread is not None:
            thread.join(timeout=5)
        client.destroy()
        motion.destroy_node()
        backend.destroy_node()
        rclpy.shutdown()
