"""``nav_sim``: the SIL/HIL role switch and the two state contracts.

The node exists as **one** node with two mutually exclusive parameters (chosen
over two nodes so the SIL/HIL switch stays a launch decision, not a node-set
decision).  What has to be true:

* both parameters true is a startup **error** -- that configuration would put two
  publishers on ``/zit6/state/*`` (invariant I2), which is the single most
  expensive silent failure in this stack;
* ``publish_state`` produces the 6-element ``[x, y, z, roll_rad, pitch_rad,
  yaw_rad]`` / body-velocity messages the MCU contract describes;
* ``publish_sim_nav`` produces the 12-float ``pos_world + vel_body`` injection;

and, for both, the linear velocity is the velocity of the **body origin** taken
from odometry -- not the DVL's value, which differs by ``omega x r_dvl`` (B19).
"""

from __future__ import annotations

import math

import pytest
import rclpy
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from sensor_msgs.msg import Imu

from uv_sim.nav_sim import NavSimNode


@pytest.fixture(scope="module")
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


class _Recorder:
    def __init__(self) -> None:
        self.messages: list = []

    def publish(self, msg) -> None:
        self.messages.append(msg)


def _node(ros_context, name: str, **overrides):
    parameters = [Parameter(key, value=value) for key, value in overrides.items()]
    return NavSimNode(node_name=name, parameter_overrides=parameters or None)


def _odometry(yaw_deg: float = 0.0, linear=(0.3, -0.2, 0.05), angular=(0.01, -0.02, 0.5)):
    msg = Odometry()
    msg.header.frame_id = "world_ned"
    msg.child_frame_id = "dynamics"
    msg.pose.pose.position.x = 1.0
    msg.pose.pose.position.y = 2.0
    msg.pose.pose.position.z = 3.0
    half = math.radians(yaw_deg) / 2.0
    msg.pose.pose.orientation.z = math.sin(half)
    msg.pose.pose.orientation.w = math.cos(half)
    msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z = linear
    msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z = angular
    return msg


def _imu(angular=(0.01, -0.02, 0.5)):
    msg = Imu()
    msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = angular
    return msg


# --------------------------------------------------------------------------- #
# The role switch
# --------------------------------------------------------------------------- #
def test_both_roles_true_is_a_startup_error(ros_context) -> None:
    with pytest.raises(RuntimeError, match="cannot both be true"):
        _node(ros_context, "nav_sim_both_true",
              publish_state=True, publish_sim_nav=True)


def test_both_roles_false_only_warns(ros_context) -> None:
    """A directly-run node with no output is useless, but must not be fatal."""
    node = _node(ros_context, "nav_sim_both_false",
                 publish_state=False, publish_sim_nav=False)
    try:
        assert not hasattr(node, "zit6_pos_pub")
        assert not hasattr(node, "zit6_vel_pub")
        assert not hasattr(node, "sim_nav_pub")
    finally:
        node.destroy_node()


def test_state_mode_creates_only_the_state_publishers(ros_context) -> None:
    node = _node(ros_context, "nav_sim_state_mode")
    try:
        assert hasattr(node, "zit6_pos_pub") and hasattr(node, "zit6_vel_pub")
        assert not hasattr(node, "sim_nav_pub"), (
            "creating the sim/nav publisher in SIL mode would advertise an HIL-only "
            "topic with no traffic"
        )
    finally:
        node.destroy_node()


def test_sim_nav_mode_creates_only_the_injection_publisher(ros_context) -> None:
    node = _node(ros_context, "nav_sim_hil_mode",
                 publish_state=False, publish_sim_nav=True)
    try:
        assert hasattr(node, "sim_nav_pub")
        assert not hasattr(node, "zit6_pos_pub")
        assert not hasattr(node, "zit6_vel_pub")
    finally:
        node.destroy_node()


# --------------------------------------------------------------------------- #
# The state contract (SIL)
# --------------------------------------------------------------------------- #
def test_state_messages_have_the_documented_shape(ros_context) -> None:
    node = _node(ros_context, "nav_sim_state_shape")
    try:
        node._odom_cb(_odometry(yaw_deg=90.0))
        node._imu_cb(_imu())

        pos_recorder, vel_recorder = _Recorder(), _Recorder()
        node.zit6_pos_pub, node.zit6_vel_pub = pos_recorder, vel_recorder
        node._publish_pos()
        node._publish_vel()

        assert len(pos_recorder.messages[0].data) == 6
        assert list(pos_recorder.messages[0].data) == pytest.approx(
            [1.0, 2.0, 3.0, 0.0, 0.0, math.radians(90.0)], abs=1e-6
        )
        # body-frame velocity, angular from the IMU in rad/s
        assert list(vel_recorder.messages[0].data) == pytest.approx(
            [0.3, -0.2, 0.05, 0.01, -0.02, 0.5], abs=1e-6
        )
    finally:
        node.destroy_node()


def test_state_velocity_is_the_body_origin_velocity(ros_context) -> None:
    """B19: the linear channel must be the origin twist, never omega x r_dvl."""
    node = _node(ros_context, "nav_sim_origin_velocity")
    try:
        node._odom_cb(_odometry(yaw_deg=90.0, linear=(0.3, 0.0, 0.0)))
        node._publish_vel()   # no crash without a subscriber

        assert node.vel["x"] == pytest.approx(0.3)
        assert node.vel["y"] == pytest.approx(0.0)
    finally:
        node.destroy_node()


# --------------------------------------------------------------------------- #
# The injection contract (HIL)
# --------------------------------------------------------------------------- #
def test_sim_nav_payload_is_pos_world_then_vel_body(ros_context) -> None:
    node = _node(ros_context, "nav_sim_payload",
                 publish_state=False, publish_sim_nav=True)
    try:
        recorder = _Recorder()
        node.sim_nav_pub = recorder

        # The IMU only refreshes the rates; the odometry callback does the
        # publishing (kept from the pre-T5 aggregation), so the 200 Hz IMU usually
        # lands first and the 30 Hz odometry sample carries the freshest rates.
        node._imu_cb(_imu())
        node._odom_cb(_odometry(yaw_deg=90.0))

        data = list(recorder.messages[-1].data)
        assert len(data) == 12
        assert data[:3] == pytest.approx([1.0, 2.0, 3.0])          # pos_world
        assert data[5] == pytest.approx(math.radians(90.0), abs=1e-6)  # yaw in radians
        assert data[6:9] == pytest.approx([0.3, -0.2, 0.05])       # vel_body
        assert data[9:12] == pytest.approx([0.01, -0.02, 0.5])     # body rates
    finally:
        node.destroy_node()


def test_the_imu_refreshes_rates_without_publishing(ros_context) -> None:
    """An IMU sample must not emit a nav injection of its own (30 Hz contract)."""
    node = _node(ros_context, "nav_sim_imu_only",
                 publish_state=False, publish_sim_nav=True)
    try:
        node.sim_nav_pub = _Recorder()
        node._imu_cb(_imu())

        assert node.sim_nav_count == 0
        assert node._sim_vel[3:6] == pytest.approx([0.01, -0.02, 0.5])
    finally:
        node.destroy_node()


def test_sim_nav_is_published_once_per_odometry_sample(ros_context) -> None:
    node = _node(ros_context, "nav_sim_rate",
                 publish_state=False, publish_sim_nav=True)
    try:
        node.sim_nav_pub = _Recorder()
        for _ in range(3):
            node._odom_cb(_odometry())
        node._imu_cb(_imu())          # the IMU refreshes rates, it does not publish

        assert node.sim_nav_count == 3
    finally:
        node.destroy_node()
