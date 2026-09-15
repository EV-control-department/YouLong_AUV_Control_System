"""Frame-convention regressions for ``sim_bridge``'s velocity channels.

``/auv/odometry`` is a ``nav_msgs/Odometry`` whose **twist is published in the
child (body) frame** -- Stonefish applies ``R_sensor<-world^T`` before filling
the message (``stonefish/Library/src/sensors/scalar/Odometry.cpp:59,65``, and
``ROS2Interface::PublishOdometry`` sets ``header.frame_id="world_ned"`` /
``child_frame_id="<sensor>"``).  Rotating that twist by yaw a second time
produces ``R(-psi) * v_body``: at psi=90 deg a pure surge of 0.3 m/s is reported
as ``(0, -0.3)``.

Two channels consume it:

* ``/zit6/state/vel`` (SIL) -- filled by ``_odom_cb`` and published at 60 Hz;
* ``/zit6/sim/nav`` (HIL) -- 12 floats whose contract is ``pos_world`` +
  ``vel_body`` (``third_party/AUV_zit6_cmake/docs/protocols/与AGX.md``).

Both must therefore pass the twist through unchanged.  These tests drive the
callbacks with a synthetic message whose pose yaw is deliberately non-zero, so
a reintroduced rotation fails immediately.
"""

from __future__ import annotations

import math

import pytest
import rclpy
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter

from uv_sim.sim_bridge import SimBridgeNode

YAW_DEG = 90.0
BODY_LINEAR = (0.3, -0.2, 0.05)      # u, v, w in m/s
BODY_ANGULAR = (0.01, -0.02, 0.5)    # p, q, r in rad/s


def _odometry_with_yaw(yaw_deg: float) -> Odometry:
    """A body-frame-twist odometry message whose pose is rotated by ``yaw_deg``."""
    msg = Odometry()
    msg.header.frame_id = "world_ned"
    msg.child_frame_id = "dynamics"

    msg.pose.pose.position.x = 1.0
    msg.pose.pose.position.y = 2.0
    msg.pose.pose.position.z = 3.0
    half = math.radians(yaw_deg) / 2.0
    msg.pose.pose.orientation.x = 0.0
    msg.pose.pose.orientation.y = 0.0
    msg.pose.pose.orientation.z = math.sin(half)
    msg.pose.pose.orientation.w = math.cos(half)

    msg.twist.twist.linear.x = BODY_LINEAR[0]
    msg.twist.twist.linear.y = BODY_LINEAR[1]
    msg.twist.twist.linear.z = BODY_LINEAR[2]
    msg.twist.twist.angular.x = BODY_ANGULAR[0]
    msg.twist.twist.angular.y = BODY_ANGULAR[1]
    msg.twist.twist.angular.z = BODY_ANGULAR[2]
    return msg


@pytest.fixture(scope="module")
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture(scope="module")
def sil_node(ros_context):
    """SIL node: owns the ``/zit6/state/vel`` channel."""
    node = SimBridgeNode(node_name="sim_bridge_sil_test")
    yield node
    node.destroy_node()


@pytest.fixture(scope="module")
def hil_node(ros_context):
    """HIL node: owns the ``/zit6/sim/nav`` aggregation."""
    node = SimBridgeNode(
        node_name="sim_bridge_hil_test",
        parameter_overrides=[Parameter("hil_mode", value=True)],
    )
    yield node
    node.destroy_node()


def test_state_vel_linear_is_body_frame(sil_node) -> None:
    sil_node._odom_cb(_odometry_with_yaw(YAW_DEG))

    assert sil_node.vel["x"] == pytest.approx(BODY_LINEAR[0], abs=1e-9)
    assert sil_node.vel["y"] == pytest.approx(BODY_LINEAR[1], abs=1e-9)
    assert sil_node.vel["z"] == pytest.approx(BODY_LINEAR[2], abs=1e-9)


def test_state_vel_angular_passthrough_is_degrees(sil_node) -> None:
    sil_node._odom_cb(_odometry_with_yaw(YAW_DEG))

    # self.vel stores degrees per second (see _publish_vel).
    assert sil_node.vel["rz"] == pytest.approx(
        math.degrees(BODY_ANGULAR[2]), abs=1e-9
    )


def test_pose_is_still_world_frame(sil_node) -> None:
    sil_node._odom_cb(_odometry_with_yaw(YAW_DEG))

    assert sil_node.pos["x"] == pytest.approx(1.0)
    assert sil_node.pos["y"] == pytest.approx(2.0)
    assert sil_node.pos["z"] == pytest.approx(3.0)
    assert sil_node.pos["rz"] == pytest.approx(YAW_DEG, abs=1e-6)


def test_sim_nav_velocity_is_body_frame(hil_node) -> None:
    hil_node._sim_nav_odom_cb(_odometry_with_yaw(YAW_DEG))

    assert hil_node._sim_vel[0] == pytest.approx(BODY_LINEAR[0], abs=1e-9)
    assert hil_node._sim_vel[1] == pytest.approx(BODY_LINEAR[1], abs=1e-9)
    assert hil_node._sim_vel[2] == pytest.approx(BODY_LINEAR[2], abs=1e-9)


def test_sim_nav_position_is_world_frame(hil_node) -> None:
    hil_node._sim_nav_odom_cb(_odometry_with_yaw(YAW_DEG))

    assert hil_node._sim_pos[:3] == pytest.approx([1.0, 2.0, 3.0])
    assert hil_node._sim_pos[5] == pytest.approx(math.radians(YAW_DEG), abs=1e-6)
