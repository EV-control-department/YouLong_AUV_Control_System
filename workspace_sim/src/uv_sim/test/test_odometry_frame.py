"""Frame-convention regressions for the velocity channels that consume odometry.

``/auv/odometry`` is a ``nav_msgs/Odometry`` whose **twist is published in the
child (body) frame** -- Stonefish applies ``R_sensor<-world^T`` before filling the
message (``stonefish/Library/src/sensors/scalar/Odometry.cpp:59,65``, and
``ROS2Interface::PublishOdometry`` sets ``header.frame_id="world_ned"`` /
``child_frame_id="<sensor>"``).  Rotating that twist by yaw a second time produces
``R(-psi) * v_body``: at psi=90 deg a pure surge of 0.3 m/s is reported as
``(0, -0.3)``.  That was bug B12.

After T5 the decode lives in exactly one place (``uv_sim.odometry``) and has two
consumers, both covered here:

* ``nav_sim`` -- fills ``/zit6/state/{pos,vel}`` (SIL) and ``/zit6/sim/nav`` (HIL);
* ``mcu_sim`` -- feeds the PID loops.

Each fixture drives the callback with a synthetic message whose pose yaw is
deliberately non-zero, so a reintroduced rotation fails immediately.
"""

from __future__ import annotations

import math

import pytest
import rclpy
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter

from uv_sim.mcu_sim import McuSimNode
from uv_sim.nav_sim import NavSimNode

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
def state_node(ros_context):
    """SIL role: owns the ``/zit6/state/{pos,vel}`` channels."""
    node = NavSimNode(node_name="nav_sim_sil_test")
    yield node
    node.destroy_node()


@pytest.fixture(scope="module")
def injection_node(ros_context):
    """HIL role: owns the ``/zit6/sim/nav`` injection."""
    node = NavSimNode(
        node_name="nav_sim_hil_test",
        parameter_overrides=[
            Parameter("publish_state", value=False),
            Parameter("publish_sim_nav", value=True),
        ],
    )
    yield node
    node.destroy_node()


@pytest.fixture(scope="module")
def mcu_node(ros_context):
    """The PID consumer, which must decode the same twist the same way."""
    node = McuSimNode(node_name="mcu_sim_frame_test")
    yield node
    node.destroy_node()


# --------------------------------------------------------------------------- #
# nav_sim: the state topics (SIL)
# --------------------------------------------------------------------------- #
def test_state_vel_linear_is_body_frame(state_node) -> None:
    state_node._odom_cb(_odometry_with_yaw(YAW_DEG))

    assert state_node.vel["x"] == pytest.approx(BODY_LINEAR[0], abs=1e-9)
    assert state_node.vel["y"] == pytest.approx(BODY_LINEAR[1], abs=1e-9)
    assert state_node.vel["z"] == pytest.approx(BODY_LINEAR[2], abs=1e-9)


def test_state_vel_angular_passthrough_is_degrees(state_node) -> None:
    state_node._odom_cb(_odometry_with_yaw(YAW_DEG))

    # the internal dict stores degrees per second (see nav_sim._publish_vel)
    assert state_node.vel["rz"] == pytest.approx(
        math.degrees(BODY_ANGULAR[2]), abs=1e-9
    )


def test_pose_is_still_world_frame(state_node) -> None:
    state_node._odom_cb(_odometry_with_yaw(YAW_DEG))

    assert state_node.pos["x"] == pytest.approx(1.0)
    assert state_node.pos["y"] == pytest.approx(2.0)
    assert state_node.pos["z"] == pytest.approx(3.0)
    assert state_node.pos["rz"] == pytest.approx(YAW_DEG, abs=1e-6)


# --------------------------------------------------------------------------- #
# nav_sim: the injection (HIL)
# --------------------------------------------------------------------------- #
def test_sim_nav_velocity_is_body_frame(injection_node) -> None:
    injection_node._odom_cb(_odometry_with_yaw(YAW_DEG))

    assert injection_node._sim_vel[0] == pytest.approx(BODY_LINEAR[0], abs=1e-9)
    assert injection_node._sim_vel[1] == pytest.approx(BODY_LINEAR[1], abs=1e-9)
    assert injection_node._sim_vel[2] == pytest.approx(BODY_LINEAR[2], abs=1e-9)


def test_sim_nav_position_is_world_frame(injection_node) -> None:
    injection_node._odom_cb(_odometry_with_yaw(YAW_DEG))

    assert injection_node._sim_pos[:3] == pytest.approx([1.0, 2.0, 3.0])
    assert injection_node._sim_pos[5] == pytest.approx(math.radians(YAW_DEG), abs=1e-6)


# --------------------------------------------------------------------------- #
# mcu_sim: the PID feedback path
# --------------------------------------------------------------------------- #
def test_mcu_state_velocity_is_body_frame(mcu_node) -> None:
    mcu_node._odom_cb(_odometry_with_yaw(YAW_DEG))

    assert mcu_node._controller.vel["x"] == pytest.approx(BODY_LINEAR[0], abs=1e-9)
    assert mcu_node._controller.vel["y"] == pytest.approx(BODY_LINEAR[1], abs=1e-9)
    assert mcu_node._controller.current_pose_ready is True


def test_mcu_pose_is_world_frame(mcu_node) -> None:
    mcu_node._odom_cb(_odometry_with_yaw(YAW_DEG))

    assert mcu_node._controller.pos["x"] == pytest.approx(1.0)
    assert mcu_node._controller.pos["rz"] == pytest.approx(YAW_DEG, abs=1e-6)
