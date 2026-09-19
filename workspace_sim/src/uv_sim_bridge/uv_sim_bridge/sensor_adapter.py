"""Stonefish sensor messages to canonical AUV sensor messages."""

from __future__ import annotations

import math

try:
    from stonefish_ros2.msg import DVL
except ImportError:  # pragma: no cover - only used outside a Stonefish install
    DVL = None

from auv_protocol.topics import (
    DVL_ALTITUDE,
    DVL_VELOCITY,
    SIM_RAW_DVL_ALTITUDE,
    SIM_RAW_DVL_VELOCITY,
)
from sensor_msgs.msg import Range
from uv_msgs.msg import DvlAltitude, DvlVelocity


# Stonefish reports the DVL velocity in the frame described by the sensor
# origin. YouLong's simulated DVL is mounted as
# ``rpy="3.1416 0 -0.785003"`` in youlong.scn. The estimator consumes body
# FRD velocity, so apply that fixed sensor-to-body rotation at this boundary.
# The real vehicle DVL pose is intentionally still unspecified in the real
# geometry configuration and is not silently reused here.
_DVL_ROLL = math.pi
_DVL_PITCH = 0.0
_DVL_YAW = -0.785003


def dvl_to_body_velocity(x: float, y: float, z: float) -> tuple[float, float, float]:
    """Convert the simulated Stonefish DVL vector into body-FRD axes."""
    # RPY rotation matching the Stonefish sensor origin. Keeping the
    # elementary form avoids introducing a geometry dependency into this
    # small transport adapter.
    cr, sr = math.cos(_DVL_ROLL), math.sin(_DVL_ROLL)
    cp, sp = math.cos(_DVL_PITCH), math.sin(_DVL_PITCH)
    cy, sy = math.cos(_DVL_YAW), math.sin(_DVL_YAW)
    # R = Rz(yaw) * Ry(pitch) * Rx(roll)
    r00 = cy * cp
    r01 = cy * sp * sr - sy * cr
    r02 = cy * sp * cr + sy * sr
    r10 = sy * cp
    r11 = sy * sp * sr + cy * cr
    r12 = sy * sp * cr - cy * sr
    r20 = -sp
    r21 = cp * sr
    r22 = cp * cr
    return (
        r00 * x + r01 * y + r02 * z,
        r10 * x + r11 * y + r12 * z,
        r20 * x + r21 * y + r22 * z,
    )


class SensorAdapter:
    """Own only Stonefish DVL conversion and canonical sensor publication."""

    def __init__(self, node, velocity_publisher=None, altitude_publisher=None):
        self.node = node
        self.velocity_pub = velocity_publisher or node.create_publisher(
            DvlVelocity, DVL_VELOCITY, 10)
        self.altitude_pub = altitude_publisher or node.create_publisher(
            DvlAltitude, DVL_ALTITUDE, 10)
        self.subscriptions = []
        if DVL is not None:
            self.subscriptions.append(node.create_subscription(
                DVL, SIM_RAW_DVL_VELOCITY, self._dvl_cb, 10))
        self.subscriptions.append(node.create_subscription(
            Range, SIM_RAW_DVL_ALTITUDE, self._altitude_cb, 10))

    def _dvl_cb(self, msg) -> None:
        vx, vy, vz = dvl_to_body_velocity(
            msg.velocity.x, msg.velocity.y, msg.velocity.z)
        canonical = DvlVelocity()
        canonical.header = msg.header
        # The vector below is now expressed at the vehicle reference point in
        # body-FRD axes. Keep the canonical frame id consistent with that
        # contract instead of leaking Stonefish's sensor-frame name.
        canonical.header.frame_id = 'base_link'
        canonical.velocity.x = vx
        canonical.velocity.y = vy
        canonical.velocity.z = vz
        canonical.velocity_covariance = list(msg.velocity_covariance)
        canonical.valid = all(math.isfinite(float(value)) for value in (
            vx, vy, vz))
        self.velocity_pub.publish(canonical)

    def _altitude_cb(self, msg: Range) -> None:
        canonical = DvlAltitude()
        canonical.header = msg.header
        canonical.altitude = float(msg.range)
        canonical.valid = math.isfinite(canonical.altitude) and canonical.altitude >= 0.0
        self.altitude_pub.publish(canonical)
