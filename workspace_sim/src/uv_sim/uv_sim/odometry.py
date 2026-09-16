"""``/auv/odometry`` (and IMU) decoding -- the single definition of the frames.

``nav_msgs/Odometry`` carries ``pose`` in ``header.frame_id`` (here
``world_ned``) but ``twist`` in ``child_frame_id``.  Stonefish honours that split
by rotating the rigid-body velocity into the sensor frame before filling the
message (``Odometry.cpp:59,65``: ``v = R_sensor<-world^T * v_point``); the
odometry sensor sits at the body origin with ``rpy~0``, so the twist *is* the
body velocity at the origin.

Rotating it again by yaw would turn ``v_body`` into ``R(-psi) * v_body`` (at
psi=90 deg a pure surge ``u`` becomes ``(0, -u)``), which corrupted both
``/zit6/state/vel`` (SIL) and ``/zit6/sim/nav`` (HIL) -- bug B12.

Moved verbatim out of ``sim_bridge`` (T5) so the two consumers that must agree on
this (`nav_sim`, which publishes the state topics, and `mcu_sim`, which closes the
PID loop) share one decoder instead of each owning a copy: the B12 class of bug
came from exactly such a copy.  Both fill the *same dict shape*, in **degrees**,
which is the internal convention of the control code (invariant I5: only ROS
messages carry radians).
"""

from __future__ import annotations

import math

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from uv_sim.mathutil import clamp


def body_twist_from_odometry(msg: Odometry) -> tuple[float, float, float, float, float, float]:
    """Return ``(u, v, w, p, q, r)`` in the **body** frame from ``/auv/odometry``.

    Linear parts are the velocity of the **body origin** -- not of the DVL
    acoustic centre, which sits 0.375 m forward and carries an extra
    ``omega x r_dvl`` term (that difference is bug B19; use
    ``dvl_sim``/``/sim/sensor/dvl`` for the DVL-frame value).
    """
    twist = msg.twist.twist
    return (
        twist.linear.x,
        twist.linear.y,
        twist.linear.z,
        twist.angular.x,
        twist.angular.y,
        twist.angular.z,
    )


def quat_to_rpy(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    """Roll/pitch/yaw (radians) from a quaternion; the usual ZYX extraction."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.asin(clamp(sinp, -1.0, 1.0))

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def apply_odometry_to_state(msg: Odometry, pos: dict, vel: dict) -> None:
    """Fill ``pos`` (NED, degrees) and ``vel`` (body, deg/s) from one odometry sample.

    ``pos`` gains ``x, y, z, rx, ry, rz``; ``vel`` gains ``x, y, z, rx, ry, rz``.
    The angular parts are refreshed again by the faster IMU (see
    :func:`apply_imu_to_state`); both sources carry the *same* physical quantity,
    unlike the DVL/odometry conflict of bug B19.
    """
    pos['x'] = msg.pose.pose.position.x
    pos['y'] = msg.pose.pose.position.y
    pos['z'] = msg.pose.pose.position.z

    q = msg.pose.pose.orientation
    roll, pitch, yaw = quat_to_rpy(q.x, q.y, q.z, q.w)
    pos['rx'] = math.degrees(roll)
    pos['ry'] = math.degrees(pitch)
    pos['rz'] = math.degrees(yaw)

    u, v, w, p, q_rate, r = body_twist_from_odometry(msg)
    vel['x'] = u
    vel['y'] = v
    vel['z'] = w
    vel['rx'] = math.degrees(p)
    vel['ry'] = math.degrees(q_rate)
    vel['rz'] = math.degrees(r)


def apply_imu_to_state(msg: Imu, vel: dict) -> None:
    """Refresh the body rates (deg/s) in ``vel`` from a 200 Hz IMU sample."""
    vel['rx'] = math.degrees(msg.angular_velocity.x)
    vel['ry'] = math.degrees(msg.angular_velocity.y)
    vel['rz'] = math.degrees(msg.angular_velocity.z)

