"""``/auv/odometry`` frame conventions -- the single decoder for its twist.

``nav_msgs/Odometry`` carries ``pose`` in ``header.frame_id`` (here
``world_ned``) but ``twist`` in ``child_frame_id``.  Stonefish honours that split
by rotating the rigid-body velocity into the sensor frame before filling the
message (``Odometry.cpp:59,65``: ``v = R_sensor<-world^T * v_point``); the
odometry sensor sits at the body origin with ``rpy~0``, so the twist *is* the
body velocity at the origin.

Rotating it again by yaw would turn ``v_body`` into ``R(-psi) * v_body`` (at
psi=90 deg a pure surge ``u`` becomes ``(0, -u)``), which corrupted both
``/zit6/state/vel`` (SIL) and ``/zit6/sim/nav`` (HIL) -- bug B12.

Moved verbatim out of ``sim_bridge`` (T5) so the two consumers that must agree
(``nav_sim`` for the state topics, ``mcu_sim`` for the PID feedback) share this
one function instead of each owning a copy.
"""

from __future__ import annotations

from nav_msgs.msg import Odometry


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
