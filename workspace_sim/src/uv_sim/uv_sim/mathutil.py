"""Scalar helpers shared by the simulation control code.

Deliberately ROS-free and numpy-free: :mod:`uv_sim.geometry` owns the frame and
DVL geometry, this module owns the two scalar clamps the control path needs.
Keeping them here (instead of copy-pasting ``max(low, min(high, value))`` into
the PID, the mixer and the MCU controller) is what makes "one clamp, one
behaviour" checkable.
"""

from __future__ import annotations


def clamp(value: float, low: float, high: float) -> float:
    """Return ``value`` limited to ``[low, high]``."""
    return max(low, min(high, value))


def wrap_angle_deg(angle: float) -> float:
    """Wrap an angle in degrees into ``(-180, 180]``.

    Used for the yaw error of the position loop: without it a target of -179 deg
    against a current +179 deg is a 358 deg error instead of -2 deg.
    """
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle
