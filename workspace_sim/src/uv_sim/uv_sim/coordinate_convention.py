"""Coordinate conversion between the Stonefish scene and project odom/NED.

The cruise scene is drawn such that, in its top view, Stonefish ``+Y`` points
to the pool's north and Stonefish ``-X`` points to its east.  The control,
localization and GUI interfaces use conventional NED instead: ``+X`` north,
``+Y`` east and ``+Z`` down.
"""

import math


def scene_to_odom_ned(x: float, y: float, z: float) -> tuple[float, float, float]:
    """Rotate a relative Stonefish scene vector into local odom/NED."""
    return float(y), -float(x), float(z)


def scene_yaw_to_odom_ned(yaw_rad: float) -> float:
    """Convert a Stonefish world yaw to the matching NED yaw, wrapped to π."""
    yaw = float(yaw_rad) - math.pi / 2.0
    return math.atan2(math.sin(yaw), math.cos(yaw))
