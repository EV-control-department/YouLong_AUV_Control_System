"""Coordinate conversion at the Stonefish-to-AUV adapter boundary."""

from __future__ import annotations

import math


def scene_to_odom_ned(
        x: float, y: float, z: float) -> tuple[float, float, float]:
    """Rotate a relative Stonefish scene vector into local NED."""
    return float(y), -float(x), float(z)


def scene_yaw_to_odom_ned(yaw_rad: float) -> float:
    """Convert Stonefish world yaw to local NED yaw, wrapped to pi."""
    yaw = float(yaw_rad) - math.pi / 2.0
    return math.atan2(math.sin(yaw), math.cos(yaw))


__all__ = ['scene_to_odom_ned', 'scene_yaw_to_odom_ned']
