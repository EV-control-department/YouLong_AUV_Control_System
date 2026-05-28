"""NED coordinate transforms and angle utilities."""

import math


def wrap_angle_rad(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def wrap_angle_deg(angle: float) -> float:
    """Wrap angle to [-180, 180]."""
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


def world_to_body(dx_w: float, dy_w: float, yaw_rad: float) -> tuple:
    """Rotate world-frame displacement to body frame (NED).

    Args:
        dx_w: North displacement (world)
        dy_w: East displacement (world)
        yaw_rad: Current yaw in radians

    Returns:
        (dx_b, dy_b): Body-frame displacement (forward, right)
    """
    cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
    dx_b = cy * dx_w + sy * dy_w
    dy_b = -sy * dx_w + cy * dy_w
    return dx_b, dy_b


def body_to_world(dx_b: float, dy_b: float, yaw_rad: float) -> tuple:
    """Rotate body-frame displacement to world frame (NED).

    Args:
        dx_b: Forward displacement (body)
        dy_b: Right displacement (body)
        yaw_rad: Current yaw in radians

    Returns:
        (dx_w, dy_w): World-frame displacement (North, East)
    """
    cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
    dx_w = cy * dx_b - sy * dy_b
    dy_w = sy * dx_b + cy * dy_b
    return dx_w, dy_w
