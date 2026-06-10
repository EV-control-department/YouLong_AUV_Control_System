"""Coordinate transform utilities — backward-compatible wrappers.

Delegates to Coordinate class. Kept for existing import compatibility.
"""

from uv_control.coordinate import Coordinate

_world_ref = Coordinate()


def wrap_angle_rad(angle: float) -> float:
    return Coordinate.wrap_rad(angle)


def wrap_angle_deg(angle: float) -> float:
    return Coordinate.wrap_deg(angle)


def world_to_body(dx_w: float, dy_w: float, yaw_rad: float) -> tuple:
    c = Coordinate(rz=yaw_rad)
    r = c.world_to_body(dx_w, dy_w)
    return r.x, r.y


def body_to_world(dx_b: float, dy_b: float, yaw_rad: float) -> tuple:
    c = Coordinate(rz=yaw_rad)
    r = c.body_to_world(dx_b, dy_b)
    return r.x, r.y
