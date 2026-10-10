"""Minimal calibrated stereo geometry used by the localizer node."""

from __future__ import annotations

import math
from uv_camera.image_geometry import normalized_pixel


def camera_ray(info, pixel_x: float, pixel_y: float):
    x, y = normalized_pixel(info.k, getattr(info, 'd', ()), pixel_x, pixel_y)
    length = math.sqrt(x * x + y * y + 1.0)
    return (x / length, y / length, 1.0 / length)


def triangulate(left_info, right_info, left_x, right_x, pixel_y, baseline_m):
    lk = list(left_info.k)
    rk = list(right_info.k)
    fx = float(lk[0])
    disparity = float(left_x) - float(right_x)
    if fx <= 0.0 or baseline_m <= 0.0 or disparity <= 1e-4:
        return None
    z = fx * float(baseline_m) / disparity
    x = (float(left_x) - float(lk[2])) * z / fx
    y = (float(pixel_y) - float(lk[5])) * z / float(lk[4])
    return (x, y, z)
