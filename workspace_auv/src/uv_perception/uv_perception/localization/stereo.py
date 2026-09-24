"""Minimal calibrated stereo geometry used by the localizer node."""

from __future__ import annotations

import math


def camera_ray(info, pixel_x: float, pixel_y: float):
    k = list(info.k)
    fx, fy, cx, cy = float(k[0]), float(k[4]), float(k[2]), float(k[5])
    if fx <= 0.0 or fy <= 0.0:
        return (0.0, 0.0, 1.0)
    x = (float(pixel_x) - cx) / fx
    y = (float(pixel_y) - cy) / fy
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
