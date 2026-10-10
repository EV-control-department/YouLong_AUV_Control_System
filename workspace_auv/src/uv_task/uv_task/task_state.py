"""Shared mission state and the coordinate bias owned by one task runner.

``bias = [dx_m, dy_m, dz_m, dyaw_deg]`` maps raw odom into task coordinates:

    task_xy = R(dyaw) @ odom_xy + [dx, dy]
    task_z = odom_z + dz
    task_yaw = wrap(odom_yaw + dyaw)

With zero yaw bias this is ordinary position addition. A nonzero yaw bias
rotates world XY coordinates as well as heading so body/world conversions
remain consistent. Configured object positions stay in task coordinates.
Use the inverse mapping for absolute controller targets and the forward
mapping for controller endpoints and newly received world measurements.

This module holds state and explicit conversions. It does not modify ROS
messages, estimate drift, or change an already running controller goal.
"""
from __future__ import annotations

import math
from dataclasses import replace
from threading import RLock
from uv_task.pickup_progress import PickupProgress
from typing import Iterable


def _finite_values(values: Iterable[float], sizes: tuple[int, ...], name: str) -> tuple[float, ...]:
    try:
        result = tuple(values)
    except TypeError as error:
        raise ValueError(f'{name} 必须是数值序列') from error
    if len(result) not in sizes:
        raise ValueError(f'{name} 的长度必须为 {sizes}')
    if any(isinstance(value, bool) or not isinstance(value, (int, float))
           or not math.isfinite(value) for value in result):
        raise ValueError(f'{name} 必须包含有限数值')
    return tuple(float(value) for value in result)


def _wrap_degrees(angle: float) -> float:
    return (angle + 180.0) % 360.0 - 180.0


class TaskState:
    """One bias shared by all tasks in the same runner process.

    Bias defaults to zero and survives task changes. ``reset_bias()`` starts
    a new coordinate reference explicitly. Restarting the runner creates a
    new zero state. Read-only snapshots prevent a reader from observing a
    partly updated four-value bias: assign the whole ``bias`` or use
    ``set_bias([dx, dy, dz, dyaw])`` instead of editing one element in place.
    """
    def __init__(self, bias: Iterable[float] = (0.0, 0.0, 0.0, 0.0)):
        self._lock = RLock()
        self._bias = (0.0, 0.0, 0.0, 0.0)
        self._pickup = PickupProgress()
        self.set_bias(bias)

    @property
    def bias(self) -> tuple[float, float, float, float]:
        with self._lock:
            return self._bias

    @bias.setter
    def bias(self, values: Iterable[float]) -> None:
        self.set_bias(values)

    def set_bias(self, values: Iterable[float]) -> None:
        """Atomically replace [dx_m, dy_m, dz_m, dyaw_deg]."""
        dx, dy, dz, yaw = _finite_values(values, (4,), 'bias')
        with self._lock:
            self._bias = (dx, dy, dz, _wrap_degrees(yaw))

    def reset_bias(self) -> None:
        self.set_bias((0.0, 0.0, 0.0, 0.0))

    def _map(self, values: Iterable[float], *, inverse: bool, translation: bool) -> tuple[float, ...]:
        # Points and vectors have three values; poses have four or six.
        sizes = (3, 4, 6) if translation else (3,)
        result = list(_finite_values(values, sizes, '坐标'))
        dx, dy, dz, yaw = self.bias
        angle = math.radians(yaw)
        c, s = math.cos(angle), math.sin(angle)
        x, y, z = result[:3]
        if inverse:
            if translation:
                x, y, z = x-dx, y-dy, z-dz
            result[:3] = (c*x+s*y, -s*x+c*y, z)
        else:
            result[:3] = (c*x-s*y, s*x+c*y, z)
            if translation:
                result[:3] = (result[0]+dx, result[1]+dy, result[2]+dz)
        if len(result) in (4, 6):
            result[-1] = _wrap_degrees(result[-1] + (-yaw if inverse else yaw))
        return tuple(result)

    def odom_to_task(self, values: Iterable[float]) -> tuple[float, ...]:
        """Convert XYZ, XYZ+yaw, or XYZ+roll/pitch/yaw (angles in degrees)."""
        return self._map(values, inverse=False, translation=True)

    def task_to_odom(self, values: Iterable[float]) -> tuple[float, ...]:
        """Convert a task point/pose back to the controller's raw odom."""
        return self._map(values, inverse=True, translation=True)

    def odom_vector_to_task(self, values: Iterable[float]) -> tuple[float, ...]:
        """Rotate a world displacement/ray/velocity without adding position bias."""
        return self._map(values, inverse=False, translation=False)

    def task_vector_to_odom(self, values: Iterable[float]) -> tuple[float, ...]:
        """Inverse world-vector mapping; body-frame commands use no bias."""
        return self._map(values, inverse=True, translation=False)


    @property
    def pickup(self) -> PickupProgress:
        """Immutable raw-odom frame anchor, attempt counters and pickup results."""
        with self._lock:
            return self._pickup

    def reset_pickup(self, start_xy=None) -> None:
        with self._lock:
            origin = self._pickup.start_xy if start_xy is None else _finite_values(start_xy, (2,), '出发点')
            self._pickup = PickupProgress(start_xy=origin)

    def update_pickup(self, **values) -> None:
        for key in ('frame_pose', 'golf_camera_pose', 'ring_camera_pose', 'last_servo_pose'):
            if key in values and values[key] is not None:
                values[key] = _finite_values(values[key], (4,), key)
        if 'first_frame_seen_at' in values and values['first_frame_seen_at'] is not None:
            stamp = values['first_frame_seen_at']
            if isinstance(stamp, bool) or not isinstance(stamp, (int, float)) or not math.isfinite(stamp):
                raise ValueError('first_frame_seen_at必须是有限单调时间')
        if 'depth' in values:
            values['depth'] = _finite_values((values['depth'], 0.0, 0.0), (3,), '抓取深度')[0]
        for key in ('golf_attempts', 'ring_attempts'):
            if key in values and (isinstance(values[key], bool) or not isinstance(values[key], int) or values[key] < 0):
                raise ValueError(key+'必须是非负整数')
        for key in ('golf_status', 'ring_status'):
            if key in values and values[key] not in ('pending', 'success', 'failed'):
                raise ValueError(key+'状态无效')
        with self._lock:
            self._pickup = replace(self._pickup, **values)
