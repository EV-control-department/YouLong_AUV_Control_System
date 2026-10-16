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
from dataclasses import dataclass, replace
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


@dataclass(frozen=True)
class DownTargetObservation:
    xy: tuple[float, float]
    center_distance: float
    observed_at: float
    camera: str


DOWN_TARGET_DEFAULTS = {'frame': (.7, .8, .02), 'rack': (.8, .8, .02)}


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
        self._down_targets = {}
        self._down_target_keys = {}
        self._down_target_settings = dict(DOWN_TARGET_DEFAULTS)
        self._down_target_epoch = 0
        self._down_target_after = float('-inf')
        self._odom_generation = None
        self.set_bias(bias)

    def configure_down_target(self, kind, projection_depth, freshness, min_confidence=.02):
        if kind not in DOWN_TARGET_DEFAULTS:
            raise ValueError('下视预设点仅支持 frame/rack')
        depth, age, confidence = _finite_values(
            (projection_depth, freshness, min_confidence), (3,), '下视观测参数')
        if depth <= 0 or age <= 0 or not 0 <= confidence <= 1:
            raise ValueError('投影尺度和检测有效期须为正，置信度须在[0,1]')
        with self._lock:
            if depth != self._down_target_settings[kind][0]:
                self._down_targets.pop(kind, None)
            self._down_target_settings[kind] = (depth, age, confidence)

    def down_target_settings(self, kind):
        with self._lock:
            return self._down_target_settings[kind]

    @property
    def down_target_epoch(self):
        with self._lock:
            return self._down_target_epoch

    def claim_down_target_frame(self, kind, camera, key, received, observed_at, epoch):
        """Consume a source frame once, including frames with no valid target."""
        with self._lock:
            if (epoch != self._down_target_epoch or received <= self._down_target_after
                    or observed_at <= self._down_target_after):
                return False
            previous = self._down_target_keys.get((kind, camera))
            if previous is not None and key[0] == previous[0] and key[1] <= previous[1]:
                return False
            self._down_target_keys[(kind, camera)] = key
            return True

    def remember_down_target(self, kind, xy, center_distance, observed_at, camera, *, epoch=None):
        if kind not in DOWN_TARGET_DEFAULTS or camera not in ('down_left', 'down_right'):
            raise ValueError('无效下视目标或相机')
        point = _finite_values(xy, (2,), '下视目标世界XY')
        distance, stamp = _finite_values((center_distance, observed_at), (2,), '下视观测指标')
        if distance < 0:
            raise ValueError('中心距离不能为负')
        with self._lock:
            if (epoch is not None and epoch != self._down_target_epoch) or stamp <= self._down_target_after:
                return False
            previous = self._down_targets.get(kind)
            if previous is not None:
                better = distance < previous.center_distance-1e-9
                tied_newer = (abs(distance-previous.center_distance) <= 1e-9
                              and stamp > previous.observed_at)
                if not better and not tied_newer:
                    return False
            self._down_targets[kind] = DownTargetObservation(point, distance, stamp, camera)
            return True

    def down_target(self, kind):
        with self._lock:
            return self._down_targets.get(kind)

    def down_target_position(self, kind, configured):
        """Replace world XY only; configured Z remains a depth/sentinel value."""
        position = _finite_values(configured, (3,), '预设目标位置')
        observation = self.down_target(kind)
        return position if observation is None else (*observation.xy, position[2])

    def reset_down_targets(self, *, received_after):
        cutoff = _finite_values((received_after,), (1,), '观测清理时间')[0]
        with self._lock:
            self._down_targets.clear()
            self._down_target_epoch += 1
            self._down_target_after = cutoff
            # Keep source identities across setorigin: replaying an old camera
            # frame must not project it with the new origin's robot pose.

    def note_odom_generation(self, generation, received_at):
        with self._lock:
            if self._odom_generation is not None and generation != self._odom_generation:
                self.reset_down_targets(received_after=received_at)
            self._odom_generation = generation

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
        for key in ('frame_pose', 'golf_camera_pose', 'ring_camera_pose',
                    'yellow_camera_pose', 'red_camera_pose', 'last_servo_pose'):
            if key in values and values[key] is not None:
                values[key] = _finite_values(values[key], (4,), key)
        if 'first_frame_seen_at' in values and values['first_frame_seen_at'] is not None:
            stamp = values['first_frame_seen_at']
            if isinstance(stamp, bool) or not isinstance(stamp, (int, float)) or not math.isfinite(stamp):
                raise ValueError('first_frame_seen_at必须是有限单调时间')
        if 'depth' in values:
            values['depth'] = _finite_values((values['depth'], 0.0, 0.0), (3,), '抓取深度')[0]
        for key in ('golf_attempts', 'ring_attempts', 'yellow_attempts', 'red_attempts'):
            if key in values and (isinstance(values[key], bool) or not isinstance(values[key], int) or values[key] < 0):
                raise ValueError(key+'必须是非负整数')
        for key in ('golf_status', 'ring_status', 'yellow_status', 'red_status'):
            if key in values and values[key] not in ('pending', 'success', 'failed'):
                raise ValueError(key+'状态无效')
        with self._lock:
            self._pickup = replace(self._pickup, **values)
