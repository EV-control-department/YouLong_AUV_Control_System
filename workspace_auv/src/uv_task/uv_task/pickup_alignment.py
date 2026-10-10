"""Monotonic acquisition windows and geometry for combined pickup."""
from __future__ import annotations
from dataclasses import dataclass, field
import time
import numpy as np
from uv_task.down_camera_servo import best_detection, normalized_image_error, body_to_world_rotation


@dataclass
class PickupWindow:
    started: float
    search_seconds: float
    servo_seconds: float
    loss_seconds: float
    checking: bool = False
    seen: bool = False
    mode: str = 'frame'
    missing_since: float | None = None
    observations_valid: bool = True
    last_messages: dict = field(default_factory=dict)
    paused_seconds: float = 0.0
    paused_at: float | None = None

    def _pause_extension(self, now=None):
        if self.paused_at is None:
            return self.paused_seconds
        current = time.monotonic() if now is None else float(now)
        return self.paused_seconds + max(0.0, current-self.paused_at)

    def pause(self, now=None):
        if self.paused_at is None:
            self.paused_at = time.monotonic() if now is None else float(now)

    def resume(self, now=None):
        if self.paused_at is not None:
            current = time.monotonic() if now is None else float(now)
            self.paused_seconds += max(0.0, current-self.paused_at)
            self.paused_at = None

    @property
    def search_deadline(self):
        return self.started + self.search_seconds + self._pause_extension()

    def search_deadline_at(self, now):
        return self.started + self.search_seconds + self._pause_extension(now)

    @property
    def deadline(self):
        return self.started + self.servo_seconds + self._pause_extension()

    def deadline_at(self, now):
        return self.started + self.servo_seconds + self._pause_extension(now)

    def observe(self, received, camera, message, class_id, freshness):
        if received < self.started or received > self.search_deadline_at(received):
            return
        previous = self.last_messages.get(camera, self.started)
        if received-previous > freshness:
            self.observations_valid = False
        self.last_messages[camera] = received
        if best_detection(message, class_id) is not None:
            self.seen = True

    def absent_result(self, now, freshness):
        if self.seen or now < self.search_deadline_at(now):
            return None
        valid = self.observations_valid and all(
            camera in self.last_messages and
            self.search_deadline_at(now)-self.last_messages[camera] <= freshness
            for camera in ('down_left', 'down_right'))
        return 'absent' if self.checking and valid else 'unobserved'

    def transition(self, now, target_visible, frame_visible):
        previous = self.mode
        if target_visible and self.seen:
            self.mode = 'target'
            self.missing_since = None
        elif self.mode == 'target':
            if self.missing_since is None:
                self.missing_since = now
            if now-self.missing_since >= self.loss_seconds:
                self.mode = 'frame'
                self.missing_since = now
        elif frame_visible:
            self.mode = 'frame'
            self.missing_since = None
        else:
            if self.missing_since is None:
                self.missing_since = now
            if now-self.missing_since >= self.loss_seconds:
                self.mode = 'return'
        return previous != self.mode


def target_world_xy(node, camera_name, detection, pose, projection_depth):
    """Estimate an off-centre target on the existing fixed-depth servo plane.

    This is the configured pixel-to-metre scale, not a measured target range.
    A timeout must retain the image residual rather than pretend it is centred.
    """
    camera = node.camera_extrinsics[camera_name]
    du, dv = normalized_image_error(node, camera_name, detection)
    rotation = body_to_world_rotation(pose)
    ray = rotation @ camera.optical_to_body @ np.array([du, dv, 1.0])
    if not np.all(np.isfinite(ray)) or ray[2] <= 1e-6:
        raise ValueError('下视目标射线不能投影到水平面')
    origin = np.asarray(pose[:3]) + rotation @ np.asarray(camera.translation)
    point = origin + ray * (projection_depth / ray[2])
    return tuple(float(v) for v in point[:2])


def bounded_velocity(delta, gain, speed):
    velocity = np.asarray(delta, dtype=float)*gain
    if not np.all(np.isfinite(velocity)):
        raise ValueError('闭环位置误差无效')
    length = float(np.linalg.norm(velocity))
    if length > speed:
        velocity *= speed/length
    return velocity
