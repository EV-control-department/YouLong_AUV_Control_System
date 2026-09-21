"""Runtime camera mounting geometry from the vehicle TF tree.

Camera YAML intentionally contains only optical calibration. This module is
the single ROS-facing boundary for camera installation transforms used by
perception and task nodes.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np
try:
    from rclpy.duration import Duration
    from rclpy.time import Time
    import tf2_ros
except ModuleNotFoundError:  # Allows pure geometry tests outside a ROS install.
    Duration = None
    Time = None
    tf2_ros = None


CAMERA_FRAME_IDS = {
    "front_left": "front_left_camera_optical_frame",
    "front_right": "front_right_camera_optical_frame",
    "down_left": "downward_left_camera_optical_frame",
    "down_right": "downward_right_camera_optical_frame",
}


class CameraExtrinsicsUnavailable(RuntimeError):
    """Raised when the required camera TFs are not available yet."""


def quaternion_to_rotation(quaternion) -> np.ndarray:
    """Convert a geometry-msg quaternion [x, y, z, w] to SO(3)."""
    value = np.asarray(quaternion, dtype=np.float64).reshape(4)
    norm = float(np.linalg.norm(value))
    if not np.isfinite(norm) or norm < 1e-12:
        raise ValueError("camera TF quaternion must be finite and non-zero")
    x, y, z, w = value / norm
    rotation = np.array([
        [1.0 - 2.0 * (y * y + z * z),
         2.0 * (x * y - z * w),
         2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w),
         1.0 - 2.0 * (x * x + z * z),
         2.0 * (y * z - x * w)],
        [2.0 * (x * z + y * w),
         2.0 * (y * z - x * w),
         1.0 - 2.0 * (x * x + y * y)],
    ], dtype=np.float64)
    if not np.all(np.isfinite(rotation)):
        raise ValueError("camera TF rotation is non-finite")
    if not np.allclose(rotation @ rotation.T, np.eye(3), atol=2e-6):
        raise ValueError("camera TF rotation is not orthogonal")
    if not np.isclose(np.linalg.det(rotation), 1.0, atol=2e-6):
        raise ValueError("camera TF rotation determinant must be 1")
    return rotation


@dataclass(frozen=True)
class CameraExtrinsic:
    """Transform from one optical frame into the configured body frame."""

    name: str
    frame_id: str
    translation: np.ndarray
    optical_to_body: np.ndarray


class CameraExtrinsicsProvider:
    """Look up and validate all four camera optical-frame transforms."""

    def __init__(self, node, *, base_frame: str = "base_link",
                 timeout_sec: float = 5.0, retry_period_sec: float = 0.1,
                 buffer=None, listener=None):
        self.node = node
        self.base_frame = str(base_frame).strip() or "base_link"
        self.timeout_sec = float(timeout_sec)
        self.retry_period_sec = float(retry_period_sec)
        if self.timeout_sec < 0.0 or not math.isfinite(self.timeout_sec):
            raise ValueError(
                "camera_tf_timeout_sec must be finite and non-negative")
        if self.retry_period_sec <= 0.0 or not math.isfinite(self.retry_period_sec):
            raise ValueError(
                "camera_tf_retry_period_sec must be finite and positive")
        if buffer is None and tf2_ros is None:
            raise RuntimeError("tf2_ros is required for live camera TF lookup")
        self.buffer = buffer or tf2_ros.Buffer()
        self.listener = listener or tf2_ros.TransformListener(
            self.buffer, node, spin_thread=True)

    @staticmethod
    def _transform_to_extrinsic(name: str, frame_id: str,
                                transform) -> CameraExtrinsic:
        message = transform.transform if hasattr(transform, "transform") else transform
        translation = np.array([
            float(message.translation.x),
            float(message.translation.y),
            float(message.translation.z),
        ], dtype=np.float64)
        if not np.all(np.isfinite(translation)):
            raise ValueError(f"{name} camera TF translation is non-finite")
        if np.max(np.abs(translation)) > 100.0:
            raise ValueError(
                f"{name} camera TF translation appears invalid: {translation}")
        quaternion = [
            float(message.rotation.x), float(message.rotation.y),
            float(message.rotation.z), float(message.rotation.w),
        ]
        rotation = quaternion_to_rotation(quaternion)
        return CameraExtrinsic(name, frame_id, translation, rotation)

    def lookup_transform(self, target_frame: str, source_frame: str):
        """Look up target <- source at the latest available time."""
        try:
            if Time is None:
                stamp = None
                timeout = self.timeout_sec
            else:
                stamp = Time()
                timeout = Duration(seconds=self.timeout_sec)
            return self.buffer.lookup_transform(
                str(target_frame), str(source_frame), stamp,
                timeout=timeout)
        except Exception as error:
            raise CameraExtrinsicsUnavailable(
                f"TF not ready for {target_frame} <- {source_frame}: {error}") from error

    def snapshot(self) -> dict[str, CameraExtrinsic]:
        """Return a complete, validated snapshot or raise without partial data."""
        result = {}
        missing = []
        for name, frame_id in CAMERA_FRAME_IDS.items():
            try:
                transform = self.lookup_transform(self.base_frame, frame_id)
                result[name] = self._transform_to_extrinsic(
                    name, frame_id, transform)
            except CameraExtrinsicsUnavailable:
                missing.append(frame_id)
            except (TypeError, ValueError, AttributeError) as error:
                raise CameraExtrinsicsUnavailable(
                    f"invalid TF for {name} ({frame_id}): {error}") from error
        if missing:
            raise CameraExtrinsicsUnavailable(
                f"missing camera TF frames under {self.base_frame}: "
                f"{', '.join(missing)}")
        return result

    def try_snapshot(self, logger=None) -> dict[str, CameraExtrinsic] | None:
        """Return a complete snapshot, logging one concise retry warning."""
        try:
            return self.snapshot()
        except CameraExtrinsicsUnavailable as error:
            if logger is not None:
                logger.warn(str(error))
            return None


__all__ = [
    "CAMERA_FRAME_IDS",
    "CameraExtrinsic",
    "CameraExtrinsicsProvider",
    "CameraExtrinsicsUnavailable",
    "quaternion_to_rotation",
]