"""Independent front/down stereo static target localization node.

This node deliberately replaces the legacy monocular position implementation.
It consumes timestamped detection metadata, loads the stereo calibration profiles,
and fuses valid observations into independent front/down static 3-D estimates:

* FRONT_STEREO/FRONT_MULTI_VIEW: raw front-camera bearing factors.  Stereo is
  simply two bearings captured at the same time; it is not a second XYZ
  measurement in the fusion backend.
* DOWN_DIRECT: a down-camera known-height plane intersection (or optional
  stereo measurement for scenes without a target-height constraint).

The two camera pairs have separate observation pools.  Front bearings are
associated in a batch from pairwise geometric hypotheses and then fused by a
robust tangent-plane bearing estimator; the raw bearing is never assigned to
an existing front track on arrival.  Down estimates retain their known-height
pool and filter.  A front estimate never gates, reanchors, or updates a down
estimate, and vice versa.

The public compatibility output is ObjectPositionArray on /perception/objects.
TargetPositionArray additionally exposes covariance and observation provenance.
"""

from __future__ import annotations

import bisect
import json
import math
import os
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from types import SimpleNamespace
from typing import Callable

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo

from uv_msgs.msg import (
    Detection,
    DetectionArray,
    ObjectPosition,
    ObjectPositionArray,
    PoseInfo,
    TargetPosition,
    TargetPositionArray,
    TargetObservation,
    TargetObservationArray,
)


FORM_FRONT_STEREO = 1
FORM_FRONT_MULTI_VIEW = 2
FORM_DOWN_DIRECT = 4
UNASSIGNED_INSTANCE_ID = (1 << 32) - 1

DEFAULT_CLASS_NAMES = [
    "collection_frame_down",
    "collection_frame_front",
    "gate_down",
    "gate_front",
    "guide_line",
    "impact_ball_blue",
    "impact_ball_red",
    "pink_golf",
    "red_ring",
    "target_rack_down",
    "target_rack_front",
    "yellow_golf",
]

# Heights are scene depths in the project's NED convention, where positive Z
# is down from the water surface.  They are converted to the local odom frame
# with ``down_scene_origin_z_m`` before intersecting a camera ray.  In
# particular, the suspended impact balls are not on the pool floor, so they
# must not use the guide-line/floor height.
DEFAULT_DOWN_TARGET_Z = {
    "guide_line": 1.294,
    "target_rack": 1.00,
    "collection_frame": 0.94,
    # Objects placed on the target rack in the fixed simulator scene.
    "yellow_golf": 0.964,
    "pink_golf": 0.964,
    "red_ring": 0.925,
}


def _stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _finite_vector(value, size: int) -> np.ndarray | None:
    try:
        array = np.asarray(value, dtype=np.float64).reshape(-1)
    except (TypeError, ValueError):
        return None
    if array.size != size or not np.all(np.isfinite(array)):
        return None
    return array


def _rpy_to_rotation(roll_deg: float, pitch_deg: float,
                      yaw_deg: float) -> np.ndarray:
    """Return the ZYX rotation from body coordinates to odom/NED."""
    roll, pitch, yaw = map(math.radians, (roll_deg, pitch_deg, yaw_deg))
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ], dtype=np.float64)


def _rotation_to_quaternion(rotation: np.ndarray) -> np.ndarray:
    """Convert a proper rotation matrix to ``[w, x, y, z]``."""
    matrix = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = np.array([
            0.25 * scale,
            (matrix[2, 1] - matrix[1, 2]) / scale,
            (matrix[0, 2] - matrix[2, 0]) / scale,
            (matrix[1, 0] - matrix[0, 1]) / scale,
        ])
    else:
        diagonal = np.diag(matrix)
        index = int(np.argmax(diagonal))
        if index == 0:
            scale = math.sqrt(max(1.0 + matrix[0, 0]
                                  - matrix[1, 1] - matrix[2, 2], 1e-12)) * 2.0
            quaternion = np.array([
                (matrix[2, 1] - matrix[1, 2]) / scale,
                0.25 * scale,
                (matrix[0, 1] + matrix[1, 0]) / scale,
                (matrix[0, 2] + matrix[2, 0]) / scale,
            ])
        elif index == 1:
            scale = math.sqrt(max(1.0 + matrix[1, 1]
                                  - matrix[0, 0] - matrix[2, 2], 1e-12)) * 2.0
            quaternion = np.array([
                (matrix[0, 2] - matrix[2, 0]) / scale,
                (matrix[0, 1] + matrix[1, 0]) / scale,
                0.25 * scale,
                (matrix[1, 2] + matrix[2, 1]) / scale,
            ])
        else:
            scale = math.sqrt(max(1.0 + matrix[2, 2]
                                  - matrix[0, 0] - matrix[1, 1], 1e-12)) * 2.0
            quaternion = np.array([
                (matrix[1, 0] - matrix[0, 1]) / scale,
                (matrix[0, 2] + matrix[2, 0]) / scale,
                (matrix[1, 2] + matrix[2, 1]) / scale,
                0.25 * scale,
            ])
    norm = float(np.linalg.norm(quaternion))
    if not np.isfinite(norm) or norm < 1e-12:
        raise ValueError("rotation cannot be converted to quaternion")
    return quaternion / norm


def _quaternion_to_rotation(quaternion: np.ndarray) -> np.ndarray:
    """Convert ``[w, x, y, z]`` into a rotation matrix."""
    value = np.asarray(quaternion, dtype=np.float64).reshape(4)
    norm = float(np.linalg.norm(value))
    if not np.isfinite(norm) or norm < 1e-12:
        raise ValueError("quaternion must be finite and non-zero")
    w, x, y, z = value / norm
    return np.array([
        [1.0 - 2.0 * (y * y + z * z),
         2.0 * (x * y - z * w),
         2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w),
         1.0 - 2.0 * (x * x + z * z),
         2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w),
         2.0 * (y * z + x * w),
         1.0 - 2.0 * (x * x + y * y)],
    ], dtype=np.float64)


def _slerp_rotation(first: np.ndarray, second: np.ndarray,
                    alpha: float) -> np.ndarray:
    """Interpolate two rotations along the shortest SO(3) path."""
    q0 = _rotation_to_quaternion(first)
    q1 = _rotation_to_quaternion(second)
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    dot = float(np.clip(dot, -1.0, 1.0))
    if dot > 0.9995:
        quaternion = q0 + float(alpha) * (q1 - q0)
    else:
        angle = math.acos(dot)
        sine = math.sin(angle)
        first_weight = math.sin((1.0 - float(alpha)) * angle) / sine
        second_weight = math.sin(float(alpha) * angle) / sine
        quaternion = first_weight * q0 + second_weight * q1
    return _quaternion_to_rotation(quaternion)


def _axis_angle_rotation(axis_angle: np.ndarray) -> np.ndarray:
    """Small SO(3) rotation used for numerical extrinsic Jacobians."""
    v = np.asarray(axis_angle, dtype=np.float64).reshape(3)
    theta = float(np.linalg.norm(v))
    if theta < 1e-12:
        return np.eye(3) + np.array([
            [0.0, -v[2], v[1]],
            [v[2], 0.0, -v[0]],
            [-v[1], v[0], 0.0],
        ])
    axis = v / theta
    x, y, z = axis
    c, s = math.cos(theta), math.sin(theta)
    one = 1.0 - c
    return np.array([
        [c + x * x * one, x * y * one - z * s, x * z * one + y * s],
        [y * x * one + z * s, c + y * y * one, y * z * one - x * s],
        [z * x * one - y * s, z * y * one + x * s, c + z * z * one],
    ], dtype=np.float64)


def _regularize_covariance(covariance: np.ndarray,
                           minimum_variance: float = 1e-8) -> np.ndarray:
    cov = np.asarray(covariance, dtype=np.float64)
    cov = 0.5 * (cov + cov.T)
    if cov.ndim != 2 or cov.shape[0] != cov.shape[1]:
        raise ValueError("covariance must be square")
    if not np.all(np.isfinite(cov)):
        return np.eye(cov.shape[0], dtype=np.float64) * 1e6
    try:
        eigenvalues, eigenvectors = np.linalg.eigh(cov)
        eigenvalues = np.maximum(eigenvalues, minimum_variance)
        cov = (eigenvectors * eigenvalues) @ eigenvectors.T
    except np.linalg.LinAlgError:
        cov = np.eye(cov.shape[0], dtype=np.float64) * 1e6
    return 0.5 * (cov + cov.T)


def _safe_inverse(matrix: np.ndarray) -> np.ndarray:
    try:
        return np.linalg.inv(matrix)
    except np.linalg.LinAlgError:
        return np.linalg.pinv(matrix)


def _numeric_jacobian(function: Callable[[np.ndarray], np.ndarray],
                      value: np.ndarray,
                      steps: np.ndarray) -> np.ndarray:
    value = np.asarray(value, dtype=np.float64).reshape(-1)
    base = np.asarray(function(value), dtype=np.float64).reshape(-1)
    jacobian = np.zeros((base.size, value.size), dtype=np.float64)
    for index, step in enumerate(np.asarray(steps, dtype=np.float64)):
        plus = value.copy()
        minus = value.copy()
        plus[index] += step
        minus[index] -= step
        jacobian[:, index] = (
            np.asarray(function(plus), dtype=np.float64).reshape(-1)
            - np.asarray(function(minus), dtype=np.float64).reshape(-1)
        ) / (2.0 * step)
    return jacobian


@dataclass
class PoseSample:
    stamp: float
    position: np.ndarray
    roll_deg: float
    pitch_deg: float
    yaw_deg: float


@dataclass
class PoseAt:
    stamp: float
    position: np.ndarray
    rotation: np.ndarray
    covariance: np.ndarray
    age_sec: float
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0


@dataclass
class FrontPixelObservation:
    """One raw front image-feature bearing used by the unified solver."""

    stamp: float
    camera: str
    pixel: np.ndarray
    covariance: np.ndarray
    pose: PoseAt
    confidence: float
    # Monotonic identity for one detector output.  Derived stereo/multi-view
    # points may be replaced, but this source pixel must never be counted
    # twice by the reprojection backend.
    raw_observation_id: int = 0
    # The feature is named rather than silently treated as a physical
    # corner/edge.  Old detections use the bbox center as the fallback.
    feature_id: str = "bbox_center"
    # Keep the calibration object used at capture time.  CameraInfo can be
    # refreshed in a running node; old pixels must not be reprojected with a
    # newer calibration profile.
    calibration: StereoCalibration | None = None
    target_instance_id: int = UNASSIGNED_INSTANCE_ID


@dataclass
class RayObservation:
    stamp: float
    origin: np.ndarray
    direction: np.ndarray
    sigma_angle: float
    confidence: float
    # Monotonic identity for duplicate-evidence accounting.  A ray can be
    # retained in a track while its derived multi-view point is replaced.
    ray_id: int = 0
    raw_observation: FrontPixelObservation | None = None
    # The ray is a first-class observation.  It is deliberately not assigned
    # to a TargetTrack when it enters the pool; cluster_memberships are filled
    # only after the class-wide batch association pass.
    class_id: int = -1
    camera: str = ""
    bearing_camera: np.ndarray | None = None
    bearing_covariance: np.ndarray | None = None
    observation_form: int = FORM_FRONT_MULTI_VIEW
    cluster_memberships: dict[int, float] = field(default_factory=dict)


# Public name used by the unified bearing design.  Keep ``RayObservation`` as
# a compatibility alias for existing tests and offline tools.
BearingObservation = RayObservation


@dataclass
class DownDirectObservation:
    """One accepted raw DOWN_DIRECT factor in a track-local sliding window."""

    stamp: float
    position: np.ndarray
    covariance: np.ndarray
    confidence: float
    class_id: int = -1
    # Pose/extrinsic/known-height errors can be shared by many frames and
    # must not disappear merely because the sliding window grows.
    shared_covariance: np.ndarray | None = None


@dataclass
class FrontPositionObservation:
    """One front stereo or multi-view 3-D observation in the front pool."""

    stamp: float
    position: np.ndarray
    covariance: np.ndarray
    confidence: float
    form: int
    class_id: int = -1
    # A multi-view pool slot is keyed by the ray track, not by every repeated
    # solve over the same retained rays.  Direct stereo observations keep 0.
    pool_key: int = 0
    ray_ids: tuple[int, ...] = ()
    observation_record_id: int = 0
    # Common-mode geometry uncertainty retained separately from per-frame
    # pixel/triangulation noise.
    shared_covariance: np.ndarray | None = None
    # Underlying front image features used by the unified front solver.
    raw_observations: tuple[FrontPixelObservation, ...] = ()


@dataclass
class TargetTrack:
    """One physical static target, possibly seen by two detector labels."""

    class_id: int
    instance_id: int
    physical_class_name: str
    observed_class_ids: set[int] = field(default_factory=set)
    position: np.ndarray | None = None
    covariance: np.ndarray | None = None
    rays: deque[RayObservation] = field(default_factory=deque)
    down_observations: deque[DownDirectObservation] = field(default_factory=deque)
    front_observations: deque[FrontPositionObservation] = field(
        default_factory=deque)
    down_filter_position: np.ndarray | None = None
    down_filter_covariance: np.ndarray | None = None
    front_filter_position: np.ndarray | None = None
    front_filter_covariance: np.ndarray | None = None
    last_stamp: float = 0.0
    last_confidence: float = 0.0
    observation_count: int = 0
    front_stereo_count: int = 0
    front_multi_view_count: int = 0
    down_direct_count: int = 0
    observation_form_mask: int = 0
    last_observation_form: int = 0
    multi_view_pool_key: int = 0
    # A conservative floor for errors shared by the retained observations.
    # It prevents a long window from claiming centimetre-level certainty when
    # the pose, mounting transform, or target-height reference is uncertain.
    systematic_covariance: np.ndarray | None = None
    front_pixel_observations: deque[FrontPixelObservation] = field(
        default_factory=deque)
    # Diagnostics for the batch bearing estimator.  ``observation_count`` is
    # the raw member count kept for message compatibility; effective count
    # reflects soft cluster membership.
    front_effective_observations: float = 0.0
    front_inlier_observations: int = 0
    front_mean_residual: float = float("inf")
    front_information_eigenvalues: np.ndarray | None = None
    front_covariance_eigenvalues: np.ndarray | None = None
    front_condition_number: float = float("inf")
    last_update_monotonic: float = field(default_factory=time.monotonic)


@dataclass
class ObservationRecord:
    """One accepted geometric factor, retained for diagnostics and the GUI."""

    observation_id: int
    stamp: float
    class_id: int
    instance_id: int
    physical_class_name: str
    form: int
    confidence: float
    position: np.ndarray | None = None
    covariance: np.ndarray | None = None
    ray_origin: np.ndarray | None = None
    ray_direction: np.ndarray | None = None
    source: str = "down"
    source_raw_observation_ids: tuple[int, ...] = ()
    feature_id: str = ""


@dataclass
class StereoCalibration:
    name: str
    path: str
    camera_matrix_left: np.ndarray
    camera_matrix_right: np.ndarray
    dist_left: np.ndarray
    dist_right: np.ndarray
    rotation: np.ndarray
    translation: np.ndarray
    projection_left: np.ndarray
    projection_right: np.ndarray
    rectification_left: np.ndarray
    rectification_right: np.ndarray
    reprojection: np.ndarray
    baseline_m: float

    @classmethod
    def load(cls, name: str, path: str) -> "StereoCalibration":
        required = (
            "camera_matrix_left", "camera_matrix_right",
            "dist_coeffs_left", "dist_coeffs_right",
            "R", "T", "P1", "P2", "R1", "R2", "Q",
        )
        with np.load(path, allow_pickle=False) as archive:
            missing = [key for key in required if key not in archive.files]
            if missing:
                raise ValueError(f"missing arrays: {', '.join(missing)}")
            arrays = {
                key: np.asarray(archive[key], dtype=np.float64)
                for key in required
            }

        def require_shape(key: str, shape: tuple[int, ...]) -> np.ndarray:
            value = arrays[key]
            if value.shape != shape or not np.all(np.isfinite(value)):
                raise ValueError(
                    f"{key} has shape {value.shape}, expected {shape}")
            return value

        k_left = require_shape("camera_matrix_left", (3, 3))
        k_right = require_shape("camera_matrix_right", (3, 3))
        rotation = require_shape("R", (3, 3))
        projection_left = require_shape("P1", (3, 4))
        projection_right = require_shape("P2", (3, 4))
        rectification_left = require_shape("R1", (3, 3))
        rectification_right = require_shape("R2", (3, 3))
        reprojection = require_shape("Q", (4, 4))
        translation = arrays["T"].reshape(-1)
        if translation.size != 3 or not np.all(np.isfinite(translation)):
            raise ValueError(f"T has shape {arrays['T'].shape}, expected 3 values")

        dist_left = arrays["dist_coeffs_left"].reshape(-1)
        dist_right = arrays["dist_coeffs_right"].reshape(-1)
        if dist_left.size not in (4, 5, 8, 12, 14):
            raise ValueError("unsupported left distortion coefficient count")
        if dist_right.size not in (4, 5, 8, 12, 14):
            raise ValueError("unsupported right distortion coefficient count")

        fx = float(projection_right[0, 0])
        if abs(fx) < 1e-9:
            raise ValueError("P2 has zero focal length")
        baseline = abs(float(projection_right[0, 3] / fx))
        if not np.isfinite(baseline) or baseline <= 1e-6:
            raise ValueError("P2 does not contain a usable stereo baseline")

        return cls(
            name=name,
            path=path,
            camera_matrix_left=k_left,
            camera_matrix_right=k_right,
            dist_left=dist_left,
            dist_right=dist_right,
            rotation=rotation,
            translation=translation,
            projection_left=projection_left,
            projection_right=projection_right,
            rectification_left=rectification_left,
            rectification_right=rectification_right,
            reprojection=reprojection,
            baseline_m=baseline,
        )

    @classmethod
    def from_camera_info(cls, name: str, left_info: CameraInfo,
                         right_info: CameraInfo,
                         left_translation: np.ndarray,
                         left_body_rotation: np.ndarray,
                         right_translation: np.ndarray,
                         right_body_rotation: np.ndarray) -> "StereoCalibration":
        """Build a stereo profile from the cameras currently used by Stonefish.

        Stonefish publishes one ``CameraInfo`` per rendered camera.  Its P
        matrix intentionally has no stereo baseline, so the relative pose is
        derived from the two camera poses in the active scenario and OpenCV
        computes the rectification/projection matrices used below.
        """
        width, height = int(left_info.width), int(left_info.height)
        if width <= 1 or height <= 1:
            raise ValueError("left CameraInfo has an invalid image size")
        if (int(right_info.width), int(right_info.height)) != (width, height):
            raise ValueError(
                "left/right CameraInfo image sizes differ: "
                f"{width}x{height} vs {right_info.width}x{right_info.height}")

        def camera_matrix(message: CameraInfo, side: str) -> np.ndarray:
            matrix = np.asarray(message.k, dtype=np.float64).reshape(3, 3)
            if not np.all(np.isfinite(matrix)) or matrix[0, 0] <= 1e-6 \
                    or matrix[1, 1] <= 1e-6 or matrix[2, 2] <= 1e-6:
                raise ValueError(f"{side} CameraInfo has an invalid K matrix")
            return matrix

        def distortion(message: CameraInfo, side: str) -> np.ndarray:
            coefficients = np.asarray(message.d, dtype=np.float64).reshape(-1)
            # Empty D is valid ROS CameraInfo shorthand for an ideal camera.
            if coefficients.size == 0:
                coefficients = np.zeros(5, dtype=np.float64)
            if coefficients.size not in (4, 5, 8, 12, 14) \
                    or not np.all(np.isfinite(coefficients)):
                raise ValueError(
                    f"{side} CameraInfo has unsupported distortion coefficients")
            return coefficients

        k_left = camera_matrix(left_info, "left")
        k_right = camera_matrix(right_info, "right")
        d_left = distortion(left_info, "left")
        d_right = distortion(right_info, "right")
        r_body_left = np.asarray(left_body_rotation, dtype=np.float64).reshape(3, 3)
        r_body_right = np.asarray(right_body_rotation, dtype=np.float64).reshape(3, 3)
        t_body_left = np.asarray(left_translation, dtype=np.float64).reshape(3)
        t_body_right = np.asarray(right_translation, dtype=np.float64).reshape(3)

        # x_right = R_right_left x_left + T_right_left.
        rotation = r_body_right.T @ r_body_left
        translation = r_body_right.T @ (t_body_left - t_body_right)
        baseline = float(np.linalg.norm(translation))
        if not np.isfinite(baseline) or baseline <= 1e-6:
            raise ValueError("CameraInfo stereo profile has a zero baseline")

        rectification_left, rectification_right, projection_left, \
            projection_right, reprojection, _, _ = cv2.stereoRectify(
                k_left, d_left, k_right, d_right, (width, height),
                rotation, translation, flags=cv2.CALIB_ZERO_DISPARITY,
                alpha=0.0,
            )
        arrays = (
            rectification_left, rectification_right, projection_left,
            projection_right, reprojection,
        )
        if not all(np.all(np.isfinite(array)) for array in arrays):
            raise ValueError("CameraInfo stereo rectification contains non-finite values")

        return cls(
            name=name,
            path=(f"CameraInfo:{left_info.header.frame_id}|"
                  f"{right_info.header.frame_id}"),
            camera_matrix_left=k_left,
            camera_matrix_right=k_right,
            dist_left=d_left,
            dist_right=d_right,
            rotation=rotation,
            translation=translation,
            projection_left=projection_left,
            projection_right=projection_right,
            rectification_left=rectification_left,
            rectification_right=rectification_right,
            reprojection=reprojection,
            baseline_m=baseline,
        )

    def rectified_pixel(self, side: str, pixel: np.ndarray) -> np.ndarray:
        pixel = np.asarray(pixel, dtype=np.float64).reshape(2)
        if side == "left":
            k, d, r, p = (
                self.camera_matrix_left, self.dist_left,
                self.rectification_left, self.projection_left,
            )
        else:
            k, d, r, p = (
                self.camera_matrix_right, self.dist_right,
                self.rectification_right, self.projection_right,
            )
        result = cv2.undistortPoints(
            pixel.reshape(1, 1, 2), k, d, R=r, P=p[:, :3])
        return result.reshape(2)

    def ray_in_left_optical(self, side: str, pixel: np.ndarray) -> np.ndarray:
        rectified = self.rectified_pixel(side, pixel)
        if side == "left":
            projection = self.projection_left
            rectification = self.rectification_left
        else:
            projection = self.projection_right
            rectification = self.rectification_right
        ray_rectified = np.array([
            (rectified[0] - projection[0, 2]) / projection[0, 0],
            (rectified[1] - projection[1, 2]) / projection[1, 1],
            1.0,
        ], dtype=np.float64)
        # R1/R2 map original optical coordinates to rectified coordinates.
        ray_optical = rectification.T @ ray_rectified
        return ray_optical / max(np.linalg.norm(ray_optical), 1e-12)

    def triangulate_left_rectified(self, left_pixel: np.ndarray,
                                   right_pixel: np.ndarray) -> np.ndarray:
        left_rectified = self.rectified_pixel("left", left_pixel)
        right_rectified = self.rectified_pixel("right", right_pixel)
        homogeneous = cv2.triangulatePoints(
            self.projection_left,
            self.projection_right,
            left_rectified.reshape(2, 1),
            right_rectified.reshape(2, 1),
        ).reshape(4)
        if abs(float(homogeneous[3])) < 1e-12:
            raise ValueError("triangulation produced a point at infinity")
        point = homogeneous[:3] / homogeneous[3]
        if not np.all(np.isfinite(point)):
            raise ValueError("triangulation produced non-finite point")
        return point


class ObjectLocalizer(Node):
    """Static target localizer with front stereo and down height geometry."""

    def __init__(self):
        super().__init__("object_localizer")
        self._declare_parameters()
        self._read_parameters()

        self._pose_buffer: deque[PoseSample] = deque(maxlen=300)
        self._pending = {
            "front_left": deque(),
            "front_right": deque(),
            "down_left": deque(),
            "down_right": deque(),
        }
        # Downward estimates.  Front and down deliberately have separate
        # namespaces: a same-class front estimate must not consume or move a
        # down estimate.
        self._tracks: dict[tuple[str, int], TargetTrack] = {}
        self._next_instance_id: dict[str, int] = {}
        self._front_tracks: dict[tuple[str, int], TargetTrack] = {}
        self._front_next_instance_id: dict[str, int] = {}
        # Large class-separated pools of geometrically valid observations.
        # Down keeps its known-height/direct measurement pool.  Front keeps a
        # separate raw bearing pool: rays are not assigned to a target track
        # on arrival.  The pool is reclustered in batch and each cluster is
        # fitted by the bearing estimator below.
        self._down_observation_pool: dict[
            str, deque[DownDirectObservation]] = {}
        self._front_observation_pool: dict[
            str, deque[FrontPositionObservation]] = {}
        self._front_bearing_pool: dict[str, deque[RayObservation]] = {}
        self._front_bearing_dirty_classes: set[str] = set()
        self._next_ray_id = 1
        self._next_raw_observation_id = 1
        self._next_multiview_pool_key = 1
        self._warned: set[str] = set()
        self._counters = {
            "front_stereo_accepted": 0,
            "front_stereo_rejected": 0,
            "front_stereo_range_rejected": 0,
            "front_pair_invalid": 0,
            "front_aspect_match_rejected": 0,
            "front_epipolar_match_rejected": 0,
            "front_bbox_match_rejected": 0,
            "front_multi_view_rays": 0,
            "front_multi_view_points": 0,
            "front_multi_view_range_rejected": 0,
            "front_pool_observations": 0,
            "front_cluster_updates": 0,
            "front_unified_optimizations": 0,
            "front_track_first_associations": 0,
            "front_cluster_support_rejected": 0,
            "front_duplicate_merged": 0,
            "front_multi_view_angle_rejected": 0,
            "front_bearing_pool_added": 0,
            "front_bearing_seed_candidates": 0,
            "front_bearing_clusters": 0,
            "front_bearing_optimizations": 0,
            "front_bearing_soft_reassignments": 0,
            "front_bearing_noise_observations": 0,
            "front_bearing_rank_deficient": 0,
            "down_direct_accepted": 0,
            "down_direct_rejected": 0,
            "down_direct_reanchored": 0,
            "down_direct_queue_rejected": 0,
            "down_duplicate_merged": 0,
            "association_rejected": 0,
            "instance_limit_rejected": 0,
            "camera_label_rejected": 0,
        }
        self._observation_history: deque[ObservationRecord] = deque(
            maxlen=self.observation_history_size)
        self._next_observation_id = 1
        self._last_detection_stamp = 0.0
        self._last_summary_monotonic = time.monotonic()

        self._front_calibration: StereoCalibration | None = None
        self._down_calibration: StereoCalibration | None = None
        self._camera_info_messages: dict[str, CameraInfo | None] = {
            "front_left": None,
            "front_right": None,
            "down_left": None,
            "down_right": None,
        }
        self._camera_info_signatures: dict[str, tuple | None] = {
            camera: None for camera in self._camera_info_messages
        }
        self._initialise_calibrations()

        self._create_subscriptions()
        self._pub_compat = self.create_publisher(
            ObjectPositionArray, "/perception/objects", 10)
        self._pub_targets = self.create_publisher(
            TargetPositionArray, "/perception/target_positions", 10)
        self._pub_observations = self.create_publisher(
            TargetObservationArray, "/perception/target_observations", 10)
        self.create_timer(self.publish_period, self._publish)
        self.create_timer(0.05, self._flush_stale_pending)
        self.create_timer(self.front_bearing_rebuild_period,
                         self._rebuild_dirty_front_bearings)
        self.create_timer(5.0, self._summary)

        if self._calibration_ready or self._front_calibration_ready:
            self.get_logger().info(
                "object_localizer started with independent front/down stereo "
                "estimation")
        elif self.calibration_source == "sim_camera_info":
            self.get_logger().info(
                "object_localizer waiting for Stonefish CameraInfo calibration")
        else:
            self.get_logger().error(
                "object_localizer has invalid calibration; positions disabled")

    def _declare_parameters(self):
        self.declare_parameter("calibration_source", "npz")
        self.declare_parameter("front_calibration_file", "")
        self.declare_parameter("down_calibration_file", "")
        self.declare_parameter(
            "front_left_camera_info_topic",
            "/sim/front_cam/left/camera_info")
        self.declare_parameter(
            "front_right_camera_info_topic",
            "/sim/front_cam/right/camera_info")
        self.declare_parameter(
            "down_left_camera_info_topic",
            "/sim/down_cam/left/camera_info")
        self.declare_parameter(
            "down_right_camera_info_topic",
            "/sim/down_cam/right/camera_info")
        self.declare_parameter("front_image_width", 1280)
        self.declare_parameter("front_image_height", 960)
        self.declare_parameter("down_image_width", 1280)
        self.declare_parameter("down_image_height", 960)

        self.declare_parameter("stereo_sync_slop_sec", 0.04)
        self.declare_parameter("stereo_pending_timeout_sec", 0.15)
        self.declare_parameter("pose_max_age_sec", 0.08)
        self.declare_parameter("front_edge_margin_px", 8.0)
        self.declare_parameter("front_edge_margin_ratio", 0.02)
        # A bbox touching the image border is not a trustworthy geometric
        # center.  It may still be useful as a bearing while the vehicle
        # moves, so edge rays are retained with inflated angular noise.
        self.declare_parameter("front_edge_ray_noise_scale", 3.0)
        # Compare the shape of the two detection boxes, rather than assuming
        # that their absolute widths/heights scale identically.
        self.declare_parameter("front_bbox_aspect_ratio_max", 1.6)
        # Kept for launch-file compatibility; front stereo admission no longer
        # gates independently on left/right width and height ratios.
        self.declare_parameter("front_bbox_width_ratio_max", 1.6)
        self.declare_parameter("front_bbox_height_ratio_max", 1.6)
        self.declare_parameter("front_bbox_area_ratio_max", 2.2)
        self.declare_parameter("front_epipolar_error_px", 8.0)
        self.declare_parameter("min_disparity_px", 2.0)
        self.declare_parameter("min_depth_m", 0.05)
        # The fixed-scene targets are close to the vehicle.  Reject direct
        # down-camera intersections/triangulations beyond 2 m.
        self.declare_parameter("max_depth_m", 2.0)
        self.declare_parameter("use_rejected_front_pairs_for_multiview", True)

        # ``known_height`` is the preferred down-camera mode for the fixed
        # competition fixtures.  ``stereo`` remains available for scenes that
        # do not provide a reliable target height.  The old boolean is kept as
        # a compatibility alias: setting it true selects the legacy arbitrary
        # plane mode.
        self.declare_parameter("down_geometry_mode", "known_height")
        self.declare_parameter("down_plane_enabled", False)
        self.declare_parameter("down_plane_normal", [0.0, 0.0, 1.0])
        self.declare_parameter("down_plane_c", 0.0)
        self.declare_parameter("down_plane_sigma_m", 0.03)
        self.declare_parameter("down_plane_consistency_m", 0.12)
        self.declare_parameter("down_default_target_z_m", 1.294)
        # The simulator publishes robot_z relative to the spawn pose.  Real
        # vehicles that publish absolute depth should leave this at zero.
        self.declare_parameter("down_scene_origin_z_m", 0.0)
        self.declare_parameter("down_target_z_sigma_m", 0.03)
        self.declare_parameter("down_target_z_json", "")
        self.declare_parameter("down_ignored_classes", ["gate"])
        self.declare_parameter("down_min_plane_incidence", 0.15)
        self.declare_parameter("down_observation_pool_size", 300)
        self.declare_parameter("down_direct_queue_size", 50)
        self.declare_parameter("down_direct_queue_gate_chi2", 16.0)
        self.declare_parameter("guide_line_min_spacing_m", 0.5)
        self.declare_parameter("down_duplicate_merge_distance_m", 0.25)

        self.declare_parameter("pixel_sigma_fraction", 0.08)
        self.declare_parameter("pixel_sigma_min_px", 1.0)
        self.declare_parameter("pixel_sigma_max_px", 12.0)
        self.declare_parameter("calibration_pixel_sigma_px", 0.5)
        self.declare_parameter("pose_position_sigma_m", 0.03)
        self.declare_parameter("pose_angle_sigma_deg", 1.0)
        self.declare_parameter("extrinsic_position_sigma_m", 0.005)
        self.declare_parameter("extrinsic_angle_sigma_deg", 0.5)
        # Common-mode geometry errors are not independent from frame to
        # frame.  Keep their contribution as a non-averaging covariance term.
        self.declare_parameter("shared_error_scale", 1.0)
        # Simulator stereo depth is more reliable mainly in this working
        # range.  Outside it, keep the 3-D estimate but inflate its
        # covariance so it has less influence on fusion and clustering.
        self.declare_parameter("front_stereo_noise_scale", 1.8)
        self.declare_parameter("front_stereo_trusted_min_range_m", 0.5)
        self.declare_parameter("front_stereo_trusted_max_range_m", 2.5)
        self.declare_parameter("front_stereo_out_of_range_noise_scale", 6.0)
        # Deprecated compatibility switches.  Front stereo is no longer
        # hard-filtered or converted to a ray solely because of its range.
        self.declare_parameter("front_stereo_trusted_range_only", False)
        self.declare_parameter("front_stereo_out_of_range_as_ray", False)
        self.declare_parameter("front_multi_view_noise_scale", 1.5)
        self.declare_parameter("down_direct_noise_scale", 0.75)

        self.declare_parameter("front_multi_view_max_rays", 20)
        self.declare_parameter("front_multi_view_min_angle_deg", 5.0)
        self.declare_parameter("front_multi_view_max_line_error_m", 1.0)
        # Front bearings are retained in a class-wide pool and associated in
        # batches.  These parameters control candidate generation/clustering,
        # not admission of a raw ray into the pool.
        self.declare_parameter("front_bearing_seed_max_rays", 80)
        self.declare_parameter("front_bearing_seed_max_pairs", 2400)
        self.declare_parameter("front_bearing_seed_cluster_max_candidates", 600)
        self.declare_parameter("front_bearing_cluster_radius_m", 0.35)
        self.declare_parameter("front_bearing_min_cluster_rays", 2)
        self.declare_parameter("front_bearing_max_clusters", 8)
        self.declare_parameter("front_bearing_clutter_likelihood", 0.08)
        self.declare_parameter("front_bearing_lm_iterations", 10)
        self.declare_parameter("front_bearing_lm_initial_damping", 1e-3)
        self.declare_parameter("front_bearing_track_match_distance_m", 2.0)
        self.declare_parameter("front_bearing_rebuild_period_sec", 0.50)
        # The first implementation assumes pose and camera extrinsics are
        # exact as requested by the bearing model.  It can be enabled later
        # after the estimator itself is validated.
        self.declare_parameter("front_bearing_include_geometry_uncertainty", False)
        self.declare_parameter("front_observation_pool_size", 300)
        self.declare_parameter("front_direct_queue_size", 50)
        self.declare_parameter("front_raw_observation_window_size", 100)
        self.declare_parameter("front_duplicate_merge_distance_m", 0.25)
        self.declare_parameter("front_gate_duplicate_merge_distance_m", 0.50)
        self.declare_parameter("front_gate_min_cluster_observations", 3)
        self.declare_parameter("front_gate_min_publish_confidence", 0.05)
        self.declare_parameter("front_gate_stable_covariance_trace_m2", 0.25)
        self.declare_parameter("front_gate_model_sigma_m", 0.12)
        self.declare_parameter("front_ray_model_sigma_m", 0.05)
        self.declare_parameter("front_min_publish_confidence", 0.15)
        self.declare_parameter("ray_association_angle_deg", 25.0)
        self.declare_parameter("gate_ray_association_angle_deg", 32.0)
        self.declare_parameter("front_gate_reassociation_distance_m", 1.25)
        self.declare_parameter("ray_association_distance_m", 1.0)
        self.declare_parameter("position_gate_chi2", 16.0)
        self.declare_parameter("down_direct_reanchor_chi2", 9.0)
        self.declare_parameter("ray_gate_chi2", 16.0)
        self.declare_parameter("huber_delta", 2.5)
        self.declare_parameter("track_timeout_sec", 2.0)
        # A gate can leave the narrow front FOV while the vehicle continues
        # its scan. Keep its last world estimate available during that gap;
        # it is still labelled STALE after this timeout.
        self.declare_parameter("front_gate_track_timeout_sec", 10.0)
        self.declare_parameter("stable_covariance_trace_m2", 0.04)
        self.declare_parameter("minimum_stable_observations", 2)
        self.declare_parameter("max_instances_default", 1)
        self.declare_parameter("max_instances_guide_line", 6)
        self.declare_parameter("max_instances_gate", 4)
        self.declare_parameter("publish_period_sec", 0.1)
        self.declare_parameter("observation_history_size", 500)
        self.declare_parameter("class_names", DEFAULT_CLASS_NAMES)

        self.declare_parameter(
            "front_left_translation", [0.23, -0.05, 0.076])
        self.declare_parameter(
            "front_right_translation", [0.23, 0.05, 0.076])
        self.declare_parameter(
            "down_left_translation", [-0.13, -0.05, 0.0645])
        self.declare_parameter(
            "down_right_translation", [-0.13, 0.05, 0.0645])
        self.declare_parameter(
            "front_left_rotation", [0.0, 0.0, 1.0,
                                    -1.0, 0.0, 0.0,
                                    0.0, -1.0, 0.0])
        self.declare_parameter(
            "front_right_rotation", [0.0, 0.0, 1.0,
                                     -1.0, 0.0, 0.0,
                                     0.0, -1.0, 0.0])
        self.declare_parameter(
            "down_left_rotation", [0.0, -1.0, 0.0,
                                   1.0, 0.0, 0.0,
                                   0.0, 0.0, 1.0])
        self.declare_parameter(
            "down_right_rotation", [0.0, -1.0, 0.0,
                                    1.0, 0.0, 0.0,
                                    0.0, 0.0, 1.0])

    def _read_parameters(self):
        get = self.get_parameter
        self.calibration_source = str(
            get("calibration_source").value).strip().lower()
        if self.calibration_source not in {"npz", "sim_camera_info"}:
            raise ValueError(
                "calibration_source must be 'npz' or 'sim_camera_info'")
        self.front_calibration_file = str(get("front_calibration_file").value)
        self.down_calibration_file = str(get("down_calibration_file").value)
        self.camera_info_topics = {
            "front_left": str(get("front_left_camera_info_topic").value),
            "front_right": str(get("front_right_camera_info_topic").value),
            "down_left": str(get("down_left_camera_info_topic").value),
            "down_right": str(get("down_right_camera_info_topic").value),
        }
        self.front_width = int(get("front_image_width").value)
        self.front_height = int(get("front_image_height").value)
        self.down_width = int(get("down_image_width").value)
        self.down_height = int(get("down_image_height").value)

        self.stereo_sync_slop = float(get("stereo_sync_slop_sec").value)
        self.pending_timeout = float(get("stereo_pending_timeout_sec").value)
        self.pose_max_age_sec = float(get("pose_max_age_sec").value)
        self.edge_margin_px = float(get("front_edge_margin_px").value)
        self.edge_margin_ratio = float(get("front_edge_margin_ratio").value)
        self.front_edge_ray_noise_scale = max(
            1.0, float(get("front_edge_ray_noise_scale").value))
        self.bbox_aspect_ratio_max = float(
            get("front_bbox_aspect_ratio_max").value)
        # Legacy parameters are intentionally read for compatibility with
        # existing parameter files, but are not used for admission decisions.
        self.bbox_width_ratio_max = float(get("front_bbox_width_ratio_max").value)
        self.bbox_height_ratio_max = float(get("front_bbox_height_ratio_max").value)
        self.bbox_area_ratio_max = float(get("front_bbox_area_ratio_max").value)
        self.epipolar_error_max = float(get("front_epipolar_error_px").value)
        self.min_disparity = float(get("min_disparity_px").value)
        self.min_depth = float(get("min_depth_m").value)
        self.max_depth = float(get("max_depth_m").value)
        self.use_rejected_front_pairs_for_multiview = bool(
            get("use_rejected_front_pairs_for_multiview").value)

        self.down_geometry_mode = str(
            get("down_geometry_mode").value).strip().lower()
        if self.down_geometry_mode not in {"known_height", "plane", "stereo"}:
            raise ValueError(
                "down_geometry_mode must be 'known_height', 'plane', or 'stereo'")
        # Preserve the old parameter for launch files that explicitly enabled
        # the arbitrary plane path before ``down_geometry_mode`` existed.
        if bool(get("down_plane_enabled").value):
            self.down_geometry_mode = "plane"
        self.down_plane_enabled = self.down_geometry_mode != "stereo"
        normal = _finite_vector(get("down_plane_normal").value, 3)
        if normal is None or np.linalg.norm(normal) < 1e-9:
            raise ValueError("down_plane_normal must be a nonzero 3-vector")
        self.down_plane_normal = normal / np.linalg.norm(normal)
        self.down_plane_c = float(get("down_plane_c").value)
        self.down_plane_sigma = float(get("down_plane_sigma_m").value)
        self.down_plane_consistency = float(
            get("down_plane_consistency_m").value)
        self.down_default_target_z = float(
            get("down_default_target_z_m").value)
        self.down_scene_origin_z = float(
            get("down_scene_origin_z_m").value)
        if not np.isfinite(self.down_scene_origin_z):
            raise ValueError("down_scene_origin_z_m must be finite")
        ignored_classes = get("down_ignored_classes").value
        if isinstance(ignored_classes, str):
            ignored_classes = [ignored_classes]
        self.down_ignored_classes = set()
        for value in ignored_classes:
            normalized_key = str(value).strip().lower()
            for suffix in ("_front", "_down"):
                if normalized_key.endswith(suffix):
                    normalized_key = normalized_key[:-len(suffix)]
                    break
            if normalized_key in {"gate", "door"}:
                normalized_key = "gate"
            if normalized_key:
                self.down_ignored_classes.add(normalized_key)
        self.down_target_z_sigma = max(
            1e-6, float(get("down_target_z_sigma_m").value))
        self.down_min_plane_incidence = np.clip(
            float(get("down_min_plane_incidence").value), 1e-4, 1.0)
        self.down_target_z_by_class = dict(DEFAULT_DOWN_TARGET_Z)
        configured_target_z = str(get("down_target_z_json").value).strip()
        if configured_target_z:
            try:
                values = json.loads(configured_target_z)
            except (TypeError, ValueError, json.JSONDecodeError) as error:
                raise ValueError(
                    "down_target_z_json must be a JSON object") from error
            if not isinstance(values, dict):
                raise ValueError("down_target_z_json must be a JSON object")
            for key, value in values.items():
                target_z = float(value)
                if not np.isfinite(target_z):
                    raise ValueError(
                        f"down target height for {key!r} is not finite")
                normalized_key = str(key).strip().lower()
                for suffix in ("_front", "_down"):
                    if normalized_key.endswith(suffix):
                        normalized_key = normalized_key[:-len(suffix)]
                        break
                if normalized_key in {"gate", "door"}:
                    normalized_key = "gate"
                self.down_target_z_by_class[normalized_key] = target_z
        self.down_observation_pool_size = max(
            50, int(get("down_observation_pool_size").value))
        self.down_direct_queue_size = max(
            1, int(get("down_direct_queue_size").value))
        self.down_direct_queue_gate_chi2 = max(
            0.0, float(get("down_direct_queue_gate_chi2").value))
        self.guide_line_min_spacing = max(
            0.5, float(get("guide_line_min_spacing_m").value))
        self.down_duplicate_merge_distance = max(
            0.01, float(get("down_duplicate_merge_distance_m").value))

        self.pixel_sigma_fraction = float(get("pixel_sigma_fraction").value)
        self.pixel_sigma_min = float(get("pixel_sigma_min_px").value)
        self.pixel_sigma_max = float(get("pixel_sigma_max_px").value)
        self.calibration_pixel_sigma = float(
            get("calibration_pixel_sigma_px").value)
        self.pose_position_sigma = float(get("pose_position_sigma_m").value)
        self.pose_angle_sigma_rad = math.radians(
            float(get("pose_angle_sigma_deg").value))
        self.extrinsic_position_sigma = float(
            get("extrinsic_position_sigma_m").value)
        self.extrinsic_angle_sigma_rad = math.radians(
            float(get("extrinsic_angle_sigma_deg").value))
        self.shared_error_scale = max(
            0.0, float(get("shared_error_scale").value))
        self.front_stereo_scale = max(
            1.0, float(get("front_stereo_noise_scale").value))
        self.front_stereo_trusted_min_range = max(
            self.min_depth,
            float(get("front_stereo_trusted_min_range_m").value),
        )
        self.front_stereo_trusted_max_range = max(
            self.front_stereo_trusted_min_range + 1e-3,
            float(get("front_stereo_trusted_max_range_m").value),
        )
        self.front_stereo_out_of_range_scale = max(
            1.0,
            float(get("front_stereo_out_of_range_noise_scale").value),
        )
        self.front_stereo_trusted_range_only = bool(
            get("front_stereo_trusted_range_only").value)
        self.front_stereo_out_of_range_as_ray = bool(
            get("front_stereo_out_of_range_as_ray").value)
        self.front_multi_scale = float(
            get("front_multi_view_noise_scale").value)
        self.down_direct_scale = float(get("down_direct_noise_scale").value)
        self.front_bearing_seed_max_rays = max(
            4, int(get("front_bearing_seed_max_rays").value))
        self.front_bearing_seed_max_pairs = max(
            1, int(get("front_bearing_seed_max_pairs").value))
        self.front_bearing_seed_cluster_max_candidates = max(
            20, int(get("front_bearing_seed_cluster_max_candidates").value))
        self.front_bearing_cluster_radius = max(
            0.01, float(get("front_bearing_cluster_radius_m").value))
        self.front_bearing_min_cluster_rays = max(
            2, int(get("front_bearing_min_cluster_rays").value))
        self.front_bearing_max_clusters = max(
            1, int(get("front_bearing_max_clusters").value))
        self.front_bearing_clutter_likelihood = max(
            1e-9, float(get("front_bearing_clutter_likelihood").value))
        self.front_bearing_lm_iterations = max(
            1, int(get("front_bearing_lm_iterations").value))
        self.front_bearing_lm_initial_damping = max(
            1e-9, float(get("front_bearing_lm_initial_damping").value))
        self.front_bearing_track_match_distance = max(
            0.05, float(get("front_bearing_track_match_distance_m").value))
        self.front_bearing_rebuild_period = max(
            0.05, float(get("front_bearing_rebuild_period_sec").value))
        self.front_bearing_include_geometry_uncertainty = bool(
            get("front_bearing_include_geometry_uncertainty").value)
        self.front_observation_pool_size = max(
            50, int(get("front_observation_pool_size").value))
        self.front_direct_queue_size = max(
            1, int(get("front_direct_queue_size").value))
        self.front_raw_observation_window_size = max(
            2, int(get("front_raw_observation_window_size").value))
        self.front_duplicate_merge_distance = max(
            0.01, float(get("front_duplicate_merge_distance_m").value))
        self.front_gate_duplicate_merge_distance = max(
            self.front_duplicate_merge_distance,
            float(get("front_gate_duplicate_merge_distance_m").value))
        self.front_gate_min_cluster_observations = max(
            1, int(get("front_gate_min_cluster_observations").value))
        self.front_gate_min_publish_confidence = float(np.clip(
            get("front_gate_min_publish_confidence").value, 0.0, 1.0))
        self.front_gate_stable_trace = max(
            0.0, float(get("front_gate_stable_covariance_trace_m2").value))
        self.front_gate_model_covariance = np.eye(3) * max(
            0.0, float(get("front_gate_model_sigma_m").value)) ** 2
        self.front_ray_model_covariance = np.eye(3) * max(
            0.0, float(get("front_ray_model_sigma_m").value)) ** 2
        self.front_min_publish_confidence = float(np.clip(
            get("front_min_publish_confidence").value, 0.0, 1.0))

        self.max_rays = int(get("front_multi_view_max_rays").value)
        self.min_ray_angle_rad = math.radians(
            float(get("front_multi_view_min_angle_deg").value))
        self.max_line_error = float(
            get("front_multi_view_max_line_error_m").value)
        self.ray_assoc_angle_rad = math.radians(
            float(get("ray_association_angle_deg").value))
        self.gate_ray_assoc_angle_rad = math.radians(
            float(get("gate_ray_association_angle_deg").value))
        self.front_gate_reassociation_distance = max(
            0.05, float(get("front_gate_reassociation_distance_m").value))
        self.ray_assoc_distance = float(
            get("ray_association_distance_m").value)
        self.position_gate_chi2 = float(get("position_gate_chi2").value)
        self.down_direct_reanchor_chi2 = max(
            0.0, float(get("down_direct_reanchor_chi2").value))
        self.ray_gate_chi2 = float(get("ray_gate_chi2").value)
        self.huber_delta = float(get("huber_delta").value)
        self.track_timeout = float(get("track_timeout_sec").value)
        self.front_gate_track_timeout = max(
            self.track_timeout,
            float(get("front_gate_track_timeout_sec").value))
        self.stable_trace = float(get("stable_covariance_trace_m2").value)
        self.minimum_stable_observations = int(
            get("minimum_stable_observations").value)
        self.max_instances_default = max(
            1, int(get("max_instances_default").value))
        self.max_instances_guide_line = max(
            1, int(get("max_instances_guide_line").value))
        self.max_instances_gate = max(
            1, int(get("max_instances_gate").value))
        self.publish_period = float(get("publish_period_sec").value)
        self.observation_history_size = max(
            1, int(get("observation_history_size").value))

        names = get("class_names").value
        self.class_names = [str(name) for name in names] if names else []

        self.body_translation = {}
        self.body_rotation = {}
        for side in ("front_left", "front_right", "down_left", "down_right"):
            translation = _finite_vector(
                get(f"{side}_translation").value, 3)
            rotation = _finite_vector(
                get(f"{side}_rotation").value, 9)
            if translation is None or rotation is None:
                raise ValueError(f"invalid body extrinsic for {side}")
            rotation = rotation.reshape(3, 3)
            if not np.allclose(rotation @ rotation.T, np.eye(3), atol=2e-3):
                raise ValueError(f"{side}_rotation is not orthonormal")
            self.body_translation[side] = translation
            self.body_rotation[side] = rotation

        self.pose_position_covariance = np.eye(3) * self.pose_position_sigma**2
        self.pose_angle_covariance = np.eye(3) * self.pose_angle_sigma_rad**2
        self.extrinsic_position_covariance = (
            np.eye(3) * self.extrinsic_position_sigma**2)
        self.extrinsic_angle_covariance = (
            np.eye(3) * self.extrinsic_angle_sigma_rad**2)

    def _resolve_calibration_path(self, name: str, configured: str) -> str:
        if configured.strip():
            path = Path(os.path.expanduser(configured.strip()))
            if path.is_file():
                return str(path)
            raise FileNotFoundError(f"{name} calibration not found: {path}")

        candidates = []
        try:
            candidates.append(
                Path(get_package_share_directory("uv_camera"))
                / "config" / f"{name}.npz")
        except Exception:
            pass
        module_path = Path(__file__).resolve()
        candidates.append(module_path.parents[1] / "config" / f"{name}.npz")
        for candidate in candidates:
            if candidate.is_file():
                return str(candidate)
        raise FileNotFoundError(
            f"cannot find {name}.npz; checked "
            + ", ".join(str(candidate) for candidate in candidates))

    def _load_calibration(self, name: str,
                          configured: str) -> StereoCalibration | None:
        try:
            path = self._resolve_calibration_path(name, configured)
            calibration = StereoCalibration.load(name, path)
            ratio = np.linalg.norm(calibration.translation) / calibration.baseline_m
            if ratio > 10.0:
                self.get_logger().warn(
                    f"{name}.npz T norm={np.linalg.norm(calibration.translation):.3f} "
                    f"differs from P2 baseline={calibration.baseline_m:.5f} m; "
                    "using P1/P2 baseline for triangulation")
            self.get_logger().info(
                f"Loaded {name} calibration: {path}, "
                f"baseline={calibration.baseline_m:.5f} m")
            return calibration
        except (OSError, ValueError, FileNotFoundError) as error:
            self.get_logger().error(f"Failed to load {name} calibration: {error}")
            return None

    def _initialise_calibrations(self):
        if self.calibration_source == "npz":
            self._front_calibration = self._load_calibration(
                "front", self.front_calibration_file)
            self._down_calibration = self._load_calibration(
                "down", self.down_calibration_file)
        # In sim_camera_info mode these are completed after both CameraInfo
        # messages of each pair arrive.  Never silently fall back to an NPZ.
        self._front_calibration_ready = self._front_calibration is not None
        self._calibration_ready = self._down_calibration is not None

    @staticmethod
    def _camera_info_signature(message: CameraInfo) -> tuple:
        return (
            int(message.width),
            int(message.height),
            tuple(float(value) for value in message.k),
            tuple(float(value) for value in message.d),
            str(message.distortion_model),
        )

    def _camera_info_callback(self, camera: str, message: CameraInfo):
        try:
            signature = self._camera_info_signature(message)
        except (TypeError, ValueError) as error:
            self._warn_once(
                f"camera_info_invalid_{camera}",
                f"ignoring invalid {camera} CameraInfo: {error}")
            return
        if signature == self._camera_info_signatures[camera]:
            return
        self._camera_info_messages[camera] = message
        self._camera_info_signatures[camera] = signature
        self._refresh_sim_calibration(
            "front" if camera.startswith("front") else "down")

    def _refresh_sim_calibration(self, camera_pair: str):
        was_ready = self._calibration_ready or self._front_calibration_ready
        left_key = f"{camera_pair}_left"
        right_key = f"{camera_pair}_right"
        left_info = self._camera_info_messages[left_key]
        right_info = self._camera_info_messages[right_key]
        if left_info is None or right_info is None:
            return

        try:
            calibration = StereoCalibration.from_camera_info(
                camera_pair, left_info, right_info,
                self.body_translation[left_key], self.body_rotation[left_key],
                self.body_translation[right_key], self.body_rotation[right_key],
            )
        except (ValueError, cv2.error, np.linalg.LinAlgError) as error:
            if camera_pair == "front":
                self._front_calibration = None
                self._front_calibration_ready = False
            else:
                self._down_calibration = None
            self._front_calibration_ready = (
                self._front_calibration is not None)
            self._calibration_ready = self._down_calibration is not None
            self.get_logger().error(
                f"Failed to build {camera_pair} simulator calibration: {error}")
            return

        if camera_pair == "front":
            self._front_calibration = calibration
            self._front_calibration_ready = True
            self.front_width = int(left_info.width)
            self.front_height = int(left_info.height)
        else:
            self._down_calibration = calibration
            self.down_width = int(left_info.width)
            self.down_height = int(left_info.height)

        self._front_calibration_ready = self._front_calibration is not None
        self._calibration_ready = self._down_calibration is not None
        self.get_logger().info(
            f"Loaded simulator {camera_pair} calibration from CameraInfo: "
            f"{left_info.width}x{left_info.height}, "
            f"baseline={calibration.baseline_m:.5f} m, "
            f"fx={calibration.camera_matrix_left[0, 0]:.4f}, "
            f"fy={calibration.camera_matrix_left[1, 1]:.4f}")
        if ((self._calibration_ready or self._front_calibration_ready)
                and not was_ready):
            self.get_logger().info(
                "Simulator stereo calibration ready; front/down localization "
                "enabled")

    def _create_subscriptions(self):
        self.create_subscription(
            PoseInfo, "/basic_motion/pose_info", self._pose_callback, 20)
        self.create_subscription(
            DetectionArray, "/perception/detection/front_left",
            self._front_left_callback, 20)
        self.create_subscription(
            DetectionArray, "/perception/detection/front_right",
            self._front_right_callback, 20)
        self.create_subscription(
            DetectionArray, "/perception/detection/down_left",
            self._down_left_callback, 20)
        self.create_subscription(
            DetectionArray, "/perception/detection/down_right",
            self._down_right_callback, 20)
        if self.calibration_source == "sim_camera_info":
            for camera in ("front_left", "front_right", "down_left",
                           "down_right"):
                topic = self.camera_info_topics[camera]
                topic = topic.strip()
                if not topic:
                    raise ValueError(
                        f"{camera}_camera_info_topic must not be empty in sim mode")
                self.create_subscription(
                    CameraInfo, topic,
                    lambda message, source=camera:
                    self._camera_info_callback(source, message),
                    10)

    def _pose_callback(self, message: PoseInfo):
        stamp = _stamp_seconds(message.stamp)
        if stamp <= 0.0:
            stamp = self.get_clock().now().nanoseconds * 1e-9
        sample = PoseSample(
            stamp=stamp,
            position=np.array([
                float(message.robot_x),
                float(message.robot_y),
                float(message.robot_z),
            ], dtype=np.float64),
            roll_deg=float(message.robot_roll),
            pitch_deg=float(message.robot_pitch),
            yaw_deg=float(message.robot_yaw),
        )
        if self._pose_buffer and stamp < self._pose_buffer[-1].stamp:
            values = list(self._pose_buffer)
            index = bisect.bisect_left([item.stamp for item in values], stamp)
            values.insert(index, sample)
            self._pose_buffer = deque(values[-300:], maxlen=300)
        else:
            self._pose_buffer.append(sample)

    def _front_left_callback(self, message: DetectionArray):
        self._queue_detection("front_left", message)

    def _front_right_callback(self, message: DetectionArray):
        self._queue_detection("front_right", message)

    def _down_left_callback(self, message: DetectionArray):
        self._queue_detection("down_left", message)

    def _down_right_callback(self, message: DetectionArray):
        self._queue_detection("down_right", message)

    def _queue_detection(self, camera: str, message: DetectionArray):
        arrival = time.monotonic()
        self._pending[camera].append((arrival, message))
        self._try_pair("front" if camera.startswith("front") else "down")

    @staticmethod
    def _pop_deque_index(values: deque, index: int):
        items = list(values)
        result = items.pop(index)
        values.clear()
        values.extend(items)
        return result

    def _try_pair(self, camera_pair: str):
        if camera_pair not in ("front", "down"):
            return
        left_key = f"{camera_pair}_left"
        right_key = f"{camera_pair}_right"
        left_queue = self._pending[left_key]
        right_queue = self._pending[right_key]
        while left_queue and right_queue:
            best = None
            for left_index, (_, left_message) in enumerate(left_queue):
                left_stamp = _stamp_seconds(left_message.header.stamp)
                left_pair_id = int(getattr(left_message, "stereo_pair_id", 0))
                for right_index, (_, right_message) in enumerate(right_queue):
                    right_stamp = _stamp_seconds(right_message.header.stamp)
                    right_pair_id = int(
                        getattr(right_message, "stereo_pair_id", 0))
                    # The simulator preserves the original left/right image
                    # stamps, which can differ by a render interval.  A
                    # non-zero pair id is stronger than wall-clock proximity
                    # and prevents adjacent frames from stealing a match.
                    if left_pair_id > 0 and right_pair_id > 0:
                        if left_pair_id != right_pair_id:
                            continue
                        difference = 0.0
                    elif left_stamp <= 0.0 or right_stamp <= 0.0:
                        difference = abs(
                            left_queue[left_index][0] - right_queue[right_index][0])
                    else:
                        difference = abs(left_stamp - right_stamp)
                    if best is None or difference < best[0]:
                        best = (difference, left_index, right_index)
            if best is None or best[0] > self.stereo_sync_slop:
                break
            _, left_index, right_index = best
            _, left_message = self._pop_deque_index(left_queue, left_index)
            _, right_message = self._pop_deque_index(right_queue, right_index)
            if camera_pair == "front":
                self._process_front_pair(left_message, right_message)
            else:
                self._process_down_pair(left_message, right_message)

    def _flush_stale_pending(self):
        now = time.monotonic()
        for camera_pair in ("front", "down"):
            for side in ("left", "right"):
                key = f"{camera_pair}_{side}"
                queue = self._pending[key]
                while queue and now - queue[0][0] > self.pending_timeout:
                    _, message = queue.popleft()
                    if camera_pair == "front":
                        self._process_front_single(message, side)
                    elif self.down_plane_enabled:
                        self._process_down_single(message, side)

    def _lookup_pose(self, stamp: float) -> PoseAt | None:
        if not self._pose_buffer:
            self._warn_once("pose_missing", "No PoseInfo received yet")
            return None
        values = list(self._pose_buffer)
        if stamp <= 0.0:
            sample = values[-1]
            age = 0.0
            return self._pose_from_sample(sample, age)

        times = [item.stamp for item in values]
        index = bisect.bisect_left(times, stamp)
        if index <= 0:
            before = after = values[0]
        elif index >= len(values):
            before = after = values[-1]
        else:
            before, after = values[index - 1], values[index]

        if before.stamp == after.stamp:
            sample = before
            age = abs(stamp - sample.stamp)
            rotation_override = None
        else:
            span = after.stamp - before.stamp
            if before.stamp <= stamp <= after.stamp:
                alpha = (stamp - before.stamp) / span
                position = before.position * (1.0 - alpha) + after.position * alpha
                roll = before.roll_deg * (1.0 - alpha) + after.roll_deg * alpha
                pitch = before.pitch_deg * (1.0 - alpha) + after.pitch_deg * alpha
                yaw_delta = (after.yaw_deg - before.yaw_deg + 180.0) % 360.0 - 180.0
                yaw = before.yaw_deg + alpha * yaw_delta
                sample = PoseSample(stamp, position, roll, pitch, yaw)
                age = max(stamp - before.stamp, after.stamp - stamp)
                rotation_override = _slerp_rotation(
                    _rpy_to_rotation(
                        before.roll_deg, before.pitch_deg, before.yaw_deg),
                    _rpy_to_rotation(
                        after.roll_deg, after.pitch_deg, after.yaw_deg),
                    alpha,
                )
            else:
                sample = before if abs(stamp - before.stamp) <= abs(
                    stamp - after.stamp) else after
                age = abs(stamp - sample.stamp)
                rotation_override = None

        if age > self.pose_max_age_sec:
            self._warn_once(
                "pose_stale",
                f"discarding detections with pose age>{self.pose_max_age_sec:.3f}s")
            return None
        return self._pose_from_sample(sample, age, rotation_override)

    def _pose_from_sample(self, sample: PoseSample, age: float,
                          rotation_override: np.ndarray | None = None) -> PoseAt:
        age_scale = 1.0 + min(age / max(self.pose_max_age_sec, 1e-3), 5.0)
        covariance = np.zeros((6, 6), dtype=np.float64)
        covariance[:3, :3] = self.pose_position_covariance * age_scale**2
        covariance[3:, 3:] = self.pose_angle_covariance * age_scale**2
        return PoseAt(
            stamp=sample.stamp,
            position=sample.position,
            rotation=(
                _rpy_to_rotation(
                    sample.roll_deg, sample.pitch_deg, sample.yaw_deg)
                if rotation_override is None else rotation_override),
            covariance=covariance,
            age_sec=age,
            roll_deg=sample.roll_deg,
            pitch_deg=sample.pitch_deg,
            yaw_deg=sample.yaw_deg,
        )

    def _message_stamp(self, message: DetectionArray) -> float:
        return _stamp_seconds(message.header.stamp)

    def _process_front_pair(self, left_message: DetectionArray,
                            right_message: DetectionArray):
        left_pose = self._lookup_pose(self._message_stamp(left_message))
        right_pose = self._lookup_pose(self._message_stamp(right_message))
        if (left_pose is None or right_pose is None
                or not self._front_calibration_ready):
            return
        calibration = self._front_calibration
        left_detections = self._detections_for_camera(
            "front", left_message.detections)
        right_detections = self._detections_for_camera(
            "front", right_message.detections)
        pairs, unmatched_left, unmatched_right = self._match_detections(
            left_detections, right_detections, calibration,
            hard_front_geometry=True)

        used_left = set()
        used_right = set()

        for left_index, right_index in pairs:
            left = left_detections[left_index]
            right = right_detections[right_index]
            try:
                valid, reason, metrics = self._front_stereo_valid(left, right)
            except (ValueError, cv2.error, np.linalg.LinAlgError) as error:
                valid, reason, metrics = False, f"front pair check: {error}", {}
            if valid:
                try:
                    point, covariance, quality = self._stereo_measurement(
                        "front", left, right, left_pose,
                        right_pose=right_pose)
                except (ValueError, cv2.error, np.linalg.LinAlgError) as error:
                    valid = False
                    reason = f"triangulation: {error}"
                if valid:
                    # Keep every finite front stereo position in the pool.
                    # The measurement covariance already reflects whether its
                    # range is inside the calibrated 0.5--2.5 m band, so a
                    # weak far/near measurement is down-weighted rather than
                    # discarded or converted into a different observation.
                    raw_observations = (
                        self._make_front_pixel_observation(
                            left, "left", left_pose),
                        self._make_front_pixel_observation(
                            right, "right", right_pose),
                    )
                    # Every front object is represented by a bearing bundle,
                    # not by a stream of independently triangulated XYZ
                    # samples.  A gate simply receives a larger model-error
                    # floor because its apparent feature changes with view.
                    if raw_observations:
                        try:
                            left_ray = self._make_ray(
                                "front", "left", left, left_pose)
                            right_ray = self._make_ray(
                                "front", "right", right, right_pose)
                        except (ValueError, cv2.error, np.linalg.LinAlgError) as error:
                            self._warn_quality_once(f"front ray: {error}")
                            left_ray = right_ray = None
                        if left_ray is not None and right_ray is not None:
                            self._add_front_ray(
                                int(left.class_id), left_ray,
                                min(float(left.confidence),
                                    float(right.confidence)),
                                left_pose.stamp,
                                raw_observation=raw_observations[0],
                                observation_form=FORM_FRONT_STEREO)
                            self._add_front_ray(
                                int(right.class_id), right_ray,
                                min(float(left.confidence),
                                    float(right.confidence)),
                                right_pose.stamp,
                                raw_observation=raw_observations[1],
                                observation_form=FORM_FRONT_STEREO)
                            used_left.add(left_index)
                            used_right.add(right_index)
                            self._counters["front_stereo_accepted"] += 1
                            continue
                    # Once a front target has a 3-D initialization, the
                    # stereo pair is two ordinary monocular reprojection
                    # factors.  Do not create another XYZ pool sample from
                    # the same pixels; that used to make K-means split one
                    # physical gate/ball when a later triangulation drifted.
                    if self._update_front_from_raw_observations(
                            int(left.class_id), raw_observations,
                            min(float(left.confidence),
                                float(right.confidence)), FORM_FRONT_STEREO):
                        used_left.add(left_index)
                        used_right.add(right_index)
                        self._counters["front_stereo_accepted"] += 1
                        continue
                    accepted = self._handle_front_position_measurement(
                        int(left.class_id), point, covariance, left_pose,
                        FORM_FRONT_STEREO, min(
                            float(left.confidence), float(right.confidence)),
                        metrics | quality,
                        raw_observations=raw_observations)
                    used_left.add(left_index)
                    used_right.add(right_index)
                    if accepted:
                        self._counters["front_stereo_accepted"] += 1
                    continue

            self._counters["front_pair_invalid"] += 1
            self._counters["front_stereo_rejected"] += 1
            # A rejected stereo pair still contains two distinct raw pixel
            # observations.  Keep both bearings; the ray path itself will
            # reject a near-duplicate baseline, while the unified reprojection
            # window must not silently lose the right-eye constraint.
            # A stereo correspondence failure is not a failure of either
            # individual bearing.  The raw pool must retain both sides so a
            # later viewpoint can disambiguate them.  The old compatibility
            # switch is intentionally ignored by the unified bearing path.
            self._process_front_mono_detection(left, "left", left_pose)
            self._process_front_mono_detection(right, "right", right_pose)
            used_left.add(left_index)
            used_right.add(right_index)
            if reason:
                self._warn_quality_once(reason)

        for index in unmatched_left:
            self._process_front_mono_detection(
                left_detections[index], "left", left_pose)
        for index in unmatched_right:
            det = right_detections[index]
            # An unmatched right detection is a valid independent bearing.
            # Do not suppress it merely because the same class appeared on
            # the left: the left/right pair may have failed an epipolar or
            # bbox-shape gate, and both raw pixels belong in the window.
            self._process_front_mono_detection(det, "right", right_pose)

        for index in range(len(left_detections)):
            if index not in used_left and index not in unmatched_left:
                self._process_front_mono_detection(
                    left_detections[index], "left", left_pose)

    def _process_front_single(self, message: DetectionArray, side: str):
        stamp = self._message_stamp(message)
        pose = self._lookup_pose(stamp)
        if pose is None or not self._front_calibration_ready:
            return
        for detection in self._detections_for_camera("front", message.detections):
            self._process_front_mono_detection(detection, side, pose)

    def _process_front_mono_detection(self, detection: Detection,
                                      side: str, pose: PoseAt):
        try:
            ray = self._make_ray("front", side, detection, pose)
        except (ValueError, cv2.error, np.linalg.LinAlgError) as error:
            self._warn_quality_once(f"front ray: {error}")
            return
        raw_observation = self._make_front_pixel_observation(
            detection, side, pose)
        self._add_front_ray(
            int(detection.class_id), ray, float(detection.confidence),
            pose.stamp, raw_observation=raw_observation,
            observation_form=(
                FORM_FRONT_MULTI_VIEW
                if self._semantic_class(int(detection.class_id)) == "gate"
                else FORM_FRONT_MULTI_VIEW))

    def _find_front_raw_track(self, class_id: int,
                              observations: tuple[FrontPixelObservation, ...]):
        """Associate pixels to an existing front state by reprojection.

        This is deliberately independent of the newly triangulated point.
        It makes stereo and temporal monocular observations share the same
        identity test: project the existing world estimate into every image
        and compare pixels in capture-time camera poses.
        """
        semantic_class = self._semantic_class(class_id)
        candidates = []
        ray_candidates = []
        for track in self._front_tracks.values():
            if (track.physical_class_name != semantic_class
                    or track.position is None):
                continue
            residuals = []
            ray_angles = []
            for observation in observations:
                projected = self._front_project_pixel(
                    observation, track.position, self.body_translation,
                    self.body_rotation, self._front_calibration)
                if projected is None:
                    residuals = []
                    break
                sigma = max(
                    math.sqrt(max(float(observation.covariance[0, 0]), 1e-6)),
                    math.sqrt(max(float(observation.covariance[1, 1]), 1e-6)))
                residuals.append(float(np.linalg.norm(
                    observation.pixel - projected)) / sigma)
                if semantic_class == "gate":
                    ray = self._raw_front_ray(observation)
                    if ray is not None:
                        origin, direction = ray
                        vector = track.position - origin
                        distance = float(np.linalg.norm(vector))
                        if distance > 1e-6:
                            ray_angles.append(math.acos(np.clip(
                                float(np.dot(direction, vector / distance)),
                                -1.0, 1.0)))
            if residuals and max(residuals) <= 8.0:
                candidates.append((sum(value * value for value in residuals),
                                  track))
            elif (semantic_class == "gate" and ray_angles
                  and max(ray_angles) <= self.gate_ray_assoc_angle_rad):
                # Gate anchors move with viewpoint (bbox centre, visible
                # opening and red-pipe centreline are not the same physical
                # pixel).  Direction is therefore the identity cue; pixels
                # are still used afterwards to refine the state.
                ray_candidates.append((sum(value * value
                                           for value in ray_angles), track))
        if not candidates:
            if ray_candidates:
                return min(ray_candidates, key=lambda item: item[0])[1]
            return None
        return min(candidates, key=lambda item: item[0])[1]

    def _raw_front_ray(self, observation: FrontPixelObservation):
        """Return the world ray represented by one stored front pixel."""
        calibration = getattr(observation, "calibration", None)
        if calibration is None:
            calibration = getattr(self, "_front_calibration", None)
        if calibration is None:
            return None
        try:
            side = "left" if observation.camera.endswith("_left") else "right"
            ray_optical = calibration.ray_in_left_optical(
                side, observation.pixel)
            camera = observation.camera
            direction_body = self.body_rotation[camera] @ ray_optical
            direction = observation.pose.rotation @ direction_body
            direction /= max(float(np.linalg.norm(direction)), 1e-12)
            origin = observation.pose.position + observation.pose.rotation @ (
                self.body_translation[camera])
            return origin, direction
        except (KeyError, TypeError, ValueError, cv2.error,
                np.linalg.LinAlgError):
            return None

    def _update_front_from_raw_observations(
            self, class_id: int,
            observations: tuple[FrontPixelObservation, ...],
            confidence: float, form: int) -> bool:
        """Fuse new front pixels into an existing target without XYZ pooling."""
        track = self._find_front_raw_track(class_id, observations)
        if track is None:
            return False
        old_position = track.position.copy()
        old_covariance = track.covariance.copy()
        self._append_front_raw_observations(track, observations)
        optimized = self._optimize_front_track(track)
        if (not optimized or track.position is None
                or track.covariance is None):
            track.position = old_position
            track.covariance = old_covariance
        else:
            # A new pair must not make a static target jump or become less
            # certain.  Keep the old state, while retaining the pixels for a
            # later robust solve when more viewpoints arrive.
            if (float(np.trace(track.covariance))
                    > max(float(np.trace(old_covariance)) * 1.5,
                          float(np.trace(old_covariance)) + 0.05)
                    or float(np.linalg.norm(
                        track.position[:2] - old_position[:2])) > 1.25):
                track.position = old_position
                track.covariance = old_covariance
                self._counters["front_unified_update_rejected"] = (
                    self._counters.get("front_unified_update_rejected", 0) + 1)
        track.observed_class_ids.add(int(class_id))
        track.observation_count += 1
        if form == FORM_FRONT_STEREO:
            track.front_stereo_count += 1
        track.observation_form_mask |= int(form)
        track.last_observation_form = int(form)
        track.last_confidence = max(track.last_confidence, float(confidence))
        stamp = max(float(item.stamp) for item in observations)
        track.last_stamp = max(track.last_stamp, stamp)
        track.last_update_monotonic = time.monotonic()
        self._last_detection_stamp = max(self._last_detection_stamp, stamp)
        self._record_position_observation(
            track, class_id, stamp, track.position, track.covariance,
            form, confidence)
        self._counters["front_unified_pixel_updates"] = (
            self._counters.get("front_unified_pixel_updates", 0) + 1)
        return True

    def _process_down_pair(self, left_message: DetectionArray,
                           right_message: DetectionArray):
        left_pose = self._lookup_pose(self._message_stamp(left_message))
        right_pose = self._lookup_pose(self._message_stamp(right_message))
        if (left_pose is None or right_pose is None
                or not self._calibration_ready):
            return
        calibration = self._down_calibration
        left_detections = self._detections_for_camera(
            "down", left_message.detections)
        right_detections = self._detections_for_camera(
            "down", right_message.detections)
        pairs, unmatched_left, unmatched_right = self._match_detections(
            left_detections, right_detections, calibration)

        for left_index, right_index in pairs:
            left = left_detections[left_index]
            right = right_detections[right_index]
            if (self._down_class_ignored(int(left.class_id))
                    or self._down_class_ignored(int(right.class_id))):
                continue
            if self._down_height_plane_available(int(left.class_id)):
                self._process_down_plane_pair(
                    left, right, left_pose, right_pose)
            else:
                self._process_down_stereo_pair(
                    left, right, left_pose, right_pose)

        if self.down_geometry_mode in {"plane", "known_height"}:
            for index in unmatched_left:
                detection = left_detections[index]
                if self._down_height_plane_available(int(detection.class_id)):
                    self._process_down_plane_single(
                        detection, "left", left_pose)
            for index in unmatched_right:
                detection = right_detections[index]
                if self._down_height_plane_available(int(detection.class_id)):
                    self._process_down_plane_single(
                        detection, "right", right_pose)

    def _process_down_single(self, message: DetectionArray, side: str):
        if self.down_geometry_mode == "stereo":
            return
        pose = self._lookup_pose(self._message_stamp(message))
        if pose is None or not self._calibration_ready:
            return
        for detection in self._detections_for_camera("down", message.detections):
            if self._down_class_ignored(int(detection.class_id)):
                continue
            if self._down_height_plane_available(int(detection.class_id)):
                self._process_down_plane_single(detection, side, pose)

    def _process_down_stereo_pair(self, left: Detection, right: Detection,
                                  pose: PoseAt,
                                  right_pose: PoseAt | None = None):
        """Estimate a non-coplanar down target only when stereo is available."""
        try:
            valid, reason, _ = self._stereo_geometry_valid(
                left, right, self._down_calibration)
            if not valid:
                raise ValueError(reason)
            point, covariance, quality = self._stereo_measurement(
                "down", left, right, pose, right_pose=right_pose)
            accepted = self._handle_position_measurement(
                int(left.class_id), point, covariance, pose,
                FORM_DOWN_DIRECT, min(
                    float(left.confidence), float(right.confidence)),
                quality)
            if accepted:
                self._counters["down_direct_accepted"] += 1
        except (ValueError, cv2.error, np.linalg.LinAlgError) as error:
            self._counters["down_direct_rejected"] += 1
            self._warn_quality_once(f"down stereo: {error}")

    def _process_down_plane_pair(self, left: Detection, right: Detection,
                                 pose: PoseAt,
                                 right_pose: PoseAt | None = None):
        class_id = int(left.class_id)
        try:
            left_result = self._plane_measurement(
                "left", left, pose, class_id=class_id)
        except (ValueError, cv2.error, np.linalg.LinAlgError) as error:
            left_result = None
            self._warn_quality_once(f"down plane left: {error}")
        try:
            right_result = self._plane_measurement(
                "right", right, right_pose or pose,
                class_id=int(right.class_id))
        except (ValueError, cv2.error, np.linalg.LinAlgError) as error:
            right_result = None
            self._warn_quality_once(f"down plane right: {error}")
        if left_result is None and right_result is None:
            self._counters["down_direct_rejected"] += 1
            return
        if left_result is not None and right_result is not None:
            left_point = left_result[0]
            right_point = right_result[0]
            if np.linalg.norm(left_point - right_point) > self.down_plane_consistency:
                self._counters["down_direct_rejected"] += 1
                self._warn_quality_once("down plane left/right disagreement")
                return
            # The left measurement is the representative. The right one is a
            # consistency check, not a second independent factor.
            result = left_result
            confidence = min(float(left.confidence), float(right.confidence))
        else:
            result = left_result if left_result is not None else right_result
            confidence = float(
                left.confidence if left_result is not None else right.confidence)
        point, covariance, quality = result
        result_pose = pose if left_result is not None else (right_pose or pose)
        accepted = self._handle_position_measurement(
            class_id, point, covariance, result_pose,
            FORM_DOWN_DIRECT, confidence, quality)
        if accepted:
            self._counters["down_direct_accepted"] += 1

    def _process_down_plane_single(self, detection: Detection, side: str,
                                   pose: PoseAt):
        try:
            result = self._plane_measurement(
                side, detection, pose, class_id=int(detection.class_id))
        except (ValueError, cv2.error, np.linalg.LinAlgError) as error:
            self._counters["down_direct_rejected"] += 1
            self._warn_quality_once(f"down plane: {error}")
            return
        if result is None:
            self._counters["down_direct_rejected"] += 1
            return
        point, covariance, quality = result
        accepted = self._handle_position_measurement(
            int(detection.class_id), point, covariance, pose,
            FORM_DOWN_DIRECT, float(detection.confidence), quality)
        if accepted:
            self._counters["down_direct_accepted"] += 1

    def _pair_stamp(self, left: DetectionArray,
                    right: DetectionArray) -> float:
        left_stamp = self._message_stamp(left)
        right_stamp = self._message_stamp(right)
        if left_stamp <= 0.0:
            return right_stamp
        if right_stamp <= 0.0:
            return left_stamp
        return 0.5 * (left_stamp + right_stamp)

    def _match_detections(self, left_detections: list[Detection],
                          right_detections: list[Detection],
                          calibration: StereoCalibration,
                          hard_front_geometry: bool = False):
        """Match detections with a gated one-to-one assignment.

        The old implementation greedily consumed the cheapest candidate and
        had no notion of an unmatched detection.  That is unsafe for bbox-only
        models: one false positive can steal the right-eye detection belonging
        to a real object.  Front stereo uses hard visual gates here; down
        known-height processing intentionally keeps the looser class/geometry
        pairing because each eye can still provide an independent plane ray.
        """
        candidates = []
        for left_index, left in enumerate(left_detections):
            for right_index, right in enumerate(right_detections):
                if int(left.class_id) != int(right.class_id):
                    continue
                left_center = self._detection_feature_pixel(left)
                right_center = self._detection_feature_pixel(right)
                try:
                    left_rect = calibration.rectified_pixel("left", left_center)
                    right_rect = calibration.rectified_pixel("right", right_center)
                    epipolar = abs(float(left_rect[1] - right_rect[1]))
                except (ValueError, cv2.error):
                    epipolar = abs(float(left_center[1] - right_center[1]))
                lw, lh = self._bbox_size(left)
                rw, rh = self._bbox_size(right)
                if min(lw, lh, rw, rh) <= 0.0:
                    aspect_cost = 1e6
                    if hard_front_geometry:
                        self._counters["front_bbox_match_rejected"] = (
                            self._counters.get(
                                "front_bbox_match_rejected", 0) + 1)
                        continue
                else:
                    left_aspect = lw / lh
                    right_aspect = rw / rh
                    semantic_class = self._semantic_class(int(left.class_id))
                    aspect_ratio = max(
                        left_aspect / max(right_aspect, 1e-6),
                        right_aspect / max(left_aspect, 1e-6),
                    )
                    aspect_cost = abs(math.log(
                        max(left_aspect, 1e-6) /
                        max(right_aspect, 1e-6)))
                    if (hard_front_geometry
                            and semantic_class != "gate"
                            and aspect_ratio > self.bbox_aspect_ratio_max):
                        self._counters["front_aspect_match_rejected"] = (
                            self._counters.get(
                                "front_aspect_match_rejected", 0) + 1)
                        continue
                if (hard_front_geometry
                        and epipolar > self.epipolar_error_max):
                    # A severe rectified-y mismatch is evidence that the
                    # boxes are not a stereo correspondence.  Leave both
                    # detections unmatched so the front path can retain at
                    # most one bearing instead of inventing a 3-D point.
                    self._counters["front_epipolar_match_rejected"] = (
                        self._counters.get(
                            "front_epipolar_match_rejected", 0) + 1)
                    continue
                # A gate is an open frame, not a cuboid.  Its visible bbox
                # changes substantially when one post is occluded, so shape
                # is only a weak tie-breaker for gates; epipolar agreement
                # remains the primary stereo correspondence cue.
                aspect_weight = 1.0 if self._semantic_class(
                    int(left.class_id)) == "gate" else 5.0
                cost = epipolar + aspect_weight * aspect_cost
                candidates.append((cost, left_index, right_index))

        # Solve the small assignment problem exactly.  Unmatched left boxes
        # are allowed by construction; unmatched right boxes are handled in
        # the returned list below.  Object counts in this application are
        # small (normally <= a few per eye), so recursive enumeration is both
        # clearer and cheaper than adding a scipy dependency to the node.
        by_left = {}
        for cost, left_index, right_index in candidates:
            by_left.setdefault(left_index, []).append(
                (cost, right_index))
        for values in by_left.values():
            values.sort(key=lambda item: (item[0], item[1]))

        best = (0, float("inf"), ())

        def visit(left_index: int, used_right: frozenset,
                  selected: tuple[tuple[int, int], ...], total_cost: float):
            nonlocal best
            if left_index >= len(left_detections):
                score = (len(selected), -total_cost)
                best_score = (best[0], -best[1])
                if score > best_score:
                    best = (len(selected), total_cost, selected)
                return

            # Explicitly leave this left detection unmatched.
            visit(left_index + 1, used_right, selected, total_cost)
            for cost, right_index in by_left.get(left_index, ()):
                if right_index in used_right:
                    continue
                visit(
                    left_index + 1,
                    used_right | frozenset((right_index,)),
                    selected + ((left_index, right_index),),
                    total_cost + cost,
                )

        visit(0, frozenset(), (), 0.0)
        pairs = list(best[2])
        used_left = {left_index for left_index, _ in pairs}
        used_right = {right_index for _, right_index in pairs}
        unmatched_left = [
            index for index in range(len(left_detections))
            if index not in used_left
        ]
        unmatched_right = [
            index for index in range(len(right_detections))
            if index not in used_right
        ]
        return pairs, unmatched_left, unmatched_right

    def _bbox_size(self, detection: Detection) -> tuple[float, float]:
        width = float(detection.bbox_x2 - detection.bbox_x1)
        height = float(detection.bbox_y2 - detection.bbox_y1)
        return max(width, 0.0), max(height, 0.0)

    @staticmethod
    def _detection_feature_pixel(detection: Detection) -> np.ndarray:
        """Return the image point used by the front geometric estimator.

        ``pixel_x/pixel_y`` remain the detector bbox center for transport and
        legacy consumers.  Gate detections may additionally carry a stable
        opening anchor from a centerline or segmentation result.  Keep the
        fallback tolerant of old bags/tests whose ``Detection`` message does
        not have the optional fields yet.
        """
        try:
            feature_type = int(getattr(detection, "feature_type", 0) or 0)
        except (TypeError, ValueError):
            feature_type = 0
        # The current optional feature contract is specifically for the
        # front gate class.  Do not let a malformed/forward-version field on
        # another class silently change its geometric meaning.
        if feature_type > 0 and int(getattr(detection, "class_id", -1)) == 3:
            try:
                feature = np.array([
                    float(getattr(detection, "feature_pixel_x")),
                    float(getattr(detection, "feature_pixel_y")),
                ], dtype=np.float64)
                if np.all(np.isfinite(feature)):
                    return feature
            except (AttributeError, TypeError, ValueError):
                pass
        return np.array([
            float(detection.pixel_x), float(detection.pixel_y),
        ], dtype=np.float64)

    @staticmethod
    def _detection_feature_id(detection: Detection) -> str:
        """Name the physical image feature represented by a detection."""
        try:
            feature_type = int(getattr(detection, "feature_type", 0) or 0)
        except (TypeError, ValueError):
            feature_type = 0
        if int(getattr(detection, "class_id", -1)) != 3:
            return "bbox_center"
        return {
            1: "gate_centerline",
            2: "gate_segmentation",
        }.get(feature_type, "bbox_center")

    def _bbox_at_edge(self, detection: Detection) -> bool:
        values = np.array([
            detection.bbox_x1, detection.bbox_y1,
            detection.bbox_x2, detection.bbox_y2,
        ], dtype=np.float64)
        if not np.all(np.isfinite(values)):
            return True
        width = self.front_width
        height = self.front_height
        margin = max(
            self.edge_margin_px,
            self.edge_margin_ratio * min(width, height),
        )
        return (
            float(detection.bbox_x1) <= margin
            or float(detection.bbox_y1) <= margin
            or float(detection.bbox_x2) >= width - margin
            or float(detection.bbox_y2) >= height - margin
        )

    def _front_stereo_valid(self, left: Detection, right: Detection):
        """Check the remaining visual correspondence conditions for a pair.

        The caller has already applied the hard rectified-epipolar gate while
        building the one-to-one assignment.  Disparity and range remain
        quality indicators for front stereo: weak but finite points are kept
        with inflated covariance rather than discarded just for being far.
        Non-finite or behind-camera triangulation falls back to a mono ray.
        """
        if self._bbox_at_edge(left) or self._bbox_at_edge(right):
            return False, "front pair rejected: bbox touches image edge", {}
        lw, lh = self._bbox_size(left)
        rw, rh = self._bbox_size(right)
        if min(lw, lh, rw, rh) <= 0.0:
            return False, "front pair rejected: invalid bbox", {}
        left_aspect = lw / lh
        right_aspect = rw / rh
        aspect_ratio = max(
            left_aspect / right_aspect,
            right_aspect / left_aspect,
        )
        if (self._semantic_class(int(left.class_id)) != "gate"
                and aspect_ratio > self.bbox_aspect_ratio_max):
            return False, "front pair rejected: bbox aspect ratio mismatch", {}

        try:
            metrics = self._stereo_geometry_metrics(
                left, right, self._front_calibration)
        except (ValueError, cv2.error, np.linalg.LinAlgError):
            metrics = {}
        metrics.update({
            "left_bbox_aspect_ratio": left_aspect,
            "right_bbox_aspect_ratio": right_aspect,
            "bbox_aspect_ratio": aspect_ratio,
        })
        return True, "", metrics

    @staticmethod
    def _stereo_geometry_metrics(left: Detection, right: Detection,
                                 calibration: StereoCalibration):
        left_rect = calibration.rectified_pixel(
            "left", ObjectLocalizer._detection_feature_pixel(left))
        right_rect = calibration.rectified_pixel(
            "right", ObjectLocalizer._detection_feature_pixel(right))
        return {
            "epipolar_error_px": abs(float(left_rect[1] - right_rect[1])),
            "disparity_px": abs(float(left_rect[0] - right_rect[0])),
        }

    def _stereo_geometry_valid(self, left: Detection, right: Detection,
                               calibration: StereoCalibration):
        metrics = self._stereo_geometry_metrics(left, right, calibration)
        epipolar_error = metrics["epipolar_error_px"]
        disparity = metrics["disparity_px"]
        if epipolar_error > self.epipolar_error_max:
            return False, "stereo pair rejected: epipolar error", metrics
        if disparity < self.min_disparity:
            return False, "stereo pair rejected: disparity too small", metrics
        return True, "", metrics

    def _front_stereo_range_trusted(self, range_m: float) -> bool:
        """Return whether a front 3-D estimate is in its reliable range."""
        minimum = getattr(self, "front_stereo_trusted_min_range", 0.5)
        maximum = getattr(self, "front_stereo_trusted_max_range", 2.5)
        return bool(
            np.isfinite(range_m)
            and minimum <= float(range_m) <= maximum
        )

    def _front_stereo_range_scale(self, range_m: float) -> float:
        """Inflate front uncertainty as a point leaves the trusted band."""
        minimum = getattr(self, "front_stereo_trusted_min_range", 0.5)
        maximum = getattr(self, "front_stereo_trusted_max_range", 2.5)
        if np.isfinite(range_m) and minimum <= float(range_m) <= maximum:
            return 1.0
        limit = getattr(self, "front_stereo_out_of_range_scale", 6.0)
        if not np.isfinite(range_m) or float(range_m) <= 0.0:
            return float(limit)
        ratio = (
            minimum / float(range_m)
            if float(range_m) < minimum
            else float(range_m) / maximum
        )
        return float(np.clip(ratio, 1.0, limit))

    def _pixel_covariance(self, detection: Detection,
                          width: int, height: int) -> np.ndarray:
        box_width, box_height = self._bbox_size(detection)
        sigma_u = np.clip(
            self.pixel_sigma_fraction * max(box_width, 1.0),
            self.pixel_sigma_min, self.pixel_sigma_max)
        sigma_v = np.clip(
            self.pixel_sigma_fraction * max(box_height, 1.0),
            self.pixel_sigma_min, self.pixel_sigma_max)
        confidence = max(float(detection.confidence), 0.05)
        confidence_scale = np.clip(0.8 / confidence, 1.0, 4.0)
        sigma_u *= confidence_scale
        sigma_v *= confidence_scale
        return np.diag([sigma_u**2, sigma_v**2])

    @staticmethod
    def _covariance_diagonal_envelope(covariances) -> np.ndarray | None:
        """Return a conservative per-axis envelope for common errors.

        Cross-frame pose/extrinsic errors are not independent observations.
        A full cross-time covariance would require a joint estimator, so the
        current bounded-window estimator keeps a conservative diagonal floor.
        Taking the largest retained variance per axis prevents the floor from
        shrinking when an old view leaves the window.
        """
        values = []
        for covariance in covariances:
            if covariance is None:
                continue
            try:
                value = np.asarray(covariance, dtype=np.float64)
            except (TypeError, ValueError):
                continue
            if value.shape != (3, 3) or not np.all(np.isfinite(value)):
                continue
            values.append(np.maximum(np.diag(value), 0.0))
        if not values:
            return None
        return np.diag(np.max(np.asarray(values), axis=0))

    @staticmethod
    def _apply_covariance_floor(covariance: np.ndarray,
                                floor: np.ndarray | None) -> np.ndarray:
        """Keep a covariance above a non-averaging common-error floor."""
        result = _regularize_covariance(covariance)
        if floor is None:
            return result
        value = np.asarray(floor, dtype=np.float64)
        if value.shape != (3, 3) or not np.all(np.isfinite(value)):
            return result
        diagonal = np.maximum(np.diag(result), np.maximum(np.diag(value), 0.0))
        result = result.copy()
        for index in range(3):
            result[index, index] = diagonal[index]
        return _regularize_covariance(result)

    def _merge_track_systematic_covariance(
            self, track: TargetTrack,
            covariance: np.ndarray | None) -> np.ndarray | None:
        """Accumulate shared geometry uncertainty without double counting.

        The track covariance already contains the conditional Kalman result;
        this method only updates the separate floor.  Callers then apply the
        floor once after each update/window refit.
        """
        if covariance is None:
            return getattr(track, "systematic_covariance", None)
        value = np.asarray(covariance, dtype=np.float64)
        if value.shape != (3, 3) or not np.all(np.isfinite(value)):
            return getattr(track, "systematic_covariance", None)
        current = getattr(track, "systematic_covariance", None)
        envelope = self._covariance_diagonal_envelope((current, value))
        track.systematic_covariance = envelope
        return envelope

    def _ray_shared_covariance(self, position, rays) -> np.ndarray:
        """Approximate common pose/mounting uncertainty for a ray solution."""
        point = np.asarray(position, dtype=np.float64).reshape(3)
        ranges = [
            float(np.linalg.norm(point - np.asarray(ray.origin)))
            for ray in rays
        ]
        range_m = max([value for value in ranges if np.isfinite(value)] or [0.0])
        pose_position_sigma = float(getattr(self, "pose_position_sigma", 0.0))
        extrinsic_position_sigma = float(
            getattr(self, "extrinsic_position_sigma", 0.0))
        pose_angle_sigma = float(getattr(self, "pose_angle_sigma_rad", 0.0))
        extrinsic_angle_sigma = float(
            getattr(self, "extrinsic_angle_sigma_rad", 0.0))
        translation_sigma = math.hypot(
            pose_position_sigma, extrinsic_position_sigma)
        angle_sigma = math.hypot(pose_angle_sigma, extrinsic_angle_sigma)
        scale = float(getattr(self, "shared_error_scale", 1.0))
        sigma = scale * (translation_sigma + range_m * angle_sigma)
        return np.eye(3, dtype=np.float64) * max(sigma, 0.0)**2

    def _world_point(self, point_in_left_optical: np.ndarray,
                     camera_pair: str, side: str,
                     pose: PoseAt,
                     body_translation: np.ndarray | None = None,
                     body_rotation: np.ndarray | None = None) -> np.ndarray:
        camera = f"{camera_pair}_{side}"
        translation = (
            self.body_translation[camera]
            if body_translation is None else body_translation)
        rotation = (
            self.body_rotation[camera]
            if body_rotation is None else body_rotation)
        point_body = translation + rotation @ point_in_left_optical
        return pose.position + pose.rotation @ point_body

    def _stereo_world_from_pixels(self, camera_pair: str, pixels: np.ndarray,
                                  pose_vector: np.ndarray,
                                  side: str = "left") -> np.ndarray:
        calibration = (
            self._front_calibration if camera_pair == "front"
            else self._down_calibration)
        left_pixel = pixels[:2]
        right_pixel = pixels[2:]
        point_rectified = calibration.triangulate_left_rectified(
            left_pixel, right_pixel)
        point_optical = calibration.rectification_left.T @ point_rectified
        pose = self._pose_from_vector(pose_vector)
        return self._world_point(point_optical, camera_pair, side, pose)

    def _world_ray_from_pose_vector(
            self, camera_pair: str, side: str, pixel: np.ndarray,
            pose_vector: np.ndarray, translation=None, rotation=None):
        """Build one world ray for a camera at its own capture-time pose."""
        calibration = (
            self._front_calibration if camera_pair == "front"
            else self._down_calibration)
        camera = f"{camera_pair}_{side}"
        ray_optical = calibration.ray_in_left_optical(side, pixel)
        if translation is None:
            translation = self.body_translation[camera]
        if rotation is None:
            rotation = self.body_rotation[camera]
        pose = self._pose_from_vector(pose_vector)
        origin = pose.position + pose.rotation @ np.asarray(
            translation, dtype=np.float64)
        direction = pose.rotation @ (
            np.asarray(rotation, dtype=np.float64) @ ray_optical)
        direction /= max(float(np.linalg.norm(direction)), 1e-12)
        return origin, direction

    def _async_stereo_world_from_pixels(
            self, camera_pair: str, pixels: np.ndarray,
            left_pose_vector: np.ndarray,
            right_pose_vector: np.ndarray,
            left_translation=None, left_rotation=None,
            right_translation=None, right_rotation=None):
        """Triangulate two rays expressed at their actual capture poses."""
        left_origin, left_direction = self._world_ray_from_pose_vector(
            camera_pair, "left", pixels[:2], left_pose_vector,
            left_translation, left_rotation)
        right_origin, right_direction = self._world_ray_from_pose_vector(
            camera_pair, "right", pixels[2:], right_pose_vector,
            right_translation, right_rotation)
        projectors = (
            np.eye(3) - np.outer(left_direction, left_direction),
            np.eye(3) - np.outer(right_direction, right_direction),
        )
        normal_matrix = projectors[0] + projectors[1]
        if np.linalg.matrix_rank(normal_matrix, tol=1e-8) < 3:
            raise ValueError("asynchronous stereo rays are near parallel")
        point = _safe_inverse(normal_matrix) @ (
            projectors[0] @ left_origin + projectors[1] @ right_origin)
        if not np.all(np.isfinite(point)):
            raise ValueError("asynchronous stereo point is non-finite")
        depths = (
            float(np.dot(point - left_origin, left_direction)),
            float(np.dot(point - right_origin, right_direction)),
        )
        if any(not np.isfinite(depth) or depth <= 1e-6 for depth in depths):
            raise ValueError("asynchronous stereo point is behind a camera")
        return point, left_origin, left_direction, right_origin, right_direction

    def _async_stereo_measurement(self, camera_pair: str, left: Detection,
                                  right: Detection, pose: PoseAt,
                                  right_pose: PoseAt):
        """Measure a moving-platform stereo pair without averaging poses."""
        calibration = (
            self._front_calibration if camera_pair == "front"
            else self._down_calibration)
        pixels = np.r_[
            self._detection_feature_pixel(left),
            self._detection_feature_pixel(right),
        ].astype(np.float64)
        left_pose_vector = self._pose_vector(pose)
        right_pose_vector = self._pose_vector(right_pose)
        (point_world, left_origin, left_direction,
         right_origin, right_direction) = self._async_stereo_world_from_pixels(
             camera_pair, pixels, left_pose_vector, right_pose_vector)

        image_width = self.front_width if camera_pair == "front" else self.down_width
        image_height = self.front_height if camera_pair == "front" else self.down_height
        pixel_covariance = np.zeros((4, 4), dtype=np.float64)
        pixel_covariance[:2, :2] = self._pixel_covariance(
            left, image_width, image_height)
        pixel_covariance[2:, 2:] = self._pixel_covariance(
            right, image_width, image_height)
        pixel_covariance += np.eye(4) * self.calibration_pixel_sigma**2
        pixel_jacobian = _numeric_jacobian(
            lambda value: self._async_stereo_world_from_pixels(
                camera_pair, value, left_pose_vector, right_pose_vector)[0],
            pixels, np.full(4, 0.25, dtype=np.float64))

        pose_value = np.r_[left_pose_vector, right_pose_vector]
        pose_steps = np.tile(
            np.array([1e-3, 1e-3, 1e-3, 1e-4, 1e-4, 1e-4]), 2)
        pose_jacobian = _numeric_jacobian(
            lambda value: self._async_stereo_world_from_pixels(
                camera_pair, pixels, value[:6], value[6:])[0],
            pose_value, pose_steps)
        pose_covariance = np.zeros((12, 12), dtype=np.float64)
        pose_covariance[:6, :6] = pose.covariance
        pose_covariance[6:, 6:] = right_pose.covariance
        covariance = pixel_jacobian @ pixel_covariance @ pixel_jacobian.T
        covariance += pose_jacobian @ pose_covariance @ pose_jacobian.T

        def world_from_extrinsic(value):
            left_translation = self.body_translation[f"{camera_pair}_left"] \
                + value[:3]
            left_rotation = _axis_angle_rotation(value[3:6]) @ \
                self.body_rotation[f"{camera_pair}_left"]
            right_translation = self.body_translation[f"{camera_pair}_right"] \
                + value[6:9]
            right_rotation = _axis_angle_rotation(value[9:12]) @ \
                self.body_rotation[f"{camera_pair}_right"]
            return self._async_stereo_world_from_pixels(
                camera_pair, pixels, left_pose_vector, right_pose_vector,
                left_translation, left_rotation,
                right_translation, right_rotation)[0]

        extrinsic_jacobian = _numeric_jacobian(
            world_from_extrinsic, np.zeros(12, dtype=np.float64),
            np.full(12, 1e-4, dtype=np.float64))
        extrinsic_covariance = np.zeros((12, 12), dtype=np.float64)
        extrinsic_covariance[:6, :6] = np.block([
            [self.extrinsic_position_covariance, np.zeros((3, 3))],
            [np.zeros((3, 3)), self.extrinsic_angle_covariance],
        ])
        extrinsic_covariance[6:, 6:] = extrinsic_covariance[:6, :6]
        covariance += (
            extrinsic_jacobian @ extrinsic_covariance
            @ extrinsic_jacobian.T)

        shared_covariance = np.zeros((3, 3), dtype=np.float64)
        nominal_pose_covariance = np.zeros((12, 12), dtype=np.float64)
        nominal_pose_covariance[:6, :6] = np.block([
            [self.pose_position_covariance, np.zeros((3, 3))],
            [np.zeros((3, 3)), self.pose_angle_covariance],
        ])
        nominal_pose_covariance[6:, 6:] = nominal_pose_covariance[:6, :6]
        shared_covariance += (
            pose_jacobian @ nominal_pose_covariance @ pose_jacobian.T)
        shared_covariance += (
            extrinsic_jacobian @ extrinsic_covariance
            @ extrinsic_jacobian.T)
        shared_covariance *= float(getattr(self, "shared_error_scale", 1.0))**2

        scale = (
            self.front_stereo_scale if camera_pair == "front"
            else self.down_direct_scale)
        covariance = _regularize_covariance(covariance * scale**2)
        metrics = self._stereo_geometry_metrics(left, right, calibration)
        epipolar_scale = max(
            1.0, metrics["epipolar_error_px"]
            / max(self.epipolar_error_max, 1e-6))
        disparity_scale = max(
            1.0, self.min_disparity / max(metrics["disparity_px"], 1e-6))
        covariance = _regularize_covariance(
            covariance * max(epipolar_scale, disparity_scale)**2)
        range_m = float(np.linalg.norm(point_world - pose.position))
        range_scale = 1.0
        if camera_pair == "front":
            range_scale = self._front_stereo_range_scale(range_m)
            covariance = _regularize_covariance(covariance * range_scale**2)
        elif range_m < self.min_depth or range_m > self.max_depth:
            raise ValueError(f"stereo range {range_m:.3f}m out of range")
        quality = {
            "depth_m": float(np.dot(point_world - left_origin, left_direction)),
            "baseline_m": calibration.baseline_m,
            "range_m": range_m,
            "range_scale": range_scale,
            "trusted_range": (
                self._front_stereo_range_trusted(range_m)
                if camera_pair == "front" else True),
            "async_stereo": True,
            "shared_covariance": _regularize_covariance(shared_covariance),
        }
        return point_world, covariance, quality

    def _pose_from_vector(self, value: np.ndarray) -> PoseAt:
        position = np.asarray(value[:3], dtype=np.float64)
        rotation = _rpy_to_rotation(*np.asarray(value[3:6], dtype=np.float64))
        covariance = np.zeros((6, 6), dtype=np.float64)
        return PoseAt(
            0.0, position, rotation, covariance, 0.0,
            float(value[3]), float(value[4]), float(value[5]))

    def _stereo_measurement(self, camera_pair: str, left: Detection,
                            right: Detection, pose: PoseAt,
                            right_pose: PoseAt | None = None):
        if (right_pose is not None
                and abs(float(right_pose.stamp) - float(pose.stamp)) > 1e-6):
            return self._async_stereo_measurement(
                camera_pair, left, right, pose, right_pose)
        calibration = (
            self._front_calibration if camera_pair == "front"
            else self._down_calibration)
        pixels = np.r_[
            self._detection_feature_pixel(left),
            self._detection_feature_pixel(right),
        ].astype(np.float64)
        point_rectified = calibration.triangulate_left_rectified(
            pixels[:2], pixels[2:])
        point_optical = calibration.rectification_left.T @ point_rectified
        point_world = self._world_point(
            point_optical, camera_pair, "left", pose)
        if not np.all(np.isfinite(point_world)):
            raise ValueError("non-finite stereo world point")

        image_width = self.front_width if camera_pair == "front" else self.down_width
        image_height = self.front_height if camera_pair == "front" else self.down_height
        pixel_covariance = np.zeros((4, 4), dtype=np.float64)
        pixel_covariance[:2, :2] = self._pixel_covariance(
            left, image_width, image_height)
        pixel_covariance[2:, 2:] = self._pixel_covariance(
            right, image_width, image_height)
        calibration_pixel_variance = self.calibration_pixel_sigma**2
        pixel_covariance += np.eye(4) * calibration_pixel_variance

        pixel_steps = np.full(4, 0.25, dtype=np.float64)
        pixel_jacobian = _numeric_jacobian(
            lambda p: self._stereo_world_from_pixels(
                camera_pair, p, self._pose_vector(pose)),
            pixels, pixel_steps)

        pose_vector = self._pose_vector(pose)
        pose_steps = np.array([
            1e-3, 1e-3, 1e-3,
            1e-4, 1e-4, 1e-4,
        ], dtype=np.float64)
        pose_jacobian = _numeric_jacobian(
            lambda p: self._stereo_world_from_pixels(
                camera_pair, pixels, p),
            pose_vector, pose_steps)

        covariance = pixel_jacobian @ pixel_covariance @ pixel_jacobian.T
        covariance += pose_jacobian @ pose.covariance @ pose_jacobian.T
        shared_covariance = np.zeros((3, 3), dtype=np.float64)
        nominal_pose_covariance = np.zeros((6, 6), dtype=np.float64)
        nominal_pose_covariance[:3, :3] = getattr(
            self, "pose_position_covariance", np.zeros((3, 3)))
        nominal_pose_covariance[3:, 3:] = getattr(
            self, "pose_angle_covariance", np.zeros((3, 3)))
        shared_covariance += (
            pose_jacobian @ nominal_pose_covariance @ pose_jacobian.T)

        camera = f"{camera_pair}_left"
        translation = self.body_translation[camera]
        rotation = self.body_rotation[camera]
        extrinsic_value = np.zeros(6, dtype=np.float64)
        extrinsic_steps = np.array([
            1e-4, 1e-4, 1e-4,
            1e-4, 1e-4, 1e-4,
        ], dtype=np.float64)

        def world_from_extrinsic(value):
            translated = translation + value[:3]
            rotated = _axis_angle_rotation(value[3:]) @ rotation
            return self._world_point(
                point_optical, camera_pair, "left", pose,
                translated, rotated)

        extrinsic_jacobian = _numeric_jacobian(
            world_from_extrinsic, extrinsic_value, extrinsic_steps)
        extrinsic_covariance = np.zeros((6, 6), dtype=np.float64)
        extrinsic_covariance[:3, :3] = self.extrinsic_position_covariance
        extrinsic_covariance[3:, 3:] = self.extrinsic_angle_covariance
        covariance += (
            extrinsic_jacobian @ extrinsic_covariance
            @ extrinsic_jacobian.T
        )
        shared_covariance += (
            extrinsic_jacobian @ extrinsic_covariance
            @ extrinsic_jacobian.T
        )
        shared_covariance *= float(getattr(self, "shared_error_scale", 1.0))**2

        scale = (
            self.front_stereo_scale if camera_pair == "front"
            else self.down_direct_scale)
        covariance = _regularize_covariance(covariance * scale**2)
        depth_left = float(point_rectified[2])
        depth_right = depth_left
        if not np.isfinite(depth_right):
            raise ValueError("right depth is non-finite")

        if camera_pair == "front":
            # Do not reject front pairs for a particular epipolar or disparity
            # threshold.  Their geometry is represented by the covariance so
            # weak pairs have less influence in the front pool/filter.  A
            # non-positive or almost-zero depth is not a quality preference:
            # it cannot represent a point in front of the camera, so the
            # caller falls back to a mono multi-view ray instead.
            if depth_left <= 1e-6:
                raise ValueError(
                    f"front stereo depth {depth_left:.3f}m is not usable")
            try:
                metrics = self._stereo_geometry_metrics(
                    left, right, calibration)
                epipolar_scale = max(
                    1.0,
                    metrics["epipolar_error_px"]
                    / max(self.epipolar_error_max, 1e-6),
                )
                disparity_scale = max(
                    1.0,
                    self.min_disparity
                    / max(metrics["disparity_px"], 1e-6),
                )
                covariance = _regularize_covariance(
                    covariance * max(epipolar_scale, disparity_scale)**2)
            except (ValueError, cv2.error, np.linalg.LinAlgError):
                pass
            range_m = float(np.linalg.norm(point_world - pose.position))
            if not np.isfinite(range_m):
                raise ValueError("front stereo range is non-finite")
            range_scale = self._front_stereo_range_scale(range_m)
            covariance = _regularize_covariance(
                covariance * range_scale**2)
        elif depth_left <= self.min_depth or depth_left > self.max_depth:
            raise ValueError(f"stereo depth {depth_left:.3f}m out of range")
        quality = {
            "depth_m": depth_left,
            "baseline_m": calibration.baseline_m,
        }
        if camera_pair == "front":
            quality.update({
                "range_m": range_m,
                "range_scale": range_scale,
                "trusted_range": self._front_stereo_range_trusted(range_m),
            })
        quality["shared_covariance"] = _regularize_covariance(
            shared_covariance)
        return point_world, covariance, quality

    @staticmethod
    def _pose_vector(pose: PoseAt) -> np.ndarray:
        # Numerical pose Jacobians use position plus the local RPY values.
        return np.r_[pose.position, pose.roll_deg, pose.pitch_deg, pose.yaw_deg]

    def _down_height_plane_available(self, class_id: int) -> bool:
        """Return whether a down detection has a valid known-height plane."""
        if self._down_class_ignored(class_id):
            return False
        mode = getattr(self, "down_geometry_mode", "plane")
        if mode == "plane":
            return True
        if mode != "known_height":
            return False
        semantic_class = self._semantic_class(int(class_id))
        target_heights = getattr(
            self, "down_target_z_by_class", DEFAULT_DOWN_TARGET_Z)
        # No implicit default: a class must be explicitly known to be
        # coplanar with a configured target surface.
        return semantic_class in target_heights

    def _down_class_ignored(self, class_id: int) -> bool:
        """Return whether a class is intentionally disabled for down view."""
        return self._semantic_class(int(class_id)) in getattr(
            self, "down_ignored_classes", {"gate"})

    def _down_plane_parameters(self, class_id: int | None = None):
        """Return the plane used by one down-camera observation.

        In known-height mode every class gets its own horizontal target plane.
        The configured heights are absolute scene depths, while the incoming
        pose is local odom; subtracting ``down_scene_origin_z`` makes the
        geometry equivalent to ``target_depth - current_depth``.  This is
        important for the scene's suspended impact balls: their centre is well
        above the pool floor.  The legacy ``plane`` mode keeps the configured
        arbitrary plane for backwards compatibility.
        """
        mode = getattr(self, "down_geometry_mode", "plane")
        if mode == "known_height":
            semantic_class = self._semantic_class(int(class_id)) \
                if class_id is not None else ""
            if semantic_class in getattr(
                    self, "down_ignored_classes", {"gate"}):
                return None
            target_heights = getattr(
                self, "down_target_z_by_class", DEFAULT_DOWN_TARGET_Z)
            if semantic_class not in target_heights:
                return None
            scene_target_z = float(target_heights[semantic_class])
            if not np.isfinite(scene_target_z):
                raise ValueError(
                    f"down target height for {semantic_class!r} is invalid")
            target_z = scene_target_z - getattr(
                self, "down_scene_origin_z", 0.0)
            return (
                np.array([0.0, 0.0, 1.0], dtype=np.float64),
                -target_z,
                target_z,
            )
        return self.down_plane_normal, self.down_plane_c, None

    def _plane_world_from_pixel(self, camera_pair: str, side: str,
                                pixel: np.ndarray, pose_vector: np.ndarray,
                                plane_normal: np.ndarray | None = None,
                                plane_c: float | None = None):
        calibration = (
            self._front_calibration if camera_pair == "front"
            else self._down_calibration)
        ray_optical = calibration.ray_in_left_optical(side, pixel)
        camera = f"{camera_pair}_{side}"
        translation = self.body_translation[camera]
        rotation = self.body_rotation[camera]
        pose = self._pose_from_vector(pose_vector)
        direction = pose.rotation @ (rotation @ ray_optical)
        direction /= max(np.linalg.norm(direction), 1e-12)
        origin = pose.position + pose.rotation @ translation
        if plane_normal is None:
            plane_normal = self.down_plane_normal
        if plane_c is None:
            plane_c = self.down_plane_c
        plane_normal = np.asarray(plane_normal, dtype=np.float64).reshape(3)
        denominator = float(plane_normal @ direction)
        if abs(denominator) < 1e-5:
            raise ValueError("down ray is parallel to plane")
        scale = -(
            float(plane_normal @ origin) + float(plane_c)
        ) / denominator
        if scale <= 0.0 or not np.isfinite(scale):
            raise ValueError("down plane intersection is behind camera")
        return origin + scale * direction

    def _plane_measurement(self, side: str, detection: Detection,
                           pose: PoseAt, class_id: int | None = None):
        pixels = np.array([detection.pixel_x, detection.pixel_y],
                          dtype=np.float64)
        pose_value = self._pose_vector(pose)
        plane_parameters = self._down_plane_parameters(class_id)
        if plane_parameters is None:
            raise ValueError(
                f"no known-height plane for down class "
                f"{self._semantic_class(int(class_id)) if class_id is not None else 'unknown'}")
        plane_normal, plane_c, target_z = plane_parameters
        if abs(float(plane_normal[2])) < 1e-6 and target_z is not None:
            raise ValueError("known-height down plane must be horizontal")
        point = self._plane_world_from_pixel(
            "down", side, pixels, pose_value, plane_normal, plane_c)
        if not np.all(np.isfinite(point)):
            return None

        pixel_covariance = self._pixel_covariance(
            detection, self.down_width, self.down_height)
        pixel_covariance += np.eye(2) * self.calibration_pixel_sigma**2
        pixel_jacobian = _numeric_jacobian(
            lambda p: self._plane_world_from_pixel(
                "down", side, p, pose_value, plane_normal, plane_c),
            pixels, np.full(2, 0.25))
        pose_jacobian = _numeric_jacobian(
            lambda p: self._plane_world_from_pixel(
                "down", side, pixels, p, plane_normal, plane_c),
            pose_value, np.array([1e-3, 1e-3, 1e-3,
                                  1e-4, 1e-4, 1e-4]))
        covariance = pixel_jacobian @ pixel_covariance @ pixel_jacobian.T
        covariance += pose_jacobian @ pose.covariance @ pose_jacobian.T
        shared_covariance = np.zeros((3, 3), dtype=np.float64)
        nominal_pose_covariance = np.zeros((6, 6), dtype=np.float64)
        nominal_pose_covariance[:3, :3] = getattr(
            self, "pose_position_covariance", np.zeros((3, 3)))
        nominal_pose_covariance[3:, 3:] = getattr(
            self, "pose_angle_covariance", np.zeros((3, 3)))
        shared_covariance += (
            pose_jacobian @ nominal_pose_covariance @ pose_jacobian.T)
        if target_z is None:
            covariance += np.eye(3) * self.down_plane_sigma**2
            shared_covariance += np.eye(3) * self.down_plane_sigma**2
        else:
            target_z_sigma = getattr(
                self, "down_target_z_sigma", self.down_plane_sigma)
            height_jacobian = _numeric_jacobian(
                lambda value: self._plane_world_from_pixel(
                    "down", side, pixels, pose_value,
                    plane_normal, -float(value[0])),
                np.array([target_z], dtype=np.float64),
                np.array([1e-4], dtype=np.float64))
            covariance += height_jacobian @ height_jacobian.T * target_z_sigma**2
            shared_covariance += (
                height_jacobian @ height_jacobian.T * target_z_sigma**2)

        # Uncertainty of the camera mounting transform.
        camera = f"down_{side}"
        translation = self.body_translation[camera]
        rotation = self.body_rotation[camera]
        camera_origin = pose.position + pose.rotation @ translation
        extrinsic_value = np.zeros(6)

        def point_from_extrinsic(value):
            calibration = self._down_calibration
            ray = calibration.ray_in_left_optical(side, pixels)
            translated = translation + value[:3]
            rotated = _axis_angle_rotation(value[3:]) @ rotation
            origin = pose.position + pose.rotation @ translated
            direction = pose.rotation @ (rotated @ ray)
            denominator = float(plane_normal @ direction)
            if abs(denominator) < 1e-5:
                raise ValueError("down ray became parallel to plane")
            scale = -(
                float(plane_normal @ origin) + float(plane_c)
            ) / denominator
            return origin + scale * direction

        extrinsic_jacobian = _numeric_jacobian(
            point_from_extrinsic, extrinsic_value,
            np.full(6, 1e-4))
        extrinsic_covariance = np.zeros((6, 6))
        extrinsic_covariance[:3, :3] = self.extrinsic_position_covariance
        extrinsic_covariance[3:, 3:] = self.extrinsic_angle_covariance
        covariance += (
            extrinsic_jacobian @ extrinsic_covariance
            @ extrinsic_jacobian.T
        )
        shared_covariance += (
            extrinsic_jacobian @ extrinsic_covariance
            @ extrinsic_jacobian.T
        )
        shared_covariance *= float(getattr(self, "shared_error_scale", 1.0))**2
        covariance = _regularize_covariance(
            covariance * self.down_direct_scale**2)
        incidence = abs(float(plane_normal @ (
            pose.rotation @ (
                self.body_rotation[camera] @
                self._down_calibration.ray_in_left_optical(side, pixels)))))
        if incidence < getattr(self, "down_min_plane_incidence", 0.0):
            raise ValueError(
                f"down ray incidence {incidence:.3f} is too shallow")
        return point, covariance, {
            "plane_incidence": incidence,
            # target_z_m is local odom/NED, which is the frame of the
            # published target position.  The scene value is retained for
            # debugging/calibration checks.
            "target_z_m": float(point[2]) if target_z is None
            else float(target_z),
            "target_scene_z_m": None if target_z is None else float(
                target_z + getattr(self, "down_scene_origin_z", 0.0)),
            "robot_scene_depth_m": float(
                pose.position[2] + getattr(
                    self, "down_scene_origin_z", 0.0)),
            "target_height_m": None if target_z is None else float(
                target_z - camera_origin[2]),
            "geometry_mode": getattr(self, "down_geometry_mode", "plane"),
            "shared_covariance": _regularize_covariance(
                shared_covariance),
        }

    def _make_ray(self, camera_pair: str, side: str,
                  detection: Detection, pose: PoseAt):
        calibration = (
            self._front_calibration if camera_pair == "front"
            else self._down_calibration)
        pixel = self._detection_feature_pixel(detection)
        ray_optical = calibration.ray_in_left_optical(side, pixel)
        camera = f"{camera_pair}_{side}"
        direction_body = self.body_rotation[camera] @ ray_optical
        direction_world = pose.rotation @ direction_body
        direction_world /= max(np.linalg.norm(direction_world), 1e-12)
        origin_world = pose.position + pose.rotation @ self.body_translation[camera]
        sigma_pixel = self._pixel_covariance(
            detection,
            self.front_width if camera_pair == "front" else self.down_width,
            self.front_height if camera_pair == "front" else self.down_height)
        focal = (
            calibration.projection_left[0, 0]
            if side == "left" else calibration.projection_right[0, 0])
        sigma_angle = max(
            math.sqrt(float(sigma_pixel[0, 0])) / max(focal, 1e-6),
            math.sqrt(float(sigma_pixel[1, 1])) / max(
                calibration.projection_left[1, 1]
                if side == "left" else calibration.projection_right[1, 1],
                1e-6))
        sigma_angle = max(sigma_angle, math.radians(0.03))
        if camera_pair == "front" and self._bbox_at_edge(detection):
            # With bbox-only labels there is no visible corner/edge from
            # which to recover the true object anchor after truncation.  Keep
            # the bearing for search and multi-view identity, but make its
            # influence explicitly weaker than a fully visible box.
            sigma_angle *= getattr(self, "front_edge_ray_noise_scale", 3.0)
        return origin_world, direction_world, sigma_angle

    def _make_front_pixel_observation(self, detection: Detection, side: str,
                                      pose: PoseAt) -> FrontPixelObservation:
        """Snapshot a front image feature and its capture-time pose."""
        raw_observation_id = int(getattr(
            self, "_next_raw_observation_id", 1))
        self._next_raw_observation_id = raw_observation_id + 1
        pose_copy = PoseAt(
            stamp=float(pose.stamp),
            position=np.asarray(pose.position, dtype=np.float64).copy(),
            rotation=np.asarray(pose.rotation, dtype=np.float64).copy(),
            covariance=np.asarray(pose.covariance, dtype=np.float64).copy(),
            age_sec=float(pose.age_sec),
            roll_deg=float(pose.roll_deg),
            pitch_deg=float(pose.pitch_deg),
            yaw_deg=float(pose.yaw_deg),
        )
        return FrontPixelObservation(
            stamp=float(pose.stamp),
            camera=f"front_{side}",
            pixel=self._detection_feature_pixel(detection),
            covariance=self._pixel_covariance(
                detection, self.front_width, self.front_height),
            pose=pose_copy,
            confidence=float(detection.confidence),
            raw_observation_id=raw_observation_id,
            feature_id=self._detection_feature_id(detection),
            calibration=getattr(self, "_front_calibration", None),
        )

    @staticmethod
    def _bearing_tangent_basis(direction: np.ndarray) -> np.ndarray:
        """Return an orthonormal 3x2 basis of a unit bearing's tangent plane."""
        value = np.asarray(direction, dtype=np.float64).reshape(3)
        norm = float(np.linalg.norm(value))
        if not np.isfinite(norm) or norm < 1e-12:
            raise ValueError("bearing direction must be finite and non-zero")
        value = value / norm
        reference = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        if abs(float(np.dot(value, reference))) > 0.9:
            reference = np.array([0.0, 1.0, 0.0], dtype=np.float64)
        first = np.cross(value, reference)
        first /= max(float(np.linalg.norm(first)), 1e-12)
        second = np.cross(value, first)
        second /= max(float(np.linalg.norm(second)), 1e-12)
        return np.column_stack((first, second))

    def _front_bearing_covariance(
            self, observation: FrontPixelObservation,
            direction_world: np.ndarray,
            sigma_angle: float) -> np.ndarray:
        """Propagate the captured pixel covariance into bearing tangent space.

        The first implementation keeps the capture pose and calibration fixed
        while solving.  This is intentional: pose/extrinsic uncertainty is a
        later common-mode term, whereas the current estimator is meant to
        validate the bearing likelihood itself.
        """
        calibration = (getattr(observation, "calibration", None)
                       or getattr(self, "_front_calibration", None))
        fallback = np.eye(2, dtype=np.float64) * max(
            float(sigma_angle), math.radians(0.03)) ** 2
        if calibration is None:
            return fallback
        camera = str(observation.camera)
        side = "left" if camera.endswith("_left") else "right"
        try:
            def direction_from_pixel(pixel):
                ray_optical = calibration.ray_in_left_optical(side, pixel)
                direction_body = self.body_rotation[camera] @ ray_optical
                value = observation.pose.rotation @ direction_body
                return value / max(float(np.linalg.norm(value)), 1e-12)

            jacobian = _numeric_jacobian(
                direction_from_pixel,
                np.asarray(observation.pixel, dtype=np.float64),
                np.array([0.25, 0.25], dtype=np.float64),
            )
            pixel_covariance = _regularize_covariance(
                np.asarray(observation.covariance, dtype=np.float64)
                + np.eye(2, dtype=np.float64) *
                float(getattr(self, "calibration_pixel_sigma", 0.0)) ** 2,
                minimum_variance=1e-12,
            )
            world_covariance = jacobian @ pixel_covariance @ jacobian.T
            basis = self._bearing_tangent_basis(direction_world)
            covariance = basis.T @ world_covariance @ basis
            # Preserve the existing edge-ray inflation even if the numerical
            # pixel propagation happens to produce a smaller value.
            largest = math.sqrt(max(float(np.max(np.diag(covariance))), 1e-12))
            scale = max(float(sigma_angle), math.radians(0.03)) / largest
            if scale > 1.0:
                covariance *= scale * scale
            return _regularize_covariance(covariance, minimum_variance=1e-12)
        except (KeyError, TypeError, ValueError, cv2.error,
                np.linalg.LinAlgError):
            return fallback

    def _add_front_bearing_pool_ray(
            self, class_id: int, ray, confidence: float, stamp: float,
            raw_observation: FrontPixelObservation,
            observation_form: int) -> RayObservation:
        """Retain one raw front bearing and rebuild its class hypotheses.

        This is deliberately track-free.  A new ray cannot consume an
        instance slot and cannot be rejected because it is outside an old
        track's direction gate.  Only the finite ray itself is validated.
        """
        origin, direction, sigma_angle = ray
        origin = np.asarray(origin, dtype=np.float64).reshape(3)
        direction = np.asarray(direction, dtype=np.float64).reshape(3)
        norm = float(np.linalg.norm(direction))
        if (not np.all(np.isfinite(origin)) or not np.all(np.isfinite(direction))
                or not np.isfinite(norm) or norm < 1e-12):
            raise ValueError("front bearing is non-finite")
        direction /= norm
        bearing_camera = None
        calibration = (getattr(raw_observation, "calibration", None)
                       or getattr(self, "_front_calibration", None))
        if calibration is not None:
            side = ("left" if raw_observation.camera.endswith("_left")
                    else "right")
            try:
                bearing_camera = calibration.ray_in_left_optical(
                    side, raw_observation.pixel)
            except (ValueError, cv2.error, np.linalg.LinAlgError):
                bearing_camera = None
        covariance = self._front_bearing_covariance(
            raw_observation, direction, sigma_angle)
        ray_id = int(getattr(self, "_next_ray_id", 1))
        self._next_ray_id = ray_id + 1
        observation = RayObservation(
            stamp=float(stamp),
            origin=origin.copy(),
            direction=direction.copy(),
            sigma_angle=float(sigma_angle),
            confidence=float(np.clip(confidence, 0.0, 1.0)),
            ray_id=ray_id,
            raw_observation=raw_observation,
            class_id=int(class_id),
            camera=str(raw_observation.camera),
            bearing_camera=(None if bearing_camera is None else
                            np.asarray(bearing_camera, dtype=np.float64).copy()),
            bearing_covariance=covariance,
            observation_form=int(observation_form),
        )
        semantic_class = self._semantic_class(class_id)
        pool = getattr(self, "_front_bearing_pool", None)
        if pool is None:
            self._front_bearing_pool = {}
            pool = self._front_bearing_pool
        values = pool.setdefault(
            semantic_class,
            deque(maxlen=max(50, int(getattr(
                self, "front_observation_pool_size", 300)))),
        )
        # Raw observation IDs are unique.  This also makes replayed callbacks
        # idempotent if a transport retries the same DetectionArray.
        for index, previous in enumerate(values):
            if (int(getattr(previous, "raw_observation_id", 0))
                    == int(raw_observation.raw_observation_id)
                    and int(raw_observation.raw_observation_id) > 0):
                values[index] = observation
                break
        else:
            values.append(observation)
            self._counters["front_bearing_pool_added"] = (
                self._counters.get("front_bearing_pool_added", 0) + 1)
            self._counters["front_multi_view_rays"] = (
                self._counters.get("front_multi_view_rays", 0) + 1)
        self._record_ray_observation(
            None, class_id, stamp, origin, direction, confidence,
            source_raw_observation_ids=(int(raw_observation.raw_observation_id),),
            feature_id=raw_observation.feature_id,
            form=observation_form)
        self._last_detection_stamp = max(
            getattr(self, "_last_detection_stamp", 0.0), float(stamp))
        dirty_classes = getattr(self, "_front_bearing_dirty_classes", None)
        if dirty_classes is None:
            # Unit/offline callers that construct the node without __init__
            # retain the old synchronous helper behaviour.
            self._rebuild_front_bearing_clusters(semantic_class)
        else:
            dirty_classes.add(semantic_class)
        return observation

    @staticmethod
    def _closest_ray_pair(first: RayObservation, second: RayObservation):
        """Return the closest-point midpoint and line gap for two rays."""
        d1 = np.asarray(first.direction, dtype=np.float64)
        d2 = np.asarray(second.direction, dtype=np.float64)
        o1 = np.asarray(first.origin, dtype=np.float64)
        o2 = np.asarray(second.origin, dtype=np.float64)
        dot = float(np.clip(np.dot(d1, d2), -1.0, 1.0))
        denominator = 1.0 - dot * dot
        if denominator <= 1e-10:
            return None
        delta = o1 - o2
        d = float(np.dot(d1, delta))
        e = float(np.dot(d2, delta))
        lambda_1 = (dot * e - d) / denominator
        lambda_2 = (e - dot * d) / denominator
        if (not np.isfinite(lambda_1) or not np.isfinite(lambda_2)
                or lambda_1 <= 1e-6 or lambda_2 <= 1e-6):
            return None
        point_1 = o1 + lambda_1 * d1
        point_2 = o2 + lambda_2 * d2
        midpoint = 0.5 * (point_1 + point_2)
        gap = float(np.linalg.norm(point_1 - point_2))
        if not np.all(np.isfinite(midpoint)) or not np.isfinite(gap):
            return None
        return midpoint, gap

    def _front_bearing_seed_clusters(self, rays: list[RayObservation],
                                     semantic_class: str):
        """Build provisional clusters from pairwise ray intersections.

        This is only an initialization for the probabilistic estimator.  A
        ray that fails to form a good pair remains in the raw pool and can be
        assigned later by a cluster initialized from another pair.
        """
        max_rays = max(4, int(getattr(
            self, "front_bearing_seed_max_rays", 80)))
        selected = rays[-max_rays:]
        selected_offset = len(rays) - len(selected)
        max_pairs = max(1, int(getattr(
            self, "front_bearing_seed_max_pairs", 2400)))
        line_error_limit = max(0.01, float(getattr(
            self, "max_line_error", 1.0)))
        min_angle = float(getattr(
            self, "min_ray_angle_rad", math.radians(3.0)))
        candidates = []
        for first in range(len(selected)):
            for second in range(first + 1, len(selected)):
                if len(candidates) >= max_pairs:
                    break
                first_ray = selected[first]
                second_ray = selected[second]
                angle = math.acos(np.clip(float(np.dot(
                    first_ray.direction, second_ray.direction)), -1.0, 1.0))
                if angle < min_angle:
                    continue
                result = self._closest_ray_pair(first_ray, second_ray)
                if result is None:
                    continue
                point, gap = result
                if gap > line_error_limit:
                    continue
                weight = (max(float(first_ray.confidence), 0.05)
                          * max(float(second_ray.confidence), 0.05)
                          / max(gap * gap + 1e-4, 1e-4))
                candidates.append((
                    point, weight, selected_offset + first,
                    selected_offset + second))
            if len(candidates) >= max_pairs:
                break
        self._counters["front_bearing_seed_candidates"] = (
            self._counters.get("front_bearing_seed_candidates", 0)
            + len(candidates))
        if not candidates:
            return []

        seed_candidate_limit = max(20, int(getattr(
            self, "front_bearing_seed_cluster_max_candidates", 600)))
        if len(candidates) > seed_candidate_limit:
            # Candidate generation may inspect all recent ray pairs, but
            # density seeding only needs the most consistent pairs.  All raw
            # rays still participate in the later soft bearing solve.
            candidates.sort(key=lambda item: item[1], reverse=True)
            candidates = candidates[:seed_candidate_limit]

        radius = max(0.01, float(getattr(
            self, "front_bearing_cluster_radius", 0.35)))
        if semantic_class == "gate":
            radius = max(radius, float(getattr(
                self, "front_gate_duplicate_merge_distance", 0.50)))
        parent = list(range(len(candidates)))

        def find(value):
            while parent[value] != value:
                parent[value] = parent[parent[value]]
                value = parent[value]
            return value

        def union(first, second):
            first_root, second_root = find(first), find(second)
            if first_root != second_root:
                parent[second_root] = first_root

        # Spatial hashing keeps the density pass bounded.  A full candidate
        # matrix would be O(max_seed_pairs^2) and would make the estimator
        # compete with the AI thread when the pool is busy.
        grid = {}
        for index, candidate in enumerate(candidates):
            cell = tuple(np.floor(candidate[0] / radius).astype(np.int64))
            grid.setdefault(cell, []).append(index)
        neighbour_offsets = [
            (dn, de, dd)
            for dn in (-1, 0, 1)
            for de in (-1, 0, 1)
            for dd in (-1, 0, 1)
        ]
        for first, candidate in enumerate(candidates):
            cell = tuple(np.floor(candidate[0] / radius).astype(np.int64))
            for offset in neighbour_offsets:
                neighbour = tuple(cell[index] + offset[index]
                                  for index in range(3))
                for second in grid.get(neighbour, ()):
                    if second <= first:
                        continue
                    distance = float(np.linalg.norm(
                        candidate[0] - candidates[second][0]))
                    if distance <= radius:
                        union(first, second)

        components = {}
        for index, candidate in enumerate(candidates):
            components.setdefault(find(index), []).append(candidate)
        ordered = sorted(
            components.values(),
            key=lambda items: (-len({value for item in items
                                      for value in item[2:]}),
                               -sum(item[1] for item in items)),
        )
        limit = max(1, int(getattr(
            self, "front_bearing_max_clusters", 8)))
        seeds = []
        for component in ordered[:limit]:
            weights = np.asarray([item[1] for item in component],
                                 dtype=np.float64)
            points = np.asarray([item[0] for item in component],
                                dtype=np.float64)
            center = np.average(points, axis=0, weights=weights)
            ray_indices = sorted({index for item in component
                                  for index in item[2:]})
            if np.all(np.isfinite(center)) and len(ray_indices) >= 2:
                seeds.append((center, ray_indices))
        return seeds

    def _front_bearing_terms(self, ray: RayObservation,
                             position: np.ndarray):
        """Return tangent residual/Jacobian/covariance for one world bearing."""
        vector = np.asarray(position, dtype=np.float64) - ray.origin
        range_m = float(np.linalg.norm(vector))
        if not np.isfinite(range_m) or range_m <= 1e-6:
            return None
        predicted = vector / range_m
        depth = float(np.dot(vector, ray.direction))
        if not np.isfinite(depth) or depth <= 1e-6:
            return None
        basis = self._bearing_tangent_basis(ray.direction)
        residual = basis.T @ predicted
        jacobian = basis.T @ (
            (np.eye(3) - np.outer(predicted, predicted)) / range_m)
        covariance = getattr(ray, "bearing_covariance", None)
        if covariance is None:
            covariance = np.eye(2, dtype=np.float64) * max(
                float(ray.sigma_angle), math.radians(0.03)) ** 2
        covariance = _regularize_covariance(
            covariance, minimum_variance=1e-12)
        # Bbox centres are view-dependent semantic anchors.  Their model
        # error is expressed in metres and converted to angular noise at the
        # current range; it is not treated as a reason to discard the view.
        model_sigma_m = (float(getattr(
            self, "front_gate_model_sigma_m", 0.12))
                         if self._semantic_class(ray.class_id) == "gate"
                         else float(getattr(
                             self, "front_ray_model_sigma_m", 0.05)))
        covariance = covariance + np.eye(2) * max(
            0.0, model_sigma_m / range_m) ** 2
        if bool(getattr(self, "front_bearing_include_geometry_uncertainty",
                       False)):
            shared = self._ray_shared_covariance(position, [ray])
            covariance += np.eye(2) * max(
                float(np.trace(shared)) / 3.0, 0.0) / max(range_m**2, 1e-9)
        covariance = _regularize_covariance(
            covariance, minimum_variance=1e-12)
        inverse = _safe_inverse(covariance)
        squared = float(residual.T @ inverse @ residual)
        return residual, jacobian, covariance, squared

    def _front_bearing_batch_terms(self, rays: list[RayObservation],
                                   position: np.ndarray):
        """Vectorized tangent residuals for one candidate position.

        The estimator may retain hundreds of bearings.  Keeping the geometry
        in arrays avoids thousands of Python calls and tiny matrix inversions
        during every LM/soft-association pass.
        """
        count = len(rays)
        if count == 0:
            return (np.zeros(0, dtype=bool), np.zeros((0, 2)),
                    np.zeros((0, 2, 3)), np.zeros((0, 2, 2)),
                    np.full(0, np.inf))
        origins = np.asarray([ray.origin for ray in rays], dtype=np.float64)
        directions = np.asarray([ray.direction for ray in rays],
                                dtype=np.float64)
        vectors = np.asarray(position, dtype=np.float64).reshape(1, 3) - origins
        ranges = np.linalg.norm(vectors, axis=1)
        valid = np.isfinite(ranges) & (ranges > 1e-6)
        predicted = np.zeros_like(vectors)
        predicted[valid] = vectors[valid] / ranges[valid, None]
        depths = np.einsum("ij,ij->i", vectors, directions)
        valid &= np.isfinite(depths) & (depths > 1e-6)

        references = np.zeros_like(directions)
        references[:, 2] = 1.0
        use_y_reference = np.abs(directions[:, 2]) > 0.9
        references[use_y_reference] = np.array([0.0, 1.0, 0.0])
        first = np.cross(directions, references)
        first_norm = np.linalg.norm(first, axis=1)
        first /= np.maximum(first_norm[:, None], 1e-12)
        second = np.cross(directions, first)
        second /= np.maximum(np.linalg.norm(second, axis=1)[:, None], 1e-12)
        basis = np.stack((first, second), axis=2)  # N x 3 x 2
        residuals = np.einsum("nki,nk->ni", basis, predicted)
        projection = (np.eye(3, dtype=np.float64)[None, :, :]
                      - predicted[:, :, None] * predicted[:, None, :])
        jacobians = np.einsum(
            "nki,nij->nkj", np.transpose(basis, (0, 2, 1)),
            projection / np.maximum(ranges, 1e-6)[:, None, None])

        covariances = np.asarray([
            (np.eye(2, dtype=np.float64) * max(
                float(ray.sigma_angle), math.radians(0.03)) ** 2
             if getattr(ray, "bearing_covariance", None) is None
             else np.asarray(ray.bearing_covariance, dtype=np.float64))
            for ray in rays
        ], dtype=np.float64)
        covariances = 0.5 * (covariances + np.transpose(covariances, (0, 2, 1)))
        model_sigmas = np.asarray([
            (float(getattr(self, "front_gate_model_sigma_m", 0.12))
             if self._semantic_class(ray.class_id) == "gate" else
             float(getattr(self, "front_ray_model_sigma_m", 0.05)))
            for ray in rays
        ], dtype=np.float64)
        covariances += np.eye(2, dtype=np.float64)[None, :, :] * (
            model_sigmas / np.maximum(ranges, 1e-6))[:, None, None] ** 2
        if bool(getattr(self, "front_bearing_include_geometry_uncertainty",
                       False)):
            translation_sigma = math.hypot(
                float(getattr(self, "pose_position_sigma", 0.0)),
                float(getattr(self, "extrinsic_position_sigma", 0.0)))
            angle_sigma = math.hypot(
                float(getattr(self, "pose_angle_sigma_rad", 0.0)),
                float(getattr(self, "extrinsic_angle_sigma_rad", 0.0)))
            shared_scale = float(getattr(self, "shared_error_scale", 1.0))
            geometry_sigma = shared_scale * (
                translation_sigma + ranges * angle_sigma)
            covariances += np.eye(2, dtype=np.float64)[None, :, :] * (
                geometry_sigma / np.maximum(ranges, 1e-6))[:, None, None] ** 2
        determinants = (covariances[:, 0, 0] * covariances[:, 1, 1]
                        - covariances[:, 0, 1] * covariances[:, 1, 0])
        determinants = np.maximum(determinants, 1e-24)
        inverse = np.empty_like(covariances)
        inverse[:, 0, 0] = covariances[:, 1, 1] / determinants
        inverse[:, 1, 1] = covariances[:, 0, 0] / determinants
        inverse[:, 0, 1] = -covariances[:, 0, 1] / determinants
        inverse[:, 1, 0] = -covariances[:, 1, 0] / determinants
        squared = np.einsum(
            "ni,nij,nj->n", residuals, inverse, residuals)
        squared = np.maximum(squared, 0.0)
        valid &= np.all(np.isfinite(residuals), axis=1)
        valid &= np.all(np.isfinite(jacobians), axis=(1, 2))
        valid &= np.all(np.isfinite(covariances), axis=(1, 2))
        squared[~valid] = np.inf
        return valid, residuals, jacobians, covariances, squared

    def _front_bearing_linear_initialization(
            self, rays: list[RayObservation], weights=None):
        """Point-to-ray least-squares initialisation from the pasted design."""
        if len(rays) < 2:
            return None
        if weights is None:
            weights = np.ones(len(rays), dtype=np.float64)
        weights = np.asarray(weights, dtype=np.float64).reshape(-1)
        if weights.size != len(rays):
            return None
        normal = np.zeros((3, 3), dtype=np.float64)
        rhs = np.zeros(3, dtype=np.float64)
        for weight, ray in zip(weights, rays):
            if not np.isfinite(weight) or weight <= 0.0:
                continue
            projector = np.eye(3) - np.outer(ray.direction, ray.direction)
            normal += float(weight) * projector
            rhs += float(weight) * projector @ ray.origin
        if np.linalg.matrix_rank(normal, tol=1e-9) < 3:
            return None
        result = _safe_inverse(normal) @ rhs
        return result if np.all(np.isfinite(result)) else None

    @staticmethod
    def _huber_cost(squared: float, delta: float) -> float:
        value = math.sqrt(max(float(squared), 0.0))
        if value <= delta:
            return 0.5 * value * value
        return delta * (value - 0.5 * delta)

    def _optimize_front_bearing_cluster(
            self, rays: list[RayObservation], initial: np.ndarray,
            memberships: np.ndarray | None = None):
        """Robust LM/Huber maximum-likelihood estimate for one ray cluster."""
        if len(rays) < 2:
            return None
        if memberships is None:
            memberships = np.ones(len(rays), dtype=np.float64)
        memberships = np.clip(np.asarray(memberships, dtype=np.float64), 0.0, 1.0)
        if memberships.size != len(rays) or float(np.sum(memberships)) < 2.0:
            return None
        position = np.asarray(initial, dtype=np.float64).reshape(3).copy()
        if not np.all(np.isfinite(position)):
            return None
        delta_huber = max(0.5, float(getattr(self, "huber_delta", 2.5)))

        def system(value):
            valid_mask, residuals, jacobians, covariances, squared = (
                self._front_bearing_batch_terms(rays, value))
            valid_mask &= memberships > 1e-6
            whitened = np.sqrt(np.maximum(squared, 0.0))
            robust = np.ones(len(rays), dtype=np.float64)
            large = whitened > delta_huber
            robust[large] = delta_huber / np.maximum(whitened[large], 1e-9)
            weights = memberships * robust * valid_mask.astype(np.float64)
            inverse = np.zeros_like(covariances)
            determinants = (covariances[:, 0, 0] * covariances[:, 1, 1]
                            - covariances[:, 0, 1] * covariances[:, 1, 0])
            determinants = np.maximum(determinants, 1e-24)
            inverse[:, 0, 0] = covariances[:, 1, 1] / determinants
            inverse[:, 1, 1] = covariances[:, 0, 0] / determinants
            inverse[:, 0, 1] = -covariances[:, 0, 1] / determinants
            inverse[:, 1, 0] = -covariances[:, 1, 0] / determinants
            normal = np.einsum(
                "n,nki,nkl,nlj->ij", weights, jacobians, inverse,
                jacobians)
            gradient = np.einsum(
                "n,nki,nkl,nl->i", weights, jacobians, inverse, residuals)
            safe_squared = np.where(valid_mask, squared, 0.0)
            safe_whitened = np.sqrt(np.maximum(safe_squared, 0.0))
            huber_cost = np.where(
                safe_whitened <= delta_huber,
                0.5 * safe_squared,
                delta_huber * (safe_whitened - 0.5 * delta_huber),
            )
            cost = float(np.sum(memberships * huber_cost))
            return normal, gradient, cost, int(np.count_nonzero(valid_mask)), (
                valid_mask, residuals, jacobians, covariances, squared)

        damping = max(1e-9, float(getattr(
            self, "front_bearing_lm_initial_damping", 1e-3)))
        normal, gradient, cost, valid, _ = system(position)
        if valid < 2 or np.linalg.matrix_rank(normal, tol=1e-9) < 3:
            self._counters["front_bearing_rank_deficient"] = (
                self._counters.get("front_bearing_rank_deficient", 0) + 1)
            return None
        for _ in range(max(1, int(getattr(
                self, "front_bearing_lm_iterations", 10)))):
            diagonal = np.maximum(np.diag(normal), 1e-9)
            step = -_safe_inverse(normal + damping * np.diag(diagonal)) @ gradient
            if not np.all(np.isfinite(step)):
                return None
            step_norm = float(np.linalg.norm(step))
            if step_norm > 2.0:
                step *= 2.0 / step_norm
            trial = position + step
            trial_normal, trial_gradient, trial_cost, trial_valid, _ = system(trial)
            if (trial_valid >= 2 and np.isfinite(trial_cost)
                    and trial_cost <= cost):
                position = trial
                normal, gradient, cost, valid = (
                    trial_normal, trial_gradient, trial_cost, trial_valid)
                damping = max(damping * 0.5, 1e-9)
                if step_norm < 1e-5:
                    break
            else:
                damping = min(damping * 10.0, 1e12)

        normal, _, _, valid, batch = system(position)
        if valid < 2 or np.linalg.matrix_rank(normal, tol=1e-9) < 3:
            self._counters["front_bearing_rank_deficient"] = (
                self._counters.get("front_bearing_rank_deficient", 0) + 1)
            return None
        effective_count = float(np.sum(memberships))
        valid_mask, _, _, _, squared = batch
        weighted_squared = float(np.sum(
            memberships[valid_mask] * squared[valid_mask]))
        inlier_count = int(np.count_nonzero(
            (memberships >= 0.5) & valid_mask
            & (squared <= delta_huber**2)))
        dof = max(1.0, 2.0 * effective_count - 3.0)
        scale_squared = max(1.0, weighted_squared / dof)
        covariance = _regularize_covariance(
            _safe_inverse(normal) * scale_squared,
            minimum_variance=1e-10)
        if bool(getattr(self, "front_bearing_include_geometry_uncertainty",
                       False)):
            covariance = covariance + self._ray_shared_covariance(position, rays)
        return {
            "position": position,
            "covariance": _regularize_covariance(covariance),
            "normal": normal,
            "cost": float(cost),
            "effective_count": effective_count,
            "inlier_count": int(inlier_count),
            "mean_whitened_residual": math.sqrt(max(
                weighted_squared / max(2.0 * effective_count, 1.0), 0.0)),
            "information_eigenvalues": np.sort(
                np.maximum(np.linalg.eigvalsh(normal), 0.0))[::-1],
            "covariance_eigenvalues": np.sort(
                np.maximum(np.linalg.eigvalsh(covariance), 0.0))[::-1],
            "condition_number": float(
                np.linalg.cond(normal)) if np.all(np.isfinite(normal))
                else float("inf"),
        }

    def _front_bearing_memberships(self, rays, positions):
        """Compute soft cluster probabilities plus an explicit clutter term."""
        if not positions:
            return np.zeros((len(rays), 0), dtype=np.float64)
        result = np.zeros((len(rays), len(positions)), dtype=np.float64)
        clutter_log = math.log(max(float(getattr(
            self, "front_bearing_clutter_likelihood", 0.08)), 1e-12))
        log_likelihood = np.full(
            (len(rays), len(positions)), -1e9, dtype=np.float64)
        for position_index, position in enumerate(positions):
            valid, _, _, covariance, squared = self._front_bearing_batch_terms(
                rays, position)
            determinant = (covariance[:, 0, 0] * covariance[:, 1, 1]
                           - covariance[:, 0, 1] * covariance[:, 1, 0])
            values = (-0.5 * np.minimum(squared, 200.0)
                      - 0.5 * np.log(np.maximum(determinant, 1e-24)))
            log_likelihood[valid, position_index] = values[valid]
        maximum = np.maximum(
            clutter_log, np.max(log_likelihood, axis=1))
        values = np.exp(np.clip(
            log_likelihood - maximum[:, None], -80.0, 0.0))
        clutter = np.exp(np.clip(clutter_log - maximum, -80.0, 0.0))
        result = values / np.maximum(
            clutter[:, None] + np.sum(values, axis=1)[:, None], 1e-12)
        return result

    def _update_front_track_from_bearing_cluster(
            self, track: TargetTrack, class_id: int,
            rays: list[RayObservation], estimate: dict):
        """Copy one batch cluster into the persistent output track."""
        track.class_id = int(class_id)
        track.physical_class_name = self._semantic_class(class_id)
        track.observed_class_ids = {int(class_id)}
        track.position = np.asarray(estimate["position"], dtype=np.float64).copy()
        track.covariance = _regularize_covariance(estimate["covariance"])
        track.front_filter_position = track.position.copy()
        track.front_filter_covariance = track.covariance.copy()
        track.rays = deque(rays[-max(2, int(getattr(
            self, "front_multi_view_max_rays", 20))):])
        raw = [ray.raw_observation for ray in rays
               if ray.raw_observation is not None]
        limit = max(2, int(getattr(
            self, "front_raw_observation_window_size", 100)))
        track.front_pixel_observations = deque(raw[-limit:])
        for observation in track.front_pixel_observations:
            observation.target_instance_id = int(track.instance_id)
        track.observation_count = len(rays)
        track.front_stereo_count = sum(
            int(ray.observation_form) == FORM_FRONT_STEREO for ray in rays)
        track.front_multi_view_count = sum(
            int(ray.observation_form) != FORM_FRONT_STEREO for ray in rays)
        track.down_direct_count = 0
        track.observation_form_mask = 0
        for ray in rays:
            track.observation_form_mask |= int(ray.observation_form)
        track.last_observation_form = int(rays[-1].observation_form)
        track.last_confidence = max(
            [float(ray.confidence) for ray in rays] or [0.0])
        track.last_stamp = max([float(ray.stamp) for ray in rays] or [0.0])
        track.last_update_monotonic = time.monotonic()
        track.systematic_covariance = None
        track.front_effective_observations = float(
            estimate.get("effective_count", len(rays)))
        track.front_inlier_observations = int(
            estimate.get("inlier_count", len(rays)))
        track.front_mean_residual = float(
            estimate.get("mean_whitened_residual", 0.0))
        information_eigenvalues = estimate.get("information_eigenvalues")
        track.front_information_eigenvalues = (
            None if information_eigenvalues is None else
            np.asarray(information_eigenvalues, dtype=np.float64).copy())
        covariance_eigenvalues = estimate.get("covariance_eigenvalues")
        track.front_covariance_eigenvalues = (
            None if covariance_eigenvalues is None else
            np.asarray(covariance_eigenvalues, dtype=np.float64).copy())
        track.front_condition_number = float(
            estimate.get("condition_number", float("inf")))
        self._record_position_observation(
            track, class_id, track.last_stamp, track.position,
            track.covariance, FORM_FRONT_MULTI_VIEW,
            track.last_confidence, source="front")

    def _rebuild_front_bearing_clusters(self, semantic_class: str):
        """Reassociate a complete front class pool and update output tracks."""
        pool = getattr(self, "_front_bearing_pool", {}).get(
            semantic_class, ())
        rays = list(pool)
        if len(rays) < 2:
            return
        seeds = self._front_bearing_seed_clusters(rays, semantic_class)
        if not seeds:
            self._counters["front_bearing_noise_observations"] = (
                self._counters.get("front_bearing_noise_observations", 0)
                + len(rays))
            return
        positions = []
        for center, ray_indices in seeds:
            seed_rays = [rays[index] for index in ray_indices
                         if 0 <= int(index) < len(rays)]
            linear = self._front_bearing_linear_initialization(seed_rays)
            positions.append(center if linear is None else linear)
        memberships = self._front_bearing_memberships(rays, positions)
        if memberships.size:
            strongest = np.max(memberships, axis=1)
            ambiguous = int(np.count_nonzero(
                (strongest >= 0.10) & (strongest < 0.90)))
            self._counters["front_bearing_soft_reassignments"] = (
                self._counters.get("front_bearing_soft_reassignments", 0)
                + ambiguous)
        estimates = []
        for _ in range(3):
            estimates = []
            for index, initial in enumerate(positions):
                weights = memberships[:, index]
                estimate = self._optimize_front_bearing_cluster(
                    rays, initial, weights)
                if estimate is not None:
                    estimates.append(estimate)
            if not estimates:
                return
            positions = [estimate["position"] for estimate in estimates]
            memberships = self._front_bearing_memberships(rays, positions)
        if not estimates:
            return

        min_support = max(2, int(getattr(
            self, "front_bearing_min_cluster_rays", 2)))
        accepted = []
        for index, estimate in enumerate(estimates):
            weights = memberships[:, index]
            if (float(estimate["effective_count"]) >= min_support
                    and estimate["inlier_count"] >= 2):
                estimate["memberships"] = weights.copy()
                estimate["cluster_id"] = len(accepted)
                accepted.append(estimate)
        if not accepted:
            return

        # Cluster-to-track matching happens only after the batch geometry is
        # solved.  It keeps instance IDs stable without using a ray-to-track
        # angle gate to decide whether a new observation may enter the pool.
        old_tracks = [track for track in self._front_tracks.values()
                      if track.physical_class_name == semantic_class
                      and track.position is not None]
        used_tracks = set()
        for estimate in sorted(accepted, key=lambda item: item["cluster_id"]):
            distances = sorted(
                (float(np.linalg.norm(
                    estimate["position"] - track.position)), index, track)
                for index, track in enumerate(old_tracks)
                if index not in used_tracks)
            track = None
            if distances and distances[0][0] <= float(getattr(
                    self, "front_bearing_track_match_distance", 2.0)):
                _, index, track = distances[0]
                used_tracks.add(index)
            if track is None:
                track = self._new_front_track(
                    int(rays[0].class_id) if rays else -1)
                if track is None:
                    self._reject_instance_limit(
                        int(rays[0].class_id) if rays else -1)
                    continue
            member_weights = estimate["memberships"]
            member_rays = [ray for ray, weight in zip(rays, member_weights)
                           if float(weight) >= 0.15]
            if len(member_rays) < 2:
                member_rays = [rays[index] for index in np.argsort(
                    member_weights)[-2:]]
            self._update_front_track_from_bearing_cluster(
                track, int(rays[0].class_id), member_rays, estimate)
            estimate["track"] = track
            self._counters["front_multi_view_points"] = (
                self._counters.get("front_multi_view_points", 0) + 1)
            self._counters["front_bearing_optimizations"] = (
                self._counters.get("front_bearing_optimizations", 0) + 1)

        # Keep soft probabilities on every raw ray and remap diagnostic ray
        # records.  A ray may remain unassigned/noise instead of being forced
        # into the nearest instance.
        for ray_index, ray in enumerate(rays):
            ray.cluster_memberships = {
                int(getattr(estimate.get("track"), "instance_id",
                                estimate["cluster_id"])): float(
                    estimate["memberships"][ray_index])
                for estimate in accepted
                if estimate.get("track") is not None
            }
        self._counters["front_bearing_clusters"] = (
            self._counters.get("front_bearing_clusters", 0) + len(accepted))
        self._remap_front_bearing_history(semantic_class, rays, accepted)

    def _rebuild_dirty_front_bearings(self):
        """Batch raw arrivals so the expensive pool solve runs at a fixed rate."""
        dirty_classes = getattr(self, "_front_bearing_dirty_classes", None)
        if not dirty_classes:
            return
        classes = sorted(dirty_classes)
        dirty_classes.clear()
        for semantic_class in classes:
            self._rebuild_front_bearing_clusters(semantic_class)

    def _remap_front_bearing_history(self, semantic_class: str, rays, estimates):
        raw_to_instance = {}
        for estimate in estimates:
            # The track lookup is by the solved position and class.  It is
            # intentionally tolerant of a cluster being left unpublished by
            # the instance cap.
            track = estimate.get("track")
            if track is None:
                continue
            for ray, probability in zip(rays, estimate["memberships"]):
                raw = getattr(ray, "raw_observation", None)
                raw_id = int(getattr(raw, "raw_observation_id", 0))
                if raw_id > 0 and float(probability) >= 0.5:
                    raw_to_instance[raw_id] = int(track.instance_id)
                    raw.target_instance_id = int(track.instance_id)
        for record in getattr(self, "_observation_history", ()):
            if (record.source != "front"
                    or record.physical_class_name != semantic_class):
                continue
            instances = [raw_to_instance.get(int(value))
                         for value in getattr(
                             record, "source_raw_observation_ids", ())]
            instances = [value for value in instances if value is not None]
            if instances:
                record.instance_id = int(instances[0])

    def _add_front_ray(self, class_id: int, ray, confidence: float,
                       stamp: float,
                       raw_observation: FrontPixelObservation | None = None,
                       observation_form: int = FORM_FRONT_MULTI_VIEW):
        if raw_observation is not None:
            return self._add_front_bearing_pool_ray(
                class_id, ray, confidence, stamp, raw_observation,
                observation_form)
        return self._add_front_ray_legacy(
            class_id, ray, confidence, stamp, raw_observation,
            observation_form)

    def _add_front_ray_legacy(self, class_id: int, ray, confidence: float,
                              stamp: float,
                              raw_observation: FrontPixelObservation | None = None,
                              observation_form: int = FORM_FRONT_MULTI_VIEW):
        origin, direction, sigma_angle = ray
        track = self._associate_front_ray(class_id, origin, direction)
        if track is None:
            self._reject_instance_limit(class_id)
            return
        if raw_observation is not None:
            # Raw observations are retained independently of whether this
            # bearing adds a new triangulation baseline.  This preserves all
            # bbox pixels for the later unified reprojection solve without
            # pretending near-parallel rays are independent 3-D factors.
            self._append_front_raw_observations(track, (raw_observation,))
        self._ensure_multiview_pool_key(track)
        # The two rays from one stereo pair are both valid factors even when
        # their parallax is below the temporal-view threshold.  Their weak
        # depth is reflected by the bundle covariance; rejecting one would
        # prevent far targets from being initialized at all.  The 5-degree
        # rule remains for repeated monocular views.
        if (observation_form != FORM_FRONT_STEREO
                and not self._front_ray_has_new_baseline(track, direction)):
            self._counters["front_multi_view_angle_rejected"] += 1
            return
        ray_id = getattr(self, "_next_ray_id", 1)
        self._next_ray_id = ray_id + 1
        track.observed_class_ids.add(int(class_id))
        track.rays.append(RayObservation(
            stamp=stamp,
            origin=origin,
            direction=direction,
            sigma_angle=sigma_angle,
            confidence=confidence,
            ray_id=ray_id,
            raw_observation=raw_observation,
        ))
        while len(track.rays) > self.max_rays:
            track.rays.popleft()
        track.observation_count += 1
        if observation_form == FORM_FRONT_STEREO:
            track.front_stereo_count += 1
        else:
            track.front_multi_view_count += 1
        track.observation_form_mask |= int(observation_form)
        track.last_observation_form = int(observation_form)
        track.last_confidence = max(track.last_confidence, confidence)
        track.last_update_monotonic = time.monotonic()
        track.last_stamp = max(track.last_stamp, stamp)
        self._counters["front_multi_view_rays"] += 1
        ray_record = self._record_ray_observation(
            track, class_id, stamp, origin, direction, confidence,
            source_raw_observation_ids=(
                (int(raw_observation.raw_observation_id),)
                if raw_observation is not None
                and int(getattr(raw_observation, "raw_observation_id", 0)) > 0
                else ()),
            feature_id=(getattr(raw_observation, "feature_id", "bbox_center")
                        if raw_observation is not None else "bbox_center"))

        result = self._solve_rays(track)
        if result is None:
            return
        position, covariance = result
        shared_covariance = self._ray_shared_covariance(position, track.rays)
        self._merge_track_systematic_covariance(track, shared_covariance)
        if raw_observation is not None:
            # Live front detections use one robust ray-bundle estimate for
            # every semantic class.  The model-error floor differs by class,
            # but no repeated intersection is fed back into K-means as a new
            # XYZ observation.
            semantic_class = self._semantic_class(class_id)
            model_covariance = (
                self.front_gate_model_covariance
                if semantic_class == "gate"
                else self.front_ray_model_covariance)
            covariance = self._apply_covariance_floor(
                covariance, model_covariance + shared_covariance)
            old_position = track.position.copy() if track.position is not None else None
            old_covariance = track.covariance.copy() if track.covariance is not None else None
            max_displacement = (self.front_gate_reassociation_distance
                                if semantic_class == "gate" else
                                max(0.25, self.front_gate_reassociation_distance))
            if (old_position is not None and old_covariance is not None
                    and (float(np.trace(covariance))
                         > max(float(np.trace(old_covariance)) * 1.5,
                               float(np.trace(old_covariance)) + 0.05)
                         or float(np.linalg.norm(
                             position[:2] - old_position[:2])) >
                         max_displacement)):
                position = old_position
                covariance = old_covariance
                counter = ("front_gate_bundle_update_rejected"
                           if semantic_class == "gate"
                           else "front_bundle_update_rejected")
                self._counters[counter] = self._counters.get(counter, 0) + 1
            track.position = np.asarray(position, dtype=np.float64).copy()
            track.covariance = _regularize_covariance(covariance)
            track.front_filter_position = track.position.copy()
            track.front_filter_covariance = track.covariance.copy()
            ray_ids = tuple(
                int(item.ray_id) for item in track.rays if int(item.ray_id) > 0)
            ray_record.instance_id = track.instance_id
            self._record_position_observation(
                track, class_id, stamp, track.position, track.covariance,
                int(observation_form), confidence)
            counter = ("front_gate_bundle_updates"
                       if semantic_class == "gate"
                       else "front_bundle_updates")
            self._counters[counter] = self._counters.get(counter, 0) + 1
            if observation_form == FORM_FRONT_MULTI_VIEW:
                self._counters["front_multi_view_points"] += 1
            return
        # A ray hypothesis becomes a normal front-pool hypothesis as soon as
        # two sufficiently different views define a 3-D point.  The pool
        # entry is a replaceable slot keyed by this ray track: every solve
        # reuses the same retained rays and must not be counted as another
        # independent measurement.
        track.position = position.copy()
        track.covariance = self._apply_covariance_floor(
            covariance, track.systematic_covariance)
        ray_ids = tuple(
            int(item.ray_id) for item in track.rays if int(item.ray_id) > 0)
        raw_observations = tuple(
            getattr(track, "front_pixel_observations", ()))
        front_track = self._handle_front_position_measurement(
            class_id, position, covariance,
            None,
            FORM_FRONT_MULTI_VIEW, confidence,
            {"multi_view": True, "shared_covariance": shared_covariance},
            stamp=stamp,
            pool_key=track.multi_view_pool_key, ray_ids=ray_ids,
            raw_observations=raw_observations)
        if front_track is not None:
            ray_record.instance_id = front_track.instance_id
            self._counters["front_multi_view_points"] += 1

    def _ensure_multiview_pool_key(self, track: TargetTrack) -> int:
        """Return the stable pool slot used by one front ray hypothesis."""
        key = int(getattr(track, "multi_view_pool_key", 0))
        if key > 0:
            return key
        key = int(getattr(self, "_next_multiview_pool_key", 1))
        self._next_multiview_pool_key = key + 1
        track.multi_view_pool_key = key
        return key

    def _front_ray_has_new_baseline(self, track: TargetTrack,
                                    direction: np.ndarray) -> bool:
        """Keep only front bearings that add a distinct multi-view baseline.

        The first bearing creates a candidate track.  Every later bearing must
        be more than ``front_multi_view_min_angle_deg`` away from its closest
        retained bearing, so slow frame-by-frame motion cannot fill the queue
        with near-parallel copies of the same constraint.
        """
        if not track.rays:
            return True
        direction = np.asarray(direction, dtype=np.float64).reshape(3)
        direction /= max(float(np.linalg.norm(direction)), 1e-12)
        nearest_angle = min(
            math.acos(np.clip(
                float(np.dot(direction, ray.direction)), -1.0, 1.0))
            for ray in track.rays)
        return nearest_angle > self.min_ray_angle_rad

    def _associate_front_ray(self, class_id: int, origin: np.ndarray,
                             direction: np.ndarray) -> TargetTrack | None:
        candidates = []
        semantic_class = self._semantic_class(class_id)
        association_angle = (
            self.gate_ray_assoc_angle_rad
            if semantic_class == "gate" else self.ray_assoc_angle_rad)
        for track in self._front_tracks.values():
            if self._semantic_class(track.class_id) != semantic_class:
                continue
            if track.position is not None:
                vector = track.position - origin
                distance = float(np.linalg.norm(vector))
                if distance < 1e-6:
                    continue
                angle = math.acos(np.clip(
                    float(np.dot(direction, vector / distance)), -1.0, 1.0))
                line_error = float(np.linalg.norm(
                    np.cross(track.position - origin, direction)))
                if angle <= association_angle:
                    candidates.append((angle + line_error / max(
                        distance, 0.1), track))
            elif track.rays:
                best_angle = min(
                    math.acos(np.clip(
                        float(np.dot(direction, ray.direction)), -1.0, 1.0))
                    for ray in track.rays)
                if best_angle <= association_angle:
                    candidates.append((best_angle + 0.5, track))
        if candidates:
            return min(candidates, key=lambda item: item[0])[1]
        return self._new_front_track(class_id)

    def _front_ray_candidate_expired(self, track: TargetTrack,
                                      reference_stamp: float = 0.0) -> bool:
        """Return whether a positionless ray candidate may be released."""
        timeout = max(0.0, float(getattr(self, "track_timeout", 2.0)))
        if (reference_stamp > 0.0 and track.last_stamp > 0.0):
            return reference_stamp - track.last_stamp > timeout
        return time.monotonic() - track.last_update_monotonic > timeout

    # Kept as a small compatibility alias for offline callers of the old
    # helper.  It is intentionally front-only; it no longer touches down
    # tracks.
    def _associate_ray(self, class_id: int, origin: np.ndarray,
                       direction: np.ndarray) -> TargetTrack | None:
        return self._associate_front_ray(class_id, origin, direction)

    def _solve_rays(self, track: TargetTrack):
        if len(track.rays) < 2:
            return None
        rays = list(track.rays)
        directions = [ray.direction for ray in rays]
        max_angle = 0.0
        for first in range(len(directions)):
            for second in range(first + 1, len(directions)):
                angle = math.acos(np.clip(
                    float(np.dot(directions[first], directions[second])),
                    -1.0, 1.0))
                max_angle = max(max_angle, angle)
        if max_angle < self.min_ray_angle_rad:
            return None

        # Robust IRLS on the perpendicular distance to every bearing.  The
        # old implementation first discarded rays using an absolute 1 m line
        # threshold and then treated the remaining set as equally valid.  A
        # changing gate feature can be a biased but still useful observation;
        # Huber weights keep it as a reference without allowing it to move the
        # solution as much as a consistent ray.
        position = None
        normal_matrix = None
        for iteration in range(8):
            normal_matrix = np.zeros((3, 3), dtype=np.float64)
            rhs = np.zeros(3, dtype=np.float64)
            if position is None:
                scale = 1.0
            else:
                scale = 0.0
            for ray in rays:
                projector = np.eye(3) - np.outer(ray.direction, ray.direction)
                if position is not None:
                    scale = max(scale, float(np.linalg.norm(
                        position - ray.origin)))
            scale = max(scale, 0.1)
            for ray in rays:
                projector = np.eye(3) - np.outer(ray.direction, ray.direction)
                sigma_perpendicular = max(
                    0.01, scale * float(ray.sigma_angle))
                if position is None:
                    robust_weight = 1.0
                else:
                    residual = float(np.linalg.norm(
                        projector @ (position - ray.origin)))
                    normalized = residual / sigma_perpendicular
                    delta = max(0.5, float(getattr(
                        self, "huber_delta", 2.5)))
                    robust_weight = 1.0 if normalized <= delta else (
                        delta / max(normalized, 1e-9))
                weight = max(float(ray.confidence), 0.05) \
                    * robust_weight / sigma_perpendicular**2
                normal_matrix += weight * projector
                rhs += weight * projector @ ray.origin
            if np.linalg.matrix_rank(normal_matrix, tol=1e-8) < 3:
                return None
            next_position = _safe_inverse(normal_matrix) @ rhs
            if (position is not None and float(np.linalg.norm(
                    next_position - position)) < 1e-5):
                position = next_position
                break
            position = next_position

        if position is None or not np.all(np.isfinite(position)):
            return None
        # The least-squares solve uses unoriented line projectors.  Two
        # bearing lines can therefore intersect behind both cameras; that is
        # not a valid multi-view target and was the source of rearward ball
        # estimates after a rejected front stereo pair.
        ray_depths = [
            float(np.dot(position - ray.origin, ray.direction))
            for ray in rays
        ]
        if (not ray_depths
                or any(not np.isfinite(depth) or depth <= 1e-6
                       for depth in ray_depths)):
            return None
        covariance = _regularize_covariance(
            _safe_inverse(normal_matrix) * self.front_multi_scale**2)
        return position, covariance

    def _fuse_ray(self, track: TargetTrack, origin: np.ndarray,
                   direction: np.ndarray, sigma_angle: float):
        vector = track.position - origin
        range_m = max(float(np.linalg.norm(vector)), 0.1)
        projector = np.eye(3) - np.outer(direction, direction)
        sigma = max(0.01, range_m * sigma_angle)
        measurement_covariance = np.eye(3) * sigma**2
        z = projector @ origin
        h = projector
        self._linear_update(
            track, z, h, measurement_covariance,
            FORM_FRONT_MULTI_VIEW, track.last_confidence)

    def _associate_position(self, class_id: int, position: np.ndarray,
                            covariance: np.ndarray,
                            origin: np.ndarray | None,
                            form: int | None = None):
        candidates = []
        close_guide_tracks = []
        semantic_class = self._semantic_class(class_id)
        for track in self._tracks.values():
            if self._semantic_class(track.class_id) != semantic_class:
                continue
            if track.position is not None:
                # All competition targets are static on the scene plane.  A
                # bad stereo disparity can move a point substantially in D
                # while its N/E projection remains correct.  Use the N/E
                # innovation for identity association; D is still retained
                # and filtered by the per-instance 3-D Kalman window below.
                horizontal_distance = float(
                    np.linalg.norm(position[:2] - track.position[:2]))
                if (semantic_class == "guide_line"
                        and horizontal_distance < self.guide_line_min_spacing):
                    close_guide_tracks.append((horizontal_distance, track))
                innovation = position[:2] - track.position[:2]
                innovation_covariance = _regularize_covariance(
                    track.covariance[:2, :2] + covariance[:2, :2])
                distance = float(innovation.T @ _safe_inverse(
                    innovation_covariance) @ innovation)
                if distance <= self.position_gate_chi2:
                    candidates.append((distance, track))
            elif origin is not None and track.rays:
                line_errors = [
                    float(np.linalg.norm(np.cross(
                        position - ray.origin, ray.direction)))
                    for ray in track.rays
                ]
                error = min(line_errors)
                if error <= self.ray_assoc_distance:
                    candidates.append((error, track))
        if candidates:
            distance, track = min(candidates, key=lambda item: item[0])
            # A direct down observation is the reliable anchor in this
            # application.  Do not let a previously accepted, front-only
            # estimate pull it away when the two disagree materially.
            if (
                form == FORM_DOWN_DIRECT
                and track.down_direct_count == 0
                and track.position is not None
                and distance > self.down_direct_reanchor_chi2
            ):
                self._reanchor_from_down(track, class_id, position, covariance)
            return track

        # The rule-defined guide lines are at least 0.5 m apart.  A new point
        # inside that exclusion radius cannot be a new guide-line instance,
        # even if its covariance is temporarily very small and the Mahalanobis
        # gate rejects the noisy update.  Returning the nearest track lets the
        # per-instance Kalman/innovation gate decide whether to accept it,
        # while preventing duplicate IDs closer than the rule distance.
        if semantic_class == "guide_line" and close_guide_tracks:
            return min(close_guide_tracks, key=lambda item: item[0])[1]

        track = self._new_track(class_id)
        if track is not None or form != FORM_DOWN_DIRECT:
            return track

        # For a unique class (or a full gate/guide-line namespace), a distant
        # down observation used to be dropped because front-only tracks had
        # already consumed every instance slot.  Replace the least credible
        # matching front-only track instead; a track already supported by a
        # down observation is never displaced by this fallback.
        replacement = self._front_only_replacement(class_id, position)
        if replacement is not None:
            self._reanchor_from_down(
                replacement, class_id, position, covariance)
            return replacement
        return None

    def _front_only_replacement(self, class_id: int,
                                position: np.ndarray) -> TargetTrack | None:
        semantic_class = self._semantic_class(class_id)
        candidates = [
            track for track in self._tracks.values()
            if self._semantic_class(track.class_id) == semantic_class
            and track.down_direct_count == 0
        ]
        if not candidates and semantic_class == "gate":
            # A gate's depth covariance can temporarily become large.  Do
            # not turn every later observation into a new instance merely
            # because the full 3-D Mahalanobis gate rejected it.  The scene
            # gate spacing is greater than this horizontal recovery radius.
            nearby = []
            for track in self._front_tracks.values():
                if (track.physical_class_name == semantic_class
                        and track.position is not None):
                    distance = float(np.linalg.norm(
                        measurement[:2] - track.position[:2]))
                    if distance <= self.front_gate_reassociation_distance:
                        nearby.append((distance, track))
            if nearby:
                candidates = [(0.0, min(nearby, key=lambda item: item[0])[1])]
        if not candidates:
            return None

        def score(track: TargetTrack):
            distance = (
                float(np.linalg.norm(position - track.position))
                if track.position is not None else float("inf"))
            # Prefer replacing an unlocalized ray-only hypothesis; otherwise
            # use spatial proximity before evidence count/confidence.
            return (
                track.position is not None,
                distance,
                track.observation_count,
                track.last_confidence,
            )

        return min(candidates, key=score)

    def _reanchor_from_down(self, track: TargetTrack, class_id: int,
                            position: np.ndarray, covariance: np.ndarray):
        """Replace an inconsistent front-only hypothesis with a down anchor."""
        track.class_id = int(class_id)
        track.observed_class_ids = {int(class_id)}
        track.position = np.asarray(position, dtype=np.float64).copy()
        track.covariance = _regularize_covariance(covariance)
        track.rays.clear()
        track.observation_count = 0
        track.front_stereo_count = 0
        track.front_multi_view_count = 0
        track.down_direct_count = 0
        track.observation_form_mask = 0
        track.last_observation_form = 0
        track.last_confidence = 0.0
        track.last_stamp = 0.0
        track.last_update_monotonic = time.monotonic()
        track.down_observations.clear()
        track.down_filter_position = None
        track.down_filter_covariance = None
        track.systematic_covariance = None
        track.front_pixel_observations.clear()

        # The discarded front-only hypothesis must not remain visible as an
        # accepted factor for the new down-anchored physical instance.
        if hasattr(self, "_observation_history"):
            self._observation_history = deque(
                (
                    record for record in self._observation_history
                    if not (
                        record.physical_class_name == track.physical_class_name
                        and record.instance_id == track.instance_id
                    )
                ),
                maxlen=getattr(self, "observation_history_size", None),
            )
        self._counters["down_direct_reanchored"] = (
            self._counters.get("down_direct_reanchored", 0) + 1)

    def _accept_down_direct_in_window(self, track: TargetTrack,
                                      position: np.ndarray,
                                      covariance: np.ndarray,
                                      stamp: float,
                                      confidence: float) -> bool:
        """Gate a direct down point against its 50-sample Kalman window.

        The track EKF receives each accepted raw point once.  This separate
        down-only filter is deliberately a finite sliding window: it protects
        the high-trust down anchor from an occasional bad down detection,
        without double-counting the same 50 measurements in the main fusion
        state.
        """
        if (track.down_filter_position is not None
                and track.down_filter_covariance is not None):
            # Identity is decided in the horizontal scene plane.  The depth
            # component remains part of the 3-D Kalman state, but a disparity
            # jump must not create a second instance or reject all later good
            # observations of the same object.
            innovation = position[:2] - track.down_filter_position[:2]
            innovation_covariance = _regularize_covariance(
                track.down_filter_covariance[:2, :2] + covariance[:2, :2])
            mahalanobis = float(innovation.T @ _safe_inverse(
                innovation_covariance) @ innovation)
            if mahalanobis > self.down_direct_queue_gate_chi2:
                self._counters["down_direct_queue_rejected"] = (
                    self._counters.get("down_direct_queue_rejected", 0) + 1)
                return False

        track.down_observations.append(DownDirectObservation(
            stamp=float(stamp),
            position=np.asarray(position, dtype=np.float64).copy(),
            covariance=_regularize_covariance(covariance),
            confidence=float(confidence),
        ))
        while len(track.down_observations) > self.down_direct_queue_size:
            track.down_observations.popleft()
        (track.down_filter_position,
         track.down_filter_covariance) = self._fit_down_direct_window(
             track.down_observations)
        window_shared_covariance = self._covariance_diagonal_envelope(
            getattr(observation, "shared_covariance", None)
            for observation in track.down_observations)
        track.systematic_covariance = self._merge_track_systematic_covariance(
            track, window_shared_covariance)
        track.down_filter_covariance = self._apply_covariance_floor(
            track.down_filter_covariance, track.systematic_covariance)
        return True

    def _down_duplicate_merge_radius(self, semantic_class: str) -> float:
        if semantic_class == "guide_line":
            # The rule says different guide lines are at least 0.5 m apart.
            return self.guide_line_min_spacing
        return self.down_duplicate_merge_distance

    def _merge_close_down_tracks(self):
        """Collapse old down-only duplicate hypotheses in the N/E plane.

        This is also run from ``_publish`` so a node that accumulated bad
        early stereo points can repair its public topic without waiting for a
        restart.  The lowest instance ID in a duplicate group is retained;
        the state with the strongest down evidence is copied into it.
        """
        while True:
            tracks = [
                track for track in self._tracks.values()
                if track.position is not None and track.down_direct_count > 0
            ]
            merged = False
            for index, first in enumerate(tracks):
                semantic_class = first.physical_class_name
                radius = self._down_duplicate_merge_radius(semantic_class)
                for second in tracks[index + 1:]:
                    if second.physical_class_name != semantic_class:
                        continue
                    horizontal_distance = float(np.linalg.norm(
                        first.position[:2] - second.position[:2]))
                    if horizontal_distance >= radius:
                        continue
                    self._merge_down_track_pair(first, second)
                    merged = True
                    break
                if merged:
                    break
            if not merged:
                return

    def _merge_down_track_pair(self, first: TargetTrack,
                               second: TargetTrack):
        """Merge two same-class down hypotheses while preserving the low ID."""
        if first.instance_id <= second.instance_id:
            keeper, loser = first, second
        else:
            keeper, loser = second, first

        candidates = [keeper, loser]

        def evidence_score(track: TargetTrack):
            trace = (float(np.trace(track.covariance))
                     if track.covariance is not None else float("inf"))
            return (track.down_direct_count, len(track.down_observations),
                    -trace, track.last_stamp)

        source = max(candidates, key=evidence_score)
        if source.position is not None:
            keeper.position = source.position.copy()
        if source.covariance is not None:
            keeper.covariance = source.covariance.copy()
        keeper.down_observations = deque(source.down_observations)
        if source.down_filter_position is not None:
            keeper.down_filter_position = source.down_filter_position.copy()
        if source.down_filter_covariance is not None:
            keeper.down_filter_covariance = source.down_filter_covariance.copy()
        if source.systematic_covariance is not None:
            keeper.systematic_covariance = source.systematic_covariance.copy()
        keeper.class_id = source.class_id
        keeper.observed_class_ids.update(loser.observed_class_ids)
        keeper.observation_count += loser.observation_count
        keeper.front_stereo_count += loser.front_stereo_count
        keeper.front_multi_view_count += loser.front_multi_view_count
        keeper.down_direct_count += loser.down_direct_count
        keeper.observation_form_mask |= loser.observation_form_mask
        if loser.last_stamp > keeper.last_stamp:
            keeper.last_stamp = loser.last_stamp
            keeper.last_observation_form = loser.last_observation_form
        keeper.last_confidence = max(
            keeper.last_confidence, loser.last_confidence)
        keeper.last_update_monotonic = max(
            keeper.last_update_monotonic, loser.last_update_monotonic)

        for record in getattr(self, "_observation_history", ()):
            if (record.physical_class_name == keeper.physical_class_name
                    and record.instance_id == loser.instance_id):
                record.instance_id = keeper.instance_id

        self._tracks.pop((loser.physical_class_name, loser.instance_id), None)
        self._counters["down_duplicate_merged"] = (
            self._counters.get("down_duplicate_merged", 0) + 1)

    @staticmethod
    def _fit_down_direct_window(
            observations: deque[DownDirectObservation]) -> tuple[np.ndarray, np.ndarray]:
        """Run an identity-measurement Kalman filter over the retained window."""
        first = observations[0]
        position = first.position.copy()
        covariance = first.covariance.copy()
        identity = np.eye(3)
        for observation in list(observations)[1:]:
            measurement_covariance = observation.covariance
            innovation = observation.position - position
            innovation_covariance = _regularize_covariance(
                covariance + measurement_covariance)
            gain = covariance @ _safe_inverse(innovation_covariance)
            prior_covariance = covariance.copy()
            position = position + gain @ innovation
            covariance = _regularize_covariance(
                (identity - gain) @ prior_covariance @ (identity - gain).T
                + gain @ measurement_covariance @ gain.T)
        covariance = ObjectLocalizer._apply_covariance_floor(
            covariance,
            ObjectLocalizer._covariance_diagonal_envelope(
                getattr(observation, "shared_covariance", None)
                for observation in observations),
        )
        return position, covariance

    def _handle_position_measurement(self, class_id: int,
                                     position: np.ndarray,
                                     covariance: np.ndarray,
                                     pose: PoseAt,
                                     form: int,
                                     confidence: float,
                                     quality: dict):
        if not np.all(np.isfinite(position)):
            return False
        distance = float(np.linalg.norm(position - pose.position))
        if distance < self.min_depth or distance > self.max_depth:
            return False
        covariance = _regularize_covariance(covariance)
        origin = pose.position
        if form == FORM_DOWN_DIRECT:
            observation = self._add_down_observation_pool(
                class_id, position, covariance, pose.stamp, confidence,
                shared_covariance=quality.get("shared_covariance"))
            track = self._recluster_down_class(class_id, observation)
            if track is None:
                # A valid pooled observation must not be discarded because a
                # previous false hypothesis consumed the instance limit.
                self._counters["down_direct_rejected"] += 1
                return False
            self._last_detection_stamp = max(
                self._last_detection_stamp, pose.stamp)
            self._record_position_observation(
                track, class_id, pose.stamp, position, covariance, form,
                confidence)
            return True
        track = self._associate_position(
            class_id, position, covariance, origin, form)
        if track is None:
            self._reject_instance_limit(class_id)
            return False
        if form == FORM_DOWN_DIRECT and not self._accept_down_direct_in_window(
                track, position, covariance, pose.stamp, confidence):
            self._counters["down_direct_rejected"] += 1
            return False
        if form == FORM_DOWN_DIRECT:
            # The retained 50-sample window is the active static-target
            # estimate.  Do not feed the same point once into the window and
            # once into an unlimited all-history filter.
            track.position = track.down_filter_position.copy()
            track.covariance = track.down_filter_covariance.copy()
            accepted = True
        else:
            accepted = self._linear_update(
                track, position, np.eye(3), covariance, form, confidence,
                shared_covariance=quality.get("shared_covariance"))
        if not accepted:
            self._counters["association_rejected"] += 1
            return False
        track.observed_class_ids.add(int(class_id))
        track.observation_count += 1
        if form == FORM_FRONT_STEREO:
            track.front_stereo_count += 1
        elif form == FORM_DOWN_DIRECT:
            track.down_direct_count += 1
        track.observation_form_mask |= form
        track.last_observation_form = form
        track.last_confidence = max(track.last_confidence, confidence)
        track.last_update_monotonic = time.monotonic()
        track.last_stamp = max(track.last_stamp, pose.stamp)
        self._last_detection_stamp = max(self._last_detection_stamp, pose.stamp)
        self._record_position_observation(
            track, class_id, pose.stamp, position, covariance, form, confidence)
        return True

    def _handle_front_position_measurement(
            self, class_id: int, position: np.ndarray,
            covariance: np.ndarray, pose: PoseAt | None, form: int,
            confidence: float, quality: dict,
            stamp: float | None = None, pool_key: int = 0,
            ray_ids: tuple[int, ...] = (),
            raw_observations: tuple[FrontPixelObservation, ...] = ()
    ) -> TargetTrack | None:
        """Put one valid front 3-D factor through pool/K-means/Kalman.

        ``pose`` is present for a front stereo pair.  A multi-view point is
        already expressed in world coordinates and only needs its ray stamp,
        so that path passes ``pose=None`` and supplies ``stamp`` explicitly.
        """
        if not np.all(np.isfinite(position)):
            return None
        if pose is not None:
            distance = float(np.linalg.norm(position - pose.position))
            observation_stamp = float(pose.stamp)
            # A direct stereo point must be in front of the current camera
            # pose.  Multi-view points are already world-frame intersections;
            # they deliberately have no robot-distance gate here.
            if not np.isfinite(distance) or distance < self.min_depth:
                return None
        else:
            observation_stamp = float(stamp or 0.0)
        covariance = _regularize_covariance(covariance)
        semantic_class = self._semantic_class(class_id)
        pool = self._front_observation_pool.setdefault(
            semantic_class,
            deque(maxlen=self.front_observation_pool_size),
        )
        replacing = bool(
            pool_key and any(
                int(getattr(item, "pool_key", 0)) == int(pool_key)
                for item in pool
            )
        )
        observation = self._add_front_observation_pool(
            class_id, position, covariance, observation_stamp, confidence, form,
            pool_key=pool_key, ray_ids=ray_ids,
            shared_covariance=quality.get("shared_covariance"),
            raw_observations=raw_observations)
        # Pool admission and instance creation are intentionally separate.
        # A gate needs several observations before a new instance is created,
        # but the earlier observations must still be visible to diagnostics.
        if not replacing:
            self._counters["front_pool_observations"] += 1
        track = self._recluster_front_class(class_id, observation)
        if track is None:
            self._update_or_record_front_position_observation(
                observation, None, class_id, observation_stamp, position,
                covariance, form, confidence)
            return None
        self._last_detection_stamp = max(
            self._last_detection_stamp, observation_stamp)
        self._update_or_record_front_position_observation(
            observation, track, class_id, observation_stamp, position,
            covariance, form, confidence)
        return track

    def _update_or_record_front_position_observation(
            self, observation: FrontPositionObservation,
            track: TargetTrack | None, class_id: int, stamp: float,
            position: np.ndarray, covariance: np.ndarray, form: int,
            confidence: float):
        """Keep one diagnostic record for one replaceable multi-view slot."""
        raw_observation_ids = tuple(sorted({
            int(getattr(item, "raw_observation_id", 0))
            for item in getattr(observation, "raw_observations", ())
            if int(getattr(item, "raw_observation_id", 0)) > 0
        }))
        feature_ids = {
            str(getattr(item, "feature_id", ""))
            for item in getattr(observation, "raw_observations", ())
            if str(getattr(item, "feature_id", ""))
        }
        feature_id = (next(iter(feature_ids)) if len(feature_ids) == 1
                      else "mixed" if feature_ids else "")
        if (int(form) == FORM_FRONT_MULTI_VIEW
                and int(getattr(observation, "observation_record_id", 0)) > 0):
            record_id = int(observation.observation_record_id)
            for record in self._observation_history:
                if int(record.observation_id) != record_id:
                    continue
                record.stamp = float(stamp)
                record.position = np.asarray(position, dtype=np.float64).copy()
                record.covariance = np.asarray(
                    covariance, dtype=np.float64).copy()
                record.confidence = float(confidence)
                record.source_raw_observation_ids = raw_observation_ids
                record.feature_id = feature_id
                if track is not None:
                    record.instance_id = int(track.instance_id)
                    record.physical_class_name = track.physical_class_name
                return record

        record = self._record_position_observation(
            track, class_id, stamp, position, covariance, form, confidence,
            source="front", source_raw_observation_ids=raw_observation_ids,
            feature_id=feature_id)
        if int(form) == FORM_FRONT_MULTI_VIEW:
            observation.observation_record_id = int(record.observation_id)
        return record

    def _add_front_observation_pool(
            self, class_id: int, position: np.ndarray,
            covariance: np.ndarray, stamp: float, confidence: float,
            form: int, pool_key: int = 0,
            ray_ids: tuple[int, ...] = (),
            shared_covariance: np.ndarray | None = None,
            raw_observations: tuple[FrontPixelObservation, ...] = ()
    ) -> FrontPositionObservation:
        semantic_class = self._semantic_class(class_id)
        pool = self._front_observation_pool.setdefault(
            semantic_class,
            deque(maxlen=self.front_observation_pool_size),
        )
        if pool_key:
            for observation in pool:
                if int(getattr(observation, "pool_key", 0)) != int(pool_key):
                    continue
                observation.stamp = float(stamp)
                observation.position = np.asarray(
                    position, dtype=np.float64).copy()
                observation.covariance = _regularize_covariance(covariance)
                observation.confidence = float(confidence)
                observation.form = int(form)
                observation.class_id = int(class_id)
                observation.ray_ids = tuple(int(value) for value in ray_ids)
                observation.shared_covariance = (
                    None if shared_covariance is None else
                    _regularize_covariance(shared_covariance))
                observation.raw_observations = tuple(raw_observations)
                return observation
        observation = FrontPositionObservation(
            stamp=float(stamp),
            position=np.asarray(position, dtype=np.float64).copy(),
            covariance=_regularize_covariance(covariance),
            confidence=float(confidence),
            form=int(form),
            class_id=int(class_id),
            pool_key=int(pool_key),
            ray_ids=tuple(int(value) for value in ray_ids),
            shared_covariance=(
                None if shared_covariance is None else
                _regularize_covariance(shared_covariance)),
            raw_observations=tuple(raw_observations),
        )
        pool.append(observation)
        return observation

    def _front_duplicate_merge_radius(self, semantic_class: str) -> float:
        if semantic_class == "guide_line":
            return self.guide_line_min_spacing
        if semantic_class == "gate":
            return self.front_gate_duplicate_merge_distance
        return self.front_duplicate_merge_distance

    def _merge_close_front_cluster_centers(
            self, observations, assignments, centers, semantic_class: str):
        """Merge K-means centers that cannot be separate front instances."""
        radius = self._front_duplicate_merge_radius(semantic_class)
        while len(centers) > 1:
            distance_matrix = np.linalg.norm(
                centers[:, None, :] - centers[None, :, :], axis=2)
            distance_matrix += np.eye(len(centers)) * 1e9
            first, second = np.unravel_index(
                int(np.argmin(distance_matrix)), distance_matrix.shape)
            if distance_matrix[first, second] >= radius:
                break
            centers = np.delete(centers, second, axis=0)
            assignments = np.argmin(
                np.sum((np.asarray([observation.position[:2]
                                     for observation in observations])[:, None, :]
                        - centers[None, :, :])**2, axis=2), axis=1)
        return assignments, centers

    def _front_track_first_update(
            self, current_observation: FrontPositionObservation,
            class_id: int) -> TargetTrack | None:
        """Associate a new front point to a predicted track before pooling.

        The rolling pool remains available for bootstrap and recovery, but a
        positioned track gets first refusal using a full 3-D Mahalanobis gate.
        This prevents a noisy K-means split from changing an established
        identity merely because the N/E projection happens to be closer to a
        different cluster.
        """
        semantic_class = self._semantic_class(class_id)
        candidates = []
        measurement = np.asarray(current_observation.position,
                                 dtype=np.float64)
        measurement_covariance = np.asarray(
            current_observation.covariance, dtype=np.float64)
        for track in self._front_tracks.values():
            if (track.physical_class_name != semantic_class
                    or track.position is None or track.covariance is None):
                continue
            innovation = measurement - track.position
            innovation_covariance = _regularize_covariance(
                track.covariance + measurement_covariance)
            distance = float(innovation.T @ _safe_inverse(
                innovation_covariance) @ innovation)
            if np.isfinite(distance) and distance <= self.position_gate_chi2:
                candidates.append((distance, track))
        if not candidates:
            return None

        _, track = min(candidates, key=lambda item: item[0])
        members = list(getattr(track, "front_observations", ()))
        pool_key = int(getattr(current_observation, "pool_key", 0))
        if pool_key > 0:
            members = [
                member for member in members
                if int(getattr(member, "pool_key", 0)) != pool_key
            ]
        members.append(current_observation)
        self._update_track_from_front_cluster(track, class_id, members)
        self._counters["front_track_first_associations"] = (
            self._counters.get("front_track_first_associations", 0) + 1)
        return track

    def _recluster_front_class(
            self, class_id: int,
            current_observation: FrontPositionObservation) -> TargetTrack | None:
        """Cluster the complete front pool and update one track per cluster."""
        semantic_class = self._semantic_class(class_id)
        observations = list(self._front_observation_pool.get(semantic_class, ()))
        if not observations:
            return None
        track = self._front_track_first_update(current_observation, class_id)
        if track is not None:
            return track
        cluster_count = min(self._instance_limit(class_id), len(observations))
        assignments, centers = self._kmeans_horizontal(
            observations, cluster_count)
        assignments, centers = self._merge_close_front_cluster_centers(
            observations, assignments, centers, semantic_class)

        existing_tracks = [
            track for track in self._front_tracks.values()
            if track.physical_class_name == semantic_class
            and track.position is not None
        ]
        cluster_tracks = self._cluster_to_existing_tracks(
            centers, existing_tracks, observations, assignments)
        # Do not delete a positioned front track merely because this K-means
        # pass did not select its cluster center.  The pool is rolling and
        # noisy observations can temporarily merge/split centers; deleting
        # here made an unchanged target disappear and caused instance IDs to
        # jump (0 -> 2 -> 4 in the recent session).  The track remains in the
        # world map and will become STALE through the normal age policy if it
        # is no longer observed.

        # A ray-only hypothesis is promoted to a positioned track before this
        # method is called by _add_front_ray.  Keep active ray-only hypotheses
        # because another position observation must not destroy a pending
        # multi-view intersection.  Release only empty/expired candidates so
        # they cannot consume an instance slot forever.
        for key, track in list(self._front_tracks.items()):
            if (track.physical_class_name == semantic_class
                    and track.position is None
                    and (not track.rays
                         or self._front_ray_candidate_expired(
                             track, current_observation.stamp))):
                self._front_tracks.pop(key, None)

        min_support = (
            getattr(self, "front_gate_min_cluster_observations", 1)
            if semantic_class == "gate" else 1
        )
        for cluster_index in range(len(centers)):
            if cluster_index not in cluster_tracks:
                member_indices = [
                    index for index, assignment in enumerate(assignments)
                    if int(assignment) == cluster_index
                ]
                if len(member_indices) < min_support:
                    self._counters["front_cluster_support_rejected"] = (
                        self._counters.get("front_cluster_support_rejected", 0)
                        + 1)
                    continue
                track = self._new_front_track(class_id)
                if track is None:
                    continue
                cluster_tracks[cluster_index] = track

        for cluster_index, track in cluster_tracks.items():
            members = [
                observations[index] for index, assignment
                in enumerate(assignments) if int(assignment) == cluster_index
            ]
            if members:
                self._update_track_from_front_cluster(
                    track, class_id, members)

        self._remap_front_observation_history(
            semantic_class, observations, assignments, cluster_tracks)
        current_index = min(
            range(len(observations)),
            key=lambda index: abs(
                observations[index].stamp - current_observation.stamp)
                + float(np.linalg.norm(
                    observations[index].position[:2]
                    - current_observation.position[:2])) * 1e-6,
        )
        self._counters["front_cluster_updates"] += 1
        return cluster_tracks.get(int(assignments[current_index]))

    def _update_track_from_front_cluster(
            self, track: TargetTrack, class_id: int,
            observations: list[FrontPositionObservation]):
        """Fit the newest 50 members of one horizontal front cluster."""
        old_state = None
        if (track.position is not None
                and track.covariance is not None):
            old_state = {
                "position": track.position.copy(),
                "covariance": track.covariance.copy(),
                "front_filter_position": (
                    None if track.front_filter_position is None
                    else track.front_filter_position.copy()),
                "front_filter_covariance": (
                    None if track.front_filter_covariance is None
                    else track.front_filter_covariance.copy()),
                "systematic_covariance": (
                    None if track.systematic_covariance is None
                    else track.systematic_covariance.copy()),
                "front_observations": deque(track.front_observations),
                "front_pixel_observations": deque(
                    track.front_pixel_observations),
                "observation_count": track.observation_count,
                "front_stereo_count": track.front_stereo_count,
                "front_multi_view_count": track.front_multi_view_count,
                "observation_form_mask": track.observation_form_mask,
                "last_observation_form": track.last_observation_form,
                "last_confidence": track.last_confidence,
                "last_stamp": track.last_stamp,
                "last_update_monotonic": track.last_update_monotonic,
            }
        ordered = sorted(observations, key=lambda observation: observation.stamp)
        window = ordered[-self.front_direct_queue_size:]
        track.class_id = int(class_id)
        track.physical_class_name = self._semantic_class(class_id)
        track.observed_class_ids = {
            observation.class_id for observation in observations
            if observation.class_id >= 0
        } or {int(class_id)}
        raw_observations = [
            raw for observation in observations
            for raw in getattr(observation, "raw_observations", ())
        ]
        self._append_front_raw_observations(track, raw_observations)
        track.front_observations = deque(window)
        (track.front_filter_position,
         track.front_filter_covariance) = self._fit_front_window(
             track.front_observations)
        window_shared_covariance = self._covariance_diagonal_envelope(
            getattr(observation, "shared_covariance", None)
            for observation in track.front_observations)
        track.systematic_covariance = self._merge_track_systematic_covariance(
            track, window_shared_covariance)
        track.front_filter_covariance = self._apply_covariance_floor(
            track.front_filter_covariance, track.systematic_covariance)
        self._optimize_front_track(track)
        track.position = track.front_filter_position.copy()
        track.covariance = track.front_filter_covariance.copy()
        track.observation_count = len(observations)
        track.front_stereo_count = sum(
            observation.form == FORM_FRONT_STEREO for observation in observations)
        track.front_multi_view_count = sum(
            observation.form == FORM_FRONT_MULTI_VIEW
            for observation in observations)
        track.down_direct_count = 0
        track.observation_form_mask = 0
        for observation in observations:
            track.observation_form_mask |= int(observation.form)
        track.last_observation_form = int(window[-1].form)
        track.last_confidence = max(
            float(observation.confidence) for observation in observations)
        track.last_stamp = max(
            float(observation.stamp) for observation in observations)
        track.last_update_monotonic = time.monotonic()
        if old_state is not None:
            old_trace = float(np.trace(old_state["covariance"]))
            new_trace = float(np.trace(track.covariance))
            displacement = float(np.linalg.norm(
                track.position[:2] - old_state["position"][:2]))
            max_displacement = float(getattr(
                self, "front_gate_reassociation_distance", 0.75))
            if (new_trace > max(old_trace * 1.5, old_trace + 0.05)
                    or displacement > max_displacement):
                for name, value in old_state.items():
                    setattr(track, name, value)
                counter = ("front_gate_update_rejected"
                           if track.physical_class_name == "gate"
                           else "front_update_rejected")
                self._counters[counter] = self._counters.get(counter, 0) + 1

    def _append_front_raw_observations(
            self, track: TargetTrack,
            observations: list[FrontPixelObservation] | tuple[
                FrontPixelObservation, ...]):
        """Merge raw bbox observations into one bounded per-target window."""
        by_key = {}
        for observation in getattr(track, "front_pixel_observations", ()):
            observation.target_instance_id = int(track.instance_id)
            key = int(getattr(observation, "raw_observation_id", 0))
            if key <= 0:
                key = (observation.camera,
                       self._observation_stamp_key(observation.stamp))
            by_key[key] = observation
        for observation in observations:
            if observation is None:
                continue
            observation.target_instance_id = int(track.instance_id)
            key = int(getattr(observation, "raw_observation_id", 0))
            if key <= 0:
                key = (observation.camera,
                       self._observation_stamp_key(observation.stamp))
            by_key[key] = observation
        values = sorted(by_key.values(), key=lambda item: item.stamp)
        limit = max(2, int(getattr(
            self, "front_raw_observation_window_size", 100)))
        track.front_pixel_observations = deque(values[-limit:])

    @staticmethod
    def _front_project_pixel(observation: FrontPixelObservation,
                             position: np.ndarray,
                             body_translation: dict,
                             body_rotation: dict,
                             calibration: StereoCalibration):
        """Project a world point into one captured front image."""
        calibration = getattr(observation, "calibration", None) or calibration
        if calibration is None:
            return None
        camera = observation.camera
        pose = observation.pose
        point_body = pose.rotation.T @ (
            np.asarray(position, dtype=np.float64) - pose.position)
        point_optical = body_rotation[camera].T @ (
            point_body - body_translation[camera])
        if point_optical[2] <= 1e-6 or not np.all(np.isfinite(point_optical)):
            return None
        side = "left" if camera.endswith("_left") else "right"
        if side == "left":
            matrix, distortion = (
                calibration.camera_matrix_left, calibration.dist_left)
        else:
            matrix, distortion = (
                calibration.camera_matrix_right, calibration.dist_right)
        projected, _ = cv2.projectPoints(
            point_optical.reshape(1, 1, 3), np.zeros(3), np.zeros(3),
            matrix, distortion)
        result = projected.reshape(2)
        return result if np.all(np.isfinite(result)) else None

    def _optimize_front_track(self, track: TargetTrack) -> bool:
        """Robustly solve one target from all retained raw front bearings.

        This is a small bounded Gauss-Newton reprojection solver.  The
        existing stereo/multi-view XYZ point initializes the state, while the
        raw front image features provide the actual constraints.  Huber weighting
        keeps a wrong bbox from becoming a stable target.  With fewer than
        two useful views or a rank-deficient normal matrix the previous
        geometric estimate is retained.
        """
        observations = list(getattr(track, "front_pixel_observations", ()))
        if track.position is None or len(observations) < 2:
            return False
        calibration = getattr(self, "_front_calibration", None)
        if calibration is None and not any(
                getattr(observation, "calibration", None) is not None
                for observation in observations):
            return False
        if calibration is None:
            calibration = next(
                observation.calibration for observation in observations
                if getattr(observation, "calibration", None) is not None)
        position = np.asarray(track.position, dtype=np.float64).copy()
        for _ in range(6):
            normal = np.zeros((3, 3), dtype=np.float64)
            rhs = np.zeros(3, dtype=np.float64)
            valid_count = 0
            for observation in observations:
                projected = self._front_project_pixel(
                    observation, position, self.body_translation,
                    self.body_rotation, calibration)
                if projected is None:
                    continue
                residual = observation.pixel - projected
                covariance = _regularize_covariance(observation.covariance)
                inverse = _safe_inverse(covariance)
                whitened = math.sqrt(max(
                    float(residual.T @ inverse @ residual), 0.0))
                robust_weight = 1.0 if whitened <= self.huber_delta else (
                    self.huber_delta / max(whitened, 1e-9))
                jacobian = _numeric_jacobian(
                    lambda value: self._front_project_pixel(
                        observation, value, self.body_translation,
                        self.body_rotation, calibration),
                    position, np.full(3, 1e-4, dtype=np.float64))
                weight = inverse * robust_weight
                normal += jacobian.T @ weight @ jacobian
                rhs += jacobian.T @ weight @ residual
                valid_count += 1
            if valid_count < 2 or np.linalg.matrix_rank(
                    normal, tol=1e-8) < 3:
                return False
            delta = _safe_inverse(normal) @ rhs
            if not np.all(np.isfinite(delta)):
                return False
            step_norm = float(np.linalg.norm(delta))
            if step_norm > 1.0:
                delta *= 1.0 / step_norm
            position += delta
            if float(np.linalg.norm(delta)) < 1e-5:
                break
        if not np.all(np.isfinite(position)):
            return False

        normal = np.zeros((3, 3), dtype=np.float64)
        for observation in observations:
            projected = self._front_project_pixel(
                observation, position, self.body_translation,
                self.body_rotation, calibration)
            if projected is None:
                continue
            residual = observation.pixel - projected
            covariance = _regularize_covariance(observation.covariance)
            whitened = math.sqrt(max(float(
                residual.T @ _safe_inverse(covariance) @ residual), 0.0))
            robust_weight = 1.0 if whitened <= self.huber_delta else (
                self.huber_delta / max(whitened, 1e-9))
            jacobian = _numeric_jacobian(
                lambda value: self._front_project_pixel(
                    observation, value, self.body_translation,
                    self.body_rotation, calibration),
                position, np.full(3, 1e-4, dtype=np.float64))
            normal += jacobian.T @ _safe_inverse(covariance) \
                @ jacobian * robust_weight
        if np.linalg.matrix_rank(normal, tol=1e-8) < 3:
            return False
        covariance = _regularize_covariance(_safe_inverse(normal))
        shared_covariance = self._ray_shared_covariance(
            position,
            [SimpleNamespace(origin=observation.pose.position)
             for observation in observations])
        self._merge_track_systematic_covariance(track, shared_covariance)
        track.front_filter_position = position.copy()
        track.front_filter_covariance = self._apply_covariance_floor(
            covariance, track.systematic_covariance)
        track.position = position.copy()
        track.covariance = track.front_filter_covariance.copy()
        counters = getattr(self, "_counters", None)
        if counters is not None:
            counters["front_unified_optimizations"] = (
                counters.get("front_unified_optimizations", 0) + 1)
        return True

    @staticmethod
    def _trim_extreme_front_observations(
            observations, min_samples: int = 20,
            trim_fraction: float = 0.20):
        """Remove the most distant front samples once the window is large.

        Front observations are expected to be static in the world frame, so
        the component-wise median is a robust center for the retained window.
        Using distance from that center avoids letting a small number of bad
        triangulations pull the identity-measurement filter toward an extreme
        position.  The original time order is preserved for the remaining
        samples.
        """
        values = list(observations)
        count = len(values)
        if count <= int(min_samples):
            return values

        fraction = float(np.clip(trim_fraction, 0.0, 0.5))
        remove_count = min(count - 1, max(1, int(math.floor(count * fraction))))
        positions = np.asarray([
            np.asarray(observation.position, dtype=np.float64)
            for observation in values
        ])
        if positions.ndim != 2 or positions.shape[1] != 3 \
                or not np.all(np.isfinite(positions)):
            return values

        center = np.median(positions, axis=0)
        distances = np.linalg.norm(positions - center, axis=1)
        keep_count = count - remove_count
        keep_indices = np.argsort(distances, kind="stable")[:keep_count]
        # Keep the chronological order used by the Kalman pass.
        keep_indices = np.sort(keep_indices)
        return [values[int(index)] for index in keep_indices]

    @staticmethod
    def _fit_front_window(
            observations: deque[FrontPositionObservation]):
        """Identity-measurement Kalman filter for a front cluster window."""
        filtered = ObjectLocalizer._trim_extreme_front_observations(observations)
        first = filtered[0]
        position = first.position.copy()
        covariance = first.covariance.copy()
        identity = np.eye(3)
        for observation in filtered[1:]:
            measurement_covariance = observation.covariance
            innovation = observation.position - position
            innovation_covariance = _regularize_covariance(
                covariance + measurement_covariance)
            gain = covariance @ _safe_inverse(innovation_covariance)
            prior_covariance = covariance.copy()
            position = position + gain @ innovation
            covariance = _regularize_covariance(
                (identity - gain) @ prior_covariance @ (identity - gain).T
                + gain @ measurement_covariance @ gain.T)
        covariance = ObjectLocalizer._apply_covariance_floor(
            covariance,
            ObjectLocalizer._covariance_diagonal_envelope(
                getattr(observation, "shared_covariance", None)
                for observation in filtered),
        )
        return position, covariance

    def _remap_front_observation_history(
            self, semantic_class: str, observations, assignments,
            cluster_tracks):
        history = getattr(self, "_observation_history", ())
        if not history or not observations:
            return
        stamp_indices = self._observation_stamp_indices(observations)
        for record in history:
            if (record.source != "front"
                    or record.physical_class_name != semantic_class
                    or record.position is None):
                continue
            matching = stamp_indices.get(self._observation_stamp_key(
                record.stamp), ())
            if not matching:
                continue
            index = min(
                matching,
                key=lambda item: float(np.linalg.norm(
                    record.position[:2] - observations[item].position[:2])))
            track = cluster_tracks.get(int(assignments[index]))
            if track is not None:
                record.instance_id = track.instance_id

    def _new_front_track(self, class_id: int) -> TargetTrack | None:
        semantic_class = self._semantic_class(class_id)
        # Ray-only candidates are useful while their baseline is being built,
        # but an old candidate must not reserve a permanent instance slot.
        for key, candidate in list(self._front_tracks.items()):
            if (candidate.physical_class_name == semantic_class
                    and candidate.position is None
                    and self._front_ray_candidate_expired(candidate)):
                self._front_tracks.pop(key, None)
        existing = sum(
            1 for track in self._front_tracks.values()
            if self._semantic_class(track.class_id) == semantic_class
        )
        if existing >= self._instance_limit(class_id):
            return None
        instance_id = self._front_next_instance_id.get(semantic_class, 0)
        self._front_next_instance_id[semantic_class] = instance_id + 1
        track = TargetTrack(
            class_id=class_id,
            instance_id=instance_id,
            physical_class_name=semantic_class,
            observed_class_ids={class_id},
        )
        self._front_tracks[(semantic_class, instance_id)] = track
        return track

    def _add_down_observation_pool(self, class_id: int,
                                   position: np.ndarray,
                                   covariance: np.ndarray,
                                   stamp: float,
                                   confidence: float,
                                   shared_covariance: np.ndarray | None = None):
        """Keep every geometrically valid down point before instance gating."""
        semantic_class = self._semantic_class(class_id)
        pool = self._down_observation_pool.setdefault(
            semantic_class,
            deque(maxlen=self.down_observation_pool_size),
        )
        observation = DownDirectObservation(
            stamp=float(stamp),
            position=np.asarray(position, dtype=np.float64).copy(),
            covariance=_regularize_covariance(covariance),
            confidence=float(confidence),
            class_id=int(class_id),
            shared_covariance=(
                None if shared_covariance is None else
                _regularize_covariance(shared_covariance)),
        )
        pool.append(observation)
        return observation

    @staticmethod
    def _kmeans_horizontal(observations: list[DownDirectObservation],
                           cluster_count: int):
        """Deterministic covariance-aware K-means on N/E coordinates.

        Confidence alone is not enough for bbox-only stereo: a far, small-
        disparity point can have a high detector confidence but a very large
        geometric covariance.  Its influence on a cluster center is therefore
        bounded by the inverse horizontal variance.  The floor/clip prevents a
        single tiny-covariance point from dominating the complete rolling
        pool.
        """
        points = np.asarray([observation.position[:2]
                             for observation in observations],
                            dtype=np.float64)
        weights = []
        for observation in observations:
            confidence = max(float(observation.confidence), 0.05)
            covariance = np.asarray(
                getattr(observation, "covariance", np.eye(3)),
                dtype=np.float64)
            if covariance.shape == (3, 3) and np.all(np.isfinite(covariance)):
                horizontal_variance = max(
                    float(np.trace(covariance[:2, :2])) / 2.0,
                    0.01**2,
                )
            else:
                horizontal_variance = 0.25**2
            weights.append(float(np.clip(
                confidence / horizontal_variance, 0.05, 100.0)))
        weights = np.asarray(weights, dtype=np.float64)
        count = points.shape[0]
        cluster_count = max(1, min(int(cluster_count), count))

        centers = np.empty((cluster_count, 2), dtype=np.float64)
        centers[0] = points[0]
        for index in range(1, cluster_count):
            distance_squared = np.min(
                np.sum((points[:, None, :] - centers[None, :index, :])**2,
                       axis=2), axis=1)
            centers[index] = points[int(np.argmax(distance_squared))]

        assignments = np.zeros(count, dtype=np.int32)
        for _ in range(20):
            distance_squared = np.sum(
                (points[:, None, :] - centers[None, :, :])**2, axis=2)
            new_assignments = np.argmin(distance_squared, axis=1)
            new_centers = centers.copy()
            for cluster_index in range(cluster_count):
                members = new_assignments == cluster_index
                if not np.any(members):
                    new_centers[cluster_index] = points[int(np.argmax(
                        np.min(distance_squared, axis=1)))]
                    continue
                member_weights = weights[members]
                new_centers[cluster_index] = np.average(
                    points[members], axis=0, weights=member_weights)
            if (np.array_equal(assignments, new_assignments)
                    and np.allclose(centers, new_centers)):
                assignments = new_assignments
                centers = new_centers
                break
            assignments = new_assignments
            centers = new_centers

        # K-means with K=6 can split detector jitter into nearby guide-line
        # clusters before all six physical objects have been seen.  Collapse
        # centers that violate the known horizontal spacing rule and assign
        # all points again.  No Z value enters this operation.
        return assignments, centers

    def _merge_close_cluster_centers(self, observations, assignments, centers,
                                     semantic_class: str):
        """Merge K-means centers that cannot be separate physical objects."""
        radius = self._down_duplicate_merge_radius(semantic_class)
        while len(centers) > 1:
            distance_matrix = np.linalg.norm(
                centers[:, None, :] - centers[None, :, :], axis=2)
            distance_matrix += np.eye(len(centers)) * 1e9
            first, second = np.unravel_index(
                int(np.argmin(distance_matrix)), distance_matrix.shape)
            if distance_matrix[first, second] >= radius:
                break
            centers = np.delete(centers, second, axis=0)
            assignments = np.argmin(
                np.sum((np.asarray([observation.position[:2]
                                     for observation in observations])[:, None, :]
                        - centers[None, :, :])**2, axis=2), axis=1)
        return assignments, centers

    def _cluster_to_existing_tracks(self, centers, existing_tracks,
                                    observations=None, assignments=None):
        """Associate clusters to tracks with a gated global assignment.

        K-means is retained as a pool bootstrapper, but it must not be allowed
        to force a distant noisy cluster onto an existing instance.  A cluster
        can remain unmatched and a track can remain untouched; this preserves
        identity through dropouts and lets the normal age policy mark stale
        tracks.  The cost is a horizontal Mahalanobis distance using both the
        track covariance and the observed cluster spread.
        """
        centers = np.asarray(centers, dtype=np.float64).reshape(-1, 2)
        cluster_count = len(centers)
        track_count = len(existing_tracks)
        if cluster_count == 0 or track_count == 0:
            return {}

        cluster_covariances = [np.eye(2, dtype=np.float64) * 0.25**2
                               for _ in range(cluster_count)]
        if observations is not None and assignments is not None:
            for cluster_index in range(cluster_count):
                members = [
                    observation for observation, assignment in zip(
                        observations, assignments)
                    if int(assignment) == cluster_index
                ]
                if not members:
                    continue
                values = np.asarray([
                    observation.position[:2] for observation in members
                ], dtype=np.float64)
                covariance_values = [
                    np.asarray(observation.covariance[:2, :2],
                               dtype=np.float64)
                    for observation in members
                    if np.asarray(observation.covariance).shape == (3, 3)
                ]
                spread = np.zeros((2, 2), dtype=np.float64)
                if len(values) > 1:
                    spread = np.cov(values, rowvar=False, ddof=1)
                    spread = np.asarray(spread, dtype=np.float64).reshape(2, 2)
                measurement = (
                    np.mean(covariance_values, axis=0)
                    if covariance_values else np.eye(2) * 0.25**2
                )
                cluster_covariances[cluster_index] = _regularize_covariance(
                    measurement + spread, minimum_variance=1e-6)

        costs = [[float("inf")] * track_count
                 for _ in range(cluster_count)]
        for cluster_index, center in enumerate(centers):
            for track_index, track in enumerate(existing_tracks):
                if track.position is None or track.covariance is None:
                    continue
                track_covariance = np.asarray(
                    track.covariance[:2, :2], dtype=np.float64)
                innovation_covariance = _regularize_covariance(
                    track_covariance + cluster_covariances[cluster_index],
                    minimum_variance=1e-6)
                innovation = center - track.position[:2]
                distance = float(innovation.T @ _safe_inverse(
                    innovation_covariance) @ innovation)
                if np.isfinite(distance) and distance <= self.position_gate_chi2:
                    costs[cluster_index][track_index] = distance

        # Enumerate assignments with an explicit unmatched option.  The
        # number of instances is bounded by the scene priors (1/4/6), making
        # this exact search inexpensive and deterministic.
        best = (-1, float("inf"), ())

        def visit(cluster_index: int, used_tracks: frozenset,
                  selected: tuple[tuple[int, int], ...], total_cost: float):
            nonlocal best
            if cluster_index >= cluster_count:
                score = (len(selected), -total_cost)
                best_score = (best[0], -best[1])
                if score > best_score:
                    best = (len(selected), total_cost, selected)
                return

            # Leave this cluster unmatched when it is outside every gate.
            visit(cluster_index + 1, used_tracks, selected, total_cost)
            for track_index, cost in enumerate(costs[cluster_index]):
                if not np.isfinite(cost) or track_index in used_tracks:
                    continue
                visit(
                    cluster_index + 1,
                    used_tracks | frozenset((track_index,)),
                    selected + ((cluster_index, track_index),),
                    total_cost + cost,
                )

        visit(0, frozenset(), (), 0.0)
        return {
            cluster_index: existing_tracks[track_index]
            for cluster_index, track_index in best[2]
        }

    def _update_track_from_down_cluster(
            self, track: TargetTrack, class_id: int,
            observations: list[DownDirectObservation]):
        """Replace one instance state with its current cluster window."""
        if (track.front_stereo_count > 0 or track.front_multi_view_count > 0
                or track.rays):
            self._reanchor_from_down(
                track, class_id, observations[-1].position,
                observations[-1].covariance)

        ordered = sorted(observations, key=lambda observation: observation.stamp)
        window = ordered[-self.down_direct_queue_size:]
        track.class_id = int(class_id)
        track.physical_class_name = self._semantic_class(class_id)
        track.observed_class_ids = {
            observation.class_id for observation in observations
            if observation.class_id >= 0
        } or {int(class_id)}
        track.rays.clear()
        track.front_stereo_count = 0
        track.front_multi_view_count = 0
        track.down_observations = deque(window)
        (track.down_filter_position,
         track.down_filter_covariance) = self._fit_down_direct_window(
             track.down_observations)
        window_shared_covariance = self._covariance_diagonal_envelope(
            getattr(observation, "shared_covariance", None)
            for observation in track.down_observations)
        track.systematic_covariance = self._merge_track_systematic_covariance(
            track, window_shared_covariance)
        track.down_filter_covariance = self._apply_covariance_floor(
            track.down_filter_covariance, track.systematic_covariance)
        track.position = track.down_filter_position.copy()
        track.covariance = track.down_filter_covariance.copy()
        track.observation_count = len(observations)
        track.down_direct_count = len(observations)
        track.observation_form_mask = FORM_DOWN_DIRECT
        track.last_observation_form = FORM_DOWN_DIRECT
        track.last_confidence = max(
            float(observation.confidence) for observation in observations)
        track.last_stamp = max(
            float(observation.stamp) for observation in observations)
        track.last_update_monotonic = time.monotonic()

    def _remap_down_observation_history(self, semantic_class: str,
                                        observations, assignments,
                                        cluster_tracks):
        """Keep GUI/raw observation instance IDs consistent after reclustering."""
        history = getattr(self, "_observation_history", ())
        if not history or not observations:
            return
        stamp_indices = self._observation_stamp_indices(observations)
        for record in history:
            if (record.form != FORM_DOWN_DIRECT
                    or record.physical_class_name != semantic_class
                    or record.position is None):
                continue
            matching = stamp_indices.get(self._observation_stamp_key(
                record.stamp), ())
            if not matching:
                continue
            index = min(
                matching,
                key=lambda item: float(np.linalg.norm(
                    record.position[:2] - observations[item].position[:2])))
            track = cluster_tracks.get(int(assignments[index]))
            if track is not None:
                record.instance_id = track.instance_id

    @staticmethod
    def _observation_stamp_key(stamp: float) -> int:
        """Make ROS timestamps directly indexable for history remapping."""
        return int(round(float(stamp) * 1e6))

    @classmethod
    def _observation_stamp_indices(cls, observations):
        """Index observations by timestamp instead of scanning them repeatedly."""
        indices = {}
        for index, observation in enumerate(observations):
            key = cls._observation_stamp_key(observation.stamp)
            indices.setdefault(key, []).append(index)
        return indices

    def _recluster_down_class(self, class_id: int,
                              current_observation: DownDirectObservation):
        """Cluster a complete class pool, then update one track per cluster."""
        semantic_class = self._semantic_class(class_id)
        observations = list(self._down_observation_pool.get(semantic_class, ()))
        if not observations:
            return None

        cluster_count = min(self._instance_limit(class_id), len(observations))
        assignments, centers = self._kmeans_horizontal(
            observations, cluster_count)
        assignments, centers = self._merge_close_cluster_centers(
            observations, assignments, centers, semantic_class)

        existing_tracks = [
            track for track in self._tracks.values()
            if track.physical_class_name == semantic_class
            and track.position is not None
        ]
        cluster_tracks = self._cluster_to_existing_tracks(
            centers, existing_tracks, observations, assignments)

        # A rolling pool can temporarily lose a target because of detector
        # dropout or a bad cluster split.  Keep the old track and let its
        # normal age state become STALE instead of deleting its identity on
        # one reclustering pass.

        for cluster_index in range(len(centers)):
            if cluster_index not in cluster_tracks:
                track = self._new_track(class_id)
                if track is None:
                    # A reliable down observation may be the first valid
                    # evidence for a class whose instance slot was consumed
                    # by an earlier front-only hypothesis.  Re-anchor that
                    # hypothesis instead of dropping the pooled observation.
                    member_indices = [
                        index for index, assignment in enumerate(assignments)
                        if int(assignment) == cluster_index
                    ]
                    if member_indices:
                        replacement = self._front_only_replacement(
                            class_id,
                            observations[member_indices[-1]].position)
                        if replacement is not None:
                            self._reanchor_from_down(
                                replacement, class_id,
                                observations[member_indices[-1]].position,
                                observations[member_indices[-1]].covariance)
                            track = replacement
                if track is None:
                    continue
                cluster_tracks[cluster_index] = track

        for cluster_index, track in cluster_tracks.items():
            members = [
                observations[index] for index, assignment
                in enumerate(assignments) if int(assignment) == cluster_index
            ]
            if members:
                self._update_track_from_down_cluster(
                    track, class_id, members)

        self._remap_down_observation_history(
            semantic_class, observations, assignments, cluster_tracks)
        current_index = len(observations) - 1
        return cluster_tracks.get(int(assignments[current_index]))

    def _record_position_observation(self, track: TargetTrack | None,
                                     observed_class_id: int, stamp: float,
                                     position: np.ndarray,
                                     covariance: np.ndarray, form: int,
                                     confidence: float,
                                     source: str = "down",
                                     source_raw_observation_ids: tuple[int, ...] = (),
                                     feature_id: str = ""):
        """Retain a direct 3-D factor; this is not the fused track state."""
        physical_class_name = (
            track.physical_class_name if track is not None
            else self._semantic_class(observed_class_id)
        )
        instance_id = (
            track.instance_id if track is not None else UNASSIGNED_INSTANCE_ID
        )
        record = ObservationRecord(
            observation_id=self._next_observation_id,
            stamp=stamp,
            class_id=observed_class_id,
            instance_id=instance_id,
            physical_class_name=physical_class_name,
            form=form,
            confidence=confidence,
            position=np.asarray(position, dtype=np.float64).copy(),
            covariance=np.asarray(covariance, dtype=np.float64).copy(),
            source=source,
            source_raw_observation_ids=tuple(
                int(value) for value in source_raw_observation_ids
                if int(value) > 0),
            feature_id=str(feature_id),
        )
        self._observation_history.append(record)
        self._next_observation_id += 1
        return record

    def _record_ray_observation(self, track: TargetTrack | None,
                                observed_class_id: int, stamp: float,
                                origin: np.ndarray, direction: np.ndarray,
                                confidence: float,
                                source: str = "front",
                                source_raw_observation_ids: tuple[int, ...] = (),
                                feature_id: str = "bbox_center",
                                form: int = FORM_FRONT_MULTI_VIEW):
        """Retain a front bearing as a ray, without inventing a 3-D point."""
        record = ObservationRecord(
            observation_id=self._next_observation_id,
            stamp=stamp,
            class_id=observed_class_id,
            instance_id=(track.instance_id if track is not None
                         else UNASSIGNED_INSTANCE_ID),
            physical_class_name=(
                track.physical_class_name if track is not None
                else self._semantic_class(observed_class_id)),
            form=int(form),
            confidence=confidence,
            ray_origin=np.asarray(origin, dtype=np.float64).copy(),
            ray_direction=np.asarray(direction, dtype=np.float64).copy(),
            source=source,
            source_raw_observation_ids=tuple(
                int(value) for value in source_raw_observation_ids
                if int(value) > 0),
            feature_id=str(feature_id),
        )
        self._observation_history.append(record)
        self._next_observation_id += 1
        return record

    def _linear_update(self, track: TargetTrack, measurement: np.ndarray,
                       jacobian: np.ndarray, covariance: np.ndarray,
                       form: int, confidence: float,
                       shared_covariance: np.ndarray | None = None) -> bool:
        measurement = np.asarray(measurement, dtype=np.float64).reshape(-1)
        jacobian = np.asarray(jacobian, dtype=np.float64)
        covariance = _regularize_covariance(covariance)
        systematic_covariance = self._merge_track_systematic_covariance(
            track, shared_covariance)
        if track.position is None or track.covariance is None:
            if jacobian.shape == (3, 3) and np.allclose(
                    jacobian, np.eye(3)):
                track.position = measurement.copy()
                track.covariance = self._apply_covariance_floor(
                    covariance, systematic_covariance)
                return True
            return False

        innovation = measurement - jacobian @ track.position
        innovation_covariance = _regularize_covariance(
            jacobian @ track.covariance @ jacobian.T + covariance)
        mahalanobis = float(innovation.T @ _safe_inverse(
            innovation_covariance) @ innovation)
        if mahalanobis > self.ray_gate_chi2:
            return False

        distance = math.sqrt(max(mahalanobis, 0.0))
        weight = 1.0 if distance <= self.huber_delta else (
            self.huber_delta / max(distance, 1e-9))
        effective_covariance = covariance / weight
        innovation_covariance = _regularize_covariance(
            jacobian @ track.covariance @ jacobian.T + effective_covariance)
        kalman_gain = (
            track.covariance @ jacobian.T
            @ _safe_inverse(innovation_covariance)
        )
        identity = np.eye(track.position.size)
        prior_covariance = track.covariance.copy()
        track.position = track.position + kalman_gain @ innovation
        track.covariance = _regularize_covariance(
            (identity - kalman_gain @ jacobian) @ prior_covariance
            @ (identity - kalman_gain @ jacobian).T
            + kalman_gain @ effective_covariance @ kalman_gain.T)
        track.covariance = self._apply_covariance_floor(
            track.covariance, systematic_covariance)
        track.last_confidence = max(track.last_confidence, confidence)
        track.last_update_monotonic = time.monotonic()
        return True

    def _semantic_class(self, class_id: int) -> str:
        """Map camera-specific labels onto one physical target category."""
        name = self._class_name(class_id).strip().lower()
        for suffix in ("_front", "_down"):
            if name.endswith(suffix):
                name = name[:-len(suffix)]
                break
        # The legacy model uses gate_* while the 8029 model uses door.
        if name in ("gate", "door"):
            return "gate"
        return name

    def _label_camera_hint(self, class_id: int) -> str | None:
        """Return the camera model implied by a camera-specific YOLO label."""
        name = self._class_name(class_id).strip().lower()
        if name.endswith("_front"):
            return "front"
        if name.endswith("_down"):
            return "down"
        return None

    def _detections_for_camera(self, camera_pair: str, detections):
        """Reject labels produced by the wrong camera-specific detector head.

        A ``foo_front`` and ``foo_down`` pair describes one physical object,
        but their detector weights are camera-specific.  The semantic binding
        happens later in association; this filter first prevents e.g. a
        ``foo_down`` false positive in a front image from entering a front
        stereo or multi-view factor.
        """
        compatible = []
        for detection in detections:
            hint = self._label_camera_hint(int(detection.class_id))
            if hint is None or hint == camera_pair:
                compatible.append(detection)
            else:
                self._counters["camera_label_rejected"] += 1
        return compatible

    def _instance_limit(self, class_id: int) -> int:
        semantic_class = self._semantic_class(class_id)
        if semantic_class == "guide_line":
            return self.max_instances_guide_line
        if semantic_class == "gate":
            return self.max_instances_gate
        return self.max_instances_default

    def _new_track(self, class_id: int) -> TargetTrack | None:
        semantic_class = self._semantic_class(class_id)
        existing = sum(
            1 for track in self._tracks.values()
            if self._semantic_class(track.class_id) == semantic_class
        )
        if existing >= self._instance_limit(class_id):
            return None
        instance_id = self._next_instance_id.get(semantic_class, 0)
        self._next_instance_id[semantic_class] = instance_id + 1
        track = TargetTrack(
            class_id=class_id,
            instance_id=instance_id,
            physical_class_name=semantic_class,
            observed_class_ids={class_id},
        )
        self._tracks[(semantic_class, instance_id)] = track
        return track

    def _reject_instance_limit(self, class_id: int):
        semantic_class = self._semantic_class(class_id)
        self._counters["instance_limit_rejected"] += 1
        self._warn_quality_once(
            f"instance limit for {semantic_class} "
            f"({self._instance_limit(class_id)})")

    def _class_name(self, class_id: int) -> str:
        if 0 <= class_id < len(self.class_names):
            return self.class_names[class_id]
        return f"class_{class_id}"

    def _track_age(self, track: TargetTrack) -> float:
        if track.last_stamp <= 0.0:
            return max(0.0, time.monotonic() - track.last_update_monotonic)
        now_stamp = self.get_clock().now().nanoseconds * 1e-9
        return max(0.0, now_stamp - track.last_stamp)

    def _track_status(self, track: TargetTrack, age: float) -> int:
        if track.position is None or track.covariance is None:
            return int(TargetPosition.STATUS_UNINITIALIZED)
        timeout = (
            self.front_gate_track_timeout
            if track.physical_class_name == "gate"
            else self.track_timeout)
        if age > timeout:
            return int(TargetPosition.STATUS_STALE)
        stable_trace = (
            self.front_gate_stable_trace
            if track.physical_class_name == "gate"
            else self.stable_trace)
        if (track.observation_count >= self.minimum_stable_observations
                and float(np.trace(track.covariance)) <= stable_trace):
            return int(TargetPosition.STATUS_STABLE)
        return int(TargetPosition.STATUS_ESTIMATING)

    def _publish(self):
        self._merge_close_down_tracks()
        now_msg = self.get_clock().now().to_msg()
        compat = ObjectPositionArray()
        compat.header.stamp = now_msg
        compat.header.frame_id = "odom"
        target_array = TargetPositionArray()
        target_array.header.stamp = now_msg
        target_array.header.frame_id = "odom"
        observation_array = TargetObservationArray()
        observation_array.header.stamp = now_msg
        observation_array.header.frame_id = "odom"

        for estimate_source, tracks in (
                ("down", self._tracks), ("front", self._front_tracks)):
            for track in tracks.values():
                if track.position is None or track.covariance is None:
                    continue
                age = self._track_age(track)
                status = self._track_status(track, age)
                if estimate_source == "front":
                    # Keep stale tracks in the message with an explicit
                    # STATUS_STALE marker.  The GUI filters them by default,
                    # while consumers that maintain a world map can still
                    # see the last estimate instead of losing the track.
                    if (track.physical_class_name == "gate"
                            and (track.observation_count
                                 < getattr(self,
                                           "front_gate_min_cluster_observations",
                                           1)
                                 or self._track_confidence(track)
                                 < getattr(self,
                                           "front_gate_min_publish_confidence",
                                           getattr(self,
                                                   "front_min_publish_confidence",
                                                   0.0)))):
                        continue
                target = TargetPosition()
                target.class_id = int(track.class_id)
                target.instance_id = int(track.instance_id)
                target.class_name = self._class_name(track.class_id)
                target.physical_class_name = track.physical_class_name
                target.observed_class_ids = sorted(track.observed_class_ids)
                target.estimate_source = estimate_source
                target.world_x = float(track.position[0])
                target.world_y = float(track.position[1])
                target.world_z = float(track.position[2])
                target.position_covariance = [
                    float(value) for value in track.covariance.reshape(-1)
                ]
                target.confidence = float(self._track_confidence(track))
                target.num_observations = int(track.observation_count)
                target.front_stereo_count = int(track.front_stereo_count)
                target.front_multi_view_count = int(track.front_multi_view_count)
                target.down_direct_count = int(track.down_direct_count)
                target.observation_form_mask = int(track.observation_form_mask)
                target.last_observation_form = int(track.last_observation_form)
                target.status = status
                target.age_sec = float(age)
                if track.last_stamp > 0.0:
                    target.last_observation_stamp.sec = int(track.last_stamp)
                    target.last_observation_stamp.nanosec = int(
                        (track.last_stamp - int(track.last_stamp)) * 1e9)
                target_array.targets.append(target)

                # Keep TaskRunner/Navigator compatibility output conservative:
                # only the trusted down estimator is exposed on /objects.
                if (estimate_source == "down"
                        and status != int(TargetPosition.STATUS_STALE)):
                    compat_object = ObjectPosition()
                    compat_object.class_id = int(track.class_id)
                    compat_object.instance_id = int(track.instance_id)
                    compat_object.class_name = self._class_name(track.class_id)
                    compat_object.world_x = float(track.position[0])
                    compat_object.world_y = float(track.position[1])
                    compat_object.world_z = float(track.position[2])
                    compat_object.confidence = float(
                        self._track_confidence(track))
                    compat_object.num_observations = int(track.observation_count)
                    compat.objects.append(compat_object)

        for record in self._observation_history:
            observation = TargetObservation()
            observation.observation_id = int(record.observation_id)
            observation.observation_stamp.sec = int(record.stamp)
            observation.observation_stamp.nanosec = int(
                (record.stamp - int(record.stamp)) * 1e9)
            observation.source_raw_observation_ids = [
                int(value) for value in getattr(
                    record, "source_raw_observation_ids", ())]
            observation.feature_id = str(getattr(record, "feature_id", ""))
            observation.class_id = int(record.class_id)
            observation.instance_id = int(record.instance_id)
            observation.class_name = self._class_name(record.class_id)
            observation.physical_class_name = record.physical_class_name
            observation.estimate_source = record.source
            observation.confidence = float(record.confidence)
            observation.observation_form = int(record.form)
            observation.has_position = record.position is not None
            if record.position is not None:
                observation.world_x = float(record.position[0])
                observation.world_y = float(record.position[1])
                observation.world_z = float(record.position[2])
                if record.covariance is not None:
                    observation.position_covariance = [
                        float(value) for value in record.covariance.reshape(-1)
                    ]
            if record.ray_origin is not None:
                observation.ray_origin_x = float(record.ray_origin[0])
                observation.ray_origin_y = float(record.ray_origin[1])
                observation.ray_origin_z = float(record.ray_origin[2])
            if record.ray_direction is not None:
                observation.ray_direction_x = float(record.ray_direction[0])
                observation.ray_direction_y = float(record.ray_direction[1])
                observation.ray_direction_z = float(record.ray_direction[2])
            observation_array.observations.append(observation)

        self._pub_targets.publish(target_array)
        self._pub_compat.publish(compat)
        self._pub_observations.publish(observation_array)

    def _track_confidence(self, track: TargetTrack) -> float:
        if track.covariance is None:
            return 0.0
        uncertainty = float(np.trace(track.covariance))
        geometry_confidence = math.exp(-uncertainty / 0.25)
        return float(np.clip(
            max(track.last_confidence, 0.05) * geometry_confidence, 0.0, 1.0))

    def _summary(self):
        if time.monotonic() - self._last_summary_monotonic < 4.0:
            return
        self._last_summary_monotonic = time.monotonic()
        gate_tracks = []
        for track in self._front_tracks.values():
            if track.physical_class_name != "gate":
                continue
            covariance_trace = (
                float(np.trace(track.covariance))
                if track.covariance is not None else float("inf"))
            gate_tracks.append(
                f"{track.instance_id}:n={track.observation_count},"
                f"c={self._track_confidence(track):.2f},"
                f"tr={covariance_trace:.3f},"
                f"eff={getattr(track, 'front_effective_observations', 0.0):.1f},"
                f"k={getattr(track, 'front_condition_number', float('inf')):.1f},"
                f"s={self._track_status(track, self._track_age(track))}")
        self.get_logger().info(
            "localizer: down_tracks=%d front_tracks=%d down_pool=%s "
            "front_pool=%s front_bearing_pool=%s gate_tracks=[%s] %s" % (
                len(self._tracks),
                len(self._front_tracks),
                {key: len(value) for key, value in
                 self._down_observation_pool.items()},
                {key: len(value) for key, value in
                 self._front_observation_pool.items()},
                {key: len(value) for key, value in
                 getattr(self, "_front_bearing_pool", {}).items()},
                ";".join(gate_tracks),
                ", ".join(f"{key}={value}" for key, value in self._counters.items()),
            ))

    def _warn_once(self, key: str, message: str):
        if key in self._warned:
            return
        self._warned.add(key)
        self.get_logger().warn(message)

    def _warn_quality_once(self, reason: str):
        self._warn_once(reason, reason)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectLocalizer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        # launch may already have shut down this context while delivering a
        # process-wide shutdown signal.  Calling rclpy.shutdown() twice turns
        # an otherwise normal stop into exit code 1.
        if rclpy.ok():
            rclpy.shutdown()
