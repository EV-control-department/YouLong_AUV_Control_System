"""Downward-stereo static target localization node.

This node deliberately replaces the legacy monocular position implementation.
It consumes timestamped detection metadata, loads the stereo calibration profiles,
and fuses valid downward-stereo observations into one static 3-D position per target:

* DOWN_DIRECT: a down-camera plane intersection or a down stereo measurement.

The current competition localization policy intentionally uses only the
downward camera.  The old front-stereo and front-multi-view implementation is
kept below for compatibility with old offline tests, but it is not subscribed
or executed by this node.

The public compatibility output is ObjectPositionArray on /perception/objects.
TargetPositionArray additionally exposes covariance and observation provenance.
"""

from __future__ import annotations

import bisect
from itertools import permutations
import math
import os
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
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
        [-sp, cy * sr, cy * cr],
    ], dtype=np.float64)


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
class RayObservation:
    stamp: float
    origin: np.ndarray
    direction: np.ndarray
    sigma_angle: float
    confidence: float


@dataclass
class DownDirectObservation:
    """One accepted raw DOWN_DIRECT factor in a track-local sliding window."""

    stamp: float
    position: np.ndarray
    covariance: np.ndarray
    confidence: float
    class_id: int = -1


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
    down_filter_position: np.ndarray | None = None
    down_filter_covariance: np.ndarray | None = None
    last_stamp: float = 0.0
    last_confidence: float = 0.0
    observation_count: int = 0
    front_stereo_count: int = 0
    front_multi_view_count: int = 0
    down_direct_count: int = 0
    observation_form_mask: int = 0
    last_observation_form: int = 0
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
    """Static target localizer using calibrated stereo and robust EKF updates."""

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
        # Key by physical semantic class, never by a camera-specific YOLO ID.
        # For example, collection_frame_front/collection_frame_down share the
        # same "collection_frame" instance namespace.
        self._tracks: dict[tuple[str, int], TargetTrack] = {}
        self._next_instance_id: dict[str, int] = {}
        # Large class-separated pool of geometrically valid down observations.
        # Instance association consumes this pool into each track's bounded
        # down_observations Kalman window.
        self._down_observation_pool: dict[
            str, deque[DownDirectObservation]] = {}
        self._warned: set[str] = set()
        self._counters = {
            "front_stereo_accepted": 0,
            "front_stereo_rejected": 0,
            "front_pair_invalid": 0,
            "front_multi_view_rays": 0,
            "front_multi_view_angle_rejected": 0,
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
        self.create_timer(5.0, self._summary)

        if self._calibration_ready:
            self.get_logger().info(
                "object_localizer started with down-only stereo calibration")
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
        self.declare_parameter("front_bbox_width_ratio_max", 1.6)
        self.declare_parameter("front_bbox_height_ratio_max", 1.6)
        self.declare_parameter("front_bbox_area_ratio_max", 2.2)
        self.declare_parameter("front_epipolar_error_px", 8.0)
        self.declare_parameter("min_disparity_px", 2.0)
        self.declare_parameter("min_depth_m", 0.05)
        self.declare_parameter("max_depth_m", 50.0)
        self.declare_parameter("use_rejected_front_pairs_for_multiview", False)

        self.declare_parameter("down_plane_enabled", False)
        self.declare_parameter("down_plane_normal", [0.0, 0.0, 1.0])
        self.declare_parameter("down_plane_c", 0.0)
        self.declare_parameter("down_plane_sigma_m", 0.03)
        self.declare_parameter("down_plane_consistency_m", 0.12)
        self.declare_parameter("down_observation_pool_size", 2000)
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
        self.declare_parameter("front_stereo_noise_scale", 1.0)
        self.declare_parameter("front_multi_view_noise_scale", 1.5)
        self.declare_parameter("down_direct_noise_scale", 0.75)

        self.declare_parameter("front_multi_view_max_rays", 20)
        self.declare_parameter("front_multi_view_min_angle_deg", 5.0)
        self.declare_parameter("front_multi_view_max_line_error_m", 1.0)
        self.declare_parameter("ray_association_angle_deg", 25.0)
        self.declare_parameter("ray_association_distance_m", 1.0)
        self.declare_parameter("position_gate_chi2", 16.0)
        self.declare_parameter("down_direct_reanchor_chi2", 9.0)
        self.declare_parameter("ray_gate_chi2", 16.0)
        self.declare_parameter("huber_delta", 2.5)
        self.declare_parameter("track_timeout_sec", 2.0)
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
        self.bbox_width_ratio_max = float(get("front_bbox_width_ratio_max").value)
        self.bbox_height_ratio_max = float(get("front_bbox_height_ratio_max").value)
        self.bbox_area_ratio_max = float(get("front_bbox_area_ratio_max").value)
        self.epipolar_error_max = float(get("front_epipolar_error_px").value)
        self.min_disparity = float(get("min_disparity_px").value)
        self.min_depth = float(get("min_depth_m").value)
        self.max_depth = float(get("max_depth_m").value)
        self.use_rejected_front_pairs_for_multiview = bool(
            get("use_rejected_front_pairs_for_multiview").value)

        self.down_plane_enabled = bool(get("down_plane_enabled").value)
        normal = _finite_vector(get("down_plane_normal").value, 3)
        if normal is None or np.linalg.norm(normal) < 1e-9:
            raise ValueError("down_plane_normal must be a nonzero 3-vector")
        self.down_plane_normal = normal / np.linalg.norm(normal)
        self.down_plane_c = float(get("down_plane_c").value)
        self.down_plane_sigma = float(get("down_plane_sigma_m").value)
        self.down_plane_consistency = float(
            get("down_plane_consistency_m").value)
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
        self.front_stereo_scale = float(get("front_stereo_noise_scale").value)
        self.front_multi_scale = float(
            get("front_multi_view_noise_scale").value)
        self.down_direct_scale = float(get("down_direct_noise_scale").value)

        self.max_rays = int(get("front_multi_view_max_rays").value)
        self.min_ray_angle_rad = math.radians(
            float(get("front_multi_view_min_angle_deg").value))
        self.max_line_error = float(
            get("front_multi_view_max_line_error_m").value)
        self.ray_assoc_angle_rad = math.radians(
            float(get("ray_association_angle_deg").value))
        self.ray_assoc_distance = float(
            get("ray_association_distance_m").value)
        self.position_gate_chi2 = float(get("position_gate_chi2").value)
        self.down_direct_reanchor_chi2 = max(
            0.0, float(get("down_direct_reanchor_chi2").value))
        self.ray_gate_chi2 = float(get("ray_gate_chi2").value)
        self.huber_delta = float(get("huber_delta").value)
        self.track_timeout = float(get("track_timeout_sec").value)
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
            # Front calibration is intentionally not loaded: front detections
            # are not part of the current localization policy.
            self._front_calibration = None
            self._down_calibration = self._load_calibration(
                "down", self.down_calibration_file)
        # ``sim_camera_info`` is completed after both downward CameraInfo
        # messages arrive.  Do not fall back to a hardware NPZ: that would
        # silently apply the wrong focal length and baseline in sim.
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
        if camera_pair != "down":
            # The current localizer deliberately uses only downward stereo.
            return
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
            else:
                self._down_calibration = None
            self._calibration_ready = False
            self.get_logger().error(
                f"Failed to build {camera_pair} simulator calibration: {error}")
            return

        if camera_pair == "front":
            self._front_calibration = calibration
            self.front_width = int(left_info.width)
            self.front_height = int(left_info.height)
        else:
            self._down_calibration = calibration
            self.down_width = int(left_info.width)
            self.down_height = int(left_info.height)

        was_ready = self._calibration_ready
        self._calibration_ready = self._down_calibration is not None
        self.get_logger().info(
            f"Loaded simulator {camera_pair} calibration from CameraInfo: "
            f"{left_info.width}x{left_info.height}, "
            f"baseline={calibration.baseline_m:.5f} m, "
            f"fx={calibration.camera_matrix_left[0, 0]:.4f}, "
            f"fy={calibration.camera_matrix_left[1, 1]:.4f}")
        if self._calibration_ready and not was_ready:
            self.get_logger().info(
                "Simulator stereo calibration ready; target localization enabled")

    def _create_subscriptions(self):
        self.create_subscription(
            PoseInfo, "/basic_motion/pose_info", self._pose_callback, 20)
        self.create_subscription(
            DetectionArray, "/perception/detection/down_left",
            self._down_left_callback, 20)
        self.create_subscription(
            DetectionArray, "/perception/detection/down_right",
            self._down_right_callback, 20)
        if self.calibration_source == "sim_camera_info":
            for camera in ("down_left", "down_right"):
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
        if camera.startswith("front"):
            # Defensive guard in case an external caller invokes the legacy
            # callback directly.  No front detection subscription is created.
            return
        arrival = time.monotonic()
        self._pending[camera].append((arrival, message))
        self._try_pair("down")

    @staticmethod
    def _pop_deque_index(values: deque, index: int):
        items = list(values)
        result = items.pop(index)
        values.clear()
        values.extend(items)
        return result

    def _try_pair(self, camera_pair: str):
        if camera_pair != "down":
            return
        left_key = f"{camera_pair}_left"
        right_key = f"{camera_pair}_right"
        left_queue = self._pending[left_key]
        right_queue = self._pending[right_key]
        while left_queue and right_queue:
            best = None
            for left_index, (_, left_message) in enumerate(left_queue):
                left_stamp = _stamp_seconds(left_message.header.stamp)
                for right_index, (_, right_message) in enumerate(right_queue):
                    right_stamp = _stamp_seconds(right_message.header.stamp)
                    if left_stamp <= 0.0 or right_stamp <= 0.0:
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
        for side in ("left", "right"):
            key = f"down_{side}"
            queue = self._pending[key]
            while queue and now - queue[0][0] > self.pending_timeout:
                _, message = queue.popleft()
                if self.down_plane_enabled:
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
            else:
                sample = before if abs(stamp - before.stamp) <= abs(
                    stamp - after.stamp) else after
                age = abs(stamp - sample.stamp)

        if age > self.pose_max_age_sec:
            self._warn_once(
                "pose_stale",
                f"discarding detections with pose age>{self.pose_max_age_sec:.3f}s")
            return None
        return self._pose_from_sample(sample, age)

    def _pose_from_sample(self, sample: PoseSample, age: float) -> PoseAt:
        age_scale = 1.0 + min(age / max(self.pose_max_age_sec, 1e-3), 5.0)
        covariance = np.zeros((6, 6), dtype=np.float64)
        covariance[:3, :3] = self.pose_position_covariance * age_scale**2
        covariance[3:, 3:] = self.pose_angle_covariance * age_scale**2
        return PoseAt(
            stamp=sample.stamp,
            position=sample.position,
            rotation=_rpy_to_rotation(
                sample.roll_deg, sample.pitch_deg, sample.yaw_deg),
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
        stamp = self._pair_stamp(left_message, right_message)
        pose = self._lookup_pose(stamp)
        if pose is None or not self._calibration_ready:
            return
        calibration = self._front_calibration
        left_detections = self._detections_for_camera(
            "front", left_message.detections)
        right_detections = self._detections_for_camera(
            "front", right_message.detections)
        pairs, unmatched_left, unmatched_right = self._match_detections(
            left_detections, right_detections, calibration)

        used_left = set()
        used_right = set()
        left_classes = {int(det.class_id) for det in left_detections}

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
                        "front", left, right, pose)
                except (ValueError, cv2.error, np.linalg.LinAlgError) as error:
                    valid = False
                    reason = f"triangulation: {error}"
                if valid:
                    accepted = self._handle_position_measurement(
                        int(left.class_id), point, covariance, pose,
                        FORM_FRONT_STEREO, min(
                            float(left.confidence), float(right.confidence)),
                        metrics | quality)
                    used_left.add(left_index)
                    used_right.add(right_index)
                    if accepted:
                        self._counters["front_stereo_accepted"] += 1
                    continue

            self._counters["front_pair_invalid"] += 1
            self._counters["front_stereo_rejected"] += 1
            # A rejected pair contributes at most one mono bearing. This
            # avoids counting the same invalid stereo frame twice.
            if self.use_rejected_front_pairs_for_multiview:
                self._process_front_mono_detection(left, "left", pose)
            used_left.add(left_index)
            used_right.add(right_index)
            if reason:
                self._warn_quality_once(reason)

        for index in unmatched_left:
            self._process_front_mono_detection(
                left_detections[index], "left", pose)
        for index in unmatched_right:
            # If this class exists on the left image, the left bearing is the
            # representative for this frame; otherwise preserve right-only data.
            det = right_detections[index]
            if int(det.class_id) not in left_classes:
                self._process_front_mono_detection(det, "right", pose)

        for index in range(len(left_detections)):
            if index not in used_left and index not in unmatched_left:
                self._process_front_mono_detection(
                    left_detections[index], "left", pose)

    def _process_front_single(self, message: DetectionArray, side: str):
        stamp = self._message_stamp(message)
        pose = self._lookup_pose(stamp)
        if pose is None or not self._calibration_ready:
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
        self._add_front_ray(
            int(detection.class_id), ray, float(detection.confidence), pose.stamp)

    def _process_down_pair(self, left_message: DetectionArray,
                           right_message: DetectionArray):
        stamp = self._pair_stamp(left_message, right_message)
        pose = self._lookup_pose(stamp)
        if pose is None or not self._calibration_ready:
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
            if self.down_plane_enabled:
                self._process_down_plane_pair(left, right, pose)
            else:
                try:
                    valid, reason, _ = self._stereo_geometry_valid(
                        left, right, self._down_calibration)
                    if not valid:
                        raise ValueError(reason)
                    point, covariance, quality = self._stereo_measurement(
                        "down", left, right, pose)
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

        if self.down_plane_enabled:
            for index in unmatched_left:
                self._process_down_plane_single(
                    left_detections[index], "left", pose)
            for index in unmatched_right:
                self._process_down_plane_single(
                    right_detections[index], "right", pose)

    def _process_down_single(self, message: DetectionArray, side: str):
        if not self.down_plane_enabled:
            return
        pose = self._lookup_pose(self._message_stamp(message))
        if pose is None or not self._calibration_ready:
            return
        for detection in self._detections_for_camera("down", message.detections):
            self._process_down_plane_single(detection, side, pose)

    def _process_down_plane_pair(self, left: Detection, right: Detection,
                                 pose: PoseAt):
        try:
            left_result = self._plane_measurement("left", left, pose)
        except (ValueError, cv2.error, np.linalg.LinAlgError) as error:
            left_result = None
            self._warn_quality_once(f"down plane left: {error}")
        try:
            right_result = self._plane_measurement("right", right, pose)
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
        accepted = self._handle_position_measurement(
            int(left.class_id), point, covariance, pose,
            FORM_DOWN_DIRECT, confidence, quality)
        if accepted:
            self._counters["down_direct_accepted"] += 1

    def _process_down_plane_single(self, detection: Detection, side: str,
                                   pose: PoseAt):
        try:
            result = self._plane_measurement(side, detection, pose)
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
                          calibration: StereoCalibration):
        candidates = []
        for left_index, left in enumerate(left_detections):
            for right_index, right in enumerate(right_detections):
                if int(left.class_id) != int(right.class_id):
                    continue
                left_center = np.array([left.pixel_x, left.pixel_y])
                right_center = np.array([right.pixel_x, right.pixel_y])
                try:
                    left_rect = calibration.rectified_pixel("left", left_center)
                    right_rect = calibration.rectified_pixel("right", right_center)
                    epipolar = abs(float(left_rect[1] - right_rect[1]))
                except (ValueError, cv2.error):
                    epipolar = abs(float(left.pixel_y - right.pixel_y))
                lw, lh = self._bbox_size(left)
                rw, rh = self._bbox_size(right)
                size_cost = abs(math.log(max(lw, 1e-3) / max(rw, 1e-3)))
                size_cost += abs(math.log(max(lh, 1e-3) / max(rh, 1e-3)))
                cost = epipolar + 5.0 * size_cost
                candidates.append((cost, left_index, right_index))

        candidates.sort(key=lambda item: item[0])
        used_left, used_right, pairs = set(), set(), []
        for _, left_index, right_index in candidates:
            if left_index in used_left or right_index in used_right:
                continue
            used_left.add(left_index)
            used_right.add(right_index)
            pairs.append((left_index, right_index))
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
        if self._bbox_at_edge(left) or self._bbox_at_edge(right):
            return False, "front pair rejected: bbox touches image edge", {}
        lw, lh = self._bbox_size(left)
        rw, rh = self._bbox_size(right)
        if min(lw, lh, rw, rh) < 2.0:
            return False, "front pair rejected: bbox too small", {}
        width_ratio = max(lw / rw, rw / lw)
        height_ratio = max(lh / rh, rh / lh)
        area_ratio = max((lw * lh) / (rw * rh), (rw * rh) / (lw * lh))
        if width_ratio > self.bbox_width_ratio_max:
            return False, "front pair rejected: bbox width mismatch", {}
        if height_ratio > self.bbox_height_ratio_max:
            return False, "front pair rejected: bbox height mismatch", {}
        if area_ratio > self.bbox_area_ratio_max:
            return False, "front pair rejected: bbox area mismatch", {}

        geometry_valid, geometry_reason, metrics = (
            self._stereo_geometry_valid(
                left, right, self._front_calibration))
        metrics.update({
            "bbox_width_ratio": width_ratio,
            "bbox_height_ratio": height_ratio,
            "bbox_area_ratio": area_ratio,
        })
        if not geometry_valid:
            return False, geometry_reason, metrics
        return True, "", metrics

    def _stereo_geometry_valid(self, left: Detection, right: Detection,
                               calibration: StereoCalibration):
        left_rect = calibration.rectified_pixel(
            "left", np.array([left.pixel_x, left.pixel_y]))
        right_rect = calibration.rectified_pixel(
            "right", np.array([right.pixel_x, right.pixel_y]))
        epipolar_error = abs(float(left_rect[1] - right_rect[1]))
        disparity = abs(float(left_rect[0] - right_rect[0]))
        metrics = {
            "epipolar_error_px": epipolar_error,
            "disparity_px": disparity,
        }
        if epipolar_error > self.epipolar_error_max:
            return False, "stereo pair rejected: epipolar error", metrics
        if disparity < self.min_disparity:
            return False, "stereo pair rejected: disparity too small", metrics
        return True, "", metrics

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

    def _pose_from_vector(self, value: np.ndarray) -> PoseAt:
        position = np.asarray(value[:3], dtype=np.float64)
        rotation = _rpy_to_rotation(*np.asarray(value[3:6], dtype=np.float64))
        covariance = np.zeros((6, 6), dtype=np.float64)
        return PoseAt(
            0.0, position, rotation, covariance, 0.0,
            float(value[3]), float(value[4]), float(value[5]))

    def _stereo_measurement(self, camera_pair: str, left: Detection,
                            right: Detection, pose: PoseAt):
        calibration = (
            self._front_calibration if camera_pair == "front"
            else self._down_calibration)
        pixels = np.array([
            float(left.pixel_x), float(left.pixel_y),
            float(right.pixel_x), float(right.pixel_y),
        ], dtype=np.float64)
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

        scale = (
            self.front_stereo_scale if camera_pair == "front"
            else self.down_direct_scale)
        covariance = _regularize_covariance(covariance * scale**2)
        depth_left = float(point_rectified[2])
        depth_right = float(
            calibration.triangulate_left_rectified(
                pixels[:2], pixels[2:])[2])
        if depth_left <= self.min_depth or depth_left > self.max_depth:
            raise ValueError(f"stereo depth {depth_left:.3f}m out of range")
        if not np.isfinite(depth_right):
            raise ValueError("right depth is non-finite")
        return point_world, covariance, {
            "depth_m": depth_left,
            "baseline_m": calibration.baseline_m,
        }

    @staticmethod
    def _pose_vector(pose: PoseAt) -> np.ndarray:
        # Numerical pose Jacobians use position plus the local RPY values.
        return np.r_[pose.position, pose.roll_deg, pose.pitch_deg, pose.yaw_deg]

    def _plane_world_from_pixel(self, camera_pair: str, side: str,
                                pixel: np.ndarray, pose_vector: np.ndarray):
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
        denominator = float(self.down_plane_normal @ direction)
        if abs(denominator) < 1e-5:
            raise ValueError("down ray is parallel to plane")
        scale = -(
            float(self.down_plane_normal @ origin) + self.down_plane_c
        ) / denominator
        if scale <= 0.0 or not np.isfinite(scale):
            raise ValueError("down plane intersection is behind camera")
        return origin + scale * direction

    def _plane_measurement(self, side: str, detection: Detection,
                           pose: PoseAt):
        pixels = np.array([detection.pixel_x, detection.pixel_y],
                          dtype=np.float64)
        pose_value = self._pose_vector(pose)
        point = self._plane_world_from_pixel(
            "down", side, pixels, pose_value)
        if not np.all(np.isfinite(point)):
            return None

        pixel_covariance = self._pixel_covariance(
            detection, self.down_width, self.down_height)
        pixel_covariance += np.eye(2) * self.calibration_pixel_sigma**2
        pixel_jacobian = _numeric_jacobian(
            lambda p: self._plane_world_from_pixel(
                "down", side, p, pose_value),
            pixels, np.full(2, 0.25))
        pose_jacobian = _numeric_jacobian(
            lambda p: self._plane_world_from_pixel(
                "down", side, pixels, p),
            pose_value, np.array([1e-3, 1e-3, 1e-3,
                                  1e-4, 1e-4, 1e-4]))
        covariance = pixel_jacobian @ pixel_covariance @ pixel_jacobian.T
        covariance += pose_jacobian @ pose.covariance @ pose_jacobian.T
        covariance += np.eye(3) * self.down_plane_sigma**2

        # Uncertainty of the camera mounting transform.
        camera = f"down_{side}"
        translation = self.body_translation[camera]
        rotation = self.body_rotation[camera]
        extrinsic_value = np.zeros(6)

        def point_from_extrinsic(value):
            calibration = self._down_calibration
            ray = calibration.ray_in_left_optical(side, pixels)
            translated = translation + value[:3]
            rotated = _axis_angle_rotation(value[3:]) @ rotation
            origin = pose.position + pose.rotation @ translated
            direction = pose.rotation @ (rotated @ ray)
            denominator = float(self.down_plane_normal @ direction)
            if abs(denominator) < 1e-5:
                raise ValueError("down ray became parallel to plane")
            scale = -(
                float(self.down_plane_normal @ origin) + self.down_plane_c
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
        covariance = _regularize_covariance(
            covariance * self.down_direct_scale**2)
        return point, covariance, {
            "plane_incidence": abs(
                float(self.down_plane_normal @ (
                    pose.rotation @ (
                        self.body_rotation[camera] @
                        self._down_calibration.ray_in_left_optical(
                            side, pixels)))))
        }

    def _make_ray(self, camera_pair: str, side: str,
                  detection: Detection, pose: PoseAt):
        calibration = (
            self._front_calibration if camera_pair == "front"
            else self._down_calibration)
        pixel = np.array([detection.pixel_x, detection.pixel_y],
                         dtype=np.float64)
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
        return origin_world, direction_world, sigma_angle

    def _add_front_ray(self, class_id: int, ray, confidence: float,
                       stamp: float):
        origin, direction, sigma_angle = ray
        track = self._associate_ray(class_id, origin, direction)
        if track is None:
            self._reject_instance_limit(class_id)
            return
        if not self._front_ray_has_new_baseline(track, direction):
            self._counters["front_multi_view_angle_rejected"] += 1
            return
        track.observed_class_ids.add(int(class_id))
        track.rays.append(RayObservation(
            stamp=stamp,
            origin=origin,
            direction=direction,
            sigma_angle=sigma_angle,
            confidence=confidence,
        ))
        while len(track.rays) > self.max_rays:
            track.rays.popleft()
        track.observation_count += 1
        track.front_multi_view_count += 1
        track.observation_form_mask |= FORM_FRONT_MULTI_VIEW
        track.last_observation_form = FORM_FRONT_MULTI_VIEW
        track.last_confidence = max(track.last_confidence, confidence)
        track.last_update_monotonic = time.monotonic()
        track.last_stamp = max(track.last_stamp, stamp)
        self._counters["front_multi_view_rays"] += 1
        self._record_ray_observation(
            track, class_id, stamp, origin, direction, confidence)

        if track.position is None:
            result = self._solve_rays(track)
            if result is not None:
                position, covariance = result
                track.position = position
                track.covariance = covariance
                track.last_stamp = stamp
        else:
            self._fuse_ray(track, origin, direction, sigma_angle)

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

    def _associate_ray(self, class_id: int, origin: np.ndarray,
                       direction: np.ndarray) -> TargetTrack | None:
        candidates = []
        semantic_class = self._semantic_class(class_id)
        for track in self._tracks.values():
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
                if angle <= self.ray_assoc_angle_rad:
                    candidates.append((angle + line_error / max(
                        distance, 0.1), track))
            elif track.rays:
                best_angle = min(
                    math.acos(np.clip(
                        float(np.dot(direction, ray.direction)), -1.0, 1.0))
                    for ray in track.rays)
                if best_angle <= self.ray_assoc_angle_rad:
                    candidates.append((best_angle + 0.5, track))
        if candidates:
            return min(candidates, key=lambda item: item[0])[1]
        return self._new_track(class_id)

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

        position = None
        inliers = rays
        for _ in range(3):
            normal_matrix = np.zeros((3, 3), dtype=np.float64)
            rhs = np.zeros(3, dtype=np.float64)
            for ray in inliers:
                projector = np.eye(3) - np.outer(ray.direction, ray.direction)
                if position is None:
                    sigma_perpendicular = 1.0
                else:
                    range_m = max(
                        float(np.linalg.norm(position - ray.origin)), 0.1)
                    sigma_perpendicular = max(
                        0.01, range_m * ray.sigma_angle)
                weight = max(ray.confidence, 0.05) / sigma_perpendicular**2
                normal_matrix += weight * projector
                rhs += weight * projector @ ray.origin
            if np.linalg.matrix_rank(normal_matrix, tol=1e-8) < 3:
                return None
            position = _safe_inverse(normal_matrix) @ rhs
            residuals = [
                float(np.linalg.norm(np.cross(
                    position - ray.origin, ray.direction)))
                for ray in rays
            ]
            median = float(np.median(residuals))
            threshold = max(self.max_line_error, 2.5 * median)
            inliers = [
                ray for ray, residual in zip(rays, residuals)
                if residual <= threshold
            ]
            if len(inliers) < 2:
                return None

        if position is None or not np.all(np.isfinite(position)):
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
                class_id, position, covariance, pose.stamp, confidence)
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
                track, position, np.eye(3), covariance, form, confidence)
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

    def _add_down_observation_pool(self, class_id: int,
                                   position: np.ndarray,
                                   covariance: np.ndarray,
                                   stamp: float,
                                   confidence: float):
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
        )
        pool.append(observation)
        return observation

    @staticmethod
    def _kmeans_horizontal(observations: list[DownDirectObservation],
                           cluster_count: int):
        """Deterministic weighted K-means using only the N/E coordinates."""
        points = np.asarray([observation.position[:2]
                             for observation in observations],
                            dtype=np.float64)
        weights = np.asarray([
            max(float(observation.confidence), 0.05)
            for observation in observations
        ], dtype=np.float64)
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

    def _cluster_to_existing_tracks(self, centers, existing_tracks):
        """Find the lowest-cost one-to-one mapping between clusters and tracks."""
        cluster_count = len(centers)
        track_count = len(existing_tracks)
        if not existing_tracks:
            return {}

        best_cost = float("inf")
        best_mapping = {}
        if cluster_count <= track_count:
            for selected_tracks in permutations(range(track_count),
                                                cluster_count):
                cost = sum(float(np.linalg.norm(
                    centers[cluster_index]
                    - existing_tracks[track_index].position[:2]))
                            for cluster_index, track_index
                            in enumerate(selected_tracks))
                if cost < best_cost:
                    best_cost = cost
                    best_mapping = {
                        cluster_index: existing_tracks[track_index]
                        for cluster_index, track_index
                        in enumerate(selected_tracks)
                    }
        else:
            for selected_clusters in permutations(range(cluster_count),
                                                  track_count):
                cost = sum(float(np.linalg.norm(
                    centers[cluster_index]
                    - existing_tracks[track_index].position[:2]))
                            for track_index, cluster_index
                            in enumerate(selected_clusters))
                if cost < best_cost:
                    best_cost = cost
                    best_mapping = {
                        cluster_index: existing_tracks[track_index]
                        for track_index, cluster_index
                        in enumerate(selected_clusters)
                    }
        return best_mapping

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
        for record in history:
            if (record.form != FORM_DOWN_DIRECT
                    or record.physical_class_name != semantic_class
                    or record.position is None):
                continue
            matching = [
                index for index, observation in enumerate(observations)
                if abs(float(record.stamp) - float(observation.stamp)) < 1e-6
            ]
            if not matching:
                continue
            index = min(
                matching,
                key=lambda item: float(np.linalg.norm(
                    record.position[:2] - observations[item].position[:2])))
            track = cluster_tracks.get(int(assignments[index]))
            if track is not None:
                record.instance_id = track.instance_id

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
            centers, existing_tracks)

        used_track_keys = {
            (track.physical_class_name, track.instance_id)
            for track in cluster_tracks.values()
        }
        for track in existing_tracks:
            if ((track.physical_class_name, track.instance_id)
                    not in used_track_keys):
                self._tracks.pop((track.physical_class_name,
                                  track.instance_id), None)

        for cluster_index in range(len(centers)):
            if cluster_index not in cluster_tracks:
                track = self._new_track(class_id)
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

    def _record_position_observation(self, track: TargetTrack,
                                     observed_class_id: int, stamp: float,
                                     position: np.ndarray,
                                     covariance: np.ndarray, form: int,
                                     confidence: float):
        """Retain a direct 3-D factor; this is not the fused track state."""
        self._observation_history.append(ObservationRecord(
            observation_id=self._next_observation_id,
            stamp=stamp,
            class_id=observed_class_id,
            instance_id=track.instance_id,
            physical_class_name=track.physical_class_name,
            form=form,
            confidence=confidence,
            position=np.asarray(position, dtype=np.float64).copy(),
            covariance=np.asarray(covariance, dtype=np.float64).copy(),
        ))
        self._next_observation_id += 1

    def _record_ray_observation(self, track: TargetTrack,
                                observed_class_id: int, stamp: float,
                                origin: np.ndarray, direction: np.ndarray,
                                confidence: float):
        """Retain a front bearing as a ray, without inventing a 3-D point."""
        self._observation_history.append(ObservationRecord(
            observation_id=self._next_observation_id,
            stamp=stamp,
            class_id=observed_class_id,
            instance_id=track.instance_id,
            physical_class_name=track.physical_class_name,
            form=FORM_FRONT_MULTI_VIEW,
            confidence=confidence,
            ray_origin=np.asarray(origin, dtype=np.float64).copy(),
            ray_direction=np.asarray(direction, dtype=np.float64).copy(),
        ))
        self._next_observation_id += 1

    def _linear_update(self, track: TargetTrack, measurement: np.ndarray,
                       jacobian: np.ndarray, covariance: np.ndarray,
                       form: int, confidence: float) -> bool:
        measurement = np.asarray(measurement, dtype=np.float64).reshape(-1)
        jacobian = np.asarray(jacobian, dtype=np.float64)
        covariance = _regularize_covariance(covariance)
        if track.position is None or track.covariance is None:
            if jacobian.shape == (3, 3) and np.allclose(
                    jacobian, np.eye(3)):
                track.position = measurement.copy()
                track.covariance = covariance.copy()
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
        if age > self.track_timeout:
            return int(TargetPosition.STATUS_STALE)
        if (
            track.observation_count >= self.minimum_stable_observations
            and float(np.trace(track.covariance)) <= self.stable_trace
        ):
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

        for track in self._tracks.values():
            if track.position is None or track.covariance is None:
                continue
            age = self._track_age(track)
            status = self._track_status(track, age)
            target = TargetPosition()
            target.class_id = int(track.class_id)
            target.instance_id = int(track.instance_id)
            target.class_name = self._class_name(track.class_id)
            target.physical_class_name = track.physical_class_name
            target.observed_class_ids = sorted(track.observed_class_ids)
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

            if status != int(TargetPosition.STATUS_STALE):
                compat_object = ObjectPosition()
                compat_object.class_id = int(track.class_id)
                compat_object.instance_id = int(track.instance_id)
                compat_object.class_name = self._class_name(track.class_id)
                compat_object.world_x = float(track.position[0])
                compat_object.world_y = float(track.position[1])
                compat_object.world_z = float(track.position[2])
                compat_object.confidence = float(self._track_confidence(track))
                compat_object.num_observations = int(track.observation_count)
                compat.objects.append(compat_object)

        for record in self._observation_history:
            observation = TargetObservation()
            observation.observation_id = int(record.observation_id)
            observation.observation_stamp.sec = int(record.stamp)
            observation.observation_stamp.nanosec = int(
                (record.stamp - int(record.stamp)) * 1e9)
            observation.class_id = int(record.class_id)
            observation.instance_id = int(record.instance_id)
            observation.class_name = self._class_name(record.class_id)
            observation.physical_class_name = record.physical_class_name
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
        self.get_logger().info(
            "localizer: tracks=%d down_pool=%s %s" % (
                len(self._tracks),
                {key: len(value) for key, value in
                 self._down_observation_pool.items()},
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
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
