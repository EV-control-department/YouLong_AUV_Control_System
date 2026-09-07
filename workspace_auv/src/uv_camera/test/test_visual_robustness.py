"""Regression tests for bbox-only stereo and multi-view robustness."""

import math
from collections import deque
from types import SimpleNamespace

import cv2
import numpy as np
from std_msgs.msg import Header

from uv_camera.ai import Ai
from uv_camera.common import bgr_to_image_msg, image_msg_to_bgr
from uv_camera.object_localizer import (
    FrontPixelObservation,
    ObjectLocalizer,
    PoseAt,
    TargetTrack,
    _rpy_to_rotation,
    _slerp_rotation,
)
from uv_msgs.msg import DetectionArray, LineState


def _node():
    node = ObjectLocalizer.__new__(ObjectLocalizer)
    node.class_names = ["target"]
    node.front_width = 1280
    node.front_height = 960
    node.bbox_aspect_ratio_max = 1.6
    node.epipolar_error_max = 8.0
    node._counters = {}
    return node


def _detection(x, y, width=40.0, height=40.0):
    return SimpleNamespace(
        class_id=0,
        confidence=0.9,
        pixel_x=float(x),
        pixel_y=float(y),
        bbox_x1=float(x - width / 2.0),
        bbox_y1=float(y - height / 2.0),
        bbox_x2=float(x + width / 2.0),
        bbox_y2=float(y + height / 2.0),
    )


class _IdentityCalibration:
    @staticmethod
    def rectified_pixel(side, pixel):
        return np.asarray(pixel, dtype=np.float64)


class _PinholeCalibration:
    def __init__(self):
        self.camera_matrix_left = np.array(
            [[100.0, 0.0, 320.0], [0.0, 100.0, 240.0], [0.0, 0.0, 1.0]])
        self.camera_matrix_right = self.camera_matrix_left.copy()
        self.dist_left = np.zeros(5)
        self.dist_right = np.zeros(5)
        self.baseline_m = 0.1

    def ray_in_left_optical(self, side, pixel):
        pixel = np.asarray(pixel, dtype=np.float64)
        matrix = self.camera_matrix_left if side == "left" \
            else self.camera_matrix_right
        ray = np.array([
            (pixel[0] - matrix[0, 2]) / matrix[0, 0],
            (pixel[1] - matrix[1, 2]) / matrix[1, 1],
            1.0,
        ])
        return ray / np.linalg.norm(ray)

    def rectified_pixel(self, side, pixel):
        return np.asarray(pixel, dtype=np.float64)


def _pose(stamp, x=0.0):
    return PoseAt(
        stamp=float(stamp),
        position=np.array([x, 0.0, 0.0]),
        rotation=np.eye(3),
        covariance=np.zeros((6, 6)),
        age_sec=0.0,
    )


def test_front_match_leaves_severe_epipolar_mismatch_unmatched():
    node = _node()
    left = [_detection(400.0, 400.0)]
    right = [_detection(400.0, 420.0)]

    pairs, unmatched_left, unmatched_right = node._match_detections(
        left, right, _IdentityCalibration(), hard_front_geometry=True)

    assert pairs == []
    assert unmatched_left == [0]
    assert unmatched_right == [0]


def test_front_match_compares_left_right_bbox_shapes_not_object_shape():
    node = _node()
    # The target may be any shape.  Only the two camera observations must have
    # compatible bbox aspect ratios.
    left = [_detection(400.0, 400.0, width=80.0, height=20.0)]
    right = [_detection(400.0, 400.0, width=20.0, height=80.0)]

    pairs, unmatched_left, unmatched_right = node._match_detections(
        left, right, _IdentityCalibration(), hard_front_geometry=True)

    assert pairs == []
    assert unmatched_left == [0]
    assert unmatched_right == [0]


def test_front_match_uses_gate_feature_pixels_for_epipolar_check():
    node = _node()
    left_detection = _detection(400.0, 400.0)
    right_detection = _detection(400.0, 400.0)
    left_detection.class_id = 3
    right_detection.class_id = 3
    # The bbox centers agree, but the selected opening center does not.  The
    # correspondence gate must use the stable feature rather than silently
    # falling back to the detector rectangles.
    left_detection.feature_type = 1
    left_detection.feature_pixel_x = 400.0
    left_detection.feature_pixel_y = 400.0
    right_detection.feature_type = 1
    right_detection.feature_pixel_x = 400.0
    right_detection.feature_pixel_y = 420.0

    pairs, unmatched_left, unmatched_right = node._match_detections(
        [left_detection], [right_detection], _IdentityCalibration(),
        hard_front_geometry=True)

    assert pairs == []
    assert unmatched_left == [0]
    assert unmatched_right == [0]


def test_front_gate_match_also_compares_left_right_bbox_shapes():
    node = _node()
    left_detection = _detection(400.0, 400.0, width=80.0, height=20.0)
    right_detection = _detection(400.0, 400.0, width=20.0, height=80.0)
    left_detection.class_id = 3
    right_detection.class_id = 3

    pairs, unmatched_left, unmatched_right = node._match_detections(
        [left_detection], [right_detection], _IdentityCalibration(),
        hard_front_geometry=True)

    assert pairs == []
    assert unmatched_left == [0]
    assert unmatched_right == [0]


def test_front_raw_observation_uses_stable_gate_feature():
    node = _node()
    node.pixel_sigma_fraction = 0.08
    node.pixel_sigma_min = 1.0
    node.pixel_sigma_max = 12.0
    detection = _detection(400.0, 400.0)
    detection.class_id = 3
    detection.feature_type = 1
    detection.feature_pixel_x = 430.0
    detection.feature_pixel_y = 410.0

    observation = node._make_front_pixel_observation(
        detection, "left", _pose(1.0))

    assert np.allclose(observation.pixel, [430.0, 410.0])
    assert observation.feature_id == "gate_centerline"


def test_ai_centerline_anchor_uses_red_gate_frame_extent():
    image = np.zeros((120, 160, 3), dtype=np.uint8)
    red = (0, 0, 255)  # BGR
    cv2.line(image, (35, 35), (35, 85), red, 5)
    cv2.line(image, (125, 35), (125, 85), red, 5)
    cv2.line(image, (35, 35), (125, 35), red, 5)
    cv2.line(image, (35, 85), (125, 85), red, 5)

    center = Ai._centerline_from_red_pipes(image, (20, 20, 140, 100))

    assert center is not None
    assert np.allclose(center, [80.0, 60.0], atol=2.0)


def test_front_match_rejects_non_positive_bbox_dimensions_before_pairing():
    node = _node()
    left = [_detection(400.0, 400.0, width=0.0, height=40.0)]
    right = [_detection(400.0, 400.0)]

    pairs, unmatched_left, unmatched_right = node._match_detections(
        left, right, _IdentityCalibration(), hard_front_geometry=True)

    assert pairs == []
    assert unmatched_left == [0]
    assert unmatched_right == [0]


def test_front_match_requires_exact_class_id():
    node = _node()
    left = [_detection(400.0, 400.0)]
    right = [_detection(400.0, 400.0)]
    right[0].class_id = 1

    pairs, unmatched_left, unmatched_right = node._match_detections(
        left, right, _IdentityCalibration(), hard_front_geometry=True)

    assert pairs == []
    assert unmatched_left == [0]
    assert unmatched_right == [0]


def test_stereo_pair_id_matches_true_capture_times_without_frame_stealing():
    node = ObjectLocalizer.__new__(ObjectLocalizer)
    node.stereo_sync_slop = 0.01
    node._pending = {
        "front_left": deque(), "front_right": deque(),
        "down_left": deque(), "down_right": deque(),
    }
    processed = []
    node._process_front_pair = lambda left, right: processed.append((left, right))

    def message(sec, nanosec, pair_id):
        return SimpleNamespace(
            header=SimpleNamespace(
                stamp=SimpleNamespace(sec=sec, nanosec=nanosec)),
            stereo_pair_id=pair_id,
            detections=[],
        )

    left = message(10, 0, 4)
    right = message(10, 100_000_000, 4)
    node._pending["front_left"].append((0.0, left))
    node._pending["front_right"].append((0.0, right))

    node._try_pair("front")

    assert processed == [(left, right)]
    assert not node._pending["front_left"]
    assert not node._pending["front_right"]


def test_ai_forwards_right_capture_stamp_and_pair_id_to_detector():
    ai = Ai.__new__(Ai)
    captured = []
    ai._process_frame = lambda *args: captured.append(args)
    left_stamp = SimpleNamespace(sec=10, nanosec=0)
    right_stamp = SimpleNamespace(sec=10, nanosec=100_000_000)
    frame = np.zeros((2, 4, 3), dtype=np.uint8)

    ai.process(
        "front",
        ("opencv", frame, left_stamp, False, right_stamp, 7),
    )

    assert len(captured) == 1
    assert captured[0][1] is frame
    assert captured[0][2] == "front"
    assert captured[0][3] is right_stamp
    assert captured[0][4] == 7


def test_ai_process_frame_can_build_right_header_after_model_load():
    ai = Ai.__new__(Ai)
    ai.node = SimpleNamespace(stream_requested=lambda *args: False)
    ai._active_channels = {"front_left"}
    ai._model_loaded = True
    ai._allow_inference = lambda camera: True
    ai._front_K = None
    ai._front_D = None
    ai._save_dataset = False
    ai._aruco_detector = None
    ai._pub_det = {"front_left": SimpleNamespace(publish=lambda msg: None),
                   "front_right": SimpleNamespace(publish=lambda msg: None)}
    ai._pub_line = {"front_left": SimpleNamespace(publish=lambda msg: None),
                    "front_right": SimpleNamespace(publish=lambda msg: None)}
    ai._detect = lambda header, camera, image, pair_id: (
        DetectionArray(), [], LineState(), {})
    ai._update_annotated = lambda *args: None

    # Before the fix this call raised NameError: Header is not defined when
    # _process_frame constructs the right-eye header.
    ai._process_frame(Header(), np.zeros((4, 8, 3), dtype=np.uint8), "front")


def test_ros_image_codec_round_trips_bgr_without_cv_bridge():
    frame = np.arange(24, dtype=np.uint8).reshape(2, 4, 3)
    message = bgr_to_image_msg(frame)

    decoded = image_msg_to_bgr(message)

    assert np.array_equal(decoded, frame)


def test_front_raw_observations_have_unique_source_ids():
    node = _node()
    node.pixel_sigma_fraction = 0.08
    node.pixel_sigma_min = 1.0
    node.pixel_sigma_max = 12.0

    left = node._make_front_pixel_observation(
        _detection(400.0, 400.0), "left", _pose(1.0))
    right = node._make_front_pixel_observation(
        _detection(400.0, 400.0), "right", _pose(1.0))

    assert left.raw_observation_id > 0
    assert right.raw_observation_id == left.raw_observation_id + 1


def test_pose_rotation_interpolation_uses_shortest_slerp_path():
    first = _rpy_to_rotation(0.0, 0.0, 179.0)
    second = _rpy_to_rotation(0.0, 0.0, -179.0)

    halfway = _slerp_rotation(first, second, 0.5)

    assert np.allclose(
        halfway, _rpy_to_rotation(0.0, 0.0, 180.0), atol=1e-6)


def test_multiview_replaces_one_pool_slot_when_old_rays_are_reused():
    node = ObjectLocalizer.__new__(ObjectLocalizer)
    node.class_names = ["target"]
    node._front_tracks = {}
    node._front_next_instance_id = {}
    node._front_observation_pool = {}
    node.front_observation_pool_size = 300
    node.front_direct_queue_size = 50
    node.front_duplicate_merge_distance = 0.25
    node.guide_line_min_spacing = 0.5
    node.max_instances_default = 1
    node.max_instances_guide_line = 6
    node.max_instances_gate = 4
    node.position_gate_chi2 = 16.0
    node.min_ray_angle_rad = math.radians(5.0)
    node.ray_assoc_angle_rad = math.radians(25.0)
    node.ray_assoc_distance = 1.0
    node.max_rays = 20
    node.max_line_error = 1.0
    node.front_multi_scale = 1.5
    node.min_depth = 0.05
    node.ray_gate_chi2 = 16.0
    node.huber_delta = 2.5
    node.observation_history_size = 100
    node._observation_history = []
    node._next_observation_id = 1
    node._last_detection_stamp = 0.0
    node._counters = {
        "front_multi_view_angle_rejected": 0,
        "front_multi_view_rays": 0,
        "front_multi_view_points": 0,
        "front_pool_observations": 0,
        "front_cluster_updates": 0,
        "front_cluster_support_rejected": 0,
    }

    node._add_front_ray(
        0,
        (np.array([0.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0]), 0.01),
        0.9,
        1.0,
    )
    node._add_front_ray(
        0,
        (np.array([0.0, 1.0, 0.0]),
         np.array([1.0, -0.2, 0.0]) / math.sqrt(1.04), 0.01),
        0.9,
        2.0,
    )
    node._add_front_ray(
        0,
        (np.array([0.0, -1.0, 0.0]),
         np.array([1.0, 0.2, 0.0]) / math.sqrt(1.04), 0.01),
        0.9,
        3.0,
    )

    assert len(node._front_observation_pool["target"]) == 1
    assert node._counters["front_pool_observations"] == 1
    assert len(node._front_tracks) == 1


def test_cluster_assignment_can_leave_a_distant_cluster_unmatched():
    node = _node()
    node.position_gate_chi2 = 16.0
    track = SimpleNamespace(
        position=np.array([0.0, 0.0, 1.0]),
        covariance=np.eye(3) * 0.01,
    )

    mapping = node._cluster_to_existing_tracks(
        np.array([[10.0, 10.0]]), [track], [], np.array([], dtype=np.int32))

    assert mapping == {}


def test_async_stereo_uses_left_and_right_capture_poses():
    node = ObjectLocalizer.__new__(ObjectLocalizer)
    node._front_calibration = _PinholeCalibration()
    node.body_translation = {
        "front_left": np.zeros(3),
        "front_right": np.array([0.1, 0.0, 0.0]),
    }
    node.body_rotation = {
        "front_left": np.eye(3),
        "front_right": np.eye(3),
    }
    node.front_width = node.front_height = 640
    node.pixel_sigma_fraction = 0.01
    node.pixel_sigma_min = 1.0
    node.pixel_sigma_max = 12.0
    node.calibration_pixel_sigma = 0.0
    node.pose_position_covariance = np.eye(3) * 0.03**2
    node.pose_angle_covariance = np.eye(3) * math.radians(1.0)**2
    node.extrinsic_position_covariance = np.eye(3) * 0.005**2
    node.extrinsic_angle_covariance = np.eye(3) * math.radians(0.5)**2
    node.pose_position_sigma = 0.03
    node.extrinsic_position_sigma = 0.005
    node.pose_angle_sigma_rad = math.radians(1.0)
    node.extrinsic_angle_sigma_rad = math.radians(0.5)
    node.shared_error_scale = 1.0
    node.front_stereo_scale = 1.0
    node.front_stereo_trusted_min_range = 0.5
    node.front_stereo_trusted_max_range = 2.5
    node.front_stereo_out_of_range_scale = 6.0
    node.min_depth = 0.05
    node.max_depth = 2.0
    node.min_disparity = 2.0
    node.epipolar_error_max = 8.0

    target = np.array([0.0, 0.0, 2.0])
    left_pixel = np.array([320.0, 240.0])
    # The vehicle moved +0.05 m before the right image, so the right camera
    # origin is x=0.15 m rather than the simultaneous x=0.10 m origin.
    right_pixel = np.array([312.5, 240.0])
    left = _detection(*left_pixel)
    right = _detection(*right_pixel)

    point, _, quality = node._stereo_measurement(
        "front", left, right, _pose(1.0), right_pose=_pose(1.1, 0.05))

    assert quality["async_stereo"] is True
    assert np.allclose(point, target, atol=1e-5)


def test_unified_front_solver_uses_raw_left_right_and_motion_observations():
    node = ObjectLocalizer.__new__(ObjectLocalizer)
    node._front_calibration = _PinholeCalibration()
    node.body_translation = {
        "front_left": np.zeros(3),
        "front_right": np.array([0.1, 0.0, 0.0]),
    }
    node.body_rotation = {
        "front_left": np.eye(3),
        "front_right": np.eye(3),
    }
    node.front_raw_observation_window_size = 100
    node.huber_delta = 2.5
    node.pose_position_sigma = 0.03
    node.extrinsic_position_sigma = 0.005
    node.pose_angle_sigma_rad = math.radians(1.0)
    node.extrinsic_angle_sigma_rad = math.radians(0.5)
    node.shared_error_scale = 1.0
    node._front_calibration = _PinholeCalibration()

    target = np.array([0.0, 0.0, 2.0])
    observations = []
    for camera, pose in (
            ("front_left", _pose(1.0)),
            ("front_right", _pose(1.0)),
            ("front_left", _pose(2.0, 0.2))):
        camera_translation = node.body_translation[camera]
        point_camera = pose.rotation.T @ (target - pose.position)
        point_camera -= camera_translation
        pixel = np.array([
            320.0 + 100.0 * point_camera[0] / point_camera[2],
            240.0 + 100.0 * point_camera[1] / point_camera[2],
        ])
        observations.append(FrontPixelObservation(
            stamp=pose.stamp,
            camera=camera,
            pixel=pixel,
            covariance=np.eye(2) * 0.5**2,
            pose=pose,
            confidence=0.9,
        ))

    track = TargetTrack(
        class_id=0,
        instance_id=0,
        physical_class_name="target",
        position=np.array([0.2, -0.1, 2.3]),
        covariance=np.eye(3),
    )
    track.front_pixel_observations = deque(observations)

    assert node._optimize_front_track(track)
    assert np.allclose(track.position, target, atol=1e-3)
    assert np.all(np.isfinite(track.covariance))
