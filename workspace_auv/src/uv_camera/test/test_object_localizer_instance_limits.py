"""Regression tests for target association and simulator calibration."""

import math
from collections import deque
from types import SimpleNamespace

import numpy as np

from uv_camera.object_localizer import (
    FORM_DOWN_DIRECT,
    FORM_FRONT_MULTI_VIEW,
    FORM_FRONT_STEREO,
    ObjectLocalizer,
    RayObservation,
    StereoCalibration,
)


def _localizer_for_association():
    """Create only the state required by pure association helpers."""
    node = ObjectLocalizer.__new__(ObjectLocalizer)
    node.class_names = [
        "red_ball", "blue_ball", "gate_down", "gate_front",
        "guide_line", "collection_frame_down", "collection_frame_front",
    ]
    node._tracks = {}
    node._next_instance_id = {}
    node.max_instances_default = 1
    node.max_instances_guide_line = 6
    node.max_instances_gate = 4
    node.position_gate_chi2 = 16.0
    node.down_direct_reanchor_chi2 = 9.0
    node.guide_line_min_spacing = 0.5
    node.ray_assoc_distance = 1.0
    node.ray_assoc_angle_rad = math.radians(25.0)
    node.max_line_error = 1.0
    node.front_multi_scale = 1.5
    node.min_ray_angle_rad = math.radians(5.0)
    node.min_depth = 0.05
    node.max_depth = 50.0
    node.ray_gate_chi2 = 16.0
    node.huber_delta = 2.5
    node.observation_history_size = 100
    node._observation_history = deque(maxlen=node.observation_history_size)
    node._down_observation_pool = {}
    node._front_tracks = {}
    node._front_next_instance_id = {}
    node._front_observation_pool = {}
    node.down_observation_pool_size = 2000
    node.front_observation_pool_size = 2000
    node.down_direct_queue_size = 50
    node.front_direct_queue_size = 50
    node.down_direct_queue_gate_chi2 = 16.0
    node.down_duplicate_merge_distance = 0.25
    node.front_duplicate_merge_distance = 0.25
    node._next_observation_id = 1
    node._last_detection_stamp = 0.0
    node._counters = {
        "camera_label_rejected": 0,
        "front_cluster_updates": 0,
        "front_pool_observations": 0,
        "front_multi_view_points": 0,
        "front_multi_view_angle_rejected": 0,
        "front_multi_view_rays": 0,
        "association_rejected": 0,
        "instance_limit_rejected": 0,
    }
    node.max_rays = 20
    return node


def test_competition_instance_limits():
    node = _localizer_for_association()

    for _ in range(4):
        assert node._new_track(2) is not None
    assert node._new_track(3) is None

    for _ in range(6):
        assert node._new_track(4) is not None
    assert node._new_track(4) is None

    assert node._new_track(0) is not None
    assert node._new_track(0) is None


def test_front_and_down_gate_share_one_physical_track():
    node = _localizer_for_association()
    down_track = node._new_track(2)
    down_track.position = np.array([4.0, 2.0, 1.0])
    down_track.covariance = np.eye(3) * 0.04

    matched = node._associate_position(
        3, np.array([4.05, 2.0, 1.0]), np.eye(3) * 0.04, np.zeros(3))

    assert matched is down_track
    assert down_track.physical_class_name == "gate"
    assert ("gate", 0) in node._tracks
    assert len(node._tracks) == 1


def test_front_and_down_detector_labels_bind_to_one_collection_frame():
    node = _localizer_for_association()
    down_track = node._new_track(5)
    down_track.position = np.array([1.0, -0.5, 0.8])
    down_track.covariance = np.eye(3) * 0.04

    matched = node._associate_position(
        6, np.array([1.03, -0.5, 0.8]), np.eye(3) * 0.04, np.zeros(3))

    assert matched is down_track
    assert down_track.physical_class_name == "collection_frame"
    assert ("collection_frame", 0) in node._tracks
    assert len(node._tracks) == 1


def test_far_down_measurement_reanchors_front_only_unique_track():
    node = _localizer_for_association()
    front_track = node._new_track(6)
    front_track.position = np.array([0.0, 0.0, 1.0])
    front_track.covariance = np.eye(3) * 0.01
    front_track.observed_class_ids = {6}
    front_track.front_stereo_count = 3
    front_track.front_multi_view_count = 2
    front_track.observation_count = 5
    front_track.observation_form_mask = 3
    front_track.rays.append(RayObservation(
        stamp=1.0, origin=np.zeros(3), direction=np.array([1.0, 0.0, 0.0]),
        sigma_angle=0.02, confidence=0.7))

    down_position = np.array([4.0, -1.0, 1.1])
    accepted = node._handle_position_measurement(
        5, down_position, np.eye(3) * 0.01,
        SimpleNamespace(position=np.zeros(3), stamp=2.0),
        FORM_DOWN_DIRECT, 0.95, {})

    assert accepted
    assert len(node._tracks) == 1
    assert front_track.class_id == 5
    assert front_track.observed_class_ids == {5}
    assert np.allclose(front_track.position, down_position)
    assert not front_track.rays
    assert front_track.front_stereo_count == 0
    assert front_track.front_multi_view_count == 0
    assert front_track.down_direct_count == 1
    assert front_track.observation_form_mask == FORM_DOWN_DIRECT
    assert node._counters["down_direct_reanchored"] == 1
    assert len(node._observation_history) == 1


def test_guide_line_instances_never_split_inside_half_meter_spacing():
    node = _localizer_for_association()
    first = node._new_track(4)
    first.position = np.array([1.0, 2.0, 1.0])
    first.covariance = np.eye(3) * 0.001

    near = node._associate_position(
        4, np.array([1.49, 2.0, 0.1]), np.eye(3) * 0.001,
        np.zeros(3), FORM_DOWN_DIRECT)
    assert near is first
    assert len(node._tracks) == 1

    distinct = node._associate_position(
        4, np.array([2.0, 2.0, 1.0]), np.eye(3) * 0.001,
        np.zeros(3), FORM_DOWN_DIRECT)
    assert distinct is not None
    assert distinct is not first
    assert len(node._tracks) == 2


def test_old_down_duplicate_tracks_are_merged_in_horizontal_plane():
    node = _localizer_for_association()
    first = node._new_track(4)
    second = node._new_track(4)
    first.position = np.array([1.0, 2.0, 1.0])
    first.covariance = np.eye(3) * 0.01
    first.down_direct_count = 2
    second.position = np.array([1.03, 2.02, 0.2])
    second.covariance = np.eye(3) * 0.01
    second.down_direct_count = 10

    node._merge_close_down_tracks()

    assert len(node._tracks) == 1
    assert ("guide_line", 0) in node._tracks
    assert np.allclose(node._tracks[("guide_line", 0)].position,
                       second.position)


def test_depth_jump_keeps_one_down_instance_and_accepts_later_good_point():
    node = _localizer_for_association()
    pose = SimpleNamespace(position=np.zeros(3), stamp=1.0)
    covariance = np.eye(3) * 0.01

    assert node._handle_position_measurement(
        4, np.array([1.0, 2.0, 1.0]), covariance, pose,
        FORM_DOWN_DIRECT, 0.8, {})
    assert node._handle_position_measurement(
        4, np.array([1.03, 2.02, 0.2]), covariance,
        SimpleNamespace(position=np.zeros(3), stamp=2.0),
        FORM_DOWN_DIRECT, 0.9, {})

    assert len(node._tracks) == 1
    track = next(iter(node._tracks.values()))
    assert track.down_direct_count == 2
    assert len(track.down_observations) == 2
    assert track.position[2] < 1.0


def test_unique_class_does_not_reject_later_position_after_initial_false_positive():
    node = _localizer_for_association()
    covariance = np.eye(3) * 0.01

    assert node._handle_position_measurement(
        5, np.array([1.0, 1.0, 1.0]), covariance,
        SimpleNamespace(position=np.zeros(3), stamp=1.0),
        FORM_DOWN_DIRECT, 0.4, {})
    assert node._handle_position_measurement(
        5, np.array([4.0, 3.0, 1.0]), covariance,
        SimpleNamespace(position=np.zeros(3), stamp=2.0),
        FORM_DOWN_DIRECT, 0.95, {})

    assert len(node._tracks) == 1
    track = next(iter(node._tracks.values()))
    assert track.down_direct_count == 2
    assert len(node._down_observation_pool["collection_frame"]) == 2


def test_guide_line_kmeans_keeps_only_horizontally_separated_clusters():
    node = _localizer_for_association()
    covariance = np.eye(3) * 0.01
    locations = [np.array([0.8 * index, 0.0, 0.5])
                 for index in range(6)]

    for stamp, location in enumerate(locations * 3, start=1):
        assert node._handle_position_measurement(
            4, location, covariance,
            SimpleNamespace(position=np.zeros(3), stamp=float(stamp)),
            FORM_DOWN_DIRECT, 0.9, {})

    assert len(node._tracks) == 6
    horizontal_positions = sorted(
        track.position[0] for track in node._tracks.values())
    assert np.allclose(horizontal_positions, [
        location[0] for location in locations
    ], atol=0.05)


def test_each_instance_keeps_fifty_down_samples_and_rejects_an_outlier():
    node = _localizer_for_association()
    track = node._new_track(5)
    for index in range(60):
        position = np.array([
            1.0 + 0.001 * (index % 2),
            -0.5 + 0.001 * ((index + 1) % 2),
            0.8,
        ])
        assert node._accept_down_direct_in_window(
            track, position, np.eye(3) * 0.01, float(index), 0.9)
    assert len(track.down_observations) == 50
    assert np.allclose(track.down_filter_position[:2], [1.0005, -0.4995], atol=0.01)

    assert not node._accept_down_direct_in_window(
        track, np.array([4.0, 3.0, 0.8]), np.eye(3) * 0.01, 61.0, 0.95)
    assert len(track.down_observations) == 50
    assert node._counters["down_direct_queue_rejected"] == 1


def test_front_pool_fuses_stereo_and_multiview_without_touching_down_tracks():
    node = _localizer_for_association()
    covariance = np.eye(3) * 0.01
    pose = SimpleNamespace(position=np.zeros(3), stamp=1.0)

    stereo = node._handle_front_position_measurement(
        0, np.array([3.0, 1.0, 0.8]), covariance, pose,
        FORM_FRONT_STEREO, 0.8, {})
    multi_view = node._handle_front_position_measurement(
        0, np.array([3.04, 1.02, 0.8]), covariance,
        SimpleNamespace(position=np.zeros(3), stamp=2.0),
        FORM_FRONT_MULTI_VIEW, 0.9, {})

    assert stereo is not None
    assert multi_view is stereo
    assert len(node._front_observation_pool["red_ball"]) == 2
    assert len(node._front_tracks) == 1
    track = next(iter(node._front_tracks.values()))
    assert track.front_stereo_count == 1
    assert track.front_multi_view_count == 1
    assert track.down_direct_count == 0
    assert node._tracks == {}


def test_front_pool_kmeans_uses_horizontal_plane_for_six_guide_lines():
    node = _localizer_for_association()
    covariance = np.eye(3) * 0.01
    locations = [np.array([0.8 * index, 0.0, 0.5])
                 for index in range(6)]

    for stamp, location in enumerate(locations * 3, start=1):
        assert node._handle_front_position_measurement(
            4, location, covariance,
            SimpleNamespace(position=np.zeros(3), stamp=float(stamp)),
            FORM_FRONT_STEREO, 0.9, {}) is not None

    assert len(node._front_tracks) == 6
    horizontal_positions = sorted(
        track.position[0] for track in node._front_tracks.values())
    assert np.allclose(horizontal_positions,
                       [location[0] for location in locations], atol=0.05)
    assert node._tracks == {}


def test_front_filter_trims_farthest_twenty_percent_after_twenty_samples():
    good = [SimpleNamespace(
        stamp=float(index),
        position=np.array([3.0 + 0.005 * (index % 3), 1.0, 0.8]),
        covariance=np.eye(3) * 0.01,
    ) for index in range(20)]
    outliers = [SimpleNamespace(
        stamp=float(20 + index),
        position=np.array([12.0 + index, -4.0, 3.0]),
        covariance=np.eye(3) * 0.01,
    ) for index in range(5)]
    observations = deque(good + outliers)

    filtered = ObjectLocalizer._trim_extreme_front_observations(observations)
    assert len(filtered) == 20
    assert len(ObjectLocalizer._trim_extreme_front_observations(
        deque(list(observations)[:20]))) == 20

    position, _ = ObjectLocalizer._fit_front_window(observations)
    assert np.linalg.norm(position - np.array([3.0, 1.0, 0.8])) < 0.05


def test_front_multiview_ray_intersection_enters_front_pool_only():
    node = _localizer_for_association()
    node._add_front_ray(
        0, (np.array([0.0, 0.0, 0.0]),
            np.array([1.0, 0.0, 0.0]), 0.01), 0.9, 1.0)
    node._add_front_ray(
        0, (np.array([0.0, 1.0, 0.0]),
            np.array([1.0, -0.2, 0.0]) / math.sqrt(1.04), 0.01), 0.9, 2.0)

    assert node._counters["front_multi_view_points"] == 1
    assert len(node._front_observation_pool["red_ball"]) == 1
    assert len(node._front_tracks) == 1
    assert node._tracks == {}


def test_camera_specific_labels_are_rejected_on_the_wrong_camera():
    node = _localizer_for_association()
    detections = [
        SimpleNamespace(class_id=2),  # gate_down
        SimpleNamespace(class_id=3),  # gate_front
        SimpleNamespace(class_id=4),  # camera-neutral guide_line
    ]

    front = node._detections_for_camera("front", detections)
    down = node._detections_for_camera("down", detections)

    assert [detection.class_id for detection in front] == [3, 4]
    assert [detection.class_id for detection in down] == [2, 4]
    assert node._counters["camera_label_rejected"] == 2


def test_front_multiview_queue_keeps_only_rays_separated_by_more_than_five_degrees():
    node = _localizer_for_association()
    track = node._new_track(0)
    forward = np.array([1.0, 0.0, 0.0])
    track.rays.append(RayObservation(
        stamp=0.0, origin=np.zeros(3), direction=forward,
        sigma_angle=0.01, confidence=1.0))

    def direction_at(angle_deg):
        angle = math.radians(angle_deg)
        return np.array([math.cos(angle), math.sin(angle), 0.0])

    assert not node._front_ray_has_new_baseline(track, direction_at(4.9))
    assert node._front_ray_has_new_baseline(track, direction_at(5.1))


def _sim_camera_info(focal: float):
    return SimpleNamespace(
        width=1280,
        height=960,
        k=[focal, 0.0, 640.0,
           0.0, focal, 480.0,
           0.0, 0.0, 1.0],
        d=[0.0] * 5,
        distortion_model="plumb_bob",
        header=SimpleNamespace(frame_id="sim_camera"),
    )


def test_simulator_stereo_profiles_use_scene_intrinsics_and_baselines():
    """xunyun_fixed.scn defines 0.10 m pairs and the two FOV values."""
    front_rotation = np.array([
        [0.0, 0.0, 1.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    ])
    down_rotation = np.array([
        [0.0, -1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
    ])

    profiles = (
        ("front", 57.19, front_rotation,
         [0.23, -0.05, 0.276], [0.23, 0.05, 0.276]),
        ("down", 87.19, down_rotation,
         [-0.13, -0.05, 0.2645], [-0.13, 0.05, 0.2645]),
    )
    for name, hfov_deg, rotation, left, right in profiles:
        focal = 640.0 / math.tan(math.radians(hfov_deg) / 2.0)
        calibration = StereoCalibration.from_camera_info(
            name, _sim_camera_info(focal), _sim_camera_info(focal),
            left, rotation, right, rotation)

        assert np.isclose(calibration.camera_matrix_left[0, 0], focal)
        assert np.isclose(calibration.camera_matrix_left[1, 1], focal)
        assert np.isclose(calibration.baseline_m, 0.1)
        assert np.isclose(abs(
            calibration.projection_right[0, 3]
            / calibration.projection_right[0, 0]), 0.1)


def test_simulator_stereo_extrinsics_reconstruct_front_and_down_world_points():
    """The active scene's optical-to-body transforms close the 3-D loop."""
    front_rotation = np.array([
        [0.0, 0.0, 1.0],
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
    ])
    down_rotation = np.array([
        [0.0, -1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
    ])
    profiles = (
        ("front", 1174.086809, front_rotation,
         np.array([0.23, -0.05, 0.276]),
         np.array([0.23, 0.05, 0.276]),
         np.array([4.0, 0.6, 0.7])),
        ("down", 672.183656, down_rotation,
         np.array([-0.13, -0.05, 0.2645]),
         np.array([-0.13, 0.05, 0.2645]),
         np.array([0.8, 0.7, 1.1])),
    )

    for name, focal, rotation, left, right, world_point in profiles:
        calibration = StereoCalibration.from_camera_info(
            name, _sim_camera_info(focal), _sim_camera_info(focal),
            left, rotation, right, rotation)

        def project(translation):
            optical = rotation.T @ (world_point - translation)
            assert optical[2] > 0.0
            return np.array([
                focal * optical[0] / optical[2] + 640.0,
                focal * optical[1] / optical[2] + 480.0,
            ])

        rectified_point = calibration.triangulate_left_rectified(
            project(left), project(right))
        recovered_world = left + rotation @ (
            calibration.rectification_left.T @ rectified_point)
        assert np.allclose(recovered_world, world_point, atol=1e-6)
