"""Unit tests for the bbox geometry and exclusive association primitives."""

import math

import numpy as np

from uv_camera.bbox_geometry import (
    CameraContext,
    CuboidModel,
    FrameModel,
    GateModel,
    SphereModel,
    bbox_measurement,
    bbox_measurement_covariance,
    bbox_residual,
    bbox_truncation_mask,
    exclusive_assignments,
    normalize_yaw_pi,
)
from uv_camera.object_localizer import FrontPixelObservation, ObjectLocalizer, PoseAt


def _camera():
    def project(point):
        point = np.asarray(point, dtype=np.float64)
        if point[2] <= 0.0:
            return None
        return np.array([
            320.0 + 200.0 * point[0] / point[2],
            240.0 + 200.0 * point[1] / point[2],
        ])

    return CameraContext(
        project=project,
        depth=lambda point: float(np.asarray(point)[2]),
        fx=200.0,
        fy=200.0,
    )


def test_bbox_measurement_uses_log_dimensions():
    value = bbox_measurement([300.0, 220.0, 340.0, 260.0])
    assert np.allclose(value, [320.0, 240.0, math.log(40.0), math.log(40.0)])
    covariance = bbox_measurement_covariance([300.0, 220.0, 340.0, 260.0], 2.0)
    assert covariance.shape == (4, 4)
    assert covariance[2, 2] < covariance[0, 0]


def test_gate_and_sphere_project_known_shapes():
    camera = _camera()
    sphere = SphereModel(0.1)
    assert np.allclose(
        sphere.project_bbox([0.0, 0.0, 2.0], camera),
        [310.0, 230.0, 330.0, 250.0],
    )
    gate = GateModel(0.7, 0.5)
    projected = gate.project_bbox([0.0, 0.0, 2.0, 0.0], camera)
    assert projected is not None
    assert projected[2] - projected[0] > projected[3] - projected[1]
    assert np.isclose(normalize_yaw_pi(math.pi), 0.0)
    assert FrameModel(0.4, 0.3, 0.3).name == "frame"


def test_truncation_masks_only_the_unreliable_axis():
    mask = bbox_truncation_mask([0.0, 100.0, 80.0, 180.0], 640, 480, 0.0)
    assert np.array_equal(mask, [False, True, False, True])


def test_bbox_residual_can_ignore_truncated_width():
    residual = bbox_residual(
        [0.0, 100.0, 80.0, 180.0],
        [10.0, 100.0, 90.0, 180.0],
        np.eye(4),
        valid_mask=[False, True, False, True],
    )
    assert residual is not None
    raw, whitened = residual
    assert np.allclose(raw, [0.0, 0.0])
    assert np.allclose(whitened, [0.0, 0.0])


def test_exclusive_assignment_keeps_duplicate_same_frame_as_clutter():
    assignments, costs = exclusive_assignments(
        np.array([[1.0, 10.0], [1.2, 10.0], [10.0, 1.0]]),
        clutter_cost=9.0,
        groups=["frame_a", "frame_a", "frame_a"],
    )
    assert sorted(value for value in assignments if value >= 0) == [0, 1]
    assert np.count_nonzero(assignments == -1) == 1
    assert np.all(costs[assignments == -1] == 9.0)


def test_localizer_bbox_lm_recovers_gate_from_multiple_capture_poses():
    class Calibration:
        projection_left = np.array(
            [[200.0, 0.0, 320.0, 0.0],
             [0.0, 200.0, 240.0, 0.0],
             [0.0, 0.0, 1.0, 0.0]])
        projection_right = projection_left.copy()
        camera_matrix_left = projection_left[:, :3]
        camera_matrix_right = camera_matrix_left.copy()
        dist_left = np.zeros(5)
        dist_right = np.zeros(5)

    node = ObjectLocalizer.__new__(ObjectLocalizer)
    node.body_translation = {
        "front_left": np.zeros(3),
        "front_right": np.array([0.0, 0.1, 0.0]),
    }
    node.body_rotation = {
        "front_left": np.eye(3), "front_right": np.eye(3),
    }
    node._front_calibration = Calibration()
    node.front_bbox_model_pixel_sigma = 4.0
    node.front_bbox_lm_iterations = 20
    node.front_bbox_lm_initial_damping = 1e-3
    node.huber_delta = 2.5
    node.edge_margin_px = 0.0

    model = GateModel(0.7, 0.5)
    target = np.array([0.0, 0.0, 2.0, 0.2])
    observations = []
    for index, (camera, position) in enumerate((
            ("front_left", np.array([0.0, 0.0, 0.0])),
            ("front_right", np.array([0.0, 0.1, 0.0])),
            ("front_left", np.array([0.15, 0.0, 0.0])),
            ("front_right", np.array([0.15, 0.1, 0.0])))):
        pose = PoseAt(index + 1.0, position, np.eye(3), np.zeros((6, 6)), 0.0)
        observation = FrontPixelObservation(
            stamp=pose.stamp, camera=camera, pixel=np.zeros(2),
            covariance=np.eye(2), pose=pose, confidence=0.9,
            bbox=np.zeros(4), image_size=(640, 480), class_id=0,
        )
        context = node._front_bbox_camera_context(observation)
        observation.bbox = model.project_bbox(target, context)
        observation.initialization_position = np.array([0.05, -0.03, 2.02])
        observations.append(observation)

    estimate = node._front_bbox_optimize(
        observations, model, np.array([0.05, -0.03, 2.02, 0.0]))

    assert estimate is not None
    assert np.linalg.norm(estimate["position"] - target[:3]) < 1e-3
    assert abs(normalize_yaw_pi(estimate["state"][3] - target[3])) < 1e-3


def test_sphere_bbox_generates_a_single_view_depth_seed():
    class Calibration:
        projection_left = np.array(
            [[200.0, 0.0, 320.0, 0.0],
             [0.0, 200.0, 240.0, 0.0],
             [0.0, 0.0, 1.0, 0.0]])
        projection_right = projection_left.copy()
        camera_matrix_left = projection_left[:, :3]
        camera_matrix_right = camera_matrix_left.copy()
        dist_left = np.zeros(5)
        dist_right = np.zeros(5)

        @staticmethod
        def ray_in_left_optical(side, pixel):
            pixel = np.asarray(pixel, dtype=np.float64)
            ray = np.array([(pixel[0] - 320.0) / 200.0,
                            (pixel[1] - 240.0) / 200.0, 1.0])
            return ray / np.linalg.norm(ray)

    node = ObjectLocalizer.__new__(ObjectLocalizer)
    node.class_names = ["impact_ball_red"]
    node.body_translation = {"front_left": np.zeros(3)}
    node.body_rotation = {"front_left": np.eye(3)}
    node._front_calibration = Calibration()
    node.front_impact_ball_radius = 0.1
    node.front_bbox_models_enabled = True
    node.front_observation_pool_size = 300
    node.max_instances_default = 1
    node.max_instances_gate = 4
    node.max_instances_guide_line = 6

    model = SphereModel(0.1, name="impact_ball")
    pose = PoseAt(1.0, np.zeros(3), np.eye(3), np.zeros((6, 6)), 0.0)
    observation = FrontPixelObservation(
        stamp=1.0, camera="front_left", pixel=np.array([320.0, 240.0]),
        covariance=np.eye(2), pose=pose, confidence=0.9,
        bbox=np.zeros(4), image_size=(640, 480), class_id=0,
    )
    context = node._front_bbox_camera_context(observation)
    observation.bbox = model.project_bbox([0.0, 0.0, 2.0], context)

    seeds = node._front_bbox_seed_states(
        "impact_ball_red", model, [observation])

    assert seeds
    assert min(np.linalg.norm(seed - [0.0, 0.0, 2.0]) for seed in seeds) < 0.05


def test_known_shape_birth_requires_unique_evidence_and_two_camera_poses():
    node = ObjectLocalizer.__new__(ObjectLocalizer)
    node.body_translation = {
        "front_left": np.zeros(3),
        "front_right": np.array([0.0, 0.1, 0.0]),
    }
    node.body_rotation = {
        "front_left": np.eye(3), "front_right": np.eye(3),
    }
    pose = PoseAt(1.0, np.zeros(3), np.eye(3), np.zeros((6, 6)), 0.0)

    def observation(raw_id, camera):
        return FrontPixelObservation(
            stamp=1.0, camera=camera, pixel=np.zeros(2),
            covariance=np.eye(2), pose=pose, confidence=0.9,
            raw_observation_id=raw_id,
            bbox=np.array([300.0, 220.0, 340.0, 260.0]),
        )

    same_pose = [observation(index, "front_left") for index in range(1, 4)]
    assert not node._front_bbox_has_birth_support(same_pose)
    stereo_supported = [
        observation(1, "front_left"),
        observation(2, "front_right"),
        observation(3, "front_left"),
    ]
    assert node._front_bbox_has_birth_support(stereo_supported)
