"""Tests for the track-free front bearing pool and bearing estimator."""

import math
from types import SimpleNamespace

import numpy as np

from uv_camera.object_localizer import (
    FORM_FRONT_MULTI_VIEW,
    FrontPixelObservation,
    ObjectLocalizer,
    PoseAt,
    RayObservation,
)


class _Calibration:
    def ray_in_left_optical(self, side, pixel):
        value = np.asarray(pixel, dtype=np.float64)
        ray = np.array([value[0], value[1], 1.0], dtype=np.float64)
        return ray / np.linalg.norm(ray)


def _estimator_node():
    node = ObjectLocalizer.__new__(ObjectLocalizer)
    node.class_names = ["target"]
    node._front_tracks = {}
    node._front_next_instance_id = {}
    node._front_bearing_pool = {}
    node._front_observation_pool = {}
    node._observation_history = []
    node._next_observation_id = 1
    node._next_raw_observation_id = 1
    node._next_ray_id = 1
    node._last_detection_stamp = 0.0
    node._counters = {}
    node.front_observation_pool_size = 300
    node.front_bearing_seed_max_rays = 80
    node.front_bearing_seed_max_pairs = 2400
    node.front_bearing_cluster_radius = 0.35
    node.front_bearing_min_cluster_rays = 2
    node.front_bearing_max_clusters = 8
    node.front_bearing_clutter_likelihood = 0.08
    node.front_bearing_lm_iterations = 12
    node.front_bearing_lm_initial_damping = 1e-3
    node.front_bearing_track_match_distance = 2.0
    node.front_bearing_include_geometry_uncertainty = False
    node.front_ray_model_sigma_m = 0.0
    node.front_gate_model_sigma_m = 0.0
    node.calibration_pixel_sigma = 0.0
    node.front_raw_observation_window_size = 100
    node.front_multi_view_max_rays = 20
    node.max_instances_default = 4
    node.max_instances_guide_line = 6
    node.max_instances_gate = 4
    node.track_timeout = 2.0
    node.max_line_error = 0.5
    node.min_ray_angle_rad = math.radians(3.0)
    node.huber_delta = 2.5
    node.body_rotation = {"front_left": np.eye(3)}
    node._front_calibration = _Calibration()
    node._record_ray_observation = ObjectLocalizer._record_ray_observation.__get__(node)
    node._record_position_observation = (
        ObjectLocalizer._record_position_observation.__get__(node))
    return node


def _ray(origin, target, ray_id):
    origin = np.asarray(origin, dtype=np.float64)
    direction = np.asarray(target, dtype=np.float64) - origin
    direction /= np.linalg.norm(direction)
    return RayObservation(
        stamp=float(ray_id),
        origin=origin,
        direction=direction,
        sigma_angle=0.001,
        confidence=1.0,
        ray_id=ray_id,
        class_id=0,
        bearing_covariance=np.eye(2) * 0.001**2,
        observation_form=FORM_FRONT_MULTI_VIEW,
    )


def test_bearing_lm_recovers_position_and_covariance():
    node = _estimator_node()
    target = np.array([5.0, 2.0, 3.0])
    rays = [
        _ray([0.0, 0.0, 0.0], target, 1),
        _ray([0.0, 1.0, 0.0], target, 2),
        _ray([0.0, 0.0, 1.0], target, 3),
    ]

    estimate = node._optimize_front_bearing_cluster(
        rays, np.array([4.7, 1.8, 2.8]), np.ones(3))

    assert estimate is not None
    assert np.linalg.norm(estimate["position"] - target) < 1e-3
    assert np.all(np.isfinite(estimate["covariance"]))
    assert np.all(np.linalg.eigvalsh(estimate["covariance"]) > 0.0)


def test_batch_pool_creates_track_only_after_geometry_is_available():
    node = _estimator_node()
    target = np.array([5.0, 2.0, 3.0])
    rays = [
        _ray([0.0, 0.0, 0.0], target, 1),
        _ray([0.0, 1.0, 0.0], target, 2),
        _ray([0.0, 0.0, 1.0], target, 3),
    ]
    node._front_bearing_pool["target"] = rays

    node._rebuild_front_bearing_clusters("target")

    assert len(node._front_tracks) == 1
    track = next(iter(node._front_tracks.values()))
    assert np.linalg.norm(track.position - target) < 1e-3
    assert track.front_effective_observations >= 2.0


def test_raw_bearing_is_pooled_without_ray_to_track_association():
    node = _estimator_node()
    node._rebuild_front_bearing_clusters = lambda semantic_class: None
    node._associate_front_ray = lambda *args: (_ for _ in ()).throw(
        AssertionError("raw bearing must not use online track association"))
    pose = PoseAt(
        stamp=1.0,
        position=np.zeros(3),
        rotation=np.eye(3),
        covariance=np.zeros((6, 6)),
        age_sec=0.0,
    )
    raw = FrontPixelObservation(
        stamp=1.0,
        camera="front_left",
        pixel=np.array([0.0, 0.0]),
        covariance=np.eye(2),
        pose=pose,
        confidence=0.9,
        raw_observation_id=1,
        calibration=node._front_calibration,
    )
    node._add_front_ray(
        0,
        (np.zeros(3), np.array([1.0, 0.0, 0.0]), 0.01),
        0.9,
        1.0,
        raw_observation=raw,
        observation_form=FORM_FRONT_MULTI_VIEW,
    )

    assert len(node._front_bearing_pool["target"]) == 1
    assert not node._front_tracks
