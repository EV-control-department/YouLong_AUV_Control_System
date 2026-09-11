"""Pure regression tests for camera-only gate geometry."""

from collections import deque
import threading
from types import SimpleNamespace

import numpy as np

from importlib import import_module

_gate = import_module('uv_task.26rb_gate_task')
GateTask = _gate.RB26GateTask
GateCandidate = _gate.GateCandidate
GateObservation = _gate.GateObservation
_DEFAULT_K = _gate._DEFAULT_K
_FRONT_OFFSET_LEFT = _gate._FRONT_OFFSET_LEFT
_FRONT_OFFSET_RIGHT = _gate._FRONT_OFFSET_RIGHT
_OPTICAL_TO_BODY = _gate._OPTICAL_TO_BODY
def _project(point, offset):
    camera = _OPTICAL_TO_BODY.T @ (np.asarray(point) - offset)
    return np.array([
        _DEFAULT_K[0, 0] * camera[0] / camera[2] + _DEFAULT_K[0, 2],
        _DEFAULT_K[1, 1] * camera[1] / camera[2] + _DEFAULT_K[1, 2],
    ])


def test_stereo_gate_uses_front_extrinsics_for_body_geometry():
    # Four corners of a 0.70 x 0.50 m gate two metres in front of the body.
    corners_body = np.array([
        [2.0, -0.35, -0.25],
        [2.0, 0.35, -0.25],
        [2.0, 0.35, 0.25],
        [2.0, -0.35, 0.25],
    ])
    left_pixels = np.array([
        _project(point, _FRONT_OFFSET_LEFT) for point in corners_body])
    right_pixels = np.array([
        _project(point, _FRONT_OFFSET_RIGHT) for point in corners_body])

    def candidate(pixels):
        return GateCandidate(
            center=pixels.mean(axis=0), corners=pixels,
            width_px=700.0, height_px=500.0,
            extent_fraction=0.5, frame_score=0.5)

    task = GateTask.__new__(GateTask)
    task._front_left_offset = _FRONT_OFFSET_LEFT
    task._front_right_offset = _FRONT_OFFSET_RIGHT
    task._optical_to_body = _OPTICAL_TO_BODY
    observation = task._make_observation(
        candidate(left_pixels), candidate(right_pixels),
        _DEFAULT_K, _DEFAULT_K)

    assert observation is not None
    assert np.allclose(observation.center_body, [2.0, 0.0, 0.0], atol=0.02)
    assert np.allclose(observation.normal_body, [1.0, 0.0, 0.0], atol=0.02)


def test_left_eye_observation_uses_left_camera_extrinsics():
    task = GateTask.__new__(GateTask)
    task._front_left_offset = _FRONT_OFFSET_LEFT
    task._optical_to_body = _OPTICAL_TO_BODY
    task._min_extent = 0.25
    task._target_extent = 0.75
    task._monocular_reference_distance = 1.5
    task._distance_control_min_m = 0.5
    task._distance_control_max_m = 5.0
    candidate = GateCandidate(
        center=np.array([640.0, 480.0]),
        bbox_center=np.array([760.0, 520.0]),
        corners=np.zeros((4, 2)), width_px=600.0, height_px=500.0,
        extent_fraction=0.5, frame_score=0.9)

    observation = task._make_left_observation(candidate, _DEFAULT_K)

    assert observation.monocular
    assert observation.left is observation.right
    assert np.allclose(observation.center_px, candidate.bbox_center)
    origin, ray = task._ray_in_body(
        candidate.bbox_center, _DEFAULT_K, _FRONT_OFFSET_LEFT)
    assert np.allclose(
        observation.center_body,
        origin + ray * (1.5 * 0.75 / 0.5),
        atol=1e-9)


def test_locked_gate_does_not_switch_to_a_larger_distant_candidate():
    task = GateTask.__new__(GateTask)
    task._last_observation_reason = ''

    def candidate(center, extent):
        center = np.asarray(center, dtype=np.float64)
        return GateCandidate(
            center=center,
            bbox_center=center,
            corners=np.zeros((4, 2)),
            width_px=500.0,
            height_px=extent * 960.0,
            extent_fraction=extent,
            frame_score=0.9,
        )

    locked = candidate([520.0, 480.0], 0.45)
    same_gate = candidate([550.0, 490.0], 0.55)
    larger_other_gate = candidate([1120.0, 480.0], 0.90)

    selected = task._select_locked_candidate(
        [larger_other_gate, same_gate], locked)

    assert selected is same_gate


def _stereo_observation(left_center, right_center, extent=0.60,
                        left_bbox_center=None, right_bbox_center=None):
    def candidate(center, bbox_center):
        return GateCandidate(
            center=np.asarray(center, dtype=np.float64),
            corners=np.zeros((4, 2)),
            width_px=500.0,
            height_px=500.0,
            extent_fraction=extent,
            frame_score=0.5,
            bbox_center=(None if bbox_center is None else
                         np.asarray(bbox_center, dtype=np.float64)),
        )

    left = candidate(left_center, left_bbox_center)
    right = candidate(right_center, right_bbox_center)
    return GateObservation(
        center_body=np.array([2.0, 0.0, 0.0]),
        normal_body=np.array([1.0, 0.0, 0.0]),
        distance_m=2.0,
        extent_fraction=extent,
        center_px=0.5 * (left.center + right.center),
        left=left,
        right=right,
    )


def test_stereo_centering_uses_midpoint_not_raw_left_right_pixel_match():
    task = GateTask.__new__(GateTask)
    task._camera_k = {'left': _DEFAULT_K.copy(), 'right': _DEFAULT_K.copy()}
    task._image_center_tolerance = 0.04
    task._stereo_vertical_tolerance = 0.04

    # The +/-10 pixel horizontal disparity is expected from the stereo
    # baseline.  The calibrated midpoint is exactly at the optical centre.
    observation = _stereo_observation([650.0, 480.0], [630.0, 480.0])

    errors = task._image_center_errors(observation)
    assert errors['u'] == 0.0
    assert task._image_is_centered(observation)


def test_yaw_pid_uses_stereo_center_error():
    task = GateTask.__new__(GateTask)
    task._camera_k = {'left': _DEFAULT_K.copy(), 'right': _DEFAULT_K.copy()}
    task._velocity_period = 0.1
    task._max_yaw_rate = 36.0
    task._yaw_pid_kp = 3.0
    task._yaw_pid_ki = 0.0
    task._yaw_pid_kd = 0.0
    task._yaw_pid_integral_limit_deg = 10.0
    task._image_center_tolerance = 0.04
    task._stereo_vertical_tolerance = 0.04
    task._yaw_pid_integral = 0.0
    task._yaw_pid_previous_error = None
    task._yaw_pid_previous_time = None

    observation = _stereo_observation(
        [680.0, 480.0], [660.0, 480.0],
        left_bbox_center=[680.0, 480.0],
        right_bbox_center=[660.0, 480.0])
    errors = task._image_center_errors(observation)
    output, error_deg, derivative = task._yaw_pid_update(errors['u'], 0.0)

    assert error_deg > 0.0
    assert derivative == 0.0
    assert np.isclose(output, 3.0 * error_deg)


def test_arc_servo_uses_lateral_arc_and_center_feedback():
    task = GateTask.__new__(GateTask)
    task._camera_k = {'left': _DEFAULT_K.copy(), 'right': _DEFAULT_K.copy()}
    task._distance_control_min_m = 0.5
    task._distance_control_max_m = 5.0
    task._target_extent = 0.75
    task._target_height_fraction = 2.0 / 3.0
    task._distance_velocity_gain = 0.45
    task._vertical_velocity_gain = 0.55
    task._yaw_velocity_gain = 0.8
    task._max_forward_speed = 0.18
    task._max_reverse_speed = 0.12
    task._max_lateral_speed = 0.18
    task._max_vertical_speed = 0.12
    task._max_yaw_rate = 12.0
    task._bbox_filter_alpha = 1.0
    task._filtered_gate = None
    task._arc_direction = 1.0
    task._arc_lateral_speed = 0.08
    task._arc_near_extremum = False
    task._arc_near_extremum_speed_scale = 0.5
    task._yaw_center_kp = 1.2
    task._image_center_tolerance = 0.04

    # The geometric anchor is centred, but the detector bbox is to the right.
    # The arc search owns vy; bbox center only contributes to yaw feedback.
    observation = _stereo_observation(
            [640.0, 480.0], [640.0, 480.0], extent=0.75,
            left_bbox_center=[760.0, 480.0],
            right_bbox_center=[740.0, 480.0])

    filtered = task._filter_gate(observation)
    forward, lateral, vertical, yaw_rate = task._servo_velocity(
        observation, filtered, height_output=0.0,
        height_active=False, attitude_active=True)
    assert forward >= 0.0
    expected_forward = 0.45 * ((2.0 / 3.0) - 500.0 / 960.0) * 2.0
    assert np.isclose(forward, expected_forward)
    assert np.isclose(lateral, 0.08)
    assert vertical == 0.0
    expected_yaw = np.degrees(
        -0.08 / 2.0 + 1.2 * np.arctan(filtered.center_u))
    assert np.isclose(yaw_rate, expected_yaw)


def test_arc_direction_updates_only_at_the_end_of_a_window():
    task = GateTask.__new__(GateTask)

    class Logger:
        def info(self, _message):
            pass

    task._logger = Logger()
    task._arc_probe_window_seconds = 2.0
    task._arc_window_seconds = 5.0
    task._arc_objective_deadband = 0.001
    task._arc_reverse_cooldown = 0.5
    task._arc_last_reversal_time = float('-inf')
    task._arc_direction = 1.0
    task._arc_near_extremum = False
    task._arc_window_start_objective = None
    task._arc_window_start_observation_angle = None
    task._arc_window_start_time = None
    task._arc_initial_window_pending = True

    start = type('Filtered', (), {
        'width_px': 500.0, 'height_px': 500.0, 'center_u': 0.0})()
    worse = type('Filtered', (), {
        'width_px': 400.0, 'height_px': 500.0, 'center_u': 0.0})()

    task._update_arc_window(start, now=0.0)
    _, _, mid_delta = task._update_arc_window(worse, now=1.9)
    assert mid_delta == 0.0
    assert task._arc_direction == 1.0

    worse.center_u = 0.1
    _, _, end_delta = task._update_arc_window(worse, now=2.0)
    assert np.isclose(end_delta, -0.2)
    assert task._arc_direction == -1.0
    assert np.isclose(
        task._arc_window_last_observation_angle_delta,
        np.degrees(np.arctan(0.1)))

    # The first probe is two seconds; subsequent gradient windows use five.
    _, _, next_mid_delta = task._update_arc_window(start, now=6.9)
    assert next_mid_delta == 0.0
    _, _, next_end_delta = task._update_arc_window(start, now=7.0)
    assert np.isclose(next_end_delta, 0.2)


def test_detection_pair_recovers_previous_matching_id_from_short_history():
    """左右目回调错位时，不应丢掉上一组仍然新鲜的双目帧。"""
    task = GateTask.__new__(GateTask)
    task._lock = threading.RLock()
    task._detection_timeout = 1.5
    task._pair_slop = 0.25
    task._last_observation_reason = ''
    task._detection_history = {
        'left': deque(maxlen=16),
        'right': deque(maxlen=16),
    }
    task._latest_left_detections = None
    task._latest_right_detections = None

    def message(pair_id):
        return SimpleNamespace(stereo_pair_id=pair_id, detections=[])

    # 最新状态是 left=11/right=10，旧的 pair=10 仍在缓存中。
    task._left_detection_cb(message(10))
    task._right_detection_cb(message(10))
    task._left_detection_cb(message(11))

    pair = task._detection_pair()

    assert pair is not None
    assert pair[0].stereo_pair_id == 10
    assert pair[1].stereo_pair_id == 10


def test_clipped_gate_does_not_claim_a_reliable_plane_normal():
    task = GateTask.__new__(GateTask)
    task._front_left_offset = _FRONT_OFFSET_LEFT
    task._front_right_offset = _FRONT_OFFSET_RIGHT
    task._optical_to_body = _OPTICAL_TO_BODY

    def candidate(center):
        return GateCandidate(
            center=np.asarray(center, dtype=np.float64),
            corners=np.array([
                [0.0, 0.0], [1279.0, 0.0],
                [1279.0, 959.0], [0.0, 959.0],
            ]),
            width_px=1279.0,
            height_px=959.0,
            extent_fraction=0.90,
            frame_score=0.5,
            clipped=True,
        )

    observation = task._make_observation(
        candidate([650.0, 480.0]), candidate([630.0, 480.0]),
        _DEFAULT_K, _DEFAULT_K)

    assert observation is not None
    assert not observation.normal_reliable


def test_forward_distance_command_is_disabled_until_stereo_is_centered():
    task = GateTask.__new__(GateTask)
    task._camera_k = {'left': _DEFAULT_K.copy(), 'right': _DEFAULT_K.copy()}
    task._image_center_tolerance = 0.04
    task._stereo_vertical_tolerance = 0.04
    task._distance_control_min_m = 0.5
    task._distance_control_max_m = 5.0
    task._target_extent = 0.75
    task._target_height_fraction = 2.0 / 3.0
    task._distance_gain = 0.8
    task._max_forward_step = 0.12
    task._max_back_step = 0.08
    task._lateral_gain = 0.8
    task._vertical_gain = 0.8
    task._yaw_gain = 0.8
    task._max_lateral_step = 0.10
    task._max_vertical_step = 0.08
    task._max_yaw_step = 10.0

    off_center = _stereo_observation([730.0, 480.0], [710.0, 480.0], extent=0.25)
    centered = _stereo_observation([650.0, 480.0], [630.0, 480.0], extent=0.25)

    assert task._servo_delta(off_center)[0] == 0.0
    assert task._servo_delta(centered)[0] > 0.0
