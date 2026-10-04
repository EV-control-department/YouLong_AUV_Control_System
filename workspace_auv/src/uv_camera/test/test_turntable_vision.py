"""转盘视觉在 camera 内由盘体掩膜和原图提取，不依赖 DDS 图像。"""

from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from uv_camera.turntable_vision import _sgbm_disk, estimate


def detection(class_id, points):
    return SimpleNamespace(class_id=class_id, confidence=0.9,
                           mask_x=[x for x, _ in points],
                           mask_y=[y for _, y in points])


def test_off_center_label_yields_angle():
    disk = detection(10, [(40, 50), (50, 40), (60, 40), (70, 50),
                          (70, 60), (60, 70), (50, 70), (40, 60)])
    label = detection(11, [(61, 52), (67, 52), (67, 58), (61, 58)])
    result = estimate(SimpleNamespace(detections=[disk, label]), 10, 11)
    assert result['valid']
    assert abs(result['angle_deg']) < 5


def test_centered_label_is_rejected_instead_of_guessing_angle():
    disk = detection(10, [(40, 50), (50, 40), (60, 40), (70, 50),
                          (70, 60), (60, 70), (50, 70), (40, 60)])
    label = detection(11, [(52, 52), (58, 52), (58, 58), (52, 58)])
    result = estimate(SimpleNamespace(detections=[disk, label]), 10, 11)
    assert result['valid']
    assert not result['phase_valid']
    assert result['phase_reason'] == 'label_not_on_radial_track'


def test_hsv_marker_works_without_yolo_label():
    points = [(40, 50), (50, 40), (60, 40), (70, 50),
              (70, 60), (60, 70), (50, 70), (40, 60)]
    frame = np.zeros((120, 120, 3), np.uint8)
    cv2.circle(frame, (65, 55), 4, (0, 255, 255), -1)
    result = estimate(SimpleNamespace(detections=[detection(3, points)]),
                      3, frame=frame)
    assert result['valid'] and result['phase_valid']
    assert result['phase_source'] == 'hsv'


def test_no_yellow_never_creates_phase():
    points = [(40, 50), (50, 40), (60, 40), (70, 50),
              (70, 60), (60, 70), (50, 70), (40, 60)]
    result = estimate(SimpleNamespace(detections=[detection(3, points)]),
                      3, frame=np.zeros((120, 120, 3), np.uint8))
    assert result['valid'] and not result['phase_valid']


def test_black_mask_stereo_plane_center():
    class Calibration:
        camera_matrix_left = camera_matrix_right = np.array(
            [[200., 0., 160.], [0., 200., 120.], [0., 0., 1.]])
        dist_left = dist_right = np.zeros(5)
        rectification_left = rectification_right = np.eye(3)
        projection_left = np.array([[200., 0., 160., 0.],
                                    [0., 200., 120., 0.], [0., 0., 1., 0.]])
        projection_right = np.array([[200., 0., 160., -20.],
                                     [0., 200., 120., 0.], [0., 0., 1., 0.]])
        baseline_m = 0.1

        def rectified_pixel(self, _side, pixel):
            return pixel

    class Matcher:
        def compute(self, _left, _right):
            return np.full((240, 320), 320, np.int16)

    frame = np.zeros((240, 320, 3), np.uint8)
    polygon = cv2.ellipse2Poly((160, 120), (55, 55), 0, 0, 360, 10)
    center, normal, count, residual = _sgbm_disk(
        frame, frame, polygon, Calibration(), Matcher(), 95, 30)
    assert center == pytest.approx([0, 0, 1], abs=0.01)
    assert normal == pytest.approx([0, 0, 1], abs=0.01)
    assert count >= 30 and residual < 0.001
