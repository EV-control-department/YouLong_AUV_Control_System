"""Regression checks for real 1280x480 stitched front/down cameras."""

from pathlib import Path
import json
from types import SimpleNamespace

import numpy as np

from uv_camera.down_calibration import load_real_down_json
from uv_camera.object_localizer import StereoCalibration
from uv_camera.ai import Ai


def test_json_scaled_to_640x480_per_eye():
    path = Path(__file__).resolve().parents[1] / 'config/down_real.json'
    width, height, k1, d1, k2, d2, _, translation = load_real_down_json(path)
    assert (width, height) == (640, 480)
    assert np.isclose(k1[0, 0], 1159.0997987420169 / 2)
    assert np.isclose(k1[0, 1], 0.6582010360123447 / 2)
    assert np.isclose(k1[0, 2], 679.11958604320034 / 2)
    assert np.isclose(k2[1, 2], 549.46998215035433 / 2)
    assert np.isclose(d1[0], -0.36428009567333464)
    assert np.isclose(d2[0], -0.36148274221566018)
    assert np.isclose(translation[0], -0.06106714057703352)

    calibration = StereoCalibration.load('down', str(path))
    assert 0.060 < calibration.baseline_m < 0.063
    assert np.isclose(calibration.projection_right[0, 0],
                      calibration.projection_left[0, 0])


def test_packaged_copy_matches_supplied_json_when_present():
    source = Path(__file__).resolve().parents[3] / 'docs/stereo_parameters.json'
    packaged = Path(__file__).resolve().parents[1] / 'config/down_real.json'
    if source.is_file():
        assert json.loads(source.read_text()) == json.loads(packaged.read_text())


def test_real_detection_keeps_raw_pixels_for_localizer():
    path = Path(__file__).resolve().parents[1] / 'config/down_real.json'
    _, _, k1, d1, k2, d2, _, _ = load_real_down_json(path)
    view = SimpleNamespace(_sim_mode=False, _mapping_callback=None,
                           _down_K=k1, _down_D=d1,
                           _down_right_K=k2, _down_right_D=d2)
    frame = np.arange(480 * 1280 * 3, dtype=np.uint8).reshape(480, 1280, 3)
    _, left, right = Ai._prepare_stereo_views(view, 'down', frame)
    assert np.array_equal(left, frame[:, :640])
    assert np.array_equal(right, frame[:, 640:])


def test_front_native_calibration_scales_to_640x480_per_eye():
    path = Path(__file__).resolve().parents[1] / 'config/front.npz'
    native = StereoCalibration.load('front', str(path))
    scaled = native.scaled(0.5, 0.5)
    assert np.allclose(scaled.camera_matrix_left[0, :],
                       native.camera_matrix_left[0, :] * 0.5)
    assert np.allclose(scaled.camera_matrix_right[1, :],
                       native.camera_matrix_right[1, :] * 0.5)
    assert np.allclose(scaled.projection_right[:2, :],
                       native.projection_right[:2, :] * 0.5)
    assert np.allclose(scaled.dist_left, native.dist_left)
    assert np.isclose(scaled.baseline_m, native.baseline_m)
    assert np.isclose(abs(scaled.projection_right[0, 3] /
                          scaled.projection_right[0, 0]), native.baseline_m, atol=1e-5)
    pixel = np.array([300.0, 220.0, 12.0, 1.0])
    assert np.allclose(scaled.reprojection @ pixel,
                       native.reprojection @ np.array([600.0, 440.0, 24.0, 1.0]))


def test_real_front_detection_keeps_raw_pixels():
    view = SimpleNamespace(_sim_mode=False, _mapping_callback=None,
                           _turntable_callback=None,
                           _front_K=np.eye(3), _front_D=np.ones(5))
    frame = np.arange(480 * 1280 * 3, dtype=np.uint8).reshape(480, 1280, 3)
    _, left, right = Ai._prepare_stereo_views(view, 'front', frame)
    assert np.array_equal(left, frame[:, :640])
    assert np.array_equal(right, frame[:, 640:])
