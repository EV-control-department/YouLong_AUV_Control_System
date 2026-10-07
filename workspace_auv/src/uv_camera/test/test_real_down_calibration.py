"""Regression checks for real 1280x480 stitched front/down cameras."""

from pathlib import Path
import json
from types import SimpleNamespace

import numpy as np

from uv_camera.down_calibration import load_real_down_json
from uv_camera.object_localizer import StereoCalibration
from uv_camera.ai import Ai
from uv_camera.common import rotate_stereo_180, unrotate_points_180


def test_mapping_http_path_strips_only_exact_jpg_suffix():
    from uv_camera.common import _MjpegHandler
    for path, expected in (('/mapping/depth.jpg?refresh=1', 'depth'),
                           ('/mapping/sgbm', 'sgbm'),
                           ('/mapping/depth.jpeg', 'depth.jpeg')):
        names, errors = [], []
        vision = SimpleNamespace(debug_jpeg=lambda name: names.append(name))
        handler = SimpleNamespace(path=path, node=SimpleNamespace(mapping_vision=vision),
                                  send_error=lambda *args: errors.append(args))
        _MjpegHandler.do_GET(handler)
        assert names == [expected]
        assert errors[0][0] == 503


def test_mjpeg_shutdown_is_bounded_even_if_shutdown_waits():
    import threading
    from uv_camera.common import _MjpegServer
    release = threading.Event()
    closed = []
    server = SimpleNamespace(shutdown=lambda: None,
                             server_close=lambda: closed.append(True))
    assert _MjpegServer.stop_bounded(server, timeout=1.0)
    assert closed == [True]
    server.shutdown = release.wait
    try:
        assert not _MjpegServer.stop_bounded(server, timeout=0.02)
        assert closed == [True, True]
    finally:
        release.set()


def test_down_rotation_preserves_stereo_identity_and_pixel_geometry():
    left = np.arange(12, dtype=np.uint8).reshape(3, 4)
    right = left + 100
    raw = np.hstack((left, right))
    upright = rotate_stereo_180(raw)
    np.testing.assert_array_equal(upright[:, :4], left[::-1, ::-1])
    np.testing.assert_array_equal(upright[:, 4:], right[::-1, ::-1])
    np.testing.assert_array_equal(rotate_stereo_180(upright), raw)
    points = np.array([[0, 0], [3, 2], [1.25, 0.75]], np.float32)
    rotated = unrotate_points_180(points, 4, 3)
    np.testing.assert_allclose(unrotate_points_180(rotated, 4, 3), points)


def test_real_down_inverted_installation_extrinsics():
    import yaml
    root = Path(__file__).resolve().parents[1] / 'config/profiles'
    original = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
    expected = original @ np.diag([-1., -1., 1.])
    for profile in ('real_default.yaml', 'real_safe.yaml'):
        config = yaml.safe_load((root / profile).read_text())
        camera = config['/uv_camera']['ros__parameters']
        localizer = config['/object_localizer']['ros__parameters']
        rotation = np.array(camera['mapping_camera_rotation']).reshape(3, 3)
        np.testing.assert_allclose(rotation, expected)
        np.testing.assert_allclose(rotation @ rotation.T, np.eye(3))
        assert np.isclose(np.linalg.det(rotation), 1.)
        for side in ('left', 'right'):
            np.testing.assert_allclose(localizer[f'down_{side}_rotation'], expected.ravel())
            np.testing.assert_allclose(localizer[f'down_{side}_translation'],
                                       camera[f'mapping_{side}_translation'])
        # Camera1 remains the left sensor in calibration, now physically on +y.
        assert camera['mapping_left_translation'][1] > 0
        assert camera['mapping_right_translation'][1] < 0


def test_rotated_yolo_outputs_return_to_calibration_pixels():
    import threading
    from std_msgs.msg import Header
    from uv_camera.ai import GATE_FRONT_CLASS_ID

    class Boxes:
        cls = [GATE_FRONT_CLASS_ID + 100]
        conf = [0.9]
        xyxy = np.array([[1., 1., 3., 2.]])

        def __len__(self):
            return 1

    polygon = np.array([[1., 1.], [2., 1.], [2., 2.], [1., 2.]])
    image = np.arange(4 * 6 * 3, dtype=np.uint8).reshape(4, 6, 3)
    inputs = []

    def model(frame, **kwargs):
        inputs.append(frame.copy())
        return [SimpleNamespace(boxes=Boxes(), masks=SimpleNamespace(xy=[polygon]))]

    view = SimpleNamespace(
        node=SimpleNamespace(camera_rotate_180=lambda camera: camera == 'down'),
        _inference_lock=threading.Lock(), _model=model, _confidence=0.35,
        _device='cpu', _last_inference_diagnostic={'down_left': float('inf')},
        _set_gate_feature=lambda *args: None)
    detections, polygons, _, _ = Ai._detect(view, Header(), 'down_left', image, 17)
    np.testing.assert_array_equal(inputs[0], image[::-1, ::-1])
    det = detections.detections[0]
    np.testing.assert_allclose(
        [det.bbox_x1, det.bbox_y1, det.bbox_x2, det.bbox_y2], [3, 2, 5, 3])
    np.testing.assert_allclose(polygons[0], [5, 3] - polygon)
    np.testing.assert_allclose(det.mask_x, polygons[0][:, 0])
    assert detections.stereo_pair_id == 17


def test_real_model_discovery_prefers_compatible_weights(tmp_path, monkeypatch):
    import sys
    import uv_camera.ai as ai_module

    package = tmp_path / 'uv_camera'
    (package / 'uv_camera').mkdir(parents=True)
    (package / 'resource').mkdir()
    original = package / 'resource/last.pt'
    compatible = package / 'resource/last_inference.pt'
    original.touch()
    compatible.touch()
    monkeypatch.setattr(ai_module, '__file__', str(package / 'uv_camera/ai.py'))
    monkeypatch.setitem(sys.modules, 'torch', None)
    loaded = []

    def fake_yolo(path):
        loaded.append(path)
        return SimpleNamespace(
            names=dict(enumerate(ai_module.DEFAULT_CLASS_NAMES)),
            to=lambda device: None)

    monkeypatch.setitem(sys.modules, 'ultralytics', SimpleNamespace(YOLO=fake_yolo))
    logger = SimpleNamespace(info=lambda text: None, warn=lambda text: None,
                             error=lambda text: None)
    view = SimpleNamespace(_sim_mode=False, _inference_threads=1,
                           _model_loaded=False, node=SimpleNamespace(get_logger=lambda: logger))
    Ai.load_model(view)
    assert view._model_loaded
    assert loaded == [str(compatible)]
    # Explicit model paths remain authoritative, even when a sibling exists.
    Ai.load_model(view, str(original))
    assert loaded[-1] == str(original)


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
