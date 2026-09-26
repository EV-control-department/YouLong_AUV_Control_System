from uv_msgs.msg import Detection, DetectionArray
from sensor_msgs.msg import CameraInfo

from uv_perception.object_localizer import ObjectLocalizer


def _localizer():
    localizer = ObjectLocalizer.__new__(ObjectLocalizer)
    localizer.edge_margin_px = 8.0
    localizer.edge_margin_ratio = 0.02
    localizer.stereo_epipolar_tolerance_px = 10.0
    localizer.baseline_m = 0.10
    localizer.max_stereo_range_m = 30.0
    return localizer


def _info(width=1280, height=960):
    info = CameraInfo()
    info.width = width
    info.height = height
    info.k = [500.0, 0.0, width / 2.0,
              0.0, 500.0, height / 2.0,
              0.0, 0.0, 1.0]
    return info


def _array(camera, box, pixel_x):
    detection = Detection()
    detection.class_id = 1
    detection.confidence = 0.9
    detection.bbox_x1, detection.bbox_y1 = box[0], box[1]
    detection.bbox_x2, detection.bbox_y2 = box[2], box[3]
    detection.pixel_x, detection.pixel_y = pixel_x, 400.0
    message = DetectionArray()
    message.camera_name = camera
    message.detections = [detection]
    return message


def test_edge_eye_is_rejected_before_stereo_and_valid_eye_remains_eligible():
    localizer = _localizer()
    left = _array('front_left', (8.0, 300.0, 120.0, 500.0), 600.0)
    right = _array('front_right', (40.0, 300.0, 150.0, 500.0), 580.0)
    info = _info()

    left_eligible = localizer._eligible(left, info)
    right_eligible = localizer._eligible(right, info)

    assert left_eligible == []
    assert right_eligible == [0]
    assert localizer._stereo_pairs(
        left, right, info, info, left_eligible, right_eligible) == []


def test_valid_rectified_stereo_pair_is_assigned_once():
    localizer = _localizer()
    left = _array('front_left', (100.0, 300.0, 220.0, 500.0), 600.0)
    right = _array('front_right', (80.0, 300.0, 200.0, 500.0), 580.0)
    info = _info()

    assert localizer._stereo_pairs(
        left, right, info, info, [0], [0]) == [(0, 0)]
