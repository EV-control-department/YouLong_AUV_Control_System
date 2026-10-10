"""Production detector mapping/publishing integration, without real inference."""
import threading
from types import SimpleNamespace as NS

import cv2
import numpy as np
import pytest
from rclpy.serialization import deserialize_message, serialize_message
from uv_msgs.msg import Detection, DetectionArray, LineState
from sensor_msgs.msg import CameraInfo
from uv_camera.image_geometry import EyeUndistorter

import uv_perception.object_detector as module
from uv_perception.object_detector import ObjectDetector
from test_ring_orientation import angular_error, line_image


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class Node:
    def __init__(self, params=None):
        self.params = params or {}
        self.declared = {}
        self.log = []

    def create_publisher(self, *args):
        return Publisher()

    def create_subscription(self, *args):
        return None

    def create_timer(self, *args):
        return None

    def declare_parameter(self, key, default):
        self.declared[key] = default
        return NS(value=self.params.get(key, default))

    def get_logger(self):
        return NS(info=self.log.append, warning=self.log.append, error=self.log.append)


@pytest.fixture
def detector():
    instance = ObjectDetector.__new__(ObjectDetector)
    instance.node = Node()
    instance.publisher = Publisher()
    instance._line_publishers = {f'{camera}_{eye}': Publisher()
                                 for camera in ('front', 'down')
                                 for eye in ('left', 'right')}
    instance._line_state = lambda *_: LineState()
    instance._gate_feature_mode = 'bbox'
    instance._guide_line_class_id = None
    instance._gate_front_class_id = 41
    # Deliberately differs from the default model's class 8.
    instance._red_ring_class_id = 37
    instance._ring_orientation_enabled = True
    instance._ring_roi_scale = 1.2
    instance._ring_min_pixels = 24
    instance._ring_min_quality = .8
    instance._mapping_ready = threading.Event()
    instance._stop = threading.Event()
    instance._frame_health_lock = threading.Lock()
    instance._last_frame_at = {'front': 0., 'down': 0.}
    instance.confidence = .5
    instance._detectors = {'front': None, 'down': None}
    instance._model_valid = {'front': True, 'down': True}
    instance._model_errors = {}
    instance._last_warning = {}
    instance._mapping_registry = NS(class_info=lambda _: NS(camera=None))
    def correct(camera, packet, image):
        height, width = image.shape[:2];half=width//2
        calibration = EyeUndistorter(CameraInfo(width=half, height=height,
            k=[100.,0.,half/2,0.,100.,height/2,0.,0.,1.],d=[0.]*5))
        return [(side, image[:, offset:offset+half], calibration)
                for side, offset in (('left',0),('right',half))]
    instance._correct_eyes = correct
    return instance


def packet(image=None):
    return NS(header=NS(timestamp_ns=12_345_000_123, capture_id=101,
                        stereo_pair_id=99), bgr=lambda: image)


def publish(instance, image, bbox, *, camera='down', eye='left', class_id=37,
            polygon=None):
    instance._publish(camera, eye, packet(), image,
                      [(class_id, .9, bbox)], [polygon])
    return instance.publisher.messages[-1]


@pytest.mark.parametrize('camera,eye,angle,valid', [
    ('down', 'left', 30, True), ('down', 'right', 120, True),
    ('front', 'left', 30, False), ('front', 'right', 120, False),
])
def test_camera_gating_and_same_frame_metadata(detector, camera, eye, angle, valid):
    image, bbox = line_image(angle)
    message = publish(detector, image, bbox, camera=camera, eye=eye)
    detection = message.detections[0]
    assert detection.orientation_valid == valid
    if valid:
        assert angular_error(detection.orientation_axis_deg, angle) <= 3
        assert .8 <= detection.orientation_quality <= 1
    else:
        assert detection.orientation_axis_deg == detection.orientation_quality == 0
    assert message.camera_name == f'{camera}_{eye}'
    assert message.header.frame_id == module._camera_optical_frame(camera, eye)
    assert message.header.stamp.sec == 12
    assert message.header.stamp.nanosec == 345_000_123
    assert (message.capture_id, message.stereo_pair_id) == (101, 99)
    assert (detection.bbox_x1, detection.bbox_y1,
            detection.bbox_x2, detection.bbox_y2) == bbox
    assert detection.pixel_x == (bbox[0]+bbox[2])/2
    assert detection.pixel_y == (bbox[1]+bbox[3])/2


@pytest.mark.parametrize('reason', ['other_class', 'disabled', 'mapping_missing', 'no_red'])
def test_invalid_fields_are_zero_and_bbox_still_published(detector, reason, monkeypatch):
    image, bbox = line_image(30)
    class_id = 37
    if reason == 'other_class':
        class_id = 36
    elif reason == 'disabled':
        detector._ring_orientation_enabled = False
    elif reason == 'mapping_missing':
        detector._red_ring_class_id = None
    else:
        image[:] = 0
    if reason != 'no_red':
        monkeypatch.setattr(module, 'estimate_ring_orientation',
                            lambda *a, **k: pytest.fail('CV must not be called'))
    message = publish(detector, image, bbox, class_id=class_id)
    assert len(message.detections) == 1
    detection = message.detections[0]
    assert not detection.orientation_valid
    assert detection.orientation_axis_deg == detection.orientation_quality == 0
    assert detection.confidence == .9 and detection.class_id == class_id


def test_cv_failure_does_not_drop_detection(detector, monkeypatch):
    image, bbox = line_image(30)
    def fail(*_):
        raise cv2.error('synthetic error')
    monkeypatch.setattr(cv2, 'cvtColor', fail)
    message = publish(detector, image, bbox)
    assert len(message.detections) == 1
    assert not message.detections[0].orientation_valid


def test_reused_detection_clears_old_orientation(detector):
    image, bbox = line_image(30)
    detection = Detection()
    detection.class_id = 37
    detection.orientation_valid = True
    detection.orientation_axis_deg = 130.
    detection.orientation_quality = .9
    detector._set_ring_orientation(detection, 'front', image)
    assert not detection.orientation_valid
    assert detection.orientation_axis_deg == detection.orientation_quality == 0


def test_mapping_lookup_is_semantic_not_hardcoded(detector, monkeypatch):
    calls = []
    def model_class_id(name, required=False):
        calls.append((name, required))
        return {'guide_line': 31, 'gate_front': 41, 'red_ring': 37}[name]
    registry = NS(model='custom.pt', model_class_id=model_class_id)
    monkeypatch.setattr(module.ModelClassRegistry, 'from_message', lambda _: registry)
    detector._mapping_callback(NS())
    assert detector._red_ring_class_id == 37
    assert ('red_ring', False) in calls
    assert detector._mapping_ready.is_set()


def test_front_gate_segmentation_feature_remains_usable(detector):
    image = np.zeros((240, 320, 3), np.uint8)
    detector._gate_feature_mode = 'auto'
    polygon = np.array([[100, 70], [220, 70], [220, 170], [100, 170]], np.float32)
    message = publish(detector, image, (100., 70., 220., 170.),
                      camera='front', class_id=41, polygon=polygon)
    detection = message.detections[0]
    assert detection.feature_type == Detection.FEATURE_GATE_SEGMENTATION
    assert detection.feature_pixel_x == pytest.approx(160.)
    assert detection.feature_pixel_y == pytest.approx(120.)
    assert not detection.orientation_valid


def test_one_inference_per_eye_and_independent_directions(detector, monkeypatch):
    left, _ = line_image(30)
    right, _ = line_image(120)
    combined = np.hstack([left, right])
    packets = iter([packet(combined), None])
    closed = []
    reader = NS(read=lambda: next(packets), close=lambda: closed.append(True))
    monkeypatch.setattr(module, 'Iceoryx2Reader', lambda _: reader)
    inference_calls = []
    def infer(eye):
        inference_calls.append(eye.shape)
        ys, xs = np.where(eye[:, :, 2] > 0)
        bbox = (float(xs.min()-2), float(ys.min()-2),
                float(xs.max()+2), float(ys.max()+2))
        return [(37, .9, bbox)], [None]
    detector._detectors['down'] = NS(detect_with_masks=infer)
    detector._mapping_ready.set()
    detector._read_loop('down', 'synthetic_camera')
    assert inference_calls == [(240, 320, 3), (240, 320, 3)]
    assert closed == [True]
    messages = detector.publisher.messages
    assert [m.camera_name for m in messages] == ['down_left', 'down_right']
    assert angular_error(messages[0].detections[0].orientation_axis_deg, 30) <= 3
    assert angular_error(messages[1].detections[0].orientation_axis_deg, 120) <= 3
    assert messages[0].capture_id == messages[1].capture_id == 101


def test_jpeg_is_decoded_once_and_bad_frame_does_not_stop_detector(detector, monkeypatch):
    from uv_image_transport import ENCODING_JPEG, FrameHeader, FramePacket, InvalidFrameError
    image = np.zeros((32, 64, 3), np.uint8)
    image[:, :32] = (200, 0, 0)
    image[:, 32:] = (0, 0, 200)
    payload = cv2.imencode('.jpg', image)[1].tobytes()
    valid = FramePacket(FrameHeader(101, 1_000_000_000, 99, 2, 64, 32, 0,
                                    encoding=ENCODING_JPEG), payload)
    values = iter([InvalidFrameError('bad frame'), valid, None])

    def read():
        value = next(values)
        if isinstance(value, Exception):
            raise value
        return value

    monkeypatch.setattr(module, 'Iceoryx2Reader', lambda _: NS(read=read, close=lambda: None))
    original_decode = cv2.imdecode
    decodes = []

    def decode(*args):
        decodes.append(True)
        return original_decode(*args)

    monkeypatch.setattr(cv2, 'imdecode', decode)
    published = []
    detector._publish = lambda camera, side, packet, eye, *_: published.append(
        (side, packet.header.capture_id, eye.copy()))
    detector._detectors['down'] = NS(detect_with_masks=lambda eye: ((),()))
    detector._mapping_ready.set()
    detector._read_loop('down', 'jpeg_test_camera')
    assert len(decodes) == 1
    assert [item[:2] for item in published] == [('left', 101), ('right', 101)]
    assert published[0][2][16, 16, 0] > 190
    assert published[1][2][16, 16, 2] > 190


def test_generated_message_serialization_preserves_orientation(detector):
    image, bbox = line_image(45)
    original = publish(detector, image, bbox)
    decoded = deserialize_message(serialize_message(original), DetectionArray)
    assert decoded.header == original.header
    assert decoded.capture_id == original.capture_id
    detection = decoded.detections[0]
    assert detection.orientation_valid
    assert detection.orientation_axis_deg == pytest.approx(45.)
    assert detection.orientation_quality == pytest.approx(original.detections[0].orientation_quality)


@pytest.mark.parametrize('params', [None, {'ring_orientation_enabled': False},
                                    {'ring_roi_scale': 1.5, 'ring_min_pixels': 50,
                                     'ring_min_quality': .9}])
def test_four_parameters_are_declared_and_applied(params, monkeypatch):
    monkeypatch.setattr(module, '_model_default', lambda: '')
    monkeypatch.setattr(ObjectDetector, '_make_aruco_detector', lambda _: None)
    monkeypatch.setattr(module.threading, 'Thread', lambda **_: NS(start=lambda: None))
    node = Node(params)
    instance = ObjectDetector(node)
    assert node.declared['ring_orientation_enabled'] is True
    assert node.declared['ring_roi_scale'] == 1.2
    assert node.declared['ring_min_pixels'] == 24
    assert node.declared['ring_min_quality'] == .8
    for key in ('ring_orientation_enabled', 'ring_roi_scale', 'ring_min_pixels', 'ring_min_quality'):
        assert getattr(instance, '_'+key) == (params or {}).get(key, node.declared[key])


@pytest.mark.parametrize('params', [{'ring_roi_scale': .9}, {'ring_roi_scale': float('nan')},
                                    {'ring_min_pixels': 1}, {'ring_min_quality': 1.1}])
def test_invalid_configuration_fails_before_starting_readers(params, monkeypatch):
    monkeypatch.setattr(module.threading, 'Thread',
                        lambda **_: pytest.fail('Must fail before starting readers'))
    with pytest.raises(ValueError):
        ObjectDetector(Node(params))
