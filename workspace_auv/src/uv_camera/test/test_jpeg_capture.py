"""Compressed capture, direction, and simulator publication contracts."""

from types import SimpleNamespace as NS
import threading

import cv2
import numpy as np
import pytest

from uv_camera.jpeg_transform import JpegRotator
from uv_camera.sensor import Sensor, capture_jpeg
from uv_image_transport import ENCODING_JPEG, FrameHeader, FramePacket
from uv_image_transport.jpeg import jpeg_dimensions


def pattern():
    image = np.zeros((32, 64, 3), np.uint8)
    image[:16, :32] = (20, 50, 200)
    image[16:, :32] = (10, 180, 40)
    image[:16, 32:] = (210, 30, 20)
    image[16:, 32:] = (40, 210, 200)
    return image


def encode(image):
    return cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 95])[1].tobytes()


def decode(payload):
    width, height = jpeg_dimensions(payload)
    return FramePacket(FrameHeader(1, 2, 3, 1, width, height, 0,
                                   encoding=ENCODING_JPEG), payload).bgr()


def test_lossless_rotation_direction_eyes_and_round_trip():
    payload = encode(pattern())
    with JpegRotator() as rotator:
        rotated = rotator.rotate_180(payload)
        restored = rotator.rotate_180(rotated)
    assert jpeg_dimensions(rotated) == (64, 32)
    np.testing.assert_allclose(
        decode(rotated).astype(int), cv2.rotate(decode(payload), cv2.ROTATE_180).astype(int),
        atol=2)
    np.testing.assert_array_equal(decode(restored), decode(payload))
    with pytest.raises(RuntimeError, match='closed'):
        rotator.rotate_180(payload)


def test_lossless_rotation_rejects_partial_mcu_and_corruption():
    with JpegRotator() as rotator:
        with pytest.raises(ValueError, match='rotation failed'):
            rotator.rotate_180(encode(np.zeros((31, 63, 3), np.uint8)))
        with pytest.raises(ValueError, match='rotation failed'):
            rotator.rotate_180(b'not a JPEG')


def test_uvc_mjpeg_without_huffman_tables_becomes_standalone_jpeg():
    payload = encode(pattern())
    abbreviated = payload[:2]
    position = 2
    while position < len(payload):
        marker = payload[position + 1]
        if marker == 0xDA:
            abbreviated += payload[position:]
            break
        size = int.from_bytes(payload[position + 2:position + 4], 'big')
        if marker != 0xC4:  # UVC devices may omit the standard Huffman tables.
            abbreviated += payload[position:position + size + 2]
        position += size + 2
    with JpegRotator() as rotator:
        corrected = rotator.rotate_180(abbreviated)
    assert b'\xff\xc4' in corrected
    np.testing.assert_allclose(
        decode(corrected).astype(int), cv2.rotate(decode(payload), cv2.ROTATE_180).astype(int),
        atol=2)


def test_capture_copies_raw_jpeg_without_decoding(monkeypatch):
    payload = encode(pattern())
    monkeypatch.setattr(cv2, 'imdecode', lambda *_: pytest.fail('capture decoded JPEG'))
    assert capture_jpeg(np.frombuffer(payload, np.uint8).reshape(1, -1), (64, 32)) == payload
    for invalid in (pattern(), np.zeros((32, 64), np.uint8)):
        with pytest.raises(ValueError, match='byte vector'):
            capture_jpeg(invalid, (64, 32))
    with pytest.raises(ValueError, match='resolution'):
        capture_jpeg(np.frombuffer(payload, np.uint8), (32, 32))


def test_v4l2_setup_requires_compressed_mjpg(monkeypatch):
    values = {}
    opened = []
    released = []
    cap = NS(set=lambda key, value: values.update({key: value}) or True,
             get=lambda key: values[key], release=lambda: released.append(True))
    monkeypatch.setattr(cv2, 'VideoCapture', lambda *args: opened.append(args) or cap)
    assert Sensor._open_cap('/dev/test', (64, 32)) is cap
    assert opened == [('/dev/test', cv2.CAP_V4L2)]
    assert values[cv2.CAP_PROP_CONVERT_RGB] == 0
    cap.set = lambda key, value: key != cv2.CAP_PROP_CONVERT_RGB
    with pytest.raises(RuntimeError, match='disable MJPEG decoding'):
        Sensor._open_cap('/dev/test', (64, 32))
    assert released


def test_real_worker_rotates_once_and_keeps_jpeg_compressed(monkeypatch):
    payload = encode(pattern())
    sensor = Sensor.__new__(Sensor)
    sensor._capture_stop = threading.Event()
    sensor._sim_mode = False
    sensor._startup_timeout_s = 1
    sensor._reconnect_interval_s = 0.01
    sensor._camera_configs = {'front': NS(capture_resolution=(64, 32))}
    sensor._front_cap = sensor._down_cap = None
    sensor.node = NS(get_logger=lambda: NS(info=lambda _: None, error=lambda _: None),
                     get_clock=lambda: NS(now=lambda: NS(to_msg=lambda: NS(sec=1, nanosec=2))))
    cap = NS(isOpened=lambda: True, release=lambda: None,
             read=lambda: (True, np.frombuffer(payload, np.uint8).reshape(1, -1)))
    sensor._open_cap = lambda *_: cap
    sensor._publish_real_sensor_frame = lambda *_: None
    received = []

    def submit(*args, **kwargs):
        received.append(args[1])
        sensor._capture_stop.set()

    sensor._frame_callback = submit
    # A real worker must never enter either pixel decode or pixel rotation.
    monkeypatch.setattr(cv2, 'imdecode', lambda *_: pytest.fail('real capture decoded JPEG'))
    monkeypatch.setattr(cv2, 'rotate', lambda *_: pytest.fail('real capture rotated BGR'))
    sensor._capture_worker('front', '/dev/test', (64, 32))
    assert len(received) == 1
    with JpegRotator() as rotator:
        assert received[0] == rotator.rotate_180(payload)


def test_simulator_encodes_once_without_rotation(monkeypatch):
    sensor = Sensor.__new__(Sensor)
    sensor._sim_mode = True
    received = []
    sensor._frame_callback = lambda *args, **kwargs: received.append((args, kwargs))
    original_encode = cv2.imencode
    qualities = []

    def counted_encode(extension, image, parameters):
        qualities.append(parameters)
        return original_encode(extension, image, parameters)

    monkeypatch.setattr(cv2, 'imencode', counted_encode)
    monkeypatch.setattr(cv2, 'rotate', lambda *_: pytest.fail('simulator was rotated'))
    sensor._submit_frame('front', pattern(), NS(sec=2, nanosec=3), stereo_pair_id=9)
    assert qualities == [[cv2.IMWRITE_JPEG_QUALITY, 95]]
    assert received[0][0][1] == encode(pattern())
    assert received[0][1]['stereo_pair_id'] == 9
