"""Monocular configuration, source bytes, capture time, and reconnect behavior."""

from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import pytest
from rclpy.parameter import Parameter, parameter_dict_from_yaml_file

from sl_camera.capture import CaptureConfig, MjpegCapture, jpeg_payload


def source_frame():
    image = np.zeros((32, 64, 3), dtype=np.uint8)
    image[:16, :32] = (20, 50, 200)
    image[16:, 32:] = (210, 180, 40)
    payload = cv2.imencode('.jpg', image)[1].tobytes()
    return payload, np.frombuffer(payload, dtype=np.uint8).reshape(1, -1)


def test_default_yaml_loads_as_ros_parameters():
    path = Path(__file__).parents[1] / 'config/default.yaml'
    params = parameter_dict_from_yaml_file(str(path))
    values = {name: Parameter.from_parameter_msg(parameter).value
              for name, parameter in params.items()}
    config = CaptureConfig(**values)
    assert (config.device, config.width, config.height, config.fps) == ('', 1280, 720, 30.0)
    with pytest.raises(ValueError, match='device is empty'):
        config.validate()


@pytest.mark.parametrize('changes', [
    {'width': 0}, {'height': -1}, {'width': True}, {'width': 1280.5},
    {'fps': 0}, {'fps': float('nan')}, {'fps': float('inf')},
    {'startup_timeout_sec': -1}, {'reconnect_interval_sec': 0},
])
def test_bad_config_rejected(changes):
    with pytest.raises(ValueError):
        replace(CaptureConfig(device='/dev/test'), **changes).validate()


def test_complete_monocular_bytes_and_direction_unchanged(monkeypatch):
    payload, frame = source_frame()
    monkeypatch.setattr(cv2, 'imdecode', lambda *_: pytest.fail('decoded source JPEG'))
    monkeypatch.setattr(cv2, 'imencode', lambda *_: pytest.fail('re-encoded source JPEG'))
    monkeypatch.setattr(cv2, 'rotate', lambda *_: pytest.fail('rotated source JPEG'))
    assert jpeg_payload(frame, 64, 32) == payload
    for invalid in (frame[:, :-2], np.zeros((32, 64, 3), dtype=np.uint8)):
        with pytest.raises(ValueError):
            jpeg_payload(invalid, 64, 32)
    with pytest.raises(ValueError, match='configured'):
        jpeg_payload(frame, 32, 32)


def test_v4l2_requests_compressed_mjpg_size_and_fps(monkeypatch):
    values = {}
    released = []
    opened = []
    cap = SimpleNamespace(
        isOpened=lambda: True,
        set=lambda key, value: values.update({key: value}) or True,
        get=lambda key: values[key], release=lambda: released.append(True))
    monkeypatch.setattr(cv2, 'VideoCapture', lambda *args: opened.append(args) or cap)
    capture = MjpegCapture(CaptureConfig(device='/dev/test'), lambda: 1,
                           lambda *_: None, lambda _: None)
    assert capture._open_capture() is cap
    assert opened == [('/dev/test', cv2.CAP_V4L2)]
    assert values[cv2.CAP_PROP_CONVERT_RGB] == 0
    assert values[cv2.CAP_PROP_FRAME_WIDTH] == 1280
    assert values[cv2.CAP_PROP_FRAME_HEIGHT] == 720
    assert values[cv2.CAP_PROP_FPS] == 30
    cap.set = lambda key, _: key != cv2.CAP_PROP_CONVERT_RGB
    with pytest.raises(RuntimeError, match='disable MJPEG decoding'):
        capture._open_capture()
    assert released == [True]


@pytest.mark.parametrize('failure', ['open', 'read', 'invalid_jpeg'])
def test_reconnect_then_stamp_before_processing_and_release(monkeypatch, failure):
    import sl_camera.capture as module

    payload, frame = source_frame()
    statuses, events, opened, released, received = [], [], [], [], []
    ticks = iter(range(1000))
    monkeypatch.setattr(module.time, 'monotonic', lambda: float(next(ticks)))
    original_payload = module.jpeg_payload

    def inspect(*args):
        events.append('inspect')
        return original_payload(*args)

    monkeypatch.setattr(module, 'jpeg_payload', inspect)

    def clock():
        events.append('clock')
        return 1234567890

    def publish(data, timestamp):
        events.append('publish')
        received.append((data, timestamp))
        capture.stop_event.set()

    capture = MjpegCapture(
        CaptureConfig(device='/dev/test', width=64, height=32,
                      startup_timeout_sec=0.1, reconnect_interval_sec=0.001),
        clock, publish, statuses.append)

    def open_capture():
        attempt = len(opened)
        opened.append(attempt)
        if attempt == 0 and failure == 'open':
            raise RuntimeError('unplugged')

        def read():
            events.append('read')
            if attempt == 0:
                return ((False, None) if failure == 'read'
                        else (True, frame[:, :-2]))
            return True, frame

        return SimpleNamespace(read=read, release=lambda: released.append(attempt))

    monkeypatch.setattr(capture, '_open_capture', open_capture)
    capture.start()
    capture.thread.join(2)
    assert capture.stop()
    assert opened == [0, 1]
    assert released == ([1] if failure == 'open' else [0, 1])
    assert received == [(payload, 1234567890)]
    assert events[-4:] == ['read', 'clock', 'inspect', 'publish']
    assert any(status.startswith('error:') for status in statuses)
    assert statuses[-1] == 'running'


def test_node_uses_yaml_and_publishes_sl_headers(tmp_path, monkeypatch):
    import rclpy
    import sl_camera.driver as module

    config = tmp_path / 'camera.yaml'
    config.write_text('/sl_camera:\n  ros__parameters:\n'
                      '    device: /dev/test\n    width: 64\n    height: 32\n'
                      '    fps: 30.0\n')
    packets, services, closed = [], [], []
    monkeypatch.setattr(module, 'Iceoryx2Publisher', lambda service:
                        services.append(service) or SimpleNamespace(
                            publish=lambda header, data: packets.append((header, data)),
                            close=lambda: closed.append(True)))
    monkeypatch.setattr(module.MjpegCapture, 'start', lambda _: None)
    rclpy.init(args=['--ros-args', '--params-file', str(config)])
    node = None
    try:
        node = module.SlCamera()
        payload, _ = source_frame()
        node._on_frame(payload, 5_123_456_789)
        node._on_frame(payload, 5_223_456_789)
        assert services == ['youlong/camera/sl']
        assert [header.capture_id for header, _ in packets] == [1, 2]
        assert [header.timestamp_ns for header, _ in packets] == [5_123_456_789, 5_223_456_789]
        assert all((header.camera_group, header.camera_name, header.stereo_pair_id,
                    header.width, header.height) == (3, 'sl', 0, 64, 32)
                   for header, _ in packets)
        assert all(data == payload for _, data in packets)
        # SIGINT may shut down the context before launch asks the node to close.
        rclpy.shutdown()
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    assert closed == [True]
