import ctypes
from types import SimpleNamespace

import numpy as np
import pytest

from uv_image_transport import (
    CAMERA_DOWN,
    CAMERA_FRONT,
    ENCODING_BGR8,
    ENCODING_JPEG,
    FrameHeader,
    FramePacket,
    Iceoryx2Error,
    Iceoryx2Publisher,
    Iceoryx2Reader,
    InvalidFrameError,
)
from uv_image_transport import iceoryx2 as transport


def test_shared_header_abi_and_binding_mapping():
    native = transport._IceoryxFrameHeader
    fields = [name for name, _ctype in native._fields_]
    assert ctypes.sizeof(native) == 48
    assert CAMERA_FRONT == 1
    assert CAMERA_DOWN == 2
    assert ENCODING_BGR8 == 1
    assert ENCODING_JPEG == 2
    assert fields == [
        'capture_id', 'timestamp_ns', 'stereo_pair_id', 'camera_group',
        'width', 'height', 'stride', 'encoding', 'camera_info_version',
    ]
    assert native.type_name() == 'YoulongCameraFrameHeader'
    assert native.capture_id.offset == 0
    assert native.timestamp_ns.offset == 8
    assert native.stereo_pair_id.offset == 16
    assert native.camera_group.offset == 24
    assert native.width.offset == 28
    assert native.height.offset == 32
    assert native.stride.offset == 36
    assert native.encoding.offset == 40
    assert native.camera_info_version.offset == 44

    raw = native()
    raw.capture_id = 12
    raw.timestamp_ns = 34
    raw.stereo_pair_id = 56
    raw.camera_group = CAMERA_FRONT
    raw.width = 640
    raw.height = 480
    raw.stride = 1920
    raw.encoding = ENCODING_BGR8
    raw.camera_info_version = 7
    header = transport._header_from_binding(raw)
    assert header == FrameHeader(12, 34, 56, CAMERA_FRONT, 640, 480, 1920, 1, 7)
    assert header.camera_name == 'front'
    assert FrameHeader(0, 0, 0, CAMERA_DOWN, 1, 1, 3).camera_name == 'down'


def test_bgr_decode_respects_row_stride():
    header = FrameHeader(1, 2, 3, CAMERA_DOWN, width=2, height=2, stride=8)
    payload = bytes([1, 2, 3, 4, 5, 6, 99, 99, 7, 8, 9, 10, 11, 12, 88, 88])

    image = FramePacket(header, payload).bgr()

    assert image.shape == (2, 2, 3)
    assert image.dtype == np.uint8
    np.testing.assert_array_equal(
        image,
        np.array([[[1, 2, 3], [4, 5, 6]], [[7, 8, 9], [10, 11, 12]]], dtype=np.uint8),
    )


def test_bgr_decode_rejects_unsupported_encoding_and_short_payload():
    header = FrameHeader(0, 0, 0, CAMERA_FRONT, 1, 1, 3, encoding=9)
    with pytest.raises(Iceoryx2Error, match='unsupported frame encoding'):
        FramePacket(header, b'123').bgr()

    header = FrameHeader(0, 0, 0, CAMERA_FRONT, 1, 1, 3)
    with pytest.raises(Iceoryx2Error, match='frame payload is short'):
        FramePacket(header, b'12').bgr()


def _native_header(*, width=1, height=1, stride=3):
    header = transport._IceoryxFrameHeader()
    header.capture_id = 1
    header.timestamp_ns = 2
    header.stereo_pair_id = 3
    header.camera_group = CAMERA_FRONT
    header.width = width
    header.height = height
    header.stride = stride
    header.encoding = ENCODING_BGR8
    header.camera_info_version = 0
    return header


class _Sample:
    def __init__(self, header, payload):
        self.header = header
        self.data = payload
        self.deleted = False

    def user_header(self):
        return SimpleNamespace(contents=self.header)

    def payload(self):
        return SimpleNamespace(as_memory_view=lambda: memoryview(self.data))

    def delete(self):
        self.deleted = True


class _Subscriber:
    def __init__(self, sample):
        self.sample = sample

    def receive(self):
        return self.sample


class _Node:
    def wait(self, _duration):
        raise AssertionError('wait should not run when a sample is available')


class _Service:
    def __init__(self, subscriber):
        self._subscriber = subscriber

    def subscriber_builder(self):
        return self

    def create(self):
        return self._subscriber


def _make_reader(monkeypatch, sample):
    class Duration:
        @staticmethod
        def from_secs_f64(value):
            return value

    monkeypatch.setattr(transport, '_load_binding', lambda: SimpleNamespace(Duration=Duration))
    monkeypatch.setattr(transport, '_new_node', lambda _iox2: _Node())
    subscriber = _Subscriber(sample)
    monkeypatch.setattr(transport, '_new_service', lambda *_args: _Service(subscriber))
    return Iceoryx2Reader('camera/test')


@pytest.mark.parametrize(
    ('header', 'payload', 'message'),
    [
        (_native_header(width=0), b'', 'invalid frame dimensions'),
        (_native_header(width=2, stride=6), b'12345', 'invalid payload length'),
    ],
)
def test_reader_rejects_invalid_dimensions_and_payload(monkeypatch, header, payload, message):
    sample = _Sample(header, payload)
    reader = _make_reader(monkeypatch, sample)
    with pytest.raises(Iceoryx2Error, match=message):
        reader.read()
    assert sample.deleted
    reader.close()


def test_reader_and_publisher_lifecycle(monkeypatch):
    class Duration:
        @staticmethod
        def from_secs_f64(value):
            return value

    class FakeIox:
        pass

    FakeIox.Duration = Duration
    FakeIox.AllocationStrategy = SimpleNamespace(PowerOfTwo=object())

    class Subscriber:
        def receive(self):
            raise AssertionError('closed reader must not receive')

    class Publisher:
        pass

    class Builder:
        def initial_max_slice_len(self, _size):
            return self

        def allocation_strategy(self, _strategy):
            return self

        def create(self):
            return Publisher()

    class Service:
        def subscriber_builder(self):
            return SimpleNamespace(create=lambda: Subscriber())

        def publisher_builder(self):
            return Builder()

    monkeypatch.setattr(transport, '_load_binding', lambda: FakeIox)
    monkeypatch.setattr(transport, '_new_node', lambda _iox2: _Node())
    monkeypatch.setattr(transport, '_new_service', lambda *_args: Service())

    reader = Iceoryx2Reader('camera/test')
    reader.close()
    reader.close()
    assert reader.read() is None

    publisher = Iceoryx2Publisher('camera/test')
    publisher.close()
    publisher.close()
    with pytest.raises(Iceoryx2Error, match='publisher is closed'):
        publisher.publish(FrameHeader(0, 0, 0, CAMERA_FRONT, 1, 1, 3), b'123')


def jpeg_packet(image=None):
    import cv2
    if image is None:
        image = np.zeros((16, 32, 3), dtype=np.uint8)
    ok, encoded = cv2.imencode('.jpg', image)
    assert ok
    header = FrameHeader(1, 2, 3, CAMERA_FRONT, image.shape[1], image.shape[0],
                         0, encoding=ENCODING_JPEG)
    return FramePacket(header, encoded.tobytes())


def test_jpeg_decode_and_variable_length_reader(monkeypatch):
    import cv2
    images = [np.zeros((16, 32, 3), dtype=np.uint8),
              np.random.default_rng(7).integers(0, 256, (16, 32, 3), dtype=np.uint8)]
    packets = [jpeg_packet(image) for image in images]
    assert len(packets[0].payload) != len(packets[1].payload)
    for packet in packets:
        native = _native_header(width=32, height=16, stride=0)
        native.encoding = ENCODING_JPEG
        sample = _Sample(native, packet.payload)
        with _make_reader(monkeypatch, sample) as reader:
            received = reader.read()
        np.testing.assert_array_equal(
            received.bgr(), cv2.imdecode(np.frombuffer(packet.payload, np.uint8), cv2.IMREAD_COLOR))
        assert received.payload == packet.payload
        assert sample.deleted


def test_jpeg_validation_and_decoder_failures(monkeypatch):
    from dataclasses import replace
    import cv2
    packet = jpeg_packet()
    for invalid in (
        FramePacket(packet.header, packet.payload[:-2]),
        FramePacket(replace(packet.header, stride=96), packet.payload),
        FramePacket(replace(packet.header, width=31), packet.payload),
        FramePacket(packet.header, b'\xff\xd8\xff\xe0\xff\xff\xff\xd9'),
    ):
        with pytest.raises(InvalidFrameError):
            invalid.validate()
    monkeypatch.setattr(cv2, 'imdecode', lambda *_args: None)
    with pytest.raises(InvalidFrameError, match='decode failed'):
        packet.bgr()


def test_publisher_copies_exact_jpeg_and_rejects_bad_payload():
    packet = jpeg_packet()
    native = transport._IceoryxFrameHeader()
    buffer = (ctypes.c_uint8 * len(packet.payload))()
    sent = []

    class Loan:
        payload_ptr = ctypes.addressof(buffer)

        def user_header(self):
            return SimpleNamespace(contents=native)

        def assume_init(self):
            return self

        def send(self):
            sent.append(True)

    publisher = Iceoryx2Publisher.__new__(Iceoryx2Publisher)
    publisher._closed = False
    publisher.service = 'camera/test'
    sizes = []

    def loan(size):
        sizes.append(size)
        return Loan()

    publisher._publisher = SimpleNamespace(loan_slice_uninit=loan)
    publisher.publish(packet.header, packet.payload)
    assert bytes(buffer) == packet.payload
    assert sizes == [len(packet.payload)]
    assert sent == [True]
    assert transport._header_from_binding(native) == packet.header
    with pytest.raises(InvalidFrameError):
        publisher.publish(packet.header, packet.payload[:-1])
    assert len(sent) == 1
