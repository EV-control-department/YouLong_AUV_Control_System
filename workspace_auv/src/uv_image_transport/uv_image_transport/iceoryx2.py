"""Direct Python binding for the iceoryx2 camera data plane.

The official iceoryx2 PyO3 binding is used in-process. There is deliberately
no subprocess, pipe, native bridge, DDS image topic, or private wire format
here: the image payload stays in an iceoryx2 publish/subscribe sample until a
consumer turns it into a NumPy frame.
"""

from __future__ import annotations

from dataclasses import dataclass
import ctypes
import threading
from typing import Any

import numpy as np


CAMERA_FRONT = 1
CAMERA_DOWN = 2
ENCODING_BGR8 = 1
_POLL_INTERVAL = 0.002


class _IceoryxFrameHeader(ctypes.Structure):
    """Shared user header for every camera sample."""

    _fields_ = [
        ('capture_id', ctypes.c_uint64),
        ('timestamp_ns', ctypes.c_uint64),
        ('stereo_pair_id', ctypes.c_uint64),
        ('camera_group', ctypes.c_uint32),
        ('width', ctypes.c_uint32),
        ('height', ctypes.c_uint32),
        ('stride', ctypes.c_uint32),
        ('encoding', ctypes.c_uint32),
        ('camera_info_version', ctypes.c_uint32),
    ]

    @staticmethod
    def type_name() -> str:
        return 'YoulongCameraFrameHeader'


class Iceoryx2Error(RuntimeError):
    """Raised when the direct iceoryx2 Python binding cannot be used."""


def _load_binding() -> Any:
    try:
        import iceoryx2
    except ImportError as error:  # pragma: no cover - deployment error
        raise Iceoryx2Error(
            'iceoryx2 Python binding is not installed; build/install '
            'third_party/iceoryx2/iceoryx2-ffi/python first') from error
    return iceoryx2


@dataclass(frozen=True)
class FrameHeader:
    capture_id: int
    timestamp_ns: int
    stereo_pair_id: int
    camera_group: int
    width: int
    height: int
    stride: int
    encoding: int = ENCODING_BGR8
    camera_info_version: int = 0

    @property
    def camera_name(self) -> str:
        if self.camera_group == CAMERA_FRONT:
            return 'front'
        if self.camera_group == CAMERA_DOWN:
            return 'down'
        return f'camera_{self.camera_group}'


@dataclass(frozen=True)
class FramePacket:
    header: FrameHeader
    payload: bytes

    def bgr(self) -> np.ndarray:
        """Create a NumPy view-compatible array from a BGR8 payload."""
        header = self.header
        if header.encoding != ENCODING_BGR8:
            raise Iceoryx2Error(
                f'unsupported frame encoding {header.encoding}; expected BGR8')
        required = header.height * header.stride
        if len(self.payload) < required:
            raise Iceoryx2Error(
                f'frame payload is short: {len(self.payload)} < {required}')
        raw = np.frombuffer(self.payload[:required], dtype=np.uint8)
        return raw.reshape(header.height, header.stride)[:, :header.width * 3].reshape(
            header.height, header.width, 3)


def _new_node(iox2: Any) -> Any:
    iox2.set_log_level_from_env_or(iox2.LogLevel.Warn)
    return iox2.NodeBuilder.new().create(iox2.ServiceType.Ipc)


def _new_service(node: Any, service: str, iox2: Any) -> Any:
    return (
        node.service_builder(iox2.ServiceName.new(str(service)))
        .publish_subscribe(iox2.Slice[ctypes.c_uint8])
        .user_header(_IceoryxFrameHeader)
        .open_or_create()
    )


def _header_from_binding(value: _IceoryxFrameHeader) -> FrameHeader:
    return FrameHeader(
        capture_id=int(value.capture_id),
        timestamp_ns=int(value.timestamp_ns),
        stereo_pair_id=int(value.stereo_pair_id),
        camera_group=int(value.camera_group),
        width=int(value.width),
        height=int(value.height),
        stride=int(value.stride),
        encoding=int(value.encoding),
        camera_info_version=int(value.camera_info_version),
    )


class Iceoryx2Reader:
    """Receive camera frames directly from an iceoryx2 subscriber."""

    def __init__(self, service: str):
        self.service = str(service)
        self._closed = threading.Event()
        self._iox2 = _load_binding()
        self._node = _new_node(self._iox2)
        self._service = _new_service(self._node, self.service, self._iox2)
        self._subscriber = self._service.subscriber_builder().create()
        self._poll_duration = self._iox2.Duration.from_secs_f64(_POLL_INTERVAL)

    def read(self) -> FramePacket | None:
        while not self._closed.is_set():
            try:
                sample = self._subscriber.receive()
            except Exception as error:
                if self._closed.is_set():
                    return None
                raise Iceoryx2Error(
                    f'iceoryx2 receive failed for {self.service}: {error}') from error
            if sample is not None:
                try:
                    header = _header_from_binding(
                        sample.user_header().contents)
                    payload = sample.payload().as_memory_view().tobytes()
                    required = header.height * header.stride
                    if header.width <= 0 or header.height <= 0 or header.stride < header.width * 3:
                        raise Iceoryx2Error(
                            f'invalid frame dimensions for {self.service}: {header}')
                    if len(payload) != required:
                        raise Iceoryx2Error(
                            f'invalid payload length for {self.service}: '
                            f'{len(payload)} != {required}')
                    return FramePacket(header, payload)
                finally:
                    sample.delete()
            try:
                self._node.wait(self._poll_duration)
            except Exception as error:
                if self._closed.is_set():
                    return None
                raise Iceoryx2Error(
                    f'iceoryx2 wait failed for {self.service}: {error}') from error
        return None

    def close(self) -> None:
        self._closed.set()
        self._subscriber = None
        self._service = None
        self._node = None

    def __enter__(self):
        return self

    def __exit__(self, *_args):
        self.close()


class Iceoryx2Publisher:
    """Publish camera frames directly through the iceoryx2 Python binding."""

    def __init__(self, service: str):
        self.service = str(service)
        self._closed = False
        self._iox2 = _load_binding()
        self._node = _new_node(self._iox2)
        self._service = _new_service(self._node, self.service, self._iox2)
        self._publisher = (
            self._service.publisher_builder()
            .initial_max_slice_len(2560 * 960 * 3)
            .allocation_strategy(self._iox2.AllocationStrategy.PowerOfTwo)
            .create()
        )

    def publish(self, header: FrameHeader,
                payload: bytes | bytearray | memoryview) -> None:
        if self._closed:
            raise Iceoryx2Error('iceoryx2 publisher is closed')
        data = bytes(payload)
        expected = header.height * header.stride
        if len(data) != expected:
            raise Iceoryx2Error(f'payload size {len(data)} != {expected}')
        try:
            sample = self._publisher.loan_slice_uninit(len(data))
            native_header = sample.user_header().contents
            native_header.capture_id = int(header.capture_id)
            native_header.timestamp_ns = int(header.timestamp_ns)
            native_header.stereo_pair_id = int(header.stereo_pair_id)
            native_header.camera_group = int(header.camera_group)
            native_header.width = int(header.width)
            native_header.height = int(header.height)
            native_header.stride = int(header.stride)
            native_header.encoding = int(header.encoding)
            native_header.camera_info_version = int(header.camera_info_version)
            ctypes.memmove(sample.payload_ptr, data, len(data))
            sample.assume_init().send()
        except Exception as error:
            raise Iceoryx2Error(
                f'iceoryx2 publish failed for {self.service}: {error}') from error

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._publisher = None
        self._service = None
        self._node = None

    def __enter__(self):
        return self

    def __exit__(self, *_args):
        self.close()
