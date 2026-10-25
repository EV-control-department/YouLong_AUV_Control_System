"""V4L2 MJPEG acquisition without decoding, resizing, or rotating pixels."""

from dataclasses import dataclass
import fcntl
import math
import os
import stat
import struct
import threading
import time

import cv2
import numpy as np

from uv_image_transport.jpeg import jpeg_dimensions


def _capture_node_error(descriptor):
    # Linux videodev2.h: VIDIOC_QUERYCAP and struct v4l2_capability (104 bytes).
    capability = bytearray(104)
    try:
        fcntl.ioctl(descriptor, 0x80685600, capability, True)
    except OSError:
        return None  # Keep the original OpenCV diagnostic for non-V4L2 devices.
    capabilities, device_caps = struct.unpack_from('=II', capability, 84)
    caps = device_caps if capabilities & 0x80000000 else capabilities
    if not caps & (0x00000001 | 0x00001000):
        kind = 'a UVC metadata-only interface' if caps & 0x00800000 else 'not a video capture interface'
        return (f'this device is {kind}; select the Video Capture node '
                '(usually /dev/v4l/by-id/...-video-index0), not the metadata node')
    return None


def describe_open_failure(device):
    """Preserve actionable OS errors instead of only OpenCV's generic warning."""
    prefix = f'cannot open camera {device}'
    try:
        info = os.stat(device)
        if not stat.S_ISCHR(info.st_mode):
            return f'{prefix}: not a character device'
        if not os.access(device, os.R_OK | os.W_OK):
            return (f'{prefix}: current user lacks read/write permission; '
                    'check the video group and container device mapping')
        descriptor = os.open(device, os.O_RDWR | os.O_NONBLOCK)
        try:
            detail = _capture_node_error(descriptor)
            if detail:
                return f'{prefix}: {detail}'
        finally:
            os.close(descriptor)
    except OSError as error:
        return f'{prefix}: [errno {error.errno}] {error.strerror}'
    return (
        f'{prefix}: device exists and is readable/writable, but V4L2 open failed; '
        f'check capture capability/formats with v4l2-ctl -d {device} --all '
        f'--list-formats-ext, and check other owners with fuser -v {device}')


@dataclass(frozen=True)
class CaptureConfig:
    device: str = ''
    width: int = 1280
    height: int = 720
    fps: float = 30.0
    startup_timeout_sec: float = 5.0
    reconnect_interval_sec: float = 1.0

    def validate(self):
        if not isinstance(self.device, str) or not self.device.strip():
            raise ValueError('device is empty; set the camera device in the sl_camera YAML')
        for name in ('width', 'height'):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise ValueError(f'{name} must be a positive integer')
        for name in ('fps', 'startup_timeout_sec', 'reconnect_interval_sec'):
            value = getattr(self, name)
            if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')


def jpeg_payload(frame, width, height):
    """Validate the full monocular source frame and retain its exact bytes."""
    if (not isinstance(frame, np.ndarray) or frame.dtype != np.uint8
            or frame.size == 0 or frame.ndim not in (1, 2)
            or (frame.ndim == 2 and 1 not in frame.shape)):
        raise ValueError('camera did not return a raw MJPEG byte vector')
    payload = frame.tobytes()
    if jpeg_dimensions(payload) != (width, height):
        raise ValueError(f'camera JPEG does not match configured {width}x{height}')
    return payload


class MjpegCapture:
    """Own the capture handle on one worker, including reconnect and release."""

    def __init__(self, config, clock_ns, on_frame, on_status, on_fps=None):
        config.validate()
        self.config = config
        self.clock_ns = clock_ns
        self.on_frame = on_frame
        self.on_status = on_status
        self.on_fps = on_fps or (lambda value: None)
        self.stop_event = threading.Event()
        self.thread = threading.Thread(
            target=self._run, name='sl-camera-capture', daemon=True)

    def _open_capture(self):
        config = self.config
        cap = cv2.VideoCapture(config.device, cv2.CAP_V4L2)
        try:
            if not cap.isOpened():
                raise RuntimeError(describe_open_failure(config.device))
            for name in ('CAP_PROP_OPEN_TIMEOUT_MSEC', 'CAP_PROP_READ_TIMEOUT_MSEC'):
                prop = getattr(cv2, name, None)
                if prop is not None:
                    cap.set(prop, 3000)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            if not cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG')):
                raise RuntimeError('camera does not support MJPG capture')
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.height)
            cap.set(cv2.CAP_PROP_FPS, config.fps)
            if not cap.set(cv2.CAP_PROP_CONVERT_RGB, 0):
                raise RuntimeError('V4L2 backend cannot disable MJPEG decoding')
            if int(cap.get(cv2.CAP_PROP_FOURCC)) != cv2.VideoWriter_fourcc(*'MJPG'):
                raise RuntimeError('camera negotiated a format other than MJPG')
            actual_fps = cap.get(cv2.CAP_PROP_FPS)
            self.on_fps(actual_fps if math.isfinite(actual_fps) and actual_fps > 0 else 0.0)
            if math.isfinite(actual_fps) and actual_fps > 0:
                if abs(actual_fps - config.fps) > 0.1:
                    self.on_status(
                        f'camera negotiated {actual_fps:g} fps; requested {config.fps:g}')
            return cap
        except Exception:
            cap.release()
            raise

    def start(self):
        self.thread.start()

    def _run(self):
        cap = None
        running = False
        try:
            while not self.stop_event.is_set():
                if cap is None:
                    try:
                        cap = self._open_capture()
                    except (RuntimeError, OSError, cv2.error) as error:
                        self.on_status(f'error: {error}; retrying')
                        self.stop_event.wait(self.config.reconnect_interval_sec)
                        continue
                    last_valid = time.monotonic()
                    running = False
                try:
                    success, frame = cap.read()
                    # Stamp immediately after read, before JPEG inspection or publication.
                    timestamp_ns = self.clock_ns() if success else 0
                    if self.stop_event.is_set():
                        break
                    if not success:
                        raise ValueError('camera read returned no frame')
                    payload = jpeg_payload(frame, self.config.width, self.config.height)
                    self.on_frame(payload, timestamp_ns)
                except (ValueError, RuntimeError, OSError, cv2.error) as error:
                    if time.monotonic() - last_valid >= self.config.startup_timeout_sec:
                        self.on_status(f'error: {error}; reconnecting {self.config.device}')
                        cap.release()
                        cap = None
                        running = False
                        self.stop_event.wait(self.config.reconnect_interval_sec)
                    else:
                        self.stop_event.wait(0.01)
                    continue
                last_valid = time.monotonic()
                if not running:
                    self.on_status('running')
                    running = True
        finally:
            if cap is not None:
                cap.release()

    def stop(self, timeout=5.0):
        self.stop_event.set()
        if self.thread.ident is not None:
            self.thread.join(timeout=timeout)
        return not self.thread.is_alive()
