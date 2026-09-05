"""Common helpers for uv_camera: constants, Kalman/line filters, MJPEG server, cv_bridge.

This module is process-internal; it does not depend on the ROS node itself
(only on ``rclpy.ok`` for the MJPEG serve loop, matching upstream vision.py).
"""

import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np


# ── Runtime switches / constants (moved verbatim from uv_perception.vision) ──
FRONT_CAMERA_DEVICE = '/dev/video2'
DOWN_CAMERA_DEVICE = '/dev/video0'
ENABLE_FRONT_CAMERA = True
ENABLE_DOWN_CAMERA = True

FRONT_CAMERA_RESOLUTION = (1280, 960)  # width, height
DOWN_CAMERA_RESOLUTION = (1280, 960)

FRONT_CAPTURE_RESOLUTION = (1280, 720)
DOWN_CAPTURE_RESOLUTION = (2560, 960)

FRONT_CAMERA_MATRIX = (
    2158.4, 0.0, 640.0,
    0.0, 2158.4, 480.0,
    0.0, 0.0, 1.0,
)
FRONT_DIST_COEFFS = (0.0, 0.0, 0.0, 0.0, 0.0)

DOWN_CAMERA_MATRIX = (
    2307.6, 0.0, 640.0,
    0.0, 2307.6, 480.0,
    0.0, 0.0, 1.0,
)
DOWN_DIST_COEFFS = (0.0, 0.0, 0.0, 0.0, 0.0)

CONFIDENCE = 0.8

ENABLE_UNDISTORT = True
ENABLE_GORTC = True
GORTC_HTTP_PORT = 1984
VISION_MJPEG_PORT = 8090
GORTC_EXECUTABLE = 'go2rtc'
STREAM_ANNOTATED = True

SIM_MODE = False
SAVE_DATASET = False
DATASET_DIR = ''
DEFAULT_MODEL_FILENAME = 'robotcup20260901.pt'


class _ScalarKalman:
    """One-dimensional Kalman filter (used to smooth pipe center/angle)."""

    def __init__(self, initial_value, process_noise=1.0, measurement_noise=3.0):
        self.x = float(initial_value)
        self.p = 100.0
        self.q = max(1e-6, float(process_noise))
        self.r = max(1e-6, float(measurement_noise))

    def predict(self):
        self.p += self.q
        return self.x

    def update(self, measurement):
        gain = self.p / (self.p + self.r)
        self.x += gain * (float(measurement) - self.x)
        self.p = (1.0 - gain) * self.p
        return self.x


class _LineFilterState:
    """Holds one camera's image width + center/heading filters."""

    def __init__(self, image_width, process_noise, measurement_noise):
        self.image_width = int(image_width)
        self.kalman_center = _ScalarKalman(
            self.image_width / 2.0, process_noise, measurement_noise)
        self.kalman_heading = _ScalarKalman(
            0.0,
            max(1e-6, process_noise * 0.5),
            max(1e-6, measurement_noise * 0.5),
        )


class _MjpegHandler(BaseHTTPRequestHandler):
    """Very small MJPEG source used by go2rtc (and directly by a browser)."""

    node = None  # set to the composed uv_camera node that owns the stream caches

    def do_GET(self):  # noqa: N802
        stream = self.path.strip('/').split('?', 1)[0]
        valid_streams = {
            'front': ('front', False),
            'down': ('down', False),
            'front_annotated': ('front', True),
            'down_annotated': ('down', True),
        }
        if stream not in valid_streams:
            self.send_error(
                404, 'use /front, /down, /front_annotated or /down_annotated')
            return
        camera, annotated = valid_streams[stream]
        if annotated and (self.node is None or not self.node._stream_annotated):
            self.send_error(404, 'annotated streams are disabled')
            return

        self.send_response(200)
        self.send_header('Cache-Control', 'no-cache, private')
        self.send_header('Connection', 'close')
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.end_headers()
        try:
            last_sequence = 0
            while self.node is not None and self.node._rclpy_ok():
                result = self.node._wait_for_stream_frame(camera, annotated, last_sequence)
                if result is None:
                    break
                payload, last_sequence, stamp_ns = result
                part_header = (
                    f'--frame\r\nContent-Type: image/jpeg\r\n'
                    f'X-Frame-Sequence: {last_sequence}\r\n'
                    f'X-Frame-Stamp-Ns: {stamp_ns}\r\n'
                    f'Content-Length: {len(payload)}\r\n\r\n'
                ).encode('ascii')
                self.wfile.write(part_header)
                self.wfile.write(payload)
                self.wfile.write(b'\r\n')
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass

    def log_message(self, *_args):
        pass


class _MjpegServer(ThreadingHTTPServer):
    """MJPEG server whose client threads cannot keep node shutdown alive."""

    daemon_threads = True
    allow_reuse_address = True


CvBridge = None


def import_cv_bridge():
    """Lazily import CvBridge once. Returns the CvBridge class or raises."""
    global CvBridge
    if CvBridge is None:
        from cv_bridge import CvBridge as _CB
        CvBridge = _CB
    return CvBridge


def normalize_frame(frame):
    """Idempotent BGR normalization: reject empty; GRAY->BGR, BGRA->BGR."""
    if not isinstance(frame, np.ndarray) or frame.size == 0:
        return None
    if frame.ndim == 2:
        return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    if frame.ndim != 3 or frame.shape[2] not in (3, 4):
        return None
    if frame.shape[2] == 4:
        return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
    return frame


class FrameGate:
    """Single-latest in-process frame handoff with pool + anti-backlog.

    This is the A3 bridge between uv_sensor (producer) and uv_ai (consumer):
    * uv_sensor calls ``submit(camera, work)`` with the newest frame.
    * uv_ai provides ``consumer(camera, work)`` which runs detection.
    * Only ONE worker per camera runs at a time; while it runs, new frames
      overwrite ``_pending_work[camera]`` (latest-wins, drop-oldest), so a slow
      detector never accumulates backlog. Mirrors the original vision.py
      ``_submit_work`` flow-control exactly.
    """

    def __init__(self, consumer, cameras=('front', 'down'), max_workers=2,
                 log_warn=None):
        from concurrent.futures import ThreadPoolExecutor
        self.consumer = consumer
        self._log = log_warn or (lambda m: None)
        self._pool = ThreadPoolExecutor(max_workers=max_workers,
                                        thread_name_prefix='vision')
        self._lock = threading.Lock()
        self._processing = {c: False for c in cameras}
        self._pending_work = {c: None for c in cameras}
        self._stop = threading.Event()

    def submit(self, camera: str, work):
        if camera not in self._processing:
            self._log(f'Unknown camera source: {camera}')
            return
        with self._lock:
            self._pending_work[camera] = work
            if self._processing[camera]:
                return
            self._processing[camera] = True

        def worker():
            while True:
                with self._lock:
                    work_item = self._pending_work[camera]
                    self._pending_work[camera] = None
                    if work_item is None:
                        self._processing[camera] = False
                        return
                try:
                    self.consumer(camera, work_item)
                except Exception as e:
                    self._log(f'Frame processing failed ({camera}): {e}')

        try:
            self._pool.submit(worker)
        except RuntimeError:
            with self._lock:
                self._pending_work[camera] = None
                self._processing[camera] = False
            self._log(f'Cannot schedule {camera} frame')

    def shutdown(self):
        self._stop.set()
        self._pool.shutdown(wait=True, cancel_futures=True)
