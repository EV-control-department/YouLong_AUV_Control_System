"""Crash-resilient, lossless recorder for sampled YOLO input frames.

The recorder deliberately stores decoded BGR frames as PNG.  PNG is lossless
with respect to the BGR array handed to the detector; it does not attempt to
reconstruct the original camera MJPG bitstream.

Each process creates a new session directory.  A frame is first written to a
temporary PNG, fsynced, atomically renamed into ``images/<channel>``, and only
then appended to the fsynced JSONL manifest.  Therefore a power loss can leave
an ignored temporary file or a final PNG, but cannot leave a manifest entry
pointing at a partially written PNG.
"""

import json
import io
import os
import queue
import threading
import time
import zlib
from pathlib import Path
from typing import Any, Dict, Optional, Union

import cv2
import numpy as np


class DatasetRecorder:
    """Asynchronously persist every frame submitted by the YOLO path.

    ``submit`` applies backpressure when the queue is full instead of silently
    dropping a frame.  This protects the dataset's frame completeness; the
    trade-off is that a slow disk can slow the inference worker.
    """

    _SENTINEL = object()

    def __init__(
        self,
        root_dir: Union[str, Path],
        queue_size: int = 32,
        png_compression: int = 3,
        image_format: str = 'png',
        logger: Optional[Any] = None,
        debug: bool = False,
        debug_period_s: float = 1.0,
    ):
        self._logger = logger
        self._debug_enabled = bool(debug)
        self._debug_period_s = max(0.1, float(debug_period_s))
        self._root_dir = Path(root_dir).expanduser()
        self._root_dir.mkdir(parents=True, exist_ok=True)
        self._session_dir = self._make_session_dir(self._root_dir)
        self._images_dir = self._session_dir / 'images'
        self._images_dir.mkdir()
        self._fsync_directory(self._session_dir)
        self._manifest_path = self._session_dir / 'frames.jsonl'
        self._status_path = self._session_dir / 'status.json'
        self._manifest = open(self._manifest_path, 'a', encoding='utf-8')
        self._queue = queue.Queue(maxsize=max(1, int(queue_size)))
        self._png_compression = min(9, max(0, int(png_compression)))
        requested_format = str(image_format).strip().lower()
        if requested_format not in {'png', 'webp_lossless'}:
            raise ValueError(
                f'unsupported dataset image format: {image_format!r}')
        self._image_format = requested_format
        if self._image_format == 'webp_lossless':
            try:
                from PIL import features
                if not features.check('webp'):
                    raise ImportError('Pillow has no WebP encoder')
            except (ImportError, OSError):
                self._image_format = 'png'
                if self._logger is not None:
                    warn = getattr(self._logger, 'warn', None)
                    if warn is None:
                        warn = getattr(self._logger, 'warning', None)
                    if warn is not None:
                        warn('Pillow is unavailable; dataset recorder fell back '
                             'to lossless PNG')

        self._state_lock = threading.Lock()
        self._submit_lock = threading.Lock()
        self._accepting = True
        self._closed = False
        self._worker_error: Optional[BaseException] = None
        self._next_sequence = 0
        self._submitted = 0
        self._written = 0
        self._failed = 0
        self._last_status_s = 0.0
        self._debug_last_log_s = 0.0
        self._debug_blocked_submits = 0
        self._debug_blocked_wait_s = 0.0
        self._debug_last_write_ms = 0.0
        self._debug_max_write_ms = 0.0

        self._write_json_atomic(
            self._session_dir / 'session.json',
            {
                'format': 'yolo_input_images_v1',
                'created_unix_ns': time.time_ns(),
                'source': 'decoded BGR sensor frame split/undistorted as YOLO input',
                'pixel_format': 'bgr8',
                'image_format': self._image_format,
                'png_compression': self._png_compression,
            },
        )
        self._write_status('recording')
        self._worker = threading.Thread(
            target=self._run, name='dataset-png-writer', daemon=True)
        self._worker.start()

    @property
    def session_dir(self) -> Path:
        return self._session_dir

    @property
    def written(self) -> int:
        with self._state_lock:
            return self._written

    @property
    def submitted(self) -> int:
        with self._state_lock:
            return self._submitted

    @property
    def image_format(self) -> str:
        return self._image_format

    @staticmethod
    def _make_session_dir(root_dir: Path) -> Path:
        base = time.strftime('session_%Y%m%d_%H%M%S') + f'_{os.getpid()}'
        candidate = root_dir / base
        suffix = 1
        while True:
            try:
                candidate.mkdir()
                DatasetRecorder._fsync_directory(root_dir)
                return candidate
            except FileExistsError:
                candidate = root_dir / f'{base}_{suffix}'
                suffix += 1

    @staticmethod
    def _fsync_directory(directory: Path) -> None:
        fd = os.open(str(directory), os.O_RDONLY)
        try:
            os.fsync(fd)
        finally:
            os.close(fd)

    @staticmethod
    def _write_bytes_fsync(path: Path, data: bytes) -> None:
        fd = os.open(str(path), os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o644)
        try:
            view = memoryview(data)
            while view:
                written = os.write(fd, view)
                view = view[written:]
            os.fsync(fd)
        finally:
            os.close(fd)

    def _write_json_atomic(self, path: Path, value: Dict[str, Any]) -> None:
        data = (json.dumps(value, ensure_ascii=False, sort_keys=True) + '\n').encode('utf-8')
        tmp = path.with_name(f'.{path.name}.{os.getpid()}.tmp')
        self._write_bytes_fsync(tmp, data)
        os.replace(str(tmp), str(path))
        self._fsync_directory(path.parent)

    def _write_status(self, state: str) -> None:
        with self._state_lock:
            payload = {
                'state': state,
                'session_dir': str(self._session_dir),
                'updated_unix_ns': time.time_ns(),
                'frames_submitted': self._submitted,
                'frames_written': self._written,
                'frames_failed': self._failed,
                'queue_depth': self._queue.qsize(),
            }
        self._write_json_atomic(self._status_path, payload)

    def _set_worker_error(self, error: BaseException) -> None:
        with self._state_lock:
            if self._worker_error is None:
                self._worker_error = error
            self._failed += 1

    def _raise_worker_error(self) -> None:
        with self._state_lock:
            error = self._worker_error
        if error is not None:
            raise RuntimeError(f'dataset writer failed: {error}')

    @staticmethod
    def _stamp_ns(header: Optional[Any]) -> Optional[int]:
        if header is None or not hasattr(header, 'stamp'):
            return None
        stamp = header.stamp
        try:
            return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        except (AttributeError, TypeError, ValueError):
            return None

    def submit(
        self,
        image: np.ndarray,
        channel: str,
        header: Optional[Any] = None,
        stereo_pair_id: int = 0,
    ) -> bool:
        """Queue one exact YOLO input frame; block rather than drop on pressure."""
        if not isinstance(image, np.ndarray) or image.dtype != np.uint8:
            raise ValueError('DatasetRecorder expects a uint8 BGR ndarray')
        if image.ndim != 3 or image.shape[2] != 3:
            raise ValueError('DatasetRecorder expects an HxWx3 BGR ndarray')

        # The detector and ArUco worker may continue reading the source array,
        # so the queued item must own its pixels.
        image_copy = np.ascontiguousarray(image).copy()
        with self._submit_lock:
            with self._state_lock:
                if not self._accepting or self._closed:
                    return False
                sequence = self._next_sequence
                self._next_sequence += 1
                self._submitted += 1
            item = (sequence, image_copy, str(channel), header, int(stereo_pair_id))
            wait_started = time.monotonic()
            blocked_events = 0
            while True:
                self._raise_worker_error()
                try:
                    self._queue.put(item, timeout=0.5)
                    wait_s = time.monotonic() - wait_started
                    if self._debug_enabled:
                        with self._state_lock:
                            self._debug_blocked_submits += blocked_events
                            self._debug_blocked_wait_s += wait_s
                        self._maybe_log_debug()
                    return True
                except queue.Full:
                    blocked_events += 1
                    continue

    def _write_item(
        self,
        sequence: int,
        image: np.ndarray,
        channel: str,
        header: Optional[Any],
        stereo_pair_id: int,
    ) -> None:
        write_started = time.monotonic()
        channel_dir = self._images_dir / channel
        if not channel_dir.exists():
            channel_dir.mkdir(parents=True)
            self._fsync_directory(self._images_dir)
        if self._image_format == 'webp_lossless':
            from PIL import Image
            output = io.BytesIO()
            # Pillow's explicit lossless flag is required.  OpenCV's WebP
            # quality=100 is not pixel-exact on all builds.
            Image.fromarray(image[:, :, ::-1], mode='RGB').save(
                output, format='WEBP', lossless=True, method=4)
            data = output.getvalue()
            extension = 'webp'
        else:
            ok, encoded = cv2.imencode(
                '.png', image,
                [cv2.IMWRITE_PNG_COMPRESSION, self._png_compression])
            if not ok:
                raise RuntimeError(f'cv2.imencode(.png) failed for {channel}')
            data = encoded.tobytes()
            extension = 'png'
        filename = f'{sequence:012d}.{extension}'
        final_path = channel_dir / filename
        tmp_path = channel_dir / f'.{filename}.{os.getpid()}.tmp.png'
        self._write_bytes_fsync(tmp_path, data)
        os.replace(str(tmp_path), str(final_path))
        self._fsync_directory(channel_dir)

        metadata = {
            'sequence': sequence,
            'channel': channel,
            'path': str(final_path.relative_to(self._session_dir)),
            'stamp_ns': self._stamp_ns(header),
            'frame_id': str(getattr(header, 'frame_id', '')) if header is not None else '',
            'stereo_pair_id': stereo_pair_id,
            'height': int(image.shape[0]),
            'width': int(image.shape[1]),
            'pixel_format': 'bgr8',
            'image_format': self._image_format,
            'bytes': len(data),
            'crc32': f'{zlib.crc32(data) & 0xffffffff:08x}',
        }
        line = (json.dumps(metadata, ensure_ascii=False, sort_keys=True) + '\n')
        self._manifest.write(line)
        self._manifest.flush()
        os.fsync(self._manifest.fileno())
        with self._state_lock:
            self._written += 1
            if self._debug_enabled:
                write_ms = (time.monotonic() - write_started) * 1000.0
                self._debug_last_write_ms = write_ms
                self._debug_max_write_ms = max(
                    self._debug_max_write_ms, write_ms)

    def debug_snapshot(self) -> Dict[str, Union[int, float]]:
        """Return recorder counters used by the opt-in capture diagnostics."""
        with self._state_lock:
            return {
                'submitted': self._submitted,
                'written': self._written,
                'failed': self._failed,
                'queue_depth': self._queue.qsize(),
                'queue_capacity': self._queue.maxsize,
                'blocked_submits': self._debug_blocked_submits,
                'blocked_wait_ms': self._debug_blocked_wait_s * 1000.0,
                'last_write_ms': self._debug_last_write_ms,
                'max_write_ms': self._debug_max_write_ms,
            }

    def _maybe_log_debug(self) -> None:
        if not self._debug_enabled or self._logger is None:
            return
        now = time.monotonic()
        with self._state_lock:
            if now - self._debug_last_log_s < self._debug_period_s:
                return
            self._debug_last_log_s = now
            snapshot = {
                'submitted': self._submitted,
                'written': self._written,
                'failed': self._failed,
                'queue_depth': self._queue.qsize(),
                'queue_capacity': self._queue.maxsize,
                'blocked_submits': self._debug_blocked_submits,
                'blocked_wait_ms': self._debug_blocked_wait_s * 1000.0,
                'last_write_ms': self._debug_last_write_ms,
                'max_write_ms': self._debug_max_write_ms,
            }
            self._debug_blocked_submits = 0
            self._debug_blocked_wait_s = 0.0
            self._debug_max_write_ms = 0.0
        log_info = getattr(self._logger, 'info', None)
        if log_info is not None:
            log_info(
                'dataset-debug recorder: '
                f"submitted={snapshot['submitted']} "
                f"written={snapshot['written']} failed={snapshot['failed']} "
                f"queue={snapshot['queue_depth']}/{snapshot['queue_capacity']} "
                f"blocked_submits={snapshot['blocked_submits']} "
                f"blocked_wait_ms={snapshot['blocked_wait_ms']:.1f} "
                f"last_write_ms={snapshot['last_write_ms']:.1f} "
                f"max_write_ms={snapshot['max_write_ms']:.1f}")

    def _run(self) -> None:
        try:
            while True:
                item = self._queue.get()
                try:
                    if item is self._SENTINEL:
                        return
                    self._write_item(*item)
                    self._maybe_log_debug()
                    now = time.monotonic()
                    if now - self._last_status_s >= 1.0:
                        self._last_status_s = now
                        self._write_status('recording')
                except BaseException as error:
                    self._set_worker_error(error)
                    # Keep the worker alive until close() supplies the
                    # sentinel.  This lets queue.join() complete even after
                    # a disk/encoder error and avoids a shutdown deadlock.
                    while True:
                        pending = self._queue.get()
                        try:
                            if pending is self._SENTINEL:
                                return
                        finally:
                            self._queue.task_done()
                finally:
                    self._queue.task_done()
        except BaseException as error:
            self._set_worker_error(error)

    def close(self) -> None:
        """Stop accepting frames, drain the queue, and mark the session closed."""
        with self._submit_lock:
            with self._state_lock:
                if self._closed:
                    return
                self._accepting = False
                self._closed = True
            self._queue.put(self._SENTINEL)
        self._queue.join()
        self._worker.join(timeout=10.0)
        try:
            self._raise_worker_error()
            self._write_status('stopped')
        except Exception:
            self._write_status('failed')
            raise
        finally:
            self._manifest.flush()
            os.fsync(self._manifest.fileno())
            self._manifest.close()
