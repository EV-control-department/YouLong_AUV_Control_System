"""Reader for Stonefish simulator RGB frames in POSIX shared memory.

The writer owns one latest-frame ring per simulated eye.  This module never
creates ROS Image subscriptions or publishers: it maps the rings, snapshots a
consistent slot, and hands a stitched BGR NumPy frame to ``uv_camera``.
"""

from __future__ import annotations

import mmap
import os
import struct
from pathlib import Path

import cv2
import numpy as np


_MAGIC = 0x55564331
_VERSION = 1
_HEADER = struct.Struct("<8I2Q2q")
_HEADER_BYTES = _HEADER.size


def _shm_path(prefix: str, channel: str) -> Path:
    prefix = str(prefix or "/uv_sim_camera").strip()
    if not prefix.startswith("/"):
        prefix = "/" + prefix
    prefix = prefix.rstrip("/") or "/uv_sim_camera"
    safe = channel.replace("/", "_").replace(" ", "_")
    return Path("/dev/shm") / (prefix.lstrip("/") + "_" + safe)


class _ChannelReader:
    def __init__(self, path: Path):
        self.path = path
        self._fd = None
        self._mapping = None
        self._mapping_size = 0
        self._last_sequence = 0
        self._last_result = None
        self.last_error = ""

    def _open(self) -> bool:
        if self._mapping is not None:
            return True
        try:
            fd = os.open(str(self.path), os.O_RDONLY)
            size = os.fstat(fd).st_size
            if size < _HEADER_BYTES:
                os.close(fd)
                return False
            mapping = mmap.mmap(fd, size, access=mmap.ACCESS_READ)
        except OSError as error:
            self.last_error = str(error)
            return False
        self._fd = fd
        self._mapping = mapping
        self._mapping_size = size
        return True

    def read(self):
        if not self._open():
            return None
        mapping = self._mapping
        try:
            fields = _HEADER.unpack_from(mapping, 0)
            magic, version, width, height, step, channels, slots, _reserved, slot_bytes, sequence, sec, nsec = fields
            if (magic != _MAGIC or version != _VERSION or channels != 3
                    or width <= 0 or height <= 0 or step < width * 3
                    or slots < 2 or slot_bytes < height * step):
                self.close()
                return None
            if _HEADER_BYTES + slot_bytes * slots > self._mapping_size:
                self.close()
                return None
            if sequence == 0 or sequence & 1:
                return None
            # The timer polls faster than Stonefish renders.  Reuse the last
            # decoded frame when the writer has not advanced the ring; this
            # keeps the poll cheap while still allowing the stereo pairer to
            # combine a new eye with a cached eye from the same capture step.
            if sequence == self._last_sequence and self._last_result is not None:
                return self._last_result
            slot = (sequence // 2 - 1) % slots
            start = _HEADER_BYTES + slot * slot_bytes
            payload_bytes = height * step
            raw = np.frombuffer(
                memoryview(mapping)[start:start + payload_bytes],
                dtype=np.uint8).copy()
            sequence_after = struct.unpack_from("<Q", mapping, 40)[0]
            if sequence_after != sequence or sequence_after & 1:
                return None
            rows = raw.reshape(height, step)[:, :width * 3]
            rgb = rows.reshape(height, width, 3)
            # Stonefish's ColorCamera buffer and ROS prototype are RGB8.
            frame = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            result = frame, int(sec), int(nsec), int(sequence // 2)
            self._last_sequence = sequence
            self._last_result = result
            return result
        except (BufferError, IndexError, struct.error, ValueError):
            self.close()
            return None

    def close(self):
        if self._mapping is not None:
            self._mapping.close()
            self._mapping = None
        if self._fd is not None:
            os.close(self._fd)
            self._fd = None
        self._mapping_size = 0
        self._last_sequence = 0
        self._last_result = None


class SimStereoShmSource:
    """Poll latest complete left/right frames for the front and down cameras."""

    def __init__(self, enable_front=True, enable_down=True, prefix=None):
        prefix = prefix or os.environ.get("UV_SIM_SHM_PREFIX", "/uv_sim_camera")
        self._readers = {}
        for camera, enabled in (("front", enable_front), ("down", enable_down)):
            if enabled:
                self._readers[camera] = (
                    _ChannelReader(_shm_path(prefix, f"{camera}_left")),
                    _ChannelReader(_shm_path(prefix, f"{camera}_right")),
                )
        self._last_pair = {camera: None for camera in self._readers}

    def poll(self, camera):
        readers = self._readers.get(camera)
        if readers is None:
            return None
        left = readers[0].read()
        right = readers[1].read()
        if left is None or right is None:
            return None
        left_frame, left_sec, left_nsec, left_seq = left
        right_frame, right_sec, right_nsec, right_seq = right
        left_time = left_sec + left_nsec * 1e-9
        right_time = right_sec + right_nsec * 1e-9
        if abs(left_time - right_time) > (0.12 if camera == "front" else 0.04):
            return None
        pair_key = (left_seq, right_seq)
        if pair_key == self._last_pair[camera]:
            return None
        self._last_pair[camera] = pair_key
        frame = np.hstack((left_frame, right_frame))
        return (
            frame,
            (left_sec, left_nsec),
            (right_sec, right_nsec),
            max(left_seq, right_seq),
        )

    def close(self):
        for readers in self._readers.values():
            for reader in readers:
                reader.close()

