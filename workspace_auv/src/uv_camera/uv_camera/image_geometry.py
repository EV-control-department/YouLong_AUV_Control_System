"""Lens geometry shared by inference, task rays and annotated streams.

No ROS runtime is required. K includes skew; rotations/TF are deliberately
outside this module. The output has the original K and image dimensions.
"""
from __future__ import annotations

from collections import OrderedDict
import copy
import hashlib
import struct
import threading

import cv2
import numpy as np


def normalized_pixel(matrix, distortion, x, y):
    k = np.asarray(matrix, dtype=np.float64).reshape(3, 3)
    pixel = np.linalg.solve(k, np.array([x, y, 1.0], dtype=np.float64))
    point = pixel[:2] / pixel[2]
    d = np.asarray(distortion, dtype=np.float64).reshape(-1)
    if np.any(d):
        point = cv2.undistortPoints(point.reshape(1, 1, 2), np.eye(3), d).reshape(2)
    if not np.all(np.isfinite(point)):
        raise ValueError('invalid calibrated pixel')
    return tuple(map(float, point))


def validate_info(info):
    k = np.asarray(info.k, dtype=np.float64).reshape(3, 3)
    d = np.asarray(info.d, dtype=np.float64).reshape(-1)
    if (int(info.width) <= 0 or int(info.height) <= 0
            or not np.all(np.isfinite(k)) or not np.all(np.isfinite(d))
            or k[0, 0] <= 0 or k[1, 1] <= 0
            or not np.allclose(k[2], [0, 0, 1]) or abs(k[1, 0]) > 1e-12
            or len(d) not in (0, 4, 5, 8, 12, 14)):
        raise ValueError('invalid camera intrinsics/dimensions')
    if getattr(info, 'distortion_model', '') not in ('', 'plumb_bob', 'rational_polynomial'):
        raise ValueError('unsupported camera distortion model')
    return k, d


def calibration_id(info, version):
    k, d = validate_info(info)
    payload = (struct.pack('<III', int(info.width), int(info.height), int(version))
               + k.astype('<f8').tobytes() + d.astype('<f8').tobytes())
    return int.from_bytes(hashlib.sha256(b'lens-k-preserved-v1'+payload).digest()[:8], 'little') or 1


class EyeUndistorter:
    def __init__(self, info, version=1):
        self.k, self.d = validate_info(info)
        self.width, self.height = int(info.width), int(info.height)
        self.version = int(version)
        self.calibration_id = calibration_id(info, self.version)
        self.source_info = copy.deepcopy(info)
        self.image_info = copy.deepcopy(info)
        self.image_info.d = [0.0] * (len(self.d) or 5)
        self.image_info.distortion_model = 'rational_polynomial' if len(self.d) > 5 else 'plumb_bob'
        self.image_info.r = np.eye(3).reshape(-1).tolist()
        self.image_info.p = np.column_stack((self.k, np.zeros(3))).reshape(-1).tolist()
        self.maps = None
        self.valid = np.ones((self.height, self.width), dtype=bool)
        if np.any(self.d):
            mx, my = cv2.initUndistortRectifyMap(self.k, self.d, np.eye(3), self.k,
                                               (self.width, self.height), cv2.CV_32FC1)
            # OpenCV applies the inverse output K including skew, but source
            # projection omits K[0,1]. Restore it explicitly (also on Foxy).
            mx += np.float32(self.k[0, 1] / self.k[1, 1]) * (my - np.float32(self.k[1, 2]))
            self.maps = mx, my
            coverage = cv2.remap(np.full(self.valid.shape, 255, np.uint8), mx, my,
                                 cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
            self.valid = coverage == 255

    def apply(self, image):
        if image.shape[:2] != (self.height, self.width):
            raise ValueError('camera image and calibration dimensions differ')
        if self.maps is None:
            return image
        return cv2.remap(image, *self.maps, interpolation=cv2.INTER_LINEAR,
                         borderMode=cv2.BORDER_CONSTANT)

    def valid_point(self, x, y):
        if not np.isfinite(x) or not np.isfinite(y):
            return False
        ix, iy = int(round(x)), int(round(y))
        return 0 <= ix < self.width and 0 <= iy < self.height and bool(self.valid[iy, ix])

    def message(self, camera_name):
        from uv_msgs.msg import PerceptionCameraInfo
        result = PerceptionCameraInfo()
        result.camera_name = camera_name
        result.camera_info_version = self.version
        result.calibration_id = self.calibration_id
        result.source_info = copy.deepcopy(self.source_info)
        result.image_info = copy.deepcopy(self.image_info)
        return result


class CalibrationCache:
    """Bounded metadata cache. Never interpret corrected pixels with raw D."""
    def __init__(self):
        self._lock = threading.RLock()
        self._entries = OrderedDict()

    def add(self, message):
        source_k, _ = validate_info(message.source_info)
        image_k, image_d = validate_info(message.image_info)
        if (str(message.camera_name) not in ('front_left', 'front_right', 'down_left', 'down_right')
                or int(message.calibration_id) != calibration_id(message.source_info, message.camera_info_version)
                or np.any(image_d)
                or (message.source_info.width, message.source_info.height) !=
                   (message.image_info.width, message.image_info.height)
                or not np.allclose(source_k, image_k)):
            raise ValueError('inconsistent perception calibration')
        # Validation above is intentionally cheap: consumers build maps only
        # when selected video frames actually need them.
        key = (str(message.camera_name), int(message.calibration_id))
        with self._lock:
            self._entries[key] = copy.deepcopy(message)
            self._entries.move_to_end(key)
            while len(self._entries) > 64:
                self._entries.popitem(last=False)

    def get(self, camera_name, calibration_id):
        with self._lock:
            return self._entries.get((str(camera_name), int(calibration_id)))

    def latest(self, camera_name, version):
        with self._lock:
            for (name, _), entry in reversed(self._entries.items()):
                if name == camera_name and int(entry.camera_info_version) == int(version):
                    return entry
        return None

    def resolve(self, message, raw_info=None):
        space = int(getattr(message, 'image_space', 0))
        if space == 0:
            return raw_info
        if space != 1:
            return None
        entry = self.get(message.camera_name, getattr(message, 'calibration_id', 0))
        if entry is None or int(entry.camera_info_version) != int(getattr(message, 'camera_info_version', 0)):
            return None
        return entry.image_info
