"""Calibrated front-eye detection rays for one selected canonical target."""
from collections import deque
from dataclasses import dataclass
import math
import threading
import time

import cv2
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy
from auv_protocol.topics import PERCEPTION_DETECTIONS
from uv_msgs.msg import DetectionArray
from uv_task.down_camera_servo import best_detection


def wrap_degrees(angle):
    return (float(angle)+180.0) % 360.0-180.0


def rotation(pose):
    r, p, y = map(math.radians, pose[3:6])
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                             math.sin(p), math.cos(y), math.sin(y))
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                     [-sp, cp*sr, cp*cr]])


@dataclass(frozen=True)
class FrontObservation:
    eye: str
    sequence: int
    received: float
    stamp: float
    pair_id: int
    origin: np.ndarray
    ray: np.ndarray
    area: float


class FrontTargetObserver:
    def __init__(self, node, params):
        self.node = node
        self.p = params
        self.now = time.monotonic
        self.lock = threading.RLock()
        self.class_id = None
        self.sequence = 0
        self.first_eye = None
        self.latest = {eye: None for eye in ('front_left', 'front_right')}
        self.history = {eye: deque(maxlen=16) for eye in self.latest}
        self.last_capture = {}
        self.sub = node.create_subscription(
            DetectionArray, PERCEPTION_DETECTIONS, self._callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

    def destroy(self):
        if self.sub is not None:
            self.node.destroy_subscription(self.sub)
            self.sub = None

    def select(self, class_id):
        with self.lock:
            self.class_id = class_id
            self.first_eye = None
            for eye in self.latest:
                self.latest[eye] = None
                self.history[eye].clear()

    def cursor(self):
        with self.lock:
            return self.sequence

    def fresh(self, frame):
        return frame is not None and self.now()-frame.received <= self.p['search_detection_timeout']

    def frames(self, after=-1):
        with self.lock:
            return sorted((f for f in self.latest.values() if self.fresh(f)
                           and f.sequence > after), key=lambda f: f.sequence)

    def _callback(self, msg):
        eye = str(msg.camera_name).strip().lower()
        if eye not in self.latest:
            return
        with self.lock:
            if self.class_id is None:
                return
            capture_id = int(getattr(msg, 'capture_id', 0))
            if capture_id and self.last_capture.get(eye) == capture_id:
                return
            if capture_id:
                self.last_capture[eye] = capture_id
            self.sequence += 1
            stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            if stamp and abs(self.node.get_clock().now().nanoseconds*1e-9-stamp) > self.p['search_detection_timeout']:
                self.latest[eye] = None
                return
            detection = best_detection(msg, self.class_id)
            if detection is None or detection.confidence < self.p['search_min_confidence']:
                self.latest[eye] = None
                return
            try:
                calibration = self.node.camera_configs['front'].side(eye.removeprefix('front_'))
                xy = cv2.undistortPoints(
                    np.array([[[detection.pixel_x, detection.pixel_y]]], dtype=float),
                    calibration.matrix, calibration.distortion).reshape(2)
                pose = tuple(float(x) for x in self.node._latest_robot_pose())
                if len(pose) != 6 or not all(math.isfinite(x) for x in pose):
                    raise ValueError('实测位姿无效')
                ext = self.node.camera_extrinsics[eye]
                transform = rotation(pose)
                ray = transform @ ext.optical_to_body @ np.array([*xy, 1.0])
                norm = np.linalg.norm(ray)
                if not math.isfinite(norm) or norm < 1e-9:
                    raise ValueError('射线无效')
                ray /= norm
                origin = np.asarray(pose[:3])+transform @ ext.translation
                if not np.all(np.isfinite(origin)) or np.linalg.norm(ray[:2]) < 1e-6:
                    raise ValueError('水平射线无效')
                area = max(0.0, float(detection.bbox_x2-detection.bbox_x1)) * max(
                    0.0, float(detection.bbox_y2-detection.bbox_y1))
            except (KeyError, ValueError, TypeError, cv2.error):
                self.latest[eye] = None
                return
            frame = FrontObservation(eye, self.sequence, self.now(), stamp,
                                     int(getattr(msg, 'stereo_pair_id', 0)), origin, ray, area)
            self.latest[eye] = frame
            self.history[eye].append(frame)
            if self.first_eye is None:
                self.first_eye = eye

    def stereo_center(self, after=-1):
        with self.lock:
            if not all(self.fresh(f) for f in self.latest.values()):
                return None
            left = tuple(reversed(self.history['front_left']))
            right = tuple(reversed(self.history['front_right']))
        for a in left:
            for b in right:
                if (not self.fresh(a) or not self.fresh(b)
                        or min(a.sequence, b.sequence) <= after):
                    continue
                if a.pair_id and b.pair_id:
                    if a.pair_id != b.pair_id:
                        continue
                elif not a.stamp or not b.stamp or abs(a.stamp-b.stamp) > self.p['search_stereo_pair_slop']:
                    continue
                if abs(a.received-b.received) > self.p['search_stereo_pair_slop']:
                    continue
                angle = math.degrees(math.acos(float(np.clip(a.ray @ b.ray, -1, 1))))
                if angle < self.p['search_stereo_min_angle_deg']:
                    continue
                distances = np.linalg.lstsq(np.column_stack((a.ray, -b.ray)),
                                           b.origin-a.origin, rcond=None)[0]
                if (not np.all(np.isfinite(distances)) or min(distances) <= 0
                        or max(distances) > self.p['search_stereo_max_distance_m']):
                    continue
                x, y = a.origin+distances[0]*a.ray, b.origin+distances[1]*b.ray
                if np.linalg.norm(x-y) > self.p['search_stereo_max_ray_gap_m']:
                    continue
                if (not math.isfinite(a.area+b.area) or min(a.area, b.area) <= 0
                        or max(a.area, b.area)/min(a.area, b.area) > 2.5):
                    continue
                return (x+y)/2
        return None
