"""Shared monocular down-camera selection and pixel-servo geometry."""

from __future__ import annotations

import math
import time

import cv2
import numpy as np


def best_detection(message, class_id):
    """Select a finite detection of this class, preferring confidence then area."""
    candidates = []
    for detection in getattr(message, 'detections', ()):
        try:
            if int(detection.class_id) != class_id:
                continue
            px, py, confidence = (float(detection.pixel_x),
                                  float(detection.pixel_y),
                                  float(detection.confidence))
            if not all(math.isfinite(value) for value in (px, py, confidence)):
                continue
            width = max(0.0, float(getattr(detection, 'bbox_x2', px))
                        - float(getattr(detection, 'bbox_x1', px)))
            height = max(0.0, float(getattr(detection, 'bbox_y2', py))
                         - float(getattr(detection, 'bbox_y1', py)))
            area = width * height
            candidates.append((confidence, area if math.isfinite(area) else 0.0,
                               detection))
        except (AttributeError, TypeError, ValueError):
            continue
    return max(candidates, key=lambda item: item[:2])[-1] if candidates else None


class DownCameraPriority:
    """One target's camera lease; create a new instance for every servo phase.

    Callback order decides who acquires a fixed-duration lease. Missing target
    detections, stale messages, and lease expiry release it. Cached observations
    from the other eye cannot acquire it: a new valid callback must arrive.
    ``generation`` changes on release/acquisition so callers reset stability.
    """

    def __init__(self, node, class_id, *, priority_seconds=3.0,
                 detection_timeout=0.8, label='下视伺服'):
        self.node = node
        self.class_id = class_id
        self.priority_seconds = max(0.1, float(priority_seconds))
        self.detection_timeout = max(0.1, float(detection_timeout))
        self.label = label
        self.active_camera = None
        self.active_detection = None
        self.received_at = float('-inf')
        self.deadline = float('-inf')
        self.generation = 0
        self.first_observation = None
        with node._perception_lock:
            self._cursor = node._down_detection_sequence

    def _release(self, reason):
        if self.active_camera is not None:
            self.node.get_logger().info(
                f'{self.label}：释放 {self.active_camera} 伺服优先权：{reason}；'
                '等待下一条有效目标观测')
        self.active_camera = None
        self.active_detection = None
        self.received_at = float('-inf')
        self.deadline = float('-inf')
        self.generation += 1

    def update(self):
        """Return (camera name, detection), consuming callbacks in arrival order."""
        with self.node._perception_lock:
            events = tuple(event for event in self.node._down_detection_events
                           if event[0] > self._cursor)
        now = time.monotonic()
        for sequence, received_at, camera_name, message in events:
            self._cursor = sequence
            if self.active_camera is not None:
                if received_at >= self.deadline:
                    self._release('优先权到期')
                elif received_at - self.received_at > self.detection_timeout:
                    self._release('观测超时')
            detection = best_detection(message, self.class_id)
            if now - received_at > self.detection_timeout:
                detection = None
            if detection is not None and self.first_observation is None:
                self.first_observation = (camera_name, detection)
            if camera_name == self.active_camera:
                if detection is None:
                    self._release('该相机本次没有有效目标观测')
                else:
                    self.active_detection = detection
                    self.received_at = received_at
            elif self.active_camera is None and detection is not None:
                self.active_camera = camera_name
                self.active_detection = detection
                self.received_at = received_at
                self.deadline = received_at + self.priority_seconds
                self.generation += 1
                self.node.get_logger().info(
                    f'{self.label}：{camera_name} 首先观测到目标，'
                    f'获得 {self.priority_seconds:.1f}s 单目伺服优先权')
        if self.active_camera is not None:
            if now >= self.deadline:
                self._release('优先权到期')
            elif now - self.received_at > self.detection_timeout:
                self._release('观测超时')
        return self.active_camera, self.active_detection


def normalized_image_error(node, camera_name, detection):
    """Remove K/skew/distortion from a single eye's detection centre."""
    side = 'left' if camera_name == 'down_left' else 'right'
    calibration = node.camera_configs['down'].side(side)
    pixel = np.array([float(detection.pixel_x), float(detection.pixel_y), 1.0],
                     dtype=np.float64)
    normalized = np.linalg.solve(calibration.matrix, pixel)
    point = (normalized[:2] / normalized[2]).reshape(1, 1, 2)
    corrected = cv2.undistortPoints(
        point, np.eye(3), calibration.distortion).reshape(2)
    if not np.all(np.isfinite(corrected)):
        raise ValueError('单目目标中心误差包含无效数值')
    return float(corrected[0]), float(corrected[1])


def body_image_step(camera, du, dv, scale_m, gain, max_step_m):
    """Map optical XY error through mounting TF; no target depth is estimated."""
    step = camera.optical_to_body @ np.array([du, dv, 0.0])
    step *= float(scale_m) * float(gain)
    norm = math.hypot(float(step[0]), float(step[1]))
    if norm > max_step_m:
        step *= float(max_step_m) / norm
    return float(step[0]), float(step[1])
