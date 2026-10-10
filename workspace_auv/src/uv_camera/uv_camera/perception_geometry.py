"""Bind array-level calibration to task detections without changing their type."""
from __future__ import annotations
from collections import OrderedDict
import threading
from .image_geometry import CalibrationCache, normalized_pixel


class TaskCameraGeometry:
    def __init__(self, node):
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
        from auv_protocol.topics import PERCEPTION_CAMERA_CALIBRATION
        from uv_msgs.msg import PerceptionCameraInfo
        self.cache = CalibrationCache()
        self._lock = threading.RLock()
        self._detections = OrderedDict()
        qos = QoSProfile(depth=16, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subscriptions = [node.create_subscription(
            PerceptionCameraInfo, PERCEPTION_CAMERA_CALIBRATION(name), self._info, qos)
            for name in ('front_left', 'front_right', 'down_left', 'down_right')]
        self.node = node

    def _info(self, message):
        try:
            self.cache.add(message)
        except ValueError as error:
            self.node.get_logger().error(f'invalid perception calibration: {error}')

    def bind(self, message):
        name = str(message.camera_name)
        space = int(getattr(message, 'image_space', 0))
        info = self.cache.resolve(message)
        if space != 0 and info is None:
            return False
        with self._lock:
            for detection in message.detections:
                self._detections[(name, id(detection))] = (detection, info)
            while len(self._detections) > 8192:
                self._detections.popitem(last=False)
        return True

    def normalized(self, node, name, detection, reference=None):
        reference = detection if reference is None else reference
        with self._lock:
            bound = self._detections.get((name, id(reference)))
        if bound is not None and bound[0] is reference and bound[1] is not None:
            info = bound[1]
            return normalized_pixel(info.k, info.d, detection.pixel_x, detection.pixel_y)
        if bound is None or bound[0] is not reference:
            raise ValueError('detection has no matched image calibration')
        camera, side = name.split('_', 1)
        raw = node.camera_configs[camera].side(side)
        return normalized_pixel(raw.matrix, raw.distortion, detection.pixel_x, detection.pixel_y)


def bind_detection_geometry(node, message):
    geometry = getattr(node, '_detection_geometry', None)
    if geometry is None:
        # Compatibility for callers/tests constructed without a ROS node.
        return int(getattr(message, 'image_space', 0)) == 0
    return geometry.bind(message)


def normalized_detection(node, name, detection, reference=None):
    geometry = getattr(node, '_detection_geometry', None)
    if geometry is not None:
        return geometry.normalized(node, name, detection, reference)
    camera, side = name.split('_', 1)
    raw = node.camera_configs[camera].side(side)
    return normalized_pixel(raw.matrix, raw.distortion, detection.pixel_x, detection.pixel_y)
