"""YOLO detector: iceoryx2 stitched frames to one aggregate ROS topic."""

from __future__ import annotations

from pathlib import Path
import os
import threading
import math
import time

import cv2
import numpy as np

from std_msgs.msg import Int32MultiArray
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
)
from auv_protocol.model_mapping import ModelClassRegistry
from auv_protocol.topics import (
    ARUCO_IDS, ICEORYX_CAMERA_DOWN, ICEORYX_CAMERA_FRONT, LINES,
    MODEL_CLASS_MAPPING, PERCEPTION_DETECTIONS,
)
from uv_msgs.msg import Detection, DetectionArray, LineState, ModelClassMapping

from .detector.yolo_detector import YoloDetector
from uv_image_transport.iceoryx2 import Iceoryx2Reader


def _model_default() -> str:
    override = os.environ.get('UV_YOLO_MODEL', '').strip()
    if override:
        return override

    model_filename = 'robotcup20260901.pt'
    source_candidate = (Path(__file__).resolve().parents[1] / 'weights' /
                        model_filename)
    if source_candidate.is_file():
        return str(source_candidate)

    try:
        from ament_index_python.packages import get_package_share_directory
        candidate = (Path(get_package_share_directory('uv_perception')) /
                     'weights' / model_filename)
        if candidate.is_file():
            return str(candidate)
    except Exception:
        pass

    workspace_candidate = (Path.cwd() / 'workspace_auv' / 'src' /
                           'uv_perception' / 'weights' / model_filename)
    if workspace_candidate.is_file():
        return str(workspace_candidate)
    return ''


def _camera_optical_frame(camera, side):
    frame_prefix = 'downward' if camera == 'down' else camera
    return f'{frame_prefix}_{side}_camera_optical_frame'


def _stamp(node, timestamp_ns):
    from rclpy.time import Time
    return Time(nanoseconds=int(timestamp_ns)).to_msg()


class ObjectDetector:
    def __init__(self, node):
        self.node = node
        self.publisher = node.create_publisher(DetectionArray, PERCEPTION_DETECTIONS, 10)
        self._line_publishers = {
            name: node.create_publisher(LineState, LINES(name), 10)
            for name in ('front_left', 'front_right', 'down_left', 'down_right')
        }
        self._aruco_publisher = node.create_publisher(Int32MultiArray, ARUCO_IDS, 10)
        aruco_fps = max(0.1, float(node.declare_parameter('aruco_fps', 10.0).value))
        self._aruco_period_s = 1.0 / aruco_fps
        self._guide_line_class_id = None
        self._gate_front_class_id = None
        self._mapping_registry = None
        self._mapping_ready = threading.Event()
        mapping_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(
            ModelClassMapping, MODEL_CLASS_MAPPING,
            self._mapping_callback, mapping_qos)
        self._gate_feature_mode = str(
            node.declare_parameter('gate_feature_mode', 'auto').value).strip().lower()
        if self._gate_feature_mode not in {'auto', 'bbox', 'centerline', 'segmentation'}:
            self._gate_feature_mode = 'auto'
        self.confidence = float(node.declare_parameter('confidence', 0.5).value)
        self.device = str(node.declare_parameter('device', '').value or '')
        configured_model = str(
            node.declare_parameter('model_path', _model_default()).value or '').strip()
        model_path = configured_model or _model_default()
        self._detector = None
        if model_path and Path(model_path).is_file():
            try:
                self._detector = YoloDetector(model_path, self.confidence, self.device)
                node.get_logger().info(f'object_detector loaded YOLO model {model_path}')
            except Exception as error:
                node.get_logger().error(f'cannot load YOLO model: {error}')
        else:
            node.get_logger().warning(
                'object_detector has no model; it will publish empty detection arrays')
        self._last_aruco_s = 0.0
        self._aruco_detector = self._make_aruco_detector()
        self._stop = threading.Event()
        self._threads = []
        for camera, service in (('front', ICEORYX_CAMERA_FRONT),
                                ('down', ICEORYX_CAMERA_DOWN)):
            thread = threading.Thread(target=self._read_loop,
                                      args=(camera, service),
                                      name=f'detector-{camera}', daemon=True)
            self._threads.append(thread)
            thread.start()

    def _make_aruco_detector(self):
        try:
            aruco = cv2.aruco
            dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)
            if hasattr(aruco, 'DetectorParameters'):
                parameters = aruco.DetectorParameters()
            else:
                parameters = aruco.DetectorParameters_create()
            detector = (
                aruco.ArucoDetector(dictionary, parameters)
                if hasattr(aruco, 'ArucoDetector') else None)
            return aruco, dictionary, parameters, detector
        except (AttributeError, cv2.error) as error:
            self.node.get_logger().warning(
                f'ArUco unavailable in OpenCV; marker output disabled: {error}')
            return None

    def _publish_aruco(self, image):
        if self._aruco_detector is None:
            return
        now = time.monotonic()
        if now - self._last_aruco_s < self._aruco_period_s:
            return
        self._last_aruco_s = now
        aruco, dictionary, parameters, detector = self._aruco_detector
        detected_ids = set()
        half_width = image.shape[1] // 2
        try:
            for offset in (0, half_width):
                eye = image[:, offset:offset + half_width]
                gray = cv2.cvtColor(eye, cv2.COLOR_BGR2GRAY)
                if detector is not None:
                    _, ids, _ = detector.detectMarkers(gray)
                else:
                    _, ids, _ = aruco.detectMarkers(
                        gray, dictionary, parameters=parameters)
                if ids is not None:
                    detected_ids.update(int(value) for value in ids.reshape(-1))
            message = Int32MultiArray()
            message.data = sorted(detected_ids)
            self._aruco_publisher.publish(message)
        except Exception as error:
            self.node.get_logger().warning(f'ArUco frame processing failed: {error}')

    def _mapping_callback(self, message):
        try:
            registry = ModelClassRegistry.from_message(message)
            self._mapping_registry = registry
            self._guide_line_class_id = registry.model_class_id(
                'guide_line', required=False)
            self._gate_front_class_id = registry.model_class_id(
                'gate_front', required=False)
            self._mapping_ready.set()
            self.node.get_logger().info(
                'object_detector received model mapping {}'.format(registry.model))
        except Exception as error:
            self.node.get_logger().error(
                'invalid model class mapping: {}'.format(error))

    def _read_loop(self, camera, service):
        reader = None
        try:
            reader = Iceoryx2Reader(service)
            while not self._stop.is_set():
                if not self._mapping_ready.wait(timeout=0.1):
                    continue
                packet = reader.read()
                if packet is None:
                    return
                image = packet.bgr()
                if camera == 'front':
                    self._publish_aruco(image)
                half_width = image.shape[1] // 2
                for side, offset in (('left', 0), ('right', half_width)):
                    eye = image[:, offset:offset + half_width]
                    if self._detector:
                        results, polygons = self._detector.detect_with_masks(eye)
                    else:
                        results, polygons = (), ()
                    self._publish(camera, side, packet, eye, results, polygons)
        except Exception as error:
            self.node.get_logger().error(f'{camera} detector loop stopped: {error}')
        finally:
            if reader is not None:
                reader.close()

    def _publish(self, camera, side, packet, image, results, polygons):
        message = DetectionArray()
        message.header.stamp = _stamp(self.node, packet.header.timestamp_ns)
        message.header.frame_id = _camera_optical_frame(camera, side)
        message.camera_name = f'{camera}_{side}'
        message.capture_id = packet.header.capture_id
        message.stereo_pair_id = packet.header.stereo_pair_id
        for index, (class_id, confidence, box) in enumerate(results):
            detection = Detection()
            detection.class_id = int(class_id)
            detection.confidence = float(confidence)
            detection.bbox_x1 = float(box[0])
            detection.bbox_y1 = float(box[1])
            detection.bbox_x2 = float(box[2])
            detection.bbox_y2 = float(box[3])
            detection.pixel_x = (detection.bbox_x1 + detection.bbox_x2) * 0.5
            detection.pixel_y = (detection.bbox_y1 + detection.bbox_y2) * 0.5
            detection.feature_type = Detection.FEATURE_BBOX_CENTER
            detection.feature_pixel_x = detection.pixel_x
            detection.feature_pixel_y = detection.pixel_y
            polygon = polygons[index] if index < len(polygons) else None
            self._set_gate_feature(detection, polygon, image)
            message.detections.append(detection)
        self.publisher.publish(message)
        line = self._line_state(message, image, results, polygons)
        self._line_publishers[message.camera_name].publish(line)

    @staticmethod
    def _segmentation_center(polygon):
        if polygon is None:
            return None
        points = np.asarray(polygon, dtype=np.float32).reshape(-1, 2)
        if len(points) < 4 or not np.all(np.isfinite(points)):
            return None
        center, size, _ = cv2.minAreaRect(points)
        if (not np.all(np.isfinite(center))
                or min(float(size[0]), float(size[1])) < 2.0):
            return None
        return float(center[0]), float(center[1])

    @staticmethod
    def _centerline_from_red_pipes(image, bbox):
        x1, y1, x2, y2 = (float(value) for value in bbox)
        height, width = image.shape[:2]
        ix1 = max(0, min(width - 1, int(np.floor(x1))))
        iy1 = max(0, min(height - 1, int(np.floor(y1))))
        ix2 = max(ix1 + 1, min(width, int(np.ceil(x2))))
        iy2 = max(iy1 + 1, min(height, int(np.ceil(y2))))
        roi = image[iy1:iy2, ix1:ix2]
        if roi.size == 0:
            return None
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower = cv2.inRange(
            hsv, np.array([0, 45, 30], dtype=np.uint8),
            np.array([18, 255, 255], dtype=np.uint8))
        upper = cv2.inRange(
            hsv, np.array([165, 45, 30], dtype=np.uint8),
            np.array([180, 255, 255], dtype=np.uint8))
        mask = cv2.bitwise_or(lower, upper)
        mask = cv2.morphologyEx(
            mask, cv2.MORPH_OPEN, np.ones((3, 3), dtype=np.uint8))
        ys, xs = np.where(mask > 0)
        if len(xs) < 24:
            return None
        x_span = float(np.percentile(xs, 95) - np.percentile(xs, 5))
        y_span = float(np.percentile(ys, 95) - np.percentile(ys, 5))
        roi_width = max(float(ix2 - ix1), 1.0)
        roi_height = max(float(iy2 - iy1), 1.0)
        if (x_span < max(8.0, 0.25 * roi_width)
                or y_span < max(8.0, 0.25 * roi_height)):
            return None
        points = np.column_stack((xs + ix1, ys + iy1)).astype(np.float32)
        center, size, _ = cv2.minAreaRect(points)
        if (not np.all(np.isfinite(center))
                or min(float(size[0]), float(size[1])) < 2.0):
            return None
        return float(center[0]), float(center[1])

    def _set_gate_feature(self, detection, polygon, image):
        if (int(detection.class_id) != self._gate_front_class_id
                or self._gate_feature_mode == 'bbox'):
            return
        feature = None
        feature_type = Detection.FEATURE_BBOX_CENTER
        if self._gate_feature_mode in {'auto', 'segmentation'}:
            feature = self._segmentation_center(polygon)
            if feature is not None:
                feature_type = Detection.FEATURE_GATE_SEGMENTATION
        if feature is None and self._gate_feature_mode in {'auto', 'centerline'}:
            feature = self._centerline_from_red_pipes(
                image, (detection.bbox_x1, detection.bbox_y1,
                        detection.bbox_x2, detection.bbox_y2))
            if feature is not None:
                feature_type = Detection.FEATURE_GATE_CENTERLINE
        if feature is not None and np.all(np.isfinite(feature)):
            detection.feature_type = int(feature_type)
            detection.feature_pixel_x = float(feature[0])
            detection.feature_pixel_y = float(feature[1])

    def _line_state(self, message, image, detections, polygons):
        line = LineState()
        line.stamp = message.header.stamp
        line.camera_name = message.camera_name
        line.detected = False
        candidates = [
            (index, result) for index, result in enumerate(detections)
            if int(result[0]) == self._guide_line_class_id
        ]
        if not candidates:
            return line

        index, (_, _, box) = max(candidates, key=lambda item: item[1][1])
        height, width = image.shape[:2]
        x1, y1, x2, y2 = (float(value) for value in box)
        center_x = (x1 + x2) * 0.5
        area = max(0.0, x2 - x1) * max(0.0, y2 - y1)
        heading = 0.0 if (y2 - y1) >= (x2 - x1) else -90.0

        polygon = polygons[index] if index < len(polygons) else None
        mask_used = False
        if polygon is not None:
            try:
                points = np.asarray(polygon, dtype=np.float32).reshape(-1, 2)
                if len(points) >= 3 and np.all(np.isfinite(points)):
                    contour = points.reshape(-1, 1, 2)
                    moments = cv2.moments(contour)
                    if abs(float(moments['m00'])) > 1e-6:
                        center_x = float(moments['m10'] / moments['m00'])
                    area = abs(float(cv2.contourArea(contour)))
                    _, size, angle = cv2.minAreaRect(contour)
                    rw, rh = float(size[0]), float(size[1])
                    long_axis_deg = float(angle) + (90.0 if rh > rw else 0.0)
                    heading = (long_axis_deg - 90.0 + 90.0) % 180.0 - 90.0
                    mask_used = True
            except (TypeError, ValueError, cv2.error):
                pass

        if not mask_used:
            ix1 = max(0, min(width - 1, int(math.floor(x1))))
            iy1 = max(0, min(height - 1, int(math.floor(y1))))
            ix2 = max(ix1 + 1, min(width, int(math.ceil(x2))))
            iy2 = max(iy1 + 1, min(height, int(math.ceil(y2))))
            roi = image[iy1:iy2, ix1:ix2]
            if roi.size:
                gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
                edges = cv2.Canny(gray, 50, 150)
                min_length = max(8, int(0.25 * min(roi.shape[:2])))
                segments = cv2.HoughLinesP(
                    edges, 1, np.pi / 180.0, threshold=16,
                    minLineLength=min_length, maxLineGap=8)
                if segments is not None:
                    x_a, y_a, x_b, y_b = max(
                        segments.reshape(-1, 4),
                        key=lambda segment: math.hypot(
                            float(segment[2] - segment[0]),
                            float(segment[3] - segment[1])))
                    dx, dy = float(x_b - x_a), float(y_b - y_a)
                    angle = math.degrees(math.atan2(dy, dx))
                    heading = (angle % 180.0) - 90.0
                    center_x = ix1 + (float(x_a) + float(x_b)) * 0.5

        if width <= 0 or height <= 0:
            return line
        line.detected = True
        line.center_error = float(np.clip((center_x - width * 0.5) / (width * 0.5),
                                          -1.0, 1.0))
        line.heading_error_deg = float(heading)
        line.area_ratio = float(np.clip(area / float(width * height), 0.0, 1.0))
        return line

    def close(self):
        self._stop.set()
        for thread in self._threads:
            thread.join(timeout=1.0)


def main(args=None):
    import rclpy
    from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
    rclpy.init(args=args)
    node = rclpy.create_node('object_detector')
    detector = ObjectDetector(node)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        detector.close()
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
