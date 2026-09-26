"""Convert detections into calibrated, odom-frame static-object factors."""

from __future__ import annotations

import copy
from dataclasses import dataclass, field
import math
import threading
import time
import uuid

import numpy as np
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy,
)
from auv_protocol.topics import (
    DOWN_LEFT_INFO, DOWN_RIGHT_INFO, FRONT_LEFT_INFO, FRONT_RIGHT_INFO,
    MEASUREMENTS, PERCEPTION_DETECTIONS,
)
from sensor_msgs.msg import CameraInfo
from uv_msgs.msg import DetectionArray, ObjectMeasurement, ObjectMeasurementArray

from .localization.geometry import bbox_is_localizable, minimum_cost_assignment
from .localization.stereo import camera_ray, triangulate


@dataclass
class PendingPair:
    first_seen_ns: int
    messages: dict[str, DetectionArray] = field(default_factory=dict)
    detection_ids: dict[str, list[int]] = field(default_factory=dict)


def _stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class ObjectLocalizer:
    def __init__(self, node):
        self.node = node
        self.publisher = node.create_publisher(ObjectMeasurementArray, MEASUREMENTS, 10)
        self.baseline_m = float(node.declare_parameter('stereo_baseline_m', 0.10).value)
        self.pair_timeout_s = float(node.declare_parameter('pair_timeout_s', 0.05).value)
        self.world_frame = str(node.declare_parameter('world_frame', 'odom').value)
        self.down_ground_z = float(node.declare_parameter('down_ground_z', 0.0).value)
        self.edge_margin_px = float(node.declare_parameter('edge_margin_px', 8.0).value)
        self.edge_margin_ratio = float(
            node.declare_parameter('edge_margin_ratio', 0.02).value)
        self.stereo_epipolar_tolerance_px = float(
            node.declare_parameter('stereo_epipolar_tolerance_px', 10.0).value)
        self.max_stereo_range_m = float(
            node.declare_parameter('max_stereo_range_m', 30.0).value)
        self.pose_translation_sigma_m = float(
            node.declare_parameter('pose_translation_sigma_m', 0.03).value)
        self.pose_rotation_sigma_deg = float(
            node.declare_parameter('pose_rotation_sigma_deg', 1.0).value)
        self.extrinsic_translation_sigma_m = float(
            node.declare_parameter('extrinsic_translation_sigma_m', 0.005).value)
        self.extrinsic_rotation_sigma_deg = float(
            node.declare_parameter('extrinsic_rotation_sigma_deg', 0.5).value)
        try:
            from tf2_ros import Buffer, TransformListener
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, node)
        except Exception as error:
            self._tf_buffer = None
            self._tf_listener = None
            node.get_logger().error(f'cannot initialize TF listener: {error}')
        self._lock = threading.Lock()
        self._id_lock = threading.Lock()
        self._pending: dict[tuple[str, int, int], PendingPair] = {}
        self._infos = {}
        self._warned_tf_frames = set()
        # Distinguish IDs produced by separate localizer process lifetimes.
        self._id_prefix = (uuid.uuid4().int >> 96) & 0xffffffff
        self._observation_counter = 0
        self._detection_counter = 0
        info_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for name, topic in (
                ('front_left', FRONT_LEFT_INFO), ('front_right', FRONT_RIGHT_INFO),
                ('down_left', DOWN_LEFT_INFO), ('down_right', DOWN_RIGHT_INFO)):
            node.create_subscription(CameraInfo, topic,
                                     lambda message, key=name: self._info(key, message),
                                     info_qos)
        node.create_subscription(DetectionArray, PERCEPTION_DETECTIONS,
                                 self._detections, 10)
        self.timer = node.create_timer(0.02, self._flush)

    def _info(self, name, message):
        with self._lock:
            self._infos[name] = message

    def _detections(self, message):
        camera_name = str(message.camera_name).strip().lower()
        group = camera_name.split('_', 1)[0]
        key = (group, int(message.capture_id), int(message.stereo_pair_id))
        with self._lock:
            pending = self._pending.setdefault(key, PendingPair(time.monotonic_ns()))
            with self._id_lock:
                ids = []
                for _ in message.detections:
                    self._detection_counter += 1
                    ids.append((self._id_prefix << 32) |
                               (0x80000000 | self._detection_counter))
            pending.messages[camera_name] = message
            pending.detection_ids[camera_name] = ids

    def _flush(self):
        now = time.monotonic_ns()
        ready = []
        with self._lock:
            for key, pending in list(self._pending.items()):
                if (len(pending.messages) >= 2
                        or now - pending.first_seen_ns >= self.pair_timeout_s * 1e9):
                    ready.append(pending)
                    del self._pending[key]
            infos = dict(self._infos)
        for pending in ready:
            self._publish_pending(pending, infos)

    @staticmethod
    def _class_metadata(class_id):
        try:
            from uv_perception.model_classes import (
                CLASS_METADATA, model_class_name, physical_class_name,
            )
            entry = CLASS_METADATA.get(int(class_id), {})
            return (model_class_name(class_id), physical_class_name(class_id),
                    bool(entry.get('multi_instance', False)))
        except Exception:
            fallback = f'class_{int(class_id)}'
            return fallback, fallback, False

    def _eligible(self, message, info):
        if info is None or int(info.width) <= 0 or int(info.height) <= 0:
            return []
        try:
            fx, fy = float(info.k[0]), float(info.k[4])
        except (IndexError, TypeError, ValueError):
            return []
        if not math.isfinite(fx) or not math.isfinite(fy) or fx <= 0.0 or fy <= 0.0:
            return []
        return [index for index, detection in enumerate(message.detections)
                if bbox_is_localizable(
                    detection, info.width, info.height,
                    self.edge_margin_px, self.edge_margin_ratio)]

    def _stereo_pairs(self, left, right, left_info, right_info,
                      left_indices, right_indices):
        if left_info is None or right_info is None:
            return []
        tolerance = min(
            self.stereo_epipolar_tolerance_px,
            max(2.0, 0.01 * min(int(left_info.height), int(right_info.height))))
        costs = []
        for left_index in left_indices:
            left_detection = left.detections[left_index]
            _, left_physical, _ = self._class_metadata(left_detection.class_id)
            row = []
            for right_index in right_indices:
                right_detection = right.detections[right_index]
                _, right_physical, _ = self._class_metadata(right_detection.class_id)
                disparity = float(left_detection.pixel_x) - float(right_detection.pixel_x)
                vertical_error = abs(float(left_detection.pixel_y) -
                                     float(right_detection.pixel_y))
                plausible = (left_physical == right_physical and disparity > 1e-4 and
                             vertical_error <= tolerance)
                if plausible:
                    fx = float(left_info.k[0])
                    depth = (fx * self.baseline_m / disparity
                             if fx > 0.0 and self.baseline_m > 0.0 else float('inf'))
                    plausible = 0.05 <= depth <= self.max_stereo_range_m
                if plausible:
                    confidence = min(float(left_detection.confidence),
                                     float(right_detection.confidence))
                    row.append(0.85 * vertical_error / max(tolerance, 1e-6) +
                               0.15 * (1.0 - max(0.0, min(1.0, confidence))))
                else:
                    row.append(None)
            costs.append(row)
        return [(left_indices[row], right_indices[column])
                for row, column, _ in minimum_cost_assignment(costs, 1.0)]

    def _publish_pending(self, pending, infos):
        messages = pending.messages
        if not messages:
            return
        output = ObjectMeasurementArray()
        first = next(iter(messages.values()))
        # Generated ROS message assignment can retain the nested Header object.
        # Mutating output.header.frame_id must not rewrite the input camera frame.
        output.header = copy.deepcopy(first.header)
        output.header.frame_id = self.world_frame
        group = str(first.camera_name).split('_', 1)[0]
        left_name, right_name = f'{group}_left', f'{group}_right'
        left, right = messages.get(left_name), messages.get(right_name)
        if group == 'front' and left is not None and right is not None:
            left_info, right_info = infos.get(left_name), infos.get(right_name)
            left_indices = self._eligible(left, left_info)
            right_indices = self._eligible(right, right_info)
            pairs = self._stereo_pairs(
                left, right, left_info, right_info, left_indices, right_indices)
            used_left, used_right = set(), set()
            for left_index, right_index in pairs:
                used_left.add(left_index)
                used_right.add(right_index)
                measurement = self._stereo_measurement(
                    left, left_index, right, right_index, left_info, right_info,
                    pending.detection_ids[left_name][left_index],
                    pending.detection_ids[right_name][right_index])
                if measurement is not None:
                    output.measurements.append(measurement)
                else:
                    # Failed triangulation/TF must not discard either valid eye.
                    for message, info, index, camera_key in (
                            (left, left_info, left_index, left_name),
                            (right, right_info, right_index, right_name)):
                        bearing = self._bearing_measurement(
                            message, index, info,
                            pending.detection_ids[camera_key][index])
                        if bearing is not None:
                            output.measurements.append(bearing)
            for message, info, camera_key, used in (
                    (left, left_info, left_name, used_left),
                    (right, right_info, right_name, used_right)):
                for index in self._eligible(message, info):
                    if index in used:
                        continue
                    bearing = self._bearing_measurement(
                        message, index, info,
                        pending.detection_ids[camera_key][index])
                    if bearing is not None:
                        output.measurements.append(bearing)
        else:
            for camera_name, message in messages.items():
                info = infos.get(camera_name)
                for index in self._eligible(message, info):
                    bearing = self._bearing_measurement(
                        message, index, info,
                        pending.detection_ids[camera_name][index])
                    if bearing is not None:
                        output.measurements.append(bearing)
        if output.measurements:
            self.publisher.publish(output)

    def _base(self, message, index, form, detection_ids):
        detection = message.detections[index]
        measurement = ObjectMeasurement()
        with self._id_lock:
            self._observation_counter += 1
            measurement.observation_id = ((self._id_prefix << 32) |
                                          self._observation_counter)
        measurement.observation_stamp = message.header.stamp
        measurement.capture_id = int(message.capture_id)
        measurement.stereo_pair_id = int(message.stereo_pair_id)
        measurement.source_detection_ids = [int(value) for value in detection_ids]
        measurement.source_camera = str(message.camera_name)
        measurement.class_id = int(detection.class_id)
        (measurement.class_name, measurement.physical_class_name,
         measurement.multi_instance) = self._class_metadata(measurement.class_id)
        measurement.confidence = float(detection.confidence)
        measurement.measurement_form = form
        measurement.position_covariance = [0.0] * 9
        return measurement

    def _position_covariance(self, sigma_x, sigma_y, sigma_z, range_m):
        pose_angle = math.radians(self.pose_rotation_sigma_deg)
        extrinsic_angle = math.radians(self.extrinsic_rotation_sigma_deg)
        common = (self.pose_translation_sigma_m ** 2 +
                  self.extrinsic_translation_sigma_m ** 2 +
                  (float(range_m) * math.hypot(pose_angle, extrinsic_angle)) ** 2)
        variances = (sigma_x ** 2 + common, sigma_y ** 2 + common,
                     sigma_z ** 2 + common)
        return [variances[0], 0.0, 0.0, 0.0, variances[1], 0.0,
                0.0, 0.0, variances[2]]

    @staticmethod
    def _rotate_covariance(covariance, rotation_matrix):
        rotation = np.asarray(rotation_matrix, dtype=float).reshape(3, 3)
        covariance = np.asarray(covariance, dtype=float).reshape(3, 3)
        rotated = rotation @ covariance @ rotation.T
        return [float(value) for value in rotated.reshape(9)]

    def _bearing_measurement(self, message, index, info, detection_id):
        detection = message.detections[index]
        is_down = str(message.camera_name).startswith('down_')
        form = (ObjectMeasurement.FORM_DOWN_DIRECT if is_down else
                ObjectMeasurement.FORM_FRONT_BEARING)
        measurement = self._base(message, index, form, [detection_id])
        local_ray = camera_ray(info, detection.pixel_x, detection.pixel_y)
        world_ray = self._to_world_ray(
            (0.0, 0.0, 0.0), local_ray, message.header.frame_id,
            message.header.stamp, return_rotation=True)
        if world_ray is None:
            self._warn_tf(message.header.frame_id)
            return None
        origin, direction, rotation_matrix = world_ray
        (measurement.ray_origin_x, measurement.ray_origin_y,
         measurement.ray_origin_z) = origin
        (measurement.ray_direction_x, measurement.ray_direction_y,
         measurement.ray_direction_z) = direction
        fx, fy = float(info.k[0]), float(info.k[4])
        pixel_sigma = math.sqrt((1.0 / fx) ** 2 + (1.0 / fy) ** 2)
        measurement.ray_sigma_rad = math.hypot(
            pixel_sigma, math.radians(self.extrinsic_rotation_sigma_deg))
        measurement.has_ray = True
        if is_down and abs(direction[2]) > 1e-6:
            scale = (self.down_ground_z - origin[2]) / direction[2]
            if scale > 0.0:
                point = tuple(origin[axis] + scale * direction[axis]
                              for axis in range(3))
                measurement.has_position = True
                measurement.world_x, measurement.world_y, measurement.world_z = point
                lateral_sigma = max(0.005, scale * measurement.ray_sigma_rad)
                measurement.position_covariance = self._position_covariance(
                    lateral_sigma, lateral_sigma, 0.01, scale)
                measurement.position_covariance = self._rotate_covariance(
                    measurement.position_covariance, rotation_matrix)
        return measurement

    def _stereo_measurement(self, left, left_index, right, right_index,
                            left_info, right_info, left_detection_id,
                            right_detection_id):
        left_detection = left.detections[left_index]
        right_detection = right.detections[right_index]
        measurement = self._base(
            left, left_index, ObjectMeasurement.FORM_FRONT_STEREO,
            [left_detection_id, right_detection_id])
        position_camera = triangulate(
            left_info, right_info, left_detection.pixel_x,
            right_detection.pixel_x, left_detection.pixel_y, self.baseline_m)
        if position_camera is None:
            return None
        transformed = self._to_world_ray(
            position_camera, (0.0, 0.0, 0.0),
            left.header.frame_id, left.header.stamp, return_rotation=True)
        world = None if transformed is None else transformed[0]
        rotation_matrix = None if transformed is None else transformed[2]
        if world is None:
            self._warn_tf(left.header.frame_id)
            return None
        fx, fy = float(left_info.k[0]), float(left_info.k[4])
        depth = float(position_camera[2])
        sigma_x = max(0.005, depth / max(fx, 1.0))
        sigma_y = max(0.005, depth / max(fy, 1.0))
        sigma_z = math.sqrt(2.0) * depth * depth / max(fx * self.baseline_m, 1e-6)
        measurement.has_position = True
        measurement.world_x, measurement.world_y, measurement.world_z = world
        camera_covariance = self._position_covariance(
            sigma_x, sigma_y, sigma_z,
            float(math.sqrt(sum(value * value for value in position_camera))))
        measurement.position_covariance = self._rotate_covariance(
            camera_covariance, rotation_matrix)
        return measurement

    def _warn_tf(self, source_frame):
        source_frame = str(source_frame)
        if source_frame in self._warned_tf_frames:
            return
        self._warned_tf_frames.add(source_frame)
        self.node.get_logger().warning(
            f'no time-valid transform {source_frame!r} -> {self.world_frame!r}; '
            'dropping localization geometry until TF is available')

    def _to_world(self, position, source_frame, stamp):
        transformed = self._to_world_ray(
            position, (0.0, 0.0, 0.0), source_frame, stamp)
        return None if transformed is None else transformed[0]

    def _to_world_ray(self, origin, direction, source_frame, stamp,
                      return_rotation=False):
        if _stamp_ns(stamp) <= 0:
            return None
        if (self._tf_buffer is None or not source_frame or
                source_frame == self.world_frame):
            return None
        try:
            from rclpy.duration import Duration
            from rclpy.clock import ClockType
            from rclpy.time import Time
            from geometry_msgs.msg import PointStamped
            from tf2_geometry_msgs import do_transform_point
            transform = self._tf_buffer.lookup_transform(
                self.world_frame, source_frame,
                Time(nanoseconds=_stamp_ns(stamp), clock_type=ClockType.ROS_TIME),
                timeout=Duration(seconds=0.01))
            rotation = transform.transform.rotation
            x, y, z = (float(value) for value in origin)
            qx, qy, qz, qw = (float(rotation.x), float(rotation.y),
                              float(rotation.z), float(rotation.w))

            def rotate(vector):
                vx, vy, vz = (float(value) for value in vector)
                tx = 2.0 * (qy * vz - qz * vy)
                ty = 2.0 * (qz * vx - qx * vz)
                tz = 2.0 * (qx * vy - qy * vx)
                return (vx + qw * tx + (qy * tz - qz * ty),
                        vy + qw * ty + (qz * tx - qx * tz),
                        vz + qw * tz + (qx * ty - qy * tx))

            point = PointStamped()
            point.header.frame_id = source_frame
            point.header.stamp = stamp
            point.point.x, point.point.y, point.point.z = x, y, z
            transformed_point = do_transform_point(point, transform).point
            world_origin = (float(transformed_point.x),
                            float(transformed_point.y),
                            float(transformed_point.z))
            if not all(math.isfinite(value) for value in world_origin):
                return None
            rotation_matrix = np.column_stack((
                rotate((1.0, 0.0, 0.0)), rotate((0.0, 1.0, 0.0)),
                rotate((0.0, 0.0, 1.0))))
            world_direction = tuple(rotation_matrix @ np.asarray(direction, dtype=float))
            norm = math.sqrt(sum(value * value for value in world_direction))
            if not math.isfinite(norm):
                return None
            if norm <= 1e-12:
                if return_rotation:
                    return world_origin, (0.0, 0.0, 0.0), rotation_matrix
                return world_origin, (0.0, 0.0, 0.0)
            world_direction = tuple(value / norm for value in world_direction)
            if return_rotation:
                return world_origin, world_direction, rotation_matrix
            return world_origin, world_direction
        except Exception:
            return None


def main(args=None):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    rclpy.init(args=args)
    node = rclpy.create_node('object_localizer')
    localizer = ObjectLocalizer(node)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

