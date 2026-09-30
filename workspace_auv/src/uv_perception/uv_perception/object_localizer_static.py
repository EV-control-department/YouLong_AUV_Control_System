"""Publish odom-frame mono rays and matched instantaneous stereo positions."""

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
from .localization.stereo import camera_ray


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
        self.pair_timeout_s = float(node.declare_parameter('pair_timeout_s', 0.05).value)
        self.world_frame = str(node.declare_parameter('world_frame', 'odom').value)
        self.edge_margin_px = float(node.declare_parameter('edge_margin_px', 8.0).value)
        self.edge_margin_ratio = float(
            node.declare_parameter('edge_margin_ratio', 0.02).value)
        self.stereo_max_ray_gap_m = max(
            0.01, float(node.declare_parameter(
                'stereo_max_ray_gap_m', 0.35).value))
        self.stereo_min_parallax_deg = max(
            0.0, float(node.declare_parameter(
                'stereo_min_parallax_deg', 0.2).value))
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
        rays_by_camera = {}
        for camera_name, message in messages.items():
            info = infos.get(camera_name)
            ids = pending.detection_ids.get(camera_name, [])
            camera_rays = []
            for index in self._eligible(message, info):
                detection_id = ids[index] if index < len(ids) else index + 1
                ray = self._bearing_measurement(message, index, info, detection_id)
                if ray is not None:
                    output.measurements.append(ray)
                    camera_rays.append((index, ray))
            rays_by_camera[camera_name] = camera_rays
        # Keep the two mono rays as estimator inputs, and add a separate
        # instantaneous stereo position for RViz and the measurement UI.
        for group in ('front', 'down'):
            left_name, right_name = f'{group}_left', f'{group}_right'
            if left_name not in messages or right_name not in messages:
                continue
            output.measurements.extend(self._stereo_measurements(
                group, messages[left_name], messages[right_name],
                rays_by_camera.get(left_name, []),
                rays_by_camera.get(right_name, []),
                pending.detection_ids.get(left_name, []),
                pending.detection_ids.get(right_name, [])))
        if output.measurements:
            self.publisher.publish(output)

    def _stereo_measurements(self, group, left_message, right_message,
                             left_rays, right_rays, left_ids, right_ids):
        """Match same-class eye detections and triangulate their world rays."""
        if not left_rays or not right_rays:
            return []
        costs = []
        positions = {}
        for left_row, (left_index, left_ray) in enumerate(left_rays):
            row = []
            left_detection = left_message.detections[left_index]
            for right_column, (right_index, right_ray) in enumerate(right_rays):
                right_detection = right_message.detections[right_index]
                if int(left_detection.class_id) != int(right_detection.class_id):
                    row.append(None)
                    continue
                geometry = self._intersect_stereo_rays(left_ray, right_ray)
                if geometry is None:
                    row.append(None)
                    continue
                position, covariance, gap = geometry
                max_gap = getattr(self, 'stereo_max_ray_gap_m', 0.35)
                row.append(gap / max_gap)
                positions[(left_row, right_column)] = (position, covariance)
            costs.append(row)

        output = []
        for left_row, right_column, _cost in minimum_cost_assignment(
                costs, unmatched_cost=1.0):
            left_index, _left_ray = left_rays[left_row]
            right_index, _right_ray = right_rays[right_column]
            position, covariance = positions[(left_row, right_column)]
            left_detection = left_message.detections[left_index]
            right_detection = right_message.detections[right_index]
            left_id = (left_ids[left_index] if left_index < len(left_ids)
                       else left_index + 1)
            right_id = (right_ids[right_index] if right_index < len(right_ids)
                        else right_index + 1)
            form = (ObjectMeasurement.FORM_FRONT_STEREO if group == 'front'
                    else ObjectMeasurement.FORM_DOWN_STEREO)
            measurement = self._base(
                left_message, left_index, form, [left_id, right_id])
            measurement.source_camera = f'{group}_stereo'
            measurement.confidence = min(
                float(left_detection.confidence),
                float(right_detection.confidence))
            measurement.has_position = True
            measurement.world_x, measurement.world_y, measurement.world_z = (
                float(value) for value in position)
            measurement.position_covariance = [
                float(value) for value in covariance.reshape(-1)]
            output.append(measurement)
        return output

    def _intersect_stereo_rays(self, left, right):
        """Return midpoint, angular-error covariance, and ray gap."""
        origin_left = np.asarray((left.ray_origin_x, left.ray_origin_y,
                                  left.ray_origin_z), dtype=float)
        origin_right = np.asarray((right.ray_origin_x, right.ray_origin_y,
                                   right.ray_origin_z), dtype=float)
        direction_left = np.asarray((left.ray_direction_x, left.ray_direction_y,
                                     left.ray_direction_z), dtype=float)
        direction_right = np.asarray((right.ray_direction_x, right.ray_direction_y,
                                      right.ray_direction_z), dtype=float)
        dot = float(np.clip(np.dot(direction_left, direction_right), -1.0, 1.0))
        angle = math.degrees(math.acos(dot))
        if angle < getattr(self, 'stereo_min_parallax_deg', 0.2):
            return None
        denominator = 1.0 - dot * dot
        if denominator <= 1e-12:
            return None
        offset = origin_left - origin_right
        left_offset = float(np.dot(direction_left, offset))
        right_offset = float(np.dot(direction_right, offset))
        distance_left = (dot * right_offset - left_offset) / denominator
        distance_right = (right_offset - dot * left_offset) / denominator
        if (not math.isfinite(distance_left) or not math.isfinite(distance_right)
                or distance_left <= 0.0 or distance_right <= 0.0):
            return None
        closest_left = origin_left + distance_left * direction_left
        closest_right = origin_right + distance_right * direction_right
        gap = float(np.linalg.norm(closest_left - closest_right))
        max_gap = getattr(self, 'stereo_max_ray_gap_m', 0.35)
        if not math.isfinite(gap) or gap > max_gap:
            return None
        position = 0.5 * (closest_left + closest_right)

        # Angular pixel error becomes transverse position error proportional
        # to range. Combining both ray information matrices preserves the
        # larger uncertainty along the weakly observed depth direction.
        identity = np.eye(3)
        sigma_left = max(float(left.ray_sigma_rad), 1e-6)
        sigma_right = max(float(right.ray_sigma_rad), 1e-6)
        projector_left = identity - np.outer(direction_left, direction_left)
        projector_right = identity - np.outer(direction_right, direction_right)
        information = (
            projector_left / (distance_left * sigma_left) ** 2
            + projector_right / (distance_right * sigma_right) ** 2)
        try:
            covariance = np.linalg.inv(information)
        except np.linalg.LinAlgError:
            covariance = np.linalg.pinv(information)
        covariance = 0.5 * (covariance + covariance.T)
        if not np.all(np.isfinite(position)) or not np.all(np.isfinite(covariance)):
            return None
        return position, covariance, gap

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

    @staticmethod
    def _feature_pixel(detection):
        """Use the detector feature when present; otherwise use bbox center."""
        feature_x = float(detection.feature_pixel_x)
        feature_y = float(detection.feature_pixel_y)
        if (int(detection.feature_type) != 0 and
                math.isfinite(feature_x) and math.isfinite(feature_y)):
            return feature_x, feature_y
        bounds = (float(detection.bbox_x1), float(detection.bbox_y1),
                  float(detection.bbox_x2), float(detection.bbox_y2))
        if all(math.isfinite(value) for value in bounds):
            x1, y1, x2, y2 = bounds
            if x2 > x1 and y2 > y1:
                return 0.5 * (x1 + x2), 0.5 * (y1 + y2)
        return float(detection.pixel_x), float(detection.pixel_y)

    def _bearing_measurement(self, message, index, info, detection_id):
        detection = message.detections[index]
        is_down = str(message.camera_name).startswith('down_')
        form = (ObjectMeasurement.FORM_DOWN_BEARING if is_down else
                ObjectMeasurement.FORM_FRONT_BEARING)
        measurement = self._base(message, index, form, [detection_id])
        pixel_x, pixel_y = self._feature_pixel(detection)
        local_ray = camera_ray(info, pixel_x, pixel_y)
        world_ray = self._to_world_ray(
            (0.0, 0.0, 0.0), local_ray, message.header.frame_id,
            message.header.stamp)
        if world_ray is None:
            self._warn_tf(message.header.frame_id)
            return None
        origin, direction = world_ray
        (measurement.ray_origin_x, measurement.ray_origin_y,
         measurement.ray_origin_z) = origin
        (measurement.ray_direction_x, measurement.ray_direction_y,
         measurement.ray_direction_z) = direction
        fx, fy = float(info.k[0]), float(info.k[4])
        measurement.ray_sigma_rad = math.sqrt(
            0.5 * ((1.0 / fx) ** 2 + (1.0 / fy) ** 2))
        measurement.has_ray = True
        # Position is intentionally absent for both views. The estimator needs
        # motion parallax across raw rays before it can publish a 3D landmark.
        measurement.has_position = False
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

