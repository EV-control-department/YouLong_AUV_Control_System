"""Convert DetectionArray metadata into geometry-only measurements.

The node never subscribes to an image topic.  It waits briefly for both eyes
of a stereo pair, triangulates matched classes when calibration is available,
and always retains a bearing measurement for unmatched detections.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import threading
import time

from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy,
)
from auv_protocol.topics import (
    DOWN_LEFT_INFO, DOWN_RIGHT_INFO, FRONT_LEFT_INFO, FRONT_RIGHT_INFO,
    MEASUREMENTS, PERCEPTION_DETECTIONS,
)
from sensor_msgs.msg import CameraInfo
from uv_msgs.msg import DetectionArray, ObjectMeasurement, ObjectMeasurementArray

from .localization.covariance import diagonal
from .localization.stereo import camera_ray, triangulate


@dataclass
class PendingPair:
    first_seen_ns: int
    messages: dict[str, DetectionArray] = field(default_factory=dict)


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
        try:
            from tf2_ros import Buffer, TransformListener
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, node)
        except Exception:
            self._tf_buffer = None
            self._tf_listener = None
        self._lock = threading.Lock()
        self._pending: dict[tuple[str, int, int], PendingPair] = {}
        self._infos = {}
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
        self._observation_id = 0

    def _info(self, name, message):
        with self._lock:
            self._infos[name] = message

    def _detections(self, message):
        camera_name = str(message.camera_name).strip().lower()
        group = camera_name.split('_', 1)[0]
        key = (group, int(message.capture_id), int(message.stereo_pair_id))
        with self._lock:
            pending = self._pending.setdefault(key, PendingPair(time.monotonic_ns()))
            pending.messages[camera_name] = message

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
    def _match(left, right):
        candidates = []
        for left_detection in left.detections:
            for right_detection in right.detections:
                if int(left_detection.class_id) == int(right_detection.class_id):
                    candidates.append((left_detection, right_detection))
        return candidates

    def _publish_pending(self, pending, infos):
        messages = pending.messages
        output = ObjectMeasurementArray()
        first = next(iter(messages.values()))
        output.header = first.header
        group = str(first.camera_name).split('_', 1)[0]
        left = messages.get(f'{group}_left')
        right = messages.get(f'{group}_right')
        if left is not None and right is not None and group == 'front':
            left_info = infos.get('front_left')
            right_info = infos.get('front_right')
            matched = self._match(left, right)
            used = set()
            for left_detection, right_detection in matched:
                used.add(id(left_detection))
                used.add(id(right_detection))
                measurement = self._stereo_measurement(
                    left, left_detection, right_detection, left_info, right_info)
                output.measurements.append(measurement)
            for message in (left, right):
                info = infos.get(message.camera_name)
                for detection in message.detections:
                    if id(detection) not in used:
                        output.measurements.append(
                            self._bearing_measurement(message, detection, info))
        else:
            for message in messages.values():
                info = infos.get(message.camera_name)
                for detection in message.detections:
                    output.measurements.append(
                        self._bearing_measurement(message, detection, info))
        if output.measurements:
            self.publisher.publish(output)

    def _base(self, message, detection, form):
        measurement = ObjectMeasurement()
        self._observation_id += 1
        measurement.observation_id = self._observation_id
        measurement.observation_stamp = message.header.stamp
        measurement.capture_id = int(message.capture_id)
        measurement.stereo_pair_id = int(message.stereo_pair_id)
        measurement.source_detection_ids = [self._observation_id]
        measurement.source_camera = str(message.camera_name)
        measurement.class_id = int(detection.class_id)
        try:
            from uv_perception.model_classes import model_class_name, physical_class_name
            measurement.class_name = model_class_name(measurement.class_id)
            measurement.physical_class_name = physical_class_name(measurement.class_id)
        except Exception:
            measurement.class_name = f'class_{measurement.class_id}'
            measurement.physical_class_name = measurement.class_name
        measurement.confidence = float(detection.confidence)
        measurement.measurement_form = form
        measurement.position_covariance = diagonal(1.0)
        return measurement

    def _bearing_measurement(self, message, detection, info):
        measurement = self._base(message, detection, ObjectMeasurement.FORM_FRONT_BEARING)
        if str(message.camera_name).startswith('down_'):
            measurement.measurement_form = ObjectMeasurement.FORM_DOWN_DIRECT
        measurement.has_ray = True
        if info is not None:
            ray = camera_ray(info, detection.pixel_x, detection.pixel_y)
        else:
            ray = (0.0, 0.0, 1.0)
        (measurement.ray_origin_x, measurement.ray_origin_y,
         measurement.ray_origin_z) = (0.0, 0.0, 0.0)
        (measurement.ray_direction_x, measurement.ray_direction_y,
         measurement.ray_direction_z) = ray
        if str(message.camera_name).startswith('down_'):
            world_ray = self._to_world_ray(
                (0.0, 0.0, 0.0), ray, message.header.frame_id,
                message.header.stamp)
            if world_ray is not None and abs(world_ray[1][2]) > 1e-6:
                scale = (self.down_ground_z - world_ray[0][2]) / world_ray[1][2]
                if scale > 0.0:
                    point = tuple(world_ray[0][index] + scale * world_ray[1][index]
                                  for index in range(3))
                    measurement.has_position = True
                    measurement.world_x, measurement.world_y, measurement.world_z = point
                    measurement.position_covariance = diagonal(max(0.01, scale * 0.03))
        return measurement

    def _stereo_measurement(self, message, left_detection, right_detection,
                            left_info, right_info):
        measurement = self._base(message, left_detection,
                                 ObjectMeasurement.FORM_FRONT_STEREO)
        if left_info is None or right_info is None:
            return self._bearing_measurement(message, left_detection, left_info)
        position = triangulate(left_info, right_info,
                               left_detection.pixel_x, right_detection.pixel_x,
                               left_detection.pixel_y, self.baseline_m)
        if position is None:
            return self._bearing_measurement(message, left_detection, left_info)
        position = self._to_world(position, message.header.frame_id,
                                  message.header.stamp)
        if position is None:
            # A camera-frame point is not a valid estimator state. Keep the
            # bearing, and let the next frame retry once TF is available.
            return self._bearing_measurement(message, left_detection, left_info)
        measurement.has_position = True
        measurement.world_x, measurement.world_y, measurement.world_z = position
        measurement.position_covariance = diagonal(max(0.01, position[2] * 0.02))
        return measurement

    def _to_world(self, position, source_frame, stamp):
        ray = self._to_world_ray(position, (0.0, 0.0, 0.0), source_frame, stamp)
        return None if ray is None else ray[0]

    def _to_world_ray(self, origin, direction, source_frame, stamp):
        if self._tf_buffer is None or not source_frame:
            return None
        try:
            from rclpy.duration import Duration
            from rclpy.time import Time
            transform = self._tf_buffer.lookup_transform(
                self.world_frame, source_frame,
                Time(nanoseconds=int(stamp.sec) * 1_000_000_000 +
                     int(stamp.nanosec)),
                timeout=Duration(seconds=0.01))
            rotation = transform.transform.rotation
            translation = transform.transform.translation
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
            rotated_origin = rotate((x, y, z))
            world_origin = (rotated_origin[0] + float(translation.x),
                            rotated_origin[1] + float(translation.y),
                            rotated_origin[2] + float(translation.z))
            return world_origin, rotate(direction)
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

# Keep the established module/console entrypoint while using the static,
# odom-frame implementation.
from .object_localizer_static import ObjectLocalizer, main
