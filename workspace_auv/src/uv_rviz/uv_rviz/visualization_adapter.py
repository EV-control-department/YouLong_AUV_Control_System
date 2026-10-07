"""Read-only adapters from canonical AUV state to RViz-native messages."""

from collections import deque
import math
from typing import Deque, Dict, Optional, Tuple

from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.duration import Duration as RclpyDuration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

from auv_protocol.topics import (
    MEASUREMENTS,
    STATE_ODOM,
    STATE_RESET_RESULT,
    TRACKS,
    TRAJECTORY,
    VIZ_MEASUREMENTS,
    VIZ_ODOM,
    VIZ_ODOM_PATH,
    VIZ_PLANNED_PATH,
    VIZ_TRACKS,
)
from uv_msgs.msg import (
    StateResetResult,
    ObjectMeasurement,
    ObjectMeasurementArray,
    ObjectTrack,
    ObjectTrackArray,
    PoseInfo,
    WaypointPath,
)


ODOM_FRAME = 'odom'
BASE_FRAME = 'base_link'
PATH_SAMPLE_PERIOD_NS = 500_000_000
PATH_MAX_POINTS = 600
MEASUREMENT_MARKER_LIFETIME_S = 0.6
TRACK_MARKER_LIFETIME_S = 1.0


def stamp_to_nanoseconds(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _is_finite(*values) -> bool:
    return all(math.isfinite(float(value)) for value in values)


def yaw_degrees_to_quaternion(yaw_degrees: float):
    """Return a Z-axis quaternion matching the current odom TF convention."""
    from geometry_msgs.msg import Quaternion

    yaw = math.radians(float(yaw_degrees))
    quaternion = Quaternion()
    quaternion.z = math.sin(yaw * 0.5)
    quaternion.w = math.cos(yaw * 0.5)
    return quaternion


def pose_info_to_odometry(message: PoseInfo) -> Optional[Odometry]:
    """Convert the visual pose subset of PoseInfo to nav_msgs/Odometry."""
    values = (message.robot_x, message.robot_y, message.robot_z, message.robot_yaw)
    if not _is_finite(*values):
        return None

    odometry = Odometry()
    odometry.header.stamp = message.stamp
    odometry.header.frame_id = ODOM_FRAME
    odometry.child_frame_id = BASE_FRAME
    odometry.pose.pose.position.x = float(message.robot_x)
    odometry.pose.pose.position.y = float(message.robot_y)
    odometry.pose.pose.position.z = float(message.robot_z)
    odometry.pose.pose.orientation = yaw_degrees_to_quaternion(message.robot_yaw)
    # PoseInfo has no covariance or twist. Keep this visual projection isolated
    # from canonical topics so it cannot be mistaken for an estimator input.
    return odometry


def _pose_stamped_from_odometry(odometry: Odometry) -> PoseStamped:
    pose = PoseStamped()
    pose.header = odometry.header
    pose.pose = odometry.pose.pose
    return pose


class OdomPathHistory:
    """Time-sampled, bounded pose trail that clears when ROS time rewinds."""

    def __init__(self, max_points: int = PATH_MAX_POINTS,
                 sample_period_ns: int = PATH_SAMPLE_PERIOD_NS) -> None:
        self.poses: Deque[PoseStamped] = deque(maxlen=max(1, int(max_points)))
        self.sample_period_ns = max(1, int(sample_period_ns))
        self.last_stamp_ns: Optional[int] = None
        self.last_sample_ns: Optional[int] = None

    def clear(self) -> None:
        self.poses.clear()
        self.last_stamp_ns = None
        self.last_sample_ns = None

    def append(self, message: PoseInfo, odometry: Odometry) -> Tuple[bool, bool]:
        stamp_ns = stamp_to_nanoseconds(message.stamp)
        rewound = self.last_stamp_ns is not None and stamp_ns < self.last_stamp_ns
        if rewound:
            self.clear()

        self.last_stamp_ns = stamp_ns
        sampled = (
            self.last_sample_ns is None
            or stamp_ns - self.last_sample_ns >= self.sample_period_ns
        )
        if sampled:
            self.poses.append(_pose_stamped_from_odometry(odometry))
            self.last_sample_ns = stamp_ns
        return rewound, sampled

    def to_message(self) -> Path:
        path = Path()
        path.header.frame_id = ODOM_FRAME
        if self.poses:
            path.header.stamp = self.poses[-1].header.stamp
        path.poses = list(self.poses)
        return path


def waypoint_path_to_nav_path(message: WaypointPath) -> Optional[Path]:
    """Convert the current planner's NED WaypointPath for RViz Path display."""
    frame_id = str(message.header.frame_id).strip()
    if frame_id not in ('', ODOM_FRAME):
        return None

    path = Path()
    path.header.stamp = message.header.stamp
    path.header.frame_id = ODOM_FRAME
    for waypoint in message.waypoints:
        values = (waypoint.x, waypoint.y, waypoint.z, waypoint.yaw)
        if not _is_finite(*values):
            continue
        pose = PoseStamped()
        pose.header.stamp = message.header.stamp
        pose.header.frame_id = ODOM_FRAME
        pose.pose.position.x = float(waypoint.x)
        pose.pose.position.y = float(waypoint.y)
        pose.pose.position.z = float(waypoint.z)
        yaw = float(waypoint.yaw)
        pose.pose.orientation.z = math.sin(yaw * 0.5)
        pose.pose.orientation.w = math.cos(yaw * 0.5)
        path.poses.append(pose)
    return path


def _copy_marker_header(marker: Marker, stamp, frame_id: str = ODOM_FRAME) -> None:
    marker.header.stamp = stamp
    marker.header.frame_id = frame_id


def _set_color(marker: Marker, rgba: Tuple[float, float, float, float]) -> None:
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = rgba


def _new_marker(namespace: str, marker_id: int, marker_type: int,
                stamp, lifetime_s: float) -> Marker:
    marker = Marker()
    _copy_marker_header(marker, stamp)
    marker.ns = namespace
    marker.id = int(marker_id)
    marker.type = int(marker_type)
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.lifetime = RclpyDuration(seconds=float(lifetime_s)).to_msg()
    return marker


def _delete_marker(namespace: str, marker_id: int, stamp) -> Marker:
    marker = Marker()
    _copy_marker_header(marker, stamp)
    marker.ns = namespace
    marker.id = int(marker_id)
    marker.action = Marker.DELETE
    return marker


def build_measurement_markers(
        message: ObjectMeasurementArray,
        previous_slots: int = 0,
        ray_length: float = 3.0,
        lifetime_s: float = MEASUREMENT_MARKER_LIFETIME_S,
) -> Tuple[MarkerArray, int]:
    """Build short-lived point/ray markers; ids are stable within each array."""
    markers = MarkerArray()
    valid_frame = str(message.header.frame_id).strip() == ODOM_FRAME
    ray_length = max(0.01, float(ray_length))

    for index, measurement in enumerate(message.measurements):
        point_id = index * 2
        ray_id = point_id + 1
        has_position = (
            valid_frame and measurement.has_position
            and _is_finite(measurement.world_x, measurement.world_y,
                           measurement.world_z)
        )
        if has_position:
            marker = _new_marker(
                'measurements/position', point_id, Marker.SPHERE,
                message.header.stamp, lifetime_s)
            marker.pose.position.x = float(measurement.world_x)
            marker.pose.position.y = float(measurement.world_y)
            marker.pose.position.z = float(measurement.world_z)
            stereo_forms = (
                int(ObjectMeasurement.FORM_FRONT_STEREO),
                int(ObjectMeasurement.FORM_DOWN_STEREO),
            )
            is_stereo = int(measurement.measurement_form) in stereo_forms
            marker.scale.x = marker.scale.y = marker.scale.z = (
                0.22 if is_stereo else 0.14)
            _set_color(marker, (1.0, 0.18, 0.72, 0.98) if is_stereo
                       else (0.10, 0.48, 1.0, 0.95))
            markers.markers.append(marker)
        else:
            markers.markers.append(_delete_marker(
                'measurements/position', point_id, message.header.stamp))

        ray_values = (
            measurement.ray_origin_x, measurement.ray_origin_y,
            measurement.ray_origin_z, measurement.ray_direction_x,
            measurement.ray_direction_y, measurement.ray_direction_z,
        )
        norm = math.sqrt(sum(float(value) ** 2 for value in ray_values[3:]))
        has_ray = (
            valid_frame and measurement.has_ray and _is_finite(*ray_values)
            and norm > 1e-9
        )
        if has_ray:
            direction = [float(value) / norm for value in ray_values[3:]]
            origin = [float(value) for value in ray_values[:3]]
            end = [origin[i] + direction[i] * ray_length for i in range(3)]
            marker = _new_marker(
                'measurements/ray', ray_id, Marker.LINE_STRIP,
                message.header.stamp, lifetime_s)
            marker.scale.x = 0.035
            _set_color(marker, (0.0, 0.92, 0.94, 0.95))
            start_point = Point(x=origin[0], y=origin[1], z=origin[2])
            end_point = Point(x=end[0], y=end[1], z=end[2])
            marker.points = [start_point, end_point]
            markers.markers.append(marker)
        else:
            markers.markers.append(_delete_marker(
                'measurements/ray', ray_id, message.header.stamp))

    for index in range(len(message.measurements), max(0, int(previous_slots))):
        markers.markers.append(_delete_marker(
            'measurements/position', index * 2, message.header.stamp))
        markers.markers.append(_delete_marker(
            'measurements/ray', index * 2 + 1, message.header.stamp))
    return markers, len(message.measurements)


def _track_color(status: int) -> Tuple[float, float, float, float]:
    if status == ObjectTrack.STATUS_STABLE:
        return (0.12, 0.88, 0.25, 0.95)
    if status == ObjectTrack.STATUS_STALE:
        return (1.0, 0.52, 0.05, 0.9)
    if status == ObjectTrack.STATUS_LOST:
        return (0.55, 0.55, 0.58, 0.65)
    return (1.0, 0.88, 0.08, 0.95)  # tentative/default


def build_track_markers(
        message: ObjectTrackArray,
        marker_ids: Dict[int, int],
        next_marker_id: int,
        lifetime_s: float = TRACK_MARKER_LIFETIME_S,
) -> Tuple[MarkerArray, Dict[int, int], int]:
    """Build status-coded track markers and deletes for tracks that disappeared."""
    output = MarkerArray()
    next_ids: Dict[int, int] = {}
    valid_frame = str(message.header.frame_id).strip() == ODOM_FRAME

    if valid_frame:
        for track in message.tracks:
            coordinates = (track.world_x, track.world_y, track.world_z)
            if not _is_finite(*coordinates, track.confidence):
                continue
            track_id = int(track.track_id)
            base_id = marker_ids.get(track_id)
            if base_id is None:
                base_id = int(next_marker_id)
                next_marker_id += 2
            next_ids[track_id] = base_id

            color = _track_color(int(track.status))
            sphere = _new_marker(
                'tracks/position', base_id, Marker.SPHERE,
                message.header.stamp, lifetime_s)
            sphere.pose.position.x = float(track.world_x)
            sphere.pose.position.y = float(track.world_y)
            sphere.pose.position.z = float(track.world_z)
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.22
            _set_color(sphere, color)
            output.markers.append(sphere)

            category = str(track.physical_class_name or track.class_name or 'object')
            label = '{} #{} ({:.2f})'.format(
                category, track_id, float(track.confidence))
            if track.estimate_source:
                label += ' ' + str(track.estimate_source)
            text = _new_marker(
                'tracks/label', base_id + 1, Marker.TEXT_VIEW_FACING,
                message.header.stamp, lifetime_s)
            text.pose.position.x = float(track.world_x)
            text.pose.position.y = float(track.world_y)
            text.pose.position.z = float(track.world_z) + 0.28
            text.scale.z = 0.18
            text.text = label
            _set_color(text, (1.0, 1.0, 1.0, 0.95))
            output.markers.append(text)

    for track_id, base_id in marker_ids.items():
        if track_id not in next_ids:
            output.markers.append(_delete_marker(
                'tracks/position', base_id, message.header.stamp))
            output.markers.append(_delete_marker(
                'tracks/label', base_id + 1, message.header.stamp))
    return output, next_ids, next_marker_id


def empty_path(stamp=None) -> Path:
    path = Path()
    path.header.frame_id = ODOM_FRAME
    if stamp is not None:
        path.header.stamp = stamp
    return path


class VisualizationAdapter(Node):
    """Publish RViz-only representations without changing canonical topics."""

    def __init__(self) -> None:
        super().__init__('uv_rviz_adapter')
        self.declare_parameter('ray_length', 3.0)
        self._ray_length = max(0.01, float(self.get_parameter('ray_length').value))
        self._history = OdomPathHistory()
        self._last_cleared_generation = None
        self._reset_stamp_ns = 0
        self._measurement_slots = 0
        self._track_marker_ids: Dict[int, int] = {}
        self._next_track_marker_id = 0
        self._warned_frames = set()

        self._odom_pub = self.create_publisher(Odometry, VIZ_ODOM, 10)
        self._odom_path_pub = self.create_publisher(Path, VIZ_ODOM_PATH, 10)
        self._measurement_pub = self.create_publisher(
            MarkerArray, VIZ_MEASUREMENTS, 10)
        self._track_pub = self.create_publisher(MarkerArray, VIZ_TRACKS, 10)
        self._planned_path_pub = self.create_publisher(Path, VIZ_PLANNED_PATH, 10)

        self.create_subscription(PoseInfo, STATE_ODOM, self._pose_callback, 10)
        self.create_subscription(StateResetResult, STATE_RESET_RESULT, self._reset_callback, 10)
        self.create_subscription(
            ObjectMeasurementArray, MEASUREMENTS,
            self._measurement_callback, 10)
        self.create_subscription(
            ObjectTrackArray, TRACKS, self._track_callback, 10)
        self.create_subscription(
            WaypointPath, TRAJECTORY, self._trajectory_callback, 10)

    def _warn_frame_once(self, topic: str, frame_id: str) -> None:
        key = (topic, frame_id)
        if key not in self._warned_frames:
            self._warned_frames.add(key)
            self.get_logger().warning(
                '{} uses frame {!r}; RViz adapter expects odom'.format(
                    topic, frame_id))

    def _pose_callback(self, message: PoseInfo) -> None:
        if not message.origin_initialized:
            self._last_cleared_generation = None
            return
        if (self._last_cleared_generation is not None and
                ((int(message.origin_generation) - self._last_cleared_generation)
                 & 0xffffffff) >= 0x80000000):
            self._last_cleared_generation = None
        if self._last_cleared_generation != int(message.origin_generation):
            self._reset_callback(StateResetResult(
                success=True, origin_generation=int(message.origin_generation)))
        odometry = pose_info_to_odometry(message)
        if odometry is None:
            self.get_logger().warning('dropping non-finite /auv/state/odom pose')
            return
        rewound, sampled = self._history.append(message, odometry)
        self._odom_pub.publish(odometry)
        if rewound or sampled:
            self._odom_path_pub.publish(self._history.to_message())

    def _reset_callback(self, _message: StateResetResult) -> None:
        if not _message.success:
            return
        generation = int(_message.origin_generation)
        if self._last_cleared_generation is not None and (
                ((generation - self._last_cleared_generation) & 0xffffffff) == 0
                or ((generation - self._last_cleared_generation) & 0xffffffff)
                >= 0x80000000):
            return
        self._last_cleared_generation = generation
        self._reset_stamp_ns = stamp_to_nanoseconds(self.get_clock().now().to_msg())
        self._history.clear()
        self._odom_path_pub.publish(empty_path())
        self._planned_path_pub.publish(empty_path())
        if self._measurement_slots:
            marker_array = MarkerArray()
            stamp = self.get_clock().now().to_msg()
            for index in range(self._measurement_slots):
                marker_array.markers.append(_delete_marker(
                    'measurements/position', index * 2, stamp))
                marker_array.markers.append(_delete_marker(
                    'measurements/ray', index * 2 + 1, stamp))
            self._measurement_pub.publish(marker_array)
            self._measurement_slots = 0
        if self._track_marker_ids:
            marker_array = MarkerArray()
            stamp = self.get_clock().now().to_msg()
            for base_id in self._track_marker_ids.values():
                marker_array.markers.append(_delete_marker(
                    'tracks/position', base_id, stamp))
                marker_array.markers.append(_delete_marker(
                    'tracks/label', base_id + 1, stamp))
            self._track_pub.publish(marker_array)
            self._track_marker_ids.clear()

    def _measurement_callback(self, message: ObjectMeasurementArray) -> None:
        if (self._reset_stamp_ns > 0
                and stamp_to_nanoseconds(message.header.stamp) <= self._reset_stamp_ns):
            return
        if str(message.header.frame_id).strip() != ODOM_FRAME:
            self._warn_frame_once(MEASUREMENTS, str(message.header.frame_id))
        markers, self._measurement_slots = build_measurement_markers(
            message, self._measurement_slots, self._ray_length)
        if markers.markers:
            self._measurement_pub.publish(markers)

    def _track_callback(self, message: ObjectTrackArray) -> None:
        if (self._reset_stamp_ns > 0
                and stamp_to_nanoseconds(message.header.stamp) <= self._reset_stamp_ns):
            return
        frame_id = str(message.header.frame_id).strip()
        if frame_id != ODOM_FRAME:
            self._warn_frame_once(TRACKS, frame_id)
        markers, self._track_marker_ids, self._next_track_marker_id = (
            build_track_markers(
                message, self._track_marker_ids, self._next_track_marker_id))
        if markers.markers:
            self._track_pub.publish(markers)

    def _trajectory_callback(self, message: WaypointPath) -> None:
        if (self._reset_stamp_ns > 0
                and stamp_to_nanoseconds(message.header.stamp) <= self._reset_stamp_ns):
            return
        path = waypoint_path_to_nav_path(message)
        if path is None:
            self._warn_frame_once(TRAJECTORY, str(message.header.frame_id))
            path = empty_path(message.header.stamp)
        self._planned_path_pub.publish(path)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisualizationAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
