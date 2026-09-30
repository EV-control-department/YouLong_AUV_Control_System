import math

from nav_msgs.msg import Path
from visualization_msgs.msg import Marker

from uv_msgs.msg import (
    ObjectMeasurement,
    ObjectMeasurementArray,
    ObjectTrack,
    ObjectTrackArray,
    PoseInfo,
    Waypoint,
    WaypointPath,
)
from uv_rviz.visualization_adapter import (
    OdomPathHistory,
    build_measurement_markers,
    build_track_markers,
    pose_info_to_odometry,
    waypoint_path_to_nav_path,
)


def _pose(stamp_ns, x=0.0, yaw_deg=0.0):
    message = PoseInfo()
    message.stamp.sec = int(stamp_ns // 1_000_000_000)
    message.stamp.nanosec = int(stamp_ns % 1_000_000_000)
    message.robot_x = float(x)
    message.robot_y = 2.0
    message.robot_z = 1.0
    message.robot_yaw = float(yaw_deg)
    return message


def test_pose_info_becomes_odom_pose_with_degrees_converted_to_quaternion():
    odometry = pose_info_to_odometry(_pose(1_000_000_000, x=4.0, yaw_deg=90.0))

    assert odometry.header.frame_id == 'odom'
    assert odometry.child_frame_id == 'base_link'
    assert odometry.pose.pose.position.x == 4.0
    assert odometry.pose.pose.position.y == 2.0
    assert math.isclose(odometry.pose.pose.orientation.z, math.sqrt(0.5))
    assert math.isclose(odometry.pose.pose.orientation.w, math.sqrt(0.5))


def test_odom_history_is_sampled_bounded_and_cleared_on_time_rewind():
    history = OdomPathHistory(max_points=3, sample_period_ns=500_000_000)
    for index, stamp in enumerate((0, 500_000_000, 1_000_000_000, 1_500_000_000)):
        message = _pose(stamp, x=float(index))
        odometry = pose_info_to_odometry(message)
        rewound, sampled = history.append(message, odometry)
        assert not rewound
        assert sampled

    path = history.to_message()
    assert isinstance(path, Path)
    assert len(path.poses) == 3
    assert [pose.pose.position.x for pose in path.poses] == [1.0, 2.0, 3.0]

    message = _pose(200_000_000, x=9.0)
    rewound, sampled = history.append(message, pose_info_to_odometry(message))
    assert rewound
    assert sampled
    assert len(history.to_message().poses) == 1
    assert history.to_message().poses[0].pose.position.x == 9.0


def test_odom_history_can_be_cleared_on_explicit_reset():
    history = OdomPathHistory()
    message = _pose(2_000_000_000, x=1.0)
    history.append(message, pose_info_to_odometry(message))
    assert len(history.to_message().poses) == 1

    history.clear()
    assert len(history.to_message().poses) == 0


def test_measurement_positions_rays_and_stale_marker_deletes():
    message = ObjectMeasurementArray()
    message.header.frame_id = 'odom'
    message.header.stamp.sec = 5
    measurement = ObjectMeasurement()
    measurement.observation_id = 7
    measurement.has_position = True
    measurement.world_x = 1.0
    measurement.world_y = 2.0
    measurement.world_z = 3.0
    measurement.has_ray = True
    measurement.ray_origin_x = -1.0
    measurement.ray_origin_y = 0.0
    measurement.ray_origin_z = 1.0
    measurement.ray_direction_x = 0.0
    measurement.ray_direction_y = 2.0
    measurement.ray_direction_z = 0.0
    message.measurements.append(measurement)

    markers, slots = build_measurement_markers(message, ray_length=3.0)
    assert slots == 1
    assert len(markers.markers) == 2
    assert markers.markers[0].action == Marker.ADD
    assert markers.markers[0].pose.position.x == 1.0
    assert markers.markers[1].action == Marker.ADD
    assert math.isclose(markers.markers[1].points[1].y, 3.0)

    empty = ObjectMeasurementArray()
    empty.header.frame_id = 'odom'
    deletes, slots = build_measurement_markers(empty, previous_slots=1)
    assert slots == 0
    assert len(deletes.markers) == 2
    assert all(marker.action == Marker.DELETE for marker in deletes.markers)


def test_invalid_measurement_coordinates_delete_the_old_marker():
    message = ObjectMeasurementArray()
    message.header.frame_id = 'odom'
    measurement = ObjectMeasurement()
    measurement.has_position = True
    measurement.world_x = float('nan')
    measurement.has_ray = True
    message.measurements.append(measurement)

    markers, _ = build_measurement_markers(message)
    assert len(markers.markers) == 2
    assert all(marker.action == Marker.DELETE for marker in markers.markers)


def test_track_status_colors_and_removed_track_deletes():
    message = ObjectTrackArray()
    message.header.frame_id = 'odom'
    statuses = (
        ObjectTrack.STATUS_TENTATIVE,
        ObjectTrack.STATUS_STABLE,
        ObjectTrack.STATUS_STALE,
        ObjectTrack.STATUS_LOST,
    )
    for index, status in enumerate(statuses):
        track = ObjectTrack()
        track.track_id = index + 10
        track.class_name = 'buoy'
        track.physical_class_name = 'buoy'
        track.world_x = float(index)
        track.confidence = 0.75
        track.status = status
        message.tracks.append(track)

    markers, ids, next_id = build_track_markers(message, {}, 0)
    spheres = [marker for marker in markers.markers
               if marker.ns == 'tracks/position']
    assert len(spheres) == 4
    assert len(ids) == 4
    assert next_id == 8
    assert spheres[0].color.r == 1.0
    assert spheres[1].color.g > spheres[1].color.r
    assert spheres[2].color.r == 1.0
    assert spheres[3].color.r == spheres[3].color.g

    empty = ObjectTrackArray()
    empty.header.frame_id = 'odom'
    deleted, ids, _ = build_track_markers(empty, ids, next_id)
    assert ids == {}
    assert len(deleted.markers) == 8
    assert all(marker.action == Marker.DELETE for marker in deleted.markers)


def test_waypoint_path_conversion_preserves_ned_positions_and_yaw():
    message = WaypointPath()
    message.header.frame_id = 'odom'
    message.header.stamp.sec = 8
    waypoint = Waypoint()
    waypoint.x = 3.0
    waypoint.y = 4.0
    waypoint.z = 2.0
    waypoint.yaw = math.pi / 2.0
    message.waypoints.append(waypoint)

    path = waypoint_path_to_nav_path(message)
    assert path.header.frame_id == 'odom'
    assert len(path.poses) == 1
    assert path.poses[0].pose.position.z == 2.0
    assert math.isclose(path.poses[0].pose.orientation.z, math.sqrt(0.5))
    assert math.isclose(path.poses[0].pose.orientation.w, math.sqrt(0.5))


def test_waypoint_path_rejects_an_unexpected_frame():
    message = WaypointPath()
    message.header.frame_id = 'map'
    assert waypoint_path_to_nav_path(message) is None
