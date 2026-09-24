"""Associate geometry measurements and publish persistent object tracks."""

from __future__ import annotations

from dataclasses import dataclass
import threading
import time

from auv_protocol.topics import MEASUREMENTS, TRACKS
from uv_msgs.msg import ObjectMeasurementArray, ObjectTrack, ObjectTrackArray

from .tracking.association import nearest
from .tracking.filter import exponential


@dataclass
class TrackState:
    track_id: int
    class_id: int
    source: str
    position: tuple | None
    covariance: list
    confidence: float
    count: int
    created_ns: int
    last_ns: int
    last_stamp: object


class ObjectEstimator:
    def __init__(self, node):
        self.node = node
        self.publisher = node.create_publisher(ObjectTrackArray, TRACKS, 10)
        self.association_distance_m = float(
            node.declare_parameter('association_distance_m', 2.0).value)
        self.stale_after_s = float(node.declare_parameter('stale_after_s', 0.5).value)
        self.lost_after_s = float(node.declare_parameter('lost_after_s', 2.0).value)
        self._lock = threading.Lock()
        self._tracks = {}
        self._next_id = 1
        node.create_subscription(ObjectMeasurementArray, MEASUREMENTS,
                                 self._measurements, 10)
        self.timer = node.create_timer(0.1, self._publish)

    def _measurements(self, message):
        now = time.monotonic_ns()
        with self._lock:
            for measurement in message.measurements:
                if not measurement.has_position:
                    continue
                position = (measurement.world_x, measurement.world_y, measurement.world_z)
                candidates = {track_id: state for track_id, state in self._tracks.items()
                              if state.class_id == int(measurement.class_id)}
                track_id = nearest(candidates, position, self.association_distance_m)
                if track_id is None:
                    track_id = self._next_id
                    self._next_id += 1
                    self._tracks[track_id] = TrackState(
                        track_id=track_id,
                        class_id=int(measurement.class_id),
                        source=str(measurement.source_camera),
                        position=position,
                        covariance=list(measurement.position_covariance),
                        confidence=float(measurement.confidence), count=1,
                        created_ns=now, last_ns=now,
                        last_stamp=measurement.observation_stamp)
                    continue
                state = self._tracks[track_id]
                state.position = exponential(state.position, position, 0.5)
                state.covariance = list(measurement.position_covariance)
                state.confidence = max(state.confidence * 0.8,
                                       float(measurement.confidence))
                state.source = str(measurement.source_camera)
                state.count += 1
                state.last_ns = now
                state.last_stamp = measurement.observation_stamp

    def _publish(self):
        now = time.monotonic_ns()
        output = ObjectTrackArray()
        output.header.stamp = self.node.get_clock().now().to_msg()
        with self._lock:
            states = tuple(self._tracks.values())
        for state in states:
            age = (now - state.last_ns) / 1e9
            track = ObjectTrack()
            track.track_id = state.track_id
            track.class_id = state.class_id
            try:
                from uv_camera.model_classes import model_class_name, physical_class_name
                track.class_name = model_class_name(state.class_id)
                track.physical_class_name = physical_class_name(state.class_id)
            except Exception:
                track.class_name = f'class_{state.class_id}'
                track.physical_class_name = track.class_name
            track.estimate_source = state.source
            track.confidence = state.confidence
            track.measurement_count = state.count
            track.age_sec = max(0.0, (now - state.created_ns) / 1e9)
            track.last_measurement_stamp = state.last_stamp
            if state.position is not None:
                track.world_x, track.world_y, track.world_z = state.position
                track.position_covariance = state.covariance
            if age > self.lost_after_s:
                track.status = ObjectTrack.STATUS_LOST
            elif age > self.stale_after_s:
                track.status = ObjectTrack.STATUS_STALE
            elif state.count >= 3:
                track.status = ObjectTrack.STATUS_STABLE
            else:
                track.status = ObjectTrack.STATUS_TENTATIVE
            output.tracks.append(track)
        self.publisher.publish(output)


def main(args=None):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    rclpy.init(args=args)
    node = rclpy.create_node('object_estimator')
    ObjectEstimator(node)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
