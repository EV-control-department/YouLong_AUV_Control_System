"""Accumulate and robustly fit static odom-frame landmark tracks."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
import threading
import time

import numpy as np

from auv_protocol.topics import MEASUREMENTS, TRACKS
from uv_msgs.msg import ObjectMeasurementArray, ObjectTrack, ObjectTrackArray
from .localization.geometry import minimum_cost_assignment

from .tracking.static_landmarks import fit_static_landmark, ray_pair_geometry


@dataclass
class TrackState:
    track_id: int
    class_id: int
    class_name: str
    physical_class_name: str
    multi_instance: bool
    factors: list = field(default_factory=list)
    raw_detection_ids: set = field(default_factory=set)
    observation_ids: set = field(default_factory=set)
    source_views: set = field(default_factory=set)
    source_names: set = field(default_factory=set)
    position: tuple | None = None
    covariance: list = field(default_factory=lambda: [0.0] * 9)
    confidence: float = 0.0
    created_ns: int = 0
    last_ns: int = 0
    last_stamp: object = None
    last_observation_id: int = 0


class ObjectEstimator:
    def __init__(self, node):
        self.node = node
        self.publisher = node.create_publisher(ObjectTrackArray, TRACKS, 10)
        self.world_frame = str(node.declare_parameter('world_frame', 'odom').value)
        self.association_distance_m = float(
            node.declare_parameter('association_distance_m', 2.0).value)
        self.bearing_association_distance_m = float(
            node.declare_parameter('bearing_association_distance_m', 0.35).value)
        self.stale_after_s = float(node.declare_parameter('stale_after_s', 0.5).value)
        self.lost_after_s = float(node.declare_parameter('lost_after_s', 2.0).value)
        self.min_parallax_deg = float(node.declare_parameter('min_parallax_deg', 5.0).value)
        self.huber_delta = float(node.declare_parameter('huber_delta', 2.5).value)
        self.pose_translation_sigma_m = float(
            node.declare_parameter('pose_translation_sigma_m', 0.03).value)
        self.pose_rotation_sigma_deg = float(
            node.declare_parameter('pose_rotation_sigma_deg', 1.0).value)
        self.extrinsic_translation_sigma_m = float(
            node.declare_parameter('extrinsic_translation_sigma_m', 0.005).value)
        self.extrinsic_rotation_sigma_deg = float(
            node.declare_parameter('extrinsic_rotation_sigma_deg', 0.5).value)
        self._lock = threading.Lock()
        self._tracks = {}
        self._single_instance_tracks = {}
        self._next_id = 1
        self._warned_frames = set()
        node.create_subscription(ObjectMeasurementArray, MEASUREMENTS,
                                 self._measurements, 10)
        self.timer = node.create_timer(0.1, self._publish)

    @staticmethod
    def _finite(values):
        try:
            return all(math.isfinite(float(value)) for value in values)
        except (TypeError, ValueError):
            return False

    def _factor(self, measurement):
        source_ids = tuple(sorted({int(value)
                                   for value in measurement.source_detection_ids}))
        if not source_ids:
            # Compatibility fallback for older publishers. New localizer
            # messages always carry the raw detector source IDs.
            source_ids = (int(measurement.observation_id),)
        factor = {
            'observation_id': int(measurement.observation_id),
            'source_detection_ids': source_ids,
            'class_id': int(measurement.class_id),
            'class_name': str(measurement.class_name),
            'physical_class_name': str(measurement.physical_class_name).strip()
                                  or f'class_{int(measurement.class_id)}',
            'multi_instance': bool(measurement.multi_instance),
            'source_camera': str(measurement.source_camera),
            'confidence': max(0.0, min(1.0, float(measurement.confidence))),
            'stamp': measurement.observation_stamp,
        }
        if measurement.has_position:
            point = (float(measurement.world_x), float(measurement.world_y),
                     float(measurement.world_z))
            covariance = [float(value) for value in measurement.position_covariance]
            if not self._finite(point) or not self._finite(covariance):
                return None
            factor['position'] = point
            factor['covariance'] = covariance
            return factor
        if measurement.has_ray:
            origin = (float(measurement.ray_origin_x), float(measurement.ray_origin_y),
                      float(measurement.ray_origin_z))
            direction = np.asarray(
                (float(measurement.ray_direction_x),
                 float(measurement.ray_direction_y),
                 float(measurement.ray_direction_z)), dtype=float)
            if not self._finite(origin) or not self._finite(direction):
                return None
            norm = float(np.linalg.norm(direction))
            if norm <= 1e-12:
                return None
            factor['ray_origin'] = origin
            factor['ray_direction'] = tuple(float(value) for value in direction / norm)
            factor['ray_sigma_rad'] = max(0.0, float(measurement.ray_sigma_rad))
            if not math.isfinite(factor['ray_sigma_rad']):
                return None
            return factor
        return None

    @staticmethod
    def _view(source_camera):
        roots = []
        for source in str(source_camera).lower().split('+'):
            root = source.split('_', 1)[0]
            if root in ('front', 'down'):
                roots.append(root)
            elif root:
                roots.append(root)
        return roots

    def _new_track(self, factor, now):
        state = TrackState(
            track_id=self._next_id,
            class_id=factor['class_id'],
            class_name=factor['class_name'],
            physical_class_name=factor['physical_class_name'],
            multi_instance=factor['multi_instance'],
            created_ns=now, last_ns=now)
        self._next_id += 1
        self._tracks[state.track_id] = state
        if not state.multi_instance:
            self._single_instance_tracks[state.physical_class_name] = state.track_id
        return state

    @staticmethod
    def _point_to_ray_distance(point, ray):
        origin = np.asarray(ray['ray_origin'], dtype=float)
        direction = np.asarray(ray['ray_direction'], dtype=float)
        delta = np.asarray(point, dtype=float) - origin
        along = float(np.dot(delta, direction))
        if along <= 0.0:
            return None
        return float(np.linalg.norm(delta - along * direction))

    def _association_cost(self, factor, state):
        gate = (self.association_distance_m if factor.get('position') is not None
                else self.bearing_association_distance_m)
        if state.position is not None:
            if factor.get('position') is not None:
                distance = float(np.linalg.norm(
                    np.asarray(factor['position']) - np.asarray(state.position)))
            else:
                distance = self._point_to_ray_distance(state.position, factor)
            if distance is None or distance >= gate:
                return None
            return distance / max(gate, 1e-9)

        if factor.get('position') is not None:
            distances = [self._point_to_ray_distance(factor['position'], old)
                         for old in state.factors if old.get('ray_origin') is not None]
            distances = [value for value in distances if value is not None]
            if not distances:
                return None
            distance = min(distances)
            return distance / max(self.bearing_association_distance_m, 1e-9) \
                if distance < self.bearing_association_distance_m else None

        old_rays = [old for old in state.factors if old.get('ray_origin') is not None]
        if not old_rays:
            return None
        distances = []
        for old in old_rays:
            geometry = ray_pair_geometry(old, factor)
            if geometry is None:
                continue
            _, _, range_old, range_new, separation, _ = geometry
            if range_old > 0.0 and range_new > 0.0:
                distances.append(separation)
        if not distances:
            return None
        distance = min(distances)
        return (distance / max(self.bearing_association_distance_m, 1e-9)
                if distance < self.bearing_association_distance_m else None)

    def _append_factor(self, state, factor, now):
        state.factors.append(factor)
        state.raw_detection_ids.update(factor['source_detection_ids'])
        state.observation_ids.add(factor['observation_id'])
        state.source_names.add(factor['source_camera'])
        state.source_views.update(self._view(factor['source_camera']))
        state.class_id = factor['class_id']
        state.class_name = factor['class_name']
        state.confidence = max(state.confidence, factor['confidence'])
        state.last_observation_id = factor['observation_id']
        state.last_ns = now
        state.last_stamp = factor['stamp']
        fitted = fit_static_landmark(
            state.factors, min_parallax_deg=self.min_parallax_deg,
            huber_delta=self.huber_delta,
            pose_translation_sigma_m=self.pose_translation_sigma_m,
            pose_rotation_sigma_deg=self.pose_rotation_sigma_deg,
            extrinsic_translation_sigma_m=self.extrinsic_translation_sigma_m,
            extrinsic_rotation_sigma_deg=self.extrinsic_rotation_sigma_deg,
            initial_position=state.position)
        if fitted is not None:
            state.position, state.covariance = fitted

    def _measurements(self, message):
        frame = str(message.header.frame_id).strip()
        if frame != self.world_frame:
            if frame not in self._warned_frames:
                self._warned_frames.add(frame)
                self.node.get_logger().warning(
                    f'dropping measurements in frame {frame!r}; expected '
                    f'{self.world_frame!r}')
            return
        factors = [self._factor(item) for item in message.measurements]
        factors = [factor for factor in factors if factor is not None]
        if not factors:
            return
        now = time.monotonic_ns()
        with self._lock:
            owned_ids = set()
            for state in self._tracks.values():
                owned_ids.update(state.raw_detection_ids)
            factors = [factor for factor in factors
                       if not owned_ids.intersection(factor['source_detection_ids'])]
            groups = {}
            for factor in factors:
                groups.setdefault(factor['physical_class_name'], []).append(factor)

            for physical_name, group in groups.items():
                multi_instance = any(item['multi_instance'] for item in group)
                if not multi_instance:
                    track_id = self._single_instance_tracks.get(physical_name)
                    state = self._tracks.get(track_id) if track_id is not None else None
                    if state is None:
                        state = self._new_track(group[0], now)
                    for factor in group:
                        # Duplicate delivery of one factor is ignored.
                        if set(factor['source_detection_ids']).intersection(
                                state.raw_detection_ids):
                            continue
                        self._append_factor(state, factor, now)
                    continue

                candidates = [state for state in self._tracks.values()
                              if state.physical_class_name == physical_name and
                              state.multi_instance]
                costs = [[self._association_cost(factor, state)
                          for state in candidates] for factor in group]
                assignment = {row: column for row, column, _ in
                              minimum_cost_assignment(costs, 1.0)}
                for row, factor in enumerate(group):
                    state = (candidates[assignment[row]]
                             if row in assignment else self._new_track(factor, now))
                    if set(factor['source_detection_ids']).intersection(
                            state.raw_detection_ids):
                        continue
                    self._append_factor(state, factor, now)

    def _source_label(self, state):
        if state.source_views:
            order = [name for name in ('front', 'down') if name in state.source_views]
            order.extend(sorted(state.source_views.difference(order)))
            return '+'.join(order)
        return '+'.join(sorted(state.source_names)) or 'unknown'

    def _publish(self):
        now = time.monotonic_ns()
        output = ObjectTrackArray()
        output.header.stamp = self.node.get_clock().now().to_msg()
        output.header.frame_id = self.world_frame
        with self._lock:
            states = tuple(self._tracks.values())
        for state in states:
            # Keep ray-only hypotheses internally, but do not publish a fake
            # origin as a 3D landmark before parallax makes it observable.
            if state.position is None:
                continue
            age = (now - state.last_ns) / 1e9
            track = ObjectTrack()
            track.track_id = state.track_id
            track.last_observation_id = state.last_observation_id
            track.class_id = state.class_id
            track.class_name = state.class_name
            track.physical_class_name = state.physical_class_name
            track.estimate_source = self._source_label(state)
            track.confidence = state.confidence
            track.measurement_count = len(state.observation_ids)
            track.age_sec = max(0.0, (now - state.created_ns) / 1e9)
            track.last_measurement_stamp = state.last_stamp
            track.world_x, track.world_y, track.world_z = state.position
            track.position_covariance = state.covariance
            if age > self.lost_after_s:
                track.status = ObjectTrack.STATUS_LOST
            elif age > self.stale_after_s:
                track.status = ObjectTrack.STATUS_STALE
            elif track.measurement_count >= 3:
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
