"""Batch static landmark localization from per-view rolling ray pools."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
import re
import threading
import time

import numpy as np

from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
)
from auv_protocol.model_mapping import ModelClassRegistry
from auv_protocol.topics import (
    MEASUREMENTS, MODEL_CLASS_MAPPING, STATE_ODOM, STATE_RESET_RESULT, TRACKS,
)
from uv_msgs.msg import (
    StateResetResult, PoseInfo,
    ModelClassMapping, ObjectMeasurementArray, ObjectTrack, ObjectTrackArray,
)
from .localization.geometry import minimum_cost_assignment


@dataclass
class TrackState:
    track_id: int
    class_id: int
    class_name: str
    physical_class_name: str
    view: str
    multi_instance: bool
    position: tuple | None = None
    covariance: list = field(default_factory=lambda: [0.0] * 9)
    confidence: float = 0.0
    measurement_count: int = 0
    created_ns: int = 0
    last_ns: int = 0
    last_stamp: object = None
    last_observation_id: int = 0


class ObjectEstimator:
    """Estimate static visual anchors; each view/class owns an isolated ray pool."""

    def __init__(self, node):
        self.node = node
        self.publisher = node.create_publisher(ObjectTrackArray, TRACKS, 10)
        self.world_frame = str(node.declare_parameter('world_frame', 'odom').value)
        # Zero keeps all observations for the duration of this estimator run.
        # A positive value opts into count-bounded memory usage.
        requested_capacity = int(node.declare_parameter(
            'observation_pool_size', 0).value)
        self.pool_capacity = max(2, requested_capacity) if requested_capacity > 0 else 0
        self.seed_ray_limit = max(2, int(node.declare_parameter(
            'candidate_ray_limit', 80).value))
        self.seed_pair_limit = max(1, int(node.declare_parameter(
            'candidate_pair_limit', 2400).value))
        self.max_candidates = max(1, int(node.declare_parameter(
            'max_candidate_clusters', 8).value))
        self.seed_cluster_radius_m = max(0.01, float(node.declare_parameter(
            'seed_cluster_radius_m', 0.35).value))
        self.max_pair_gap_m = max(0.01, float(node.declare_parameter(
            'max_pair_gap_m', 1.0).value))
        self.min_parallax_deg = max(0.1, float(node.declare_parameter(
            'min_parallax_deg', 5.0).value))
        self.clutter_prior = min(0.49, max(1e-4, float(node.declare_parameter(
            'clutter_prior', 0.08).value)))
        self.huber_delta = max(0.1, float(node.declare_parameter(
            'huber_delta', 2.5).value))
        self.lm_iterations = max(1, int(node.declare_parameter(
            'lm_iterations', 10).value))
        self.association_cycles = max(1, int(node.declare_parameter(
            'association_cycles', 3).value))
        self.pose_translation_sigma_m = max(0.0, float(node.declare_parameter(
            'pose_translation_sigma_m', 0.03).value))
        self.pose_rotation_sigma_rad = math.radians(float(node.declare_parameter(
            'pose_rotation_sigma_deg', 1.0).value))
        self.extrinsic_translation_sigma_m = max(0.0, float(node.declare_parameter(
            'extrinsic_translation_sigma_m', 0.005).value))
        self.extrinsic_rotation_sigma_rad = math.radians(float(node.declare_parameter(
            'extrinsic_rotation_sigma_deg', 0.5).value))
        self.anchor_sigma_default_m = max(0.0, float(node.declare_parameter(
            'anchor_sigma_default_m', 0.10).value))
        self.anchor_sigma_by_class = self._declare_anchor_sigmas(node)
        self._mapping_registry = ModelClassRegistry.empty()
        mapping_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(
            ModelClassMapping, MODEL_CLASS_MAPPING,
            self._mapping_callback, mapping_qos)
        self.max_instance_association_m = max(0.1, float(node.declare_parameter(
            'instance_association_distance_m', 1.5).value))
        self.stable_covariance_trace_m2 = max(0.0, float(node.declare_parameter(
            'stable_covariance_trace_m2', 0.04).value))
        self._lock = threading.Lock()
        self._pools = {}                 # (view, physical class) -> ray list
        self._pool_source_ids = {}       # pool key -> retained detection IDs
        self._dirty_pools = set()
        self._tracks = {}
        self._pool_track_ids = {}        # pool key -> persistent track IDs
        self._next_id = 1
        self._reset_stamp_ns = 0
        self._last_cleared_generation = None
        self._origin_ready = True
        self._warned_frames = set()
        self._last_log_ns = {}
        node.create_subscription(ObjectMeasurementArray, MEASUREMENTS,
                                 self._measurements, 10)
        node.create_subscription(
            StateResetResult, STATE_RESET_RESULT, self._reset_callback, 10)
        node.create_subscription(
            PoseInfo, STATE_ODOM, self._origin_state_callback, 10)
        self.timer = node.create_timer(0.1, self._publish)

    def _declare_anchor_sigmas(self, node):
        # Detection centers on these objects move with visible orientation, so
        # give their ray likelihood a wider class-specific anchor uncertainty.
        defaults = {
            'collection_frame': 0.14,
            'target_rack': 0.14,
            'gate': 0.10,
            'guide_line': 0.16,
        }
        names = set(defaults)
        values = {}
        for name in sorted(name for name in names if name):
            param = 'anchor_sigma_{}_m'.format(
                re.sub(r'[^a-z0-9]+', '_', name).strip('_'))
            value = defaults.get(name, self.anchor_sigma_default_m)
            values[name] = max(0.0, float(node.declare_parameter(param, value).value))
        return values

    def _mapping_callback(self, message):
        try:
            self._mapping_registry = ModelClassRegistry.from_message(message)
            for entry in self._mapping_registry.entries:
                name = entry.object
                if name:
                    self.anchor_sigma_by_class.setdefault(
                        name, self.anchor_sigma_default_m)
        except Exception as error:
            self.node.get_logger().error(
                'invalid model class mapping: {}'.format(error))

    @staticmethod
    def _finite(values):
        try:
            return all(math.isfinite(float(value)) for value in values)
        except (TypeError, ValueError):
            return False

    @staticmethod
    def _view(source_camera):
        root = str(source_camera).strip().lower().split('_', 1)[0]
        return root if root in ('front', 'down') else None

    def _factor(self, measurement, arrival_ns):
        if not measurement.has_ray:
            return None
        source_ids = tuple(sorted({int(value)
                                   for value in measurement.source_detection_ids}))
        if not source_ids:
            source_ids = (int(measurement.observation_id),)
        view = self._view(measurement.source_camera)
        if view is None:
            return None
        physical_name = str(measurement.physical_class_name).strip().lower()
        if not physical_name:
            physical_name = 'class_{}'.format(int(measurement.class_id))
        origin = np.asarray((measurement.ray_origin_x, measurement.ray_origin_y,
                             measurement.ray_origin_z), dtype=float)
        direction = np.asarray((measurement.ray_direction_x,
                                measurement.ray_direction_y,
                                measurement.ray_direction_z), dtype=float)
        if not self._finite(origin) or not self._finite(direction):
            return None
        norm = float(np.linalg.norm(direction))
        if norm <= 1e-12:
            return None
        sigma = float(measurement.ray_sigma_rad)
        confidence = float(measurement.confidence)
        if not math.isfinite(sigma) or sigma < 0.0 or not math.isfinite(confidence):
            return None
        direction /= norm
        return {
            'observation_id': int(measurement.observation_id),
            'source_detection_ids': source_ids,
            'class_id': int(measurement.class_id),
            'class_name': str(measurement.class_name),
            'physical_class_name': physical_name,
            'multi_instance': bool(measurement.multi_instance),
            'view': view,
            'source_camera': str(measurement.source_camera),
            'confidence': max(0.0, min(1.0, confidence)),
            'stamp': measurement.observation_stamp,
            'ray_origin': origin,
            'ray_direction': direction,
            'ray_sigma_rad': sigma,
            'basis': self._tangent_basis(direction),
            'arrival_ns': arrival_ns,
        }

    @staticmethod
    def _tangent_basis(direction):
        axis = np.eye(3)[int(np.argmin(np.abs(direction)))]
        first = np.cross(direction, axis)
        first /= max(float(np.linalg.norm(first)), 1e-12)
        second = np.cross(direction, first)
        return np.column_stack((first, second))

    def _measurements(self, message):
        if not self._origin_ready:
            return
        frame = str(message.header.frame_id).strip()
        if frame != self.world_frame:
            if frame not in self._warned_frames:
                self._warned_frames.add(frame)
                self.node.get_logger().warning(
                    f'dropping measurements in frame {frame!r}; expected '
                    f'{self.world_frame!r}')
            return
        now = time.monotonic_ns()
        factors = [self._factor(item, now) for item in message.measurements]
        factors = [factor for factor in factors if factor is not None]
        if not factors:
            return
        stamp = message.header.stamp
        message_stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        with self._lock:
            # Queued measurements captured before an odom reset are expressed
            # in the old frame and must not seed the newly cleared estimator.
            if (self._reset_stamp_ns > 0
                    and message_stamp_ns <= self._reset_stamp_ns):
                return
            for factor in factors:
                key = (factor['view'], factor['physical_class_name'])
                pool = self._pools.setdefault(key, [])
                seen_ids = self._pool_source_ids.setdefault(key, set())
                source_ids = set(factor['source_detection_ids'])
                # A retransmitted camera detection must not gain extra weight.
                if not seen_ids.isdisjoint(source_ids):
                    continue
                pool.append(factor)
                seen_ids.update(source_ids)
                if self.pool_capacity and len(pool) > self.pool_capacity:
                    evicted = pool[:-self.pool_capacity]
                    del pool[:-self.pool_capacity]
                    for ray in evicted:
                        seen_ids.difference_update(ray['source_detection_ids'])
                self._dirty_pools.add(key)

    def _origin_state_callback(self, message):
        self._origin_ready = bool(message.origin_initialized)
        if not self._origin_ready:
            self._last_cleared_generation = None
            return
        if (self._last_cleared_generation is not None and
                ((int(message.origin_generation) - self._last_cleared_generation)
                 & 0xffffffff) >= 0x80000000):
            self._last_cleared_generation = None
        if self._last_cleared_generation != int(message.origin_generation):
            self._reset_callback(StateResetResult(
                success=True, origin_generation=int(message.origin_generation)))

    def _reset_callback(self, _message):
        """Clear world-frame estimates when BasicMotion redefines odom."""
        if not _message.success:
            return
        generation = int(_message.origin_generation)
        if self._last_cleared_generation is not None and (
                ((generation - self._last_cleared_generation) & 0xffffffff) == 0
                or ((generation - self._last_cleared_generation) & 0xffffffff)
                >= 0x80000000):
            return
        self._last_cleared_generation = generation
        self._origin_ready = True
        now = self.node.get_clock().now()
        reset_stamp_ns = int(getattr(now, 'nanoseconds', 0))
        if reset_stamp_ns <= 0:
            stamp = now.to_msg()
            reset_stamp_ns = (int(stamp.sec) * 1_000_000_000
                              + int(stamp.nanosec))
        with self._lock:
            self._reset_stamp_ns = reset_stamp_ns
            self._pools.clear()
            self._pool_source_ids.clear()
            self._dirty_pools.clear()
            self._tracks.clear()
            self._pool_track_ids.clear()
            self._last_log_ns.clear()
            # Keep _next_id monotonic so consumers never see reused track IDs.
        self.node.get_logger().info(
            'ObjectEstimator cleared ray pools and tracks after STATE_RESET_RESULT')
        self._publish()

    def _seed_candidates(self, rays):
        if len(rays) > self.seed_ray_limit:
            recent_count = max(1, self.seed_ray_limit // 2)
            history = rays[:-recent_count]
            history_count = self.seed_ray_limit - recent_count
            indices = np.linspace(0, len(history) - 1, history_count,
                                  dtype=int)
            sampled = [history[int(index)] for index in indices]
            sampled.extend(rays[-recent_count:])
        else:
            sampled = rays
        count = len(sampled)
        if count < 2:
            return []
        first_indices, second_indices = np.triu_indices(count, 1)
        if len(first_indices) > self.seed_pair_limit:
            sample = np.linspace(0, len(first_indices) - 1,
                                 self.seed_pair_limit, dtype=int)
            first_indices = first_indices[sample]
            second_indices = second_indices[sample]
        first = [sampled[index] for index in first_indices]
        second = [sampled[index] for index in second_indices]
        d1 = np.asarray([ray['ray_direction'] for ray in first], dtype=float)
        d2 = np.asarray([ray['ray_direction'] for ray in second], dtype=float)
        o1 = np.asarray([ray['ray_origin'] for ray in first], dtype=float)
        o2 = np.asarray([ray['ray_origin'] for ray in second], dtype=float)
        dots = np.einsum('ij,ij->i', d1, d2)
        denominators = 1.0 - dots * dots
        w0 = o1 - o2
        d1w = np.einsum('ij,ij->i', d1, w0)
        d2w = np.einsum('ij,ij->i', d2, w0)
        safe_denominators = np.maximum(denominators, 1e-12)
        t1 = (-d1w + dots * d2w) / safe_denominators
        t2 = dots * t1 + d2w
        p1 = o1 + t1[:, None] * d1
        p2 = o2 + t2[:, None] * d2
        gaps = np.linalg.norm(p1 - p2, axis=1)
        sine_angles = np.linalg.norm(np.cross(d1, d2), axis=1)
        angles = np.arcsin(np.clip(sine_angles, 0.0, 1.0))
        valid = ((denominators > 1e-12) & (t1 > 0.0) & (t2 > 0.0) &
                 (gaps < self.max_pair_gap_m) &
                 (angles >= math.radians(self.min_parallax_deg)))
        selected = np.flatnonzero(valid)
        if not len(selected):
            return []
        points = 0.5 * (p1[selected] + p2[selected])
        confidence = np.asarray([
            max(0.05, first[index]['confidence'] * second[index]['confidence'])
            for index in selected])
        weights = confidence / (gaps[selected] ** 2 + 0.01 * 0.01)
        seeds = [(points[offset], float(weights[offset]),
                  int(first_indices[index]), int(second_indices[index]))
                 for offset, index in enumerate(selected)]
        seeds.sort(key=lambda seed: seed[1], reverse=True)

        points = np.asarray([seed[0] for seed in seeds], dtype=float)
        consumed = np.zeros(len(seeds), dtype=bool)
        candidates = []
        radius2 = self.seed_cluster_radius_m ** 2
        for seed_index in range(len(seeds)):
            if consumed[seed_index]:
                continue
            delta = points - points[seed_index]
            members = np.flatnonzero(np.einsum('ij,ij->i', delta, delta) <= radius2)
            if not len(members):
                continue
            consumed[members] = True
            member_weights = np.asarray([seeds[index][1] for index in members])
            # Limit a single near-zero-gap pair from dominating an entire seed.
            cap = float(np.percentile(member_weights, 75))
            member_weights = np.minimum(member_weights, max(cap, 1e-9))
            center = np.average(points[members], axis=0, weights=member_weights)
            ray_indices = set()
            score = 0.0
            for index in members:
                _, weight, a, b = seeds[int(index)]
                ray_indices.update((a, b))
                score += min(weight, cap)
            candidate = {
                'position': center,
                'seed_score': float(score),
                'seed_support': len(ray_indices),
            }
            if any(np.linalg.norm(candidate['position'] - old['position']) <
                   self.seed_cluster_radius_m for old in candidates):
                continue
            candidates.append(candidate)
        candidates.sort(key=lambda item: (item['seed_support'], item['seed_score']),
                        reverse=True)
        return candidates[:self.max_candidates]

    def _anchor_sigma(self, physical_name):
        return self.anchor_sigma_by_class.get(
            physical_name, self.anchor_sigma_default_m)

    def _residual(self, factor, position):
        delta = position - factor['ray_origin']
        distance = float(np.linalg.norm(delta))
        if not math.isfinite(distance) or distance <= 1e-6:
            return None
        predicted = delta / distance
        if float(np.dot(delta, factor['ray_direction'])) <= 0.0:
            return None
        error = factor['basis'].T @ (predicted - factor['ray_direction'])
        return error, predicted, distance

    def _sigma(self, factor, distance):
        physical_name = factor['physical_class_name']
        translational = (self.pose_translation_sigma_m ** 2 +
                         self.extrinsic_translation_sigma_m ** 2 +
                         self._anchor_sigma(physical_name) ** 2)
        angular = (factor['ray_sigma_rad'] ** 2 +
                   self.pose_rotation_sigma_rad ** 2 +
                   self.extrinsic_rotation_sigma_rad ** 2 +
                   translational / max(distance * distance, 1e-6))
        confidence_scale = 1.0 / math.sqrt(max(0.1, factor['confidence']))
        return max(1e-5, math.sqrt(max(angular, 0.0)) * confidence_scale)

    def _ray_batch(self, rays):
        origins = np.asarray([ray['ray_origin'] for ray in rays], dtype=float)
        directions = np.asarray([ray['ray_direction'] for ray in rays], dtype=float)
        bases = np.asarray([ray['basis'] for ray in rays], dtype=float)
        confidence = np.asarray([max(0.1, ray['confidence']) for ray in rays])
        ray_variance = np.asarray([ray['ray_sigma_rad'] ** 2 for ray in rays])
        anchor_variance = np.asarray([
            self._anchor_sigma(ray['physical_class_name']) ** 2 for ray in rays])
        return {
            'origins': origins,
            'directions': directions,
            'bases': bases,
            'base_angular_variance': (ray_variance +
                self.pose_rotation_sigma_rad ** 2 +
                self.extrinsic_rotation_sigma_rad ** 2),
            'translation_variance': (anchor_variance +
                self.pose_translation_sigma_m ** 2 +
                self.extrinsic_translation_sigma_m ** 2),
            'confidence': confidence,
        }

    @staticmethod
    def _evaluate_batch(batch, position):
        delta = np.asarray(position, dtype=float)[None, :] - batch['origins']
        ranges = np.linalg.norm(delta, axis=1)
        safe_ranges = np.maximum(ranges, 1e-6)
        predicted = delta / safe_ranges[:, None]
        forward = np.einsum('ij,ij->i', delta, batch['directions']) > 0.0
        difference = predicted - batch['directions']
        error = np.einsum('nji,nj->ni', batch['bases'], difference)
        sigma = np.sqrt(np.maximum(
            1e-10,
            (batch['base_angular_variance'] +
             batch['translation_variance'] / (safe_ranges * safe_ranges)) /
            batch['confidence']))
        whitened = np.linalg.norm(error, axis=1) / sigma
        valid = forward & np.isfinite(whitened) & (ranges > 1e-6)
        log_likelihood = (-0.5 * whitened * whitened -
                           np.log(2.0 * math.pi) - 2.0 * np.log(sigma))
        log_likelihood[~valid] = -math.inf
        whitened[~valid] = math.inf
        return error, predicted, safe_ranges, sigma, whitened, valid, log_likelihood

    def _log_likelihood(self, factor, position):
        result = self._residual(factor, position)
        if result is None:
            return -math.inf, math.inf
        error, _, distance = result
        sigma = self._sigma(factor, distance)
        whitened = float(np.linalg.norm(error)) / sigma
        # Isotropic two-dimensional Gaussian in the measured ray's tangent plane.
        log_likelihood = (-0.5 * whitened * whitened -
                          math.log(2.0 * math.pi) - 2.0 * math.log(sigma))
        return log_likelihood, whitened

    def _responsibilities(self, rays, candidates, batch=None):
        if batch is None:
            batch = self._ray_batch(rays)
        count = len(candidates)
        cluster_log_prior = math.log(1.0 - self.clutter_prior) - math.log(count)
        clutter_log_prior = math.log(self.clutter_prior) - math.log(4.0 * math.pi)
        log_scores = np.empty((count + 1, len(rays)), dtype=float)
        for index, candidate in enumerate(candidates):
            *_, likelihood = self._evaluate_batch(batch, candidate['position'])
            log_scores[index] = cluster_log_prior + likelihood
        log_scores[-1].fill(clutter_log_prior)
        maximum = np.max(log_scores, axis=0)
        exponentials = np.exp(np.clip(log_scores - maximum[None, :], -700.0, 0.0))
        totals = np.sum(exponentials, axis=0)
        return exponentials[:-1] / np.maximum(totals[None, :], 1e-300)

    @staticmethod
    def _huber_cost(norm, delta):
        return 0.5 * norm * norm if norm <= delta else delta * (norm - 0.5 * delta)

    def _candidate_cost(self, batch, responsibilities, position):
        *_, whitened, valid, _ = self._evaluate_batch(batch, position)
        costs = np.where(
            whitened <= self.huber_delta,
            0.5 * whitened * whitened,
            self.huber_delta * (whitened - 0.5 * self.huber_delta))
        costs[~valid] = math.inf
        active = responsibilities > 1e-8
        if np.any(active & ~valid):
            return math.inf
        return float(np.sum(responsibilities[active] * costs[active]))

    def _optimize_position(self, rays, responsibilities, initial, batch=None):
        if batch is None:
            batch = self._ray_batch(rays)
        position = np.asarray(initial, dtype=float).copy()
        damping = 1e-3
        for _ in range(self.lm_iterations):
            error, predicted, ranges, sigma, whitened, valid, _ = \
                self._evaluate_batch(batch, position)
            active = (responsibilities > 1e-8) & valid
            if np.count_nonzero(active) < 2:
                break
            huber_weights = np.minimum(
                1.0, self.huber_delta / np.maximum(whitened, 1e-12))
            projectors = (np.eye(3)[None, :, :] -
                          predicted[:, :, None] * predicted[:, None, :])
            jacobians = np.einsum('nji,njk->nik', batch['bases'], projectors)
            jacobians /= ranges[:, None, None]
            weights = np.zeros(len(rays), dtype=float)
            weights[active] = (responsibilities[active] * huber_weights[active] /
                               (sigma[active] * sigma[active]))
            normal = np.einsum('n,nij,nik->jk', weights, jacobians, jacobians)
            gradient = np.einsum('n,nij,ni->j', weights, jacobians, error)
            diagonal = np.maximum(np.diag(normal), 1e-9)
            try:
                step = np.linalg.solve(normal + damping * np.diag(diagonal), -gradient)
            except np.linalg.LinAlgError:
                break
            if not np.all(np.isfinite(step)):
                break
            old_cost = self._candidate_cost(batch, responsibilities, position)
            candidate_position = position + step
            new_cost = self._candidate_cost(batch, responsibilities, candidate_position)
            if new_cost < old_cost:
                position = candidate_position
                damping = max(1e-7, damping * 0.3)
                if float(np.linalg.norm(step)) < 1e-5:
                    break
            else:
                damping = min(1e8, damping * 10.0)
        return position

    def _fit_candidates(self, rays, seeds):
        if not seeds:
            return [], np.zeros(len(rays), dtype=bool), 0, math.inf, 0.0
        candidates = [dict(item) for item in seeds]
        batch = self._ray_batch(rays)
        for _ in range(self.association_cycles):
            responsibilities = self._responsibilities(rays, candidates, batch)
            for index, candidate in enumerate(candidates):
                candidate['position'] = self._optimize_position(
                    rays, responsibilities[index], candidate['position'], batch)

        responsibilities = self._responsibilities(rays, candidates, batch)
        accepted = []
        for index, candidate in enumerate(candidates):
            membership = responsibilities[index]
            evaluation = self._evaluate_batch(batch, candidate['position'])
            ranges, whitened, valid = evaluation[2], evaluation[4], evaluation[5]
            support_mask = (membership > 1e-4) & valid
            effective_support = float(np.sum(membership[support_mask]))
            residuals = whitened[support_mask]
            inlier_mask = ((membership >= 0.25) & valid &
                           (whitened <= max(3.0, self.huber_delta * 1.5)))
            inlier_indices = np.flatnonzero(inlier_mask).tolist()
            if effective_support < 2.0 or len(inlier_indices) < 2:
                continue
            inlier_rays = [rays[i] for i in inlier_indices]
            if not self._has_min_parallax(inlier_rays, self.min_parallax_deg):
                continue
            normal, _ = self._information_matrix(
                rays, membership, candidate['position'], batch)
            eigenvalues = np.linalg.eigvalsh(normal)
            if (not np.all(np.isfinite(eigenvalues)) or eigenvalues[0] <= 1e-8):
                continue
            condition = float(eigenvalues[-1] / eigenvalues[0])
            if not math.isfinite(condition) or condition > 1e8:
                continue
            covariance = np.linalg.inv(normal)
            max_range = float(np.max(ranges[inlier_mask]))
            shared_floor = (
                self.pose_translation_sigma_m ** 2 +
                self.extrinsic_translation_sigma_m ** 2 +
                max_range * max_range *
                (self.pose_rotation_sigma_rad ** 2 +
                 self.extrinsic_rotation_sigma_rad ** 2))
            covariance += np.eye(3) * shared_floor
            covariance = (covariance + covariance.T) * 0.5
            candidate.update({
                'responsibilities': membership,
                'inlier_indices': inlier_indices,
                'inlier_count': len(inlier_indices),
                'effective_support': effective_support,
                'mean_residual': float(np.mean(residuals)) if residuals.size else math.inf,
                'covariance': covariance,
                'condition': condition,
                'score': (len(inlier_indices), effective_support,
                          candidate.get('seed_support', 0),
                          candidate.get('seed_score', 0.0)),
            })
            accepted.append(candidate)
        accepted.sort(key=lambda item: item['score'], reverse=True)
        deduplicated = []
        for candidate in accepted:
            if any(np.linalg.norm(candidate['position'] - old['position']) <
                   self.seed_cluster_radius_m for old in deduplicated):
                continue
            deduplicated.append(candidate)
        if not deduplicated:
            return [], np.zeros(len(rays), dtype=bool), len(rays), math.inf, 0.0
        responsibilities = self._responsibilities(rays, deduplicated, batch)
        final_whitened, final_valid = [], []
        for index, candidate in enumerate(deduplicated):
            membership = responsibilities[index]
            candidate['responsibilities'] = membership
            # Recompute diagnostics after the final soft assignment.
            *_, whitened, valid, _ = self._evaluate_batch(batch, candidate['position'])
            final_whitened.append(whitened)
            final_valid.append(valid)
            inlier_mask = ((membership >= 0.25) & valid &
                           (whitened <= max(3.0, self.huber_delta * 1.5)))
            inlier_indices = np.flatnonzero(inlier_mask).tolist()
            candidate['inlier_indices'] = inlier_indices
            candidate['inlier_count'] = len(inlier_indices)
            candidate['mean_residual'] = (float(np.mean(whitened[inlier_mask]))
                                          if inlier_indices else math.inf)
        best_membership = np.max(responsibilities, axis=0)
        best_assignment = np.argmax(responsibilities, axis=0)
        assigned_mask = best_membership >= 0.5
        clutter_count = int(np.count_nonzero(~assigned_mask))
        all_residuals = []
        for index in range(len(deduplicated)):
            selected = assigned_mask & (best_assignment == index) & final_valid[index]
            if np.any(selected):
                all_residuals.extend(final_whitened[index][selected].tolist())
        mean_residual = float(np.mean(all_residuals)) if all_residuals else math.inf
        return deduplicated, assigned_mask, clutter_count, mean_residual, float(np.mean(best_membership))

    @staticmethod
    def _has_min_parallax(rays, min_parallax_deg):
        if len(rays) < 2:
            return False
        directions = np.asarray([ray['ray_direction'] for ray in rays], dtype=float)
        threshold_dot = math.cos(math.radians(float(min_parallax_deg)))
        # Stop as soon as one pair has enough angle. This avoids allocating the
        # full N-by-N dot-product matrix for a large observation pool.
        for index in range(len(directions) - 1):
            if np.any(np.abs(directions[index + 1:] @ directions[index]) <=
                      threshold_dot):
                return True
        return False

    def _information_matrix(self, rays, memberships, position, batch=None):
        if batch is None:
            batch = self._ray_batch(rays)
        error, predicted, ranges, sigma, whitened, valid, _ = \
            self._evaluate_batch(batch, position)
        active = (memberships > 1e-8) & valid
        if not np.any(active):
            return np.zeros((3, 3), dtype=float), 0.0
        huber_weights = np.minimum(
            1.0, self.huber_delta / np.maximum(whitened, 1e-12))
        projectors = (np.eye(3)[None, :, :] -
                      predicted[:, :, None] * predicted[:, None, :])
        jacobians = np.einsum('nji,njk->nik', batch['bases'], projectors)
        jacobians /= ranges[:, None, None]
        weights = np.zeros(len(rays), dtype=float)
        weights[active] = (memberships[active] * huber_weights[active] /
                           (sigma[active] * sigma[active]))
        normal = np.einsum('n,nij,nik->jk', weights, jacobians, jacobians)
        residual_sum = float(np.sum(memberships[active] *
                                    whitened[active] * whitened[active]))
        return normal, residual_sum

    def _max_instances(self, physical_name, candidates):
        multi = any(ray['multi_instance'] for ray in candidates)
        if not multi:
            return 1, False
        if physical_name == 'gate':
            return 4, True
        if physical_name == 'guide_line':
            return 6, True
        return 1, True

    def _refresh_pool_diagnostics(self, rays, clusters):
        if not clusters:
            return len(rays), math.inf
        batch = self._ray_batch(rays)
        responsibilities = self._responsibilities(rays, clusters, batch)
        whitened_by_cluster, valid_by_cluster = [], []
        for index, cluster in enumerate(clusters):
            membership = responsibilities[index]
            evaluation = self._evaluate_batch(batch, cluster['position'])
            ranges, whitened, valid = evaluation[2], evaluation[4], evaluation[5]
            whitened_by_cluster.append(whitened)
            valid_by_cluster.append(valid)
            normal, _ = self._information_matrix(
                rays, membership, cluster['position'], batch)
            eigenvalues = np.linalg.eigvalsh(normal)
            if eigenvalues[0] > 1e-8 and np.all(np.isfinite(eigenvalues)):
                cluster['covariance'] = np.linalg.inv(normal)
                supported = (membership >= 0.25) & valid
                max_range = float(np.max(ranges[supported])) if np.any(supported) else 0.0
                shared_floor = (
                    self.pose_translation_sigma_m ** 2 +
                    self.extrinsic_translation_sigma_m ** 2 +
                    max_range * max_range *
                    (self.pose_rotation_sigma_rad ** 2 +
                     self.extrinsic_rotation_sigma_rad ** 2))
                cluster['covariance'] += np.eye(3) * shared_floor
                cluster['condition'] = float(eigenvalues[-1] / eigenvalues[0])
            residual_mask = (membership >= 0.25) & valid
            inlier_mask = (residual_mask &
                           (whitened <= max(3.0, self.huber_delta * 1.5)))
            inliers = np.flatnonzero(inlier_mask).tolist()
            cluster['responsibilities'] = membership
            cluster['inlier_indices'] = inliers
            cluster['inlier_count'] = len(inliers)
            cluster['effective_support'] = float(np.sum(membership))
            cluster['mean_residual'] = (float(np.mean(whitened[residual_mask]))
                                        if np.any(residual_mask) else math.inf)
        best_membership = np.max(responsibilities, axis=0)
        best_assignment = np.argmax(responsibilities, axis=0)
        assigned = best_membership >= 0.5
        residuals = []
        for index in range(len(clusters)):
            selected = assigned & (best_assignment == index) & valid_by_cluster[index]
            if np.any(selected):
                residuals.extend(whitened_by_cluster[index][selected].tolist())
        mean_residual = float(np.mean(residuals)) if residuals else math.inf
        return int(np.count_nonzero(~assigned)), mean_residual

    def _rebuild_pool(self, key, rays, now):
        physical_name = key[1]
        seeds = self._seed_candidates(rays)
        # Retain hypotheses supported by older pooled rays after they leave
        # the recent subset used for pair seeding.
        for track_id in self._pool_track_ids.get(key, ()):
            state = self._tracks.get(track_id)
            if state is None or state.position is None:
                continue
            position = np.asarray(state.position, dtype=float)
            if any(np.linalg.norm(position - seed['position']) <
                   self.seed_cluster_radius_m for seed in seeds):
                continue
            seeds.append({'position': position, 'seed_score': 0.0,
                          'seed_support': 0})
        clusters, _, clutter_count, mean_residual, _ = self._fit_candidates(rays, seeds)
        max_instances, multi_instance = self._max_instances(physical_name, rays)
        clusters = clusters[:max_instances]
        clutter_count, mean_residual = self._refresh_pool_diagnostics(rays, clusters)
        track_ids = list(self._pool_track_ids.get(key, ()))
        states = [self._tracks[track_id] for track_id in track_ids
                  if track_id in self._tracks]
        costs = []
        for cluster in clusters:
            row = []
            for state in states:
                distance = float(np.linalg.norm(
                    np.asarray(cluster['position']) - np.asarray(state.position)))
                row.append(distance if distance <= self.max_instance_association_m else None)
            costs.append(row)
        assignment = {row: column for row, column, _ in
                      minimum_cost_assignment(costs, self.max_instance_association_m)}
        if max_instances == 1 and clusters and states:
            # The view and physical class identify a singleton. Its estimate
            # may move far when later pooled evidence corrects an early error.
            assignment = {0: 0}
        new_pool_track_ids = set(track_ids)
        reserved_track_ids = {states[index].track_id for index in assignment.values()}
        replaceable_states = [state for state in states
                              if state.track_id not in reserved_track_ids]
        for cluster_index, cluster in enumerate(clusters):
            inlier_indices = cluster['inlier_indices']
            if len(inlier_indices) < 2:
                continue
            if cluster_index in assignment:
                state = states[assignment[cluster_index]]
            else:
                if len(new_pool_track_ids) >= max_instances:
                    if not replaceable_states:
                        continue
                    # A supported new cluster replaces an old track with no
                    # matching cluster when the instance budget is full.
                    old = min(replaceable_states, key=lambda item: (
                        item.measurement_count,
                        -sum(item.covariance[index] for index in (0, 4, 8)),
                        item.last_ns))
                    replaceable_states.remove(old)
                    new_pool_track_ids.discard(old.track_id)
                    self._tracks.pop(old.track_id, None)
                newest = max((rays[index] for index in inlier_indices),
                             key=lambda ray: ray['arrival_ns'])
                state = TrackState(
                    track_id=self._next_id,
                    class_id=newest['class_id'],
                    class_name=newest['class_name'],
                    physical_class_name=physical_name,
                    view=key[0],
                    multi_instance=multi_instance,
                    created_ns=now,
                    last_ns=now)
                self._next_id += 1
                self._tracks[state.track_id] = state
                new_pool_track_ids.add(state.track_id)
            inliers = [rays[index] for index in inlier_indices]
            newest = max(inliers, key=lambda ray: ray['arrival_ns'])
            state.position = tuple(float(value) for value in cluster['position'])
            state.covariance = [float(value) for value in
                                cluster['covariance'].reshape(9)]
            state.class_id = newest['class_id']
            state.class_name = newest['class_name']
            state.confidence = float(np.mean([ray['confidence'] for ray in inliers]))
            state.measurement_count = len({ray['observation_id'] for ray in inliers})
            state.last_ns = newest['arrival_ns']
            state.last_stamp = newest['stamp']
            state.last_observation_id = newest['observation_id']
        self._pool_track_ids[key] = new_pool_track_ids
        self._log_pool(key, len(rays), clusters, clutter_count, mean_residual, now)

    def _log_pool(self, key, observation_count, clusters, clutter_count,
                  mean_residual, now):
        last_log = self._last_log_ns.get(key, 0)
        if now - last_log < 1_000_000_000:
            return
        self._last_log_ns[key] = now
        inliers = sum(cluster.get('inlier_count', 0) for cluster in clusters)
        covariances = [float(np.trace(cluster['covariance'])) for cluster in clusters]
        conditions = [float(cluster['condition']) for cluster in clusters]
        covariance_text = (','.join('{:.4g}'.format(value) for value in covariances)
                           if covariances else 'none')
        condition_text = (','.join('{:.4g}'.format(value) for value in conditions)
                          if conditions else 'none')
        logger = self.node.get_logger()
        info = getattr(logger, 'info', None)
        if info is not None:
            info('ray_pool view={} class={} observations={} clusters={} inliers={} '
                 'mean_norm_residual={:.3f} covariance_trace_m2=[{}] '
                 'condition=[{}] clutter={}'.format(
                     key[0], key[1], observation_count, len(clusters), inliers,
                     mean_residual, covariance_text, condition_text, clutter_count))

    def _rebuild_dirty(self, now):
        with self._lock:
            dirty = tuple(self._dirty_pools)
            self._dirty_pools.clear()
            for key in dirty:
                self._rebuild_pool(key, tuple(self._pools.get(key, ())), now)

    def _publish(self):
        if not self._origin_ready:
            return
        now = time.monotonic_ns()
        self._rebuild_dirty(now)
        output = ObjectTrackArray()
        output.header.stamp = self.node.get_clock().now().to_msg()
        output.header.frame_id = self.world_frame
        with self._lock:
            states = tuple(self._tracks.values())
        for state in states:
            if state.position is None:
                continue
            track = ObjectTrack()
            track.track_id = state.track_id
            track.last_observation_id = state.last_observation_id
            track.class_id = state.class_id
            track.class_name = state.class_name
            track.physical_class_name = state.physical_class_name
            track.estimate_source = state.view
            track.confidence = state.confidence
            track.measurement_count = state.measurement_count
            track.age_sec = max(0.0, (now - state.created_ns) / 1e9)
            track.last_measurement_stamp = state.last_stamp
            track.world_x, track.world_y, track.world_z = state.position
            track.position_covariance = state.covariance
            covariance_trace = sum(state.covariance[index] for index in (0, 4, 8))
            if (track.measurement_count >= 2 and
                    covariance_trace <= self.stable_covariance_trace_m2):
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
