"""Robust static point estimation from odom-frame points and bearing rays."""

from __future__ import annotations

import math

import numpy as np


def _unit(vector):
    vector = np.asarray(vector, dtype=float).reshape(3)
    norm = float(np.linalg.norm(vector))
    if not math.isfinite(norm) or norm <= 1e-12:
        return None
    return vector / norm


def ray_pair_geometry(first, second):
    """Return closest points, forward ranges, separation, and angle for rays."""
    d1 = _unit(first['ray_direction'])
    d2 = _unit(second['ray_direction'])
    if d1 is None or d2 is None:
        return None
    o1 = np.asarray(first['ray_origin'], dtype=float).reshape(3)
    o2 = np.asarray(second['ray_origin'], dtype=float).reshape(3)
    w0 = o1 - o2
    dot = float(np.dot(d1, d2))
    denominator = max(0.0, 1.0 - dot * dot)
    if denominator < 1e-10:
        return None
    d1w = float(np.dot(d1, w0))
    d2w = float(np.dot(d2, w0))
    t1 = (-d1w + dot * d2w) / denominator
    t2 = dot * t1 + d2w
    p1, p2 = o1 + t1 * d1, o2 + t2 * d2
    angle = math.acos(max(-1.0, min(1.0, dot)))
    return p1, p2, t1, t2, float(np.linalg.norm(p1 - p2)), angle


def _ray_intersection_seed(rays):
    normal = np.zeros((3, 3), dtype=float)
    rhs = np.zeros(3, dtype=float)
    for factor in rays:
        direction = _unit(factor['ray_direction'])
        if direction is None:
            continue
        projector = np.eye(3) - np.outer(direction, direction)
        origin = np.asarray(factor['ray_origin'], dtype=float).reshape(3)
        normal += projector
        rhs += projector @ origin
    eigenvalues = np.linalg.eigvalsh(normal)
    if eigenvalues[0] <= 1e-8 or eigenvalues[-1] / eigenvalues[0] > 1e8:
        return None
    return np.linalg.solve(normal, rhs)


def _parallax_angle(rays):
    directions = [_unit(item['ray_direction']) for item in rays]
    directions = [direction for direction in directions if direction is not None]
    largest = 0.0
    for index, first in enumerate(directions):
        for second in directions[index + 1:]:
            dot = max(-1.0, min(1.0, float(np.dot(first, second))))
            largest = max(largest, math.acos(dot))
    return largest


def fit_static_landmark(factors, *, min_parallax_deg=5.0,
                        huber_delta=2.5, pose_translation_sigma_m=0.03,
                        initial_position=None,
                        pose_rotation_sigma_deg=1.0,
                        extrinsic_translation_sigma_m=0.005,
                        extrinsic_rotation_sigma_deg=0.5):
    """Fit one fixed odom-frame point; return (position, covariance) or None.

    Each factor is a dict containing either ``position`` plus a flattened
    3x3 ``covariance``, or ``ray_origin``, ``ray_direction`` and optional
    ``ray_sigma_rad``.  Point factors take precedence over a ray carried by
    the same measurement, preventing stereo/down-projection double counting.
    """
    points, rays = [], []
    for factor in factors:
        if factor.get('position') is not None:
            points.append(factor)
        elif factor.get('ray_origin') is not None:
            if _unit(factor.get('ray_direction', ())) is not None:
                rays.append(factor)
    if not points:
        if len(rays) < 2 or math.degrees(_parallax_angle(rays)) < min_parallax_deg:
            return None
        estimate = _ray_intersection_seed(rays)
        if estimate is None:
            return None
        if any(float(np.dot(estimate - np.asarray(ray['ray_origin'], dtype=float),
                            _unit(ray['ray_direction']))) <= 0.0 for ray in rays):
            return None
    else:
        weights, values = [], []
        for factor in points:
            covariance = np.asarray(factor.get('covariance', np.eye(3)),
                                    dtype=float).reshape(3, 3)
            covariance = (covariance + covariance.T) * 0.5
            covariance += np.eye(3) * 1e-9
            precision = np.linalg.pinv(covariance)
            weights.append(max(1e-12, float(np.trace(precision))))
            values.append(np.asarray(factor['position'], dtype=float).reshape(3))
        estimate = (np.asarray(initial_position, dtype=float).reshape(3)
                    if initial_position is not None else
                    np.average(np.asarray(values), axis=0, weights=np.asarray(weights)))

    pose_angle = math.radians(float(pose_rotation_sigma_deg))
    extrinsic_angle = math.radians(float(extrinsic_rotation_sigma_deg))
    systematic_angle = math.hypot(pose_angle, extrinsic_angle)
    final_normal = None
    for _ in range(12):
        normal = np.zeros((3, 3), dtype=float)
        rhs = np.zeros(3, dtype=float)
        for factor in points:
            value = np.asarray(factor['position'], dtype=float).reshape(3)
            covariance = np.asarray(factor.get('covariance', np.eye(3)),
                                    dtype=float).reshape(3, 3)
            covariance = (covariance + covariance.T) * 0.5 + np.eye(3) * 1e-9
            precision = np.linalg.pinv(covariance)
            residual = value - estimate
            norm = math.sqrt(max(0.0, float(residual @ precision @ residual)))
            robust_weight = min(1.0, float(huber_delta) / max(norm, 1e-12))
            normal += robust_weight * precision
            rhs += robust_weight * precision @ value
        for factor in rays:
            direction = _unit(factor['ray_direction'])
            origin = np.asarray(factor['ray_origin'], dtype=float).reshape(3)
            # A bearing constrains a forward half-ray, not an infinite line.
            if float(np.dot(estimate - origin, direction)) <= 0.0:
                projector = np.eye(3)
            else:
                projector = np.eye(3) - np.outer(direction, direction)
            distance = max(0.1, float(np.linalg.norm(estimate - origin)))
            pixel_sigma = max(0.0, float(factor.get('ray_sigma_rad', 0.002)))
            sigma = math.sqrt(
                (distance * math.hypot(pixel_sigma, systematic_angle)) ** 2 +
                float(pose_translation_sigma_m) ** 2 +
                float(extrinsic_translation_sigma_m) ** 2)
            sigma = max(0.005, sigma)
            residual = projector @ (estimate - origin)
            norm = float(np.linalg.norm(residual)) / sigma
            robust_weight = min(1.0, float(huber_delta) / max(norm, 1e-12))
            precision = robust_weight / (sigma * sigma)
            normal += precision * projector
            rhs += precision * (projector @ origin)
        eigenvalues = np.linalg.eigvalsh(normal)
        if eigenvalues[0] <= 1e-12 or eigenvalues[-1] / eigenvalues[0] > 1e10:
            if points:
                # A point factor should make this full-rank.  Bad covariance
                # input is rejected instead of emitting a false precise state.
                return None
            return None
        candidate = np.linalg.solve(normal, rhs)
        final_normal = normal
        if float(np.linalg.norm(candidate - estimate)) < 1e-7:
            estimate = candidate
            break
        estimate = candidate

    if not np.all(np.isfinite(estimate)):
        return None
    # Never publish a ray-only 3D estimate behind a supporting observation.
    if not points and any(
            float(np.dot(estimate - np.asarray(ray['ray_origin'], dtype=float),
                         _unit(ray['ray_direction']))) <= 0.0
            for ray in rays):
        return None
    covariance = np.linalg.pinv(final_normal)
    ranges = [float(np.linalg.norm(estimate - np.asarray(
        factor.get('ray_origin', factor.get('position')), dtype=float)))
              for factor in factors]
    max_range = max(ranges, default=0.0)
    floor = math.sqrt(
        float(pose_translation_sigma_m) ** 2 +
        float(extrinsic_translation_sigma_m) ** 2 +
        (max_range * systematic_angle) ** 2)
    covariance += np.eye(3) * floor * floor
    covariance = (covariance + covariance.T) * 0.5
    return tuple(float(value) for value in estimate), [
        float(value) for value in covariance.reshape(9)]
