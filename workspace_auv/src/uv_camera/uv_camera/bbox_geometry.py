"""Bounding-box geometry primitives for the front multi-view estimator.

The detector only gives a rectangle, so the front estimator must compare a
predicted rectangle with the complete detector measurement instead of
pretending that the rectangle centre is a measured 3-D point.  This module is
ROS independent on purpose; it can therefore be exercised with synthetic
camera callbacks in unit tests and reused by offline log tools.

The public state convention is NED/world coordinates.  A camera context is a
small object exposing ``project(point)`` and ``depth(point)`` callbacks plus
``fx`` and ``fy``.  The localizer supplies those callbacks from its captured
pose and calibration snapshot.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Iterable

import numpy as np


EPS = 1e-9


def _finite(value, size: int) -> np.ndarray | None:
    try:
        result = np.asarray(value, dtype=np.float64).reshape(-1)
    except (TypeError, ValueError):
        return None
    if result.size != size or not np.all(np.isfinite(result)):
        return None
    return result


def normalize_yaw_pi(yaw: float) -> float:
    """Normalize an unoriented frame yaw to ``[-pi/2, pi/2)``.

    A rectangular gate has the same image geometry after a 180 degree turn.
    Keeping the state in a pi-periodic interval avoids artificial jumps when
    the optimizer crosses the equivalent representation.
    """

    value = (float(yaw) + 0.5 * math.pi) % math.pi - 0.5 * math.pi
    # Keep the upper endpoint out of the interval despite floating point
    # round-off at exactly pi/2.
    return value if value < 0.5 * math.pi else value - math.pi


def yaw_difference_pi(first: float, second: float) -> float:
    """Return the shortest difference of two unoriented frame yaws."""

    return normalize_yaw_pi(float(first) - float(second))


def bbox_measurement(
        bbox: Iterable[float],
        center: Iterable[float] | None = None) -> np.ndarray | None:
    """Convert ``x1,y1,x2,y2`` to ``u,v,log(width),log(height)``."""

    values = _finite(bbox, 4)
    if values is None:
        return None
    x1, y1, x2, y2 = values
    width = x2 - x1
    height = y2 - y1
    if width <= EPS or height <= EPS:
        return None
    if center is None:
        centre = np.array([(x1 + x2) * 0.5, (y1 + y2) * 0.5])
    else:
        centre = _finite(center, 2)
        if centre is None:
            return None
    return np.array([centre[0], centre[1], math.log(width), math.log(height)],
                    dtype=np.float64)


def bbox_measurement_covariance(
        bbox: Iterable[float],
        pixel_sigma: float | Iterable[float] = 2.0,
        minimum_variance: float = 1e-8) -> np.ndarray:
    """Approximate diagonal covariance in bbox-measurement coordinates.

    Width and height are differences of two pixel edges, hence their log
    variances use the corresponding ``sqrt(2)`` propagation.  The function
    deliberately returns a conservative diagonal matrix: detector edge
    errors are not independent in practice and any common-mode error belongs
    in the localizer's geometry covariance floor.
    """

    values = _finite(bbox, 4)
    if values is None:
        return np.eye(4, dtype=np.float64) * 1e6
    width = max(float(values[2] - values[0]), EPS)
    height = max(float(values[3] - values[1]), EPS)
    sigma = np.asarray(pixel_sigma, dtype=np.float64).reshape(-1)
    if sigma.size == 1:
        sigma_u = sigma_v = float(sigma[0])
    elif sigma.size == 2:
        sigma_u, sigma_v = map(float, sigma)
    else:
        raise ValueError("pixel_sigma must contain one or two values")
    sigma_u = max(abs(sigma_u), math.sqrt(minimum_variance))
    sigma_v = max(abs(sigma_v), math.sqrt(minimum_variance))
    variances = np.array([
        sigma_u * sigma_u,
        sigma_v * sigma_v,
        2.0 * sigma_u * sigma_u / (width * width),
        2.0 * sigma_v * sigma_v / (height * height),
    ], dtype=np.float64)
    return np.diag(np.maximum(variances, minimum_variance))


def bbox_truncation_mask(
        bbox: Iterable[float], width: float, height: float,
        margin: float = 0.0) -> np.ndarray:
    """Return valid residual dimensions for a possibly truncated bbox.

    The centre and size are not reliable in the axis that touches the image
    boundary.  The mask order is ``u,v,log(width),log(height)``.  A caller may
    additionally mask dimensions when an upstream segmentation/visibility
    test knows more than the rectangle itself.
    """

    values = _finite(bbox, 4)
    if values is None or width <= 0.0 or height <= 0.0:
        return np.zeros(4, dtype=bool)
    x1, y1, x2, y2 = values
    clipped_x = x1 <= margin or x2 >= float(width) - margin
    clipped_y = y1 <= margin or y2 >= float(height) - margin
    return np.array([not clipped_x, not clipped_y, not clipped_x, not clipped_y],
                    dtype=bool)


@dataclass(frozen=True)
class CameraContext:
    """Capture-time camera operations needed by a geometry model."""

    project: Callable[[np.ndarray], np.ndarray | None]
    depth: Callable[[np.ndarray], float]
    fx: float
    fy: float


def _projected_bbox(points: np.ndarray, camera: CameraContext) -> np.ndarray | None:
    pixels = []
    for point in np.asarray(points, dtype=np.float64).reshape(-1, 3):
        pixel = camera.project(point)
        if pixel is None:
            return None
        pixel = _finite(pixel, 2)
        if pixel is None:
            return None
        pixels.append(pixel)
    if not pixels:
        return None
    pixels = np.asarray(pixels, dtype=np.float64)
    return np.array([
        np.min(pixels[:, 0]), np.min(pixels[:, 1]),
        np.max(pixels[:, 0]), np.max(pixels[:, 1]),
    ], dtype=np.float64)


class GeometryModel:
    """Base class for a known-shape bbox projection model."""

    name = "geometry"
    state_size = 3

    def project_bbox(self, state: Iterable[float],
                     camera: CameraContext) -> np.ndarray | None:
        raise NotImplementedError

    def normalize_state(self, state: Iterable[float]) -> np.ndarray:
        value = _finite(state, self.state_size)
        if value is None:
            raise ValueError("model state must be finite")
        return value


@dataclass(frozen=True)
class SphereModel(GeometryModel):
    """Perspective bbox model for a sphere with a known radius."""

    radius: float
    name: str = "sphere"
    state_size: int = 3

    def __post_init__(self):
        if not np.isfinite(self.radius) or self.radius <= 0.0:
            raise ValueError("sphere radius must be positive")

    def project_bbox(self, state: Iterable[float],
                     camera: CameraContext) -> np.ndarray | None:
        centre = _finite(state, 3)
        if centre is None:
            return None
        pixel = _finite(camera.project(centre), 2)
        if pixel is None:
            return None
        depth = float(camera.depth(centre))
        if not np.isfinite(depth) or depth <= self.radius + EPS:
            return None
        half_width = abs(float(camera.fx)) * self.radius / depth
        half_height = abs(float(camera.fy)) * self.radius / depth
        return np.array([
            pixel[0] - half_width, pixel[1] - half_height,
            pixel[0] + half_width, pixel[1] + half_height,
        ], dtype=np.float64)


@dataclass(frozen=True)
class GateModel(GeometryModel):
    """Four-corner planar frame model with pi-periodic yaw.

    State is ``[N,E,D,yaw]`` and yaw is the world direction of the gate's
    horizontal span.  The model uses the four visible rectangle corners as a
    conservative envelope; centerline/segmentation anchors can replace the
    observed rectangle centre without changing this geometry.
    """

    width: float
    height: float
    name: str = "gate"
    state_size: int = 4

    def __post_init__(self):
        if (not np.isfinite(self.width) or self.width <= 0.0
                or not np.isfinite(self.height) or self.height <= 0.0):
            raise ValueError("gate dimensions must be positive")

    def normalize_state(self, state: Iterable[float]) -> np.ndarray:
        value = super().normalize_state(state)
        value[3] = normalize_yaw_pi(value[3])
        return value

    def project_bbox(self, state: Iterable[float],
                     camera: CameraContext) -> np.ndarray | None:
        value = _finite(state, 4)
        if value is None:
            return None
        yaw = float(value[3])
        axis = np.array([math.cos(yaw), math.sin(yaw), 0.0])
        half_width = 0.5 * self.width
        half_height = 0.5 * self.height
        points = np.array([
            value[:3] + axis * sx * half_width + np.array([0.0, 0.0, sz * half_height])
            for sx in (-1.0, 1.0)
            for sz in (-1.0, 1.0)
        ], dtype=np.float64)
        return _projected_bbox(points, camera)


@dataclass(frozen=True)
class GatePositionModel(GeometryModel):
    """V2 gate model with position-only state ``[N, E, D]``.

    The 2026 rules describe an approximately front-facing, vertical 70 cm by
    50 cm gate.  Its yaw is intentionally not estimated: detector padding and
    partial visibility make yaw weakly observable and allow it to compensate
    for depth.  ``GateModel`` remains available for old offline callers.
    """

    width: float
    height: float
    name: str = "gate"
    state_size: int = 3

    def __post_init__(self):
        if (not np.isfinite(self.width) or self.width <= 0.0
                or not np.isfinite(self.height) or self.height <= 0.0):
            raise ValueError("gate dimensions must be positive")

    def project_bbox(self, state: Iterable[float],
                     camera: CameraContext) -> np.ndarray | None:
        centre = _finite(state, 3)
        if centre is None:
            return None
        half_width = 0.5 * self.width
        half_height = 0.5 * self.height
        # Gate span is fixed in world X for V2; no yaw is fitted.
        points = np.array([
            centre + np.array([sx * half_width, 0.0, sz * half_height])
            for sx in (-1.0, 1.0) for sz in (-1.0, 1.0)
        ], dtype=np.float64)
        return _projected_bbox(points, camera)


@dataclass(frozen=True)
class CuboidModel(GeometryModel):
    """Known-dimension cuboid envelope with a pi-periodic horizontal yaw."""

    width: float
    depth_size: float
    height: float
    name: str = "cuboid"
    state_size: int = 4

    def __post_init__(self):
        if any(not np.isfinite(value) or value <= 0.0 for value in (
                self.width, self.depth_size, self.height)):
            raise ValueError("cuboid dimensions must be positive")

    def normalize_state(self, state: Iterable[float]) -> np.ndarray:
        value = super().normalize_state(state)
        value[3] = normalize_yaw_pi(value[3])
        return value

    def project_bbox(self, state: Iterable[float],
                     camera: CameraContext) -> np.ndarray | None:
        value = _finite(state, 4)
        if value is None:
            return None
        yaw = float(value[3])
        axis = np.array([math.cos(yaw), math.sin(yaw), 0.0])
        side = np.array([-math.sin(yaw), math.cos(yaw), 0.0])
        half_width = 0.5 * self.width
        half_depth = 0.5 * self.depth_size
        half_height = 0.5 * self.height
        points = np.array([
            value[:3] + axis * sx * half_width + side * sy * half_depth
            + np.array([0.0, 0.0, sz * half_height])
            for sx in (-1.0, 1.0)
            for sy in (-1.0, 1.0)
            for sz in (-1.0, 1.0)
        ], dtype=np.float64)
        return _projected_bbox(points, camera)


@dataclass(frozen=True)
class FrameModel(CuboidModel):
    """Named cuboid variant for open frames/baskets with outer dimensions."""

    name: str = "frame"


def bbox_residual(
        observed_bbox: Iterable[float],
        predicted_bbox: Iterable[float],
        covariance: np.ndarray,
        feature_center: Iterable[float] | None = None,
        valid_mask: Iterable[bool] | None = None) -> tuple[np.ndarray, np.ndarray] | None:
    """Return raw and whitened bbox residuals.

    The residual is ``observed - predicted`` in the four-dimensional
    measurement space.  Invalid/truncated dimensions are removed before
    whitening, so a half-visible gate is not forced to explain an invented
    width or height.
    """

    observed = bbox_measurement(observed_bbox, feature_center)
    predicted = bbox_measurement(predicted_bbox)
    if observed is None or predicted is None:
        return None
    covariance = np.asarray(covariance, dtype=np.float64)
    if covariance.shape != (4, 4) or not np.all(np.isfinite(covariance)):
        return None
    if valid_mask is None:
        mask = np.ones(4, dtype=bool)
    else:
        mask = np.asarray(valid_mask, dtype=bool).reshape(-1)
        if mask.size != 4:
            raise ValueError("valid_mask must contain four values")
    raw = observed - predicted
    if not np.any(mask):
        return np.zeros(0, dtype=np.float64), np.zeros(0, dtype=np.float64)
    sub_covariance = covariance[np.ix_(mask, mask)]
    sub_covariance = 0.5 * (sub_covariance + sub_covariance.T)
    eigenvalues, eigenvectors = np.linalg.eigh(sub_covariance)
    eigenvalues = np.maximum(eigenvalues, 1e-10)
    inverse_sqrt = (eigenvectors * (1.0 / np.sqrt(eigenvalues))) @ eigenvectors.T
    return raw[mask], inverse_sqrt @ raw[mask]


def huber_weight(whitened_norm: float, delta: float = 2.5) -> float:
    """Return the standard scalar Huber weight for an already-whitened norm."""

    norm = max(float(whitened_norm), 0.0)
    threshold = max(float(delta), EPS)
    return 1.0 if norm <= threshold else threshold / max(norm, EPS)


def exclusive_assignments(
        cost_matrix: np.ndarray,
        clutter_cost: float,
        groups: Iterable[object] | None = None,
        max_cost: float | None = None) -> tuple[np.ndarray, np.ndarray]:
    """Assign observations to hypotheses with an explicit clutter option.

    Each observation receives at most one hypothesis.  Within one group (for
    example, one camera frame), each hypothesis is also used at most once;
    different cameras/frames may update the same physical hypothesis.  This
    small branch-and-bound solver is intentional: detector counts per frame
    are small, and it avoids adding SciPy as a runtime dependency to the ROS
    package.  The return values are ``assignment`` (``-1`` means clutter) and
    the selected costs.
    """

    costs = np.asarray(cost_matrix, dtype=np.float64)
    if costs.ndim != 2:
        raise ValueError("cost_matrix must be two-dimensional")
    observation_count, model_count = costs.shape
    if groups is None:
        group_values = [0] * observation_count
    else:
        group_values = list(groups)
        if len(group_values) != observation_count:
            raise ValueError("groups must have one entry per observation")
    clutter = float(clutter_cost)
    if not np.isfinite(clutter) or clutter < 0.0:
        raise ValueError("clutter_cost must be finite and non-negative")
    threshold = float("inf") if max_cost is None else float(max_cost)

    assignment = np.full(observation_count, -1, dtype=np.int64)
    selected_cost = np.full(observation_count, clutter, dtype=np.float64)

    # The exclusivity constraint is local to a camera frame, so solve each
    # group independently.  This keeps a 300-observation sliding window
    # linear in the number of frames instead of branching over the whole
    # window.  The detector normally contributes only one or two objects per
    # group, for which the exact search is tiny.
    grouped: dict[object, list[int]] = {}
    for index, group in enumerate(group_values):
        grouped.setdefault(group, []).append(index)

    for group_indices in grouped.values():
        order = sorted(group_indices, key=lambda i: (
            int(np.count_nonzero(np.isfinite(costs[i])
                                & (costs[i] <= threshold))), i))
        local_assignment = {}
        local_costs = {}
        best_total = [clutter * len(order)]
        best_assignment = {}
        best_costs = {}

        def search(depth: int, total: float, used: set[int]):
            if total >= best_total[0] - 1e-12:
                return
            if depth == len(order):
                best_total[0] = total
                best_assignment.update(local_assignment)
                best_costs.update(local_costs)
                return
            index = order[depth]
            options = [
                (float(costs[index, model]), model)
                for model in range(model_count)
                if model not in used
                and np.isfinite(costs[index, model])
                and float(costs[index, model]) <= threshold
                and float(costs[index, model]) < clutter
            ]
            options.sort(key=lambda item: item[0])
            for cost, model in options + [(clutter, -1)]:
                local_assignment[index] = model
                local_costs[index] = cost
                if model >= 0:
                    used.add(model)
                search(depth + 1, total + cost, used)
                if model >= 0:
                    used.remove(model)
                local_assignment.pop(index, None)
                local_costs.pop(index, None)

        search(0, 0.0, set())
        for index in group_indices:
            assignment[index] = int(best_assignment.get(index, -1))
            selected_cost[index] = float(best_costs.get(index, clutter))
    return assignment, selected_cost
