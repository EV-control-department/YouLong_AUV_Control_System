import math
from types import SimpleNamespace

import numpy as np
import pytest

from uv_perception.localization.geometry import (
    bbox_is_localizable,
    minimum_cost_assignment,
)
from uv_perception.tracking.static_landmarks import fit_static_landmark


def _box(x1=40.0, y1=40.0, x2=120.0, y2=120.0):
    return SimpleNamespace(bbox_x1=x1, bbox_y1=y1, bbox_x2=x2, bbox_y2=y2)


@pytest.mark.parametrize('box', [
    _box(x1=19.2),
    _box(y1=19.2),
    _box(x2=1280.0 - 19.2),
    _box(y2=960.0 - 19.2),
])
def test_bbox_safety_margin_rejects_each_image_edge(box):
    assert not bbox_is_localizable(box, 1280, 960)


def test_bbox_safety_margin_accepts_interior_and_rejects_bad_geometry():
    assert bbox_is_localizable(_box(20.0, 20.0, 1260.0, 940.0), 1280, 960)
    assert not bbox_is_localizable(_box(-1.0, 40.0, 120.0, 120.0), 1280, 960)
    assert not bbox_is_localizable(_box(80.0, 80.0, 40.0, 120.0), 1280, 960)
    assert not bbox_is_localizable(_box(float('nan'), 40.0, 120.0, 120.0),
                                   1280, 960)
    assert not bbox_is_localizable(_box(), 0, 960)


def test_assignment_is_global_and_one_to_one():
    costs = [[0.10, 0.20], [0.15, 0.90]]
    assigned = minimum_cost_assignment(costs, unmatched_cost=1.0)
    assert {(row, column) for row, column, _ in assigned} == {(0, 1), (1, 0)}


def _ray(origin, target, sigma=0.001):
    direction = np.asarray(target, dtype=float) - np.asarray(origin, dtype=float)
    direction /= np.linalg.norm(direction)
    return {
        'ray_origin': tuple(origin),
        'ray_direction': tuple(direction),
        'ray_sigma_rad': sigma,
    }


def test_multi_pose_bearings_recover_a_static_point():
    target = (0.7, -0.3, 3.2)
    factors = [
        _ray((-1.0, 0.0, 0.0), target),
        _ray((1.0, 0.2, 0.0), target),
        _ray((0.0, -1.0, 0.4), target),
    ]
    fitted = fit_static_landmark(factors)
    assert fitted is not None
    assert np.linalg.norm(np.asarray(fitted[0]) - np.asarray(target)) < 1e-5


def test_ray_only_fit_never_returns_a_behind_solution():
    origins = np.asarray([
        (-1.2483551, -0.5080917, 2.4426831),
        (1.8752378, 1.9296203, 0.7868992),
        (-2.9507171, -1.2832017, 0.0107964),
        (2.3907285, -0.8173294, 0.3033568),
        (2.0912690, -0.9515610, 1.4884490),
        (0.5701633, 0.1519914, 0.7787455),
    ])
    directions = np.asarray([
        (0.8204003, -0.2173343, -0.5288754),
        (0.2050165, -0.6839261, -0.7001524),
        (0.9668008, 0.1283143, -0.2209787),
        (-0.0731836, -0.1722494, -0.9823310),
        (-0.3060704, -0.7258147, -0.6160470),
        (0.5582609, -0.8016865, -0.2136433),
    ])
    factors = [
        {'ray_origin': tuple(origin), 'ray_direction': tuple(direction),
         'ray_sigma_rad': 0.1221276}
        for origin, direction in zip(origins, directions)
    ]
    fitted = fit_static_landmark(factors)
    assert fitted is None or all(
        float(np.dot(np.asarray(fitted[0]) - origin, direction)) > 0.0
        for origin, direction in zip(origins, directions))


def test_bearings_below_five_degree_parallax_do_not_create_3d():
    target = (0.0, 0.0, 5.0)
    factors = [_ray((0.0, 0.0, 0.0), target),
               _ray((0.05, 0.0, 0.0), target)]
    assert math.degrees(math.acos(float(np.dot(
        factors[0]['ray_direction'], factors[1]['ray_direction'])))) < 5.0
    assert fit_static_landmark(factors) is None


def test_huber_fit_resists_an_isolated_position_outlier():
    target = np.asarray((2.0, -1.0, 0.8))
    covariance = np.diag((0.0004, 0.0004, 0.0004)).reshape(9).tolist()
    factors = [
        {'position': tuple(target + np.asarray((dx, dy, dz))),
         'covariance': covariance}
        for dx, dy, dz in (
            (0.01, 0.0, 0.0), (-0.01, 0.0, 0.0),
            (0.0, 0.01, 0.0), (0.0, -0.01, 0.0), (0.0, 0.0, 0.01),
        )
    ]
    factors.append({'position': (8.0, 7.0, -4.0), 'covariance': covariance})
    fitted = fit_static_landmark(factors, initial_position=tuple(target))
    assert fitted is not None
    assert np.linalg.norm(np.asarray(fitted[0]) - target) < 0.03


def test_covariance_keeps_a_nonzero_systematic_floor():
    target = (2.0, 1.0, 0.5)
    covariance = np.diag((1e-6, 1e-6, 1e-6)).reshape(9).tolist()
    one = fit_static_landmark(
        [{'position': target, 'covariance': covariance}],
        pose_rotation_sigma_deg=0.0, extrinsic_rotation_sigma_deg=0.0)
    many = fit_static_landmark(
        [{'position': target, 'covariance': covariance} for _ in range(100)],
        pose_rotation_sigma_deg=0.0, extrinsic_rotation_sigma_deg=0.0)
    assert one is not None and many is not None
    one_floor = min(one[1][0], one[1][4], one[1][8])
    many_floor = min(many[1][0], many[1][4], many[1][8])
    assert one_floor >= 0.03 ** 2
    assert many_floor >= 0.03 ** 2
    assert many_floor >= one_floor * 0.99
