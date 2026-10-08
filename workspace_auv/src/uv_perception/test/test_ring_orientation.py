"""Pure CV acceptance tests; no ROS, images from cameras, or YOLO needed."""
import math

import cv2
import numpy as np
import pytest

from uv_perception.ring_orientation import (
    INVALID_ORIENTATION, estimate_ring_orientation,
)


def line_image(angle, *, thickness=3, length=140, shape=(240, 320),
               center=(160, 120), hue=None):
    image = np.zeros((*shape, 3), dtype=np.uint8)
    direction = np.array([math.cos(math.radians(angle)),
                          math.sin(math.radians(angle))])
    centre = np.asarray(center)
    a = np.rint(centre-length*.5*direction).astype(int)
    b = np.rint(centre+length*.5*direction).astype(int)
    color = (0, 0, 255)
    if hue is not None:
        color = tuple(int(v) for v in cv2.cvtColor(
            np.array([[[hue, 255, 255]]], dtype=np.uint8), cv2.COLOR_HSV2BGR)[0, 0])
    cv2.line(image, tuple(a), tuple(b), color, thickness)
    ys, xs = np.where(np.any(image != 0, axis=2))
    bbox = (float(xs.min()-2), float(ys.min()-2),
            float(xs.max()+2), float(ys.max()+2))
    return image, bbox


def angular_error(a, b):
    return abs((a-b+90) % 180-90)


@pytest.mark.parametrize('angle', [0, .1, 1, 10, 30, 45, 60, 89, 90, 120,
                                   135, 170, 179, 179.9])
def test_clear_axes_are_within_three_degrees(angle):
    image, bbox = line_image(angle)
    result = estimate_ring_orientation(image, bbox)
    assert result.valid
    assert 0 <= result.axis_deg < 180
    assert angular_error(result.axis_deg, angle) <= 3
    assert .8 <= result.quality <= 1


@pytest.mark.parametrize('hue', [0, 10, 170, 179])
def test_both_red_hue_ranges(hue):
    image, bbox = line_image(30, hue=hue)
    result = estimate_ring_orientation(image, bbox)
    assert result.valid and angular_error(result.axis_deg, 30) <= 3


def test_closing_preserves_one_pixel_line_and_bridges_small_gap():
    image, bbox = line_image(0, thickness=1)
    image[120, 159:161] = 0
    result = estimate_ring_orientation(image, bbox)
    assert result.valid and angular_error(result.axis_deg, 0) <= 3


@pytest.mark.parametrize('mode', ['empty', 'green', 'gray', 'circle', 'short', 'sparse'])
def test_unreliable_inputs_are_invalid_and_all_zero(mode):
    image = np.zeros((240, 320, 3), dtype=np.uint8)
    bbox = (90., 50., 230., 190.)
    if mode == 'green':
        cv2.line(image, (100, 120), (220, 120), (0, 255, 0), 3)
    elif mode == 'gray':
        cv2.line(image, (100, 120), (220, 120), (100, 100, 100), 3)
    elif mode == 'circle':
        cv2.circle(image, (160, 120), 45, (0, 0, 255), 4)
    elif mode == 'short':
        cv2.line(image, (155, 120), (165, 120), (0, 0, 255), 5)
    elif mode == 'sparse':
        cv2.line(image, (155, 120), (165, 120), (0, 0, 255), 1)
    assert estimate_ring_orientation(image, bbox) == INVALID_ORIENTATION


@pytest.mark.parametrize('bbox', [
    (0, 10, 100, 100), (10, 0, 100, 100), (10, 10, 319, 100),
    (10, 10, 100, 239), (-5, 10, 100, 100), (10, 10, 325, 100),
    (10, 10, 10, 100), (20, 10, 10, 100), (math.nan, 10, 100, 100),
    (10, 10, math.inf, 100),
])
def test_truncated_or_malformed_boxes(bbox):
    image, _ = line_image(30)
    assert estimate_ring_orientation(image, bbox) == INVALID_ORIENTATION


@pytest.mark.parametrize('image', [None, np.zeros((20, 20), dtype=np.uint8),
                                  np.zeros((20, 20, 4), dtype=np.uint8),
                                  np.zeros((20, 20, 3), dtype=float)])
def test_malformed_images(image):
    assert estimate_ring_orientation(image, (2, 2, 10, 10)) == INVALID_ORIENTATION


def test_roi_margin_clipped_but_complete_bbox_is_valid():
    image, bbox = line_image(90, length=140, center=(8, 120))
    result = estimate_ring_orientation(image, bbox, roi_scale=2.0)
    assert result.valid and angular_error(result.axis_deg, 90) <= 3


def test_large_neighbour_in_expanded_margin_does_not_win():
    image, bbox = line_image(0)
    # Large red areas outside the original box should not win simply because
    # a deliberately enlarged crop sees more of their pixels.
    cv2.rectangle(image, (140, 15), (180, 75), (0, 0, 255), -1)
    result = estimate_ring_orientation(image, bbox, roi_scale=20.)
    assert result.valid and angular_error(result.axis_deg, 0) <= 3


def test_tie_breaker_selects_component_nearest_bbox_centre():
    image = np.zeros((240, 320, 3), dtype=np.uint8)
    # Equal 61-pixel components, horizontal at centre and vertical away.
    cv2.line(image, (130, 120), (190, 120), (0, 0, 255), 1)
    cv2.line(image, (100, 65), (100, 125), (0, 0, 255), 1)
    result = estimate_ring_orientation(image, (80, 50, 240, 190))
    assert result.valid and angular_error(result.axis_deg, 0) <= 3


def test_configurable_pixel_and_quality_thresholds():
    image, bbox = line_image(20)
    assert estimate_ring_orientation(image, bbox).valid
    assert estimate_ring_orientation(image, bbox, min_pixels=10_000) == INVALID_ORIENTATION
    assert estimate_ring_orientation(image, bbox, min_quality=1.0) == INVALID_ORIENTATION


@pytest.mark.parametrize('kwargs', [dict(roi_scale=.5), dict(roi_scale=math.nan),
                                   dict(min_pixels=1), dict(min_quality=1.1)])
def test_invalid_parameters_do_not_leak_a_direction(kwargs):
    image, bbox = line_image(45)
    assert estimate_ring_orientation(image, bbox, **kwargs) == INVALID_ORIENTATION


def test_cv_error_returns_invalid(monkeypatch):
    image, bbox = line_image(30)
    def fail(*_):
        raise cv2.error('synthetic error')
    monkeypatch.setattr(cv2, 'cvtColor', fail)
    assert estimate_ring_orientation(image, bbox) == INVALID_ORIENTATION


def test_linear_algebra_error_returns_invalid(monkeypatch):
    image, bbox = line_image(30)
    def fail(*_):
        raise np.linalg.LinAlgError('synthetic error')
    monkeypatch.setattr(np.linalg, 'eigh', fail)
    assert estimate_ring_orientation(image, bbox) == INVALID_ORIENTATION


def test_does_not_mutate_source_image():
    image, bbox = line_image(50)
    before = image.copy()
    assert estimate_ring_orientation(image, bbox).valid
    assert np.array_equal(image, before)
