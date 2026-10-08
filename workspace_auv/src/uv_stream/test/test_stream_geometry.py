import numpy as np
import pytest

from uv_stream.stream_geometry import (
    DISPLAY_HEIGHT, DISPLAY_WIDTH, resize_stitched_bgr, scale_detection_box,
)


def test_stitched_resize_preserves_overall_and_per_eye_aspect_ratio():
    source = np.zeros((960, 2560, 3), dtype=np.uint8)
    resized = resize_stitched_bgr(source)

    assert resized.shape == (DISPLAY_HEIGHT, DISPLAY_WIDTH, 3)
    assert (resized.shape[1] / resized.shape[0]
            == source.shape[1] / source.shape[0])
    left_eye = resized[:, :DISPLAY_WIDTH // 2]
    right_eye = resized[:, DISPLAY_WIDTH // 2:]
    assert left_eye.shape[:2] == right_eye.shape[:2] == (480, 640)
    assert left_eye.shape[1] / left_eye.shape[0] == 4 / 3


def test_stitched_resize_rejects_unexpected_resolution_instead_of_stretching():
    with pytest.raises(ValueError, match='Expected stitched frame 2560x960'):
        resize_stitched_bgr(np.zeros((720, 1280, 3), dtype=np.uint8))


def test_detection_boxes_scale_both_axes_and_offset_right_eye():
    source_box = (200.0, 100.0, 1000.0, 700.0)

    assert scale_detection_box(source_box, 'front_left') == (100, 50, 500, 350)
    assert scale_detection_box(source_box, 'front_right') == (740, 50, 1140, 350)
