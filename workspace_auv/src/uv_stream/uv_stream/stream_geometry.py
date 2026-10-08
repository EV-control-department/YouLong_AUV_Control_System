"""Aspect-preserving geometry for the side-by-side stereo display stream."""

from __future__ import annotations

import cv2
import numpy as np


SOURCE_WIDTH = 2560
SOURCE_HEIGHT = 960
DISPLAY_WIDTH = 1280
DISPLAY_HEIGHT = 480
SCALE_X = DISPLAY_WIDTH / SOURCE_WIDTH
SCALE_Y = DISPLAY_HEIGHT / SOURCE_HEIGHT


def resize_stitched_bgr(image: np.ndarray) -> np.ndarray:
    """Downscale one expected stitched BGR frame uniformly by one half."""
    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError(f'Expected a 3-channel BGR frame, got shape {image.shape}')
    height, width = image.shape[:2]
    if (width, height) != (SOURCE_WIDTH, SOURCE_HEIGHT):
        raise ValueError(
            f'Expected stitched frame {SOURCE_WIDTH}x{SOURCE_HEIGHT}, '
            f'got {width}x{height}')
    return cv2.resize(image, (DISPLAY_WIDTH, DISPLAY_HEIGHT),
                      interpolation=cv2.INTER_AREA)


def scale_detection_box(box: tuple[float, float, float, float],
                        camera_name: str) -> tuple[int, int, int, int]:
    """Map a source-eye box into its half of the uniformly scaled stream."""
    x1, y1, x2, y2 = box
    half_offset = DISPLAY_WIDTH // 2 if camera_name.endswith('_right') else 0
    return (
        int(round(x1 * SCALE_X + half_offset)),
        int(round(y1 * SCALE_Y)),
        int(round(x2 * SCALE_X + half_offset)),
        int(round(y2 * SCALE_Y)),
    )
