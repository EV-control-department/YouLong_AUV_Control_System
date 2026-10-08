"""Estimate an undirected red-ring image axis without ROS or model inference.

The input is a single BGR eye image and a bbox from that exact image. Angles
are in original image coordinates: right is 0 degrees, down is 90 degrees.
This estimates a 2D projected axis, not a complete 3D ring normal.
"""

from dataclasses import dataclass
import math

import cv2
import numpy as np


@dataclass(frozen=True)
class RingOrientation:
    valid: bool = False
    axis_deg: float = 0.0
    quality: float = 0.0


INVALID_ORIENTATION = RingOrientation()
_MIN_AXIS_LENGTH_PX = 12.0
_CLOSE_KERNEL = np.ones((3, 3), dtype=np.uint8)
# Match the two red ranges already used by the gate feature extractor.
_RED_LOW = (np.array([0, 45, 30], dtype=np.uint8),
            np.array([18, 255, 255], dtype=np.uint8))
_RED_HIGH = (np.array([165, 45, 30], dtype=np.uint8),
             np.array([180, 255, 255], dtype=np.uint8))


def estimate_ring_orientation(image, bbox, *, roi_scale=1.2,
                              min_pixels=24, min_quality=0.8):
    """Return invalid with zero fields for truncated/ambiguous/error inputs.

    Select one connected component by its red-pixel count in the original
    bbox, breaking ties by proximity to the bbox centre. PCA uses the whole
    selected component in the expanded ROI; background components cannot
    win purely by occupying the extra crop margin.
    """
    try:
        if (not isinstance(image, np.ndarray) or image.dtype != np.uint8
                or image.ndim != 3 or image.shape[2] != 3):
            return INVALID_ORIENTATION
        height, width = image.shape[:2]
        if height < 3 or width < 3:
            return INVALID_ORIENTATION
        x1, y1, x2, y2 = (float(value) for value in bbox)
        if (not all(math.isfinite(value) for value in (x1, y1, x2, y2))
                or x2 <= x1 or y2 <= y1):
            return INVALID_ORIENTATION
        # A crop margin may be clipped, but a ring bbox touching the actual
        # eye-image edge cannot establish that the complete shape was seen.
        if x1 <= 0 or y1 <= 0 or x2 >= width-1 or y2 >= height-1:
            return INVALID_ORIENTATION
        if (not math.isfinite(roi_scale) or roi_scale < 1.0
                or not isinstance(min_pixels, int) or min_pixels < 2
                or not math.isfinite(min_quality) or not 0 <= min_quality <= 1):
            return INVALID_ORIENTATION
        cx, cy = (x1+x2)*0.5, (y1+y2)*0.5
        half_w, half_h = (x2-x1)*roi_scale*0.5, (y2-y1)*roi_scale*0.5
        ix1 = max(0, int(math.floor(cx-half_w)))
        iy1 = max(0, int(math.floor(cy-half_h)))
        ix2 = min(width, int(math.ceil(cx+half_w)))
        iy2 = min(height, int(math.ceil(cy+half_h)))
        roi = image[iy1:iy2, ix1:ix2]
        if roi.size == 0:
            return INVALID_ORIENTATION
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.bitwise_or(cv2.inRange(hsv, *_RED_LOW),
                             cv2.inRange(hsv, *_RED_HIGH))
        # Closing bridges small gaps without eroding thin pipe projections.
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, _CLOSE_KERNEL,
                               iterations=1)
        count, labels, _, centroids = cv2.connectedComponentsWithStats(
            mask, connectivity=8)
        if count <= 1:
            return INVALID_ORIENTATION
        inside_x = (np.arange(ix1, ix2) >= x1) & (np.arange(ix1, ix2) <= x2)
        inside_y = (np.arange(iy1, iy2) >= y1) & (np.arange(iy1, iy2) <= y2)
        inside = labels[np.ix_(inside_y, inside_x)]
        scores = np.bincount(inside.ravel(), minlength=count)
        candidates = [label for label in range(1, count) if scores[label] > 0]
        if not candidates:
            return INVALID_ORIENTATION
        centre = np.array([cx-ix1, cy-iy1])
        selected = max(candidates, key=lambda label: (
            scores[label], -float(np.sum((centroids[label]-centre)**2))))
        ys, xs = np.where(labels == selected)
        if len(xs) < min_pixels:
            return INVALID_ORIENTATION
        points = np.column_stack((xs, ys)).astype(np.float64)
        centred = points-points.mean(axis=0)
        covariance = centred.T @ centred / (len(points)-1)
        eigenvalues, eigenvectors = np.linalg.eigh(covariance)
        small, large = float(eigenvalues[0]), float(eigenvalues[1])
        total = small+large
        if not math.isfinite(total) or total <= 1e-12:
            return INVALID_ORIENTATION
        quality = float(np.clip((large-small)/total, 0.0, 1.0))
        if quality < min_quality:
            return INVALID_ORIENTATION
        axis = eigenvectors[:, 1]
        low, high = np.percentile(centred @ axis, [5, 95])
        if high-low < _MIN_AXIS_LENGTH_PX:
            return INVALID_ORIENTATION
        angle = math.degrees(math.atan2(float(axis[1]), float(axis[0]))) % 180.0
        # The message uses float32: canonicalize again after rounding so an
        # angle just below 180 cannot be published as the excluded endpoint.
        angle = float(np.float32(angle)) % 180.0
        return RingOrientation(True, angle, quality)
    except (TypeError, ValueError, OverflowError, cv2.error,
            np.linalg.LinAlgError):
        return INVALID_ORIENTATION
