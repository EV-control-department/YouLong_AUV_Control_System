"""Robust decoder for the WUURC simulated 4x4_1000 ArUco signs."""

from collections import Counter

import cv2
import numpy as np


TARGET_IDS = frozenset(range(1, 7))


def select_majority_id(votes: Counter) -> int | None:
    """Return the most frequent valid ID; use the lower ID to break a tie."""
    valid = [marker_id for marker_id in votes if marker_id in TARGET_IDS]
    if not valid:
        return None
    return min(valid, key=lambda marker_id: (-votes[marker_id], marker_id))


class ArucoDecoder:
    """Port of the verified offline detector for live complete camera frames."""

    def __init__(self):
        self._dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_4X4_1000)
        self._detector = self._make_detector()
        self._dictionary_bits = None
        self._rendered_templates = None

    def _make_detector(self):
        parameters = cv2.aruco.DetectorParameters()
        parameters.adaptiveThreshWinSizeMin = 3
        parameters.adaptiveThreshWinSizeMax = 41
        parameters.adaptiveThreshWinSizeStep = 4
        parameters.minMarkerPerimeterRate = 0.003
        parameters.maxMarkerPerimeterRate = 4.0
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        if hasattr(cv2.aruco, 'ArucoDetector'):
            return cv2.aruco.ArucoDetector(self._dictionary, parameters)

        dictionary = self._dictionary

        class LegacyDetector:
            def detectMarkers(self, image):
                return cv2.aruco.detectMarkers(
                    image, dictionary, parameters=parameters)

        return LegacyDetector()

    def _generate_marker(self, marker_id: int, side_pixels: int):
        if hasattr(cv2.aruco, 'generateImageMarker'):
            return cv2.aruco.generateImageMarker(
                self._dictionary, marker_id, side_pixels)
        return cv2.aruco.drawMarker(
            self._dictionary, marker_id, side_pixels)

    def _dictionary_payloads(self):
        if self._dictionary_bits is not None:
            return self._dictionary_bits

        values = {}
        # The WUURC regulation only accepts IDs 1..6.  Building those
        # rotations is equivalent for this task and avoids a large startup
        # delay from enumerating all 1000 dictionary entries.
        for marker_id in sorted(TARGET_IDS):
            marker = self._generate_marker(marker_id, 60)
            bits = np.array(
                [
                    [
                        int(marker[int((row + 1.5) * 10),
                                   int((col + 1.5) * 10)] > 128)
                        for col in range(4)
                    ]
                    for row in range(4)
                ],
                dtype=np.uint8,
            )
            for rotation in range(4):
                values.setdefault(tuple(np.rot90(bits, rotation).ravel()), marker_id)
        self._dictionary_bits = values
        return values

    def _template_masks(self):
        if self._rendered_templates is None:
            self._rendered_templates = {
                marker_id: self._generate_marker(marker_id, 256) > 128
                for marker_id in TARGET_IDS
            }
        return self._rendered_templates

    @staticmethod
    def _order_corners(points):
        points = points.reshape(4, 2).astype(np.float32)
        ordered = np.zeros((4, 2), dtype=np.float32)
        sums = points.sum(axis=1)
        differences = np.diff(points, axis=1).ravel()
        ordered[0] = points[np.argmin(sums)]
        ordered[2] = points[np.argmax(sums)]
        ordered[1] = points[np.argmin(differences)]
        ordered[3] = points[np.argmax(differences)]
        return ordered

    def _decode_square(self, gray, corners):
        destination = np.array(
            [[0, 0], [119, 0], [119, 119], [0, 119]], dtype=np.float32)
        transform = cv2.getPerspectiveTransform(
            self._order_corners(corners), destination)
        warped = cv2.warpPerspective(gray, transform, (120, 120))
        cell_values = np.array(
            [
                [
                    np.median(warped[row * 20 + 5:row * 20 + 15,
                                     col * 20 + 5:col * 20 + 15])
                    for col in range(6)
                ]
                for row in range(6)
            ],
            dtype=np.float32,
        )
        threshold = (float(cell_values.min()) + float(cell_values.max())) / 2.0
        cells = cell_values > threshold
        if (np.any(cells[0]) or np.any(cells[5]) or np.any(cells[:, 0])
                or np.any(cells[:, 5])):
            return None
        marker_id = self._dictionary_payloads().get(
            tuple(cells[1:5, 1:5].astype(np.uint8).ravel()))
        return marker_id

    def _match_rendered_template(self, gray, corners):
        destination = np.array(
            [[0, 0], [255, 0], [255, 255], [0, 255]], dtype=np.float32)
        transform = cv2.getPerspectiveTransform(
            self._order_corners(corners), destination)
        warped = cv2.warpPerspective(gray, transform, (256, 256))
        warped_mask = warped > np.percentile(warped, 75)
        best_id, best_score = None, 0.0
        for marker_id, template in self._template_masks().items():
            score = float(np.mean(warped_mask == template))
            if score > best_score:
                best_id, best_score = marker_id, score
        return best_id if best_score >= 0.80 else None

    def _contour_fallback(self, image):
        channel = image[:, :, 0]
        candidates = []
        for threshold in (50, 60, 70, 80, 90, 100, 110):
            binary = cv2.threshold(
                channel, threshold, 255, cv2.THRESH_BINARY_INV)[1]
            contours, _ = cv2.findContours(
                binary, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                perimeter = cv2.arcLength(contour, True)
                if perimeter == 0:
                    continue
                polygon = cv2.approxPolyDP(contour, 0.03 * perimeter, True)
                _x, _y, width, height = cv2.boundingRect(polygon)
                if len(polygon) != 4 or not (40 <= width <= 500 and 40 <= height <= 500):
                    continue
                if not 0.65 <= width / height <= 1.5:
                    continue
                candidates.append((cv2.contourArea(contour), polygon))

        detections = set()
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        for _, polygon in sorted(candidates, reverse=True, key=lambda item: item[0]):
            marker_id = self._decode_square(gray, polygon)
            if marker_id is None:
                marker_id = self._match_rendered_template(gray, polygon)
            if marker_id in TARGET_IDS:
                detections.add(marker_id)
        return detections

    @staticmethod
    def _preprocess(image):
        channels = cv2.split(image)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8))
        yield gray, (1, 2)
        yield clahe.apply(gray), (2,)
        for channel in channels:
            yield channel, (2,)

    def detect(self, image):
        """Return all valid target IDs found in one complete BGR frame."""
        # The verified contour/template path is both robust to the rendered
        # Stonefish texture and much faster than running all large-scale
        # ArUco variants.  Use it first so 50-frame live voting keeps pace
        # with the camera stream.
        detections = self._contour_fallback(image)
        if detections:
            return detections

        for variant, scales in self._preprocess(image):
            for scale in scales:
                scaled = variant if scale == 1 else cv2.resize(
                    variant, None, fx=scale, fy=scale,
                    interpolation=cv2.INTER_CUBIC)
                _corners, ids, _rejected = self._detector.detectMarkers(scaled)
                if ids is None:
                    continue
                detections.update(
                    int(marker_id) for marker_id in ids.ravel()
                    if int(marker_id) in TARGET_IDS)
        return detections
