"""Small YOLO adapter with no ROS or image-topic dependency."""

from __future__ import annotations

import threading

# CUDA synchronization can block Python while another camera owns GPU work.
# Serialize inference across independent model instances, including CPU copies.
_INFERENCE_LOCK = threading.Lock()


class YoloDetector:
    def __init__(self, model_path: str, confidence: float = 0.5, device: str = ''):
        try:
            from ultralytics import YOLO
        except Exception as error:  # pragma: no cover - install-specific
            raise RuntimeError('ultralytics is required for object_detector') from error
        self.model = YOLO(model_path)
        self.confidence = float(confidence)
        self.device = str(device or '')

    def validate_mapping(self, registry):
        names = self.model.names
        names = dict(enumerate(names)) if isinstance(names, (list, tuple)) else names
        expected = {entry.id: entry.name for entry in registry.entries}
        normalize = lambda value: str(value).strip().lower().replace('-', '_').replace(' ', '_')
        actual = {int(key): normalize(value) for key, value in names.items()}
        if not expected or actual != expected:
            raise ValueError(f'weight labels {actual} do not match shared mapping {expected}')

    def detect(self, image):
        """Return the legacy box tuples while preserving the simple API."""
        return self.detect_with_masks(image)[0]

    def detect_with_masks(self, image):
        """Return (box detections, aligned optional segmentation polygons)."""
        with _INFERENCE_LOCK:
            return self._detect_with_masks_locked(image)

    def _detect_with_masks_locked(self, image):
        kwargs = {'source': image, 'conf': self.confidence, 'verbose': False}
        if self.device:
            kwargs['device'] = self.device
        results = self.model.predict(**kwargs)
        if not results:
            return (), ()
        boxes = getattr(results[0], 'boxes', None)
        if boxes is None:
            return (), ()
        xyxy = boxes.xyxy.cpu().numpy().tolist()
        confidences = boxes.conf.cpu().numpy().tolist()
        classes = boxes.cls.cpu().numpy().tolist()
        mask_result = getattr(results[0], 'masks', None)
        polygons = getattr(mask_result, 'xy', ()) if mask_result is not None else ()
        detections = []
        aligned_polygons = []
        for index, (class_id, confidence, box) in enumerate(
                zip(classes, confidences, xyxy)):
            detections.append((
                int(class_id), float(confidence), tuple(map(float, box))))
            polygon = polygons[index] if index < len(polygons) else None
            aligned_polygons.append(polygon)
        return tuple(detections), tuple(aligned_polygons)
