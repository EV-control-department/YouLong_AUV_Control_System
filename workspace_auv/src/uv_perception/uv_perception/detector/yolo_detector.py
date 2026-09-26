"""Small YOLO adapter with no ROS or image-topic dependency."""

from __future__ import annotations


class YoloDetector:
    def __init__(self, model_path: str, confidence: float = 0.5, device: str = ''):
        try:
            from ultralytics import YOLO
        except Exception as error:  # pragma: no cover - install-specific
            raise RuntimeError('ultralytics is required for object_detector') from error
        self.model = YOLO(model_path)
        self.confidence = float(confidence)
        self.device = str(device or '')

    def detect(self, image):
        """Return the legacy box tuples while preserving the simple API."""
        return self.detect_with_masks(image)[0]

    def detect_with_masks(self, image):
        """Return (box detections, aligned optional segmentation polygons)."""
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
