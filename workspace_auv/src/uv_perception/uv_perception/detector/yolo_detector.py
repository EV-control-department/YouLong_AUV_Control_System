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
        kwargs = {'source': image, 'conf': self.confidence, 'verbose': False}
        if self.device:
            kwargs['device'] = self.device
        results = self.model.predict(**kwargs)
        if not results:
            return ()
        boxes = getattr(results[0], 'boxes', None)
        if boxes is None:
            return ()
        xyxy = boxes.xyxy.cpu().numpy().tolist()
        confidences = boxes.conf.cpu().numpy().tolist()
        classes = boxes.cls.cpu().numpy().tolist()
        return tuple((int(class_id), float(confidence), tuple(map(float, box)))
                     for class_id, confidence, box in zip(classes, confidences, xyxy))
