"""Independent camera models must not execute CUDA inference concurrently."""
from concurrent.futures import ThreadPoolExecutor
from types import SimpleNamespace
import threading
import time
import pytest
from uv_perception.detector.yolo_detector import YoloDetector


def adapter(predict):
    detector = YoloDetector.__new__(YoloDetector)
    detector.model = SimpleNamespace(predict=predict)
    detector.confidence = .5
    detector.device = ''
    return detector


def test_camera_models_share_inference_lock():
    active = 0
    peak = 0
    guard = threading.Lock()
    start = threading.Barrier(2)
    def predict(**kwargs):
        nonlocal active, peak
        with guard:
            active += 1
            peak = max(peak, active)
        time.sleep(.02)
        with guard:
            active -= 1
        return []
    models = [adapter(predict), adapter(predict)]
    def run(model):
        start.wait(timeout=2)
        for _ in range(5):
            assert model.detect_with_masks(None) == ((), ())
    with ThreadPoolExecutor(max_workers=2) as pool:
        futures = [pool.submit(run, model) for model in models]
        for future in futures:
            future.result(timeout=3)
    assert peak == 1


def test_inference_error_releases_shared_lock():
    def fail(**kwargs):
        raise RuntimeError('inference failure')
    with pytest.raises(RuntimeError, match='inference failure'):
        adapter(fail).detect_with_masks(None)
    assert adapter(lambda **kwargs: []).detect_with_masks(None) == ((), ())
