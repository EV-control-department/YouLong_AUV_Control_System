"""Exercise ArUco creation and decoding with both modern and legacy APIs."""
from types import SimpleNamespace as NS

import cv2
import numpy as np
import pytest
from uv_task.aruco_decoder import ArucoDecoder


@pytest.mark.parametrize('marker_id', range(1, 7))
def test_real_opencv_decodes_generated_marker(marker_id):
    decoder = ArucoDecoder()
    marker = decoder._generate_marker(marker_id, 120)
    image = np.full((200, 200, 3), 255, dtype=np.uint8)
    image[40:160, 40:160] = marker[:, :, None]
    assert decoder.detect(image) == {marker_id}


def test_legacy_parameter_creation_detection_and_rendering(monkeypatch):
    original = cv2.aruco
    if hasattr(original, 'DetectorParameters'):
        factory = original.DetectorParameters
    else:
        factory = original.DetectorParameters_create
    if hasattr(original, 'generateImageMarker'):
        render = original.generateImageMarker
    else:
        render = original.drawMarker
    calls = []

    def parameters():
        calls.append('parameters')
        return factory()

    def detect(image, dictionary, parameters):
        calls.append('detect')
        if hasattr(original, 'ArucoDetector'):
            return original.ArucoDetector(dictionary, parameters).detectMarkers(image)
        return original.detectMarkers(image, dictionary, parameters=parameters)

    legacy = NS(getPredefinedDictionary=original.getPredefinedDictionary,
                DICT_4X4_1000=original.DICT_4X4_1000,
                CORNER_REFINE_SUBPIX=original.CORNER_REFINE_SUBPIX,
                DetectorParameters_create=parameters, detectMarkers=detect,
                drawMarker=render)
    monkeypatch.setattr(cv2, 'aruco', legacy)
    decoder = ArucoDecoder()
    marker = decoder._generate_marker(3, 120)
    image = np.full((200, 200), 255, dtype=np.uint8)
    image[40:160, 40:160] = marker
    _, ids, _ = decoder._detector.detectMarkers(image)
    assert ids is not None and 3 in ids.ravel()
    assert calls == ['parameters', 'detect']
