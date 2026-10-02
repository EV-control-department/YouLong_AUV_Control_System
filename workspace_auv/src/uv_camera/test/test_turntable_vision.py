"""转盘角度从分割多边形提取，不依赖 DDS 图像。"""

from types import SimpleNamespace

from uv_camera.turntable_vision import estimate


def detection(class_id, points):
    return SimpleNamespace(class_id=class_id, confidence=0.9,
                           mask_x=[x for x, _ in points],
                           mask_y=[y for _, y in points])


def test_off_center_label_yields_angle():
    disk = detection(10, [(40, 50), (50, 40), (60, 40), (70, 50),
                          (70, 60), (60, 70), (50, 70), (40, 60)])
    label = detection(11, [(61, 52), (67, 52), (67, 58), (61, 58)])
    result = estimate(SimpleNamespace(detections=[disk, label]), 10, 11)
    assert result['valid']
    assert abs(result['angle_deg']) < 5


def test_centered_label_is_rejected_instead_of_guessing_angle():
    disk = detection(10, [(40, 50), (50, 40), (60, 40), (70, 50),
                          (70, 60), (60, 70), (50, 70), (40, 60)])
    label = detection(11, [(52, 52), (58, 52), (58, 58), (52, 58)])
    result = estimate(SimpleNamespace(detections=[disk, label]), 10, 11)
    assert not result['valid']
    assert result['reason'] == 'label_not_on_radial_track'
