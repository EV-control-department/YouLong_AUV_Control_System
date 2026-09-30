from uv_perception.object_detector import _camera_optical_frame


def test_detector_camera_frames_match_description_tree():
    assert _camera_optical_frame('front', 'left') == 'front_left_camera_optical_frame'
    assert _camera_optical_frame('down', 'left') == 'downward_left_camera_optical_frame'
    assert _camera_optical_frame('down', 'right') == 'downward_right_camera_optical_frame'
