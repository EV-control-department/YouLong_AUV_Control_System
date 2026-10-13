import copy

import pytest

from uv_record.scene3d import scene_from_map, marker_pattern


def test_rotated_grid_preserves_world_coordinates_and_separates_fallback():
    mapping = {'grid': {'center': [2, -4, 1.4], 'side_m': 2.4, 'yaw_deg': 90, 'floor_z': 1.4},
               'final_assignment': {'0': 0, '8': 1},
               'cells': {'0': {'position': [2.1, -3.1, 1.15], 'source': 'vision'},
                         '8': {'source': 'fallback'}},
               'tag': {'id': 18, 'position': [1.5, -2, 1.4], 'source': 'vision'},
               'submitted_config': {'fallback_square_cells': [2, 6], 'fallback_round_cells': [4, 8]},
               'conclusion': {'source': 'fallback', 'assignment': {'2': 0},
                              'cells': [{'id': 2, 'position': [8, 9, 1.4], 'source': 'fallback'}]}}
    original = copy.deepcopy(mapping)
    scene = scene_from_map(mapping)
    assert mapping == original
    cells = {cell['id']: cell for cell in scene['cells']}
    assert cells[0]['center'] == pytest.approx([2.8, -3.2, 1.4])
    assert cells[8]['center'] == pytest.approx([1.2, -4.8, 1.4])
    visual = scene['modes']['visual']['objects']
    assert visual[0]['position'] == [2.1, -3.1, 1.15] and visual[0]['anchor'] == 'center'
    assert visual[1]['position'] == pytest.approx([1.2, -4.8, 1.4]) and visual[1]['anchor'] == 'base'
    assert scene['modes']['visual']['tag']['position'] == [1.5, -2, 1.4]
    assert scene['modes']['submission']['objects'][0]['position'] == [8, 9, 1.4]
    assert scene['modes']['fallback']['objects'][0]['position'] != [8, 9, 1.4]


def test_no_position_or_marker_dictionary_does_not_invent_geometry():
    scene = scene_from_map({'final_assignment': {'0': 0}, 'cells': {}, 'tag': {'id': 18}})
    assert scene['modes']['visual']['objects'][0]['position'] is None
    assert scene['grid']['center'] is None
    assert scene['modes']['visual']['tag']['pattern'] is None
    assert scene['modes']['visual']['tag']['position'] is None
    assert not scene['modes']['submission']['available']
    assert marker_pattern(18, None) is None


def test_marker_uses_actual_recorded_dictionary():
    cv2 = pytest.importorskip('cv2')
    if not hasattr(cv2, 'aruco'):
        pytest.skip('OpenCV aruco unavailable')
    pattern = marker_pattern(18, 'DICT_APRILTAG_16h5')
    assert pattern is not None and len(pattern) == 6
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_16h5)
    expected = (cv2.aruco.generateImageMarker(dictionary, 18, 6)
                if hasattr(cv2.aruco, 'generateImageMarker') else cv2.aruco.drawMarker(dictionary, 18, 6))
    assert pattern == (expected > 127).astype(int).tolist()
    assert marker_pattern(999999, 'DICT_APRILTAG_16h5') is None
