from uv_perception.contracts import DETECTIONS, LINES, OBJECTS, TARGETS


def test_perception_contracts_are_vehicle_scoped():
    assert DETECTIONS('front_left') == '/auv/perception/detections/front/left'
    assert LINES('down_left') == '/auv/perception/lines/downward/left'
    assert OBJECTS == '/auv/perception/observations'
    assert TARGETS == '/auv/perception/targets'
