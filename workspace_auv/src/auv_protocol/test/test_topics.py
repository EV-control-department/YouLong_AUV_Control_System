"""Contract tests for the canonical YouLong AUV ROS namespace."""

from auv_protocol import topics


def test_public_topic_constants_are_vehicle_scoped():
    constants = [
        value for name, value in vars(topics).items()
        if name.isupper() and name != 'ROOT' and isinstance(value, str)
        and not name.startswith('LEGACY_')
    ]
    assert constants
    assert all(value.startswith('/auv/') for value in constants)


def test_camera_channel_names_match_the_canonical_protocol():
    assert topics.DETECTIONS('front_left') == (
        '/auv/perception/detections/front/left')
    assert topics.DETECTIONS('down_left') == (
        '/auv/perception/detections/downward/left')
    assert topics.LINES('front_right') == (
        '/auv/perception/lines/front/right')


def test_legacy_endpoints_are_explicit_compatibility_only():
    legacy = [
        topics.LEGACY_BASIC_MOTION,
        topics.LEGACY_POSE_INFO,
        topics.LEGACY_ZIT6_SETPOINT,
        topics.LEGACY_TASK_RUN,
        topics.LEGACY_TASK_STOP,
        topics.LEGACY_TASK_STATUS,
        topics.LEGACY_TASK_EXECUTE,
    ]
    assert all(not value.startswith('/auv/') for value in legacy)
