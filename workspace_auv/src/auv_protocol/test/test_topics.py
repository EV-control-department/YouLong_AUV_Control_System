"""Contract tests for the canonical YouLong AUV ROS namespace."""

from auv_protocol import topics


def test_public_topic_constants_are_vehicle_scoped():
    constants = [
        value for name, value in vars(topics).items()
        if name.isupper() and name not in ('ROOT', 'RVIZ_TF', 'RVIZ_TF_STATIC')
        and isinstance(value, str)
        and not name.startswith(('LEGACY_', 'ICEORYX_'))
    ]
    assert constants
    assert all(value.startswith('/auv/') for value in constants)


def test_stream_frame_metadata_topic_is_vehicle_scoped():
    assert topics.STREAM_FRAME_INFO == '/auv/stream/frame_info'


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


def test_origin_reset_and_arm_owner_endpoints():
    assert topics.STATE_RESET_RESULT == '/auv/state/reset_result'
    assert topics.ZIT6_ARM_HEARTBEAT == '/auv/hardware/zit6/cmd/agxhbt'
    assert topics.ZIT6_SET_ORIGIN == '/auv/hardware/zit6/cmd/setorigin'
    assert topics.ZIT6_ODOM == '/auv/hardware/zit6/state/odom'
    assert topics.LEGACY_ZIT6_SET_ORIGIN == '/zit6/cmd/setorigin'
    assert topics.LEGACY_ZIT6_ODOM == '/zit6/state/odom'
