from auv_protocol.topics import (
    MAPPING_KEYFRAMES, MAPPING_LANDMARKS,
    MAPPING_LOCALIZATION_OPPORTUNITY,
)


def test_mapping_topics_are_vehicle_scoped():
    assert all(topic.startswith('/auv/mapping/') for topic in (
        MAPPING_LANDMARKS, MAPPING_KEYFRAMES,
        MAPPING_LOCALIZATION_OPPORTUNITY))
