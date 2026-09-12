"""Tests for the detector mapping shared by all uv packages."""

from uv_camera.model_classes import (
    CLASS_IDS,
    DEFAULT_CLASS_NAMES,
    MODEL_MAPPING,
    camera_hint,
    model_class_id,
    multi_instance_class_ids,
    physical_class_name,
)


def test_mapping_ids_are_the_yaml_order():
    assert list(enumerate(DEFAULT_CLASS_NAMES)) == [
        (entry["id"], entry["name"])
        for entry in MODEL_MAPPING["classes"]
    ]
    assert [model_class_id(name) for name in DEFAULT_CLASS_NAMES] == list(
        range(len(DEFAULT_CLASS_NAMES)))


def test_mapping_contains_physical_object_and_camera_metadata():
    assert physical_class_name(model_class_id("gate_front")) == "gate"
    assert camera_hint(model_class_id("gate_front")) == "front"
    assert camera_hint(model_class_id("guide_line")) is None
    assert multi_instance_class_ids() == {2, 3, 4}


def test_detector_labels_resolve_to_the_yaml_ids():
    assert model_class_id("impact_ball_red") == CLASS_IDS["impact_ball_red"]
    assert model_class_id("yellow-golf") == CLASS_IDS["yellow_golf"]
