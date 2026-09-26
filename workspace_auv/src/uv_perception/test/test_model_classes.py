"""Tests for the detector mapping owned by uv_perception and shared by uv packages."""

from uv_perception.model_classes import (
    CLASS_IDS,
    DEFAULT_CLASS_NAMES,
    MODEL_MAPPING,
    MODEL_MAPPING_PATH,
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
    assert model_class_id("yellow_golf") == CLASS_IDS["yellow_golf"]


def test_semantic_colour_aliases_are_not_in_the_shared_mapping():
    import pytest

    with pytest.raises(ValueError):
        model_class_id("blue")
    with pytest.raises(ValueError):
        model_class_id("red")


def test_mapping_and_weights_are_owned_by_uv_perception():
    assert MODEL_MAPPING_PATH.name == "robotcup20260901.yaml"
    assert MODEL_MAPPING_PATH.is_file()
    assert (MODEL_MAPPING_PATH.parent / "robotcup20260901.pt").is_file()
