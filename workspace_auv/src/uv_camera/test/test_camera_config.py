"""Tests for the shared front/down camera registry."""

from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from uv_camera.camera_config import CameraConfigError, load_camera_config


def test_all_camera_profiles_load_intrinsics_without_npz_or_extrinsics():
    for profile in ("sim", "real"):
        for camera in ("front", "down"):
            config = load_camera_config(camera, profile)
            assert config.capture_resolution[0] > 0
            assert config.capture_resolution[1] > 0
            assert config.eye_resolution == (1280, 960)
            assert config.calibration_source == (
                "sim_camera_info" if profile == "sim" else "yaml")
            assert config.side("left").matrix.shape == (3, 3)
            assert config.side("right").distortion.shape == (5,)
            assert not hasattr(config, "calibration_npz")
            assert not hasattr(config.side("left"), "translation")
            assert not hasattr(config.side("left"), "optical_to_body")


def test_config_dir_accepts_package_config_root():
    config_root = Path(__file__).parents[1] / "config"
    config = load_camera_config("down", "sim", config_root)
    assert config.calibration_source == "sim_camera_info"


def test_front_sim_registry_topics_match_expected_names():
    config = load_camera_config("front", "sim")
    assert config.eye_image_topics == {
        "left": "/auv/sim/raw/camera/front/left/image_raw",
        "right": "/auv/sim/raw/camera/front/right/image_raw",
    }


def test_sim_camera_info_topics_match_stonefish_publishers():
    scene = (Path(__file__).resolve().parents[4] / "workspace_sim" / "src"
             / "uv_sim_assets" / "vehicles" / "youlong" / "model"
             / "youlong.scn")
    root = ET.parse(scene).getroot()
    publishers = {
        sensor.get("name"): sensor.find("ros_publisher").get("topic")
        for sensor in root.iter("sensor")
        if sensor.get("type") == "camera"
        and sensor.find("ros_publisher") is not None
    }
    for camera, prefix in (("front", "front"), ("down", "down")):
        config = load_camera_config(camera, "sim")
        for side in ("left", "right"):
            topic = publishers[f"{prefix}_cam_{side}"] + "/camera_info"
            assert config.camera_info_topics[side] == topic


def test_old_schema_requires_urdf_tf_migration(tmp_path):
    source_dir = Path(__file__).parents[1] / "config" / "cameras"
    config_dir = tmp_path / "cameras"
    config_dir.mkdir()
    source = (source_dir / "front.yaml").read_text(encoding="utf-8")
    (config_dir / "front.yaml").write_text(
        source.replace("schema_version: 2", "schema_version: 1", 1),
        encoding="utf-8")
    with pytest.raises(CameraConfigError, match="迁移到 URDF/TF"):
        load_camera_config("front", "sim", config_dir)


def test_extrinsics_are_rejected_even_with_new_schema(tmp_path):
    source_dir = Path(__file__).parents[1] / "config" / "cameras"
    config_dir = tmp_path / "cameras"
    config_dir.mkdir()
    source = (source_dir / "front.yaml").read_text(encoding="utf-8")
    source = source.replace(
        "    calibration_source: yaml\n",
        "    calibration_source: yaml\n"
        "    extrinsics: {left: {}, right: {}}\n",
        1)
    (config_dir / "front.yaml").write_text(source, encoding="utf-8")
    with pytest.raises(CameraConfigError, match="extrinsics"):
        load_camera_config("front", "real", config_dir)
