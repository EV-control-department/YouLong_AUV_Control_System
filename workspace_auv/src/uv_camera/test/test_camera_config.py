"""Tests for the shared front/down camera registry."""

from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from uv_camera.camera_config import CameraConfigError, load_camera_config


def test_all_camera_modes_load_intrinsics_without_npz_or_extrinsics():
    for mode in ("sim", "real"):
        for camera in ("front", "down"):
            config = load_camera_config(camera, mode)
            assert config.mode == mode
            assert config.capture_resolution[0] > 0
            assert config.capture_resolution[1] > 0
            assert config.eye_resolution == (1280, 960)
            assert config.calibration_source == (
                "sim_camera_info" if mode == "sim" else "yaml")
            assert config.side("left").matrix.shape == (3, 3)
            assert config.side("right").distortion.shape == (5,)
            assert not hasattr(config, "calibration_npz")
            assert not hasattr(config.side("left"), "translation")
            assert not hasattr(config.side("left"), "optical_to_body")


def test_config_dir_accepts_package_root():
    config_root = Path(__file__).parents[1]
    config = load_camera_config("down", "sim", config_root)
    assert config.calibration_source == "sim_camera_info"


def test_down_real_intrinsics_match_latest_stereo_calibration():
    config = load_camera_config("down", "real")
    left = config.side("left")
    right = config.side("right")
    assert left.matrix[0, 0] == pytest.approx(1159.0997987420169)
    assert left.matrix[0, 1] == pytest.approx(0.6582010360123447)
    assert left.distortion.tolist() == pytest.approx([
        -0.36428009567333464, 0.12966969145467228,
        0.0019532488954503787, -0.0000080454778884803021,
        0.1236567536694326,
    ])
    assert right.matrix[0, 0] == pytest.approx(1151.5984326373555)
    assert right.distortion.tolist() == pytest.approx([
        -0.36148274221566018, 0.1698694617848012,
        0.0019289386344567849, 0.0026311467098986264,
        -0.0449759012771526,
    ])


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
    source_dir = Path(__file__).parents[1] / "stereos"
    config_dir = tmp_path / "cameras"
    config_dir.mkdir()
    source = (source_dir / "front.yaml").read_text(encoding="utf-8")
    (config_dir / "front.yaml").write_text(
        source.replace("schema_version: 2", "schema_version: 1", 1),
        encoding="utf-8")
    with pytest.raises(CameraConfigError, match="迁移到 URDF/TF"):
        load_camera_config("front", "sim", config_dir)


def test_extrinsics_are_rejected_even_with_new_schema(tmp_path):
    source_dir = Path(__file__).parents[1] / "stereos"
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


def test_urdf_remains_the_source_of_stereo_baselines():
    urdf = Path(__file__).resolve().parents[2] / "auv_description" / "urdf" / "auv.urdf"
    root = ET.parse(urdf).getroot()
    origins = {
        joint.get("name"): tuple(float(value) for value in
                                 joint.find("origin").get("xyz").split())
        for joint in root.findall("joint")
        if joint.get("name") in {
            "front_left_camera_mount", "front_right_camera_mount",
            "downward_left_camera_mount", "downward_right_camera_mount",
        }
    }
    assert origins["front_left_camera_mount"][1] == pytest.approx(-0.03)
    assert origins["front_right_camera_mount"][1] == pytest.approx(0.03)
    assert origins["downward_left_camera_mount"][1] == pytest.approx(
        -0.03058633655786893)
    assert origins["downward_right_camera_mount"][1] == pytest.approx(
        0.03058633655786893)
    for left, right, expected_baseline in (
            ("front_left_camera_mount", "front_right_camera_mount", 0.06),
            ("downward_left_camera_mount", "downward_right_camera_mount",
             0.06117267311573786)):
        baseline = sum((a - b) ** 2 for a, b in
                       zip(origins[left], origins[right])) ** 0.5
        assert baseline == pytest.approx(expected_baseline)


def test_camera_calibration_files_are_installed_under_stereos():
    package_root = Path(__file__).parents[1]
    setup = (package_root / "setup.py").read_text(encoding="utf-8")
    assert "stereos/front.yaml" in setup
    assert "stereos/down.yaml" in setup
    assert "config/cameras" not in setup
    assert "config/profiles" not in setup
