"""Tests for the shared front/down camera registry."""

from pathlib import Path
import shutil

import numpy as np
import pytest

from uv_camera.camera_config import CameraConfigError, load_camera_config
from uv_camera.model_classes import model_class_id


def test_all_camera_profiles_load_and_resolve_npz():
    for profile in ("sim", "real"):
        for camera in ("front", "down"):
            config = load_camera_config(camera, profile)
            assert config.capture_resolution[0] > 0
            assert config.capture_resolution[1] > 0
            assert config.eye_resolution == (1280, 960)
            assert config.calibration_npz.is_file()
            assert config.side("left").matrix.shape == (3, 3)
            assert config.side("right").distortion.shape == (5,)


def test_config_dir_accepts_package_config_root():
    config_root = Path(__file__).parents[1] / "config"
    config = load_camera_config("down", "sim", config_root)
    assert config.calibration_npz.name == "robotcup_down.npz"


def test_front_sim_registry_contains_task_extrinsics():
    config = load_camera_config("front", "sim")
    assert config.eye_image_topics == {
        "left": "/sim/front_cam/left/image_color",
        "right": "/sim/front_cam/right/image_color",
    }
    assert config.side("left").translation.tolist() == [0.19, -0.05, 0.176]
    assert config.side("right").translation.tolist() == [0.19, 0.05, 0.176]
    assert config.side("left").optical_to_body.tolist() == [
        [0.0, 0.0, 1.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    ]


def test_yaml_npz_intrinsic_mismatch_is_rejected(tmp_path):
    source_dir = Path(__file__).parents[1] / "config" / "cameras"
    config_dir = tmp_path / "cameras"
    config_dir.mkdir()
    source = (source_dir / "front.yaml").read_text(encoding="utf-8")
    (config_dir / "front.yaml").write_text(
        source.replace("1174.086809", "1175.086809"), encoding="utf-8")
    shutil.copy(
        source_dir.parent / "robotcup_front.npz",
        config_dir.parent / "robotcup_front.npz")
    with pytest.raises(CameraConfigError, match="YAML/NPZ"):
        load_camera_config("front", "sim", config_dir)


def test_npz_must_contain_stereo_geometry(tmp_path):
    source_dir = Path(__file__).parents[1] / "config" / "cameras"
    config_dir = tmp_path / "cameras"
    config_dir.mkdir()
    source = (source_dir / "front.yaml").read_text(encoding="utf-8")
    (config_dir / "front.yaml").write_text(
        source.replace("robotcup_front.npz", "incomplete.npz"),
        encoding="utf-8")
    with np.load(
            source_dir.parent / "robotcup_front.npz",
            allow_pickle=False) as archive:
        arrays = {key: archive[key] for key in archive.files if key != "Q"}
    np.savez(config_dir.parent / "incomplete.npz", **arrays)

    with pytest.raises(CameraConfigError, match="NPZ 缺少标定字段.*Q"):
        load_camera_config("front", "sim", config_dir)


def test_task_class_selection_has_no_semantic_colour_aliases():
    assert model_class_id("impact_ball_blue") == 5
    assert model_class_id("pink_golf") == 7
    with pytest.raises(ValueError):
        model_class_id("blue")
    with pytest.raises(ValueError):
        model_class_id("red")
