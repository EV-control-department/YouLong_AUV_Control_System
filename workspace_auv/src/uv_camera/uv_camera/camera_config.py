"""Shared camera registry for the perception and task packages.

The registry is deliberately independent of ROS.  Camera YAML files describe
the logical camera (input/eye resolution, calibration and body mounting),
while the NPZ file remains the source for stereo-only R/T/P/Q data.
"""

from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path
from typing import Any, Mapping

import numpy as np
import yaml


_CAMERAS = ("front", "down")
_PROFILES = ("sim", "real")
_SIDES = ("left", "right")
_REQUIRED_NPZ_KEYS = (
    "camera_matrix_left", "camera_matrix_right",
    "dist_coeffs_left", "dist_coeffs_right",
    "R", "T", "P1", "P2", "R1", "R2", "Q",
)


@dataclass(frozen=True)
class CameraSideConfig:
    """One eye's intrinsic and body-extrinsic configuration."""

    matrix: np.ndarray
    distortion: np.ndarray
    translation: np.ndarray
    optical_to_body: np.ndarray


@dataclass(frozen=True)
class CameraConfig:
    """Validated configuration for one camera pair and one profile."""

    name: str
    profile: str
    capture_resolution: tuple[int, int]
    eye_resolution: tuple[int, int]
    image_topic: str
    eye_image_topics: Mapping[str, str]
    stereo_info_topic: str
    camera_info_topics: Mapping[str, str]
    device: str | None
    calibration_source: str
    calibration_npz: Path
    sides: Mapping[str, CameraSideConfig]

    @property
    def width(self) -> int:
        return self.eye_resolution[0]

    @property
    def height(self) -> int:
        return self.eye_resolution[1]

    def side(self, name: str) -> CameraSideConfig:
        side = str(name).strip().lower()
        if side not in self.sides:
            raise KeyError(f"unknown {self.name} camera side: {name!r}")
        return self.sides[side]


class CameraConfigError(ValueError):
    """Raised when a camera registry entry is invalid or inconsistent."""


def _as_finite_array(value: Any, shape: tuple[int, ...], context: str) -> np.ndarray:
    try:
        array = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError) as error:
        raise CameraConfigError(f"{context} 必须是数值数组") from error
    if array.shape != shape:
        raise CameraConfigError(
            f"{context} 形状无效：{array.shape}，应为 {shape}")
    if not np.all(np.isfinite(array)):
        raise CameraConfigError(f"{context} 必须全部为有限数值")
    return array


def _resolution(value: Any, context: str) -> tuple[int, int]:
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise CameraConfigError(f"{context} 必须是 [width, height]")
    if any(isinstance(item, bool) or not isinstance(item, (int, float))
           for item in value):
        raise CameraConfigError(f"{context} 必须包含两个数值")
    result = tuple(int(item) for item in value)
    if any(float(item) != int(item) or item <= 0 for item in value):
        raise CameraConfigError(f"{context} 必须是正整数尺寸")
    return result


def _mapping(value: Any, context: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise CameraConfigError(f"{context} 必须是映射")
    return value


def _config_candidates(config_dir: str | os.PathLike | None) -> list[Path]:
    if config_dir:
        root = Path(config_dir).expanduser().resolve()
        # Accept either the registry directory itself or the package config
        # root, which is convenient for deployments that override all camera
        # assets below one directory.
        return [root, root / "cameras"]

    candidates: list[Path] = []
    package_root = Path(__file__).resolve().parents[1]
    candidates.append(package_root / "config" / "cameras")
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(
            Path(get_package_share_directory("uv_camera")) /
            "config" / "cameras")
    except Exception:
        pass
    candidates.append(
        Path.cwd() / "workspace_auv" / "src" / "uv_camera" /
        "config" / "cameras")
    return candidates


def _find_config_dir(config_dir: str | os.PathLike | None, camera: str) -> Path:
    candidates = _config_candidates(config_dir)
    for candidate in candidates:
        if (candidate / f"{camera}.yaml").is_file():
            return candidate
    searched = ", ".join(str(path / f"{camera}.yaml") for path in candidates)
    raise FileNotFoundError(f"camera config not found; searched: {searched}")


def _resolve_npz(raw_path: Any, config_dir: Path) -> Path:
    if not isinstance(raw_path, str) or not raw_path.strip():
        raise CameraConfigError("calibration_npz 必须是非空路径")
    path = Path(os.path.expanduser(raw_path.strip()))
    candidates = [path] if path.is_absolute() else [
        config_dir.parent / path,
        config_dir / path,
    ]
    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()
    raise FileNotFoundError(
        f"calibration_npz not found: {raw_path!r}; "
        f"searched {', '.join(str(item) for item in candidates)}")


def _validate_npz(npz_path: Path, sides: Mapping[str, CameraSideConfig], context: str):
    try:
        with np.load(npz_path, allow_pickle=False) as archive:
            missing = [key for key in _REQUIRED_NPZ_KEYS if key not in archive]
            if missing:
                raise CameraConfigError(
                    f"{context} NPZ 缺少标定字段：{', '.join(missing)}")

            expected_shapes = {
                "R": (3, 3), "P1": (3, 4), "P2": (3, 4),
                "R1": (3, 3), "R2": (3, 3), "Q": (4, 4),
            }
            for key, shape in expected_shapes.items():
                _as_finite_array(
                    archive[key], shape, f"{context}.NPZ.{key}")
            translation = np.asarray(archive["T"], dtype=np.float64).reshape(-1)
            if translation.size != 3 or not np.all(np.isfinite(translation)):
                raise CameraConfigError(
                    f"{context}.NPZ.T 必须包含 3 个有限数值")

            for side in _SIDES:
                npz_matrix = _as_finite_array(
                    archive[f"camera_matrix_{side}"], (3, 3),
                    f"{context}.NPZ.camera_matrix_{side}")
                npz_distortion = np.asarray(
                    archive[f"dist_coeffs_{side}"], dtype=np.float64).reshape(-1)
                if len(npz_distortion) not in (4, 5, 8, 12, 14):
                    raise CameraConfigError(
                        f"{context}.NPZ.dist_coeffs_{side} 长度无效："
                        f"{len(npz_distortion)}")
                if not np.all(np.isfinite(npz_distortion)):
                    raise CameraConfigError(
                        f"{context}.NPZ.dist_coeffs_{side} 必须全部为有限数值")
                yaml_side = sides[side]
                if not np.allclose(
                        npz_matrix, yaml_side.matrix, rtol=1e-5, atol=1e-3):
                    raise CameraConfigError(
                        f"{context} 的 YAML/NPZ {side} camera matrix 不一致")
                if not np.allclose(
                        npz_distortion, yaml_side.distortion,
                        rtol=1e-5, atol=1e-5):
                    raise CameraConfigError(
                        f"{context} 的 YAML/NPZ {side} distortion 不一致")
    except CameraConfigError:
        raise
    except (OSError, ValueError) as error:
        raise CameraConfigError(
            f"无法读取 {context} NPZ：{error}") from error


def _load_yaml(path: Path) -> Mapping[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream) or {}
    if not isinstance(document, Mapping):
        raise CameraConfigError(f"{path} 必须包含 YAML 映射")
    if document.get("schema_version") != 1:
        raise CameraConfigError(f"{path} schema_version 必须为 1")
    return document


def _parse_side(raw: Any, context: str) -> CameraSideConfig:
    entry = _mapping(raw, context)
    matrix = _as_finite_array(entry.get("matrix"), (3, 3), f"{context}.matrix")
    distortion_raw = entry.get("distortion")
    if not isinstance(distortion_raw, (list, tuple)):
        raise CameraConfigError(f"{context}.distortion 必须是列表")
    distortion = np.asarray(distortion_raw, dtype=np.float64).reshape(-1)
    if len(distortion) not in (4, 5, 8, 12, 14) or not np.all(np.isfinite(distortion)):
        raise CameraConfigError(f"{context}.distortion 长度或数值无效")
    translation = _as_finite_array(
        entry.get("translation"), (3,), f"{context}.translation")
    optical_to_body = _as_finite_array(
        entry.get("optical_to_body"), (3, 3), f"{context}.optical_to_body")
    if not np.allclose(
            optical_to_body @ optical_to_body.T, np.eye(3), atol=2e-6):
        raise CameraConfigError(
            f"{context}.optical_to_body 必须是正交旋转矩阵")
    if not np.isclose(np.linalg.det(optical_to_body), 1.0, atol=2e-6):
        raise CameraConfigError(
            f"{context}.optical_to_body 行列式必须为 1")
    if matrix[0, 0] <= 0.0 or matrix[1, 1] <= 0.0 or abs(matrix[2, 2]) <= 1e-12:
        raise CameraConfigError(
            f"{context}.matrix 必须包含有效的正焦距和齐次项")
    return CameraSideConfig(matrix, distortion, translation, optical_to_body)


def load_camera_config(
    camera_name: str,
    profile: str = "auto",
    config_dir: str | os.PathLike | None = None,
) -> CameraConfig:
    """Load and strictly validate one camera's selected profile."""
    camera = str(camera_name).strip().lower()
    if camera not in _CAMERAS:
        raise CameraConfigError(
            f"unknown camera {camera_name!r}; expected one of {_CAMERAS}")
    selected_profile = str(profile or "auto").strip().lower()
    if selected_profile == "auto":
        selected_profile = os.environ.get("UV_CAMERA_PROFILE", "real").strip().lower()
    if selected_profile not in _PROFILES:
        raise CameraConfigError(
            f"unknown camera profile {profile!r}; expected one of {_PROFILES}")

    directory = _find_config_dir(config_dir, camera)
    path = directory / f"{camera}.yaml"
    document = _load_yaml(path)
    if str(document.get("camera", "")).strip().lower() != camera:
        raise CameraConfigError(f"{path} 的 camera 字段不是 {camera!r}")
    profiles = _mapping(document.get("profiles"), f"{path}.profiles")
    raw_profile = _mapping(
        profiles.get(selected_profile), f"{path}.profiles.{selected_profile}")

    capture = _resolution(
        raw_profile.get("capture_resolution"),
        f"{path}.profiles.{selected_profile}.capture_resolution")
    eye = _resolution(
        raw_profile.get("eye_resolution"),
        f"{path}.profiles.{selected_profile}.eye_resolution")
    image_topic = raw_profile.get("image_topic")
    if not isinstance(image_topic, str) or not image_topic.strip():
        raise CameraConfigError("image_topic 必须是非空字符串")
    stereo_info_topic = raw_profile.get("stereo_info_topic", "")
    if not isinstance(stereo_info_topic, str):
        raise CameraConfigError("stereo_info_topic 必须是字符串")
    raw_camera_info_topics = _mapping(
        raw_profile.get("camera_info_topics", {}),
        f"{path}.profiles.{selected_profile}.camera_info_topics")
    camera_info_topics = {}
    for side in _SIDES:
        topic = raw_camera_info_topics.get(side, "")
        if not isinstance(topic, str):
            raise CameraConfigError(
                f"camera_info_topics.{side} 必须是字符串")
        camera_info_topics[side] = topic.strip()
    raw_eye_image_topics = _mapping(
        raw_profile.get("eye_image_topics", {}),
        f"{path}.profiles.{selected_profile}.eye_image_topics")
    eye_image_topics = {}
    for side in _SIDES:
        topic = raw_eye_image_topics.get(side, "")
        if not isinstance(topic, str):
            raise CameraConfigError(
                f"eye_image_topics.{side} 必须是字符串")
        eye_image_topics[side] = topic.strip()
    device = raw_profile.get("device")
    if device is not None and (not isinstance(device, str) or not device.strip()):
        raise CameraConfigError("device 必须是字符串或 null")

    context = f"{path}.profiles.{selected_profile}"
    source = str(raw_profile.get("calibration_source", "npz")).strip().lower()
    if source not in {"npz", "sim_camera_info"}:
        raise CameraConfigError(f"不支持的 calibration_source：{source!r}")
    if selected_profile == "real" and not isinstance(device, str):
        raise CameraConfigError("real profile 必须提供 device")
    if source == "sim_camera_info" and any(
            not camera_info_topics[side] for side in _SIDES):
        raise CameraConfigError(
            "sim_camera_info 必须为左右目提供 camera_info_topics")
    npz_path = _resolve_npz(raw_profile.get("calibration_npz"), directory)
    raw_intrinsics = _mapping(
        raw_profile.get("intrinsics"), f"{context}.intrinsics")
    raw_extrinsics = _mapping(
        raw_profile.get("extrinsics"), f"{context}.extrinsics")
    for section_name, section in (
            ("intrinsics", raw_intrinsics), ("extrinsics", raw_extrinsics)):
        if set(section) != set(_SIDES):
            raise CameraConfigError(
                f"{context}.{section_name} 必须且只能包含 left、right")
    sides = {}
    for side in _SIDES:
        intrinsics = _mapping(
            raw_intrinsics.get(side), f"{path}.profiles.{selected_profile}.intrinsics.{side}")
        extrinsics = _mapping(
            raw_extrinsics.get(side), f"{path}.profiles.{selected_profile}.extrinsics.{side}")
        merged = dict(intrinsics)
        merged.update(extrinsics)
        sides[side] = _parse_side(
            merged, f"{context}.{side}")

    _validate_npz(npz_path, sides, context)
    return CameraConfig(
        name=camera,
        profile=selected_profile,
        capture_resolution=capture,
        eye_resolution=eye,
        image_topic=image_topic.strip(),
        eye_image_topics=eye_image_topics,
        stereo_info_topic=stereo_info_topic.strip(),
        camera_info_topics=camera_info_topics,
        device=device.strip() if isinstance(device, str) else None,
        calibration_source=source,
        calibration_npz=npz_path,
        sides=sides,
    )


def profile_for_mode(sim_mode: bool, requested: str = "auto") -> str:
    """Resolve a node's explicit/automatic camera profile."""
    value = str(requested or "auto").strip().lower()
    if value == "auto":
        return "sim" if bool(sim_mode) else "real"
    if value not in _PROFILES:
        raise CameraConfigError(
            f"unknown camera profile {requested!r}; expected auto, sim or real")
    return value
