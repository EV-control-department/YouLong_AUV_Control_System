"""Shared camera registry for the perception and task packages.

The registry is deliberately independent of ROS.  Camera YAML files describe
the logical camera (input/eye resolution and calibration), while the vehicle
URDF/TF tree is the source for all camera mounting geometry.  Stereo
rectification is generated at runtime from YAML K/D and the relative TF pose.
"""

from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path
from typing import Any, Mapping

import numpy as np
import yaml


_CAMERAS = ("front", "down")
_MODES = ("sim", "real")
_SIDES = ("left", "right")


@dataclass(frozen=True)
class CameraSideConfig:
    """One eye's intrinsic configuration."""

    matrix: np.ndarray
    distortion: np.ndarray


@dataclass(frozen=True)
class CameraConfig:
    """Validated configuration for one camera pair and one mode."""

    name: str
    mode: str
    capture_resolution: tuple[int, int]
    eye_resolution: tuple[int, int]
    image_topic: str
    eye_image_topics: Mapping[str, str]
    stereo_info_topic: str
    camera_info_topics: Mapping[str, str]
    device: str | None
    calibration_source: str
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


def _canonical_topic(value: str, context: str, *, allow_empty: bool = False) -> str:
    """Validate a topic used by the camera registry.

    The registry is shared by real and simulated camera adapters, therefore a
    non-empty topic must already be in the vehicle namespace.  Empty real
    eye-input topics remain valid because V4L2 supplies those frames locally.
    """
    topic = value.strip()
    if not topic and allow_empty:
        return topic
    if not topic or not topic.startswith('/auv/'):
        raise CameraConfigError(
            f"{context} 必须使用 /auv/ 前缀")
    return topic


def _config_candidates(config_dir: str | os.PathLike | None) -> list[Path]:
    if config_dir:
        root = Path(config_dir).expanduser().resolve()
        # Accept either the registry directory itself or the package config
        # root, which is convenient for deployments that override all camera
        # assets below one directory.
        return [root, root / "stereos"]

    candidates: list[Path] = []
    package_root = Path(__file__).resolve().parents[1]
    candidates.append(package_root / "stereos")
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(
            Path(get_package_share_directory("uv_camera")) /
            "stereos")
    except Exception:
        pass
    candidates.append(
        Path.cwd() / "workspace_auv" / "src" / "uv_camera" /
        "stereos")
    return candidates


def _find_config_dir(config_dir: str | os.PathLike | None, camera: str) -> Path:
    candidates = _config_candidates(config_dir)
    for candidate in candidates:
        if (candidate / f"{camera}.yaml").is_file():
            return candidate
    searched = ", ".join(str(path / f"{camera}.yaml") for path in candidates)
    raise FileNotFoundError(f"camera stereo configuration not found; searched: {searched}")




def _load_yaml(path: Path) -> Mapping[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream) or {}
    if not isinstance(document, Mapping):
        raise CameraConfigError(f"{path} 必须包含 YAML 映射")
    if document.get("schema_version") != 2:
        raise CameraConfigError(
            f"{path} schema_version 必须为 2；旧配置包含相机外参，"
            "请删除 extrinsics/calibration_npz 并迁移到 URDF/TF")
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
    if matrix[0, 0] <= 0.0 or matrix[1, 1] <= 0.0 or abs(matrix[2, 2]) <= 1e-12:
        raise CameraConfigError(
            f"{context}.matrix 必须包含有效的正焦距和齐次项")
    forbidden = {"translation", "optical_to_body"}.intersection(entry)
    if forbidden:
        fields = ", ".join(sorted(forbidden))
        raise CameraConfigError(
            f"{context} 仍包含外参字段 {fields}；请迁移到 URDF/TF")
    return CameraSideConfig(matrix, distortion)


def load_camera_config(
    camera_name: str,
    mode: str = "auto",
    config_dir: str | os.PathLike | None = None,
) -> CameraConfig:
    """Load and strictly validate one camera's selected mode."""
    camera = str(camera_name).strip().lower()
    if camera not in _CAMERAS:
        raise CameraConfigError(
            f"unknown camera {camera_name!r}; expected one of {_CAMERAS}")
    selected_mode = str(mode or "auto").strip().lower()
    if selected_mode == "auto":
        selected_mode = os.environ.get("UV_CAMERA_MODE", "real").strip().lower()
    if selected_mode not in _MODES:
        raise CameraConfigError(
            f"unknown camera mode {mode!r}; expected one of {_MODES}")

    directory = _find_config_dir(config_dir, camera)
    path = directory / f"{camera}.yaml"
    document = _load_yaml(path)
    if str(document.get("camera", "")).strip().lower() != camera:
        raise CameraConfigError(f"{path} 的 camera 字段不是 {camera!r}")
    modes = _mapping(document.get("modes"), f"{path}.modes")
    raw_mode = _mapping(
        modes.get(selected_mode), f"{path}.modes.{selected_mode}")

    capture = _resolution(
        raw_mode.get("capture_resolution"),
        f"{path}.modes.{selected_mode}.capture_resolution")
    eye = _resolution(
        raw_mode.get("eye_resolution"),
        f"{path}.modes.{selected_mode}.eye_resolution")
    context = f"{path}.modes.{selected_mode}"
    if capture[0] != 2 * eye[0] or capture[1] != eye[1]:
        raise CameraConfigError(
            f"{path}.modes.{selected_mode} 的 capture_resolution "
            "必须是左右目 eye_resolution 的水平拼接尺寸")
    image_topic = raw_mode.get("image_topic")
    if not isinstance(image_topic, str) or not image_topic.strip():
        raise CameraConfigError("image_topic 必须是非空字符串")
    image_topic = _canonical_topic(image_topic, f"{context}.image_topic")
    stereo_info_topic = raw_mode.get("stereo_info_topic", "")
    if not isinstance(stereo_info_topic, str):
        raise CameraConfigError("stereo_info_topic 必须是字符串")
    stereo_info_topic = _canonical_topic(
        stereo_info_topic, f"{context}.stereo_info_topic", allow_empty=True)
    raw_camera_info_topics = _mapping(
        raw_mode.get("camera_info_topics", {}),
        f"{path}.modes.{selected_mode}.camera_info_topics")
    camera_info_topics = {}
    for side in _SIDES:
        topic = raw_camera_info_topics.get(side, "")
        if not isinstance(topic, str):
            raise CameraConfigError(
                f"camera_info_topics.{side} 必须是字符串")
        camera_info_topics[side] = _canonical_topic(
            topic, f"{context}.camera_info_topics.{side}", allow_empty=True)
    raw_eye_image_topics = _mapping(
        raw_mode.get("eye_image_topics", {}),
        f"{path}.modes.{selected_mode}.eye_image_topics")
    eye_image_topics = {}
    for side in _SIDES:
        topic = raw_eye_image_topics.get(side, "")
        if not isinstance(topic, str):
            raise CameraConfigError(
                f"eye_image_topics.{side} 必须是字符串")
        eye_image_topics[side] = _canonical_topic(
            topic, f"{context}.eye_image_topics.{side}", allow_empty=True)
    device = raw_mode.get("device")
    if device is not None and (not isinstance(device, str) or not device.strip()):
        raise CameraConfigError("device 必须是字符串或 null")

    if "calibration_npz" in raw_mode:
        raise CameraConfigError(
            f"{context}.calibration_npz 已废弃；双目标定矩阵由 K、D 和 URDF/TF 运行时生成")
    source = str(raw_mode.get("calibration_source", "yaml")).strip().lower()
    if source not in {"yaml", "sim_camera_info"}:
        if source == "npz":
            raise CameraConfigError(
                f"{context}.calibration_source=npz 已废弃；请迁移到 YAML K/D 和 URDF/TF")
        raise CameraConfigError(f"不支持的 calibration_source：{source!r}")
    if selected_mode == "real" and not isinstance(device, str):
        raise CameraConfigError("real mode 必须提供 device")
    if source == "sim_camera_info" and any(
            not camera_info_topics[side] for side in _SIDES):
        raise CameraConfigError(
            "sim_camera_info 必须为左右目提供 camera_info_topics")
    raw_intrinsics = _mapping(
        raw_mode.get("intrinsics"), f"{context}.intrinsics")
    if set(raw_intrinsics) != set(_SIDES):
        raise CameraConfigError(
            f"{context}.intrinsics 必须且只能包含 left、right")
    if "extrinsics" in raw_mode:
        raise CameraConfigError(
            f"{context}.extrinsics 已废弃；相机安装外参必须统一从 URDF/TF 获取")
    sides = {}
    for side in _SIDES:
        intrinsics = _mapping(
            raw_intrinsics.get(side), f"{path}.modes.{selected_mode}.intrinsics.{side}")
        sides[side] = _parse_side(
            intrinsics, f"{context}.intrinsics.{side}")
    return CameraConfig(
        name=camera,
        mode=selected_mode,
        capture_resolution=capture,
        eye_resolution=eye,
        image_topic=image_topic,
        eye_image_topics=eye_image_topics,
        stereo_info_topic=stereo_info_topic,
        camera_info_topics=camera_info_topics,
        device=device.strip() if isinstance(device, str) else None,
        calibration_source=source,
        sides=sides,
    )


def camera_mode_for_sim_mode(sim_mode: bool, requested: str = "auto") -> str:
    """Resolve a node's explicit/automatic camera mode."""
    value = str(requested or "auto").strip().lower()
    if value == "auto":
        return "sim" if bool(sim_mode) else "real"
    if value not in _MODES:
        raise CameraConfigError(
            f"unknown camera mode {requested!r}; expected auto, sim or real")
    return value
