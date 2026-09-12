"""Load the shared detector class-ID mapping for all ``uv_*`` packages.

The mapping is deliberately kept next to the weight file instead of being
duplicated in Python modules.  ``uv_task`` and the legacy ``position`` node
both import this module, so a changed model mapping has one source of truth.
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Mapping, Optional

try:
    import yaml
except ImportError as error:  # pragma: no cover - exercised in bad installs
    raise ImportError(
        "uv_camera requires PyYAML to load the shared model mapping") from error


DEFAULT_MAPPING_FILENAME = "robotcup20260901.yaml"


def _normalize_name(value) -> str:
    return str(value or "").strip().lower().replace("-", "_").replace(" ", "_")


def _mapping_candidates():
    override = os.environ.get("UV_MODEL_MAPPING_FILE", "").strip()
    if override:
        yield Path(os.path.expanduser(override))

    # Source tree and colcon build-space symlink.
    package_root = Path(__file__).resolve().parents[1]
    yield package_root / "weights" / DEFAULT_MAPPING_FILENAME

    # Installed package share directory.  Keep this optional so unit tests
    # and source-tree tools work without a sourced ROS environment.
    try:
        from ament_index_python.packages import get_package_share_directory
        yield (Path(get_package_share_directory("uv_camera")) / "weights" /
               DEFAULT_MAPPING_FILENAME)
    except Exception:
        pass

    # Useful for running directly from the workspace without colcon.
    cwd = Path.cwd()
    yield cwd / "workspace_auv" / "src" / "uv_camera" / "weights" / DEFAULT_MAPPING_FILENAME


def _load_mapping():
    candidates = list(_mapping_candidates())
    mapping_path = next((path for path in candidates if path.is_file()), None)
    if mapping_path is None:
        searched = ", ".join(str(path) for path in candidates)
        raise FileNotFoundError(
            "shared detector mapping not found; searched: {}".format(searched))

    with mapping_path.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream) or {}
    if not isinstance(document, dict):
        raise ValueError("shared detector mapping must contain a YAML object")

    raw_classes = document.get("classes")
    if not isinstance(raw_classes, list) or not raw_classes:
        raise ValueError("shared detector mapping must define a non-empty classes list")

    entries = {}
    for raw_entry in raw_classes:
        if not isinstance(raw_entry, dict):
            raise ValueError("each detector class entry must be a YAML object")
        try:
            class_id = int(raw_entry["id"])
            name = _normalize_name(raw_entry["name"])
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(
                "each detector class requires integer id and name") from error
        if class_id < 0 or class_id in entries or not name:
            raise ValueError("detector class IDs must be unique non-negative integers")
        entry = dict(raw_entry)
        entry["id"] = class_id
        entry["name"] = name
        entry["object"] = _normalize_name(entry.get("object", name))
        camera = _normalize_name(entry.get("camera", "any"))
        entry["camera"] = camera if camera in {"front", "down"} else None
        entry["multi_instance"] = bool(entry.get("multi_instance", False))
        entries[class_id] = entry

    expected_ids = list(range(len(entries)))
    if sorted(entries) != expected_ids:
        raise ValueError(
            "shared detector mapping IDs must be contiguous from 0; got {}".format(
                sorted(entries)))

    labels = [entries[class_id]["name"] for class_id in expected_ids]
    if len(set(labels)) != len(labels):
        raise ValueError("shared detector mapping class names must be unique")

    class_ids = {name: class_id for class_id, name in enumerate(labels)}
    aliases = document.get("aliases", {})
    if aliases is not None:
        if not isinstance(aliases, dict):
            raise ValueError("shared detector mapping aliases must be a YAML object")
        for alias, target in aliases.items():
            alias_name = _normalize_name(alias)
            target_name = _normalize_name(target)
            if target_name not in class_ids:
                raise ValueError(
                    "mapping alias {!r} targets unknown class {!r}".format(
                        alias, target))
            if alias_name:
                class_ids[alias_name] = class_ids[target_name]

    return mapping_path, document, entries, tuple(labels), class_ids


MODEL_MAPPING_PATH, MODEL_MAPPING, CLASS_METADATA, DEFAULT_CLASS_NAMES, CLASS_IDS = \
    _load_mapping()


def model_class_id(name, required: bool = True) -> Optional[int]:
    """Return the shared detector ID for a label or configured alias."""
    if isinstance(name, bool):
        class_id = None
    elif isinstance(name, int):
        class_id = name if 0 <= name < len(DEFAULT_CLASS_NAMES) else None
    else:
        class_id = CLASS_IDS.get(_normalize_name(name))

    if class_id is None and required:
        available = ", ".join(DEFAULT_CLASS_NAMES)
        raise ValueError(
            "unknown model class {!r}; available classes: {}".format(
                name, available))
    return class_id


def model_class_name(class_id: int) -> str:
    """Return the detector label for an ID, or ``class_<id>`` if unknown."""
    try:
        return CLASS_METADATA[int(class_id)]["name"]
    except (KeyError, TypeError, ValueError):
        return "class_{}".format(class_id)


def physical_class_name(class_id: int) -> str:
    """Return the physical-object name represented by a detector ID."""
    try:
        return CLASS_METADATA[int(class_id)]["object"]
    except (KeyError, TypeError, ValueError):
        return model_class_name(class_id)


def camera_hint(class_id: int) -> Optional[str]:
    """Return ``front``/``down`` for camera-specific labels, otherwise None."""
    try:
        return CLASS_METADATA[int(class_id)]["camera"]
    except (KeyError, TypeError, ValueError):
        return None


def multi_instance_class_ids() -> set[int]:
    """Return the IDs marked as multi-instance in the shared mapping."""
    return {
        class_id for class_id, entry in CLASS_METADATA.items()
        if entry.get("multi_instance", False)
    }


def configured_class_id(
        params: Optional[Mapping[str, object]], parameter_name: str,
        class_name: str, required: bool = True) -> Optional[int]:
    """Resolve an explicit task ID override or the shared mapping ID."""
    params = params or {}
    value = params.get(parameter_name)
    if value is not None and not (isinstance(value, str) and not value.strip()):
        try:
            class_id = int(value)
        except (TypeError, ValueError) as error:
            raise ValueError(
                "{} must be an integer class ID, got {!r}".format(
                    parameter_name, value)) from error
        if model_class_id(class_id, required=False) is None:
            raise ValueError(
                "{}={} is not present in the shared model mapping".format(
                    parameter_name, class_id))
        return class_id
    return model_class_id(class_name, required=required)
