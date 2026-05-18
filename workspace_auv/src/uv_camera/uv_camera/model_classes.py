"""Shared class-name to detector-ID metadata for the default YOLO model.

Task modules import this small module so they do not duplicate detector IDs or
depend on the ROS object-localizer implementation.  The functions deliberately
use only Python's standard library and work on both ROS 2 Foxy/Python 3.8 and
newer ROS 2 environments.
"""

from typing import Mapping, Optional


# Keep this order synchronized with object_localizer.DEFAULT_CLASS_NAMES.  It
# is the class order used by the default robotcup model and its task configs.
DEFAULT_CLASS_NAMES = (
    "collection_frame_down",
    "collection_frame_front",
    "gate_down",
    "gate_front",
    "guide_line",
    "impact_ball_blue",
    "impact_ball_red",
    "pink_golf",
    "red_ring",
    "target_rack_down",
    "target_rack_front",
    "yellow_golf",
)

CLASS_IDS = {
    name: class_id for class_id, name in enumerate(DEFAULT_CLASS_NAMES)
}


def _normalize_name(value) -> str:
    return str(value or "").strip().lower().replace("-", "_").replace(" ", "_")


def model_class_id(name, required: bool = True) -> Optional[int]:
    """Return the default-model ID for *name*.

    Unknown names return ``None`` when ``required`` is false.  This is useful
    for legacy tasks whose optional marker classes are not present in the
    current model.
    """
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


def configured_class_id(
        params: Optional[Mapping[str, object]], parameter_name: str,
        class_name: str, required: bool = True) -> Optional[int]:
    """Resolve an explicit task class-ID override or the default class ID."""
    params = params or {}
    value = params.get(parameter_name)
    if value is not None and not (isinstance(value, str) and not value.strip()):
        try:
            return int(value)
        except (TypeError, ValueError) as error:
            raise ValueError(
                "{} must be an integer class ID, got {!r}".format(
                    parameter_name, value)) from error
    return model_class_id(class_name, required=required)
