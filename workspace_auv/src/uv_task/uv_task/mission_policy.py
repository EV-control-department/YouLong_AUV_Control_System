"""ROS-independent mission override policy helpers."""

from __future__ import annotations

from typing import Any


def select_failure_override(
        task: dict[str, Any], failure_code: str) -> dict[str, Any] | None:
    """Select an exact failure hook, falling back to ``default``."""
    profiles = task.get("on_failure", {})
    if failure_code in profiles:
        return profiles[failure_code]
    return profiles.get("default")


def apply_failure_override(
        base_params: dict[str, Any],
        initial_pose: dict[str, Any] | None,
        override: dict[str, Any] | None,
) -> tuple[dict[str, Any], dict[str, Any] | None]:
    """Apply one pending override to the next task.

    Parameters are merged over the task's already-resolved values.  A pose is
    replaced only when the failure profile explicitly contains ``pose``;
    otherwise the next task's own initial pose remains active.
    """
    params = dict(base_params)
    pose = initial_pose
    if override is not None:
        params.update(override.get("params", {}))
        if "pose" in override:
            pose = override["pose"]
    return params, pose
