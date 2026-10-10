"""Strict YAML mission/task configuration loading for :mod:`uv_task`.

The files intentionally use ordinary YAML only.  Task parameter groups are a
human-facing representation; the loader validates and flattens them to the
legacy keys consumed by the task implementations.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import yaml

from uv_task.golf_search_config import SEARCH_SCHEMA, validate_search_params
from uv_task.drop_search_config import DROP_SEARCH_SCHEMA, validate_drop_search_params
from uv_task.hit_ball_config import SCHEMA as HIT_SCHEMA, validate_hit_params

from uv_task.gate_config import SCHEMA as GATE_SCHEMA, ALIASES as GATE_ALIASES, validate_gate_params



class ConfigError(ValueError):
    """Raised when a mission or task YAML file is invalid."""


_MOTION_COMMANDS = {
    "SET",
    "WMOVE",
    "BMOVE",
    "WTRAVEL",
    "BTRAVEL",
}


def _validate_pose(value: Any, *, context: str) -> dict[str, Any]:
    """Validate and normalize a mission-level BasicMotion pose override."""
    if not isinstance(value, dict):
        raise ConfigError(f"{context} 必须是映射")
    unknown = set(value) - {"command", "axes", "target"}
    if unknown:
        raise ConfigError(
            f"{context} 存在未知键：{sorted(unknown)}")

    command = value.get("command")
    if not isinstance(command, str):
        raise ConfigError(f"{context}.command 必须是字符串")
    command = command.strip().upper()
    if command not in _MOTION_COMMANDS:
        raise ConfigError(
            f"{context}.command 无效：{command!r}；"
            f"应为 {sorted(_MOTION_COMMANDS)}")

    axes = value.get("axes", "")
    if not isinstance(axes, str):
        raise ConfigError(f"{context}.axes 必须是字符串")
    axes = axes.strip().lower()
    axis_text = axes.replace("rz", "")
    if "rz" in axes and axes.count("rz") != 1:
        raise ConfigError(f"{context}.axes 中 rz 最多只能出现一次")
    if any(axis not in "xyz" for axis in axis_text):
        raise ConfigError(
            f"{context}.axes 无效：{axes!r}；只能包含 x、y、z、rz")
    if len(set(axis_text)) != len(axis_text):
        raise ConfigError(f"{context}.axes 不能重复指定同一坐标轴")

    target = value.get("target")
    if not isinstance(target, list) or len(target) != 4:
        raise ConfigError(
            f"{context}.target 必须是包含 4 个数值的列表 [x,y,z,yaw]")
    normalized_target = []
    for index, item in enumerate(target):
        if isinstance(item, bool) or not isinstance(item, (int, float)):
            raise ConfigError(
                f"{context}.target[{index}] 必须是数值")
        if not math.isfinite(float(item)):
            raise ConfigError(
                f"{context}.target[{index}] 必须是有限数值")
        normalized_target.append(float(item))

    return {
        "command": command,
        "axes": axes,
        "target": normalized_target,
    }


class _UniqueKeyLoader(yaml.SafeLoader):
    """SafeLoader variant that rejects duplicate mapping keys."""


def _construct_unique_mapping(loader, node, deep=False):
    mapping = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in mapping:
            raise ConfigError(f"YAML 文件存在重复键：{key!r}")
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


_UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
    _construct_unique_mapping,
)


# ``None`` means that a task has no configurable parameters.  A tuple denotes
# a homogeneous list.  Floats accept YAML integers because ``1`` is a natural
# spelling for a value consumed as ``float(1)`` by the existing task code.
TASK_SCHEMAS: dict[str, dict[str, Any]] = {
    "start": {},
    "basic_motion_test": {
        "stage": str, "reset_origin": bool, "test_depth": bool,
        "distance_m": float, "depth_delta_m": float, "yaw_delta_deg": float,
        "speed_mps": float, "yaw_rate_deg_s": float, "pulse_seconds": float,
        "action_timeout": float, "settle_seconds": float,
        "feedback_timeout": float, "horizontal_limit_m": float,
        "vertical_limit_m": float, "min_battery_voltage": float,
        "external_battery_voltage": float,
        "max_speed_mps": float,
    },
    "return_origin": {
        "ascent_target_z_m": float,
        "ascent_speed_mps": float,
        "ascent_publish_period": float,
        "ascent_timeout": float,
    },
    "btravelx": {"dx": float},
    "bline": {"dx": float, "dy": float, "dz": float,
              "speed_mps": float, "timeout": float},
    "setz": {"z": float},
    "26rb_hit_balls": HIT_SCHEMA,
    "26rb_gate_task": GATE_SCHEMA,
    "26rb_find_collection_frame": {
        "platform_name": str,
        "rack_name": str,
        "look_order": (list, str),
        "min_confidence": float,
        "min_observations": int,
        "allow_stale_targets": bool,
        "max_target_age_seconds": float,
        "timeout": float,
        "rotate_timeout": float,
        "look_settle_seconds": float,
        "confirm_seconds": float,
        "confirm_timeout": float,
        "scan_start_yaw_deg": float,
        "scan_end_yaw_deg": float,
        "scan_yaw_step_deg": float,
        "scan_settle_seconds": float,
        "collection_depth_m": float,
        "move_timeout": float,
    },
    "26rb_grab_golf": {
        **SEARCH_SCHEMA,
        "golf_color": str,
        "collection_frame_class": str,
        "collection_frame_observe_seconds": float,
        "golf_observe_seconds": float,
        "detection_timeout": float,
        "camera_priority_seconds": float,
        "horizontal_servo_timeout": float,
        "horizontal_servo_period": float,
        "horizontal_servo_log_period": float,
        "pixel_tolerance_fraction": float,
        "horizontal_hold_seconds": float,
        "projection_depth_m": float,
        "horizontal_servo_gain": float,
        "max_horizontal_step_m": float,  # Legacy input accepted; velocity uses max_speed_mps.
        "horizontal_max_speed_mps": float,
        "depth_hold_gain": float,
        "depth_hold_max_speed_mps": float,
        "depth_hold_tolerance_m": float,
        "position_command_timeout": float,
        "claw_prepare_angle_rad": float,
        "claw_prepare_repeat_count": int,
        "claw_prepare_repeat_period": float,
        "light_pulse_seconds": float,
        "light_gap_seconds": float,
        "pre_descent_settle_seconds": float,
        "descent_speed_mps": float,
        "descent_duration_seconds": float,
        "ascent_speed_mps": float,
        "ascent_duration_seconds": float,
        "vertical_publish_period": float,
        "return_timeout": float,
        "verification_timeout": float,
        "verification_absence_hold_seconds": float,
        "max_grab_retries": int,
    },
    "26rb_drop_ball_target_rack": {
        **DROP_SEARCH_SCHEMA,
        "observation_seconds": float,
        "alignment_timeout": float,
        "alignment_observation_timeout": float,
        "alignment_settle_seconds": float,
        "release_angle_deg": float,
        "release_repeat_count": int,
        "release_repeat_period": float,
        "release_settle_seconds": float,
        "ring_release_angle_deg": float,
        "ring_release_repeat_count": int,
        "ring_release_repeat_period": float,
        "ring_release_settle_seconds": float,
        "light_pulse_seconds": float,
        "light_gap_seconds": float,
        # Retain parsing compatibility for old task files. The drop task no
        # longer consumes the track-selection/coarse-position/light keys.
        "frame_name": str,
        "light_color": str,
        "above_z_m": float,
        "min_confidence": float,
        "min_observations": int,
        "allow_stale_targets": bool,
        "target_timeout": float,
        "move_timeout": float,
        "horizontal_servo_timeout": float,
        "horizontal_servo_stable_seconds": float,
        "horizontal_servo_position_tolerance_m": float,
        "horizontal_servo_target_tolerance_m": float,
        "horizontal_servo_period": float,
        "horizontal_servo_min_update_m": float,
        "horizontal_command_timeout": float,
        # Monocular detections drive the pixel servo; alignment uses only the
        # successful camera's fixed mounting offset. Legacy stereo keys stay
        # readable for old task files but are no longer consumed.
        "down_visual_servo_timeout": float,
        "down_visual_servo_stable_seconds": float,
        "down_detection_timeout": float,
        "down_camera_priority_seconds": float,
        "down_pixel_tolerance_fraction": float,
        "down_epipolar_vertical_tolerance_fraction": float,
        "down_projection_depth_m": float,
        "down_visual_servo_gain": float,
        "down_visual_servo_max_step_m": float,  # Legacy positional-servo input.
        "down_visual_servo_max_speed_mps": float,
        "down_depth_hold_gain": float,
        "down_depth_hold_max_speed_mps": float,
        "down_depth_hold_tolerance_m": float,
        "down_visual_servo_period": float,
        "down_visual_command_timeout": float,
        "light_hold_seconds": float,
    },
}


# Short names keep the YAML readable where the runtime key carries the
# implementation detail (for example ``servo.timeout`` becomes
# ``horizontal_servo_timeout``).  The resulting keys are still exactly the
# names used by the existing task classes.
TASK_SCHEMAS["26rb_grab_ball_ring"] = {
    **{key: value for key, value in TASK_SCHEMAS["26rb_grab_golf"].items()
       if key not in {"max_grab_retries", "search_cruise_depth_m",
                      "ascent_speed_mps", "ascent_duration_seconds", "descent_duration_seconds"}},
    "work_depth_m": float,
    "golf_grab_depth_m": float, "ring_grab_depth_m": float,
    "retry_depth_step_m": float, "descent_timeout": float,
    "golf_max_attempts": int, "ring_max_attempts": int, "ring_class": str,
    "ring_observe_seconds": float, "ring_orientation_timeout": float,
    "ring_orientation_min_quality": float, "ring_orientation_min_samples": int,
    "ring_orientation_spread_deg": float,
    "ring_yaw_tolerance_deg": float, "ring_yaw_gain": float,
    "ring_max_yaw_rate_deg_s": float,
    "ring_open_angle_deg": float, "ring_close_angle_deg": float,
    "ring_servo_repeat_count": int, "ring_servo_repeat_period": float,
    "ring_servo_settle_seconds": float,
}

PARAMETER_ALIASES = {
    "26rb_hit_balls": {},
    "26rb_gate_task": GATE_ALIASES,
    "26rb_grab_golf": {
        "servo.camera_priority_seconds": "camera_priority_seconds",
        "claw.prepare_angle_rad": "claw_prepare_angle_rad",
        "claw.prepare_repeat_count": "claw_prepare_repeat_count",
        "claw.prepare_repeat_period": "claw_prepare_repeat_period",
        "collection_frame.observe_seconds": "collection_frame_observe_seconds",
        "servo.timeout": "horizontal_servo_timeout",
        "servo.period": "horizontal_servo_period",
        "servo.log_period": "horizontal_servo_log_period",
        "servo.hold_seconds": "horizontal_hold_seconds",
        "servo.gain": "horizontal_servo_gain",
        "servo.max_step_m": "max_horizontal_step_m",
        "servo.max_speed_mps": "horizontal_max_speed_mps",
        "servo.depth_gain": "depth_hold_gain",
        "servo.max_vertical_speed_mps": "depth_hold_max_speed_mps",
        "servo.depth_tolerance_m": "depth_hold_tolerance_m",
        "light.pulse_seconds": "light_pulse_seconds",
        "light.gap_seconds": "light_gap_seconds",
        "settle_seconds": "pre_descent_settle_seconds",
        "descent.speed_mps": "descent_speed_mps",
        "descent.duration_seconds": "descent_duration_seconds",
        "ascent.speed_mps": "ascent_speed_mps",
        "ascent.duration_seconds": "ascent_duration_seconds",
        "verification.timeout": "verification_timeout",
        "verification.absence_hold_seconds": "verification_absence_hold_seconds",
        "verification.max_retries": "max_grab_retries",
    },
    "26rb_drop_ball_target_rack": {
        "observation.seconds": "observation_seconds",
        "alignment.timeout": "alignment_timeout",
        "alignment.observation_timeout": "alignment_observation_timeout",
        "alignment.settle_seconds": "alignment_settle_seconds",
        "release.angle_deg": "release_angle_deg",
        "release.repeat_count": "release_repeat_count",
        "release.repeat_period": "release_repeat_period",
        "release.settle_seconds": "release_settle_seconds",
        "ring_release.angle_deg": "ring_release_angle_deg",
        "ring_release.repeat_count": "ring_release_repeat_count",
        "ring_release.repeat_period": "ring_release_repeat_period",
        "ring_release.settle_seconds": "ring_release_settle_seconds",
        "light.pulse_seconds": "light_pulse_seconds",
        "light.gap_seconds": "light_gap_seconds",
        "target.timeout": "target_timeout",
        "servo.timeout": "horizontal_servo_timeout",
        "servo.stable_seconds": "horizontal_servo_stable_seconds",
        "servo.position_tolerance_m": "horizontal_servo_position_tolerance_m",
        "servo.target_tolerance_m": "horizontal_servo_target_tolerance_m",
        "servo.period": "horizontal_servo_period",
        "servo.min_update_m": "horizontal_servo_min_update_m",
        "servo.command_timeout": "horizontal_command_timeout",
        "visual_servo.timeout": "down_visual_servo_timeout",
        "visual_servo.stable_seconds": "down_visual_servo_stable_seconds",
        "visual_servo.detection_timeout": "down_detection_timeout",
        "visual_servo.camera_priority_seconds": "down_camera_priority_seconds",
        "visual_servo.pixel_tolerance_fraction":
            "down_pixel_tolerance_fraction",
        "visual_servo.epipolar_vertical_tolerance_fraction":
            "down_epipolar_vertical_tolerance_fraction",
        "visual_servo.projection_depth_m": "down_projection_depth_m",
        "visual_servo.gain": "down_visual_servo_gain",
        "visual_servo.max_step_m": "down_visual_servo_max_step_m",
        "visual_servo.max_speed_mps": "down_visual_servo_max_speed_mps",
        "visual_servo.depth_gain": "down_depth_hold_gain",
        "visual_servo.max_vertical_speed_mps": "down_depth_hold_max_speed_mps",
        "visual_servo.depth_tolerance_m": "down_depth_hold_tolerance_m",
        "visual_servo.period": "down_visual_servo_period",
        "visual_servo.command_timeout": "down_visual_command_timeout",
        "light.color": "light_color",
        "light.hold_seconds": "light_hold_seconds",
    },
}


def _read_yaml(path: Path) -> dict[str, Any]:
    try:
        with path.open("r", encoding="utf-8") as stream:
            data = yaml.load(stream, Loader=_UniqueKeyLoader)
    except (OSError, yaml.YAMLError, ConfigError) as exc:
        raise ConfigError(f"YAML 文件解析失败：{path}：{exc}") from exc
    if not isinstance(data, dict):
        raise ConfigError(f"{path}：根节点必须是 YAML 映射")
    return data


PARAMETER_ALIASES["26rb_grab_ball_ring"] = {
    **{key: value for key, value in PARAMETER_ALIASES["26rb_grab_golf"].items()
       if value not in {"max_grab_retries", "ascent_speed_mps", "ascent_duration_seconds"}},
    # Legacy YAML spelling maps to the same work-depth key; specifying both
    # spellings is a duplicate configuration error, never two depth targets.
    "search.cruise_depth_m": "work_depth_m",
    "search_cruise_depth_m": "work_depth_m",
    "golf.grab_depth_m": "golf_grab_depth_m", "ring.grab_depth_m": "ring_grab_depth_m",
    "descent.retry_depth_step_m": "retry_depth_step_m",
    "descent.timeout": "descent_timeout",
    "descent.duration_seconds": "descent_timeout",  # Legacy spelling now means a timeout.
    "descent_duration_seconds": "descent_timeout",
    "golf.max_attempts": "golf_max_attempts", "ring.max_attempts": "ring_max_attempts",
    "ring.class": "ring_class", "ring.observe_seconds": "ring_observe_seconds",
    "ring.orientation_timeout": "ring_orientation_timeout",
    "ring.orientation_min_quality": "ring_orientation_min_quality",
    "ring.orientation_min_samples": "ring_orientation_min_samples",
    "ring.orientation_spread_deg": "ring_orientation_spread_deg",
    "ring.yaw_tolerance_deg": "ring_yaw_tolerance_deg", "ring.yaw_gain": "ring_yaw_gain",
    "ring.max_yaw_rate_deg_s": "ring_max_yaw_rate_deg_s",
    "ring.servo.open_angle_deg": "ring_open_angle_deg",
    "ring.servo.close_angle_deg": "ring_close_angle_deg",
    "ring.servo.repeat_count": "ring_servo_repeat_count",
    "ring.servo.repeat_period": "ring_servo_repeat_period",
    "ring.servo.settle_seconds": "ring_servo_settle_seconds",
}

def _flatten_params(
    value: dict[str, Any],
    schema: dict[str, Any],
    *,
    task_name: str,
    path=(),
):
    result: dict[str, Any] = {}
    for key, child in value.items():
        if not isinstance(key, str) or not key.strip():
            raise ConfigError("参数键必须是非空字符串")
        current_path = path + (key,)
        if isinstance(child, dict):
            if not child:
                raise ConfigError(
                    f"参数组 {'.'.join(current_path)!r} 不能为空")
            nested = _flatten_params(
                child, schema, task_name=task_name, path=current_path)
            for canonical, nested_value in nested.items():
                if canonical in result:
                    raise ConfigError(
                        f"展开后参数重复：{canonical!r}")
                result[canonical] = nested_value
            continue

        alias = PARAMETER_ALIASES.get(task_name, {}).get(
            ".".join(current_path))
        # A declared dotted alias is authoritative.  This keeps names such
        # as ``alignment.yaw_pid.stable_seconds`` from becoming ambiguous
        # with group-independent runtime keys.
        if alias in schema:
            prefixed_candidates = [alias]
        else:
            prefixed_candidates = []
            for index in range(len(path), 0, -1):
                candidate = "_".join(path[:index] + (key,))
                if candidate in schema and candidate not in prefixed_candidates:
                    prefixed_candidates.append(candidate)
        # Prefer the group-specific schema key when both ``timeout`` and
        # ``search_timeout`` exist.  A bare key remains valid for groups such
        # as ``velocity.max_forward_speed_mps`` whose runtime key has no
        # ``velocity_`` prefix.
        candidates = prefixed_candidates or ([key] if key in schema else [])
        if len(candidates) != 1:
            dotted = ".".join(current_path)
            if not candidates:
                raise ConfigError(f"未知参数：{dotted!r}")
            raise ConfigError(f"参数存在歧义：{dotted!r}：{candidates}")
        canonical = candidates[0]
        if canonical in result:
            raise ConfigError(f"展开后参数重复：{canonical!r}")
        result[canonical] = child
    return result


def _value_matches(value: Any, expected: Any) -> bool:
    if isinstance(expected, tuple) and expected[0] is list:
        return isinstance(value, list) and all(_value_matches(item, expected[1]) for item in value)
    if expected is float:
        return isinstance(value, (int, float)) and not isinstance(value, bool)
    if expected is int:
        return isinstance(value, int) and not isinstance(value, bool)
    if expected is bool:
        return isinstance(value, bool)
    if expected is str:
        return isinstance(value, str)
    return False


def _type_name(expected: Any) -> str:
    if isinstance(expected, tuple) and expected[0] is list:
        return f"list[{_type_name(expected[1])}]"
    return expected.__name__


def _validate_params(task_name: str, params: dict[str, Any],
                     class_registry=None, *, complete=False) -> dict[str, Any]:
    if task_name not in TASK_SCHEMAS:
        raise ConfigError(f"未知任务：{task_name!r}")
    flattened = _flatten_params(
        params, TASK_SCHEMAS[task_name], task_name=task_name)
    for key, value in flattened.items():
        expected = TASK_SCHEMAS[task_name][key]
        if isinstance(expected, tuple) and expected[0] is list:
            valid = isinstance(value, list) and all(
                _value_matches(item, expected[1]) for item in value)
        else:
            valid = _value_matches(value, expected)
        if not valid:
            raise ConfigError(
                f"参数 {key!r} 的类型为 {type(value).__name__}；"
                f"应为 {_type_name(expected)}")
    if task_name == "26rb_gate_task":
        try:
            validate_gate_params(flattened, complete=complete)
        except ValueError as exc:
            raise ConfigError(str(exc)) from exc
    if task_name == "26rb_hit_balls":
        try:
            validate_hit_params(flattened)
        except ValueError as exc:
            raise ConfigError(str(exc)) from exc
    if (class_registry is not None
            and task_name == "26rb_hit_balls" and "order" in flattened):
        values = flattened["order"]
        values = values if isinstance(values, list) else [values]
        allowed = {
            entry.name for entry in getattr(class_registry, "entries", ())
            if entry.object.startswith("impact_ball_")
        }
        if any(value not in allowed for value in values):
            raise ConfigError(
                "26rb_hit_balls.order 必须使用共享模型映射中的 "
                "canonical impact-ball class name")
    if task_name in ("26rb_grab_golf", "26rb_grab_ball_ring"):
        try:
            search_params = flattened
            if task_name == "26rb_grab_ball_ring":
                search_params = {**flattened, 'search_cruise_depth_m': flattened.get('work_depth_m', 0.2)}
            validate_search_params(search_params)
        except ValueError as exc:
            raise ConfigError(str(exc)) from exc
    if task_name == "26rb_drop_ball_target_rack":
        try:
            validate_drop_search_params(flattened)
            from importlib import import_module
            import_module('uv_task.26rb_drop_ball_target_rack').validate_ring_release_params(flattened)
        except ValueError as exc:
            raise ConfigError(str(exc)) from exc
    if task_name in ("26rb_grab_golf", "26rb_grab_ball_ring") and "golf_color" in flattened:
        allowed = {"pink_golf", "yellow_golf"}
        if class_registry is not None:
            allowed &= {
                entry.name for entry in getattr(class_registry, "entries", ())
                if entry.object in {"pink_golf", "yellow_golf"}
            }
        if flattened["golf_color"] not in allowed:
            raise ConfigError(
                "26rb_grab_golf.golf_color 必须为共享模型映射中的 "
                "pink_golf 或 yellow_golf 高尔夫球类别")
    if task_name == "26rb_grab_ball_ring":
        from importlib import import_module
        try:
            import_module('uv_task.26rb_grab_ball_ring').validate_combined_params(flattened, complete=complete)
        except ValueError as exc:
            raise ConfigError(str(exc)) from exc
    return flattened


def _resolve_task_config(mission_path: Path, config: str | None, task_name: str) -> Path:
    if config:
        candidate = Path(config).expanduser()
        if not candidate.is_absolute():
            candidate = mission_path.parent / candidate
        return candidate.resolve()
    from ament_index_python.packages import get_package_share_directory

    package_config = Path(get_package_share_directory("uv_task")) / "config"
    return (package_config / "tasks" / f"{task_name}.yaml").resolve()


def _load_task_defaults(path: Path, task_name: str) -> dict[str, Any]:
    data = _read_yaml(path)
    unknown = set(data) - {"task", "params"}
    if unknown:
        raise ConfigError(f"{path}：任务文件存在未知键：{sorted(unknown)}")
    if data.get("task") != task_name:
        raise ConfigError(
            f"{path}：task 必须为 {task_name!r}，实际为 {data.get('task')!r}")
    params = data.get("params", {})
    if not isinstance(params, dict):
        raise ConfigError(f"{path}：params 必须是 YAML 映射")
    return params


def load_task(path: str | Path, class_registry=None) -> list[dict[str, Any]]:
    """Load one standalone task YAML as a one-item runner task list."""
    task_path = Path(path).expanduser().resolve()
    data = _read_yaml(task_path)
    unknown = set(data) - {"task", "params"}
    if unknown:
        raise ConfigError(f"{task_path}：任务文件存在未知键：{sorted(unknown)}")

    task_name = data.get("task")
    if not isinstance(task_name, str) or task_name not in TASK_SCHEMAS:
        raise ConfigError(
            f"{task_path}：task 名称未知或无效：{task_name!r}")
    params = data.get("params", {})
    if not isinstance(params, dict):
        raise ConfigError(f"{task_path}：params 必须是 YAML 映射")

    return [{"name": task_name, "params": _validate_params(
        task_name, params, class_registry, complete=True)}]


def load_mission(path: str | Path, class_registry=None) -> list[dict[str, Any]]:
    """Load a mission with initial and one-hop failure overrides.

    ``params`` is retained as a legacy spelling for a mission-level initial
    parameter override.  The explicit ``initial.params`` block wins over it.
    Failure override parameters are validated against the immediately
    following task because that is where they are consumed.
    """
    mission_path = Path(path).expanduser().resolve()
    data = _read_yaml(mission_path)
    unknown = set(data) - {"mission"}
    if unknown:
        raise ConfigError(f"{mission_path}：根节点存在未知键：{sorted(unknown)}")
    mission = data.get("mission")
    if not isinstance(mission, dict):
        raise ConfigError(f"{mission_path}：缺少 mission 映射")
    unknown = set(mission) - {"name", "tasks"}
    if unknown:
        raise ConfigError(
            f"{mission_path}：mission 存在未知键：{sorted(unknown)}")
    if not isinstance(mission.get("name"), str) or not mission["name"].strip():
        raise ConfigError(f"{mission_path}：mission.name 必须是非空字符串")
    entries = mission.get("tasks")
    if not isinstance(entries, list) or not entries:
        raise ConfigError(f"{mission_path}：mission.tasks 必须是非空列表")

    entries_data = []
    for index, entry in enumerate(entries, start=1):
        if not isinstance(entry, dict):
            raise ConfigError(f"{mission_path}：第 {index} 个任务必须是映射")
        unknown = set(entry) - {
            "name", "config", "params", "initial", "on_failure",
        }
        if unknown:
            raise ConfigError(
                f"{mission_path}：第 {index} 个任务存在未知键：{sorted(unknown)}")
        task_name = entry.get("name")
        if not isinstance(task_name, str) or task_name not in TASK_SCHEMAS:
            raise ConfigError(
                f"{mission_path}：第 {index} 个任务名称未知：{task_name!r}")
        config = entry.get("config")
        if config is not None and not isinstance(config, str):
            raise ConfigError(f"{mission_path}：第 {index} 个任务的 config 必须是字符串")
        overrides = entry.get("params", {})
        if not isinstance(overrides, dict):
            raise ConfigError(f"{mission_path}：第 {index} 个任务的 params 必须是映射")

        initial = entry.get("initial", {})
        if not isinstance(initial, dict):
            raise ConfigError(f"{mission_path}：第 {index} 个任务的 initial 必须是映射")
        unknown = set(initial) - {"params", "pose", "poses"}
        if unknown:
            raise ConfigError(
                f"{mission_path}：第 {index} 个任务的 initial 存在未知键："
                f"{sorted(unknown)}")
        if "pose" in initial and "poses" in initial:
            raise ConfigError(
                f"{mission_path}：第 {index} 个任务的 initial.pose 与 "
                "initial.poses 不能同时配置")
        initial_params = initial.get("params", {})
        if not isinstance(initial_params, dict):
            raise ConfigError(
                f"{mission_path}：第 {index} 个任务的 initial.params 必须是映射")
        initial_pose = None
        if "pose" in initial:
            initial_pose = _validate_pose(
                initial["pose"],
                context=f"{mission_path}：第 {index} 个任务的 initial.pose")
        elif "poses" in initial:
            pose_sequence = initial["poses"]
            if not isinstance(pose_sequence, list) or not pose_sequence:
                raise ConfigError(
                    f"{mission_path}：第 {index} 个任务的 "
                    "initial.poses 必须是非空列表")
            initial_pose = [
                _validate_pose(
                    pose,
                    context=(
                        f"{mission_path}：第 {index} 个任务的 "
                        f"initial.poses[{pose_index}]"),
                )
                for pose_index, pose in enumerate(pose_sequence)
            ]

        on_failure = entry.get("on_failure", {})
        if not isinstance(on_failure, dict):
            raise ConfigError(
                f"{mission_path}：第 {index} 个任务的 on_failure 必须是映射")
        failure_profiles = {}
        for failure_code, profile in on_failure.items():
            if not isinstance(failure_code, str) or not failure_code.strip():
                raise ConfigError(
                    f"{mission_path}：第 {index} 个任务的失败码必须是非空字符串")
            if not isinstance(profile, dict):
                raise ConfigError(
                    f"{mission_path}：第 {index} 个任务的 on_failure"
                    f".{failure_code} 必须是映射")
            unknown = set(profile) - {"params", "pose"}
            if unknown:
                raise ConfigError(
                    f"{mission_path}：第 {index} 个任务的 on_failure"
                    f".{failure_code} 存在未知键：{sorted(unknown)}")
            failure_params = profile.get("params", {})
            if not isinstance(failure_params, dict):
                raise ConfigError(
                    f"{mission_path}：第 {index} 个任务的 on_failure"
                    f".{failure_code}.params 必须是映射")
            failure_pose = None
            if "pose" in profile:
                failure_pose = _validate_pose(
                    profile["pose"],
                    context=(
                        f"{mission_path}：第 {index} 个任务的 on_failure"
                        f".{failure_code}.pose"),
                )
            failure_profiles[failure_code.strip()] = {
                "params": failure_params,
                "pose": failure_pose,
            }

        task_path = _resolve_task_config(mission_path, config, task_name)
        defaults = _validate_params(
            task_name, _load_task_defaults(task_path, task_name), class_registry)
        overrides = _validate_params(task_name, overrides, class_registry)
        initial_params = _validate_params(
            task_name, initial_params, class_registry)
        entries_data.append({
            "index": index,
            "name": task_name,
            "defaults": defaults,
            "overrides": overrides,
            "initial_params": initial_params,
            "initial_pose": initial_pose,
            "on_failure": failure_profiles,
        })

    tasks = []
    for index, item in enumerate(entries_data):
        # Flatten each layer before merging so a canonical key such as
        # ``search_timeout`` can override a nested task default
        # ``search: {timeout: ...}`` without creating a duplicate key.
        merged = dict(item["defaults"])
        merged.update(item["overrides"])
        merged.update(item["initial_params"])
        task = {
            "name": item["name"],
            "params": _validate_params(
                item["name"], merged, class_registry, complete=True),
        }
        if item["initial_pose"] is not None:
            task["initial_pose"] = item["initial_pose"]

        if item["on_failure"]:
            next_name = (
                entries_data[index + 1]["name"]
                if index + 1 < len(entries_data) else None)
            normalized_profiles = {}
            for failure_code, profile in item["on_failure"].items():
                failure_params = profile["params"]
                if next_name is None and (
                        failure_params or profile["pose"] is not None):
                    raise ConfigError(
                        f"{mission_path}：第 {item['index']} 个任务的 on_failure"
                        f".{failure_code} 没有下一任务可接收覆盖")
                normalized = {}
                if failure_params:
                    normalized["params"] = _validate_params(
                        next_name, failure_params, class_registry)
                elif "params" in profile:
                    normalized["params"] = {}
                if profile["pose"] is not None:
                    normalized["pose"] = profile["pose"]
                normalized_profiles[failure_code] = normalized
            task["on_failure"] = normalized_profiles

        tasks.append(task)
    return tasks


def load_mission_or_task(path: str | Path,
                         class_registry=None) -> list[dict[str, Any]]:
    """Load either a mission YAML or one standalone task YAML.

    ``mission_file`` is kept as the public ROS parameter name for backwards
    compatibility, but it may point to either configuration shape.
    """
    config_path = Path(path).expanduser().resolve()
    data = _read_yaml(config_path)
    if "mission" in data and "task" in data:
        raise ConfigError(f"{config_path}：不能同时包含 mission 和 task 根节点")
    if "task" in data:
        return load_task(config_path, class_registry)
    return load_mission(config_path, class_registry)


def default_mission_path() -> Path:
    """Return the installed default mission path."""
    from ament_index_python.packages import get_package_share_directory

    return (Path(get_package_share_directory("uv_task"))
            / "config" / "missions" / "robocup_26.yaml")
