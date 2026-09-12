"""Strict YAML mission/task configuration loading for :mod:`uv_task`.

The files intentionally use ordinary YAML only.  Task parameter groups are a
human-facing representation; the loader validates and flattens them to the
legacy keys consumed by the task implementations.
"""

from __future__ import annotations

from copy import deepcopy
from pathlib import Path
from typing import Any

import yaml


class ConfigError(ValueError):
    """Raised when a mission or task YAML file is invalid."""


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
    "return_origin": {
        "state_settle_time": float,
        "timeout": float,
    },
    "btravelx": {"dx": float},
    "setz": {"z": float},
    "26rb_hit_balls": {
        "order": (list, str),
        "active_localization": bool,
        "search_yaw_step_deg": float,
        "search_settle_time": float,
        "search_timeout": float,
        "search_rotate_timeout": float,
        "min_confidence": float,
        "impact_mode": str,
        "approach_distance": float,
        "min_clearance": float,
        "z_offset": float,
        "detect_timeout": float,
        "position_correction_duration": float,
        "position_correction_period": float,
        "position_correction_command_timeout": float,
        "position_correction_min_update_m": float,
        "position_correction_min_update_deg": float,
        "charge_duration": float,
        "charge_speed_mps": float,
        "charge_publish_period": float,
        "return_timeout": float,
    },
    "26rb_gate_task": {
        "gate_count": int,
        "timeout": float,
        "image_input": str,
        "search_start_offset_deg": float,
        "search_sweep_deg": float,
        "right_yaw_sign": float,
        "search_yaw_rate_deg_s": float,
        "search_yaw_publish_period": float,
        "search_settle_time": float,
        "search_timeout": float,
        "rotate_timeout": float,
        "search_stop_height_fraction": float,
        "min_gate_extent_fraction": float,
        "target_gate_extent_fraction": float,
        "target_gate_height_fraction": float,
        "extent_tolerance": float,
        "heading_check_seconds": float,
        "post_turn_settle_seconds": float,
        "post_turn_observation_timeout": float,
        "bbox_filter_alpha": float,
        "bbox_log_period": float,
        "bbox_ratio_target": float,
        "bbox_ratio_hold_seconds": float,
        "minimum_attitude_correction_seconds": float,
        "alignment_timeout": float,
        "control_period": float,
        "command_timeout": float,
        "max_forward_step": float,
        "max_back_step": float,
        "max_lateral_step": float,
        "max_vertical_step": float,
        "lost_observation_timeout": float,
        "detection_timeout": float,
        "min_gate_detection_confidence": float,
        "image_center_tolerance_fraction": float,
        "stereo_vertical_tolerance_fraction": float,
        "distance_control_min_m": float,
        "distance_control_max_m": float,
        "monocular_reference_distance_m": float,
        "max_yaw_step_deg": float,
        "velocity_period": float,
        "max_forward_speed_mps": float,
        "max_reverse_speed_mps": float,
        "max_lateral_speed_mps": float,
        "max_vertical_speed_mps": float,
        "max_yaw_rate_deg_s": float,
        "distance_velocity_gain": float,
        "vertical_velocity_gain": float,
        "yaw_velocity_gain_deg_s": float,
        "height_pid_kp": float,
        "height_pid_ki": float,
        "height_pid_kd": float,
        "height_pid_integral_limit": float,
        "height_pid_stable_error_m": float,
        "height_pid_stable_derivative_mps": float,
        "height_pid_stable_seconds": float,
        "yaw_pid_kp": float,
        "yaw_pid_ki": float,
        "yaw_pid_kd": float,
        "yaw_pid_integral_limit_deg": float,
        "arc_lateral_speed_mps": float,
        "yaw_center_kp": float,
        "arc_probe_seconds": float,
        "arc_probe_speed_mps": float,
        "arc_gradient_window_seconds": float,
        "arc_objective_deadband": float,
        "arc_reverse_cooldown_seconds": float,
        "arc_near_extremum_speed_scale": float,
        "lost_command_hold_seconds": float,
        "pass_distance_m": float,
        "pass_timeout": float,
        "post_pass_pause": float,
        "image_timeout": float,
    },
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
    "26rb_grab_ball": {
        "ball_color": str,
        "detection_timeout": float,
        "horizontal_servo_timeout": float,
        "horizontal_servo_period": float,
        "horizontal_servo_log_period": float,
        "pixel_tolerance_fraction": float,
        "horizontal_hold_seconds": float,
        "projection_depth_m": float,
        "horizontal_servo_gain": float,
        "max_horizontal_step_m": float,
        "position_command_timeout": float,
        "gripper_offset_x_m": float,
        "gripper_offset_y_m": float,
        "pre_descent_settle_seconds": float,
        "descent_speed_mps": float,
        "descent_duration_seconds": float,
        "descent_publish_period": float,
        "return_timeout": float,
        "verification_timeout": float,
        "verification_absence_hold_seconds": float,
        "max_grab_retries": int,
    },
    "light_target_rack_return_origin": {
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
        # The final alignment is performed from the down-view stereo image.
        # Keep the old horizontal-servo keys above readable for old custom
        # task files, while the new canonical keys make the data source
        # explicit.
        "down_visual_servo_timeout": float,
        "down_visual_servo_stable_seconds": float,
        "down_detection_timeout": float,
        "down_pixel_tolerance_fraction": float,
        "down_epipolar_vertical_tolerance_fraction": float,
        "down_projection_depth_m": float,
        "down_visual_servo_gain": float,
        "down_visual_servo_max_step_m": float,
        "down_visual_servo_period": float,
        "down_visual_command_timeout": float,
        "light_hold_seconds": float,
        "return_timeout": float,
    },
}


# Short names keep the YAML readable where the runtime key carries the
# implementation detail (for example ``servo.timeout`` becomes
# ``horizontal_servo_timeout``).  The resulting keys are still exactly the
# names used by the existing task classes.
PARAMETER_ALIASES = {
    "26rb_hit_balls": {
        "search.yaw_step_deg": "search_yaw_step_deg",
        "search.settle_time": "search_settle_time",
        "search.timeout": "search_timeout",
        "search.rotate_timeout": "search_rotate_timeout",
        "impact.mode": "impact_mode",
        "approach.distance": "approach_distance",
        "approach.min_clearance": "min_clearance",
        "approach.z_offset": "z_offset",
        "approach.detect_timeout": "detect_timeout",
        "position_correction.duration": "position_correction_duration",
        "position_correction.period": "position_correction_period",
        "position_correction.command_timeout": "position_correction_command_timeout",
        "position_correction.min_update_m": "position_correction_min_update_m",
        "position_correction.min_update_deg": "position_correction_min_update_deg",
        "charge.duration": "charge_duration",
        "charge.speed_mps": "charge_speed_mps",
        "charge.publish_period": "charge_publish_period",
        "return.timeout": "return_timeout",
    },
    "26rb_gate_task": {
        "velocity.distance_gain": "distance_velocity_gain",
        "velocity.vertical_gain": "vertical_velocity_gain",
        "velocity.yaw_gain_deg_s": "yaw_velocity_gain_deg_s",
        "alignment.yaw_pid.kp": "yaw_pid_kp",
        "alignment.yaw_pid.ki": "yaw_pid_ki",
        "alignment.yaw_pid.kd": "yaw_pid_kd",
        "alignment.yaw_pid.integral_limit_deg": "yaw_pid_integral_limit_deg",
        "pass.pause": "post_pass_pause",
    },
    "26rb_grab_ball": {
        "servo.timeout": "horizontal_servo_timeout",
        "servo.period": "horizontal_servo_period",
        "servo.log_period": "horizontal_servo_log_period",
        "servo.hold_seconds": "horizontal_hold_seconds",
        "servo.gain": "horizontal_servo_gain",
        "servo.max_step_m": "max_horizontal_step_m",
        "gripper.offset_x_m": "gripper_offset_x_m",
        "gripper.offset_y_m": "gripper_offset_y_m",
        "gripper.settle_seconds": "pre_descent_settle_seconds",
        "descent.speed_mps": "descent_speed_mps",
        "descent.duration_seconds": "descent_duration_seconds",
        "descent.publish_period": "descent_publish_period",
        "verification.timeout": "verification_timeout",
        "verification.absence_hold_seconds": "verification_absence_hold_seconds",
        "verification.max_retries": "max_grab_retries",
    },
    "light_target_rack_return_origin": {
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
        "visual_servo.pixel_tolerance_fraction":
            "down_pixel_tolerance_fraction",
        "visual_servo.epipolar_vertical_tolerance_fraction":
            "down_epipolar_vertical_tolerance_fraction",
        "visual_servo.projection_depth_m": "down_projection_depth_m",
        "visual_servo.gain": "down_visual_servo_gain",
        "visual_servo.max_step_m": "down_visual_servo_max_step_m",
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


def _deep_merge(base: dict[str, Any], override: dict[str, Any]):
    """Merge mappings recursively; scalar and list values are replaced."""
    result = deepcopy(base)
    for key, value in override.items():
        if key in result and isinstance(result[key], dict) and isinstance(value, dict):
            result[key] = _deep_merge(result[key], value)
        else:
            result[key] = deepcopy(value)
    return result


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


def _validate_params(task_name: str, params: dict[str, Any]) -> dict[str, Any]:
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


def load_task(path: str | Path) -> list[dict[str, Any]]:
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

    return [{"name": task_name, "params": _validate_params(task_name, params)}]


def load_mission(path: str | Path) -> list[dict[str, Any]]:
    """Load and validate a mission, returning runner-compatible task dicts."""
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

    tasks = []
    for index, entry in enumerate(entries, start=1):
        if not isinstance(entry, dict):
            raise ConfigError(f"{mission_path}：第 {index} 个任务必须是映射")
        unknown = set(entry) - {"name", "config", "params"}
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
        task_path = _resolve_task_config(mission_path, config, task_name)
        defaults = _load_task_defaults(task_path, task_name)
        merged = _deep_merge(defaults, overrides)
        tasks.append({"name": task_name, "params": _validate_params(task_name, merged)})
    return tasks


def load_mission_or_task(path: str | Path) -> list[dict[str, Any]]:
    """Load either a mission YAML or one standalone task YAML.

    ``mission_file`` is kept as the public ROS parameter name for backwards
    compatibility, but it may point to either configuration shape.
    """
    config_path = Path(path).expanduser().resolve()
    data = _read_yaml(config_path)
    if "mission" in data and "task" in data:
        raise ConfigError(f"{config_path}：不能同时包含 mission 和 task 根节点")
    if "task" in data:
        return load_task(config_path)
    return load_mission(config_path)


def default_mission_path() -> Path:
    """Return the installed default mission path."""
    from ament_index_python.packages import get_package_share_directory

    return (Path(get_package_share_directory("uv_task"))
            / "config" / "missions" / "robocup_26.yaml")
