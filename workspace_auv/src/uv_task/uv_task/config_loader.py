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
    "start": {
        "skip_preparation": bool,
    },
    "return_origin": {
        "state_settle_time": float,
        "timeout": float,
    },
    "btravelx": {"dx": float},
    "setz": {"z": float},
    "setrz": {"rz": float},
    "wtravelxyz": {"x": float, "y": float, "z": float},
    "mapping_grid": {
        "enable_traversal": bool,
        "timeout": float,
        "move_timeout": float,
        "traversal_clearance_m": float,
        "traversal_tracking_margin_m": float,
        "observe_seconds": float,
        "stable_seconds": float,
        "stable_position_m": float,
        "stable_angle_deg": float,
        "allow_motion_observations": bool,
        "motion_window_seconds": float,
        "motion_position_m": float,
        "motion_angle_deg": float,
        "min_observations": int,
        "min_confidence": float,
        "class_vote_ratio": float,
        "expected_cones": int,
        "grid_center_x": float,
        "grid_center_y": float,
        "grid_side_m": float,
        "grid_yaw_deg": float,
        "floor_z": float,
        "survey_z": float,
        "survey_yaw_deg": float,
        "tag_x": float,
        "tag_y": float,
        "tag_id": int,
        "allowed_tag_ids": (list, int),
        "tag_dictionary": str,
        "visit_order": (list, int),
        "image_topic": str,
        "left_detection_topic": str,
        "right_detection_topic": str,
        "left_info_topic": str,
        "right_info_topic": str,
        "pose_slop_s": float,
        "image_slop_s": float,
        "detection_slop_s": float,
        "left_translation": (list, float),
        "right_translation": (list, float),
        "camera_rotation": (list, float),
        "sgbm_min_disparity": int,
        "sgbm_num_disparities": int,
        "sgbm_block_size": int,
        "min_disparity": float,
        "min_depth_m": float,
        "max_depth_m": float,
        "depth_bin_m": float,
        "depth_peak_ratio": float,
        "min_depth_points": int,
        "measurement_sigma_m": float,
        "process_noise": float,
        "mahalanobis_gate": float,
        "cell_gate_m": float,
        "tag_timeout": float,
        "fallback_tag_id": int,
        "fallback_square_cells": (list, int),
        "fallback_round_cells": (list, int),
    },
    "turntable": {
        "disk_pose_odom": (list, float),
        "front_standoff_m": float,
        "front_camera_center_body": (list, float),
        "rod_tip_body": (list, float),
        "observation_timeout_s": float,
        "post_motion_observation_timeout_s": float,
        "motion_timeout_s": float,
        "arrival_verify_timeout_s": float,
        "motion_settle_s": float,
        "allow_contact_motion": bool,
        "force_limited_control_confirmed": bool,
        "rod_radius_m": float,
        "approach_standoff_m": float,
        "insert_depth_m": float,
        "stroke_yaw_deg": float,
        "contact_ascent_m": float,
        "contact_descent_m": float,
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
    "grab_sea_cucumber": {
        "gripper_servo_id": int,
        "gripper_close_wait_seconds": float,
        "servo2_close_angle_deg": float,
        "servo2_open_angle_deg": float,
        "servo2_front_camera_body_xyz": (list, float),
        "servo2_down_camera_body_xyz": (list, float),
        "servo2_gripper_from_front_xyz": (list, float),
        "near_floor_open_loop": bool,
        "floor_z_m": float,
        "bottom_reference_offset_z_m": float,
        "open_loop_clearance_m": float,
        "press_thrust": float,
        "lift_thrust": float,
        "press_thrust_seconds": float,
        "restore_pre_press_odom_xy": bool,
        "collection_visual_align": bool,
        "collection_class_id": int,
        "collection_projection_depth_m": float,
        "collection_correct_odom_xy": bool,
        "collection_center_odom_xy": (list, float),
        "collection_camera_body_xy": (list, float),
        "odom_correction_max_m": float,
        "sea_cucumber_class_id": int,
        "down_camera_mount_yaw_deg": float,
        "image_width": int,
        "image_height": int,
        "search_pose": (list, float),
        "search_travel_timeout_seconds": float,
        "gripper_offset_x_m": float,
        "gripper_offset_y_m": float,
        "descent_speed_mps": float,
        "descent_duration_seconds": float,
        "max_press_distance_m": float,
        "drop_pose": (list, float),
        "pickup_servo_angle_rad": float,
        "release_servo_angle_rad": float,
        "pickup_servo_angle_deg": float,
        "release_servo_angle_deg": float,
        "total_timeout_seconds": float,
        "expected_count": int,
        "count_frames": int,
        "count_timeout_seconds": float,
        "min_confidence": float,
        "drop_timeout_seconds": float,
        "release_wait_seconds": float,
        "ascent_step_m": float,
        "ascent_speed_mps": float,
        "ascent_publish_period": float,
        "ascent_step_timeout_seconds": float,
        "ascent_pause_seconds": float,
        "ascent_tolerance_m": float,
        "max_failed_attempts": int,
        "pixel_tolerance_fraction": float,
        "detection_timeout": float,
        "horizontal_servo_timeout": float,
        "horizontal_servo_period": float,
        "horizontal_servo_log_period": float,
        "horizontal_hold_seconds": float,
        "projection_depth_m": float,
        "horizontal_servo_gain": float,
        "max_horizontal_step_m": float,
        "position_command_timeout": float,
        "pre_descent_settle_seconds": float,
        "descent_publish_period": float,
        "return_timeout": float,
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

# 基础动作只有少量标量参数；允许直接写在任务链里，不要求空模板文件。
INLINE_TASKS = frozenset({"btravelx", "setz", "wtravelxyz", "setrz"})


# Short names keep the YAML readable where the runtime key carries the
# implementation detail (for example ``servo.timeout`` becomes
# ``horizontal_servo_timeout``).  The resulting keys are still exactly the
# names used by the existing task classes.
PARAMETER_ALIASES = {
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
    "grab_sea_cucumber": {
        "gripper.servo_id": "gripper_servo_id",
        "gripper.close_wait_seconds": "gripper_close_wait_seconds",
        "gripper.servo2.close_angle_deg": "servo2_close_angle_deg",
        "gripper.servo2.open_angle_deg": "servo2_open_angle_deg",
        "gripper.servo2.front_camera_body_xyz": "servo2_front_camera_body_xyz",
        "gripper.servo2.down_camera_body_xyz": "servo2_down_camera_body_xyz",
        "gripper.servo2.gripper_from_front_xyz": "servo2_gripper_from_front_xyz",
        "search.pose": "search_pose",
        "search.travel_timeout_seconds": "search_travel_timeout_seconds",
        "servo.timeout": "horizontal_servo_timeout",
        "servo.period": "horizontal_servo_period",
        "servo.log_period": "horizontal_servo_log_period",
        "servo.hold_seconds": "horizontal_hold_seconds",
        "servo.gain": "horizontal_servo_gain",
        "servo.max_step_m": "max_horizontal_step_m",
        "gripper.offset_x_m": "gripper_offset_x_m",
        "gripper.offset_y_m": "gripper_offset_y_m",
        "gripper.settle_seconds": "pre_descent_settle_seconds",
        "gripper.pickup_angle_rad": "pickup_servo_angle_rad",
        "gripper.release_angle_rad": "release_servo_angle_rad",
        "gripper.pickup_angle_deg": "pickup_servo_angle_deg",
        "gripper.release_angle_deg": "release_servo_angle_deg",
        "ascent.speed_mps": "ascent_speed_mps",
        "ascent.publish_period": "ascent_publish_period",
        "descent.speed_mps": "descent_speed_mps",
        "descent.duration_seconds": "descent_duration_seconds",
        "descent.publish_period": "descent_publish_period",
        "counting.frames": "count_frames",
        "counting.timeout_seconds": "count_timeout_seconds",
        "delivery.pose": "drop_pose",
        "delivery.timeout_seconds": "drop_timeout_seconds",
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
    unknown = set(data) - {"task", "params", "comments"}
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
    unknown = set(data) - {"task", "params", "comments"}
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
        if config is None and task_name in INLINE_TASKS:
            defaults = {}
        else:
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
            / "config" / "missions" / "mapping_grid.json")
