"""Tests for the YAML mission/task configuration contract."""

from pathlib import Path

import pytest

from uv_task.config_loader import (
    ConfigError,
    load_mission,
    load_mission_or_task,
)


CONFIG_ROOT = Path(__file__).parents[1] / "config"
MISSION = CONFIG_ROOT / "missions" / "robocup_26.yaml"
FIND_TASK = CONFIG_ROOT / "tasks" / "26rb_find_collection_frame.yaml"
MAPPING_TASK = CONFIG_ROOT / "tasks" / "mapping_grid.json"
MAPPING_MISSION = CONFIG_ROOT / "missions" / "mapping_grid.json"
EXPECTED_TASKS = [
    "start",
    "return_origin",
    "btravelx",
    "setz",
    "26rb_find_collection_frame",
    "26rb_grab_ball",
    "light_target_rack_return_origin",
]


def test_default_mission_preserves_order_and_values():
    tasks = load_mission(MISSION)

    assert [task["name"] for task in tasks] == EXPECTED_TASKS
    assert tasks[1]["params"] == {
        "state_settle_time": 0.3,
        "timeout": 30.0,
    }
    assert tasks[4]["params"]["look_order"] == [
        "collection_frame", "target_rack"]
    assert tasks[5]["params"]["ball_color"] == "red"
    assert tasks[6]["params"]["light_color"] == "yellow"
    assert tasks[6]["params"]["down_visual_servo_timeout"] == 30.0
    assert tasks[6]["params"]["down_visual_servo_stable_seconds"] == 1.0
    assert tasks[6]["params"]["down_detection_timeout"] == 0.8
    assert tasks[6]["params"]["down_pixel_tolerance_fraction"] == 0.035
    assert tasks[6]["params"][
        "down_epipolar_vertical_tolerance_fraction"] == 0.04
    assert tasks[6]["params"]["down_projection_depth_m"] == 0.8
    assert tasks[6]["params"]["down_visual_servo_gain"] == 0.8
    assert tasks[6]["params"]["down_visual_servo_max_step_m"] == 0.08


def test_standalone_task_file_loads_as_one_task():
    tasks = load_mission_or_task(FIND_TASK)

    assert len(tasks) == 1
    assert tasks[0]["name"] == "26rb_find_collection_frame"
    assert tasks[0]["params"]["scan_yaw_step_deg"] == 15.0
    assert tasks[0]["params"]["move_timeout"] == 120.0


def test_mapping_json_comments_are_not_task_parameters():
    tasks = load_mission_or_task(MAPPING_TASK)
    mission_tasks = load_mission(MAPPING_MISSION)

    assert tasks[0]["name"] == "mapping_grid"
    assert tasks[0]["params"]["grid_side_m"] == 2.4
    assert "comments" not in tasks[0]["params"]
    assert mission_tasks[1]["params"] == tasks[0]["params"]


def test_nested_parameters_are_merged_and_flattened(tmp_path):
    task_file = tmp_path / "grab.yaml"
    task_file.write_text(
        """task: 26rb_grab_ball
params:
  ball_color: red
  servo:
    timeout: 60.0
    period: 0.2
  gripper:
    offset_x_m: 0.15
""",
        encoding="utf-8",
    )
    mission_file = tmp_path / "mission.yaml"
    mission_file.write_text(
        """mission:
  name: test
  tasks:
    - name: 26rb_grab_ball
      config: grab.yaml
      params:
        servo:
          timeout: 12.0
""",
        encoding="utf-8",
    )

    tasks = load_mission(mission_file)
    assert tasks == [{
        "name": "26rb_grab_ball",
        "params": {
            "ball_color": "red",
            "horizontal_servo_timeout": 12.0,
            "horizontal_servo_period": 0.2,
            "gripper_offset_x_m": 0.15,
        },
    }]


def test_down_visual_servo_parameters_are_overridable(tmp_path):
    task_file = tmp_path / "rack.yaml"
    task_file.write_text(
        """task: light_target_rack_return_origin
params:
  target:
    frame_name: target_rack
  visual_servo:
    timeout: 30.0
    stable_seconds: 1.0
    detection_timeout: 0.8
    pixel_tolerance_fraction: 0.035
    epipolar_vertical_tolerance_fraction: 0.04
    projection_depth_m: 0.8
    gain: 0.8
    max_step_m: 0.08
    period: 0.2
    command_timeout: 10.0
""",
        encoding="utf-8",
    )
    mission_file = tmp_path / "mission.yaml"
    mission_file.write_text(
        """mission:
  name: test
  tasks:
    - name: light_target_rack_return_origin
      config: rack.yaml
      params:
        visual_servo:
          stable_seconds: 2
          gain: 0.5
""",
        encoding="utf-8",
    )

    tasks = load_mission(mission_file)
    assert tasks[0]["params"]["down_visual_servo_stable_seconds"] == 2
    assert tasks[0]["params"]["down_visual_servo_gain"] == 0.5
    assert tasks[0]["params"]["down_pixel_tolerance_fraction"] == 0.035


@pytest.mark.parametrize("body", [
    """mission:\n  name: bad\n  tasks:\n    - name: does_not_exist\n""",
    """mission:\n  name: bad\n  tasks:\n    - name: setz\n      config: task.yaml\n""",
])
def test_invalid_mission_is_rejected(tmp_path, body):
    mission_file = tmp_path / "mission.yaml"
    mission_file.write_text(body, encoding="utf-8")
    if "task.yaml" in body:
        (tmp_path / "task.yaml").write_text(
            "task: setz\nparams:\n  z: wrong\n", encoding="utf-8")

    with pytest.raises(ConfigError):
        load_mission(mission_file)


def test_unknown_parameter_is_rejected(tmp_path):
    task_file = tmp_path / "task.yaml"
    task_file.write_text(
        "task: setz\nparams:\n  z: 0.1\n  typo: 1\n", encoding="utf-8")
    mission_file = tmp_path / "mission.yaml"
    mission_file.write_text(
        "mission:\n  name: bad\n  tasks:\n"
        "    - name: setz\n      config: task.yaml\n",
        encoding="utf-8",
    )

    with pytest.raises(ConfigError, match="未知参数"):
        load_mission(mission_file)


def test_duplicate_yaml_key_is_rejected(tmp_path):
    mission_file = tmp_path / "mission.yaml"
    mission_file.write_text(
        "mission:\n  name: one\n  name: two\n  tasks: []\n",
        encoding="utf-8",
    )

    with pytest.raises(ConfigError, match="YAML 文件存在重复键"):
        load_mission(mission_file)
