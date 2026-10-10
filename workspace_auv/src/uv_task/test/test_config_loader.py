"""Tests for the YAML mission/task configuration contract."""

from pathlib import Path

import pytest
import yaml

from auv_protocol.model_mapping import ClassInfo, ModelClassRegistry
from uv_task.config_loader import (
    ConfigError,
    load_mission,
    load_mission_or_task,
)


CONFIG_ROOT = Path(__file__).parents[1] / "config"
MISSION = CONFIG_ROOT / "missions" / "robocup_26.yaml"
GATE_TASK = CONFIG_ROOT / "tasks" / "26rb_gate_task.yaml"
def test_default_mission_preserves_order_and_values():
    configured = yaml.safe_load(MISSION.read_text(encoding="utf-8"))["mission"]["tasks"]
    tasks = load_mission(MISSION)

    assert [task["name"] for task in tasks] == [entry["name"] for entry in configured]
    for entry, task in zip(configured, tasks):
        if "params" not in entry and not entry.get("initial", {}).get("params"):
            defaults = load_mission_or_task(MISSION.parent / entry["config"])[0]
            assert task["params"] == defaults["params"]


def test_default_mission_has_initial_pose_and_one_hop_timeout_hooks():
    configured = yaml.safe_load(MISSION.read_text(encoding="utf-8"))["mission"]["tasks"]
    tasks = load_mission(MISSION)

    for entry, task in zip(configured, tasks):
        initial = entry["initial"]
        poses = initial.get("poses", [initial.get("pose")])
        normalized = [dict(pose, target=[float(value) for value in pose["target"]])
                      for pose in poses]
        if len(normalized) == 1:
            assert task["initial_pose"] == normalized[0]
        else:
            assert task["initial_pose"] == normalized
        assert task["on_failure"] == {
            code: {"params": {}} for code in entry["on_failure"]
        }


def test_standalone_task_file_loads_as_one_task():
    configured = yaml.safe_load(GATE_TASK.read_text(encoding="utf-8"))
    tasks = load_mission_or_task(GATE_TASK)

    assert len(tasks) == 1
    assert tasks[0]["name"] == "26rb_gate_task"
    assert tasks[0]["params"]["gate_count"] == configured["params"]["gate_count"]
    assert tasks[0]["params"]["search_timeout"] == configured["params"]["search"]["timeout"]
    assert tasks[0]["params"]["pass_yaw_kp"] == configured["params"]["pass"]["yaw_kp"]


def test_nested_parameters_are_merged_and_flattened(tmp_path):
    task_file = tmp_path / "gate.yaml"
    task_file.write_text(
        """task: 26rb_gate_task
params:
  gate_count: 4
  observation:
    poses: [[1, 0, .2, 0], [2, 0, .2, 0], [3, 0, .2, 0], [4, 0, .2, 0]]
  search:
    start_offset_deg: -30.0
    timeout: 60.0
  depth:
    kp: 0.8
  pass:
    yaw_kp: 3.6
""",
        encoding="utf-8",
    )
    mission_file = tmp_path / "mission.yaml"
    mission_file.write_text(
        """mission:
  name: test
  tasks:
    - name: 26rb_gate_task
      config: gate.yaml
      params:
        search:
          timeout: 12.0
        depth:
          kp: 1.2
""",
        encoding="utf-8",
    )

    tasks = load_mission(mission_file)
    assert tasks == [{
        "name": "26rb_gate_task",
        "params": {
            "gate_count": 4,
            "observation_poses": [[1, 0, .2, 0], [2, 0, .2, 0], [3, 0, .2, 0], [4, 0, .2, 0]],
            "search_start_offset_deg": -30.0,
            "search_timeout": 12.0,
            "depth_kp": 1.2,
            "pass_yaw_kp": 3.6,
        },
    }]


def test_down_visual_servo_parameters_are_overridable(tmp_path):
    task_file = tmp_path / "rack.yaml"
    task_file.write_text(
        """task: 26rb_drop_ball_target_rack
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
    - name: 26rb_drop_ball_target_rack
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


def test_initial_params_override_legacy_mission_params_and_task_defaults(tmp_path):
    task_file = tmp_path / "setz.yaml"
    task_file.write_text(
        "task: setz\nparams:\n  z: 0.1\n", encoding="utf-8")
    mission_file = tmp_path / "mission.yaml"
    mission_file.write_text(
        """mission:
  name: precedence
  tasks:
    - name: setz
      config: setz.yaml
      params:
        z: 0.2
      initial:
        params:
          z: 0.3
""",
        encoding="utf-8",
    )

    tasks = load_mission(mission_file)

    assert tasks[0]["params"] == {"z": 0.3}


def test_flat_initial_param_can_override_nested_task_default(tmp_path):
    task_file = tmp_path / "gate.yaml"
    task_file.write_text(
        """task: 26rb_gate_task
params:
  search:
    timeout: 60.0
""",
        encoding="utf-8",
    )
    mission_file = tmp_path / "mission.yaml"
    mission_file.write_text(
        """mission:
  name: flat_initial
  tasks:
    - name: 26rb_gate_task
      config: gate.yaml
      initial:
        params:
          search_timeout: 12.0
""",
        encoding="utf-8",
    )

    tasks = load_mission(mission_file)

    assert tasks[0]["params"] == {"search_timeout": 12.0}


def test_failure_override_is_validated_for_next_task_and_normalized(tmp_path):
    setz_file = tmp_path / "setz.yaml"
    setz_file.write_text(
        "task: setz\nparams:\n  z: 0.1\n", encoding="utf-8")
    btravel_file = tmp_path / "btravelx.yaml"
    btravel_file.write_text(
        "task: btravelx\nparams:\n  dx: 2.0\n", encoding="utf-8")
    mission_file = tmp_path / "mission.yaml"
    mission_file.write_text(
        """mission:
  name: failure_transfer
  tasks:
    - name: setz
      config: setz.yaml
      on_failure:
        "setz.timeout":
          params:
            dx: 1.5
          pose:
            command: bmove
            axes: XYZRZ
            target: [0.5, 0.0, 0.0, 15]
        default:
          pose:
            command: WMOVE
            axes: xyrz
            target: [1.0, 2.0, 0.3, 90.0]
    - name: btravelx
      config: btravelx.yaml
      initial:
        pose:
          command: SET
          axes: xyzrz
          target: [9.0, 8.0, 7.0, 6.0]
""",
        encoding="utf-8",
    )

    tasks = load_mission(mission_file)

    assert tasks[0]["on_failure"]["setz.timeout"] == {
        "params": {"dx": 1.5},
        "pose": {
            "command": "BMOVE",
            "axes": "xyzrz",
            "target": [0.5, 0.0, 0.0, 15.0],
        },
    }
    assert tasks[0]["on_failure"]["default"] == {
        "params": {},
        "pose": {
            "command": "WMOVE",
            "axes": "xyrz",
            "target": [1.0, 2.0, 0.3, 90.0],
        },
    }
    assert tasks[1]["initial_pose"]["command"] == "SET"


def test_failure_params_are_checked_against_the_next_task_schema(tmp_path):
    first_file = tmp_path / "first.yaml"
    first_file.write_text(
        "task: setz\nparams:\n  z: 0.1\n", encoding="utf-8")
    next_file = tmp_path / "next.yaml"
    next_file.write_text(
        "task: btravelx\nparams:\n  dx: 2.0\n", encoding="utf-8")
    mission_file = tmp_path / "mission.yaml"
    mission_file.write_text(
        """mission:
  name: bad_transfer
  tasks:
    - name: setz
      config: first.yaml
      on_failure:
        "setz.timeout":
          params:
            z: 1.0
    - name: btravelx
      config: next.yaml
""",
        encoding="utf-8",
    )

    with pytest.raises(ConfigError, match="未知参数"):
        load_mission(mission_file)


def test_nonempty_failure_override_requires_a_next_task(tmp_path):
    task_file = tmp_path / "task.yaml"
    task_file.write_text(
        "task: setz\nparams:\n  z: 0.1\n", encoding="utf-8")
    mission_file = tmp_path / "mission.yaml"
    mission_file.write_text(
        """mission:
  name: no_receiver
  tasks:
    - name: setz
      config: task.yaml
      on_failure:
        "setz.timeout":
          pose:
            command: BMOVE
            axes: xyzrz
            target: [0.5, 0.0, 0.0, 0.0]
""",
        encoding="utf-8",
    )

    with pytest.raises(ConfigError, match="没有下一任务"):
        load_mission(mission_file)


@pytest.mark.parametrize("pose", [
    {"command": "MOVE", "axes": "xyz", "target": [0, 0, 0, 0]},
    {"command": "BMOVE", "axes": "xyz", "target": [0, 0, 0]},
    {"command": "BMOVE", "axes": "xx", "target": [0, 0, 0, 0]},
])
def test_invalid_initial_pose_is_rejected(tmp_path, pose):
    task_file = tmp_path / "task.yaml"
    task_file.write_text(
        "task: setz\nparams:\n  z: 0.1\n", encoding="utf-8")
    mission_file = tmp_path / "mission.yaml"
    mission_file.write_text(yaml.safe_dump({
        "mission": {
            "name": "bad_pose",
            "tasks": [{
                "name": "setz",
                "config": "task.yaml",
                "initial": {"pose": pose},
            }],
        },
    }, sort_keys=False), encoding="utf-8")

    with pytest.raises(ConfigError):
        load_mission(mission_file)


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


def test_ball_parameters_reject_semantic_colour_aliases(tmp_path):
    task_file = tmp_path / "balls.yaml"
    task_file.write_text(
        "task: 26rb_hit_balls\nparams:\n  order: [blue]\n",
        encoding="utf-8",
    )

    registry = ModelClassRegistry(entries=(
        ClassInfo(0, "impact_ball_blue", "impact_ball_blue", None, False),
        ClassInfo(1, "impact_ball_red", "impact_ball_red", None, False),
    ))
    with pytest.raises(ConfigError, match="impact_ball_blue / impact_ball_red"):
        load_mission_or_task(task_file, class_registry=registry)
