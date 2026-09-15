"""Mockable tests for mission initial/failure override semantics."""

from uv_task.mission_policy import (
    apply_failure_override,
    select_failure_override,
)
from uv_task.task_outcome import TaskOutcome


def test_failure_params_override_initial_params_and_pose():
    task = {
        "on_failure": {
            "gate.timeout": {
                "params": {"gate_count": 1},
                "pose": {"command": "BMOVE", "target": [1, 0, 0, 0]},
            },
            "default": {"params": {"gate_count": 2}},
        },
    }
    own_pose = {"command": "BMOVE", "target": [0, 0, 0, 0]}

    override = select_failure_override(task, "gate.timeout")
    params, pose = apply_failure_override(
        {"gate_count": 4}, own_pose, override)

    assert params == {"gate_count": 1}
    assert pose["target"] == [1, 0, 0, 0]


def test_default_failure_hook_is_used_when_exact_code_is_absent():
    task = {"on_failure": {"default": {"params": {"dx": 0.5}}}}

    assert select_failure_override(task, "btravelx.motion") == {
        "params": {"dx": 0.5}
    }


def test_missing_failure_pose_keeps_next_task_initial_pose():
    own_pose = {"command": "SET", "target": [9, 8, 7, 6]}
    override = {"params": {"dx": 0.5}}

    params, pose = apply_failure_override({"dx": 2.0}, own_pose, override)

    assert params == {"dx": 0.5}
    assert pose is own_pose


def test_pending_failure_override_is_one_hop_when_runner_clears_it():
    transferred = {"params": {"dx": 0.5}}
    next_params, _ = apply_failure_override(
        {"dx": 2.0}, None, transferred)
    later_params, _ = apply_failure_override(
        {"dx": 3.0}, None, None)

    assert next_params == {"dx": 0.5}
    assert later_params == {"dx": 3.0}


def test_task_outcome_carries_failure_code_message_and_transfer():
    override = {"params": {"dx": 0.5}}

    outcome = TaskOutcome.failed("gate.timeout", "动作超时").with_transfer(
        override)

    assert not outcome
    assert outcome.failure_code == "gate.timeout"
    assert outcome.message == "动作超时"
    assert outcome.transfer_override is override
