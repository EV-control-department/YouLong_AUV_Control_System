"""``uv_control`` test skeleton: the motion API's contract, without a robot.

``uv_control`` had **no tests at all** until this file, which is why a doc/code
drift (``STEP_Y`` 0.4 vs 0.2, ``STEP_PERIOD`` 0.2 vs 0.3) and a duplicated
``cmd_type`` name map could sit unnoticed.  These tests are deliberately
**offline**: they cover the pure logic that the action server is built from, so
they run in a plain pytest without a simulator, a robot, or an executor.

What is pinned here, and why each one has a plausible wrong version:

* **``cmd_type`` names are derived, not hand-written.**  Three copies of the
  int -> name map existed (``basic_motion``, ``uv_task/task_runner``, the docs).
  A new command used to log ``UNKNOWN(7)`` until someone remembered all three.
* **the ``axes`` fusion rule.**  ``axes="x"`` means "x takes the new value, the
  other axes keep the *previous target*" -- not the current pose, and not zero.
  This is the single most surprising documented behaviour of the action, and it
  was copy-pasted three times in the dispatch before being extracted.
* **the step constants match their documentation** (``basic_motion_action.md``),
  which is the drift this skeleton would have caught.
"""

from __future__ import annotations

import importlib.util
import re
import threading
from pathlib import Path

import pytest
import rclpy

from uv_msgs.action import BasicMotion

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PACKAGE_ROOT.parents[2]
BASIC_MOTION_PY = PACKAGE_ROOT / "uv_control" / "basic_motion.py"
TASK_RUNNER_PY = (REPO_ROOT / "workspace_auv" / "src" / "uv_task" / "uv_task"
                  / "task_runner.py")
ACTION_DOC = PACKAGE_ROOT.parent.parent / "docs" / "basic_motion_action.md"


def _load_basic_motion():
    """Import ``basic_motion`` as a module (needs rclpy + the message package)."""
    from uv_control import basic_motion

    return basic_motion


# --------------------------------------------------------------------------- #
# cmd_type names come from the action definition
# --------------------------------------------------------------------------- #
def test_cmd_type_names_are_derived_from_the_action_definition() -> None:
    mapping = _load_basic_motion().cmd_type_names()

    assert mapping == {
        1: "WMOVE", 2: "BMOVE", 3: "SET", 4: "WTRAVEL", 5: "BTRAVEL", 6: "START",
    }, mapping
    # The derived map must agree with the constants the action actually exports,
    # i.e. it cannot silently drift from BasicMotion.action.
    for value, name in mapping.items():
        assert getattr(BasicMotion.Goal, name) == value


def test_task_runner_derives_the_same_mapping() -> None:
    """``uv_task`` cannot import ``uv_control``; both must derive, and agree.

    They are sibling packages (``docs/architecture/node_dependency.md``), so the
    mapping cannot be shared as a constant.  Deriving it from the generated
    ``BasicMotion.Goal`` in both places is what makes that safe -- this test
    pins that the two derivations cannot diverge.
    """
    text = TASK_RUNNER_PY.read_text(encoding="utf-8")
    assert "_cmd_type_names" in text, (
        "task_runner.py must derive the cmd_type names, not hard-code them"
    )
    # The hand-written map (the thing that used to drift) must be gone.
    assert not re.search(r"type_names\s*=\s*\{\s*1\s*:", text), (
        "a hand-written cmd_type name map is back in task_runner.py; derive it "
        "from BasicMotion.Goal instead (see basic_motion.cmd_type_names)"
    )


# --------------------------------------------------------------------------- #
# axes fusion
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("axes,value,previous,expected", [
    # empty = all axes (documented default)
    ("", (5.0, 6.0, 7.0, 8.0), (1.0, 2.0, 3.0, 4.0), (5.0, 6.0, 7.0, 8.0)),
    # only the listed axes move; the rest keep the PREVIOUS TARGET
    ("x", (5.0, 6.0, 7.0, 8.0), (1.0, 2.0, 3.0, 4.0), (5.0, 2.0, 3.0, 4.0)),
    ("y", (5.0, 6.0, 7.0, 8.0), (1.0, 2.0, 3.0, 4.0), (1.0, 6.0, 3.0, 4.0)),
    ("z", (5.0, 6.0, 7.0, 8.0), (1.0, 2.0, 3.0, 4.0), (1.0, 2.0, 7.0, 4.0)),
    ("rz", (5.0, 6.0, 7.0, 8.0), (1.0, 2.0, 3.0, 4.0), (1.0, 2.0, 3.0, 8.0)),
    ("xy", (5.0, 6.0, 7.0, 8.0), (1.0, 2.0, 3.0, 4.0), (5.0, 6.0, 3.0, 4.0)),
    ("xyz", (5.0, 6.0, 7.0, 8.0), (1.0, 2.0, 3.0, 4.0), (5.0, 6.0, 7.0, 4.0)),
    ("xyrz", (5.0, 6.0, 7.0, 8.0), (1.0, 2.0, 3.0, 4.0), (5.0, 6.0, 3.0, 8.0)),
])
def test_axes_fusion_keeps_unlisted_axes_at_the_previous_target(
    axes, value, previous, expected
) -> None:
    assert _load_basic_motion().fuse_target_axes(axes, value, previous) == expected


def test_axes_rz_does_not_also_move_z() -> None:
    """``"rz"`` contains a ``z``: without the strip, yaw would drag depth along."""
    out = _load_basic_motion().fuse_target_axes(
        "rz", (0.0, 0.0, 99.0, 8.0), (1.0, 2.0, 3.0, 4.0))
    assert out[2] == 3.0, f"axes='rz' must not change z, got {out}"
    assert out[3] == 8.0


# --------------------------------------------------------------------------- #
# The documented step constants match the code (regression for the doc drift)
# --------------------------------------------------------------------------- #
def test_step_constants_match_the_action_document() -> None:
    """The drift found 2026-10-12: doc said 0.2/0.3, code had 0.4/0.2."""
    module = _load_basic_motion()
    doc = ACTION_DOC.read_text(encoding="utf-8")

    def documented(name: str) -> float:
        match = re.search(rf"\|\s*{name}\s*\|\s*([0-9.]+)", doc)
        assert match, f"{name} is not documented in {ACTION_DOC.name}"
        return float(match.group(1))

    assert module.STEP_X == pytest.approx(documented("STEP_X"))
    assert module.STEP_Y == pytest.approx(documented("STEP_Y"))
    assert module.STEP_PERIOD == pytest.approx(documented("STEP_PERIOD"))
    assert module.LATERAL_LAMBDA == pytest.approx(documented("LATERAL_LAMBDA"))


# --------------------------------------------------------------------------- #
# Command serialisation and preemption (bug found 2026-10-12)
# --------------------------------------------------------------------------- #
@pytest.fixture(scope="module")
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


class _Recorder:
    """Stand-in for the setpoint publisher."""

    def __init__(self) -> None:
        self.messages: list = []

    def publish(self, msg) -> None:
        self.messages.append(msg)


class _FakeGoalHandle:
    """Minimal stand-in for an rclpy ServerGoalHandle (no action server needed)."""

    def __init__(self, goal) -> None:
        self.request = goal
        self.is_cancel_requested = False
        self.succeeded = False
        self.aborted = False
        self.canceled_flag = False
        self.feedbacks: list = []

    def succeed(self) -> None:
        self.succeeded = True

    def abort(self) -> None:
        self.aborted = True

    def canceled(self) -> None:
        self.canceled_flag = True

    def publish_feedback(self, feedback) -> None:
        self.feedbacks.append(feedback)


def _set_goal(target, timeout: float = 60.0):
    goal = BasicMotion.Goal()
    goal.cmd_type = BasicMotion.Goal.SET
    goal.target = list(target)
    goal.timeout = float(timeout)
    return goal


def _wait_until(predicate, timeout: float = 2.0, tick: float = 0.02) -> bool:
    import time as _time

    deadline = _time.monotonic() + timeout
    while _time.monotonic() < deadline:
        if predicate():
            return True
        _time.sleep(tick)
    return predicate()


def _node():
    from uv_control.basic_motion import BasicMotionNode

    node = BasicMotionNode()
    node.pub_setpoint = _Recorder()
    return node


def test_new_goal_bumps_the_generation(ros_context) -> None:
    """Preemption is driven by a monotonic generation, not a boolean.

    A boolean flag loses the preemption when three commands meet (A running,
    B and C arriving: B clears the flag on taking the lock and C's request is
    gone, so C can only queue behind B).  The generation is what makes the
    *newest* goal win.
    """
    node = _node()
    try:
        assert node._goal_generation == 0
        node._action_goal_cb(BasicMotion.Goal())
        first = node._goal_generation
        node._action_goal_cb(BasicMotion.Goal())
        assert node._goal_generation == first + 1
    finally:
        node.destroy_node()


def test_running_command_is_interrupted_when_a_newer_goal_arrives(ros_context) -> None:
    node = _node()
    try:
        node._my_generation = node._goal_generation          # 我是当前这一代
        assert node._is_interrupted() is False, (
            "a command must not treat its own generation as an interruption"
        )
        node._action_goal_cb(BasicMotion.Goal())             # 来了更新的 goal
        assert node._is_interrupted() is True, (
            "a newer goal must mark the running command as interrupted"
        )
    finally:
        node.destroy_node()


def test_no_active_command_is_never_interrupted(ros_context) -> None:
    """``_my_generation is None`` means nothing is executing."""
    node = _node()
    try:
        node._action_goal_cb(BasicMotion.Goal())
        assert node._is_interrupted() is False
    finally:
        node.destroy_node()


def test_execute_holds_the_exec_lock_for_the_whole_command(ros_context) -> None:
    """``_exec_lock`` must be held *by execute_callback*, not merely to exist.

    Regression for the observed bug: two goals executed concurrently, so both
    wrote ``/zit6/cmd/setpoint`` and ``_action_goal_handle`` was clobbered.  The
    lock is what removes the second writer, so an idle lock that nothing takes
    would not fix anything.
    """
    node = _node()
    try:
        assert node._exec_lock.acquire(blocking=False) is True, "lock must be free at rest"
        node._exec_lock.release()

        node.start()
        handle = _FakeGoalHandle(_set_goal((10.0, 0.0, 0.0, 0.0), timeout=5.0))
        worker = threading.Thread(
            target=lambda: node._action_execute_cb(handle), daemon=True)
        worker.start()
        try:
            assert _wait_until(lambda: node._exec_lock.acquire(blocking=False) is False,
                               timeout=2.0), (
                "execute_callback did not take _exec_lock, so a second goal could "
                "run concurrently and write the setpoint topic"
            )
        finally:
            node._action_goal_cb(BasicMotion.Goal())   # 打断它，别等 5 s
            worker.join(timeout=5.0)
            assert not worker.is_alive()
        # 命令退出后锁必须释放
        assert node._exec_lock.acquire(blocking=False) is True
        node._exec_lock.release()
    finally:
        node.destroy_node()


def test_interrupted_command_settles_on_a_position_hold(ros_context) -> None:
    """End-to-end: interrupt a running command, the last setpoint is a hold.

    Requirement: "when an action is interrupted it should use position".  Not
    merely "stop writing" -- dropping a command would leave the MCU still
    executing whatever it was last told (fatal for velocity/force in phase 4).
    The interrupted command must leave a position setpoint at the current pose.
    """
    from uv_control.coordinate import Coordinate

    node = _node()
    try:
        node.start()
        # 位姿钉死不动 => SET 永远到不了，命令会一直等到被打断
        node.pose = Coordinate(x=1.5, y=-2.5, z=3.25, rz=42.0)

        handle = _FakeGoalHandle(_set_goal((10.0, 0.0, 0.0, 0.0), timeout=8.0))
        worker = threading.Thread(
            target=lambda: node._action_execute_cb(handle), daemon=True)
        worker.start()
        # 等命令真的发出第一条目标
        assert _wait_until(lambda: len(node.pub_setpoint.messages) >= 1), (
            "the command never wrote a setpoint"
        )
        first = node.pub_setpoint.messages[0]
        assert node._target.x == pytest.approx(10.0, abs=1e-6), (
            "the first setpoint should be the commanded target"
        )

        # 抢占：新一代 goal 到来
        node._action_goal_cb(BasicMotion.Goal())
        worker.join(timeout=6.0)
        assert not worker.is_alive(), "the command did not yield to the newer goal"

        # 收尾必须是一条 position setpoint，目标 = 当前位姿
        assert len(node.pub_setpoint.messages) >= 2, (
            "the interrupted command published no hold setpoint"
        )
        hold = node.pub_setpoint.messages[-1]
        assert hold.control_key == 0, "the hold must be a position setpoint"
        assert hold.type_mask == 0, "the hold must command every axis"
        assert node._target.x == pytest.approx(1.5, abs=1e-6)
        assert node._target.y == pytest.approx(-2.5, abs=1e-6)
        assert node._target.z == pytest.approx(3.25, abs=1e-6)
        assert node._target.rz == pytest.approx(42.0, abs=1e-6)

        assert handle.aborted is True, "a superseded goal must abort, not succeed"
        assert first is not hold
    finally:
        node.destroy_node()


def test_cancel_also_settles_on_a_hold(ros_context) -> None:
    """An explicit ``cancel_goal`` takes the same path as preemption."""
    from uv_control.coordinate import Coordinate

    node = _node()
    try:
        node.start()
        node.pose = Coordinate(x=0.5, y=0.5, z=1.0, rz=10.0)

        handle = _FakeGoalHandle(_set_goal((10.0, 0.0, 0.0, 0.0), timeout=8.0))
        worker = threading.Thread(
            target=lambda: node._action_execute_cb(handle), daemon=True)
        worker.start()
        assert _wait_until(lambda: len(node.pub_setpoint.messages) >= 1)
        handle.is_cancel_requested = True
        worker.join(timeout=6.0)
        assert not worker.is_alive()
        assert len(node.pub_setpoint.messages) >= 2, "cancel published no hold"
        assert node._target.x == pytest.approx(0.5, abs=1e-6)
        assert handle.canceled_flag is True
    finally:
        node.destroy_node()


def test_hold_is_the_shared_primitive_for_stay_and_interrupts() -> None:
    """One implementation, so STAY (phase 3) and interrupts cannot diverge."""
    text = BASIC_MOTION_PY.read_text(encoding="utf-8")
    assert text.count("def _hold_current_position") == 1
    assert "_hold_current_position()" in text, "the interrupt path must call it"
