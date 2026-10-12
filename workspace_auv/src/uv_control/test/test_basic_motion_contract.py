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
import math
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
        7: "WVELOCITY", 8: "BVELOCITY", 9: "WFORCE", 10: "BFORCE", 11: "STAY",
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


# --------------------------------------------------------------------------- #
# 速度 / 推力 / STAY / duration（第 3、4 期）
# --------------------------------------------------------------------------- #
def _goal(cmd_type, target, *, axes="", timeout=0.0, duration=0.0):
    goal = BasicMotion.Goal()
    goal.cmd_type = cmd_type
    goal.axes = axes
    goal.target = list(target)
    goal.timeout = float(timeout)
    goal.duration = float(duration)
    return goal


def _run_to_completion(node, goal, timeout_s=8.0):
    """在后台线程跑一条命令，返回 (result, goal_handle)。"""
    handle = _FakeGoalHandle(goal)
    worker = threading.Thread(
        target=lambda: node._action_execute_cb(handle), daemon=True)
    worker.start()
    worker.join(timeout=timeout_s)
    assert not worker.is_alive(), "命令没有在预期时间内结束"
    return node, handle


@pytest.mark.parametrize("cmd_type,control_key", [
    ("WVELOCITY", 0x01), ("BVELOCITY", 0x11),
    ("WFORCE", 0x02), ("BFORCE", 0x12),
])
def test_velocity_and_force_use_the_right_control_key(ros_context, cmd_type, control_key) -> None:
    """control_key 的 frame 位必须真的发出去 —— 那是世界系/机体系的唯一表达。

    B21 之前仿真忽略这个位，世界系命令会退化成机体系；即使仿真已修好，这里
    发错位就等于选错坐标系。
    """
    from uv_control.coordinate import Coordinate

    node = _node()
    try:
        node.start()
        node.pose = Coordinate(x=0.0, y=0.0, z=1.0, rz=0.0)
        goal = _goal(getattr(BasicMotion.Goal, cmd_type),
                     (0.3, -0.2, 0.1, 15.0), duration=0.3)
        _run_to_completion(node, goal)
        sent = [m for m in node.pub_setpoint.messages
                if (m.control_key & 0x03) in (1, 2)]
        assert sent, f"{cmd_type} 没发出任何速度/推力 setpoint"
        assert sent[0].control_key == control_key, (
            f"{cmd_type} 应发 control_key={control_key:#04x}，"
            f"实际 {sent[0].control_key:#04x}")
    finally:
        node.destroy_node()


def test_stay_latches_at_the_current_pose_and_returns_at_once(ros_context) -> None:
    """STAY 是纯 latch：设目标=当前位姿后立即返回，没有 duration。"""
    from uv_control.coordinate import Coordinate
    import time as _time

    node = _node()
    try:
        node.start()
        node.pose = Coordinate(x=4.0, y=-1.5, z=2.75, rz=-30.0)
        start = _time.monotonic()
        _, handle = _run_to_completion(
            node, _goal(BasicMotion.Goal.STAY, (0.0, 0.0, 0.0, 0.0)))
        elapsed = _time.monotonic() - start
        assert elapsed < 1.0, "STAY 必须立即返回，不能阻塞"
        assert handle.succeeded is True
        msg = node.pub_setpoint.messages[-1]
        assert msg.control_key == 0, "STAY 必须用位置模式"
        assert node._target.x == pytest.approx(4.0, abs=1e-6)
        assert node._target.y == pytest.approx(-1.5, abs=1e-6)
        assert node._target.z == pytest.approx(2.75, abs=1e-6)
        assert node._target.rz == pytest.approx(-30.0, abs=1e-6)
    finally:
        node.destroy_node()


def test_effective_duration_precedence() -> None:
    """duration > timeout > DEFAULT_DURATION；速度命令**必须**有界。"""
    module = _load_basic_motion()
    node_fn = module.BasicMotionNode.effective_duration

    class _Stub:
        pass

    stub = _Stub()
    assert node_fn(stub, 3.0, 99.0) == pytest.approx(3.0), "duration 优先"
    assert node_fn(stub, 0.0, 7.5) == pytest.approx(7.5), "无 duration 时回退 timeout"
    assert node_fn(stub, 0.0, 0.0) == pytest.approx(module.DEFAULT_DURATION)
    assert node_fn(stub, -1.0, -1.0) == pytest.approx(module.DEFAULT_DURATION)


def test_velocity_command_ends_with_a_position_hold(ros_context) -> None:
    """速度命令**正常到期**也必须收尾。

    否则 MCU 会一直执行最后一条速度目标 —— "命令结束了但船还在走"。
    """
    from uv_control.coordinate import Coordinate

    node = _node()
    try:
        node.start()
        node.pose = Coordinate(x=1.0, y=2.0, z=3.0, rz=45.0)
        goal = _goal(BasicMotion.Goal.BVELOCITY, (0.5, 0.0, 0.0, 0.0),
                     duration=0.3)
        _, handle = _run_to_completion(node, goal)
        assert handle.succeeded is True
        # 第一条是速度，最后一条必须变成位置保持
        assert (node.pub_setpoint.messages[0].control_key & 0x03) == 1
        last = node.pub_setpoint.messages[-1]
        assert last.control_key == 0, "速度命令到期后必须收尾为位置保持"
        assert node._target.x == pytest.approx(1.0, abs=1e-6)
        assert node._target.z == pytest.approx(3.0, abs=1e-6)
    finally:
        node.destroy_node()


def test_velocity_axes_fusion_reuses_the_previous_velocity_target(ros_context) -> None:
    """D10=A：`axes` 沿用**上一条速度**目标（不是位置目标，也不清零）。"""
    from uv_control.coordinate import Coordinate

    node = _node()
    try:
        node.start()
        node.pose = Coordinate(x=0.0, y=0.0, z=1.0, rz=0.0)
        _run_to_completion(node, _goal(
            BasicMotion.Goal.BVELOCITY, (0.5, 0.2, 0.0, 10.0), duration=0.2))
        assert node._velocity_target == {
            'x': pytest.approx(0.5), 'y': pytest.approx(0.2),
            'z': pytest.approx(0.0), 'rz': pytest.approx(10.0)}

        _run_to_completion(node, _goal(
            BasicMotion.Goal.BVELOCITY, (0.3, 0.0, 0.0, 0.0),
            axes="x", duration=0.2))
        assert node._velocity_target['x'] == pytest.approx(0.3), "x 应更新"
        assert node._velocity_target['y'] == pytest.approx(0.2), "y 应沿用"
        assert node._velocity_target['rz'] == pytest.approx(10.0), "rz 应沿用"
    finally:
        node.destroy_node()


def test_duration_on_a_position_command_is_ignored_not_silently_dropped(ros_context) -> None:
    """位置类传 duration 要告警（不能静默忽略一个用户以为生效的参数）。"""
    from uv_control.coordinate import Coordinate

    node = _node()
    try:
        node.start()
        node.pose = Coordinate(x=0.0, y=0.0, z=0.0, rz=0.0)
        warnings = []
        node.get_logger().warning = lambda msg: warnings.append(str(msg))
        # 目标=当前位姿 => 立即到位，不会真等
        _run_to_completion(node, _goal(
            BasicMotion.Goal.SET, (0.0, 0.0, 0.0, 0.0),
            timeout=2.0, duration=5.0))
        assert any("duration" in w and "忽略" in w for w in warnings), (
            f"位置类传 duration 应该告警，实际 warnings={warnings}")
    finally:
        node.destroy_node()


def test_emergency_stop_sends_zero_body_force(ros_context) -> None:
    """退出收尾：推力归零必须用机体系推力的 control_key 且全 0。"""
    node = _node()
    try:
        node.emergency_stop()
        assert node.pub_setpoint.messages, "急停没有发出任何 setpoint"
        for msg in node.pub_setpoint.messages:
            assert msg.control_key == 0x12, (
                f"急停应为机体系推力 0x12，实际 {msg.control_key:#04x}")
            assert (msg.x, msg.y, msg.z, msg.yaw) == (0.0, 0.0, 0.0, 0.0)
    finally:
        node.destroy_node()


def test_main_registers_a_sigterm_handler() -> None:
    """`kill`(SIGTERM) 不会让 spin() 抛 KeyboardInterrupt，必须自己转换。

    否则 `ros2 launch` 的某些收尾路径会直接杀掉进程，推力归零来不及发。
    """
    text = BASIC_MOTION_PY.read_text(encoding="utf-8")
    assert "signal.SIGTERM" in text, "main() 没有处理 SIGTERM"
    assert "node.emergency_stop()" in text, "退出路径没有收尾"


def test_feedback_contains_the_new_fields(ros_context) -> None:
    """反馈必须带 velocity_current / velocity_target / control_level。"""
    node = _node()
    try:
        from uv_control.coordinate import Coordinate
        node.start()
        node.pose = Coordinate(x=0.0, y=0.0, z=0.0, rz=0.0)
        node.vel_body = {'x': 0.4, 'y': -0.1, 'z': 0.05,
                         'rx': 0.0, 'ry': 0.0, 'rz': math.pi / 6.0}   # rad/s
        node._velocity_target = {'x': 0.4, 'y': -0.1, 'z': 0.05, 'rz': 30.0}
        node._action_continuous = True
        handle = _FakeGoalHandle(_goal(BasicMotion.Goal.BVELOCITY,
                                       (0.4, -0.1, 0.05, 30.0), duration=0.2))
        node._action_goal_handle = handle
        node._action_target = None

        node._action_feedback_cb()

        assert handle.feedbacks, "没有发出反馈"
        fb = handle.feedbacks[-1]
        assert fb.distance_remaining == pytest.approx(0.0), (
            "速度命令没有位置目标，distance_remaining 应为 0")
        assert list(fb.velocity_target) == pytest.approx([0.4, -0.1, 0.05, 30.0])
        # rad/s -> 度/秒：π/6 rad/s = 30 °/s
        assert list(fb.velocity_current) == pytest.approx(
            [0.4, -0.1, 0.05, 30.0], abs=1e-6)
        assert fb.control_level == int(getattr(node.status, 'control_level', 0))
    finally:
        node.destroy_node()


def test_request_shutdown_makes_a_running_command_yield(ros_context) -> None:
    """退出前必须能打断正在执行的命令，否则进程要等它跑满 duration。

    实测（2026-10-12）：带一条 60 s 速度命令时，收到 SIGINT 后 15 s 进程仍未退出，
    最后被 SIGKILL —— 因为 `executor.shutdown()` 会等回调返回。`request_shutdown()`
    把代号 +1 让命令立刻让位。
    """
    from uv_control.coordinate import Coordinate

    node = _node()
    try:
        node.start()
        node.pose = Coordinate(x=0.0, y=0.0, z=1.0, rz=0.0)
        handle = _FakeGoalHandle(_goal(
            BasicMotion.Goal.BVELOCITY, (0.5, 0.0, 0.0, 0.0), duration=60.0))
        worker = threading.Thread(
            target=lambda: node._action_execute_cb(handle), daemon=True)
        worker.start()
        assert _wait_until(lambda: len(node.pub_setpoint.messages) >= 1)

        import time as _time
        start = _time.monotonic()
        assert node.request_shutdown(timeout=5.0) is True, (
            "request_shutdown 没能让正在执行的长命令让位")
        worker.join(timeout=5.0)
        elapsed = _time.monotonic() - start
        assert not worker.is_alive(), "命令没有退出"
        assert elapsed < 3.0, f"让位太慢（{elapsed:.2f}s）"
        # 命令必须是被打断收尾的，不是"自己跑完了"
        assert handle.succeeded is False
        assert node._target is not None, "打断应留下一条位置保持"
    finally:
        node.destroy_node()


def test_request_shutdown_is_harmless_with_nothing_running(ros_context) -> None:
    node = _node()
    try:
        assert node.request_shutdown(timeout=1.0) is True
    finally:
        node.destroy_node()


def test_shutdown_path_interrupts_before_zeroing_thrust() -> None:
    """顺序必须是「先打断命令 → 再推力归零」。

    反过来的话，被打断的命令会在推力归零**之后**再发一条位置保持，把收尾覆盖掉。
    """
    text = BASIC_MOTION_PY.read_text(encoding="utf-8")
    main_body = text[text.index("def main("):]
    # 用带 `node.` 前缀的调用形式定位，避免匹配到注释里提到的 `emergency_stop()`
    interrupt_at = main_body.index("node.request_shutdown()")
    zero_at = main_body.index("node.emergency_stop()")
    assert interrupt_at < zero_at, (
        "main() 里必须先 node.request_shutdown() 再 node.emergency_stop()"
    )
