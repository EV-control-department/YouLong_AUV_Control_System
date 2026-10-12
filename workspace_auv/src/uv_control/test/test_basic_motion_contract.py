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
from pathlib import Path

import pytest

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
