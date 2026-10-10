"""The message types of **scene-declared** publishers, pinned to their C++ evidence.

``docs/interfaces/topic_contract.md`` is checked against the source by
``scripts/check_docs_contract.py``, but that check cannot verify one class of row:
a Stonefish ``<ros_publisher topic=.../>`` is declared in a ``.scn`` file and the
message is filled by C++ the checker does not parse.  For those topics the checker
only proves "the scene declares this topic", so **the type column was a free
claim** -- and on the first live snapshot (T3b, 2026-10-10) two of them turned out
to be wrong:

* ``/auv/dvl_altitude``: documented ``std_msgs/Float32``, actually
  ``sensor_msgs/Range`` (``ROS2Interface.cpp`` ``PublishDVLAltitude`` publishes
  ``rclcpp::Publisher<sensor_msgs::msg::Range>``);
* ``/auv/thrusters_state``: documented ``std_msgs/Float64MultiArray``, actually
  ``stonefish_ros2/ThrusterState`` (``ROS2ScenarioParser.cpp`` creates
  ``create_publisher<stonefish_ros2::msg::ThrusterState>``).

So each row here carries the evidence that makes it checkable: the C++ function (or
line) that publishes that type, plus the contract row that must agree.  If upstream
ever changes a publisher's type, this test fails on the evidence rather than
letting the contract drift again.
"""

from __future__ import annotations

import importlib.util
import re
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[4]
INTERFACE_CPP = (
    REPO_ROOT / "workspace_sim" / "src" / "stonefish_ros2" / "src"
    / "stonefish_ros2" / "ROS2Interface.cpp"
)
PARSER_CPP = (
    REPO_ROOT / "workspace_sim" / "src" / "stonefish_ros2" / "src"
    / "stonefish_ros2" / "ROS2ScenarioParser.cpp"
)
SCENE = REPO_ROOT / "workspace_sim" / "src" / "stonefish_ros2" / "Data" / "xunyun_fixed.scn"
CHECKER = REPO_ROOT / "scripts" / "check_docs_contract.py"

#: ``(topic, contract type, C++ type, evidence)`` for every scene-declared publisher
#: whose ROS type the source checker cannot derive.  ``evidence`` is either a
#: ``ROS2Interface::PublishX`` function name or a ``file:line`` anchor.
SCENE_PUBLISHERS = (
    ("/auv/odometry", "nav_msgs/Odometry",
     "nav_msgs::msg::Odometry", "fn:PublishOdometry"),
    ("/auv/imu", "sensor_msgs/Imu",
     "sensor_msgs::msg::Imu", "fn:PublishIMU"),
    ("/auv/dvl", "stonefish_ros2/DVL",
     "stonefish_ros2::msg::DVL", "fn:PublishDVL"),
    ("/auv/dvl_altitude", "sensor_msgs/Range",
     "sensor_msgs::msg::Range", "fn:PublishDVLAltitude"),
    ("/auv/pressure", "sensor_msgs/FluidPressure",
     "sensor_msgs::msg::FluidPressure", "fn:PublishPressure"),
    ("/auv/thrusters_state", "stonefish_ros2/ThrusterState",
     "stonefish_ros2::msg::ThrusterState", "line:ROS2ScenarioParser.cpp:442"),
)


def _function_body(path: Path, name: str) -> str:
    """Body of ``ROS2Interface::<name>`` (up to the next top-level definition)."""
    text = path.read_text(encoding="utf-8")
    start = text.index(f"void ROS2Interface::{name}(")
    nxt = re.search(r"\nvoid ROS2Interface::", text[start + 1:])
    return text[start:start + 1 + nxt.start()] if nxt else text[start:]


def _evidence_source(evidence: str) -> tuple[Path, str]:
    """``(file, text)`` the evidence anchor lives in, for the type assertion."""
    if evidence.startswith("fn:"):
        return INTERFACE_CPP, _function_body(INTERFACE_CPP, evidence[3:])
    _, filename, line = evidence.split(":")
    path = INTERFACE_CPP if filename == "ROS2Interface.cpp" else PARSER_CPP
    lines = path.read_text(encoding="utf-8").splitlines()
    return path, lines[int(line) - 1]


@pytest.fixture(scope="module")
def checker():
    spec = importlib.util.spec_from_file_location("docs_contract_checker_types", CHECKER)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules["docs_contract_checker_types"] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def contract_types(checker) -> dict[str, str]:
    """``topic -> 消息类型`` for every publisher row of the contract."""
    declared: dict[str, str] = {}
    text = checker.DOC_TOPIC_CONTRACT.read_text(encoding="utf-8")
    for _heading, rows in checker.parse_tables(text):
        for row in rows:
            if {"话题", "消息类型", "方向"} <= set(row) and row["方向"] == "发布":
                declared.setdefault(row["话题"].strip(), row["消息类型"].strip())
    assert declared, "no publisher rows parsed from the topic contract"
    return declared


# --------------------------------------------------------------------------- #
# The evidence still says what we think it says
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("topic,contract_type,cpp_type,evidence", SCENE_PUBLISHERS,
                         ids=[row[0] for row in SCENE_PUBLISHERS])
def test_cpp_evidence_publishes_the_expected_type(topic, contract_type, cpp_type, evidence) -> None:
    _path, text = _evidence_source(evidence)
    assert f"Publisher<{cpp_type}>" in text or f"create_publisher<{cpp_type}>" in text, (
        f"{topic}: the evidence ({evidence}) no longer publishes "
        f"{cpp_type!r} -- fix this table and the contract together"
    )


# --------------------------------------------------------------------------- #
# The contract agrees with the evidence
# --------------------------------------------------------------------------- #
def _type_problems(contract_types: dict[str, str]) -> list[str]:
    """Rows whose documented type disagrees with the C++ evidence (empty = clean)."""
    problems = []
    for topic, contract_type, cpp_type, evidence in SCENE_PUBLISHERS:
        documented = contract_types.get(topic)
        if documented is None:
            problems.append(f"{topic}: no publisher row in the contract")
        elif documented != contract_type:
            problems.append(
                f"{topic}: contract says {documented!r}, C++ ({evidence}) says {contract_type!r}"
            )
    return problems


def test_contract_types_match_the_cpp_evidence(contract_types) -> None:
    problems = _type_problems(contract_types)
    assert not problems, "scene-publisher types drifted from the C++ evidence:\n  " + "\n  ".join(problems)


def test_every_pinned_topic_is_actually_declared_by_the_scene(checker) -> None:
    """A row for a topic the scene does not publish would be a dead assertion."""
    facts = checker.collect_source_facts()
    missing = [row[0] for row in SCENE_PUBLISHERS
               if row[0] not in facts.scene_topics and row[0] != "/auv/thrusters_state"]
    assert not missing, (
        f"these topics are pinned here but {SCENE.name} does not declare them: {missing}"
    )


def test_the_comparison_catches_the_drift_this_test_was_written_for(contract_types) -> None:
    """Re-introduce the 2026-10-10 defect and prove the guard reports it."""
    broken = dict(contract_types)
    broken["/auv/dvl_altitude"] = "std_msgs/Float32"      # the value the contract had
    broken["/auv/thrusters_state"] = "std_msgs/Float64MultiArray"

    problems = _type_problems(broken)
    assert len(problems) == 2, problems
    assert any("/auv/dvl_altitude" in p and "std_msgs/Float32" in p for p in problems), problems
    assert any("/auv/thrusters_state" in p for p in problems), problems

    # and the real contract is clean
    assert _type_problems(contract_types) == []
