"""T3a: the documentation checker must actually catch drift.

``scripts/check_docs_contract.py`` is only worth keeping if a wrong document makes
it fail, so these tests feed it **deliberately broken copies** of the docs and
assert each check reports the problem.  The real docs are never modified -- every
case rewrites a ``tmp_path`` copy and calls the pure check functions.

The last test runs the checker against the repository as-is, which is the
"documentation is currently in sync" claim.
"""

from __future__ import annotations

import importlib.util
import re
import shutil
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[4]
CHECKER = REPO_ROOT / "scripts" / "check_docs_contract.py"


def _load_checker():
    spec = importlib.util.spec_from_file_location("docs_contract_checker", CHECKER)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules["docs_contract_checker"] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def checker():
    return _load_checker()


def _copy_doc(checker, module_attr: str, tmp_path: Path, mutate) -> Path:
    """Copy a real doc into ``tmp_path``, mutate it, and point the checker at it."""
    original = getattr(checker, module_attr)
    target = tmp_path / original.name
    shutil.copy(original, target)
    text = target.read_text(encoding="utf-8")
    target.write_text(mutate(text), encoding="utf-8")
    return target


# --------------------------------------------------------------------------- #
# The check must find each class of drift
# --------------------------------------------------------------------------- #
def test_missing_parameter_is_reported(checker, tmp_path: Path, monkeypatch) -> None:
    """Drop one declared parameter from the doc table -> parameters check fails."""
    doc = _copy_doc(
        checker,
        "DOC_NODE_PARAMS",
        tmp_path,
        lambda text: re.sub(r"^\| `pid_config_file` \|.*\n", "", text, flags=re.M),
    )
    monkeypatch.setattr(checker, "DOC_NODE_PARAMS", doc)
    facts = checker.collect_source_facts()
    problems = checker.check_parameters(facts)
    assert any("pid_config_file" in str(p) for p in problems), problems


def test_bogus_parameter_is_reported(checker, tmp_path: Path, monkeypatch) -> None:
    """A parameter documented but not declared anywhere -> parameters check fails."""
    doc = _copy_doc(
        checker,
        "DOC_NODE_PARAMS",
        tmp_path,
        lambda text: text.replace(
            "| `pid_config_file` | string | `''` |",
            "| `pid_config_file` | string | `''` |\n| `does_not_exist` | bool | `false` |",
        ),
    )
    monkeypatch.setattr(checker, "DOC_NODE_PARAMS", doc)
    problems = checker.check_parameters(checker.collect_source_facts())
    assert any("does_not_exist" in str(p) for p in problems), problems


def test_launch_argument_is_not_treated_as_a_node_parameter(checker) -> None:
    """Launch arguments share the 参数 column; the chapter decides which they are.

    Regression: the first run of the checker reported every ``enable_ai`` /
    ``scenario`` as an undeclared ROS parameter.

    The two probes must each be unambiguous: ``enable_mcu`` is a launch argument
    only, ``heartbeat_rate`` a node parameter only.  (``pid_config_file`` would not
    do: since T5 it is legitimately both -- ``mcu_sim`` declares it and
    ``bridge_launch.py`` exposes it as a launch argument.  Before T5 the same was
    true of ``hil_mode``.)
    """
    documented_params = checker.doc_parameters(checker.DOC_NODE_PARAMS)
    documented_launch = checker.doc_launch_arguments(checker.DOC_NODE_PARAMS)
    assert "enable_mcu" in documented_launch
    assert "enable_mcu" not in documented_params
    assert "heartbeat_rate" in documented_params
    assert "heartbeat_rate" not in documented_launch


def test_removed_node_is_reported(checker, tmp_path: Path, monkeypatch) -> None:
    """Delete a row from the executable table -> nodes check fails."""
    doc = _copy_doc(
        checker,
        "DOC_NODE_PARAMS",
        tmp_path,
        lambda text: re.sub(r"^\| `task_runner` \|.*\n", "", text, flags=re.M),
    )
    monkeypatch.setattr(checker, "DOC_NODE_PARAMS", doc)
    problems = checker.check_nodes(checker.collect_source_facts())
    assert any("task_runner" in str(p) for p in problems), problems


def test_wrong_message_type_is_reported(checker, tmp_path: Path, monkeypatch) -> None:
    """Corrupt a contract row's message type -> topics check fails."""
    doc = _copy_doc(
        checker,
        "DOC_TOPIC_CONTRACT",
        tmp_path,
        lambda text: re.sub(
            r"\| `/perception/objects` \| `uv_msgs/ObjectPositionArray` \| 发布 \|",
            "| `/perception/objects` | `uv_msgs/PoseInfo` | 发布 |",
            text,
            count=1,
        ),
    )
    monkeypatch.setattr(checker, "DOC_TOPIC_CONTRACT", doc)
    problems = checker.check_topics(checker.collect_source_facts())
    assert any("/perception/objects" in str(p) for p in problems), problems


def test_wrong_evidence_file_is_reported(checker, tmp_path: Path, monkeypatch) -> None:
    """Point a row at a file that does not have that publisher -> topics fails."""
    doc = _copy_doc(
        checker,
        "DOC_TOPIC_CONTRACT",
        tmp_path,
        lambda text: text.replace(
            "| `/nav/path` | `uv_msgs/WaypointPath` | 发布 | `navigator` | — | 两者 | "
            "`workspace_auv/src/uv_nav/uv_nav/navigator.py:create_publisher` |",
            "| `/nav/path` | `uv_msgs/WaypointPath` | 发布 | `navigator` | — | 两者 | "
            "`workspace_auv/src/uv_nav/uv_nav/astar.py:create_publisher` |",
        ),
    )
    monkeypatch.setattr(checker, "DOC_TOPIC_CONTRACT", doc)
    problems = checker.check_topics(checker.collect_source_facts())
    assert any("/nav/path" in str(p) for p in problems), problems


def test_missing_dependency_exemption_is_reported(checker, tmp_path: Path, monkeypatch) -> None:
    """An unregistered gap fails; the same gap with the 已知 marker is exempt.

    Uses synthetic facts instead of a gap that happens to exist in the repository
    today: T4 fixed the only real one (D1, ``uv_bringup`` referencing ``uv_sim``),
    and a test that depends on a live defect disappears the moment it is fixed.
    Both directions are asserted here, so the exemption mechanism stays covered.
    """
    gap = {
        "workspace_auv/src/uv_bringup/launch/real_bringup.py": {"uv_sim"},
    }
    facts = checker.SourceFacts()
    facts.package_deps = {"uv_bringup": set()}
    facts.launch_refs = gap

    problems = checker.check_dependencies(facts)
    assert any("uv_sim" in str(p) for p in problems), problems

    doc = _copy_doc(
        checker,
        "DOC_NODE_DEPENDENCY",
        tmp_path,
        lambda text: text + (
            "\n| # | 缺口 | 影响 | 豁免 |\n"
            "|---|---|---|---|\n"
            "| G-test | `uv_bringup` 的 launch 引用 `uv_sim` | 演示用 | **已知** |\n"
        ),
    )
    monkeypatch.setattr(checker, "DOC_NODE_DEPENDENCY", doc)
    assert checker.check_dependencies(facts) == []


def test_undeclared_third_party_import_is_reported(checker, tmp_path: Path, monkeypatch) -> None:
    """``import cv2`` without a manifest entry must be reported.

    T4 fix: the collector used to record only *workspace* imports, which made
    this whole branch unreachable -- the registered cv2/numpy/ultralytics gaps
    were documented but never enforced.
    """
    doc = _copy_doc(checker, "DOC_NODE_DEPENDENCY", tmp_path, lambda text: "")
    monkeypatch.setattr(checker, "DOC_NODE_DEPENDENCY", doc)

    facts = checker.SourceFacts()
    facts.package_deps = {"uv_perception": set()}
    facts.package_imports = {"uv_perception": {"cv2"}}
    problems = checker.check_dependencies(facts)
    assert any("cv2" in str(p) for p in problems), problems


def test_stdlib_imports_are_not_dependencies(checker, tmp_path: Path, monkeypatch) -> None:
    """``os``/``json`` are not manifest dependencies and must stay silent."""
    doc = _copy_doc(checker, "DOC_NODE_DEPENDENCY", tmp_path, lambda text: "")
    monkeypatch.setattr(checker, "DOC_NODE_DEPENDENCY", doc)

    facts = checker.SourceFacts()
    facts.package_deps = {"uv_control": set()}
    facts.package_imports = {"uv_control": {"os", "json", "threading", "sys"}}
    assert checker.check_dependencies(facts) == []


def test_rosdep_python_key_satisfies_the_import(checker, tmp_path: Path, monkeypatch) -> None:
    """``<exec_depend>python3-numpy</exec_depend>`` must cover ``import numpy``.

    ROS packages declare Python dependencies the rosdep way while importing the
    bare module name; comparing them literally forced keys ``rosdep`` cannot
    resolve.  (Found while fixing gap D3 in T5.)
    """
    doc = _copy_doc(checker, "DOC_NODE_DEPENDENCY", tmp_path, lambda text: "")
    monkeypatch.setattr(checker, "DOC_NODE_DEPENDENCY", doc)

    facts = checker.SourceFacts()
    facts.package_deps = {"uv_sim": {"python3-numpy"}}
    facts.package_imports = {"uv_sim": {"numpy"}}
    assert checker.check_dependencies(facts) == []

    facts.package_deps = {"uv_sim": {"python3-yaml"}}
    problems = checker.check_dependencies(facts)
    assert any("numpy" in str(p) for p in problems), problems


def test_unlisted_doc_is_reported(checker, tmp_path: Path, monkeypatch) -> None:
    """A doc that the README does not mention -> readme check fails.

    The README is rewritten from an allow-list so that a real, unlisted file
    (here: the topic contract) becomes the only failure.
    """
    listed = "\n".join(
        path.name
        for path in sorted((REPO_ROOT / "docs").rglob("*.md"))
        if path.name != "topic_contract.md" and path.name != "README.md"
    )
    readme = _copy_doc(checker, "DOC_README", tmp_path, lambda text: listed + "\n")
    monkeypatch.setattr(checker, "DOC_README", readme)
    problems = checker.check_readme(checker.collect_source_facts())
    assert any("topic_contract.md" in str(p) for p in problems), problems
    assert len(problems) == 1, problems


# --------------------------------------------------------------------------- #
# The repository itself must be in sync
# --------------------------------------------------------------------------- #
def test_repository_docs_match_the_code(checker) -> None:
    """The committed docs pass every check (this is the A3-2 acceptance)."""
    facts = checker.collect_source_facts()
    problems: list[str] = []
    for name, check in sorted(checker.CHECKS.items()):
        problems.extend(f"{name}: {p}" for p in check(facts))
    assert not problems, "documentation drifted from the code:\n" + "\n".join(problems)


def test_contract_covers_every_topic_the_code_declares(checker) -> None:
    """No publish/subscribe call may be missing from the contract.

    The contract is the single source of truth for topics, so a new
    ``create_publisher`` that nobody documented must fail loudly rather than
    living only in the code.
    """
    facts = checker.collect_source_facts()
    text = checker.DOC_TOPIC_CONTRACT.read_text(encoding="utf-8")
    missing = [
        f"{kind} {topic} ({sorted(files)})"
        for (topic, kind), files in sorted(facts.topics.items())
        if topic not in text
    ]
    assert not missing, (
        "these ROS endpoints are not in docs/interfaces/topic_contract.md:\n  "
        + "\n  ".join(missing)
    )


def test_wrong_default_value_is_reported(checker, tmp_path: Path, monkeypatch) -> None:
    """A documented default that disagrees with the code -> parameters check fails.

    Names alone are not enough: the first version of the checker passed while
    `max_history` claimed a default of 999.
    """
    doc = _copy_doc(
        checker,
        "DOC_NODE_PARAMS",
        tmp_path,
        lambda text: text.replace(
            "| `max_history` | int | `30` |", "| `max_history` | int | `999` |"
        ),
    )
    monkeypatch.setattr(checker, "DOC_NODE_PARAMS", doc)
    problems = checker.check_parameters(checker.collect_source_facts())
    assert any("max_history" in str(p) and "999" in str(p) for p in problems), problems


def test_boolean_default_normalisation(checker) -> None:
    """`false` in markdown and `False` in python are the same default."""
    assert checker._documented_default_matches("False", "false")
    assert checker._documented_default_matches("False", "**`false`**")
    assert not checker._documented_default_matches("False", "true")
    # Unknown forms (a module constant) must never produce a false failure.
    assert checker._documented_default_matches("SIM_MODE", "false")
