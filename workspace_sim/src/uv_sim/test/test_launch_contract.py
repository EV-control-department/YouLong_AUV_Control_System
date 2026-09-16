"""T4: the bringup split must stay split, and every entry point must resolve.

Three defects motivated these tests, all of them invisible until someone ran the
stack:

* **B1** -- ``workspace_auv`` shipped simulation entry points, so the control
  stack could not be built or launched without the simulation overlay.
* **B2** -- ``uv_hm/launch/hm_launch.py`` ran ``uv_hm/sim_bridge`` and
  ``uv_control/launch/control_launch.py`` ran ``minimal_control``; neither
  exists, and ``ros2 launch`` only tells you that at run time.
* **B6** -- a dozen documents (and the agents' memory files) told the reader to
  run entry points that no longer existed.

The first two are now checked by ``scripts/check_docs_contract.py``; these tests
prove the checks work (deliberately broken input must fail) and that the
repository is currently clean.
"""

from __future__ import annotations

import ast
import importlib.util
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[4]
CHECKER = REPO_ROOT / "scripts" / "check_docs_contract.py"
WORSKSPACE_AUV_SRC = REPO_ROOT / "workspace_auv" / "src"

#: Packages that only exist in the simulation overlay.
SIM_PACKAGES = {"uv_sim", "uv_sim_bringup", "stonefish_ros2"}


def _load_checker():
    spec = importlib.util.spec_from_file_location("docs_contract_checker_launch", CHECKER)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules["docs_contract_checker_launch"] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def checker():
    return _load_checker()


# --------------------------------------------------------------------------- #
# B1: workspace_auv must stay free of simulation dependencies
# --------------------------------------------------------------------------- #
def _simulation_references(path: Path) -> list[str]:
    """Code-level simulation references in one file (prose does not count).

    A comment mentioning where the simulation presets live is useful; an import,
    a ``package=`` argument or a ``get_package_share_directory`` call is a real
    dependency.  Only the latter are reported.
    """
    tree = ast.parse(path.read_text(encoding="utf-8"))
    found: list[str] = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            found += [f"import {a.name}" for a in node.names if a.name.split(".")[0] in SIM_PACKAGES]
        elif isinstance(node, ast.ImportFrom) and node.module:
            if node.module.split(".")[0] in SIM_PACKAGES:
                found.append(f"from {node.module} import ...")
        elif isinstance(node, ast.Call):
            name = getattr(node.func, "attr", None) or getattr(node.func, "id", None) or ""
            positional = [
                arg.value for arg in node.args
                if isinstance(arg, ast.Constant) and isinstance(arg.value, str)
            ]
            keywords = {
                kw.arg: kw.value.value for kw in node.keywords
                if isinstance(kw.value, ast.Constant) and isinstance(kw.value.value, str)
            }
            if name == "get_package_share_directory" and positional[:1] and positional[0] in SIM_PACKAGES:
                found.append(f"get_package_share_directory({positional[0]!r})")
            elif name == "Node" and keywords.get("package") in SIM_PACKAGES:
                found.append(f"Node(package={keywords['package']!r}, ...)")
            elif name == "include" and positional[:1] and positional[0] in SIM_PACKAGES:
                found.append(f"include({positional[0]!r}, ...)")
    return found


def test_workspace_auv_has_no_simulation_dependencies() -> None:
    offenders: dict[str, list[str]] = {}
    for path in sorted(WORSKSPACE_AUV_SRC.rglob("*.py")):
        if "__pycache__" in path.parts:
            continue
        references = _simulation_references(path)
        if references:
            offenders[path.relative_to(REPO_ROOT).as_posix()] = references
    assert not offenders, (
        "workspace_auv must build and launch without the simulation overlay "
        f"(I6); found code-level simulation references: {offenders}"
    )


def test_the_layering_check_can_actually_fail(tmp_path: Path) -> None:
    """The detector above must flag a file that imports uv_sim."""
    bad = tmp_path / "bringup.py"
    bad.write_text("import uv_sim\nfrom uv_sim import sim_bridge\n", encoding="utf-8")
    assert _simulation_references(bad)
    bad.write_text("from uv_bringup.launch_common import include\n", encoding="utf-8")
    assert not _simulation_references(bad)


# --------------------------------------------------------------------------- #
# B2: a launch file may only run executables that exist
# --------------------------------------------------------------------------- #
def test_launch_targets_reports_a_missing_executable(checker) -> None:
    facts = checker.SourceFacts()
    facts.package_executables = {"uv_control": {"basic_motion"}}
    facts.launch_targets = {
        "uv_control/launch/control_launch.py": [
            ("uv_control", "basic_motion"),      # fine
            ("uv_control", "minimal_control"),   # the historical B2 defect
        ]
    }
    problems = checker.check_launch_targets(facts)
    assert len(problems) == 1, problems
    assert "minimal_control" in str(problems[0])
    assert "basic_motion" in str(problems[0])  # the message lists what does exist


def test_launch_targets_accepts_a_cxx_target(checker) -> None:
    """A C++ executable from CMakeLists.txt is a valid target too."""
    facts = checker.SourceFacts()
    facts.package_executables = {"stonefish_ros2": {"stonefish_simulator_nogpu"}}
    facts.launch_targets = {
        "stonefish_ros2/launch/stonefish_simulator_nogpu.launch.py": [
            ("stonefish_ros2", "stonefish_simulator_nogpu"),
        ]
    }
    assert checker.check_launch_targets(facts) == []


def test_repository_launch_targets_all_exist(checker) -> None:
    facts = checker.collect_source_facts()
    assert facts.launch_targets, "no launch files were parsed -- the check would be vacuous"
    assert checker.check_launch_targets(facts) == []


# --------------------------------------------------------------------------- #
# B6: documentation must not point at entry points that moved
# --------------------------------------------------------------------------- #
def _fake_repo(checker, monkeypatch, tmp_path: Path, markdown: str):
    """Point the checker at a throw-away repository containing one document."""
    monkeypatch.setattr(checker, "REPO_ROOT", tmp_path)
    (tmp_path / "docs").mkdir(parents=True, exist_ok=True)
    (tmp_path / "docs" / "guide.md").write_text(markdown, encoding="utf-8")


def _facts_with_entries(checker, entries: set[str], package: str = "uv_bringup"):
    facts = checker.SourceFacts()
    facts.package_deps = {package: set()}
    facts.launch_files = {package: entries}
    return facts


def test_launch_refs_reports_a_moved_entry_point(checker, monkeypatch, tmp_path) -> None:
    _fake_repo(
        checker, monkeypatch, tmp_path,
        "```bash\nros2 launch uv_bringup sim_bringup.py\n```\n",
    )
    problems = checker.check_launch_refs(_facts_with_entries(checker, {"real_bringup.py"}))
    assert len(problems) == 1, problems
    assert "sim_bringup.py" in str(problems[0])
    assert "real_bringup.py" in str(problems[0])


def test_launch_refs_accepts_a_valid_command(checker, monkeypatch, tmp_path) -> None:
    _fake_repo(
        checker, monkeypatch, tmp_path,
        "ros2 launch uv_bringup real_bringup.py enable_nav:=true\n",
    )
    assert checker.check_launch_refs(_facts_with_entries(checker, {"real_bringup.py"})) == []


def test_launch_refs_ignores_history_and_third_party(checker, monkeypatch, tmp_path) -> None:
    """``devlog/`` records what was true on its date -- never rewrite it."""
    _fake_repo(checker, monkeypatch, tmp_path, "")
    for directory in ("devlog", "third_party", "AQUA-SLAM"):
        nested = tmp_path / directory
        nested.mkdir(parents=True, exist_ok=True)
        (nested / "old.md").write_text(
            "ros2 launch uv_bringup sim_bringup.py\n", encoding="utf-8"
        )
    assert checker.check_launch_refs(_facts_with_entries(checker, {"real_bringup.py"})) == []


def test_repository_launch_references_all_resolve(checker) -> None:
    facts = checker.collect_source_facts()
    assert checker.check_launch_refs(facts) == []


# --------------------------------------------------------------------------- #
# End-to-end: the whole checker runs on the repository without ROS
# --------------------------------------------------------------------------- #
def test_all_checks_pass_on_the_repository(checker) -> None:
    facts = checker.collect_source_facts()
    problems = [problem for name in sorted(checker.CHECKS)
                for problem in checker.CHECKS[name](facts)]
    assert problems == [], "\n".join(str(p) for p in problems)
