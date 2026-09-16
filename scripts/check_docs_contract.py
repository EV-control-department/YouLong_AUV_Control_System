#!/usr/bin/env python3
"""T3a: check that the documentation still matches the source code.

Runs with the Python standard library only -- **no ROS, no colcon, no running
stack** -- so it works in a bare sandbox and in CI.  It compares four kinds of
fact, each with the code as the source of truth:

===========  ==============================================  ==========================
kind         code side                                       doc side
===========  ==============================================  ==========================
parameters   ``declare_parameter`` / ``DeclareLaunchArgument`` ``docs/node_parameters.md``
nodes        ``setup.py`` ``console_scripts``                 ``docs/node_parameters.md``
topics       ``create_publisher`` / ``create_subscription`` /  ``docs/interfaces/topic_contract.md``
             ``create_service`` + ``.scn`` ``<ros_publisher>``
dependencies ``package.xml`` + real imports + launch refs     ``docs/architecture/node_dependency.md``
===========  ==============================================  ==========================

Usage::

    python3 scripts/check_docs_contract.py --check      # drift -> exit 1
    python3 scripts/check_docs_contract.py --update     # refresh the generated inventory
    python3 scripts/check_docs_contract.py --check --quiet

Design notes
------------
* **No line numbers anywhere.**  Evidence is ``file:create_publisher`` style; line
  numbers drift and would make the check noisy (learned in T2).
* **Documentation is the parsed artifact, code is the truth.**  There is no golden
  JSON to keep in sync; the inventory written by ``--update`` is a review artifact.
* **Known gaps must be declared, not silently tolerated.**  A missing dependency
  fails unless the row is listed in ``node_dependency.md`` section 2 with the
  ``已知`` exemption marker; that is how a "we know, it is planned" stays visible.
"""

from __future__ import annotations

import argparse
import ast
import importlib.util
import json
import re
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]

DOC_NODE_PARAMS = REPO_ROOT / "docs" / "node_parameters.md"
DOC_TOPIC_CONTRACT = REPO_ROOT / "docs" / "interfaces" / "topic_contract.md"
DOC_NODE_DEPENDENCY = REPO_ROOT / "docs" / "architecture" / "node_dependency.md"
DOC_README = REPO_ROOT / "docs" / "README.md"
INVENTORY_JSON = REPO_ROOT / "docs" / "interfaces" / "snapshots" / "code_inventory.json"

SOURCE_ROOTS = (REPO_ROOT / "workspace_auv" / "src", REPO_ROOT / "workspace_sim" / "src")
EXTRA_SOURCE_DIRS = (REPO_ROOT / "visualization",)
SCENE_FILES = (REPO_ROOT / "workspace_sim" / "src" / "stonefish_ros2" / "Data" / "xunyun_fixed.scn",)

SKIP_DIR_PARTS = {"build", "install", "log", "__pycache__", "test", "tests", "resource", "launch"}

#: Detector channels the f-string topics expand to (vision publishes all four).
CHANNELS = ("front_left", "front_right", "down_left", "down_right")

#: Stonefish appends this to a camera `<ros_publisher topic=...>`.
CAMERA_TOPIC_SUFFIX = "/image_color"

#: Import top-level names that are not workspace packages and are not interesting
#: for the dependency check (stdlib is filtered separately by find_spec).
IGNORED_IMPORTS = {
    "ament_index_python", "launch", "launch_ros", "setuptools", "pytest",
}

#: Launch files whose arguments are NOT checked against node_parameters.md.
#: `stonefish_simulator*.launch.py` ship with the C++ engine wrapper (six args
#: like window_res_x / rendering_quality); the project's own entry points live in
#: `uv_bringup/launch/*`. Documenting the engine ones is a separate task, tracked
#: in node_parameters.md's "待补清单".
UNCHECKED_LAUNCH_FILES = {"stonefish_simulator.launch.py", "stonefish_simulator_nogpu.launch.py"}

#: ``/perception/detection/{cam}`` f-strings in task_runner/line_follower are expanded
#: with all four channels; the contract lists only the pairs those modules subscribe to.
TEMPLATE_CHANNELS = {
    "{ch}": CHANNELS,
    "{cam}": ("front_left", "front_right", "down_left", "down_right"),
}


def rel(path: Path) -> str:
    """Repository-relative path for messages (stable across machines)."""
    try:
        return str(path.relative_to(REPO_ROOT))
    except ValueError:
        return str(path)


@dataclass
class Problem:
    kind: str
    message: str

    def __str__(self) -> str:  # pragma: no cover - formatting only
        return f"[{self.kind}] {self.message}"


@dataclass
class SourceFacts:
    """Everything extracted from code, in a form the checks can diff."""

    parameters: dict[str, dict] = field(default_factory=dict)       # name -> {default, decl}
    launch_arguments: dict[str, dict] = field(default_factory=dict)  # name -> {default, file}
    entries: dict[str, dict] = field(default_factory=dict)           # node -> {package, target}
    topics: dict[tuple[str, str], set[str]] = field(default_factory=dict)  # (topic, kind) -> {file}
    topic_types: dict[tuple[str, str], str] = field(default_factory=dict)  # (topic, kind) -> msg type
    package_deps: dict[str, set[str]] = field(default_factory=dict)  # package -> declared deps
    package_imports: dict[str, set[str]] = field(default_factory=dict)   # package -> imported workspace pkgs
    launch_refs: dict[str, set[str]] = field(default_factory=dict)   # launch file -> referenced packages
    scene_topics: dict[str, str] = field(default_factory=dict)       # topic -> declaring scene
    code_endpoints: dict[tuple[str, str], set[tuple[str, str]]] = field(default_factory=dict)


# --------------------------------------------------------------------------- #
# Markdown table parsing
# --------------------------------------------------------------------------- #
_TABLE_ROW = re.compile(r"^\|(.+)\|\s*$")


def split_table_row(line: str) -> list[str] | None:
    """Split a markdown table row into cells, honouring inline code spans.

    A cell like ``` `uv_msgs`/`uv_hm` ``` contains a literal ``|`` if the
    author wrote two package names in one code span; a naive ``split("|")``
    shreds the row and silently drops the tail cells (this made the dependency
    gap table unreadable to the checker on the first run).
    """
    match = _TABLE_ROW.match(line)
    if not match:
        return None
    body = match.group(1)
    protected = re.sub(
        r"`[^`]*`", lambda m: m.group(0).replace("|", "\x00"), body
    )
    def _clean(cell: str) -> str:
        text = cell.replace("\x00", "|").strip()
        # Strip at most one wrapping backtick pair: a cell may legitimately
        # contain several code spans (`` `a` 与 `b` ``), and a blanket strip
        # destroyed the inner ones (that hid the dependency exemptions).
        if len(text) >= 2 and text.startswith("`") and text.endswith("`") and "`" not in text[1:-1]:
            text = text[1:-1]
        return text.strip()

    cells = [_clean(c) for c in protected.split("|")]
    if len(cells) < 2:
        return None
    # Drop the empty leading/trailing cells produced by the outer pipes.
    if cells and cells[0] == "":
        cells = cells[1:]
    if cells and cells[-1] == "":
        cells = cells[:-1]
    return cells


def parse_tables(text: str) -> list[tuple[str, list[dict[str, str]]]]:
    """Return ``(heading, rows)`` for every markdown table under a heading.

    ``rows`` are dicts keyed by the table's own header cell text (backticks
    stripped), so a column can be addressed by name instead of by index.
    """
    tables: list[tuple[str, list[dict[str, str]]]] = []
    heading = ""
    section = ""
    header: list[str] | None = None
    rows: list[dict[str, str]] = []
    in_table = False

    def flush() -> None:
        nonlocal rows, header, in_table
        if header is not None and rows:
            tables.append((heading, rows))
        rows, header, in_table = [], None, False

    for line in text.splitlines():
        if line.startswith("#"):
            flush()
            level = len(line) - len(line.lstrip("#"))
            title = line.lstrip("#").strip()
            if level <= 2:
                section = title
                heading = title
            else:
                # Keep the owning chapter in the path: launch sub-sections are
                # named after files ("sim_bringup.py") and would otherwise look
                # like node-parameter sections.
                heading = f"{section} › {title}"
            continue
        cells = split_table_row(line)
        if cells is None:
            flush()
            continue
        if header is None:
            header = cells
            in_table = True
            continue
        if in_table and all(set(c) <= set("-: ") and c for c in cells):
            continue  # separator row
        if header is not None and len(cells) == len(header):
            rows.append(dict(zip(header, cells)))
    flush()
    return tables


def _in_launch_section(heading: str) -> bool:
    """True for tables that live under the launch-arguments chapter."""
    return "launch" in heading.lower()


def doc_parameter_defaults(path: Path) -> dict[str, tuple[str, str]]:
    """``name -> (documented default, heading)`` for node-parameter tables."""
    defaults: dict[str, tuple[str, str]] = {}
    for heading, rows in parse_tables(path.read_text(encoding="utf-8")):
        if _in_launch_section(heading):
            continue
        for row in rows:
            if "参数" not in row:
                continue
            name = row["参数"].strip()
            if not re.fullmatch(r"[a-z_][a-z0-9_]*", name):
                continue
            default = next(
                (value for key, value in row.items() if "默认" in key),
                "",
            )
            defaults[name] = (default.strip(), heading)
    return defaults


def doc_parameters(path: Path) -> set[str]:
    """Node parameter names documented in ``path``.

    Only tables **outside** the launch-arguments chapter count: launch arguments
    use the same 参数 column but are declared by ``DeclareLaunchArgument``, and
    conflating the two made the first run of this checker report every
    ``enable_ai``/``scenario`` as an undocumented ROS parameter.
    """
    names: set[str] = set()
    for heading, rows in parse_tables(path.read_text(encoding="utf-8")):
        if _in_launch_section(heading):
            continue
        for row in rows:
            if "参数" in row:
                name = row["参数"].strip()
                if re.fullmatch(r"[a-z_][a-z0-9_]*", name):
                    names.add(name)
    return names


def doc_launch_arguments(path: Path) -> set[str]:
    """Launch argument names documented under the launch-arguments chapter."""
    names: set[str] = set()
    for heading, rows in parse_tables(path.read_text(encoding="utf-8")):
        if not _in_launch_section(heading):
            continue
        for row in rows:
            if "默认值" not in row and "默认 值" not in row:
                continue
            first = next(iter(row.values()), "").strip()
            name = (row.get("参数") or row.get("参数名") or first).strip()
            if re.fullmatch(r"[a-z_][a-z0-9_]*", name):
                names.add(name)
    return names


def doc_nodes(path: Path) -> set[str]:
    """Executable names documented in the node table of ``path``."""
    nodes: set[str] = set()
    for _heading, rows in parse_tables(path.read_text(encoding="utf-8")):
        for row in rows:
            if "可执行文件" in row:
                name = row["可执行文件"].strip()
                if re.fullmatch(r"[A-Za-z_][\w]*", name):
                    nodes.add(name)
    return nodes


# --------------------------------------------------------------------------- #
# Code extraction
# --------------------------------------------------------------------------- #
def iter_package_files(root: Path):
    """Yield Python files of workspace packages, skipping generated dirs."""
    for path in sorted(root.rglob("*.py")):
        if SKIP_DIR_PARTS & set(path.parts):
            continue
        yield path


def _import_map(tree: ast.AST) -> dict[str, str]:
    """symbol -> fully qualified name, for the message-type resolution."""
    mapping: dict[str, str] = {}
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom) and node.module:
            for alias in node.names:
                mapping[alias.asname or alias.name] = f"{node.module}.{alias.name}"
        elif isinstance(node, ast.Import):
            for alias in node.names:
                mapping[alias.asname or alias.name.split(".")[0]] = alias.name
    return mapping


def _topic_literals(node: ast.AST) -> list[str]:
    """Concrete topic strings for a ``create_*`` topic argument.

    f-strings are expanded with :data:`TEMPLATE_CHANNELS`; anything else that is
    not a literal is reported as ``<dynamic:...>`` so it shows up instead of
    being silently skipped.
    """
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return [node.value]
    if isinstance(node, ast.JoinedStr):
        # Rebuild the template spelling from the FormattedValue nodes: the
        # constant parts of an f-string stop *before* the placeholder, so the
        # ``{ch}`` itself has to be reconstructed.  Skipping that made every
        # f-string topic look like ``<dynamic:...>``.
        template = ""
        for value in node.values:
            if isinstance(value, ast.Constant):
                template += str(value.value)
            elif isinstance(value, ast.FormattedValue):
                template += "{" + ast.unparse(value.value) + "}"
        for placeholder, channels in TEMPLATE_CHANNELS.items():
            if placeholder in template:
                return [template.replace(placeholder, channel) for channel in channels]
        return [f"<dynamic:{template}>"]
    return [f"<dynamic:{ast.unparse(node)[:40]}>"]


def collect_code_endpoints() -> dict[tuple[str, str], set[tuple[str, str]]]:
    """``(topic, kind) -> (resolved message type, file)`` from the AST.

    Needed because a file-wide symbol search cannot tell a publisher from a
    subscriber: ``position.py`` mentions ``PoseInfo`` (it subscribes) and
    ``ObjectPositionArray`` (it publishes), so a corrupted contract row could
    still find its (wrong) type name somewhere in the file and pass.
    """
    kinds = {
        "create_publisher": "发布",
        "create_subscription": "订阅",
        "create_service": "服务",
    }
    endpoints: dict[tuple[str, str], set[tuple[str, str]]] = {}
    roots = [*SOURCE_ROOTS, *EXTRA_SOURCE_DIRS]
    for root in roots:
        for path in iter_package_files(root):
            try:
                tree = ast.parse(path.read_text(encoding="utf-8"))
            except (SyntaxError, UnicodeDecodeError):
                continue
            imports = _import_map(tree)
            for node in ast.walk(tree):
                if not isinstance(node, ast.Call):
                    continue
                fn = getattr(node.func, "attr", None) or ""
                if fn not in kinds or len(node.args) < 2:
                    continue
                # create_subscription is written as (Type, topic, ...) in some
                # nodes and (topic, Type, ...) in others; detect which is which.
                first, second = node.args[0], node.args[1]
                if isinstance(first, ast.Constant) and isinstance(first.value, str):
                    type_node, topic_node = second, first
                else:
                    type_node, topic_node = first, second
                symbol = ast.unparse(type_node)
                msg_type = imports.get(symbol, symbol)
                for topic in _topic_literals(topic_node):
                    endpoints.setdefault((topic, kinds[fn]), set()).add((msg_type, rel(path)))
    return endpoints


def _top_level(name: str) -> str:
    return name.split(".")[0]


def _package_of(path: Path) -> str:
    """Package name for a source file: the directory that owns ``setup.py``."""
    for parent in [path.parent, *path.parents]:
        if parent == REPO_ROOT or parent == REPO_ROOT.parent:
            break
        if (parent / "setup.py").exists() or (parent / "package.xml").exists():
            return parent.name
    return path.parent.name


def collect_source_facts() -> SourceFacts:
    facts = SourceFacts()
    ws_packages = {
        p.name
        for root in SOURCE_ROOTS
        for p in root.iterdir()
        if p.is_dir() and p.name != "__pycache__"
    }

    # Launch files are skipped by iter_package_files (their nodes are declared by
    # the launch tree, not by the file), but their arguments and package
    # references are exactly what the docs must match.
    launch_files = [
        path
        for root in SOURCE_ROOTS
        for path in sorted(root.glob("*/launch/*.py"))
    ]

    for root in SOURCE_ROOTS:
        for path in [*iter_package_files(root), *launch_files]:
            pkg = _package_of(path)
            try:
                tree = ast.parse(path.read_text(encoding="utf-8"))
            except (SyntaxError, UnicodeDecodeError):
                continue
            imports = _import_map(tree)

            for node in ast.walk(tree):
                mods: list[str] = []
                if isinstance(node, ast.Import):
                    mods = [a.name for a in node.names]
                elif isinstance(node, ast.ImportFrom) and node.module:
                    mods = [node.module]
                for mod in mods:
                    top = _top_level(mod)
                    if top in ws_packages:
                        facts.package_imports.setdefault(pkg, set()).add(top)

                if not isinstance(node, ast.Call):
                    continue
                # ``self.declare_parameter`` is an attribute call, while launch
                # files call the bare ``DeclareLaunchArgument`` name; support both
                # (the first version only handled the attribute form, which
                # silently produced zero launch arguments).
                fn = getattr(node.func, "attr", None) or getattr(node.func, "id", None) or ""

                if fn == "declare_parameter" and len(node.args) >= 1:
                    name_node = node.args[0]
                    if not isinstance(name_node, ast.Constant):
                        continue
                    default = "<none>"
                    if len(node.args) >= 2:
                        default = _constant_value(node.args[1])
                    for kw in node.keywords:
                        if kw.arg == "default_value":
                            default = _constant_value(kw.value)
                    facts.parameters[str(name_node.value)] = {
                        "default": default,
                        "decl": rel(path),
                    }

                if fn == "DeclareLaunchArgument" and len(node.args) >= 1:
                    if path.name in UNCHECKED_LAUNCH_FILES:
                        continue
                    name_node = node.args[0]
                    if not isinstance(name_node, ast.Constant):
                        continue
                    default = "<none>"
                    for kw in node.keywords:
                        if kw.arg == "default_value":
                            default = _constant_value(kw.value)
                    facts.launch_arguments[str(name_node.value)] = {
                        "default": default,
                        "file": rel(path),
                    }

                if fn in ("create_publisher", "create_subscription", "create_service") and len(node.args) >= 2:
                    kind = {
                        "create_publisher": "发布",
                        "create_subscription": "订阅",
                        "create_service": "服务",
                    }[fn]
                    symbol = ast.unparse(node.args[0])
                    msg_type = imports.get(symbol, symbol)
                    for topic in _topic_literals(node.args[1]):
                        facts.topics.setdefault((topic, kind), set()).add(rel(path))
                        facts.topic_types.setdefault((topic, kind), msg_type)


    facts.code_endpoints = collect_code_endpoints()

    # package='...' references in every launch file (dependency check)
    for launch in launch_files:
        text = launch.read_text(encoding="utf-8")
        refs = set(re.findall(r"package\s*=\s*['\"]([A-Za-z_][\w]*)['\"]", text))
        if refs:
            facts.launch_refs[rel(launch)] = refs

    # scene-declared publishers (Stonefish side)
    for scene in SCENE_FILES:
        if not scene.is_file():
            continue
        scene_root = ET.parse(str(scene)).getroot()
        for element in scene_root.iter("ros_publisher"):
            for key, value in element.attrib.items():
                if key in ("topic", "altitude_topic") and value.startswith("/"):
                    facts.scene_topics[value] = rel(scene)
                elif key in ("thrusters", "state", "servo", "light") and value.startswith("/"):
                    facts.scene_topics[value] = rel(scene)

    # console_scripts entry points
    for root in SOURCE_ROOTS:
        for setup_py in sorted(root.glob("*/setup.py")):
            try:
                tree = ast.parse(setup_py.read_text(encoding="utf-8"))
            except SyntaxError:
                continue
            for node in ast.walk(tree):
                if not isinstance(node, ast.Call):
                    continue
                for kw in node.keywords:
                    if kw.arg != "entry_points":
                        continue
                    for value in ast.walk(kw.value):
                        if isinstance(value, ast.Dict):
                            keys = [k.value for k in value.keys if isinstance(k, ast.Constant)]
                            if "console_scripts" not in keys:
                                continue
                            index = keys.index("console_scripts")
                            entries = value.values[index]
                            if isinstance(entries, ast.List):
                                for item in entries.elts:
                                    if isinstance(item, ast.Constant) and "=" in str(item.value):
                                        name, _, target = str(item.value).partition("=")
                                        facts.entries[name.strip()] = {
                                            "package": setup_py.parent.name,
                                            "target": target.strip(),
                                        }

    for root in SOURCE_ROOTS:
        for package_xml in sorted(root.glob("*/package.xml")):
            declared: set[str] = set()
            try:
                tree = ET.parse(str(package_xml))
            except ET.ParseError:
                continue
            for element in tree.getroot():
                tag = element.tag.split("}")[-1]
                if tag in ("depend", "exec_depend", "build_depend") and element.text:
                    declared.add(element.text.strip())
            facts.package_deps[package_xml.parent.name] = declared

    return facts


def _constant_value(node: ast.AST) -> str:
    """Best-effort literal/default rendering for a parameter default."""
    if isinstance(node, ast.Constant):
        return repr(node.value)
    if isinstance(node, ast.Name):
        return node.id
    return ast.unparse(node)


# --------------------------------------------------------------------------- #
# Checks
# --------------------------------------------------------------------------- #
def _normalise_default(text: str) -> str:
    """Best-effort literal form so ``false``/`False`/`` `false` `` compare equal."""
    # Markdown emphasises defaults in several ways (`false`, **false**, **`false`**),
    # so strip the decoration characters from both ends in one pass.
    cleaned = text.strip().strip("`* \t").strip()
    cleaned = cleaned.replace("**", "").strip()
    if cleaned.lower() in ("false", "true"):
        return cleaned.lower()
    return cleaned


def _documented_default_matches(code_default: str, documented: str) -> bool:
    """True when we can prove the two agree; unknown forms never fail the check.

    The code default is a Python literal (``False``, ``30``, ``'go2rtc'``) while
    the doc writes markdown (``false``, ``30``, ``go2rtc``), so only the cases we
    can normalise are compared; constants (``SIM_MODE``) are left alone.
    """
    code_text = code_default.strip()
    if not code_text or code_text in ("<none>",):
        return True
    try:
        literal = ast.literal_eval(code_text)
    except (ValueError, SyntaxError):
        return True  # a name like SIM_MODE: cannot compare, do not guess
    if isinstance(literal, bool):
        return _normalise_default(documented).lower() == str(literal).lower()
    if isinstance(literal, (int, float)):
        match = re.match(r"-?\d+(?:\.\d+)?", _normalise_default(documented))
        if not match:
            return True
        return float(match.group(0)) == float(literal)
    if isinstance(literal, str):
        return _normalise_default(documented).strip("'\"") == literal
    return True


def check_parameters(facts: SourceFacts) -> list[Problem]:
    problems: list[Problem] = []
    if not DOC_NODE_PARAMS.is_file():
        return [Problem("parameters", f"missing {rel(DOC_NODE_PARAMS)}")]
    documented_map = doc_parameter_defaults(DOC_NODE_PARAMS)
    for name, (documented_default, heading) in sorted(documented_map.items()):
        if name not in facts.parameters:
            continue
        code_default = facts.parameters[name]["default"]
        if not _documented_default_matches(code_default, documented_default):
            problems.append(
                Problem(
                    "parameters",
                    f"{name}: doc ({heading}) says default {documented_default!r} "
                    f"but {facts.parameters[name]['decl']} declares {code_default}",
                )
            )
    documented = doc_parameters(DOC_NODE_PARAMS)
    for name in sorted(set(facts.parameters) - documented):
        problems.append(
            Problem(
                "parameters",
                f"{name!r} is declared in {facts.parameters[name]['decl']} "
                f"but missing from {rel(DOC_NODE_PARAMS)}",
            )
        )
    for name in sorted(documented - set(facts.parameters)):
        problems.append(
            Problem(
                "parameters",
                f"{name!r} is documented in {rel(DOC_NODE_PARAMS)} but no "
                "declare_parameter call declares it",
            )
        )
    return problems


def check_launch_arguments(facts: SourceFacts) -> list[Problem]:
    problems: list[Problem] = []
    documented = doc_launch_arguments(DOC_NODE_PARAMS)
    for name in sorted(set(facts.launch_arguments) - documented):
        problems.append(
            Problem(
                "launch-args",
                f"{name!r} is declared in {facts.launch_arguments[name]['file']} "
                f"but missing from the launch section of {rel(DOC_NODE_PARAMS)}",
            )
        )
    for name in sorted(documented - set(facts.launch_arguments)):
        problems.append(
            Problem(
                "launch-args",
                f"{name!r} is documented as a launch argument but no "
                "DeclareLaunchArgument declares it",
            )
        )
    return problems


def check_nodes(facts: SourceFacts) -> list[Problem]:
    problems: list[Problem] = []
    documented = doc_nodes(DOC_NODE_PARAMS)
    for name in sorted(set(facts.entries) - documented):
        problems.append(
            Problem(
                "nodes",
                f"{name!r} ({facts.entries[name]['package']}) has a console_scripts "
                f"entry but is missing from {rel(DOC_NODE_PARAMS)}",
            )
        )
    return problems


def check_topics(facts: SourceFacts) -> list[Problem]:
    """Every contract row must be backed by the evidence file it names."""
    problems: list[Problem] = []
    if not DOC_TOPIC_CONTRACT.is_file():
        return [Problem("topics", f"missing {rel(DOC_TOPIC_CONTRACT)}")]

    rows: list[dict[str, str]] = []
    for _heading, table_rows in parse_tables(DOC_TOPIC_CONTRACT.read_text(encoding="utf-8")):
        for row in table_rows:
            if {"话题", "消息类型", "方向", "证据"} <= set(row):
                rows.append(row)
    if not rows:
        return [Problem("topics", f"no contract table rows found in {rel(DOC_TOPIC_CONTRACT)}")]

    for row in rows:
        topic = row["话题"].strip()
        problem = _check_contract_row(row, topic, facts)
        if problem:
            problems.append(problem)
    return problems


def _check_contract_row(row: dict[str, str], topic: str, facts: SourceFacts) -> Problem | None:
    evidence = row["证据"].strip()
    direction = row["方向"].strip()
    if not topic.startswith("/"):
        return Problem("topics", f"contract row has a non-topic first column: {topic!r}")

    if "<dynamic" in topic:
        return Problem("topics", f"contract lists an unresolved dynamic topic: {topic!r}")

    # Split on the first colon so paths with colons stay intact.
    for token in [t.strip() for t in evidence.split(";") if t.strip()]:
        path_text, _, kind = token.rpartition(":")
        if not path_text:
            return Problem("topics", f"{topic}: evidence {token!r} is not 'file:kind'")
        path = REPO_ROOT / path_text
        if not path.is_file():
            return Problem("topics", f"{topic}: evidence file {path_text} does not exist")
        text = path.read_text(encoding="utf-8", errors="replace")
        if kind == "ros_publisher":
            if topic in facts.scene_topics:
                continue
            # Stonefish appends /image_color to a camera <ros_publisher topic>.
            base = topic[: -len(CAMERA_TOPIC_SUFFIX)] if topic.endswith(CAMERA_TOPIC_SUFFIX) else None
            if base and base in facts.scene_topics:
                continue
            return Problem(
                "topics",
                f"{topic}: {path_text} does not declare it as <ros_publisher> "
                f"(scene declares {sorted(facts.scene_topics)})",
            )
        if kind not in ("create_publisher", "create_subscription", "create_service"):
            return Problem("topics", f"{topic}: unknown evidence kind {kind!r}")
        expected = {"create_publisher": "发布", "create_subscription": "订阅", "create_service": "服务"}[kind]
        expected_kind = expected
        if direction not in (expected, "动作"):
            return Problem(
                "topics",
                f"{topic}: row says 方向={direction} but evidence {kind} implies {expected}",
            )
        candidates = _code_topics(topic)
        hit = next((c for c in candidates if c in text), None)
        if hit is None:
            return Problem(
                "topics",
                f"{topic}: {path_text} has no {kind} for {topic} "
                f"(searched {candidates})",
            )
        declared = row["消息类型"].strip()
        endpoints = facts.code_endpoints.get((topic, expected_kind))
        if endpoints is None:
            # The topic may be built by an f-string; fall back to a literal check.
            if not any(candidate in text for candidate in _code_topics(topic)):
                return Problem(
                    "topics",
                    f"{topic}: no {kind} for it was found in the AST scan "
                    f"(checked {rel(path)})",
                )
            continue
        # A topic can be declared by several files (one row per direction); the
        # row is correct when *its* file declares the topic with the stated type.
        matching = {
            (msg_type, file)
            for msg_type, file in endpoints
            if file == path_text and _same_message_type(declared, msg_type)
        }
        if matching:
            continue
        same_file = sorted({(t, f) for t, f in endpoints if f == path_text})
        where = f" (declared as {same_file})" if same_file else f" (declared in {sorted(endpoints)})"
        return Problem(
            "topics",
            f"{topic}: contract says {declared} in {path_text} but the code{where}",
        )
    return None


def _code_topics(topic: str) -> list[str]:
    """Topic spellings that count as evidence for a contract row's topic.

    A concrete contract topic (``/perception/detection/front_left``) may be
    produced by an f-string in the code (``f'/perception/detection/{ch}'``), so
    the matching template spelling is accepted as evidence too.
    """
    candidates = [topic]
    if topic.endswith(CAMERA_TOPIC_SUFFIX):
        candidates.append(topic[: -len(CAMERA_TOPIC_SUFFIX)])
    if "/" in topic:
        prefix, _, leaf = topic.rpartition("/")
        for template in TEMPLATE_CHANNELS:
            if leaf.replace(template, "") != leaf or True:
                candidates.append(f"{prefix}/{template}")
    return list(dict.fromkeys(candidates))


def _type_symbol(msg_type: str) -> str:
    """``uv_msgs/DetectionArray`` -> ``DetectionArray``."""
    return msg_type.rsplit("/", 1)[-1].strip()


def type_candidates(msg_type: str) -> list[str]:
    """Spellings of a message type that count as evidence in a source file.

    The contract writes ``uv_msgs/DetectionArray`` while the code writes
    ``from uv_msgs.msg import DetectionArray`` and then uses the bare symbol, so
    several spellings have to be accepted.
    """
    packed = msg_type.strip()
    symbol = packed.rsplit("/", 1)[-1].strip()
    package = packed.split("/", 1)[0].strip()
    dotted = packed.replace("/", ".")
    for kind in ("msg", "srv", "action"):
        dotted = dotted.replace(f"/{kind}.", ".")
    return list(dict.fromkeys([packed, dotted, f"{package}.msg.{symbol}",
                               f"{package}.srv.{symbol}", symbol]))


def _expected_type(declared: str) -> str:
    """Normalise a contract type to ``package/msg/Symbol`` form for comparison."""
    return declared.strip()


def _same_message_type(declared: str, actual: str) -> bool:
    """True when the contract type and the code's type denote the same message."""
    declared_symbol = declared.rsplit("/", 1)[-1].strip()
    actual_symbol = actual.rsplit(".", 1)[-1].strip()
    if declared_symbol != actual_symbol:
        return False
    declared_package = declared.split("/", 1)[0].strip()
    actual_package = actual.split(".", 1)[0].strip()
    return declared_package == actual_package


def check_dependencies(facts: SourceFacts) -> list[Problem]:
    problems: list[Problem] = []
    exemptions = _dependency_exemptions()
    if not DOC_NODE_DEPENDENCY.is_file():
        return [Problem("dependencies", f"missing {rel(DOC_NODE_DEPENDENCY)}")]

    for launch, refs in sorted(facts.launch_refs.items()):
        declared = facts.package_deps.get("uv_bringup", set())
        for package in sorted(refs - declared):
            if f"uv_bringup->{package}" in exemptions:
                continue
            problems.append(
                Problem(
                    "dependencies",
                    f"{launch} references package {package!r} but uv_bringup/package.xml "
                    "does not declare it (register it in node_dependency.md section 2 to exempt)",
                )
            )

    for package, imports in sorted(facts.package_imports.items()):
        declared = facts.package_deps.get(package, set())
        for imported in sorted(imports - declared - {package}):
            if f"{package}->{imported}" in exemptions:
                continue
            if imported not in facts.package_deps:
                continue  # not a workspace package
            problems.append(
                Problem(
                    "dependencies",
                    f"{package} imports workspace package {imported!r} but does not "
                    "declare it in package.xml",
                )
            )

    for package, imports in sorted(facts.package_imports.items()):
        declared = facts.package_deps.get(package, set())
        for module in sorted(imports - declared - {package}):
            if module in facts.package_deps or module in IGNORED_IMPORTS:
                continue
            if f"{package}->{module}" in exemptions:
                continue
            if importlib.util.find_spec(module) is None:
                continue  # not installed here; do not report environment noise
            problems.append(
                Problem(
                    "dependencies",
                    f"{package} imports {module!r} which is not declared in package.xml "
                    "(register it in node_dependency.md section 2 to exempt)",
                )
            )
    return problems


def _dependency_exemptions() -> set[str]:
    """Read ``package->dependency`` pairs from the registered gaps in the doc.

    The dependency chapter keeps a table of known manifest gaps; a row is exempt
    from failing the check only when it carries the ``已知`` marker, so recording
    a gap is an explicit act rather than a silent tolerance.
    """
    if not DOC_NODE_DEPENDENCY.is_file():
        return set()
    exemptions: set[str] = set()
    workspace = set(collect_package_names())
    for _heading, rows in parse_tables(DOC_NODE_DEPENDENCY.read_text(encoding="utf-8")):
        for row in rows:
            if "已知" not in (row.get("豁免", "") + row.get("状态", "")):
                continue
            blob = " ".join(row.values())
            backticked = re.findall(r"`([^`]+)`", blob)
            packages = [b for b in backticked if re.fullmatch(r"[a-z_][\w]*", b) and b in workspace]
            if not packages:
                continue
            subject = packages[0]
            for module in re.findall(r"import\s+([\w.]+)", blob):
                exemptions.add(f"{subject}->{module.split('.')[0]}")
            for other in packages[1:]:
                # `uv_bringup` … `uv_sim` style: subject package references other
                exemptions.add(f"{subject}->{other}")
            for token in backticked:
                if re.fullmatch(r"package='[a-z_]+'", token) or re.fullmatch(r"'[a-z_]+'", token):
                    name = token.strip("package=").strip("'")
                    if name in workspace:
                        exemptions.add(f"{subject}->{name}")
    return exemptions


def collect_package_names() -> list[str]:
    return sorted(
        {
            p.name
            for root in SOURCE_ROOTS
            for p in root.iterdir()
            if p.is_dir() and p.name != "__pycache__"
        }
    )


def check_readme(facts: SourceFacts) -> list[Problem]:
    problems: list[Problem] = []
    if not DOC_README.is_file():
        return [Problem("readme", f"missing {rel(DOC_README)}")]
    text = DOC_README.read_text(encoding="utf-8")
    for path in sorted((REPO_ROOT / "docs").rglob("*.md")):
        if path.name == "README.md":
            continue
        if path.name not in text:
            problems.append(
                Problem("readme", f"{rel(path)} is not mentioned in {rel(DOC_README)}")
            )
    return problems


CHECKS = {
    "parameters": check_parameters,
    "launch-args": check_launch_arguments,
    "nodes": check_nodes,
    "topics": check_topics,
    "dependencies": check_dependencies,
    "readme": check_readme,
}


# --------------------------------------------------------------------------- #
# Entry point
# --------------------------------------------------------------------------- #
def build_inventory(facts: SourceFacts) -> dict:
    return {
        "generated_by": "scripts/check_docs_contract.py --update",
        "note": "review artifact only; the code is the source of truth",
        "parameters": facts.parameters,
        "launch_arguments": facts.launch_arguments,
        "entries": facts.entries,
        "topics": [
            {"topic": topic, "kind": kind, "type": facts.topic_types.get((topic, kind), ""),
             "files": sorted(files)}
            for (topic, kind), files in sorted(facts.topics.items())
        ],
        "scene_topics": facts.scene_topics,
        "package_deps": {k: sorted(v) for k, v in sorted(facts.package_deps.items())},
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--check", action="store_true", help="report drift (exit 1 on any)")
    parser.add_argument("--update", action="store_true", help="write the code inventory JSON")
    parser.add_argument("--quiet", action="store_true", help="only print problems")
    parser.add_argument("--only", choices=sorted(CHECKS), action="append",
                        help="run a subset of the checks (repeatable)")
    args = parser.parse_args(argv)

    facts = collect_source_facts()

    if args.update:
        INVENTORY_JSON.parent.mkdir(parents=True, exist_ok=True)
        INVENTORY_JSON.write_text(
            json.dumps(build_inventory(facts), indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
        print(f"wrote {rel(INVENTORY_JSON)}")

    if not args.check:
        if not args.update:
            parser.print_help()
        return 0

    selected = args.only or sorted(CHECKS)
    problems: list[Problem] = []
    for name in selected:
        problems.extend(CHECKS[name](facts))

    if not args.quiet:
        print(f"code facts: {len(facts.parameters)} parameters, "
              f"{len(facts.entries)} entry points, "
              f"{len(facts.topics)} topic endpoints, "
              f"{len(facts.package_deps)} packages")
    for problem in problems:
        print(f"  {problem}")
    if problems:
        print(f"\nFAIL  {len(problems)} documentation/contract problem(s)")
        return 1
    if not args.quiet:
        print(f"OK    docs match the code ({', '.join(selected)})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
