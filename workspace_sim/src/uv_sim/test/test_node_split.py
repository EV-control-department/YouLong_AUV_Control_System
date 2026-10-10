"""T5: the split must be complete, one responsibility per node, contract intact.

Three things this file protects, none of which a "does it import?" check would
catch:

* **A5-1 the node set**: exactly the expected node executables (T5's seven plus
  ``laser_sim``, added with the line laser), no ``sim_bridge``
  monolith, and no hand-written ``libexec`` wrapper shadowing a console script
  (``data_files`` copies win, so such a wrapper silently *becomes* the entry
  point -- that is how the package ended up with two entry points for one node).
* **A5-2 the topic contract did not move**: the topic/type/direction inventory is
  compared against ``test/data/topics_before_split.json``, which was captured from
  the AST of the pre-split tree.  Owners are expected to change; topics, directions
  and message types are not.  The only allowed additions are the ``/sim/sensor/*``
  namespace, listed explicitly below.
* **I2 one publisher per ``/zit6/state/*``**: checked at runtime by building the
  nodes in each mode and asserting that the *other* mode's publishers do not exist.
"""

from __future__ import annotations

import ast
import importlib.util
import sys
from pathlib import Path

import pytest
import rclpy
from rclpy.parameter import Parameter

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PACKAGE_ROOT.parents[2]
SETUP_PY = PACKAGE_ROOT / "setup.py"
LAUNCH_FILE = PACKAGE_ROOT / "launch" / "bridge_launch.py"
BEFORE_SPLIT = Path(__file__).resolve().parent / "data" / "topics_before_split.json"
CHECKER = REPO_ROOT / "scripts" / "check_docs_contract.py"

#: A5-1: the node set.  T5's seven, plus ``laser_sim`` (the line laser's
#: ``LaserScan`` -> stripe-points adapter, added after T5).
EXPECTED_NODES = {
    "odom_sim", "imu_sim", "dvl_sim", "pressure_sim",
    "nav_sim", "mcu_sim", "camera_relay", "laser_sim",
}

#: A5-2: the only endpoints the split may add -- the owned sensor namespace.
EXPECTED_ADDED_ENDPOINTS = {    ("/sim/sensor/odom", "发布", "nav_msgs.msg.Odometry"),
    ("/sim/sensor/odom", "订阅", "nav_msgs.msg.Odometry"),
    ("/sim/sensor/imu", "发布", "sensor_msgs.msg.Imu"),
    ("/sim/sensor/imu", "订阅", "sensor_msgs.msg.Imu"),
    ("/sim/sensor/dvl", "发布", "uv_msgs.msg.Dvl"),
    ("/sim/sensor/pressure", "发布", "sensor_msgs.msg.FluidPressure"),
    # The line laser (added after T5): one engine topic in, one owned topic out.
    ("/auv/laser_scan", "订阅", "sensor_msgs.msg.LaserScan"),
    ("/sim/sensor/laser_points", "发布", "sensor_msgs.msg.PointCloud2"),
}

#: The only launch arguments of ``bridge_launch.py`` that switch modes.  A second
#: mode knob is exactly the failure I2 guards against, so this set is pinned.
MODE_LAUNCH_ARGUMENTS = {"hil_mode", "pid_config_file"}

#: Non-mode tuning knobs ``bridge_launch.py`` is allowed to declare.  Adding one
#: here (rather than only in the launch file) keeps it a deliberate edit.
NON_MODE_LAUNCH_ARGUMENTS = {"laser_attenuation_c"}

#: I2: which node owns which state topic, per mode.
STATE_OWNERS = {    "state/pos": "nav_sim",
    "state/vel": "nav_sim",
    "state/status": "mcu_sim",
    "state/thr": "mcu_sim",
    "state/zithbt": "mcu_sim",
}

#: leaf -> attribute the publisher is stored on (``zithbt`` is abbreviated to ``hbt``).
STATE_PUBLISHER_ATTRIBUTES = {
    "pos": "zit6_pos_pub",
    "vel": "zit6_vel_pub",
    "status": "zit6_status_pub",
    "thr": "zit6_thr_pub",
    "zithbt": "zit6_hbt_pub",
}


@pytest.fixture(scope="module")
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


# --------------------------------------------------------------------------- #
# A5-1: the node set
# --------------------------------------------------------------------------- #
def _setup_entry_points() -> dict[str, str]:
    """``name -> module`` from setup.py's console_scripts."""
    tree = ast.parse(SETUP_PY.read_text(encoding="utf-8"))
    entries: dict[str, str] = {}
    for node in ast.walk(tree):
        if not isinstance(node, ast.keyword) or node.arg != "entry_points":
            continue
        for value in ast.walk(node.value):
            if not isinstance(value, ast.Dict):
                continue
            keys = [key.value for key in value.keys if isinstance(key, ast.Constant)]
            if "console_scripts" not in keys:
                continue
            scripts = value.values[keys.index("console_scripts")]
            if isinstance(scripts, ast.List):
                for item in scripts.elts:
                    if isinstance(item, ast.Constant) and "=" in str(item.value):
                        name, _, target = str(item.value).partition("=")
                        entries[name.strip()] = target.strip().partition(":")[0]
    return entries


def test_setup_declares_exactly_the_expected_nodes() -> None:
    names = set(_setup_entry_points())
    assert names == EXPECTED_NODES, (
        f"uv_sim must expose exactly {sorted(EXPECTED_NODES)}; got {sorted(names)}"
    )


def test_each_node_lives_in_its_own_module() -> None:
    entries = _setup_entry_points()
    assert set(entries.values()) == {
        f"uv_sim.{name}" for name in EXPECTED_NODES
    }, "each node must have its own module (one responsibility per file)"


def test_the_monolith_is_gone() -> None:
    assert not (PACKAGE_ROOT / "uv_sim" / "sim_bridge.py").exists(), (
        "sim_bridge.py is the pre-T5 monolith; its responsibilities now live in "
        "mcu_sim.py / nav_sim.py / camera_relay.py (docs/roadmap.md T5)"
    )


def test_no_libexec_wrapper_shadows_a_node() -> None:
    """``data_files`` copies beat generated console scripts, so wrappers must not exist."""
    wrappers = {
        path.stem for path in PACKAGE_ROOT.rglob("*")
        if path.is_file() and "libexec" in path.parts
    }
    assert not (wrappers & EXPECTED_NODES), (
        f"hand-written wrappers shadow generated console scripts: "
        f"{sorted(wrappers & EXPECTED_NODES)}"
    )


def test_console_scripts_are_installed_where_ros2_run_looks() -> None:
    """``setup.cfg`` must redirect scripts to ``lib/uv_sim`` -- found the hard way.

    Without this file ament_python installs the console scripts into
    ``install/uv_sim/bin``, and both ``ros2 pkg executables uv_sim`` and a
    ``Node(package='uv_sim', executable=...)`` launch action look in
    ``lib/uv_sim``: launching the SIL stack would fail with "executable not found".
    Before T5 nobody noticed, because the hand-written ``libexec`` wrapper was
    installed to exactly that directory and *was* the executable -- the
    ``console_scripts`` entry it shadowed pointed at a script nobody could reach.
    """
    setup_cfg = PACKAGE_ROOT / "setup.cfg"
    assert setup_cfg.is_file(), (
        "uv_sim/setup.cfg is missing: ament_python needs it so that console scripts "
        "land in lib/uv_sim (where ros2 run and launch look)"
    )
    text = setup_cfg.read_text(encoding="utf-8")
    assert "script_dir=$base/lib/uv_sim" in text, text
    assert "install_scripts=$base/lib/uv_sim" in text, text


# --------------------------------------------------------------------------- #
# A5-2: topics, directions and types are unchanged
# --------------------------------------------------------------------------- #
def _current_endpoints() -> set[tuple[str, str, str]]:
    """Recompute the (topic, direction, type) inventory from the current tree."""
    spec = importlib.util.spec_from_file_location("docs_contract_checker_split", CHECKER)
    assert spec is not None and spec.loader is not None
    checker = importlib.util.module_from_spec(spec)
    sys.modules["docs_contract_checker_split"] = checker
    spec.loader.exec_module(checker)

    facts = checker.collect_source_facts()
    endpoints = set()
    for (topic, direction), _files in facts.topics.items():
        if topic.startswith("<dynamic"):
            continue
        endpoints.add((topic, direction, facts.topic_types.get((topic, direction), "")))
    return endpoints


def _before_split_endpoints() -> set[tuple[str, str, str]]:
    import json

    payload = json.loads(BEFORE_SPLIT.read_text(encoding="utf-8"))
    return {(entry["topic"], entry["direction"], entry["type"])
            for entry in payload["endpoints"]}


def test_topic_contract_only_gained_the_sensor_namespace() -> None:
    before = _before_split_endpoints()
    after = _current_endpoints()

    removed = before - after
    assert not removed, (
        "the split removed or changed these topic endpoints -- I1 forbids it "
        f"(owners may move, topics/types may not): {sorted(removed)}"
    )
    added = after - before
    assert added == EXPECTED_ADDED_ENDPOINTS, (
        "the split added unexpected endpoints; update the contract *and* this list "
        f"deliberately. added={sorted(added)}"
    )


def test_before_split_fixture_is_not_empty() -> None:
    """A vacuous diff must not pass for the right reason."""
    assert len(_before_split_endpoints()) >= 60


# --------------------------------------------------------------------------- #
# I2: one publisher per /zit6/state/* topic
# --------------------------------------------------------------------------- #
def test_sil_and_hil_publishers_are_disjoint(ros_context) -> None:
    from uv_sim.mcu_sim import McuSimNode
    from uv_sim.nav_sim import NavSimNode

    sil_nav = NavSimNode(node_name="nav_sim_i2_sil")
    sil_nav_publishers = _state_publishers(sil_nav)
    assert sil_nav_publishers == {"pos", "vel"}
    assert not hasattr(sil_nav, "sim_nav_pub")
    sil_nav.destroy_node()

    hil_nav = NavSimNode(
        node_name="nav_sim_i2_hil",
        parameter_overrides=[Parameter("publish_sim_nav", value=True),
                             Parameter("publish_state", value=False)],
    )
    assert _state_publishers(hil_nav) == set()
    assert hasattr(hil_nav, "sim_nav_pub")
    hil_nav.destroy_node()

    sil_mcu = McuSimNode(node_name="mcu_sim_i2_sil")
    assert _state_publishers(sil_mcu) == {"status", "thr", "zithbt"}
    sil_mcu.destroy_node()

    hil_mcu = McuSimNode(
        node_name="mcu_sim_i2_hil",
        parameter_overrides=[Parameter("hil_mode", value=True)],
    )
    assert _state_publishers(hil_mcu) == set(), (
        "in HIL the real MCU owns /zit6/state/*; mcu_sim must not publish any of it"
    )
    hil_mcu.destroy_node()


def _state_publishers(node) -> set[str]:
    """Which ``/zit6/state/*`` topics this node actually created a publisher for."""
    return {
        leaf for leaf, attribute in STATE_PUBLISHER_ATTRIBUTES.items()
        if hasattr(node, attribute)
    }


def test_owner_map_matches_the_state_publishers() -> None:
    """Cross-check ``_state_publishers`` against an AST scan (no silent rename)."""
    owners: dict[str, set[str]] = {}
    for path in sorted((PACKAGE_ROOT / "uv_sim").glob("*.py")):
        tree = ast.parse(path.read_text(encoding="utf-8"))
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            if getattr(node.func, "attr", None) != "create_publisher" or len(node.args) < 2:
                continue
            topic = node.args[1]
            if isinstance(topic, ast.Constant) and str(topic.value).startswith("/zit6/state/"):
                owners.setdefault(str(topic.value).split("/")[-1], set()).add(path.stem)

    assert owners == {topic.split("/")[-1]: {owner}
                      for topic, owner in STATE_OWNERS.items()}, (
        f"unexpected /zit6/state/* publisher ownership: {owners}"
    )


# --------------------------------------------------------------------------- #
# The launch composition is the mode switch
# --------------------------------------------------------------------------- #
def _launch_nodes() -> list[tuple[str, bool, set[str]]]:
    """``(executable, has_condition, parameter names)`` for each Node in the launch."""
    tree = ast.parse(LAUNCH_FILE.read_text(encoding="utf-8"))
    nodes: list[tuple[str, bool, set[str]]] = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if getattr(node.func, "id", None) != "Node":
            continue
        executable = ""
        has_condition = False
        parameters: set[str] = set()
        for keyword in node.keywords:
            if keyword.arg == "executable" and isinstance(keyword.value, ast.Constant):
                executable = str(keyword.value.value)
            elif keyword.arg == "condition":
                has_condition = True
            elif keyword.arg == "parameters":
                for element in ast.walk(keyword.value):
                    if isinstance(element, ast.Dict):
                        parameters |= {
                            key.value for key in element.keys
                            if isinstance(key, ast.Constant)
                        }
        nodes.append((executable, has_condition, parameters))
    return nodes


def test_bridge_launch_composes_all_seven_nodes() -> None:
    nodes = _launch_nodes()
    assert {executable for executable, _c, _p in nodes} == EXPECTED_NODES


def test_only_the_unused_relays_are_condition_gated() -> None:
    """In HIL the MCU subscribes /auv/* itself, so dvl/pressure relays are pointless."""
    gated = {executable for executable, condition, _p in _launch_nodes() if condition}
    assert gated == {"dvl_sim", "pressure_sim"}, (
        "the SIL/HIL switch must be expressed by gating the two relays nothing "
        f"consumes in HIL; got {sorted(gated)}"
    )


def test_nav_mode_is_derived_from_hil_mode_in_one_place() -> None:
    """Two launch knobs that must agree is the failure I2 is about."""
    for executable, _condition, parameters in _launch_nodes():
        if executable == "nav_sim":
            assert parameters == {"publish_state", "publish_sim_nav"}, parameters
        elif executable == "mcu_sim":
            assert parameters == {"hil_mode", "pid_config_file"}, parameters


def test_bridge_launch_declares_only_the_two_mode_knobs() -> None:
    """``hil_mode`` is the only SIL/HIL switch; extra knobs must be non-mode ones.

    A second *mode* knob is what invariant I2 is about ("two switches that must
    agree"), so the exact set is pinned here.  Non-mode tuning knobs are allowed
    but must be listed in :data:`NON_MODE_LAUNCH_ARGUMENTS`, so adding one is a
    deliberate edit rather than a silent drift.
    """
    tree = ast.parse(LAUNCH_FILE.read_text(encoding="utf-8"))
    declared: set[str] = set()
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or len(node.args) < 1:
            continue
        if getattr(node.func, "id", None) != "DeclareLaunchArgument":
            continue
        if isinstance(node.args[0], ast.Constant):
            declared.add(str(node.args[0].value))
    assert declared & MODE_LAUNCH_ARGUMENTS == MODE_LAUNCH_ARGUMENTS, declared
    assert declared - MODE_LAUNCH_ARGUMENTS == NON_MODE_LAUNCH_ARGUMENTS, (
        "only these non-mode launch arguments may be added to bridge_launch.py; "
        f"got {sorted(declared - MODE_LAUNCH_ARGUMENTS)}"
    )
