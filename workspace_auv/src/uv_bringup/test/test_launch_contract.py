"""Static contracts for the runtime-preset launch layout."""

import ast
from pathlib import Path


PACKAGE_ROOT = Path(__file__).parents[1]
LAUNCH_ROOT = PACKAGE_ROOT / "launch"
PROFILE_ROOT = PACKAGE_ROOT / "config" / "profiles"
AUV_SOURCE_ROOT = PACKAGE_ROOT.parent
REPOSITORY_ROOT = PACKAGE_ROOT.parents[2]
SIM_SOURCE_ROOT = REPOSITORY_ROOT / "workspace_sim" / "src"


def _source(name):
    return (LAUNCH_ROOT / name).read_text(encoding="utf-8")


def test_formal_mode_entries_exist():
    for name in ("sim.launch.py", "hil.launch.py", "real.launch.py"):
        assert (LAUNCH_ROOT / name).is_file()


def test_mode_entries_do_not_define_component_nodes():
    """Mode files may orchestrate, but component Nodes belong to feature packages."""
    for name in ("sim.launch.py", "hil.launch.py", "real.launch.py", "core_sim.launch.py"):
        tree = ast.parse(_source(name), filename=name)
        called_names = {
            node.func.id
            for node in ast.walk(tree)
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
        }
        assert "Node" not in called_names


def test_bringup_uses_the_invoking_terminal():
    """Terminal multiplexing is deliberately outside the ROS launch layer."""
    launch_files = list(LAUNCH_ROOT.glob("*.py"))
    launch_files.extend((SIM_SOURCE_ROOT / "stonefish_ros2" / "launch").glob("*.py"))
    forbidden = (
        "t" + "mux",
        "x" + "term",
        "use_" + "x" + "term",
        "terminal_" + "prefix",
        "process_" + "prefix",
    )
    for path in launch_files:
        source = path.read_text(encoding="utf-8").lower()
        assert not any(token in source for token in forbidden), path


def test_all_profiles_are_standard_ros_parameter_files():
    expected = {
        "uv_sim": {"sim_dev.yaml", "sim_ci.yaml", "hil_lab.yaml"},
        "uv_camera": {
            "sim_dev.yaml", "sim_ci.yaml", "hil_lab.yaml",
            "real_default.yaml", "real_safe.yaml",
        },
        "uv_hm": {"real_default.yaml", "real_safe.yaml"},
    }
    assert not list(PROFILE_ROOT.glob("*.yaml"))
    for package, names in expected.items():
        source_root = SIM_SOURCE_ROOT if package == "uv_sim" else AUV_SOURCE_ROOT
        root = source_root / package / "config" / "profiles"
        assert {path.name for path in root.glob("*.yaml")} == names
        for path in root.glob("*.yaml"):
            text = path.read_text(encoding="utf-8")
            assert "ros__parameters:" in text


def test_sim_localizer_parameters_live_in_camera_package():
    config = AUV_SOURCE_ROOT / "uv_camera" / "config" / "object_localizer_sim.yaml"
    assert config.is_file()
    assert "ros__parameters:" in config.read_text(encoding="utf-8")
