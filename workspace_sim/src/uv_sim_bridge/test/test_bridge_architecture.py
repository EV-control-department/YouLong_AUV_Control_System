"""Static contracts for simulation adapter ownership."""

import ast
from pathlib import Path


PACKAGE_ROOT = Path(__file__).parents[1]
BRIDGE = PACKAGE_ROOT / 'uv_sim_bridge' / 'sim_bridge.py'
PROFILE_ROOT = PACKAGE_ROOT / 'config' / 'profiles'


def test_runtime_bridge_owns_the_control_entrypoint():
    source = (PACKAGE_ROOT / 'launch' / 'bridge.launch.py').read_text(
        encoding='utf-8')
    assert "package='uv_sim_bridge'" in source
    assert "executable='sim_bridge'" in source


def test_runtime_bridge_does_not_import_legacy_uv_sim():
    tree = ast.parse(BRIDGE.read_text(encoding='utf-8'))
    imports = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imports.extend(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            imports.append(node.module or '')
    assert not any(name == 'uv_sim' or name.startswith('uv_sim.')
                   for name in imports)


def test_runtime_bridge_has_no_ground_truth_input():
    source = BRIDGE.read_text(encoding='utf-8')
    assert 'SIM_GT_ODOM' not in source
    assert 'nav_msgs.msg' not in source


def test_sim_profiles_are_installed_at_the_bridge_boundary():
    assert {path.name for path in PROFILE_ROOT.glob('*.yaml')} == {
        'sim_dev.yaml', 'sim_ci.yaml', 'hil_lab.yaml'}
    for path in PROFILE_ROOT.glob('*.yaml'):
        assert 'ros__parameters:' in path.read_text(encoding='utf-8')


def test_control_performance_has_a_dedicated_topic():
    source = BRIDGE.read_text(encoding='utf-8')
    assert 'SIM_CONTROL_PERFORMANCE' in source
    assert 'SIM_PERFORMANCE' not in source
