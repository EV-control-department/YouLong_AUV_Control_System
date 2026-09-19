"""Static guard: the SIL bridge must never consume simulator truth."""

import ast
from pathlib import Path


BRIDGE = (Path(__file__).parents[2] / 'uv_sim_bridge' / 'uv_sim_bridge' /
          'sim_bridge.py')


def test_sim_bridge_has_no_ground_truth_subscription_or_symbol():
    source = BRIDGE.read_text(encoding='utf-8')
    tree = ast.parse(source)
    names = {node.id for node in ast.walk(tree) if isinstance(node, ast.Name)}
    assert 'SIM_GT_ODOM' not in names
    assert 'Odometry' not in names
