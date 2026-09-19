from pathlib import Path


SCENARIO_ROOT = (
    Path(__file__).parents[2] / 'uv_sim_assets' / 'worlds'
)


def test_simulated_thruster_state_does_not_impersonate_hardware_state():
    scenarios = list(SCENARIO_ROOT.rglob('*.scn'))
    assert scenarios
    for path in scenarios:
        source = path.read_text(encoding='utf-8')
        assert 'thrusters="/auv/hardware/zit6/state/thruster"' not in source
