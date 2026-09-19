from pathlib import Path


def test_sensor_adapter_does_not_reference_ground_truth():
    source = (Path(__file__).parents[1] / 'uv_sim_bridge' /
              'sensor_adapter.py').read_text(encoding='utf-8')
    assert 'GROUND_TRUTH' not in source.upper()
    assert '/auv/' not in source or 'auv_protocol.topics' in source
