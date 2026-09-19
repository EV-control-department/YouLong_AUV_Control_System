from pathlib import Path


def test_planning_launch_uses_vehicle_scoped_tf_topics():
    launch_file = Path(__file__).parents[1] / 'launch' / 'planning_launch.py'
    source = launch_file.read_text(encoding='utf-8')
    assert "('/tf', '/auv/tf')" in source
    assert "('/tf_static', '/auv/tf_static')" in source
