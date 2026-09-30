from pathlib import Path

from auv_protocol.topics import (
    VIZ_MEASUREMENTS,
    VIZ_ODOM,
    VIZ_ODOM_PATH,
    VIZ_PLANNED_PATH,
    VIZ_TRACKS,
)


PACKAGE_ROOT = Path(__file__).parents[1]


def test_visualization_topics_are_vehicle_scoped():
    assert VIZ_ODOM == '/auv/visualization/odom'
    assert VIZ_ODOM_PATH == '/auv/visualization/odom_path'
    assert VIZ_MEASUREMENTS == '/auv/visualization/measurements'
    assert VIZ_TRACKS == '/auv/visualization/tracks'
    assert VIZ_PLANNED_PATH == '/auv/visualization/planned_path'


def test_display_launch_remaps_canonical_tf_and_loads_packaged_config():
    source = (PACKAGE_ROOT / 'launch' / 'display.launch.py').read_text(encoding='utf-8')
    config = (PACKAGE_ROOT / 'config' / 'auv.rviz').read_text(encoding='utf-8')
    assert "('/tf', '/auv/tf')" in source
    assert "('/tf_static', '/auv/tf_static')" in source
    assert '/auv/visualization/odom' in config
    assert '/auv/visualization/measurements' in config
    assert '/auv/visualization/tracks' in config
    assert '/auv/visualization/planned_path' in config
    assert 'Fixed Frame: odom' in config


def test_ned_and_sim_time_defaults_are_documented():
    readme = (PACKAGE_ROOT / 'README.md').read_text(encoding='utf-8')
    launch = (PACKAGE_ROOT / 'launch' / 'display.launch.py').read_text(encoding='utf-8')
    assert 'X north, Y east, and Z depth/down' in readme
    assert "default_value='false'" in launch
