"""Static checks for the simulation-only robot description boundary."""

from pathlib import Path
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).parents[1]


def test_sim_description_is_valid_urdf_and_contains_sim_dvl_mount():
    root = ET.parse(PACKAGE_ROOT / 'urdf' / 'auv_sim.urdf').getroot()
    assert root.tag == 'robot'
    links = {link.attrib['name'] for link in root.findall('link')}
    assert {'base_link', 'dvl_link', 'front_left_camera_optical_frame'} <= links


def test_sim_description_launch_remaps_both_tf_topics():
    source = (PACKAGE_ROOT / 'launch' / 'description.launch.py').read_text(
        encoding='utf-8')
    assert "('/tf', '/auv/tf')" in source
    assert "('/tf_static', '/auv/tf_static')" in source
