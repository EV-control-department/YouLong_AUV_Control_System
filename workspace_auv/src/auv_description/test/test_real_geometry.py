"""Regression tests for the real vehicle geometry from calibration PDF v4."""

from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


PACKAGE_ROOT = Path(__file__).parents[1]


def _joint_origins(path):
    root = ET.parse(path).getroot()
    result = {}
    for joint in root.findall('joint'):
        child = joint.find('child')
        origin = joint.find('origin')
        if child is not None and origin is not None:
            result[child.attrib['link']] = tuple(
                float(value) for value in origin.attrib['xyz'].split())
    return result


def test_real_urdf_contains_pdf_reference_points():
    origins = _joint_origins(PACKAGE_ROOT / 'urdf' / 'auv.urdf')
    assert origins['front_camera_link'] == (0.23, 0.0, 0.076)
    assert origins['downward_camera_link'] == (-0.13, 0.0, 0.0645)
    assert origins['imu_link'] == (0.03, 0.0, 0.0)
    assert origins['usbl_link'] == (0.16, 0.0, 0.0235)
    assert origins['usbl_transducer_a1'] == (0.03, -0.10, 0.105)
    assert origins['usbl_transducer_a2'] == (0.03, 0.10, 0.105)
    assert origins['usbl_transducer_a3'] == (0.23, 0.10, 0.105)
    assert 'dvl_link' not in origins


def test_real_component_registry_contains_pdf_thruster_contract():
    path = PACKAGE_ROOT / 'config' / 'real_components.yaml'
    with path.open('r', encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    assert config['coordinate_frame']['name'] == 'body_frd'
    assert config['reference_points']['dvl']['position_body'] is None
    assert config['reference_points']['dvl']['orientation_status'] == (
        'not_defined_in_pdf')
    assert config['thrusters']['order'] == ['M0', 'M1', 'M2', 'M3', 'M4', 'M5']
    assert config['thrusters']['position_body']['M4'] == [-0.246, -0.132, 0.0]
    assert config['thrusters']['direction_body']['M2'] == [0.0, 0.0, -1.0]
    assert config['thrusters']['allocation_matrix'] == [
        [0.819152, 0.819152, 0.0, 0.0, -0.819152, -0.819152],
        [0.573576, -0.573576, 0.0, 0.0, 0.573576, -0.573576],
        [0.0, 0.0, -1.0, -1.0, 0.0, 0.0],
        [0.215387, -0.215387, 0.0, 0.0, -0.249228, 0.249228],
    ]
