"""Regression tests for PDF v5 geometry and the current disc-claw calibration."""

import math
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


def _joint_origins_and_rpy(path):
    root = ET.parse(path).getroot()
    result = {}
    for joint in root.findall('joint'):
        child = joint.find('child')
        origin = joint.find('origin')
        if child is None or origin is None:
            continue
        result[child.attrib['link']] = {
            'xyz': tuple(float(value) for value in origin.attrib['xyz'].split()),
            'rpy': tuple(float(value) for value in origin.attrib.get(
                'rpy', '0 0 0').split()),
        }
    return result


def _rpy_matrix(rpy):
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def _assert_matrix_close(actual, expected, tolerance=1e-5):
    for actual_row, expected_row in zip(actual, expected):
        for actual_value, expected_value in zip(actual_row, expected_row):
            assert abs(actual_value - expected_value) <= tolerance


def test_real_urdf_contains_reference_points_and_disc_claw_calibration():
    origins = _joint_origins(PACKAGE_ROOT / 'urdf' / 'auv.urdf')
    assert origins['front_camera_link'] == (0.23, 0.0, 0.076)
    assert origins['downward_camera_link'] == (-0.13, 0.0, 0.0645)
    assert origins['imu_link'] == (0.03, 0.0, 0.0)
    assert origins['usbl_link'] == (0.16, 0.0, 0.0235)
    assert origins['usbl_transducer_a1'] == (0.03, -0.10, 0.105)
    assert origins['usbl_transducer_a2'] == (0.03, 0.10, 0.105)
    assert origins['usbl_transducer_a3'] == (0.23, 0.10, 0.105)
    assert origins['usbl_array_geometry_center'] == (0.13, 0.0, 0.105)
    assert origins['disc_claw_link'] == (-0.38, 0.0, 0.29)
    assert math.isclose(origins['downward_camera_link'][0]-origins['disc_claw_link'][0],
                        0.25, abs_tol=1e-12)
    assert origins['hairpin_claw_link'] == (0.08, 0.0, 0.13)
    assert 'dvl_link' not in origins


def test_real_camera_and_gripper_rotations_match_pdf_v5():
    transforms = _joint_origins_and_rpy(PACKAGE_ROOT / 'urdf' / 'auv.urdf')
    front_rotation = (
        (0.0, 0.0, 1.0),
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
    )
    down_rotation = (
        (0.0, -1.0, 0.0),
        (1.0, 0.0, 0.0),
        (0.0, 0.0, 1.0),
    )
    gripper_rotation = (
        (0.0, 0.0, 1.0),
        (0.0, -1.0, 0.0),
        (1.0, 0.0, 0.0),
    )
    for frame in ('front_left_camera_optical_frame',
                  'front_right_camera_optical_frame'):
        _assert_matrix_close(_rpy_matrix(transforms[frame]['rpy']),
                             front_rotation)
    for frame in ('downward_left_camera_optical_frame',
                  'downward_right_camera_optical_frame'):
        _assert_matrix_close(_rpy_matrix(transforms[frame]['rpy']),
                             down_rotation)
    for frame in ('disc_claw_link', 'hairpin_claw_link'):
        _assert_matrix_close(_rpy_matrix(transforms[frame]['rpy']),
                             gripper_rotation)


def test_real_component_registry_contains_pdf_thruster_contract():
    path = PACKAGE_ROOT / 'config' / 'real_components.yaml'
    with path.open('r', encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    assert config['source'].endswith('第五版.pdf')
    assert config['coordinate_frame']['name'] == 'body_frd'
    assert config['reference_points']['dvl']['position_body'] is None
    assert config['reference_points']['dvl']['orientation_status'] == (
        'not_defined_in_pdf')
    assert config['reference_points']['ins']['orientation_status'] == (
        'not_defined')
    assert config['grippers']['G1_disc_claw']['position_body'] == [
        -0.380, 0.000, 0.290]
    assert config['grippers']['G2_hairpin_claw']['position_body'] == [
        0.080, 0.000, 0.130]
    assert config['thrusters']['order'] == ['M0', 'M1', 'M2', 'M3', 'M4', 'M5']
    assert config['thrusters']['position_body']['M4'] == [-0.246, -0.132, 0.0]
    assert config['thrusters']['direction_body']['M2'] == [0.0, 0.0, -1.0]
    assert config['thrusters']['allocation_matrix'] == [
        [0.819152, 0.819152, 0.0, 0.0, -0.819152, -0.819152],
        [0.573576, -0.573576, 0.0, 0.0, 0.573576, -0.573576],
        [0.0, 0.0, -1.0, -1.0, 0.0, 0.0],
        [0.215387, -0.215387, 0.0, 0.0, -0.249228, 0.249228],
    ]
