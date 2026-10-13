"""几何层测试：NED/FRD 约定、SE(3)、位姿插值。

这些测试的价值全在**符号**上：组合顺序写反、yaw 正负写反、四元数分支选错，
三者都能让整张地图镜像或旋转，而图看起来仍然"像一张图"。所以下面每个约定
都用**混合姿态**或**已知数值**钉死，不用纯 yaw（纯 yaw 下错误版本也会通过）。

约定权威：``docs/frames/frame_convention.md``。
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from uv_mapping.geometry import (
    interpolate_poses,
    invert_se3,
    matrix_to_quat,
    nearest_delta,
    nearest_index,
    quat_slerp,
    quat_to_matrix,
    rpy_to_matrix,
    se3,
    transform_points,
)

D = math.radians


def test_identity_rpy_is_identity():
    assert np.allclose(rpy_to_matrix(0.0, 0.0, 0.0), np.eye(3))


def test_yaw_is_clockwise_positive_about_down_z():
    """NED 下 yaw 顺时针为正：艇头朝 +x 时 yaw=+90° 应指向 +y（东）。

    写反的话整张地图会绕 z 镜像，而散点图依然"像"一张图。
    """
    R = rpy_to_matrix(0.0, 0.0, D(90.0))
    assert np.allclose(R @ np.array([1.0, 0.0, 0.0]), [0.0, 1.0, 0.0], atol=1e-12)
    assert np.allclose(R @ np.array([0.0, 1.0, 0.0]), [-1.0, 0.0, 0.0], atol=1e-12)


def test_negative_yaw_goes_the_other_way():
    R = rpy_to_matrix(0.0, 0.0, D(-90.0))
    assert np.allclose(R @ np.array([1.0, 0.0, 0.0]), [0.0, -1.0, 0.0], atol=1e-12)


def test_pitch_positive_noses_down():
    """NED：pitch 正 = 低头（+z 是下）。"""
    R = rpy_to_matrix(0.0, D(30.0), 0.0)
    assert np.allclose(R @ np.array([1.0, 0.0, 0.0]),
                       [math.cos(D(30.0)), 0.0, -math.sin(D(30.0))], atol=1e-12)


def test_composition_is_rz_ry_rx_not_the_reverse():
    """混合姿态下 Rz·Ry·Rx 与 Rx·Ry·Rz 不同 —— 这正是本测试存在的理由。"""
    roll, pitch, yaw = D(20.0), D(-35.0), D(70.0)

    def rx(a):
        return np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)],
                         [0, math.sin(a), math.cos(a)]])

    def ry(a):
        return np.array([[math.cos(a), 0, math.sin(a)], [0, 1, 0],
                         [-math.sin(a), 0, math.cos(a)]])

    def rz(a):
        return np.array([[math.cos(a), -math.sin(a), 0],
                         [math.sin(a), math.cos(a), 0], [0, 0, 1]])

    expected = rz(yaw) @ ry(pitch) @ rx(roll)
    assert np.allclose(rpy_to_matrix(roll, pitch, yaw), expected, atol=1e-12)
    assert not np.allclose(rpy_to_matrix(roll, pitch, yaw),
                           rx(roll) @ ry(pitch) @ rz(yaw), atol=1e-4)


@pytest.mark.parametrize('angles', [
    (0.0, 0.0, 0.0),
    (0.0, 0.0, math.pi),            # 180°：朴素 trace 公式会除以 ~0
    (math.pi, 0.0, 0.0),
    (0.0, math.pi, 0.0),
    (D(20.0), D(-35.0), D(70.0)),
    (D(-170.0), D(40.0), D(-95.0)),
])
def test_quat_round_trip(angles):
    R = rpy_to_matrix(*angles)
    assert np.allclose(quat_to_matrix(matrix_to_quat(R)), R, atol=1e-9)


def test_quat_round_trip_survives_non_unit_input():
    R = rpy_to_matrix(0.0, 0.0, D(30.0))
    q = matrix_to_quat(R) * 7.0
    assert np.allclose(quat_to_matrix(q), R, atol=1e-9)


def test_slerp_midpoint_is_half_the_rotation():
    q0 = matrix_to_quat(np.eye(3))
    q1 = matrix_to_quat(rpy_to_matrix(0.0, 0.0, D(90.0)))
    half = quat_to_matrix(quat_slerp(q0, q1, 0.5))
    assert np.allclose(half, rpy_to_matrix(0.0, 0.0, D(45.0)), atol=1e-9)


def test_slerp_takes_the_short_way_when_the_sign_is_flipped():
    """q 与 -q 是同一个旋转；不处理符号会绕远路 270°。"""
    q0 = matrix_to_quat(np.eye(3))
    q1 = -matrix_to_quat(rpy_to_matrix(0.0, 0.0, D(10.0)))
    half = quat_to_matrix(quat_slerp(q0, q1, 0.5))
    assert np.allclose(half, rpy_to_matrix(0.0, 0.0, D(5.0)), atol=1e-9)


def test_se3_and_inverse():
    T = se3(rpy_to_matrix(0.1, -0.2, 0.3), [1.0, 2.0, 3.0])
    assert np.allclose(T @ invert_se3(T), np.eye(4), atol=1e-12)


def test_transform_points_matches_manual_composition():
    T = se3(rpy_to_matrix(0.0, 0.0, D(90.0)), [1.0, 0.0, 0.0])
    out = transform_points(T, np.array([[1.0, 0.0, 0.0]]))
    assert np.allclose(out, [[1.0, 1.0, 0.0]], atol=1e-12)


def test_transform_points_empty():
    out = transform_points(np.eye(4), np.zeros((0, 3)))
    assert out.shape == (0, 3)


def test_nearest_index_and_delta():
    stamps = np.array([0.0, 1.0, 2.0, 3.0])
    query = np.array([0.4, 0.6, 2.9, 10.0])
    assert list(nearest_index(stamps, query)) == [0, 1, 3, 3]
    assert np.allclose(nearest_delta(stamps, query), [0.4, 0.4, 0.1, 7.0])


def test_interpolate_poses_linear_translation_is_exact():
    """匀速直线运动：插值位姿必须与服务端点连线严格重合。"""
    stamps = np.array([0.0, 1.0])
    poses = np.array([se3(np.eye(3), [0.0, 0.0, 0.0]),
                      se3(np.eye(3), [2.0, 0.0, 0.0])])
    out = interpolate_poses(stamps, poses, np.array([0.25, 0.5, 1.0]))
    assert np.allclose(out[:, 0, 3], [0.5, 1.0, 2.0], atol=1e-12)


def test_interpolate_poses_rotates_as_well_as_translates():
    stamps = np.array([0.0, 1.0])
    poses = np.array([se3(np.eye(3), [0.0, 0.0, 0.0]),
                      se3(rpy_to_matrix(0.0, 0.0, D(90.0)), [1.0, 0.0, 0.0])])
    out = interpolate_poses(stamps, poses, np.array([0.5]))[0]
    assert np.allclose(out[:3, :3], rpy_to_matrix(0.0, 0.0, D(45.0)), atol=1e-9)
    assert np.allclose(out[:3, 3], [0.5, 0.0, 0.0], atol=1e-12)


def test_interpolate_poses_clamps_outside_the_table():
    stamps = np.array([0.0, 1.0])
    poses = np.array([se3(np.eye(3), [0.0, 0.0, 0.0]),
                      se3(np.eye(3), [1.0, 0.0, 0.0])])
    out = interpolate_poses(stamps, poses, np.array([-5.0, 9.0]))
    assert np.allclose(out[:, 0, 3], [0.0, 1.0], atol=1e-12)


def test_interpolate_poses_single_sample_repeats():
    out = interpolate_poses(np.array([3.0]), np.array([se3(np.eye(3), [1.0, 2.0, 3.0])]),
                            np.array([0.0, 5.0]))
    assert out.shape == (2, 4, 4)
    assert np.allclose(out[:, :3, 3], [[1.0, 2.0, 3.0]] * 2)


def test_interpolate_poses_rejects_an_empty_table():
    with pytest.raises(ValueError):
        interpolate_poses(np.zeros(0), np.zeros((0, 4, 4)), np.array([1.0]))
