"""PointCloud2 解析测试：跨步、空云、损坏消息。

`point_step` 与实际字段范围不一致是最容易写错的一处（引擎的 PCL 写出器会补齐到
16/32 字节，``create_cloud_xyz32`` 恰好是 12），所以填充与不填充两种都要测。
"""

from __future__ import annotations

import numpy as np
import pytest
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from uv_mapping.cloud import decode_xyz, field_offsets, finite_mask


def _header() -> Header:
    header = Header()
    header.stamp.sec = 7
    header.frame_id = 'line_laser'
    return header


def test_decodes_a_cloud_built_by_the_standard_helper():
    points = [[1.0, 2.0, 3.0], [-0.5, 0.25, 12.0], [0.0, 0.0, 0.0]]
    msg = point_cloud2.create_cloud_xyz32(_header(), points)
    out = decode_xyz(msg)
    assert out.shape == (3, 3)
    assert np.allclose(out, points, atol=1e-6)


def test_round_trip_of_a_full_stripe_is_exact_in_float32():
    rng = np.random.default_rng(20261013)
    points = np.column_stack([
        rng.uniform(0.3, 15.0, 960),
        rng.uniform(-6.0, 6.0, 960),
        np.zeros(960),
    ])
    msg = point_cloud2.create_cloud_xyz32(_header(), points.tolist())
    out = decode_xyz(msg)
    assert out.shape == (960, 3)
    assert np.allclose(out, points, atol=1e-5)


def test_respects_point_step_padding():
    """手工造一条 point_step=32 的云（x/y/z 在 0/4/8，其余是填充）。

    用 ``np.frombuffer(...).reshape(-1, 3)`` 解析这种消息会读出垃圾 —— 这个测试
    就是防止有人把它"简化"回去。
    """
    msg = PointCloud2()
    msg.header = _header()
    msg.height = 1
    msg.width = 2
    msg.is_dense = True
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.point_step = 32
    msg.row_step = 64
    msg.data = (np.array([1.0, 2.0, 3.0, 0.5], dtype='<f4').tobytes()
                + bytes(16)
                + np.array([4.0, 5.0, 6.0, 0.7], dtype='<f4').tobytes()
                + bytes(16))
    out = decode_xyz(msg)
    assert out.shape == (2, 3)
    assert np.allclose(out, [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]], atol=1e-6)
    assert field_offsets(msg)['z'] == 8


def test_empty_cloud_is_an_empty_array_not_an_error():
    msg = point_cloud2.create_cloud_xyz32(_header(), [])
    out = decode_xyz(msg)
    assert out.shape == (0, 3)


def test_organized_height_is_honoured():
    points = [[float(i), 0.0, 1.0] for i in range(6)]
    msg = point_cloud2.create_cloud_xyz32(_header(), points)
    msg.height = 2
    msg.width = 3
    assert decode_xyz(msg).shape == (6, 3)


def test_missing_fields_are_reported():
    msg = PointCloud2()
    msg.header = _header()
    msg.fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1)]
    msg.width, msg.height, msg.point_step = 1, 1, 4
    msg.data = b'\x00\x00\x00\x00'
    with pytest.raises(ValueError, match='field'):
        decode_xyz(msg)


def test_truncated_buffer_is_reported_rather_than_silently_short():
    """短缓冲是损坏的消息。静默少返回几个点，看起来和"激光没打到"一模一样。"""
    msg = point_cloud2.create_cloud_xyz32(_header(), [[1.0, 2.0, 3.0]] * 10)
    msg.data = bytes(msg.data)[: 12 * 4]
    with pytest.raises(ValueError, match='buffer'):
        decode_xyz(msg)


def test_finite_mask_drops_nan_rows():
    points = np.array([[1.0, 2.0, 3.0],
                       [np.nan, 0.0, 1.0],
                       [0.0, np.inf, 1.0],
                       [4.0, 5.0, 6.0]])
    assert list(finite_mask(points)) == [True, False, False, True]


def test_finite_mask_on_empty_input():
    assert finite_mask(np.zeros((0, 3))).shape == (0,)
