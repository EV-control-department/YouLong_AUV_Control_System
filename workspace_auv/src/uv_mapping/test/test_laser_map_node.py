"""``laser_map`` 节点层测试：参数校验、位姿插值、QA 计数、结束 summary。

节点测试在这里特别值得写，因为**离线回放看不到节点独有的行为**：位姿缓冲、
"没有位姿时把这一帧算进异常计数"、以及 summary 里那份外参快照（一周后回头看
一次测绘，能解释数字的恰恰是它）。
"""

from __future__ import annotations

import json

import numpy as np
import pytest
import rclpy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32MultiArray, Header

from uv_mapping.laser_map import LaserMapNode


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.try_shutdown()


def make_node(**params) -> LaserMapNode:
    """Build the node with the parameters already in force.

    ``map_mode`` is read once in ``__init__`` (it creates the publisher), so
    ``set_parameters`` afterwards would silently test the default mode instead.
    """
    overrides = [rclpy.parameter.Parameter(key, value=value)
                 for key, value in params.items()]
    return LaserMapNode(parameter_overrides=overrides)


def stripe_msg(stamp: float, points=None) -> PointCloud2:
    header = Header()
    header.stamp.sec = int(stamp)
    header.stamp.nanosec = int(round((stamp - int(stamp)) * 1e9))
    header.frame_id = 'line_laser'
    points = points if points is not None else [[4.81, y, 0.0]
                                                for y in np.linspace(-1, 1, 5)]
    return point_cloud2.create_cloud_xyz32(header, points)


def pose_msg(x: float, z: float = 0.0, yaw: float = 0.0) -> Float32MultiArray:
    msg = Float32MultiArray()
    msg.data = [float(x), 0.0, float(z), 0.0, 0.0, float(yaw)]
    return msg


def test_invalid_map_mode_is_rejected():
    with pytest.raises(ValueError, match='map_mode'):
        make_node(map_mode='nonsense')


def test_monitor_mode_creates_no_publisher():
    node = make_node()
    try:
        assert node._map_pub is None
        assert node._mapper is None
    finally:
        node.destroy_node()


def test_live_mode_creates_the_map_publisher_and_an_empty_map():
    node = make_node(map_mode='live')
    try:
        assert node._map_pub is not None
        assert node._mapper is not None
        node._publish_tick()          # 空地图不该抛异常，也不该发东西
    finally:
        node.destroy_node()


def fill_buffer(node, entries):
    """直接摆好位姿缓冲。

    不走 ``_pos_cb``：那里的时间戳来自节点时钟（≈1.8e9），而缓冲会丢弃超过
    ``POSE_BUFFER_S`` 的旧条目 —— 用 100.0 这种假时间去喂会被立刻清空，于是
    "插值"测试其实在测空缓冲（第一版就是这样，拿到的是最后一个采样）。
    """
    from collections import deque

    from uv_mapping.geometry import se3

    node._poses = deque((stamp, se3(np.eye(3), [x, 0.0, 0.0]))
                        for stamp, x in entries)


def test_pose_is_interpolated_to_the_strip_time():
    """位姿缓冲里两个采样之间查询 —— 这是抹掉"梳齿"的那一步。"""
    node = make_node()
    try:
        fill_buffer(node, [(100.0, 0.0), (101.0, 2.0)])
        pose = node._pose_at(100.5)
        assert pose is not None
        assert pose[0, 3] == pytest.approx(1.0, abs=1e-6)
        assert node._pose_stamp == pytest.approx(100.5)
    finally:
        node.destroy_node()


def test_nearest_sample_when_interpolation_is_off():
    node = make_node(pose_interpolate=False)
    try:
        fill_buffer(node, [(100.0, 0.0), (101.0, 2.0)])
        pose = node._pose_at(100.4)
        assert pose is not None
        assert pose[0, 3] == pytest.approx(0.0, abs=1e-9)   # 贴到 100.0 那一帧
        assert node._pose_stamp == pytest.approx(100.0)
    finally:
        node.destroy_node()


def test_pose_buffer_drops_entries_older_than_the_window():
    """缓冲是有界的：否则一次长测绘会把每秒 30 个位姿全留在内存里。"""
    from uv_mapping.laser_map import POSE_BUFFER_S

    node = make_node()
    try:
        node._pos_cb(pose_msg(0.0))
        node._pos_cb(pose_msg(1.0))
        assert len(node._poses) == 2
        # 把最早那一条回拨到窗口之外，下一条位姿到达时它应当被丢掉
        node._poses[0] = (node._poses[0][0] - POSE_BUFFER_S - 1.0, node._poses[0][1])
        node._pos_cb(pose_msg(2.0))
        assert len(node._poses) == 2
        assert node._poses[0][1][0, 3] == pytest.approx(1.0)
    finally:
        node.destroy_node()


def test_pose_at_returns_none_before_any_pose():
    node = make_node()
    try:
        assert node._pose_at(123.0) is None
    finally:
        node.destroy_node()


def test_cloud_callback_counts_strips_without_a_pose():
    """没有位姿时那一帧要记进异常计数，而不是静默丢掉。"""
    node = make_node()
    try:
        node._cloud_cb(stripe_msg(50.0))
        assert node._strips == 1
        assert node._points == 5
        assert node._gap_exceeded == 1
    finally:
        node.destroy_node()


def test_cloud_callback_tracks_association_error_against_max_pose_gap():
    node = make_node(max_pose_gap=0.05)
    try:
        node._cloud_cb(stripe_msg(50.0))          # 无位姿
        node._pos_cb(pose_msg(0.0))
        node._poses[-1] = (50.0, node._poses[-1][1])
        node._cloud_cb(stripe_msg(50.2))          # 有位姿但差了 0.2 s
        assert node._strips == 2
        assert node._worst_assoc == pytest.approx(0.2, abs=1e-6)
        assert node._gap_exceeded == 2
    finally:
        node.destroy_node()


def test_live_mode_accumulates_and_publishes_a_cloud():
    node = make_node(map_mode='live')
    try:
        node._pos_cb(pose_msg(0.0))
        node._poses[-1] = (50.0, node._poses[-1][1])
        node._cloud_cb(stripe_msg(50.0))
        assert node._mapper is not None and len(node._mapper) == 1
        grid = node._mapper.grid()
        assert grid['depth'].size > 0
        assert np.allclose(grid['depth'], 5.0, atol=1e-6)
        node._publish_tick()
    finally:
        node.destroy_node()


def test_live_mode_drops_strips_whose_pose_is_too_stale():
    """位姿太旧的条纹**不进地图**：在动着的船上，把错位条纹放进去比不放过更糟 ——
    误差在空间上是平滑的，它会藏在图里，看起来不像缺失。"""
    node = make_node(map_mode='live', max_pose_gap=0.05)
    try:
        node._pos_cb(pose_msg(0.0))
        node._poses[-1] = (50.0, node._poses[-1][1])
        node._cloud_cb(stripe_msg(50.0))          # 好的一帧
        node._cloud_cb(stripe_msg(50.2))          # 位姿旧了 0.2 s
        assert len(node._mapper) == 1
        assert node._dropped_stale == 1
        assert node._strips == 2                   # 仍计入总数，不是静默丢弃
    finally:
        node.destroy_node()


def test_monitor_mode_never_drops_anything_because_it_maps_nothing():
    node = make_node(max_pose_gap=0.001)
    try:
        node._pos_cb(pose_msg(0.0))
        node._poses[-1] = (50.0, node._poses[-1][1])
        node._cloud_cb(stripe_msg(50.0))
        node._cloud_cb(stripe_msg(50.5))
        assert node._dropped_stale == 0
        assert node._dropped_no_pose == 0
    finally:
        node.destroy_node()


def test_qa_warning_bookkeeping_advances_only_on_new_gaps():
    """告警阈值计数器必须是"只报新增"，否则每 4 秒重复同一行直到测绘结束。

    每个 run 开头的 30 来帧本来就早于第一个位姿，这是正常的，不该一直告警。
    """
    node = make_node(qa_period=1.0)
    try:
        for i in range(3):
            node._cloud_cb(stripe_msg(10.0 + i))
        node._qa_tick()
        assert node._gap_exceeded_reported == node._gap_exceeded == 3
        node._qa_tick()                            # 没有新增
        assert node._gap_exceeded_reported == 3
        node._pos_cb(pose_msg(0.0))
        node._poses[-1] = (10.0, node._poses[-1][1])
        node._cloud_cb(stripe_msg(100.0))          # 新的超限帧
        assert node._gap_exceeded == 4
        node._qa_tick()
        assert node._gap_exceeded_reported == 4
    finally:
        node.destroy_node()


def test_summary_is_written_with_the_extrinsics_snapshot(tmp_path):
    node = make_node(output_dir=str(tmp_path))
    try:
        node._pos_cb(pose_msg(0.0))
        node._poses[-1] = (50.0, node._poses[-1][1])
        for i in range(3):
            node._cloud_cb(stripe_msg(50.0 + i / 30.0))
        node.write_summary()
    finally:
        node.destroy_node()

    files = list(tmp_path.glob('map_qa_*.json'))
    assert len(files) == 1
    summary = json.loads(files[0].read_text(encoding='utf-8'))
    assert summary['strips'] == 3
    assert summary['points_per_strip'] == pytest.approx(5.0)
    assert summary['mode'] == 'monitor'
    assert summary['strips_dropped_no_pose'] == 0
    assert summary['strips_dropped_stale_pose'] == 0
    # 外参快照是这份 sidecar 存在的理由：没有它，一周后没人能解释这些数字
    assert len(summary['T_b_laser']) == 4
    assert summary['T_b_laser'][0][3] == pytest.approx(0.05)
    assert summary['extrinsics_path'].endswith('laser_extrinsics.yaml')


def test_summary_without_output_dir_writes_nothing(tmp_path):
    node = make_node()
    try:
        node._pos_cb(pose_msg(0.0))
        node._cloud_cb(stripe_msg(1.0))
        node.write_summary()          # 只打日志
    finally:
        node.destroy_node()
    assert not list(tmp_path.glob('*.json'))
