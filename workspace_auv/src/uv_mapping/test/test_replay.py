"""``map_replay`` 端到端：真写一个 mcap bag，再跑 CLI 读回来。

这是"记录优先"这条路的闭环测试：rosbag2 写盘 → CLI 读 → 网格 + metrics.json。
用合成 bag 而不是录一份真数据，是为了让期望值是解析可算的（水平面 = 恒定深度），
并且测试可以离线跑、几秒钟完成。
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
import pytest
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32MultiArray, Header

from uv_mapping.replay import (
    LASER_TOPIC,
    POSE_SOURCES,
    build_parser,
    main,
    pose_series,
    read_bag,
    strip_series,
)

#: 与配置一致的安装（本文件只用旋转部分推条纹，深度由 down 直接给）
T_B_ROT = np.array([[0.0, 0.0, -1.0],
                    [0.0, 1.0, 0.0],
                    [1.0, 0.0, 0.0]])
T_B_OFFSET = np.array([0.05, 0.0, 0.19])


def _header(stamp: float) -> Header:
    header = Header()
    header.stamp.sec = int(stamp)
    header.stamp.nanosec = int(round((stamp - int(stamp)) * 1e9))
    header.frame_id = 'line_laser'
    return header


def write_bag(path: Path, strips, state_pos_poses, odometry_poses=None) -> None:
    """Write a minimal mcap bag.

    ``strips`` is ``[(stamp, [(x, y, z) in the laser plane])]``;
    ``state_pos_poses`` / ``odometry_poses`` are ``[(stamp, xyz, rpy_rad)]``.
    """
    rosbag2_py = pytest.importorskip('rosbag2_py')
    from rclpy.serialization import serialize_message
    from rosbag2_py import ConverterOptions, StorageOptions, TopicMetadata
    from sensor_msgs_py import point_cloud2

    writer = rosbag2_py.SequentialWriter()
    writer.open(StorageOptions(uri=str(path), storage_id='mcap'),
                ConverterOptions('cdr', 'cdr'))
    # TopicMetadata 的 id 是位置参数（关键字形式在 Jazzy 上会 TypeError）
    def topic(name: str, msg_type: str, topic_id: int) -> None:
        writer.create_topic(TopicMetadata(topic_id, name, msg_type, 'cdr'))

    topic(LASER_TOPIC, 'sensor_msgs/msg/PointCloud2', 0)
    topic(POSE_SOURCES['state_pos'][0], 'std_msgs/msg/Float32MultiArray', 1)
    if odometry_poses is not None:
        topic(POSE_SOURCES['odometry'][0], 'nav_msgs/msg/Odometry', 2)

    for stamp, points in strips:
        msg = point_cloud2.create_cloud_xyz32(_header(stamp), points)
        writer.write(LASER_TOPIC, serialize_message(msg), int(stamp * 1e9))

    for stamp, xyz, rpy in state_pos_poses:
        msg = Float32MultiArray()
        msg.data = [float(v) for v in (*xyz, *rpy)]
        writer.write(POSE_SOURCES['state_pos'][0], serialize_message(msg),
                     int(stamp * 1e9))

    if odometry_poses is not None:
        from nav_msgs.msg import Odometry

        for stamp, xyz, rpy in odometry_poses:
            msg = Odometry()
            msg.header = _header(stamp)
            msg.pose.pose.position.x = float(xyz[0])
            msg.pose.pose.position.y = float(xyz[1])
            msg.pose.pose.position.z = float(xyz[2])
            cr, sr = math.cos(rpy[0] / 2), math.sin(rpy[0] / 2)
            cp, sp = math.cos(rpy[1] / 2), math.sin(rpy[1] / 2)
            cy, sy = math.cos(rpy[2] / 2), math.sin(rpy[2] / 2)
            msg.pose.pose.orientation.w = cr * cp * cy + sr * sp * sy
            msg.pose.pose.orientation.x = sr * cp * cy - cr * sp * sy
            msg.pose.pose.orientation.y = cr * sp * cy + sr * cp * sy
            msg.pose.pose.orientation.z = cr * cp * sy - sr * sp * cy
            writer.write(POSE_SOURCES['odometry'][0], serialize_message(msg),
                         int(stamp * 1e9))



def flat_survey(n_strips: int = 120, spacing: float = 0.01, depth: float = 5.0,
                n_points: int = 11):
    """水平海床上的匀速直线测绘：条纹与位姿都按 30 Hz 生成。"""
    across = np.linspace(-0.25, 0.25, n_points)
    strips, poses = [], []
    for i in range(n_strips):
        stamp = 1000.0 + i / 30.0
        pose_x = i * spacing
        down = depth - T_B_OFFSET[2]           # 激光器比机身高 0.19 m
        plane = np.column_stack([np.full(across.size, down), across,
                                 np.zeros(across.size)])
        strips.append((stamp, plane.tolist()))
        poses.append((stamp, [pose_x, 0.0, 0.0], [0.0, 0.0, 0.0]))
    return strips, poses


# ── bag 读回 ────────────────────────────────────────────────────────────────
def test_read_bag_and_strip_series_round_trip(tmp_path):
    strips, poses = flat_survey(n_strips=5)
    write_bag(tmp_path / 'bag', strips, poses)

    bag, types = read_bag(str(tmp_path / 'bag'))
    assert LASER_TOPIC in bag and types[LASER_TOPIC] == 'sensor_msgs/msg/PointCloud2'
    stamps, clouds = strip_series(bag[LASER_TOPIC])
    assert len(clouds) == 5
    assert stamps[0] == pytest.approx(1000.0, abs=1e-6)
    assert clouds[0].shape == (11, 3)


def test_pose_series_state_pos_reads_the_six_element_vector(tmp_path):
    strips, poses = flat_survey(n_strips=4)
    write_bag(tmp_path / 'bag', strips, poses)
    bag, _ = read_bag(str(tmp_path / 'bag'))

    stamps, matrices = pose_series('state_pos', bag[POSE_SOURCES['state_pos'][0]])
    assert len(stamps) == 4
    assert np.allclose(matrices[0][:3, :3], np.eye(3), atol=1e-12)
    assert matrices[1][0, 3] == pytest.approx(0.01)


def test_pose_series_odometry_uses_the_header_stamp(tmp_path):
    strips, poses = flat_survey(n_strips=3)
    write_bag(tmp_path / 'bag', strips, poses, odometry_poses=poses)
    bag, _ = read_bag(str(tmp_path / 'bag'))

    stamps, matrices = pose_series('odometry', bag[POSE_SOURCES['odometry'][0]])
    assert stamps == pytest.approx([1000.0, 1000.0 + 1 / 30, 1000.0 + 2 / 30])
    assert matrices[2][0, 3] == pytest.approx(0.02)


def test_pose_series_unknown_source_raises():
    with pytest.raises(ValueError):
        pose_series('nope', [])


# ── CLI ─────────────────────────────────────────────────────────────────────
def test_cli_end_to_end_writes_a_map_at_the_right_depth(tmp_path):
    strips, poses = flat_survey(n_strips=120)
    write_bag(tmp_path / 'bag', strips, poses)
    out = tmp_path / 'map'

    assert main(['--bag', str(tmp_path / 'bag'), '--out', str(out),
                 '--pose', 'state_pos']) == 0

    assert (out / 'map.npz').is_file()
    data = np.load(out / 'map.npz')
    assert data['depth'].size > 0
    assert np.allclose(data['depth'], 5.0, atol=1e-6)
    assert data['xy'].shape == data['depth'].shape + (2,)

    metrics = json.loads((out / 'metrics.json').read_text(encoding='utf-8'))
    assert metrics['pose_sources']['state_pos']['depth_min'] == pytest.approx(5.0)
    assert metrics['extrinsics'] == data['T_b_laser'].tolist()
    assert metrics['leaf'] == 0.05


def test_cli_reports_a_bag_without_the_laser_topic(tmp_path, capsys):
    strips, poses = flat_survey(n_strips=3)
    write_bag(tmp_path / 'bag', strips, poses)
    bag = tmp_path / 'bag'
    # rename the laser topic's data away by writing a bag with only poses
    write_bag(tmp_path / 'poses_only', [], poses)

    assert main(['--bag', str(bag), '--out', str(tmp_path / 'a')]) == 0
    assert main(['--bag', str(tmp_path / 'poses_only'),
                 '--out', str(tmp_path / 'b')]) == 2
    assert 'no /sim/sensor/laser_points' in capsys.readouterr().err


def test_cli_max_strips_trims_the_run(tmp_path):
    strips, poses = flat_survey(n_strips=60)
    write_bag(tmp_path / 'bag', strips, poses)
    out = tmp_path / 'map'

    assert main(['--bag', str(tmp_path / 'bag'), '--out', str(out),
                 '--pose', 'state_pos', '--max-strips', '10']) == 0
    metrics = json.loads((out / 'metrics.json').read_text(encoding='utf-8'))
    assert metrics['pose_sources']['state_pos']['strips'] == 10


def test_cli_pose_all_builds_every_available_source(tmp_path):
    strips, poses = flat_survey(n_strips=40)
    write_bag(tmp_path / 'bag', strips, poses, odometry_poses=poses)
    out = tmp_path / 'map'

    assert main(['--bag', str(tmp_path / 'bag'), '--out', str(out),
                 '--pose', 'all']) == 0
    assert (out / 'map_odometry.npz').is_file()
    assert (out / 'map_state_pos.npz').is_file()
    metrics = json.loads((out / 'metrics.json').read_text(encoding='utf-8'))
    assert set(metrics['pose_sources']) == {'odometry', 'state_pos'}


def test_cli_nearest_pose_flag_is_recorded_in_metrics(tmp_path):
    strips, poses = flat_survey(n_strips=40)
    write_bag(tmp_path / 'bag', strips, poses)
    out = tmp_path / 'map'

    assert main(['--bag', str(tmp_path / 'bag'), '--out', str(out),
                 '--pose', 'state_pos', '--nearest-pose']) == 0
    metrics = json.loads((out / 'metrics.json').read_text(encoding='utf-8'))
    assert metrics['pose_sources']['state_pos']['pose_interpolated'] is False


def test_cli_rejects_a_missing_bag(tmp_path, capsys):
    assert main(['--bag', str(tmp_path / 'nope'), '--out', str(tmp_path / 'o')]) == 2
    assert 'no such bag' in capsys.readouterr().err


def test_parser_defaults_are_the_documented_ones():
    args = build_parser().parse_args(['--bag', 'b', '--out', 'o'])
    assert args.pose == 'state_pos'
    assert args.leaf == pytest.approx(0.05)
    assert args.nearest_pose is False
    assert args.figure is False
