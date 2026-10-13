"""``map_replay``: turn a recorded laser survey into a seabed map, offline.

This is the L2 tool the whole "record first, process later" decision was for.
It reads a ``rosbag2`` bag (``ros2 bag record`` is the recorder -- there is no
custom format to invent) and writes:

* ``map.npz``     -- the 2.5-D grid: cell keys, centres, depth, point count,
                     cross-strip spread, flat flag;
* ``metrics.json``-- everything printed, machine-readable, so a run can be
                     compared against a previous one instead of eyeballed;
* ``map.png``     -- with ``--figure``: depth map + the cross-strip histogram.

Usage::

    ros2 run uv_mapping map_replay --bag /tmp/survey/bag --out /tmp/survey/map \\
        --pose state_pos --figure

    # reproduce the pose-source comparison instead of picking one
    ros2 run uv_mapping map_replay --bag ... --pose all

Message classes are resolved **by name** through ``rosidl_runtime_py`` rather
than imported, so the tool carries no compile-time dependency on the message
packages whose topics it reads.  A missing type is reported as "this bag has no
such topic", which is what the user needs to know anyway.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path

import numpy as np

from uv_mapping.bathymetry import DEFAULT_LEAF, BathymetryMap
from uv_mapping.cloud import decode_xyz
from uv_mapping.extrinsics import load_extrinsics
from uv_mapping.geometry import (
    interpolate_poses,
    matrix_to_quat,
    nearest_delta,
    quat_to_matrix,
    rpy_to_matrix,
    se3,
)

LASER_TOPIC = '/sim/sensor/laser_points'

#: Pose source name -> (topic, message type, how to read it).  The three are the
#: ones a real survey can choose between, and the recorded comparison says they
#: are *not* interchangeable:
#:
#:   odometry   30.0 Hz, header-stamped, simulator-only ground truth
#:   state_pos  29.9 Hz, no header (receive time), the MCU topic -- the realistic one
#:   pose_info  21-25 Hz with gaps up to 0.65 s, origin-relative (B20)
POSE_SOURCES = {
    'odometry': ('/auv/odometry', 'nav_msgs/msg/Odometry'),
    'state_pos': ('/zit6/state/pos', 'std_msgs/msg/Float32MultiArray'),
    'pose_info': ('/basic_motion/pose_info', 'uv_msgs/msg/PoseInfo'),
}


# ── bag reading ──────────────────────────────────────────────────────────────
def read_bag(uri: str) -> tuple[dict, dict]:
    """``{topic: [(receive_ns, msg)]}`` and ``{topic: type}`` for one bag."""
    import rosbag2_py
    import yaml
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    storage_id = 'mcap'
    metadata = Path(uri) / 'metadata.yaml'
    if metadata.is_file():
        with metadata.open(encoding='utf-8') as handle:
            info = yaml.safe_load(handle)['rosbag2_bagfile_information']
        storage_id = info.get('storage_identifier', storage_id)

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=uri, storage_id=storage_id),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {entry.name: entry.type
             for entry in reader.get_all_topics_and_types()}

    raw: dict[str, list] = {}
    while reader.has_next():
        topic, payload, receive_ns = reader.read_next()
        raw.setdefault(topic, []).append((receive_ns, payload))

    decoded: dict[str, list] = {}
    for topic, items in raw.items():
        cls = get_message(types[topic])
        decoded[topic] = [(ns, deserialize_message(payload, cls))
                          for ns, payload in items]
    return decoded, types


def _header_stamp(msg) -> float:
    """Seconds from a message's stamp.

    ``PoseInfo`` declares a bare ``stamp`` field while everything else uses
    ``header.stamp``; the difference is not worth two code paths at every call
    site, but it *is* worth one comment.
    """
    stamp = msg.header.stamp if hasattr(msg, 'header') else msg.stamp
    return float(stamp.sec) + 1e-9 * float(stamp.nanosec)


def strip_series(items) -> tuple[np.ndarray, list]:
    """``([stamp], [(N,3) laser-plane points])`` from the laser topic messages."""
    stamps = np.array([_header_stamp(msg) for _, msg in items], dtype=float)
    return stamps, [decode_xyz(msg) for _, msg in items]


def pose_series(name: str, items) -> tuple[np.ndarray, np.ndarray]:
    """``(stamps, (N,4,4) map-frame poses)`` for one pose source."""
    if name == 'odometry':
        stamps, poses = [], []
        for _, msg in items:
            q = msg.pose.pose.orientation
            p = msg.pose.pose.position
            stamps.append(_header_stamp(msg))
            poses.append(se3(quat_to_matrix([q.x, q.y, q.z, q.w]),
                             [p.x, p.y, p.z]))
        return np.array(stamps), np.array(poses)

    if name == 'state_pos':
        # [x, y, z, roll, pitch, yaw] in radians, map frame; no header, so the
        # receive time is the measurement time -- which is also what
        # basic_motion does for PoseInfo.stamp (basic_motion.py:402).
        stamps, poses = [], []
        for receive_ns, msg in items:
            data = np.asarray(msg.data, dtype=float)
            if data.size < 6:
                continue
            stamps.append(receive_ns * 1e-9)
            poses.append(se3(rpy_to_matrix(data[3], data[4], data[5]), data[:3]))
        return np.array(stamps), np.array(poses)

    if name == 'pose_info':
        # origin-relative: T_map_body = T_map_odom(origin) . T_odom_body
        stamps, poses = [], []
        for _, msg in items:
            map_from_odom = se3(rpy_to_matrix(0.0, 0.0, math.radians(msg.origin_yaw)),
                                [msg.origin_x, msg.origin_y, msg.origin_z])
            odom_from_body = se3(
                rpy_to_matrix(math.radians(msg.robot_roll),
                              math.radians(msg.robot_pitch),
                              math.radians(msg.robot_yaw)),
                [msg.robot_x, msg.robot_y, msg.robot_z])
            stamps.append(_header_stamp(msg))
            poses.append(map_from_odom @ odom_from_body)
        return np.array(stamps), np.array(poses)

    raise ValueError(f"unknown pose source {name!r}")


def evaluate(strips, strip_stamps, pose_stamps, pose_matrices, T_b_laser,
             leaf: float, interpolate: bool, label: str) -> tuple[BathymetryMap, dict]:
    """Dead-reckon every strip and collect the numbers worth reporting."""
    if interpolate:
        per_strip = interpolate_poses(pose_stamps, pose_matrices, strip_stamps)
    else:
        from uv_mapping.geometry import nearest_index

        per_strip = pose_matrices[nearest_index(pose_stamps, strip_stamps)]

    mapper = BathymetryMap(leaf=leaf)
    for index, (points, stamp) in enumerate(zip(strips, strip_stamps)):
        mapper.add_strip(points, per_strip[index], T_b_laser)

    metrics = mapper.metrics()
    gaps = np.diff(pose_stamps) if pose_stamps.size > 1 else np.zeros(1)
    metrics.update(
        pose_source=label,
        pose_samples=int(pose_stamps.size),
        pose_rate_hz=float(1.0 / np.mean(gaps)) if gaps.size and gaps.mean() > 0 else 0.0,
        pose_gap_max_s=float(gaps.max()) if gaps.size else 0.0,
        pose_gap_p95_s=float(np.percentile(gaps, 95)) if gaps.size else 0.0,
        pose_interpolated=bool(interpolate),
        association_max_s=float(nearest_delta(pose_stamps, strip_stamps).max())
        if pose_stamps.size else float('inf'),
        association_p95_s=float(np.percentile(
            nearest_delta(pose_stamps, strip_stamps), 95)) if pose_stamps.size else float('inf'),
        strip_points_min=int(min(len(points) for points in strips)) if strips else 0,
        strip_points_p50=int(np.median([len(points) for points in strips])) if strips else 0,
        strip_points_max=int(max(len(points) for points in strips)) if strips else 0,
        empty_strips=int(sum(1 for points in strips if len(points) == 0)),
    )
    return mapper, metrics


# ── reporting ────────────────────────────────────────────────────────────────
def print_metrics(metrics: dict) -> None:
    print(f"  pose source          : {metrics['pose_source']}"
          f"{' (interpolated)' if metrics['pose_interpolated'] else ' (nearest sample)'}")
    print(f"  pose rate            : {metrics['pose_rate_hz']:.2f} Hz, "
          f"gap p95 {metrics['pose_gap_p95_s'] * 1e3:.0f} ms / "
          f"max {metrics['pose_gap_max_s'] * 1e3:.0f} ms")
    print(f"  strip->pose |dt|     : p95 {metrics['association_p95_s'] * 1e3:.1f} ms / "
          f"max {metrics['association_max_s'] * 1e3:.1f} ms")
    print(f"  strips / points      : {metrics['strips']} / {metrics['points']:,}"
          f"  ({metrics['strip_points_min']}..{metrics['strip_points_max']} per strip,"
          f" {metrics['empty_strips']} empty)")
    print(f"  cells / coverage     : {metrics['cells']:,} @ "
          f"({metrics['coverage_m2']:.2f} m^2)")
    print(f"  depth range          : {metrics['depth_min']:.2f} .. "
          f"{metrics['depth_max']:.2f} m")
    print(f"  cross-strip spread   : flat p50 {metrics['flat_spread_p50_cm']:.2f} cm / "
          f"p95 {metrics['flat_spread_p95_cm']:.2f} cm"
          f"   rough p50 {metrics['rough_spread_p50_cm']:.2f} cm")


def write_figure(mapper: BathymetryMap, metrics: dict, path: Path) -> None:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    grid = mapper.grid()
    spread = grid['spread'][grid['flat']]
    spread = spread[~np.isnan(spread)]

    fig, axes = plt.subplots(1, 2, figsize=(13, 5.6))
    scatter = axes[0].scatter(grid['xy'][:, 1], grid['xy'][:, 0],
                              c=grid['depth'], s=5, cmap='turbo_r')
    axes[0].set_title(f"{metrics['pose_source']}: dead-reckoned bathymetry\n"
                      f"{metrics['cells']:,} cells, "
                      f"{metrics['coverage_m2']:.1f} m^2")
    axes[0].set_xlabel('east y [m]')
    axes[0].set_ylabel('north x [m]')
    axes[0].set_aspect('equal')
    fig.colorbar(scatter, ax=axes[0], label='depth z [m] (NED, down = +)')

    if spread.size:
        axes[1].hist(100 * spread, bins=np.linspace(0, 20, 61))
        axes[1].set_title('cross-strip z spread, flat cells\n'
                          f"p50 {metrics['flat_spread_p50_cm']:.2f} cm / "
                          f"p95 {metrics['flat_spread_p95_cm']:.2f} cm")
    axes[1].set_xlabel('cm')
    axes[1].grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


# ── CLI ──────────────────────────────────────────────────────────────────────
def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog='map_replay',
        description='Rebuild a 2.5-D seabed map from a recorded line-laser survey.')
    parser.add_argument('--bag', required=True,
                        help='rosbag2 directory (as written by `ros2 bag record -o`)')
    parser.add_argument('--out', required=True,
                        help='output directory (created if missing)')
    parser.add_argument('--pose', default='state_pos',
                        choices=[*POSE_SOURCES, 'all'],
                        help='pose source; `all` builds each available one for '
                             'comparison (default: state_pos)')
    parser.add_argument('--leaf', type=float, default=DEFAULT_LEAF,
                        help=f'map cell size [m] (default: {DEFAULT_LEAF})')
    parser.add_argument('--extrinsics-file', default='',
                        help='override config/laser_extrinsics.yaml')
    parser.add_argument('--nearest-pose', action='store_true',
                        help='snap each strip to the nearest pose instead of '
                             'interpolating (kept so the difference is measurable, '
                             'not to be used for production maps)')
    parser.add_argument('--max-strips', type=int, default=0,
                        help='use only the first N strips (quick look)')
    parser.add_argument('--figure', action='store_true', help='write map.png')
    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    if not Path(args.bag).is_dir():
        print(f"no such bag directory: {args.bag}", file=sys.stderr)
        return 2

    print(f"reading {args.bag} ...")
    bag, types = read_bag(args.bag)
    if LASER_TOPIC not in bag:
        print(f"bag has no {LASER_TOPIC}; topics present: "
              f"{', '.join(sorted(bag))}", file=sys.stderr)
        return 2
    strip_stamps, strips = strip_series(bag[LASER_TOPIC])
    if args.max_strips > 0:
        strip_stamps = strip_stamps[:args.max_strips]
        strips = strips[:args.max_strips]
    span = float(strip_stamps[-1] - strip_stamps[0]) if strip_stamps.size > 1 else 0.0
    print(f"  {len(strips)} strips over {span:.1f} s "
          f"({len(strips) / span:.2f} Hz)" if span > 0 else f"  {len(strips)} strips")

    T_b_laser, extrinsics_meta = load_extrinsics(explicit=args.extrinsics_file)
    print(f"  extrinsics: {extrinsics_meta.get('path')}")

    if args.pose == 'all':
        wanted = [name for name in POSE_SOURCES if POSE_SOURCES[name][0] in bag]
    else:
        if POSE_SOURCES[args.pose][0] not in bag:
            print(f"bag has no {POSE_SOURCES[args.pose][0]} for --pose "
                  f"{args.pose}; topics present: {', '.join(sorted(bag))}",
                  file=sys.stderr)
            return 2
        wanted = [args.pose]
    if not wanted:
        print("bag has none of the known pose topics", file=sys.stderr)
        return 2

    all_metrics = {}
    for name in wanted:
        print(f"\nbuilding map from {name} ...")
        pose_stamps, pose_matrices = pose_series(name, bag[POSE_SOURCES[name][0]])
        mapper, metrics = evaluate(strips, strip_stamps, pose_stamps, pose_matrices,
                                   T_b_laser, args.leaf,
                                   interpolate=not args.nearest_pose, label=name)
        suffix = '' if len(wanted) == 1 else f'_{name}'
        grid = mapper.grid()
        np.savez_compressed(
            out_dir / f'map{suffix}.npz',
            keys=grid['keys'], xy=grid['xy'], depth=grid['depth'],
            count=grid['count'], spread=grid['spread'], flat=grid['flat'],
            T_b_laser=T_b_laser, leaf=np.array([args.leaf]))
        print_metrics(metrics)
        all_metrics[name] = metrics
        if args.figure:
            figure_path = out_dir / f'map{suffix}.png'
            write_figure(mapper, metrics, figure_path)
            print(f"  figure                : {figure_path}")

    with (out_dir / 'metrics.json').open('w', encoding='utf-8') as handle:
        json.dump({'bag': os.path.abspath(args.bag),
                   'leaf': args.leaf,
                   'extrinsics_path': extrinsics_meta.get('path'),
                   'extrinsics': T_b_laser.tolist(),
                   'pose_sources': all_metrics},
                  handle, indent=2, sort_keys=True)
    print(f"\nmetrics -> {out_dir / 'metrics.json'}")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
