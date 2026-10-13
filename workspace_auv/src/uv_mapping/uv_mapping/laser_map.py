"""``laser_map``: the line-laser mapping node -- record-time QA, and a live map.

Two modes, chosen by ``map_mode``:

``monitor`` (default)
    Subscribes to the stripe and the pose, and **prints the numbers that decide
    whether the run is worth keeping**.  This is not decoration.  During the
    first survey experiment a dive target below the local seabed pinned the AUV
    against the bottom; it crawled at 0.13 m/s for fifty seconds and the run was
    only thrown away after ten minutes of offline analysis.  A QA line -- pose
    gap, strip-to-pose association error, points per strip, actual ground speed
    -- shows that in the first ten seconds.

    Raw recording is **not** this node's job: ``ros2 bag record`` already stores
    the sufficient statistic (laser strips in the laser frame + the pose stream)
    with a battle-tested tool, and the offline ``map_replay`` turns it into a map.
    A custom on-disk format here would only make the raw data harder to read
    later, and the transform choice it baked in could never be revisited.

``live``
    Additionally accumulates the dead-reckoned map and publishes it as a
    ``PointCloud2`` in the map frame at ``map_publish_period``.  Implemented by
    reusing the same ``BathymetryMap`` core as the replay tool, but treated as
    **unvalidated** (L3): the offline path is where the accuracy claims were
    measured, and this mode has none of its own yet.

Pose source: ``/zit6/state/pos``, not ``/basic_motion/pose_info``.  Measured on
a recorded survey, PoseInfo ran at 21-25 Hz with gaps up to 0.65 s (B20) and its
association error degraded the map's self-consistency by 4.5x at the median; the
MCU topic ran at 29.9 Hz with a 36 ms worst-case interval and exists on the real
vehicle too.  It carries no header, so its measurement time is the receive time
-- exactly the convention ``basic_motion`` already uses for ``PoseInfo.stamp``.
"""

from __future__ import annotations

import json
import time
from collections import deque
from pathlib import Path

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32MultiArray, Header

from uv_mapping.bathymetry import BathymetryMap
from uv_mapping.cloud import decode_xyz
from uv_mapping.extrinsics import load_extrinsics
from uv_mapping.geometry import interpolate_poses, rpy_to_matrix, se3

# Topics as string literals at the create_* calls on purpose: they are the
# contract rows, and scripts/check_docs_contract.py resolves literal topic
# strings only (a module constant shows up as <dynamic:...> and fails the check
# instead of being ignored).  Every other node in this repository does the same.
#
# /sim/sensor/laser_points is the *simulation* stripe (uv_sim/laser_sim.py); the
# real line laser's topic does not exist yet, which is registered as a gap in
# docs/interfaces/topic_contract.md section 9.  /zit6/state/pos is published by
# the MCU on the real vehicle as well.

#: Poses older than this are dropped from the interpolation buffer [s].
POSE_BUFFER_S = 5.0


class LaserMapNode(Node):
    def __init__(self, **kwargs) -> None:
        # ``**kwargs`` (normally ``parameter_overrides``) is forwarded to Node so
        # a test can build the node with a non-default mode; the mode is read once
        # here, so set_parameters() after construction cannot change it.
        super().__init__('laser_map', **kwargs)

        self.declare_parameter('map_mode', 'monitor')
        self.declare_parameter('map_leaf_size', 0.05)
        self.declare_parameter('map_publish_period', 2.0)
        self.declare_parameter('map_window_strips', 600)
        self.declare_parameter('pose_interpolate', True)
        self.declare_parameter('qa_period', 10.0)
        self.declare_parameter('max_pose_gap', 0.25)
        self.declare_parameter('extrinsics_file', '')
        self.declare_parameter('output_dir', '')

        self.mode = str(self.get_parameter('map_mode').value)
        if self.mode not in ('monitor', 'live'):
            raise ValueError(f"map_mode must be monitor|live, got {self.mode!r}")
        self.leaf = float(self.get_parameter('map_leaf_size').value)
        self.publish_period = float(self.get_parameter('map_publish_period').value)
        self.window_strips = int(self.get_parameter('map_window_strips').value)
        self.interpolate = bool(self.get_parameter('pose_interpolate').value)
        self.qa_period = float(self.get_parameter('qa_period').value)
        self.max_pose_gap = float(self.get_parameter('max_pose_gap').value)
        self.output_dir = str(self.get_parameter('output_dir').value)

        self.T_b_laser, self.extrinsics_meta = load_extrinsics(
            explicit=str(self.get_parameter('extrinsics_file').value))

        self.create_subscription(PointCloud2, '/sim/sensor/laser_points',
                                 self._cloud_cb, 10)
        self.create_subscription(Float32MultiArray, '/zit6/state/pos',
                                 self._pos_cb, 10)
        self._map_pub = None
        if self.mode == 'live':
            self._map_pub = self.create_publisher(
                PointCloud2, '/mapping/bathymetry', 1)

        self._poses: deque = deque()
        self._pose_stamp: float | None = None
        self._mapper = None
        if self.mode == 'live':
            # The live map is a **rolling window**, decimated to one point per
            # cell per strip.  Rebuilding the grid costs O(points accumulated),
            # so an unbounded live map gets slower every second and eventually
            # starves the subscriptions it depends on.  Measured before this
            # bound existed: the laser stream fell from 30.0 Hz to 23.8 Hz and
            # the pose gap grew to 0.94 s, while `monitor` mode (no grid) held
            # 30.0 Hz / 44 ms for the whole run.  Offline replay is unbounded on
            # purpose: it has no deadline.
            self._mapper = BathymetryMap(leaf=self.leaf,
                                         max_strips=self.window_strips,
                                         decimate=True)

        # QA counters, all since the last log line / since start for the summary
        self._strips = 0
        self._points = 0
        self._empty = 0
        self._worst_gap = 0.0
        self._worst_assoc = 0.0
        self._gap_exceeded = 0
        self._gap_exceeded_reported = 0
        self._dropped_no_pose = 0
        self._dropped_stale = 0
        self._last_strip_stamp: float | None = None
        self._started = time.monotonic()
        self._first_pose: np.ndarray | None = None
        self._last_pose: np.ndarray | None = None
        self._path_length = 0.0
        self._stopped_report = False

        self.create_timer(self.qa_period, self._qa_tick)
        if self.mode == 'live':
            self.create_timer(self.publish_period, self._publish_tick)

        self.get_logger().info(
            f"laser_map started in '{self.mode}' mode; "
            f"extrinsics {self.extrinsics_meta.get('path')}; "
            f"pose source /zit6/state/pos "
            f"({'interpolated' if self.interpolate else 'nearest sample'})")
        if self.mode == 'live':
            self.get_logger().warning(
                'live mode is not yet validated (L3): the accuracy numbers come '
                'from the offline replay path (map_replay)')

    # ── callbacks ───────────────────────────────────────────────────────────
    def _pos_cb(self, msg: Float32MultiArray) -> None:
        data = np.asarray(msg.data, dtype=float)
        if data.size < 6:
            return
        stamp = self.get_clock().now().nanoseconds * 1e-9
        pose = se3(rpy_to_matrix(data[3], data[4], data[5]), data[:3])
        if self._last_pose is not None:
            self._path_length += float(
                np.linalg.norm(pose[:3, 3] - self._last_pose[:3, 3]))
        else:
            self._first_pose = pose
        self._last_pose = pose
        self._poses.append((stamp, pose))
        while self._poses and stamp - self._poses[0][0] > POSE_BUFFER_S:
            self._poses.popleft()

    def _cloud_cb(self, msg: PointCloud2) -> None:
        stamp = float(msg.header.stamp.sec) + 1e-9 * float(msg.header.stamp.nanosec)
        try:
            points = decode_xyz(msg)
        except ValueError as exc:
            self.get_logger().error(f'undecodable stripe: {exc}')
            return

        self._strips += 1
        self._points += points.shape[0]
        if points.shape[0] == 0:
            self._empty += 1

        if self._last_strip_stamp is not None:
            gap = stamp - self._last_strip_stamp
            if gap > self._worst_gap:
                self._worst_gap = gap
        self._last_strip_stamp = stamp

        pose = self._pose_at(stamp)
        if pose is None:
            # No pose yet: the survey has not started, or the pose topic died.
            self._gap_exceeded += 1
            self._dropped_no_pose += 1
            return
        delta = abs(stamp - self._pose_stamp)
        if delta > self._worst_assoc:
            self._worst_assoc = delta
        if delta > self.max_pose_gap:
            self._gap_exceeded += 1

        if self._mapper is None:
            return
        if delta > self.max_pose_gap:
            # Placing a strip with a pose that stale puts it metres out on a
            # moving vehicle, which is worse than not placing it: the error is
            # spatially smooth, so it hides inside the map instead of looking
            # like a gap.  Drop it and report the count instead.
            self._dropped_stale += 1
            return
        self._mapper.add_strip(points, pose, self.T_b_laser)

    def _pose_at(self, stamp: float) -> np.ndarray | None:
        """Map-frame body pose at ``stamp``, interpolated when asked for.

        Sets ``self._pose_stamp`` to the time the returned pose actually refers
        to, so the QA line can report a real association error rather than the
        buffer's age.
        """
        if not self._poses:
            return None
        stamps = np.array([entry[0] for entry in self._poses], dtype=float)
        poses = np.array([entry[1] for entry in self._poses], dtype=float)
        query = np.array([stamp], dtype=float)
        if self.interpolate and stamps.size >= 2:
            self._pose_stamp = float(np.clip(stamp, stamps[0], stamps[-1]))
            return interpolate_poses(stamps, poses, query)[0]
        index = int(np.argmin(np.abs(stamps - stamp)))
        self._pose_stamp = float(stamps[index])
        return poses[index]

    # ── timers ──────────────────────────────────────────────────────────────
    def _qa_tick(self) -> None:
        now = time.monotonic()
        elapsed = now - self._started
        if self._strips == 0:
            if elapsed > 3.0 * self.qa_period and not self._stopped_report:
                self._stopped_report = True
                self.get_logger().warning(
                    'no stripes received yet -- is the laser publishing? '
                    '(/sim/sensor/laser_points; uv_sim laser_sim owns it in SIL)')
            return
        rate = self._strips / elapsed
        speed = self._path_length / elapsed if elapsed > 0 else 0.0
        self.get_logger().info(
            f'QA: {self._strips} strips @ {rate:.1f} Hz, '
            f'{self._points / self._strips:.0f} pts/strip '
            f'({self._empty} empty), '
            f'pose gap max {self._worst_gap * 1e3:.0f} ms, '
            f'strip->pose |dt| max {self._worst_assoc * 1e3:.1f} ms, '
            f'{self._gap_exceeded} over {self.max_pose_gap * 1e3:.0f} ms'
            f'{self._dropped_text()}'
            f', path {self._path_length:.1f} m (~{speed:.2f} m/s)')
        # Warn on *new* occurrences only.  The counter is cumulative (the summary
        # needs the total), so warning whenever it is non-zero would repeat the
        # same line every tick forever -- and the first 30 strips of every run
        # legitimately arrive before the first pose.
        if self._gap_exceeded > self._gap_exceeded_reported:
            self._gap_exceeded_reported = self._gap_exceeded
            self.get_logger().warning(
                f'{self._gap_exceeded} strips had no pose within '
                f'{self.max_pose_gap * 1e3:.0f} ms. The usual cause is simply the '
                f'start of the run (poses begin ~1 s after the laser); otherwise '
                f'check /zit6/state/pos. Those strips are not put on the map in '
                f'live mode.')

    def _dropped_text(self) -> str:
        if self._mapper is None:
            return ''
        return (f' (dropped: {self._dropped_no_pose} no pose, '
                f'{self._dropped_stale} stale)')

    def _publish_tick(self) -> None:
        if self._mapper is None or self._map_pub is None or not len(self._mapper):
            return
        grid = self._mapper.grid()
        if grid['keys'].shape[0] == 0:
            return
        points = np.column_stack([grid['xy'][:, 0], grid['xy'][:, 1], grid['depth']])
        cloud = point_cloud2.create_cloud_xyz32(
            _header(self.get_clock().now().to_msg(), 'map'), points.tolist())
        cloud.is_dense = True
        self._map_pub.publish(cloud)

    # ── shutdown ────────────────────────────────────────────────────────────
    def write_summary(self) -> None:
        """Print the end-of-run summary and optionally write it to ``output_dir``.

        The sidecar is what turns a directory of bags into something a week later
        is still interpretable: it records the extrinsics that were in force, not
        just the numbers.
        """
        if self._strips == 0:
            self.get_logger().warning('laser_map shutting down: no stripes received')
            return
        elapsed = time.monotonic() - self._started
        summary = {
            'mode': self.mode,
            'map_leaf_size': self.leaf,
            'pose_interpolated': self.interpolate,
            'extrinsics_path': self.extrinsics_meta.get('path'),
            'T_b_laser': self.T_b_laser.tolist(),
            'duration_s': elapsed,
            'strips': self._strips,
            'strip_rate_hz': self._strips / elapsed if elapsed > 0 else 0.0,
            'points': self._points,
            'points_per_strip': self._points / self._strips,
            'empty_strips': self._empty,
            'pose_gap_max_s': self._worst_gap,
            'strip_pose_dt_max_s': self._worst_assoc,
            'strips_over_max_pose_gap': self._gap_exceeded,
            'strips_dropped_no_pose': self._dropped_no_pose,
            'strips_dropped_stale_pose': self._dropped_stale,
            'path_length_m': self._path_length,
            'mean_speed_mps': self._path_length / elapsed if elapsed > 0 else 0.0,
        }
        self.get_logger().info(
            f"summary: {summary['strips']} strips, "
            f"{summary['points_per_strip']:.0f} pts/strip, "
            f"{summary['path_length_m']:.1f} m at "
            f"{summary['mean_speed_mps']:.2f} m/s, "
            f"pose gap max {summary['pose_gap_max_s'] * 1e3:.0f} ms")
        if not self.output_dir:
            return
        target = Path(self.output_dir)
        try:
            target.mkdir(parents=True, exist_ok=True)
            name = time.strftime('map_qa_%Y%m%d_%H%M%S.json')
            with (target / name).open('w', encoding='utf-8') as handle:
                json.dump(summary, handle, indent=2, sort_keys=True)
            self.get_logger().info(f'QA summary written to {target / name}')
        except OSError as exc:
            self.get_logger().error(f'could not write QA summary: {exc}')


def _header(stamp, frame_id: str) -> Header:
    header = Header()
    header.stamp = stamp
    header.frame_id = frame_id
    return header


def main(args=None) -> int:
    rclpy.init(args=args)
    node = LaserMapNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Raised when the signal handler already shut the context down
        # (Ctrl-C / SIGTERM via ros2 launch); the summary still has to run.
        pass
    finally:
        try:
            node.write_summary()
        finally:
            node.destroy_node()
            rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
