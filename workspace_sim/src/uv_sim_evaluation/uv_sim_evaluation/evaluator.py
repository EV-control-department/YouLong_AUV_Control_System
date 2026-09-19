"""Publish simulation metrics without exposing Ground Truth to the stack.

This is the only V1 node allowed to subscribe to
``/auv/sim/ground_truth/odom``.  The formal state is read from
``/auv/state/odom``; no command, planner, controller, or estimator is
connected to the Ground Truth subscription.
"""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path
import time

from auv_protocol.topics import (
    EVALUATION_EVENTS,
    EVALUATION_METRICS,
    SIM_DEGRADATION_EVENTS,
    SIM_GT_ODOM,
    SIM_CONTROL_PERFORMANCE,
    STATE_HEALTH,
    STATE_ODOM,
)
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import String
from uv_msgs.msg import PoseInfo, SensorHealth

from .metrics import ate_rmse, rpe_rotation, rpe_translation


class EvaluationNode(Node):
    """Evaluation-only subscriber and metrics publisher."""

    def __init__(self) -> None:
        super().__init__('uv_sim_evaluation')
        self.declare_parameter('output_dir', '')
        self.declare_parameter('run_id', '')
        self.declare_parameter('publish_period', 1.0)
        self.declare_parameter('seed', 0)
        self._truth = None
        self._estimate = None
        self._estimates = []
        self._truths = []
        self._truth_origin = None
        self._health_samples = 0
        self._health_available = 0
        self._degradation_events = 0
        self._control_performance = None
        self._run_started = time.time()
        self._output_dir = str(self.get_parameter('output_dir').value).strip()
        run_id = str(self.get_parameter('run_id').value).strip()
        self._run_id = run_id or time.strftime('%Y%m%d_%H%M%S')

        self._metrics_pub = self.create_publisher(String, EVALUATION_METRICS, 10)
        self._events_pub = self.create_publisher(String, EVALUATION_EVENTS, 10)
        # Ground Truth is deliberately subscribed only in this package.
        self.create_subscription(Odometry, SIM_GT_ODOM, self._truth_cb, 10)
        self.create_subscription(PoseInfo, STATE_ODOM, self._estimate_cb, 10)
        self.create_subscription(SensorHealth, STATE_HEALTH,
                                 self._health_cb, 10)
        self.create_subscription(String, SIM_DEGRADATION_EVENTS,
                                 self._degradation_cb, 10)
        self.create_subscription(String, SIM_CONTROL_PERFORMANCE,
                                 self._performance_cb, 10)
        self._trajectory_path = None
        if self._output_dir:
            output = Path(self._output_dir)
            output.mkdir(parents=True, exist_ok=True)
            self._trajectory_path = output / 'trajectory.csv'
            with self._trajectory_path.open(
                    'w', newline='', encoding='utf-8') as stream:
                csv.writer(stream).writerow([
                    'stamp', 'estimate_x', 'estimate_y', 'estimate_z',
                    'estimate_yaw_rad', 'truth_x', 'truth_y', 'truth_z',
                    'truth_yaw_rad',
                ])
        period = max(0.1, float(self.get_parameter('publish_period').value))
        self.create_timer(period, self._publish_metrics)
        self._emit('evaluation_started')

    def _truth_cb(self, msg: Odometry) -> None:
        self._truth = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            float(msg.pose.pose.position.z),
            math.atan2(
                2.0 * msg.pose.pose.orientation.w *
                msg.pose.pose.orientation.z,
                1.0 - 2.0 * msg.pose.pose.orientation.z ** 2),
        )
        self._pair_if_ready()

    def _estimate_cb(self, msg: PoseInfo) -> None:
        self._estimate = (
            float(msg.robot_x), float(msg.robot_y), float(msg.robot_z),
            math.radians(float(msg.robot_yaw)),
        )
        self._pair_if_ready()

    def _health_cb(self, msg: SensorHealth) -> None:
        self._health_samples += 1
        if bool(msg.available):
            self._health_available += 1

    def _degradation_cb(self, _msg: String) -> None:
        self._degradation_events += 1

    def _performance_cb(self, msg: String) -> None:
        """Keep the bridge's measured control timing with the run metrics."""
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if isinstance(payload, dict):
            self._control_performance = payload

    def _pair_if_ready(self) -> None:
        if self._truth is None or self._estimate is None:
            return
        if self._truth_origin is None:
            # The estimator publishes local odom while Stonefish publishes
            # world coordinates.  Align the translational origin once for
            # ATE/RPE; this does not feed the transform back into the stack.
            self._truth_origin = self._truth
        aligned_truth = tuple(
            value - origin
            for value, origin in zip(self._truth, self._truth_origin))
        self._truths.append(aligned_truth)
        self._estimates.append(self._estimate)
        if self._trajectory_path is not None:
            with self._trajectory_path.open(
                    'a', newline='', encoding='utf-8') as stream:
                csv.writer(stream).writerow([
                    time.time(), *self._estimate, *aligned_truth,
                ])
        self._truth = self._estimate = None

    def _metrics(self) -> dict:
        estimates, truths = self._estimates, self._truths
        max_error = max(
            (math.sqrt(sum((a - b) ** 2
                           for a, b in zip(estimate[:3], truth[:3])))
             for estimate, truth in zip(estimates, truths)), default=0.0)
        performance = self._control_performance or {}
        return {
            'run_id': self._run_id,
            'samples': len(estimates),
            'ate_rmse_m': ate_rmse(estimates, truths),
            'rpe_translation_rmse_m': rpe_translation(estimates, truths),
            'rpe_rotation_rmse_rad': rpe_rotation(estimates, truths),
            'max_position_error_m': max_error,
            'duration_s': max(0.0, time.time() - self._run_started),
            'mission_time_s': max(0.0, time.time() - self._run_started),
            'sensor_availability': (
                self._health_available / self._health_samples
                if self._health_samples else 0.0),
            'degradation_event_count': self._degradation_events,
            'control_performance_available': bool(performance),
            'control_frequency_hz': performance.get('frequency_hz'),
            'control_period_mean_ms': performance.get('period_mean_ms'),
            'control_period_stddev_ms': performance.get('period_stddev_ms'),
            'control_max_jitter_ms': performance.get('max_jitter_ms'),
            'control_deadline_misses': performance.get('deadline_misses'),
            'seed': int(self.get_parameter('seed').value),
        }

    def _publish_metrics(self) -> None:
        payload = self._metrics()
        if self._output_dir:
            output = Path(self._output_dir)
            output.mkdir(parents=True, exist_ok=True)
            (output / 'metrics.json').write_text(
                json.dumps(payload, indent=2, sort_keys=True) + '\n',
                encoding='utf-8')
        if rclpy.ok():
            msg = String()
            msg.data = json.dumps(payload, sort_keys=True)
            try:
                self._metrics_pub.publish(msg)
            except Exception:
                # The launch shutdown sequence can invalidate the context
                # between rclpy.ok() and publish(); disk metrics remain valid.
                pass

    def _emit(self, event: str) -> None:
        msg = String()
        msg.data = json.dumps({'event': event, 'run_id': self._run_id})
        self._events_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EvaluationNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node._publish_metrics()
            if rclpy.ok():
                node._emit('evaluation_finished')
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        try:
            node.destroy_node()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
