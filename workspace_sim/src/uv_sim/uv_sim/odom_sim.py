"""``/auv/odometry`` -> ``/sim/sensor/odom`` (the simulation sensor namespace).

Why a relay instead of letting consumers read ``/auv/odometry`` directly: the
``/sim/sensor/*`` topics are *ours*.  The engine's namespace is whatever
Stonefish happens to publish (and would change if the scene or the engine
changed); the stack's namespace is a contract we own, it is where the frame
decoding is applied once, and it is what the AQUA-SLAM port (T6) will consume.

Pure passthrough -- the message, its ``header`` (``world_ned`` pose, ``dynamics``
child frame) and its body-frame twist are forwarded unchanged, because T1's
verification and ``docs/frames/frame_convention.md`` describe exactly that
message.  Timestamps are **not** re-stamped: the extra hop must not fake new
samples (that is what made the pre-T5 image-rate measurements inconsistent).
"""

from __future__ import annotations

from nav_msgs.msg import Odometry
from rclpy.node import Node

from uv_sim import runner


class OdomSimNode(Node):
    def __init__(self, node_name: str = 'odom_sim', **kwargs) -> None:
        super().__init__(node_name, **kwargs)

        self.publisher = self.create_publisher(Odometry, '/sim/sensor/odom', 10)
        self.create_subscription(Odometry, '/auv/odometry', self._odom_cb, 10)
        self.count = 0

    def _odom_cb(self, msg: Odometry) -> None:
        self.publisher.publish(msg)
        self.count += 1


def main(args=None) -> int:
    return runner.run(OdomSimNode, args)


if __name__ == "__main__":
    raise SystemExit(main())
