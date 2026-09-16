"""``/auv/imu`` -> ``/sim/sensor/imu`` (the simulation sensor namespace).

Pure passthrough: 200 Hz raw IMU with the noise injected by the scene, forwarded
unchanged (including the header stamp and the ``imu_filter`` frame id).

The fused attitude/rate channels the control stack actually consumes are *not*
built here -- ``nav_sim`` aggregates ``/sim/sensor/{odom,imu}`` into
``/zit6/state/*``, which is where the MCU protocol lives.  Keeping the raw sensor
and the MCU-protocol view in separate nodes is what makes the SIL/HIL swap
possible: with a real MCU the same raw topics exist, and ``nav_sim``'s
``publish_state:=false`` hands ``/zit6/state/*`` back to the hardware.
"""

from __future__ import annotations

from rclpy.node import Node
from sensor_msgs.msg import Imu

from uv_sim import runner


class ImuSimNode(Node):
    def __init__(self, node_name: str = 'imu_sim', **kwargs) -> None:
        super().__init__(node_name, **kwargs)

        self.publisher = self.create_publisher(Imu, '/sim/sensor/imu', 10)
        self.create_subscription(Imu, '/auv/imu', self._imu_cb, 10)
        self.count = 0

    def _imu_cb(self, msg: Imu) -> None:
        self.publisher.publish(msg)
        self.count += 1


def main(args=None) -> int:
    return runner.run(ImuSimNode, args)


if __name__ == "__main__":
    raise SystemExit(main())
