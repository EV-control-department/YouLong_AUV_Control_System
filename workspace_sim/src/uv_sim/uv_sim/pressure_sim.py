"""``/auv/pressure`` -> ``/sim/sensor/pressure`` (the simulation sensor namespace).

Pure passthrough of the 5 Hz ``sensor_msgs/FluidPressure`` sample, whose
``variance`` the engine fills from the scene's noise model.

Registered gap: nothing in the SIL stack consumes it yet (the depth channel of
the control loop comes from odometry).  It exists because ``/sim/sensor/*`` is the
complete engine->stack sensor surface -- T6's SLAM will want it for a depth prior,
and a relay is cheaper than discovering the gap then.  Tracked in
``docs/interfaces/topic_contract.md`` section 9.
"""

from __future__ import annotations

from rclpy.node import Node
from sensor_msgs.msg import FluidPressure

from uv_sim import runner


class PressureSimNode(Node):
    def __init__(self, node_name: str = 'pressure_sim', **kwargs) -> None:
        super().__init__(node_name, **kwargs)

        self.publisher = self.create_publisher(
            FluidPressure, '/sim/sensor/pressure', 10
        )
        self.create_subscription(FluidPressure, '/auv/pressure', self._pressure_cb, 10)
        self.count = 0

    def _pressure_cb(self, msg: FluidPressure) -> None:
        self.publisher.publish(msg)
        self.count += 1


def main(args=None) -> int:
    return runner.run(PressureSimNode, args)


if __name__ == "__main__":
    raise SystemExit(main())
