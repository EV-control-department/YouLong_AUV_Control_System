"""The simulated ZIT6 MCU: setpoints in, forces out, status back.

SIL role (``hil_mode:=false``, the default and the acceptance path): owns the whole
MCU contract -- ``/zit6/cmd/*`` in, cascaded PID, thruster mixing,
``/auv/thrusters_cmd`` out, ``/zit6/state/{status,thr,zithbt}`` back, plus the
``/zit6/{get,update}_params`` services.  Input: ``/sim/sensor/{odom,imu}`` (the
filtered engine namespace, **not** ``/zit6/state/*`` -- reading its own output
would make the loop self-referential).

HIL role (``hil_mode:=true``): thruster relay only.  The real MCU computes PID and
mixing and publishes ``/zit6/state/thr``; this node subscribes that and mixes it for
Stonefish with the normalised scale (``MAX_THRUST_HIL``).  It deliberately creates
**no** ``/zit6/state/*`` publisher and **no** service, so invariant I2 (one publisher
per state topic) cannot be broken by starting both roles at once, and it loads no
gain file because it runs no PID.

The control maths and the setpoint state machine live in
:mod:`uv_sim.mcu_control` (ROS-free, replayable against
``test/data/thrust_golden.json``); this file is only the ROS glue.
"""

from __future__ import annotations

import json

from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray, Float32, UInt8, UInt32, Float32MultiArray

from zit6_interfaces.msg import ZitSetpoint, ZitStatus
from zit6_interfaces.srv import GetParams, UpdateParams

from uv_sim import runner
from uv_sim.mcu_control import McuController
from uv_sim.mixer import MAX_THRUST_HIL, MAX_THRUST_SIL, mix_thruster_forces
from uv_sim.pid_config import load_pid_params, resolve_pid_config_path

HEARTBEAT_PERIOD_SECONDS = 0.1
CONTROL_PERIOD_SECONDS = 0.0167
TICK_CYCLE = 60             # LCM of the two gates below (pre-T5 gating kept)
STATUS_EVERY_N_TICKS = 6    # 10 Hz out of the 60 Hz tick
THRUST_EVERY_N_TICKS = 2    # 30 Hz


class McuSimNode(Node):
    def __init__(self, node_name: str = 'mcu_sim', **kwargs) -> None:
        super().__init__(node_name, **kwargs)

        self.declare_parameter('hil_mode', False)
        self.declare_parameter('pid_config_file', '')
        self._hil_mode = bool(self.get_parameter('hil_mode').value)
        self._pid_config_file = self.get_parameter('pid_config_file').value

        self._controller = None
        self.thrust = [0.0] * 6
        self._max_thrust = MAX_THRUST_HIL if self._hil_mode else MAX_THRUST_SIL
        self._tick = 0

        self.thruster_pub = self.create_publisher(
            Float64MultiArray, '/auv/thrusters_cmd', 10
        )

        if self._hil_mode:
            self._init_relay()
            self.get_logger().info(
                "mcu_sim started in HIL mode: relaying /zit6/state/thr to "
                "/auv/thrusters_cmd (PID, mixing and /zit6/state/* belong to the MCU)"
            )
        else:
            self._init_simulated_mcu()
            self.get_logger().info(
                "mcu_sim started (ZIT6 protocol, xunyun mixer, "
                f"max thrust {self._max_thrust:g})"
            )

    # ── HIL: thruster relay ─────────────────────────────────────────────────

    def _init_relay(self) -> None:
        self.create_subscription(
            Float32MultiArray, '/zit6/state/thr', self._thrust_cb, 10
        )

    def _thrust_cb(self, msg: Float32MultiArray) -> None:
        """Receive 4-DOF forces from the MCU, mix, publish to Stonefish."""
        if len(msg.data) >= 4:
            self._publish_thrust_from_4dof(
                msg.data[0], msg.data[1], msg.data[2], msg.data[3]
            )

    # ── SIL: the whole simulated MCU ────────────────────────────────────────

    def _init_simulated_mcu(self) -> None:
        self._pid_config_path = resolve_pid_config_path(self._pid_config_file)
        self._pid_params = load_pid_params(self._pid_config_path, self.get_logger())
        self._controller = McuController(
            self._pid_params, start_time=self._now_seconds()
        )

        self.zit6_status_pub = self.create_publisher(ZitStatus, '/zit6/state/status', 10)
        self.zit6_thr_pub = self.create_publisher(Float32MultiArray, '/zit6/state/thr', 10)
        self.zit6_hbt_pub = self.create_publisher(UInt32, '/zit6/state/zithbt', 10)

        self.create_subscription(ZitSetpoint, '/zit6/cmd/setpoint', self._setpoint_cb, 10)
        self.create_subscription(Float32, '/zit6/cmd/servo', self._servo_cb, 10)
        self.create_subscription(UInt8, '/zit6/cmd/light', self._light_cb, 10)
        self.create_subscription(UInt32, '/zit6/cmd/agxhbt', self._agxhbt_cb, 10)
        self.create_subscription(UInt8, '/zit6/cmd/ins', self._ins_cb, 10)
        self.create_subscription(Odometry, '/sim/sensor/odom', self._odom_cb, 10)
        self.create_subscription(Imu, '/sim/sensor/imu', self._imu_cb, 10)

        self.create_service(GetParams, '/zit6/get_params', self._get_params_cb)
        self.create_service(UpdateParams, '/zit6/update_params', self._update_params_cb)

        self.create_timer(HEARTBEAT_PERIOD_SECONDS, self._publish_zithbt)
        self.create_timer(CONTROL_PERIOD_SECONDS, self._tick_60hz)

        self.get_logger().info(
            "PID gains loaded from "
            f"{self._pid_config_path or '<built-in defaults>'} "
            f"({len(self._controller.pids)} channels)"
        )

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ── setpoint / command inputs ───────────────────────────────────────────

    def _setpoint_cb(self, msg: ZitSetpoint) -> None:
        immediate = self._controller.apply_setpoint(
            msg.control_key, msg.type_mask,
            msg.x, msg.y, msg.z, msg.yaw,
            self._now_seconds(),
        )
        if immediate is not None:      # force mode emits output straight away
            self._publish_thrust_from_4dof(*immediate)

    def _servo_cb(self, msg: Float32) -> None:
        self.get_logger().info(f"Servo: {msg.data:.3f} rad")

    def _light_cb(self, msg: UInt8) -> None:
        # 2=绿 3=黄（2026-10-13 更正：此前写 2=yellow/3=green，与 task_runner 的
        # `LIGHT_GREEN=2` / `LIGHT_YELLOW=3` 以及实机都相反 —— 调试时会把发的绿报成
        # 黄，把人带偏）。权威定义见 docs/interfaces/indicator_light.md §2。
        colors = {1: "red", 2: "green", 3: "yellow"}
        name = colors.get(msg.data, f"unknown ({msg.data})")
        self.get_logger().info(f"Light: {name}")

    def _agxhbt_cb(self, msg: UInt32) -> None:
        for message in self._controller.note_heartbeat(msg.data):
            self.get_logger().info(message)

    def _ins_cb(self, msg: UInt8) -> None:
        for message in self._controller.ins_command(msg.data, self._now_seconds()):
            self.get_logger().info(message)

    # ── sensor inputs ───────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry) -> None:
        self._controller.apply_odometry(msg)

    def _imu_cb(self, msg: Imu) -> None:
        self._controller.apply_imu(msg)

    # ── parameter services ──────────────────────────────────────────────────

    def _get_params_cb(self, request, response):
        """Return PID parameters as JSON for the requested paths."""
        response.success = True
        response.config_json = json.dumps(
            self._controller.get_params(list(request.paths))
        )
        return response

    def _update_params_cb(self, request, response):
        """Update PID parameters from JSON or path/value pairs (runtime only)."""
        try:
            if request.json:
                updates = json.loads(request.json)
            else:
                updates = dict(zip(request.paths, request.values))

            for message in self._controller.apply_param_updates(updates):
                self.get_logger().info(message)

            response.success = True
            response.message = "Parameters updated"
        except Exception as error:
            response.success = False
            response.message = str(error)
        return response

    # ── thruster mixer (xunyun 6-thruster geometry) ─────────────────────────

    def _publish_thrust_from_4dof(self, x: float, y: float, z: float, rz: float) -> None:
        # DISABLED: always armed for now
        # if not self._armed:
        #     x = y = z = rz = 0.0
        if self._controller is not None:
            self._controller.force_4dof = [x, y, z, rz]
        raw = mix_thruster_forces(x, y, z, rz, max_thrust=self._max_thrust)

        cmd = Float64MultiArray()
        cmd.data = raw
        self.thruster_pub.publish(cmd)

        for index in range(6):
            self.thrust[index] = float(raw[index])

    # ── control step and status publishing ──────────────────────────────────

    def _tick_60hz(self) -> None:
        """60 Hz control tick, with rate-gated status/thrust publishing."""
        self._tick = (self._tick + 1) % TICK_CYCLE
        if self._tick % STATUS_EVERY_N_TICKS == 0:
            self._publish_state()
        if self._tick % THRUST_EVERY_N_TICKS == 0:
            self._publish_thr()

        for message in self._controller.update_ins_alignment(self._now_seconds()):
            self.get_logger().info(message)

        if not self._controller.current_pose_ready:
            return

        dt = self._controller.compute_dt(self._now_seconds())
        force = self._controller.step(dt)
        if force is not None:
            self._publish_thrust_from_4dof(*force)

    def _publish_state(self) -> None:
        """Publish ZitStatus (10 Hz)."""
        controller = self._controller
        status = ZitStatus()
        status.is_armed = controller.armed
        status.arm_mode = controller.arm_mode
        status.control_level = controller.current_mode
        status.ins_state = controller.ins_state
        status.navigation_ready = controller.ins_nav_ready
        status.forces = [controller.force_4dof[0], controller.force_4dof[1],
                         controller.force_4dof[2], 0.0, 0.0, controller.force_4dof[3]]
        status.cycle_time_ms = 50.0
        status.battery_voltage = 16.8
        status.error_flags = 0
        self.zit6_status_pub.publish(status)

    def _publish_thr(self) -> None:
        """Publish thrust 6-DOF [Fx, Fy, Fz, Mx(roll), My(pitch), Mz(yaw)] (30 Hz)."""
        force = self._controller.force_4dof
        thr_msg = Float32MultiArray()
        # force_4dof = [Fx, Fy, Fz, Mz] -> 6 elements, Mx/My stay 0
        thr_msg.data = [force[0], force[1], force[2], 0.0, 0.0, force[3]]
        self.zit6_thr_pub.publish(thr_msg)

    def _publish_zithbt(self) -> None:
        # Always armed -- the heartbeat timeout is disabled (see mcu_control)
        msg = UInt32()
        msg.data = self._controller.next_heartbeat_seq()
        self.zit6_hbt_pub.publish(msg)


def main(args=None) -> int:
    return runner.run(McuSimNode, args)


if __name__ == "__main__":
    raise SystemExit(main())
