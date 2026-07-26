"""Hardware manager node: heartbeat, state monitoring.

Responsibilities:
- 10Hz heartbeat on /zit6/cmd/agxhbt to keep MCU armed
- Subscribe to /zit6/state/status, /zit6/state/zithbt, /zit6/state/thr
- Parse and log MCU state in human-readable format
- Watchdog: heartbeat timeout (7s), battery low, error flags, thrust sat
- INS startup sequence tracking
"""

import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, UInt32

from zit6_interfaces.msg import ZitStatus


# ── INS state → human-readable name ─────────────────────────────
_INS_NAMES = {
    0: '待机',
    1: '粗对准',
    2: '精对准',
    3: 'SINS/GPS/DVL',
    4: 'SINS/DVL',
    5: 'MRU',
}

# ── Control level → human-readable name ─────────────────────────
_CTRL_NAMES = {
    0: 'NONE',
    1: 'POS',
    2: 'VEL',
    3: 'FORCE',
}

# ── Error flag bit definitions (matching ZitStatus constants) ───
_ERROR_BITS = [
    (1 << 0, 'FORCE_STOP'),
    (1 << 1, 'SENSOR_FAIL'),
    (1 << 2, 'VOLTAGE_LOW'),
    (1 << 3, 'COMM_TIMEOUT'),
]


class HwManagerNode(Node):
    """Hardware manager: heartbeat keeper + state monitor."""

    # ── Startup phase enum ───────────────────────────────────────
    PHASE_INIT = 'INIT'
    PHASE_WAITING_INS = 'WAITING_INS'
    PHASE_INS_READY = 'INS_READY'
    PHASE_ARMED = 'ARMED'
    PHASE_RUNNING = 'RUNNING'
    PHASE_DEGRADED = 'DEGRADED'

    def __init__(self):
        super().__init__('hw_manager')

        # ── Parameters ───────────────────────────────────────────
        self.declare_parameter('heartbeat_rate', 10.0)
        self.declare_parameter('watchdog_timeout', 7.0)
        self.declare_parameter('arm_mode', 1)  # 1=normal, 3=force
        self.declare_parameter('battery_low_threshold', 14.0)
        self.declare_parameter('cycle_time_warn_threshold', 100.0)

        # ── Internal state ───────────────────────────────────────
        self._status_lock = threading.Lock()
        self._mcu_status = ZitStatus()
        self._last_mcu_hb_seq = 0
        self._last_mcu_hb_time = self.get_clock().now()
        self._last_status_time = self.get_clock().now()
        self._last_thr_time = self.get_clock().now()
        self._last_thr_forces = [0.0] * 6
        self._startup_phase = self.PHASE_INIT
        self._prev_armed = False

        # ── Heartbeat publisher ──────────────────────────────────
        self._heartbeat_pub = self.create_publisher(
            UInt32, '/zit6/cmd/agxhbt', 10)
        hb_rate = self.get_parameter('heartbeat_rate').value
        self._hb_timer = self.create_timer(1.0 / hb_rate, self._heartbeat_cb)

        # ── MCU state subscriptions ──────────────────────────────
        self._status_sub = self.create_subscription(
            ZitStatus, '/zit6/state/status', self._status_cb, 10)
        self._mcu_hb_sub = self.create_subscription(
            UInt32, '/zit6/state/zithbt', self._mcu_hb_cb, 10)
        self._thr_sub = self.create_subscription(
            Float32MultiArray, '/zit6/state/thr', self._thr_cb, 10)

        # ── Timers ───────────────────────────────────────────────
        self._summary_timer = self.create_timer(1.0, self._summary_cb)

        self.get_logger().info('HW Manager started')
        self.get_logger().info(
            f'  heartbeat_rate={hb_rate} Hz, '
            f'arm_mode={self.get_parameter("arm_mode").value}')
        self.get_logger().info(
            f'  watchdog_timeout='
            f'{self.get_parameter("watchdog_timeout").value}s')
        self._startup_phase = self.PHASE_WAITING_INS

    # ── Heartbeat ────────────────────────────────────────────────

    def _heartbeat_cb(self):
        """10Hz: send heartbeat to MCU."""
        msg = UInt32()
        arm_mode = self.get_parameter('arm_mode').value
        msg.data = arm_mode
        self._heartbeat_pub.publish(msg)

    # ── State callbacks ──────────────────────────────────────────

    def _status_cb(self, msg: ZitStatus):
        """Receive ZitStatus from MCU (10Hz). Run startup state machine."""
        with self._status_lock:
            old_phase = self._startup_phase
            self._mcu_status = msg
            self._last_status_time = self.get_clock().now()

            # Detect unexpected disarm
            if self._prev_armed and not msg.is_armed:
                self.get_logger().error(
                    'MCU unexpectedly DISARMED! '
                    f'(was armed, now locked, error_flags=0x{msg.error_flags:08X})')
            self._prev_armed = msg.is_armed

            # ── Startup state machine ──
            if self._startup_phase == self.PHASE_WAITING_INS:
                if msg.navigation_ready:
                    self._startup_phase = self.PHASE_INS_READY
                    ins_name = _INS_NAMES.get(msg.ins_state, f'?({msg.ins_state})')
                    self.get_logger().info(
                        f'INS navigation ready! state={ins_name}')
            elif self._startup_phase == self.PHASE_INS_READY:
                if msg.is_armed:
                    self._startup_phase = self.PHASE_ARMED
                    self.get_logger().info(
                        f'MCU ARMED! arm_mode={msg.arm_mode}')
            elif self._startup_phase == self.PHASE_ARMED:
                if msg.control_level > 0:
                    self._startup_phase = self.PHASE_RUNNING
                    ctrl_name = _CTRL_NAMES.get(msg.control_level,
                                                f'?({msg.control_level})')
                    self.get_logger().info(
                        f'MCU RUNNING: control_level={ctrl_name}')

    def _mcu_hb_cb(self, msg: UInt32):
        """Receive MCU heartbeat sequence number (1Hz)."""
        with self._status_lock:
            old_seq = self._last_mcu_hb_seq
            self._last_mcu_hb_seq = msg.data
            self._last_mcu_hb_time = self.get_clock().now()

        if old_seq != 0 and msg.data == old_seq:
            # Sequence stalled — MCU may be hung
            self.get_logger().warn(
                f'MCU heartbeat seq stalled at {msg.data}',
                throttle_duration_sec=10.0)

    def _thr_cb(self, msg: Float32MultiArray):
        """Monitor thruster forces (30Hz). Warn on saturation."""
        if len(msg.data) >= 6:
            self._last_thr_forces = list(msg.data[:6])
            self._last_thr_time = self.get_clock().now()
            max_thrust = max(abs(v) for v in msg.data[:6])
            if max_thrust > 0.95:
                t_str = ', '.join(f'{t:.2f}' for t in msg.data[:6])
                self.get_logger().warn(
                    f'Thruster saturation: max={max_thrust:.3f} '
                    f'thrusts=[{t_str}]',
                    throttle_duration_sec=5.0)

    # ── Periodic timers ──────────────────────────────────────────

    def _summary_cb(self):
        """1Hz: unified status summary + watchdog checks."""
        now = self.get_clock().now()
        timeout = self.get_parameter('watchdog_timeout').value

        with self._status_lock:
            s = self._mcu_status
            hb_seq = self._last_mcu_hb_seq
            last_hb = self._last_mcu_hb_time
            last_status = self._last_status_time
            thr_forces = list(self._last_thr_forces)

        # ── Watchdog checks (inline, event-driven) ────────────
        hb_elapsed = (now - last_hb).nanoseconds / 1e9
        status_elapsed = (now - last_status).nanoseconds / 1e9

        if hb_elapsed > timeout:
            self.get_logger().error(
                f'MCU heartbeat lost: {hb_elapsed:.0f}s (timeout={timeout}s)',
                throttle_duration_sec=5.0)
            self._startup_phase = self.PHASE_DEGRADED

        if status_elapsed > timeout:
            self.get_logger().error(
                f'MCU status lost: {status_elapsed:.0f}s',
                throttle_duration_sec=5.0)

        # ── Battery warning ───────────────────────────────────
        low_thresh = self.get_parameter('battery_low_threshold').value
        if 0.1 < s.battery_voltage < low_thresh:
            self.get_logger().warn(
                f'Battery low: {s.battery_voltage:.1f}V < {low_thresh}V',
                throttle_duration_sec=30.0)

        # ── Error flags ────────────────────────────────────────
        if s.error_flags != 0:
            err_names = [name for mask, name in _ERROR_BITS if s.error_flags & mask]
            self.get_logger().error(
                f'MCU errors: 0x{s.error_flags:08X} → {", ".join(err_names)}',
                throttle_duration_sec=5.0)

        # ── Build one-line summary ─────────────────────────────
        armed = '●' if s.is_armed else '○'
        nav_ok = '✓' if s.navigation_ready else '✗'
        ins = _INS_NAMES.get(s.ins_state, f'?{s.ins_state}')
        ctrl = _CTRL_NAMES.get(s.control_level, f'?{s.control_level}')
        phase = self._startup_phase

        # Forces: only show non-zero
        force_parts = []
        labels = ['Fx', 'Fy', 'Fz', 'Mz']
        for i, f in enumerate(thr_forces[:4]):
            if abs(f) > 0.01:
                force_parts.append(f'{labels[i]}={f:+.1f}')
        forces = ' '.join(force_parts) if force_parts else 'idle'

        self.get_logger().info(
            f'ARM:{armed} {phase:12s} | '
            f'INS:{ins:14s} NAV:{nav_ok} | '
            f'CTRL:{ctrl:4s} | '
            f'BAT:{s.battery_voltage:4.1f}V | '
            f'CYC:{s.cycle_time_ms:5.1f}ms | '
            f'SEQ:{hb_seq:4d} | '
            f'{forces}')


def main(args=None):
    rclpy.init(args=args)
    node = HwManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('HW Manager shutting down (KeyboardInterrupt)')
    except Exception as e:
        node.get_logger().fatal(f'HW Manager crashed: {e}')
        import traceback
        traceback.print_exc()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
