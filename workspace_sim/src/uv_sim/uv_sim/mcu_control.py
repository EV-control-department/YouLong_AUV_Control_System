"""Simulated ZIT6 MCU: setpoint state machine, cascaded PID, ARM/INS state.

This is a **ROS-free** port of the maths that used to live in ``sim_bridge``
(T5).  Everything that was ``self.<something>`` on the node and did not touch
rclpy moved here verbatim, so it can be driven deterministically from a unit test
(``test_mcu_control.py`` replays ``test/data/thrust_golden.json``, recorded from
the pre-split monolith).

The split step (T5-2) gives each piece of ROS glue its own node: ``mcu_sim`` owns
this controller and the ``/zit6`` interface, while ``nav_sim`` owns the state
topics.  Until then ``sim_bridge`` delegates to this object.

Conventions this class **must** keep, because the golden fixture pins them:

* ``pos``/``vel`` store angles in **degrees** (``vel['rz']`` is deg/s and the
  yaw-rate loop compares it against a deg/s target).  ROS messages convert to
  radians at the publish boundary only (invariant I5).
* ``force_4dof`` is ``[Fx, Fy, Fz, Mz]`` in the units of the tuned gains.
* Mode dispatch is ``control_key & 0x03``; a value of 3 matches no branch and
  therefore leaves the previous mode running (pinned as
  ``mode_3_keeps_previous_mode``, not "fixed" here).
"""

from __future__ import annotations

import math
from typing import Optional

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from uv_sim.mathutil import clamp, wrap_angle_deg
from uv_sim.odometry import body_twist_from_odometry
from uv_sim.pid import Pid
from uv_sim.pid_config import build_pids

#: Coarse-alignment and SINS/DVL alignment durations of the simulated INS.
INS_COARSE_ALIGN_SECONDS = 1.5
INS_DVL_ALIGN_SECONDS = 1.5

#: Yaw-rate ceiling of the position cascade's outer loop (deg/s).
MAX_YAW_RATE_DEG = 45.0

#: Modes as reported in ``/zit6/state/status.control_level``.
MODE_NONE = 0
MODE_POSITION = 1
MODE_VELOCITY = 2
MODE_FORCE = 3


class McuController:
    """Vehicle state plus the setpoint/PID logic a ZIT6 MCU would run."""

    def __init__(self, pid_params: dict, start_time: float = 0.0) -> None:
        # Vehicle state as last measured (NED degrees for angles).
        self.pos = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'rx': 0.0, 'ry': 0.0, 'rz': 0.0}
        self.vel = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'rx': 0.0, 'ry': 0.0, 'rz': 0.0}
        self.target_pos = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'rz': 0.0}
        self.target_vel = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'rz': 0.0}
        self.force_4dof = [0.0, 0.0, 0.0, 0.0]

        self.current_mode = MODE_NONE
        self.position_ctrl_enabled = False
        self.vel_ctrl_enabled = False
        self.current_pose_ready = False
        self.last_pid_time = start_time

        # ARM / heartbeat state (always armed; the heartbeat timeout is disabled)
        self.armed = True
        self.arm_mode = 3
        self._hb_seq = 0

        # INS state: 0=待机 1=粗对准 2=精对准 3=SINS/GPS/DVL 4=SINS/DVL 5=MRU
        self.ins_state = 1            # start in coarse alignment
        self.ins_nav_ready = False    # not ready until coarse done
        self.dvl_enabled = False      # DVL starts off
        self._ins_boot_time = start_time
        self._ins_align_request: Optional[float] = None

        # PID gains: the file, the /zit6/get_params view and the live controllers
        # are all built from this one dict (see pid_config.build_pids).
        self.pid_params = pid_params
        self.pids: dict[str, Pid] = build_pids(pid_params)

    # ── sensor input ────────────────────────────────────────────────────────

    def apply_odometry(self, msg: Odometry) -> None:
        """Take position/orientation from ``/auv/odometry`` and mark pose ready."""
        self.pos['x'] = msg.pose.pose.position.x
        self.pos['y'] = msg.pose.pose.position.y
        self.pos['z'] = msg.pose.pose.position.z

        q = msg.pose.pose.orientation
        roll, pitch, yaw = quat_to_rpy(q.x, q.y, q.z, q.w)
        self.pos['rx'] = math.degrees(roll)
        self.pos['ry'] = math.degrees(pitch)
        self.pos['rz'] = math.degrees(yaw)

        # Twist is already body-frame -- no yaw rotation here (see
        # body_twist_from_odometry).  The DVL used to overwrite x/y/z with its own
        # measurement (bug B19); it no longer does, so this channel has exactly one
        # meaning: the velocity of the body origin.
        u, v, w, p, q_rate, r = body_twist_from_odometry(msg)
        self.vel['x'] = u
        self.vel['y'] = v
        self.vel['z'] = w
        self.vel['rx'] = math.degrees(p)
        self.vel['ry'] = math.degrees(q_rate)
        self.vel['rz'] = math.degrees(r)

        self.current_pose_ready = True

    def apply_imu(self, msg: Imu) -> None:
        """Refresh the body rates from the 200 Hz IMU.

        Benign second writer for ``vel['rx'..'rz']``: both sources carry the same
        physical quantity (body angular velocity), and the IMU is the faster and
        intended one.  Unlike the old DVL/odometry conflict this cannot differ in
        meaning, which is why it is kept rather than removed.
        """
        self.vel['rx'] = math.degrees(msg.angular_velocity.x)
        self.vel['ry'] = math.degrees(msg.angular_velocity.y)
        self.vel['rz'] = math.degrees(msg.angular_velocity.z)

    # ── setpoint state machine ──────────────────────────────────────────────

    def apply_setpoint(
        self,
        control_key: int,
        type_mask: int,
        x: float,
        y: float,
        z: float,
        yaw: float,
        now: float,
    ) -> Optional[list[float]]:
        """Apply a ``ZitSetpoint``; return the 4-DOF force when it must be sent now.

        Force mode (``mode == 2``) produces output immediately, so the caller
        publishes the returned list right away.  Position/velocity modes just
        retarget and reset their loops; the caller drives them from its control
        timer.  ``yaw`` is in **radians** (the existing protocol).
        """
        mode = control_key & 0x03
        frame = control_key & 0x10        # 0x10 = body
        incremental = control_key & 0x20  # 0x20 = incremental

        if mode == 2:
            # Force mode: bit=1 = keep current, bit=0 = apply
            self.position_ctrl_enabled = False
            self.vel_ctrl_enabled = False
            self.current_mode = MODE_FORCE
            self.force_4dof = [
                self.force_4dof[0] if (type_mask & 0x01) else x,
                self.force_4dof[1] if (type_mask & 0x02) else y,
                self.force_4dof[2] if (type_mask & 0x04) else z,
                self.force_4dof[3] if (type_mask & 0x08) else yaw,
            ]
            return list(self.force_4dof)

        if mode == 1:
            # Velocity mode: bit=1 = ignore, bit=0 = apply
            self.position_ctrl_enabled = False
            self.vel_ctrl_enabled = True
            self.current_mode = MODE_VELOCITY
            if not (type_mask & 0x01):
                self.target_vel['x'] = x
            if not (type_mask & 0x02):
                self.target_vel['y'] = y
            if not (type_mask & 0x04):
                self.target_vel['z'] = z
            if not (type_mask & 0x08):
                self.target_vel['rz'] = math.degrees(yaw)
            self.last_pid_time = now
            for path in ('chassis.pid.vel.x', 'chassis.pid.vel.y',
                         'chassis.pid.vel.z', 'chassis.pid.vel.yaw'):
                self.pids[path].reset()
            return None

        if mode == 0:
            # Position mode: bit=1 = ignore, bit=0 = apply
            self.vel_ctrl_enabled = False
            self.current_mode = MODE_POSITION
            yaw_deg = math.degrees(yaw)

            if incremental:
                dx = 0.0 if (type_mask & 0x01) else x
                dy = 0.0 if (type_mask & 0x02) else y
                yaw_rad = math.radians(self.pos['rz'])
                cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
                self.target_pos['x'] += cy * dx - sy * dy
                self.target_pos['y'] += sy * dx + cy * dy
                if not (type_mask & 0x04):
                    self.target_pos['z'] += z
                if not (type_mask & 0x08):
                    self.target_pos['rz'] += yaw_deg
            else:
                if frame:
                    # Absolute body -> world
                    yaw_rad = math.radians(self.pos['rz'])
                    cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
                    if not (type_mask & 0x01):
                        self.target_pos['x'] = self.pos['x'] + cy * x - sy * y
                    if not (type_mask & 0x02):
                        self.target_pos['y'] = self.pos['y'] + sy * x + cy * y
                else:
                    # Absolute world
                    if not (type_mask & 0x01):
                        self.target_pos['x'] = x
                    if not (type_mask & 0x02):
                        self.target_pos['y'] = y
                if not (type_mask & 0x04):
                    self.target_pos['z'] = z
                if not (type_mask & 0x08):
                    self.target_pos['rz'] = yaw_deg

            self.position_ctrl_enabled = True
            self.last_pid_time = now
            for path in ('chassis.pid.pos.x', 'chassis.pid.pos.y', 'chassis.pid.pos.z',
                         'chassis.pid.pos.yaw', 'chassis.pid.pos.yaw_rate'):
                self.pids[path].reset()
            return None

        # control_key & 0x03 == 3: no branch matched.  The previous mode, its
        # targets and its enabled flag all survive, so the loop keeps running.
        return None

    # ── control step ────────────────────────────────────────────────────────

    def compute_dt(self, now: float) -> float:
        """Seconds since the previous call, clamped to a sane control period."""
        dt = now - self.last_pid_time
        self.last_pid_time = now
        return clamp(dt, 0.001, 0.2)

    def step_position(self, dt: float) -> list[float]:
        """Position mode: x/y/z single loops + yaw two-level cascade.

        Returns the 4-DOF body force ``[Fx, Fy, Fz, Mz]`` for the caller to mix.
        """
        x, y, z, yaw = self.pos['x'], self.pos['y'], self.pos['z'], self.pos['rz']

        ex_w = self.target_pos['x'] - x
        ey_w = self.target_pos['y'] - y

        yaw_rad = math.radians(yaw)
        cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
        ex_body = cy * ex_w + sy * ey_w
        ey_body = -sy * ex_w + cy * ey_w

        ez = self.target_pos['z'] - z
        eyaw = wrap_angle_deg(self.target_pos['rz'] - yaw)

        target_yaw_rate = self.pids['chassis.pid.pos.yaw'].step(eyaw, dt)
        target_yaw_rate = clamp(target_yaw_rate, -MAX_YAW_RATE_DEG, MAX_YAW_RATE_DEG)
        eyaw_rate = target_yaw_rate - self.vel['rz']

        cmd_x = float(self.pids['chassis.pid.pos.x'].step(ex_body, dt))
        cmd_y = float(self.pids['chassis.pid.pos.y'].step(ey_body, dt))
        cmd_z = float(self.pids['chassis.pid.pos.z'].step(ez, dt))
        cmd_rz = float(self.pids['chassis.pid.pos.yaw_rate'].step(eyaw_rate, dt))
        return [cmd_x, cmd_y, cmd_z, cmd_rz]

    def step_velocity(self, dt: float) -> list[float]:
        """Velocity mode: four independent single loops."""
        evx = self.target_vel['x'] - self.vel['x']
        evy = self.target_vel['y'] - self.vel['y']
        evz = self.target_vel['z'] - self.vel['z']
        evyaw = self.target_vel['rz'] - self.vel['rz']

        cmd_x = float(self.pids['chassis.pid.vel.x'].step(evx, dt))
        cmd_y = float(self.pids['chassis.pid.vel.y'].step(evy, dt))
        cmd_z = float(self.pids['chassis.pid.vel.z'].step(evz, dt))
        cmd_rz = float(self.pids['chassis.pid.vel.yaw'].step(evyaw, dt))
        return [cmd_x, cmd_y, cmd_z, cmd_rz]

    def step(self, dt: float) -> Optional[list[float]]:
        """Run whichever loop is enabled; ``None`` when neither is."""
        if self.position_ctrl_enabled:
            return self.step_position(dt)
        if self.vel_ctrl_enabled:
            return self.step_velocity(dt)
        return None

    # ── INS state machine ───────────────────────────────────────────────────

    def update_ins_alignment(self, now: float) -> list[str]:
        """Advance the INS alignment state machine; return log lines."""
        messages: list[str] = []

        # Phase 1: coarse alignment (1 -> 5, ~1.5 s after boot)
        if self.ins_state == 1:
            if now - self._ins_boot_time >= INS_COARSE_ALIGN_SECONDS:
                self.ins_state = 5
                self.ins_nav_ready = True
                messages.append("INS: coarse alignment done → MRU mode")
                # If DVL was already requested while in state 1, start alignment now
                if self.dvl_enabled:
                    self._ins_align_request = now

        # Phase 2: SINS/DVL alignment (5 -> 4, ~1.5 s after request)
        if self.ins_state == 5 and self._ins_align_request is not None:
            if now - self._ins_align_request >= INS_DVL_ALIGN_SECONDS:
                self.ins_state = 4
                self._ins_align_request = None
                messages.append("INS: SINS/DVL alignment complete")

        return messages

    def ins_command(self, command: int, now: float) -> list[str]:
        """Handle ``/zit6/cmd/ins``: 1=enable DVL 2=disable 3=restart alignment."""
        messages: list[str] = []
        if command == 1:
            self.dvl_enabled = True
            messages.append("INS: DVL enabled, requesting SINS/DVL alignment")
            if self.ins_state == 5:
                self._ins_align_request = now
            elif self.ins_state == 1:
                messages.append("INS: waiting for coarse alignment first")
        elif command == 2:
            self.dvl_enabled = False
            self._ins_align_request = None
            messages.append("INS: DVL disabled, fallback to MRU")
            if self.ins_state == 4:
                self.ins_state = 5
        elif command == 3:
            messages.append("INS: restarting alignment...")
            self.ins_state = 1
            self.ins_nav_ready = False
            self.dvl_enabled = False
            self._ins_align_request = None
            self._ins_boot_time = now
        return messages

    # ── ARM / heartbeat ─────────────────────────────────────────────────────

    def note_heartbeat(self, arm_value: int) -> list[str]:
        """Handle ``/zit6/cmd/agxhbt``: 3=force arm, 1=normal arm (INS must be ready).

        The heartbeat *timeout* is deliberately not implemented (the vehicle stays
        armed); only the arm request is honoured.
        """
        messages: list[str] = []
        if arm_value == 3:
            self.arm_mode = 3
            if not self.armed:
                messages.append("ARM: force arm (mode 3)")
            self.armed = True
        elif arm_value == 1 and self.ins_nav_ready:
            self.arm_mode = 0
            if not self.armed:
                messages.append("ARM: normal arm (mode 0)")
            self.armed = True
        return messages

    def next_heartbeat_seq(self) -> int:
        """Return the next ``/zit6/state/zithbt`` sequence number."""
        self._hb_seq += 1
        return self._hb_seq

    # ── parameter service ───────────────────────────────────────────────────

    def get_params(self, paths: Optional[list[str]] = None) -> dict:
        """Gains for ``paths`` (all channels when ``paths`` is empty)."""
        if not paths:
            return dict(self.pid_params)
        result = {}
        for path in paths:
            value = self.pid_params.get(path)
            if value is not None:
                result[path] = value
        return result

    def apply_param_updates(self, updates: dict) -> list[str]:
        """Write ``{path: gains-or-kp}`` through to the live PID objects.

        Raises whatever ``float()`` raises for a bad value; the service wrapper
        turns that into an unsuccessful response.  Only the running process is
        touched -- the YAML file is never rewritten.
        """
        messages: list[str] = []
        for path, value in updates.items():
            if path not in self.pid_params:
                continue
            gains = self.pid_params[path]
            if isinstance(value, dict):
                # Full gain dict: {"kp": 800, "ki": 130.0, ...}
                for key, item in value.items():
                    if key in gains:
                        gains[key] = float(item)
            elif isinstance(value, (int, float)):
                # Single value: interpreted as kp
                gains['kp'] = float(value)

            pid = self.pids.get(path)
            if pid is not None:
                pid.kp = gains['kp']
                pid.ki = gains.get('ki', 0.0)
                pid.kd = gains.get('kd', 0.0)
                pid.i_limit = gains.get('i_limit', 0.5)
                if 'z_thrust_ff' in gains:
                    pid.z_thrust_ff = gains['z_thrust_ff']
            messages.append(f"Params: {path} → {gains}")
        return messages


def quat_to_rpy(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    """Roll/pitch/yaw (radians) from a quaternion; the usual ZYX extraction."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.asin(clamp(sinp, -1.0, 1.0))

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw
