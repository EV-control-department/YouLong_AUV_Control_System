#!/usr/bin/env python3
"""T5-0: record the **pre-split** thrust golden fixture (A5-4 numerical regression).

Why this exists
---------------
T5 splits the 919-line ``sim_bridge`` monolith into seven nodes and, on the way,
moves the control math into a ROS-free core (``uv_sim.mcu_control``) plus a
ROS-free mixer (``uv_sim.mixer``).  A refactor of live control code is only
trustworthy if it can be *proven* to be behaviour preserving, so this script
captures the exact 4-DOF force and 6-thruster output of the **unmodified**
implementation, and ``test_mcu_control.py`` replays that fixture against the new
code.

Run it **before** touching ``uv_sim/sim_bridge.py``::

    export ROS_HOME=$PWD/.ros_tmp ROS_LOG_DIR=$PWD/.ros_tmp/log
    source /opt/ros/jazzy/setup.bash
    source workspace_auv/install/setup.bash
    source workspace_sim/install/setup.bash
    python3 workspace_sim/tools/record_thrust_golden.py

Two design choices keep the fixture meaningful for years:

1. **The gains are recorded, not re-read.**  The fixture embeds the exact
   ``pid_params`` dict the node ran with, and the replay test builds the
   controller from *those* numbers.  Tuning ``config/sim_pid.yaml`` therefore
   never invalidates the golden -- it is a pure math regression.  (A test that
   fails on every tuning edit gets deleted rather than obeyed; same reasoning as
   ``test_frames_geometry.test_pid_builtin_fallback_is_structurally_complete``.)
2. **The vehicle truth is recorded per step.**  Poses/velocities are written into
   the fixture, so the replay feeds byte-identical sensor messages instead of
   re-running a formula that could drift.

Only ``/auv/odometry`` and ``/auv/imu`` feed the controller here (never the DVL):
before T5 the DVL callback silently overwrote the same ``vel`` fields (B19), and
a fixture that depends on which of the two writers landed last would be flaky.

Exit codes: 0 written, 2 environment not ready.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

DT = 0.02
POSITION_STEPS = 25
VELOCITY_STEPS = 15

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_OUT = (
    REPO_ROOT / "workspace_sim" / "src" / "uv_sim" / "test" / "data" / "thrust_golden.json"
)


# --------------------------------------------------------------------------- #
# Deterministic vehicle truth
# --------------------------------------------------------------------------- #
def truth_for(step: int) -> dict:
    """A small deterministic trajectory: means and rates sweep gently.

    Values are recorded verbatim into the fixture, so this function only has to
    be *stable*, not clever.
    """
    return {
        "x": 0.05 * step,
        "y": -0.02 * step,
        "z": 0.01 * step,
        "roll": math.radians(2.0),
        "pitch": math.radians(-1.0),
        "yaw": math.radians(3.0 * step),
        "u": 0.30 + 0.01 * step,
        "v": -0.05,
        "w": 0.02,
        "p": 0.01,
        "q": -0.02,
        "r": 0.15 - 0.005 * step,
    }


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """Return ``(x, y, z, w)`` for ``R = Rz(yaw) @ Ry(pitch) @ Rx(roll)``.

    Same euler order the scene files use (see ``uv_sim.geometry.euler_rpy_to_matrix``),
    written out here so this tool stays a single self-contained file.
    """
    half_roll, half_pitch, half_yaw = roll / 2.0, pitch / 2.0, yaw / 2.0
    cr, sr = math.cos(half_roll), math.sin(half_roll)
    cp, sp = math.cos(half_pitch), math.sin(half_pitch)
    cy, sy = math.cos(half_yaw), math.sin(half_yaw)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def odometry_from_truth(truth: dict):
    """Build the ``nav_msgs/Odometry`` the SIL stack feeds the controller."""
    from nav_msgs.msg import Odometry

    message = Odometry()
    message.header.frame_id = "world_ned"
    message.child_frame_id = "dynamics"
    message.pose.pose.position.x = truth["x"]
    message.pose.pose.position.y = truth["y"]
    message.pose.pose.position.z = truth["z"]
    qx, qy, qz, qw = quaternion_from_rpy(truth["roll"], truth["pitch"], truth["yaw"])
    message.pose.pose.orientation.x = qx
    message.pose.pose.orientation.y = qy
    message.pose.pose.orientation.z = qz
    message.pose.pose.orientation.w = qw
    # Twist is body-frame at the origin (see body_twist_from_odometry).
    message.twist.twist.linear.x = truth["u"]
    message.twist.twist.linear.y = truth["v"]
    message.twist.twist.linear.z = truth["w"]
    message.twist.twist.angular.x = truth["p"]
    message.twist.twist.angular.y = truth["q"]
    message.twist.twist.angular.z = truth["r"]
    return message


def imu_from_truth(truth: dict):
    """Build the ``sensor_msgs/Imu`` the controller takes its rates from."""
    from sensor_msgs.msg import Imu

    message = Imu()
    message.header.frame_id = "imu_filter"
    message.angular_velocity.x = truth["p"]
    message.angular_velocity.y = truth["q"]
    message.angular_velocity.z = truth["r"]
    return message


# --------------------------------------------------------------------------- #
# Case definitions
# --------------------------------------------------------------------------- #
def _setpoint(**fields):
    from zit6_interfaces.msg import ZitSetpoint

    message = ZitSetpoint()
    message.control_key = fields.get("control_key", 0)
    message.type_mask = fields.get("type_mask", 0)
    message.x = fields.get("x", 0.0)
    message.y = fields.get("y", 0.0)
    message.z = fields.get("z", 0.0)
    message.yaw = fields.get("yaw", 0.0)
    return message


#: ``(name, setpoint fields, steps, which PID step to drive)``.
#: ``control_key & 0x03`` picks the mode: 0=position, 1=velocity, 2=force, 3=none.
SETPOINT_CASES = (
    (
        "pos_world_absolute",
        {"control_key": 0x00, "type_mask": 0x00, "x": 1.5, "y": -0.5, "z": 2.0,
         "yaw": math.radians(45.0)},
        POSITION_STEPS,
        "position",
    ),
    (
        "pos_body_absolute",
        {"control_key": 0x10, "type_mask": 0x00, "x": 0.8, "y": 0.4, "z": 1.0,
         "yaw": math.radians(-30.0)},
        POSITION_STEPS,
        "position",
    ),
    (
        "pos_incremental",
        {"control_key": 0x20, "type_mask": 0x00, "x": 0.6, "y": -0.3, "z": 0.5,
         "yaw": math.radians(15.0)},
        POSITION_STEPS,
        "position",
    ),
    (
        "pos_masked_y_yaw_only",
        {"control_key": 0x00, "type_mask": 0x05, "x": 9.0, "y": 0.7, "z": 9.0,
         "yaw": math.radians(20.0)},
        POSITION_STEPS,
        "position",
    ),
    (
        "vel_world",
        {"control_key": 0x01, "type_mask": 0x00, "x": 0.35, "y": -0.10, "z": 0.05,
         "yaw": math.radians(10.0)},
        VELOCITY_STEPS,
        "velocity",
    ),
    (
        "vel_masked_y_yaw",
        {"control_key": 0x01, "type_mask": 0x0A, "x": 0.20, "y": 9.0, "z": -0.15,
         "yaw": math.radians(99.0)},
        VELOCITY_STEPS,
        "velocity",
    ),
    (
        # control_key & 0x03 == 3 matches no branch, so the previous mode and its
        # targets survive; the PID keeps running.  Pinned because a future
        # "reject unknown modes" change would be a behaviour change, not a fix.
        "mode_3_keeps_previous_mode",
        {"control_key": 0x03, "type_mask": 0x00, "x": 5.0, "y": 5.0, "z": 5.0,
         "yaw": math.radians(5.0)},
        3,
        "position",
    ),
)

#: Immediate-output cases: each row is a setpoint whose effect is visible without
#: a PID step (force mode publishes straight away; the "keep current" mask must
#: hold the previous force).  Driven on a controller already in force mode.
FORCE_CASES = (
    ("force_apply_all", {"control_key": 0x02, "type_mask": 0x00, "x": 300.0,
                         "y": -150.0, "z": 420.0, "yaw": 90.0}),
    ("force_keep_all", {"control_key": 0x02, "type_mask": 0x0F, "x": 0.0,
                        "y": 0.0, "z": 0.0, "yaw": 0.0}),
    ("force_apply_x_only", {"control_key": 0x02, "type_mask": 0x0E, "x": -500.0,
                            "y": 0.0, "z": 0.0, "yaw": 0.0}),
)

#: Direct mixer probes (4-DOF force in, 6 thrusters out) for both maximum-thrust
#: scalings.  SIL saturates at +/-1 after dividing by 1000; HIL divides by 1.0,
#: because the MCU hands over normalised forces there.
MIXING_CASES = (
    ("sil_typical", False, (600.0, -200.0, 300.0, 150.0)),
    ("sil_saturating", False, (2500.0, -2500.0, 4000.0, -3000.0)),
    ("sil_yaw_only", False, (0.0, 0.0, 0.0, 1000.0)),
    ("sil_heave_scale", False, (0.0, 0.0, 1250.0, 0.0)),
    ("hil_typical", True, (0.6, -0.2, 0.3, 0.15)),
    ("hil_partial_saturation", True, (0.9, -0.9, 1.2, -0.5)),
)


def _load_sim_bridge():
    """Import ``uv_sim.sim_bridge``; exit 2 with a hint when the env is not set up."""
    try:
        import uv_sim.sim_bridge as sim_bridge
    except ImportError as error:  # pragma: no cover - environment dependent
        print(
            f"cannot import uv_sim.sim_bridge ({error}); source both workspaces "
            "(workspace_auv then workspace_sim) first",
            file=sys.stderr,
        )
        raise SystemExit(2)
    return sim_bridge


def record() -> dict:
    import rclpy
    from rclpy.parameter import Parameter

    sim_bridge = _load_sim_bridge()

    rclpy.init()
    node = None
    hil_node = None
    try:
        config_path = sim_bridge.resolve_pid_config_path()
        if config_path is None:  # pragma: no cover - shipped file always resolves
            print("no sim_pid.yaml resolvable; refusing to record", file=sys.stderr)
            raise SystemExit(2)
        node = sim_bridge.SimBridgeNode(
            node_name="thrust_golden_recorder",
            parameter_overrides=[Parameter("pid_config_file", value=str(config_path))],
        )
        hil_node = sim_bridge.SimBridgeNode(
            node_name="thrust_golden_hil_recorder",
            parameter_overrides=[
                Parameter("pid_config_file", value=str(config_path)),
                Parameter("hil_mode", value=True),
            ],
        )

        cases = []
        for name, fields, steps, step_kind in SETPOINT_CASES:
            node.pos.update({"x": 0.0, "y": 0.0, "z": 0.0, "rz": 0.0})
            node.vel.update({"x": 0.0, "y": 0.0, "z": 0.0, "rx": 0.0, "ry": 0.0, "rz": 0.0})
            node._setpoint_cb(_setpoint(**fields))

            recorded_steps = []
            for step in range(steps):
                truth = truth_for(step)
                # Sensor path first: the PID step consumes self.pos/self.vel.
                node._odom_cb(odometry_from_truth(truth))
                node._imu_cb(imu_from_truth(truth))
                if step_kind == "position":
                    node._position_pid_step(DT)
                else:
                    node._velocity_pid_step(DT)
                recorded_steps.append({
                    "truth": truth,
                    "force_4dof": list(node.force_4dof),
                    "thrust": list(node.thrust),
                })
            cases.append({
                "name": name,
                "setpoint": fields,
                "step_kind": step_kind,
                "steps": recorded_steps,
            })

        force_cases = []
        node._setpoint_cb(_setpoint(**FORCE_CASES[0][1]))
        for name, fields in FORCE_CASES:
            node._setpoint_cb(_setpoint(**fields))
            force_cases.append({
                "name": name,
                "setpoint": fields,
                "force_4dof": list(node.force_4dof),
                "thrust": list(node.thrust),
            })

        mixing_cases = []
        for name, hil, values in MIXING_CASES:
            target = hil_node if hil else node
            target._publish_thrust_from_4dof(*values)
            mixing_cases.append({
                "name": name,
                "hil": hil,
                "input_4dof": list(values),
                "thrust": list(target.thrust),
            })

        return {
            "generated_by": (
                "workspace_sim/tools/record_thrust_golden.py "
                "(uv_sim.sim_bridge.SimBridgeNode, pre-split monolithic implementation)"
            ),
            "note": (
                "Replayed by workspace_sim/src/uv_sim/test/test_mcu_control.py. "
                "Gains are embedded so tuning sim_pid.yaml cannot invalidate it."
            ),
            "dt": DT,
            "pid_config_file": str(config_path),
            "pid_params": {channel: dict(gains) for channel, gains in node._pid_params.items()},
            "setpoint_cases": cases,
            "force_cases": force_cases,
            "mixing_cases": mixing_cases,
        }
    finally:
        for active in (node, hil_node):
            if active is not None:
                active.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out", default=str(DEFAULT_OUT),
                        help=f"fixture to write (default: {DEFAULT_OUT})")
    args = parser.parse_args(argv)

    payload = record()
    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    total = sum(len(case["steps"]) for case in payload["setpoint_cases"])
    print(f"wrote {out_path}")
    print(f"  {len(payload['setpoint_cases'])} setpoint cases / {total} PID steps, "
          f"{len(payload['force_cases'])} immediate cases, "
          f"{len(payload['mixing_cases'])} mixer probes, "
          f"{len(payload['pid_params'])} PID channels")
    return 0


if __name__ == "__main__":
    sys.exit(main())
