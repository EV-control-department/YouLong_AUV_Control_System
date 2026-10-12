"""T5/A5-4: the extracted control core must reproduce the pre-split monolith.

``test/data/thrust_golden.json`` was recorded from the **unmodified**
``sim_bridge.SimBridgeNode`` before any of T5's refactoring, by a one-shot tool
(``tools/record_thrust_golden.py``) that was deleted together with the monolith:
its output is the fixture, and *this file* is the permanent verifier -- it names the
failing case and step index, which is what a regeneration tool would print anyway.
Re-recording is therefore a deliberate act (see devlog section 9) rather than a
command someone can run by reflex whenever the maths changes.

Two deliberate properties of the fixture keep this test honest:

* the gains are **inside** the fixture, so tuning ``config/sim_pid.yaml`` never
  fails this test -- it measures the maths, not the tuning;
* the vehicle truth of every step is inside the fixture too, so the replay feeds
  the same sensor messages instead of re-deriving them.

The setpoint cases **must be replayed in order on one controller** with only
``pos``/``vel`` reset between cases: the recorder did exactly that, and the
incremental (``control_key & 0x20``) and masked (``type_mask``) cases legitimately
carry targets over from the previous case.  Isolating them would look tidier and
fail; the carry-over is the recorded behaviour.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import pytest
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from uv_sim.mcu_control import McuController
from uv_sim.mixer import MAX_THRUST_HIL, MAX_THRUST_SIL, mix_thruster_forces
from uv_sim.pid import Pid
from uv_sim.pid_config import PID_PARAM_PATHS

FIXTURE = Path(__file__).resolve().parent / "data" / "thrust_golden.json"

ZERO_POSE = {"x": 0.0, "y": 0.0, "z": 0.0, "rx": 0.0, "ry": 0.0, "rz": 0.0}
ZERO_TWIST = {"x": 0.0, "y": 0.0, "z": 0.0, "rx": 0.0, "ry": 0.0, "rz": 0.0}


@pytest.fixture(scope="module")
def golden() -> dict:
    assert FIXTURE.is_file(), f"missing golden fixture: {FIXTURE}"
    return json.loads(FIXTURE.read_text(encoding="utf-8"))


def _controller(golden: dict) -> McuController:
    """A controller built from the gains recorded in the fixture (never the file)."""
    return McuController({channel: dict(gains)
                          for channel, gains in golden["pid_params"].items()})


# --------------------------------------------------------------------------- #
# Feeding the recorded truth
# --------------------------------------------------------------------------- #
def _quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """``R = Rz(yaw) @ Ry(pitch) @ Rx(roll)`` -- mirrors the recorder's helper."""
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _odometry_from_truth(truth: dict) -> Odometry:
    message = Odometry()
    message.header.frame_id = "world_ned"
    message.child_frame_id = "dynamics"
    message.pose.pose.position.x = truth["x"]
    message.pose.pose.position.y = truth["y"]
    message.pose.pose.position.z = truth["z"]
    qx, qy, qz, qw = _quaternion_from_rpy(truth["roll"], truth["pitch"], truth["yaw"])
    message.pose.pose.orientation.x = qx
    message.pose.pose.orientation.y = qy
    message.pose.pose.orientation.z = qz
    message.pose.pose.orientation.w = qw
    message.twist.twist.linear.x = truth["u"]
    message.twist.twist.linear.y = truth["v"]
    message.twist.twist.linear.z = truth["w"]
    message.twist.twist.angular.x = truth["p"]
    message.twist.twist.angular.y = truth["q"]
    message.twist.twist.angular.z = truth["r"]
    return message


def _imu_from_truth(truth: dict) -> Imu:
    message = Imu()
    message.header.frame_id = "imu_filter"
    message.angular_velocity.x = truth["p"]
    message.angular_velocity.y = truth["q"]
    message.angular_velocity.z = truth["r"]
    return message


def _apply_truth(controller: McuController, truth: dict) -> None:
    controller.apply_odometry(_odometry_from_truth(truth))
    controller.apply_imu(_imu_from_truth(truth))


# --------------------------------------------------------------------------- #
# A5-4: the fixture replays exactly
# --------------------------------------------------------------------------- #
def test_setpoint_cases_reproduce_the_recorded_forces(golden: dict) -> None:
    controller = _controller(golden)
    dt = golden["dt"]

    for case in golden["setpoint_cases"]:
        # Mirror the recorder: zero the measured state, keep targets/flags.
        controller.pos.update(ZERO_POSE)
        controller.vel.update(ZERO_TWIST)

        fields = case["setpoint"]
        immediate = controller.apply_setpoint(
            fields["control_key"], fields["type_mask"],
            fields["x"], fields["y"], fields["z"], fields["yaw"], 0.0,
        )
        assert immediate is None, f"{case['name']}: no setpoint here should emit force"

        for index, step in enumerate(case["steps"]):
            _apply_truth(controller, step["truth"])
            force = (
                controller.step_position(dt)
                if case["step_kind"] == "position"
                else controller.step_velocity(dt)
            )
            assert force == step["force_4dof"], (
                f"{case['name']} step {index}: 4-DOF force drifted from the "
                f"pre-split recording ({force} vs {step['force_4dof']})"
            )
            thrust = mix_thruster_forces(*force, max_thrust=MAX_THRUST_SIL)
            assert thrust == step["thrust"], (
                f"{case['name']} step {index}: thruster output drifted "
                f"({thrust} vs {step['thrust']})"
            )


def test_force_mode_cases_reproduce_the_recorded_output(golden: dict) -> None:
    controller = _controller(golden)
    for case in golden["force_cases"]:
        fields = case["setpoint"]
        immediate = controller.apply_setpoint(
            fields["control_key"], fields["type_mask"],
            fields["x"], fields["y"], fields["z"], fields["yaw"], 0.0,
        )
        assert immediate == case["force_4dof"], f"{case['name']}: force mode output drifted"
        assert mix_thruster_forces(*immediate, max_thrust=MAX_THRUST_SIL) == case["thrust"]


def test_mixer_cases_reproduce_the_recorded_output(golden: dict) -> None:
    for case in golden["mixing_cases"]:
        max_thrust = MAX_THRUST_HIL if case["hil"] else MAX_THRUST_SIL
        thrust = mix_thruster_forces(*case["input_4dof"], max_thrust=max_thrust)
        assert thrust == case["thrust"], f"{case['name']}: mixer output drifted"


def test_the_sensor_round_trip_matches_the_recorded_truth(golden: dict) -> None:
    """Sanity check of the fixture plumbing itself (not of the control maths)."""
    case = golden["setpoint_cases"][0]
    truth = case["steps"][0]["truth"]
    controller = _controller(golden)
    _apply_truth(controller, truth)

    assert controller.pos["x"] == pytest.approx(truth["x"], abs=1e-12)
    assert controller.pos["y"] == pytest.approx(truth["y"], abs=1e-12)
    assert controller.pos["z"] == pytest.approx(truth["z"], abs=1e-12)
    assert controller.pos["rz"] == pytest.approx(math.degrees(truth["yaw"]), abs=1e-9)
    assert controller.pos["ry"] == pytest.approx(math.degrees(truth["pitch"]), abs=1e-9)
    assert controller.vel["x"] == pytest.approx(truth["u"], abs=1e-12)
    assert controller.vel["rz"] == pytest.approx(math.degrees(truth["r"]), abs=1e-9)
    assert controller.current_pose_ready is True


# --------------------------------------------------------------------------- #
# The same maths, stated directly (so a broken fixture cannot hide a broken mixer)
# --------------------------------------------------------------------------- #
def test_mixer_yaw_and_heave_scales_are_the_live_ones() -> None:
    """Yaw enters at x1.0 and heave at x0.8 -- pinning what the dead class got wrong.

    An unimported ``thrust_mixer.ThrustMixer`` in this package used yaw x0.5 and
    heave x1.0; "reusing" it during the split would have silently halved yaw
    authority and raised depth authority by 25%.
    """
    assert mix_thruster_forces(0.0, 0.0, 0.0, 1000.0) == [-1.0, 1.0, 0.0, 0.0, 1.0, -1.0]
    assert mix_thruster_forces(0.0, 0.0, 1250.0, 0.0) == [0.0, 0.0, 1.0, 1.0, -0.0, -0.0]


def test_hil_scaling_uses_normalised_forces() -> None:
    """The MCU hands over forces in [-1, 1]; dividing by 1000 would null them."""
    values = (0.6, -0.2, 0.3, 0.15)
    assert mix_thruster_forces(*values, max_thrust=MAX_THRUST_HIL) == pytest.approx(
        [0.25, 0.95, 0.24, 0.24, -0.65, -0.55]
    )
    assert all(abs(v) < 1e-3 for v in mix_thruster_forces(*values, max_thrust=MAX_THRUST_SIL))


def test_mixer_clamps_every_channel() -> None:
    thrust = mix_thruster_forces(2500.0, -2500.0, 4000.0, -3000.0)
    assert thrust == [1.0, 1.0, 1.0, 1.0, -1.0, 1.0]


# --------------------------------------------------------------------------- #
# Pid: the small behaviours the port must keep
# --------------------------------------------------------------------------- #
def test_pid_clamps_its_integrator_and_keeps_the_feed_forward() -> None:
    pid = Pid(kp=1.0, ki=10.0, kd=0.0, i_limit=0.5)
    for _ in range(10):
        pid.step(1.0, 0.1)
    assert pid.integral == pytest.approx(0.5)          # clamped, not 1.0

    depth = Pid(kp=1.0, ki=0.0, kd=0.0, i_limit=1.0, z_thrust_ff=42.0)
    assert depth.step(0.0, 0.0) == 42.0                # bias survives a zero dt
    assert depth.step(1.0, 0.0) == 43.0


def test_pid_reset_clears_history() -> None:
    pid = Pid(kp=0.0, ki=1.0, kd=0.0, i_limit=10.0)
    pid.step(1.0, 0.5)
    pid.reset()
    assert pid.integral == 0.0 and pid.prev_err == 0.0


# --------------------------------------------------------------------------- #
# Controller details that the fixture does not cover
# --------------------------------------------------------------------------- #
def test_force_mode_returns_output_immediately() -> None:
    controller = McuController({channel: {"kp": 1.0, "ki": 0.0, "kd": 0.0,
                                          "i_limit": 1.0} for channel in PID_PARAM_PATHS})
    immediate = controller.apply_setpoint(0x02, 0x00, 300.0, -150.0, 420.0, 90.0, 0.0)
    assert immediate == [300.0, -150.0, 420.0, 90.0]
    assert controller.force_4dof == immediate
    assert controller.position_ctrl_enabled is False
    assert controller.vel_ctrl_enabled is False


def test_parameter_updates_write_through_to_the_live_pids() -> None:
    controller = _controller_from_defaults()
    controller.apply_param_updates({"chassis.pid.pos.z": {"kp": 555.0, "z_thrust_ff": 9.0}})
    assert controller.pid_params["chassis.pid.pos.z"]["kp"] == 555.0
    assert controller.pids["chassis.pid.pos.z"].kp == 555.0
    assert controller.pids["chassis.pid.pos.z"].z_thrust_ff == 9.0

    controller.apply_param_updates({"chassis.pid.vel.x": 111.0})
    assert controller.pids["chassis.pid.vel.x"].kp == 111.0

    # unknown channels are ignored rather than fatal
    assert controller.apply_param_updates({"chassis.pid.unknown": 1.0}) == []


def _controller_from_defaults() -> McuController:
    from uv_sim.pid_config import DEFAULT_PID_PARAMS

    return McuController({channel: dict(gains) for channel, gains in DEFAULT_PID_PARAMS.items()})


def test_ins_alignment_walks_coarse_then_dvl() -> None:
    controller = _controller_from_defaults()
    assert controller.ins_nav_ready is False

    assert controller.update_ins_alignment(1.0) == []          # still aligning
    assert "MRU" in controller.update_ins_alignment(1.5)[0]
    assert controller.ins_nav_ready is True
    assert controller.ins_state == 5

    controller.ins_command(1, 1.5)                             # enable DVL
    assert controller.update_ins_alignment(2.9) == []
    assert "SINS/DVL" in controller.update_ins_alignment(3.0)[0]
    assert controller.ins_state == 4


def test_normal_arm_waits_for_the_ins() -> None:
    controller = _controller_from_defaults()
    controller.armed = False
    assert controller.note_heartbeat(1) == []                   # INS not ready yet
    assert controller.armed is False
    controller.ins_nav_ready = True
    assert "normal arm" in controller.note_heartbeat(1)[0]
    assert controller.armed is True and controller.arm_mode == 0

    controller.armed = False
    assert "force arm" in controller.note_heartbeat(3)[0]
    assert controller.arm_mode == 3


def test_heartbeat_sequence_increments() -> None:
    controller = _controller_from_defaults()
    assert [controller.next_heartbeat_seq() for _ in range(3)] == [1, 2, 3]


# --------------------------------------------------------------------------- #
# World-frame velocity / force honour the frame bit (fixed 2026-10-12)
# --------------------------------------------------------------------------- #
# 为什么上面那些 golden 用例抓不到这个 bug：`test_setpoint_cases_*` 在调用
# `apply_setpoint` 之前把状态清零（``controller.pos.update(ZERO_POSE)``，三个角
# 都是 0），level 姿态下 R^T = I，世界系与机体系**数值相同** —— 于是"仿真忽略
# frame 位"这件事完全不可见。下面这些用例显式给一个**非水平、带偏航**的姿态，
# 才能真正区分两种坐标系。
#
# 对照的是真实 MCU 的实现：`SetpointRouter` 对 VELOCITY 用 `applyRotationToBody`、
# 对 ACTUATOR(推力) 用 `applyWrenchToBody`，两者都取 R_ZYX(roll,pitch,yaw)^T
# （`third_party/AUV_zit6_cmake/UserApp/Algorithm/MathUtils.hpp`）。

def _tilted_controller(golden: dict, roll_deg: float, pitch_deg: float, yaw_deg: float):
    controller = _controller(golden)
    controller.pos.update({
        "x": 0.0, "y": 0.0, "z": 0.0,
        "rx": roll_deg, "ry": pitch_deg, "rz": yaw_deg,
    })
    return controller


@pytest.mark.parametrize("yaw_deg", [0.0, 90.0, -45.0, 180.0])
def test_world_velocity_is_rotated_into_the_body_frame(golden, yaw_deg) -> None:
    """``control_key`` 0x01 (world) must differ from 0x11 (body) when yawed."""
    world_v = (1.0, 0.0, 0.0)
    body_ctrl = _tilted_controller(golden, 0.0, 0.0, yaw_deg)
    world_ctrl = _tilted_controller(golden, 0.0, 0.0, yaw_deg)

    body_ctrl.apply_setpoint(0x11, 0, *world_v, 0.0, 0.0)
    world_ctrl.apply_setpoint(0x01, 0, *world_v, 0.0, 0.0)

    # 机体系命令：原样进 target_vel
    assert body_ctrl.target_vel["x"] == pytest.approx(1.0)
    assert body_ctrl.target_vel["y"] == pytest.approx(0.0)

    # 世界系命令：转到机体系 —— 期望 R_z(yaw)^T @ [1,0,0] = [cos, -sin, 0]
    expected = (
        math.cos(math.radians(yaw_deg)),
        -math.sin(math.radians(yaw_deg)),
    )
    assert world_ctrl.target_vel["x"] == pytest.approx(expected[0], abs=1e-9)
    assert world_ctrl.target_vel["y"] == pytest.approx(expected[1], abs=1e-9)
    if yaw_deg not in (0.0,):
        assert world_ctrl.target_vel["x"] != pytest.approx(body_ctrl.target_vel["x"]), (
            "world-frame velocity behaved like body-frame: the frame bit is ignored"
        )


def test_world_velocity_matches_the_geometry_rotation(golden) -> None:
    """Pin the conversion to ``geometry.euler_rpy_to_matrix`` (= firmware R_ZYX^T)."""
    from uv_sim.geometry import euler_rpy_to_matrix

    roll, pitch, yaw = math.radians(10.0), math.radians(-7.0), math.radians(35.0)
    controller = _tilted_controller(golden, 10.0, -7.0, 35.0)
    world = (0.4, -0.3, 0.2)

    got = controller.world_to_body_vector(*world)
    rotation = euler_rpy_to_matrix(roll, pitch, yaw)
    expected = tuple(
        sum(rotation[j][i] * world[j] for j in range(3)) for i in range(3)
    )
    assert got == pytest.approx(expected, abs=1e-12)


def test_body_frame_velocity_is_never_rotated(golden) -> None:
    controller = _tilted_controller(golden, 12.0, -8.0, 130.0)
    controller.apply_setpoint(0x11, 0, 0.4, -0.3, 0.2, 0.0, 0.0)
    assert controller.target_vel["x"] == pytest.approx(0.4)
    assert controller.target_vel["y"] == pytest.approx(-0.3)
    assert controller.target_vel["z"] == pytest.approx(0.2)


def test_world_force_is_rotated_into_the_body_frame(golden) -> None:
    """``control_key`` 0x02 (world) vs 0x12 (body), same mask semantics."""
    body_ctrl = _tilted_controller(golden, 0.0, 0.0, 90.0)
    world_ctrl = _tilted_controller(golden, 0.0, 0.0, 90.0)

    body_force = body_ctrl.apply_setpoint(0x12, 0, 100.0, 0.0, 0.0, 0.0, 0.0)
    world_force = world_ctrl.apply_setpoint(0x02, 0, 100.0, 0.0, 0.0, 0.0, 0.0)

    assert body_force[0] == pytest.approx(100.0), "body force must pass through"
    assert body_force[1] == pytest.approx(0.0)
    # 世界系 +x 在 yaw=90° 下是机体的 -y
    assert world_force[0] == pytest.approx(0.0, abs=1e-9)
    assert world_force[1] == pytest.approx(-100.0, abs=1e-9)


def test_world_commands_are_masked_after_the_rotation(golden) -> None:
    """Firmware converts first, then applies the per-axis mask.

    Masking *before* rotating would let one world axis leak into a body axis the
    caller asked to leave alone.
    """
    controller = _tilted_controller(golden, 0.0, 0.0, 90.0)
    controller.target_vel.update({"x": 7.0, "y": 7.0, "z": 7.0, "rz": 7.0})
    # mask 0x02 => 忽略 y，只应用 x/z/rz（世界系）
    controller.apply_setpoint(0x01, 0x02, 1.0, 0.0, 0.5, 0.0, 0.0)

    # 世界系 (1,0,0.5) 在 yaw=90° 下 -> 机体 (0, -1, 0.5)
    assert controller.target_vel["x"] == pytest.approx(0.0, abs=1e-9), "x 被 y 的旋转漏进来了"
    assert controller.target_vel["y"] == pytest.approx(7.0), "mask 说忽略 y，必须保留旧值"
    assert controller.target_vel["z"] == pytest.approx(0.5)
