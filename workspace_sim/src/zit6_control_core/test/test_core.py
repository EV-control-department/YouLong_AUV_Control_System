"""单元测试:验证 ZIT6 原生控制核产生合理分力。

运行:
  cd workspace_auv && colcon build --packages-select zit6_control_core
  source install/setup.bash
  python3 -m pytest src/zit6_control_core/test/
"""

import math

import pytest

from zit6_control_core import Zit6Controller

# firmware config.json 的 chassis 段(planner off, 4-DOF 为主)
CHASSIS = {
    "planner_enabled": False,
    "x": {"pos_kp": 0.5, "pos_ki": 0.0, "pos_kd": 0.0, "pos_i_limit": 0.3,
          "pos_output_limit": 0.5, "vel_kp": 6.0, "vel_ki": 0.02,
          "vel_kd": 0.01, "vel_i_limit": 0.03, "vel_output_limit": 0.8,
          "max_v": 0.8, "max_a": 0.4, "mass": 0.013, "drag": 2.0},
    "y": {"pos_kp": 1.0, "pos_ki": 0.0, "pos_kd": 0.0, "pos_i_limit": 0.2,
          "pos_output_limit": 0.2, "vel_kp": 12.0, "vel_ki": 0.015,
          "vel_kd": 0.03, "vel_i_limit": 0.8, "vel_output_limit": 0.9,
          "max_v": 0.6, "max_a": 0.3, "mass": 0.013, "drag": 2.5},
    "z": {"pos_kp": 1.0, "pos_ki": 0.0, "pos_kd": 0.0, "pos_i_limit": 0.2,
          "pos_output_limit": 0.5, "vel_kp": 13.0, "vel_ki": 0.1,
          "vel_kd": 0.0, "vel_i_limit": 0.2, "vel_output_limit": 0.8,
          "max_v": 0.5, "max_a": 0.3, "mass": 0.016, "drag": 5.0},
    "roll": {"pos_kp": 0.0, "pos_ki": 0.0, "pos_kd": 0.0, "pos_i_limit": 0.0,
             "pos_output_limit": 0.0, "vel_kp": 0.0, "vel_ki": 0.0,
             "vel_kd": 0.0, "vel_i_limit": 0.0, "vel_output_limit": 0.0,
             "max_v": 0.0, "max_a": 0.0, "mass": 0.0, "drag": 0.0},
    "pitch": {"pos_kp": 0.0, "pos_ki": 0.0, "pos_kd": 0.0, "pos_i_limit": 0.0,
              "pos_output_limit": 0.0, "vel_kp": 0.0, "vel_ki": 0.0,
              "vel_kd": 0.0, "vel_i_limit": 0.0, "vel_output_limit": 0.0,
              "max_v": 0.0, "max_a": 0.0, "mass": 0.0, "drag": 0.0},
    "yaw": {"pos_kp": 2.5, "pos_ki": 0.0, "pos_kd": 0.02, "pos_i_limit": 0.5,
            "pos_output_limit": 1.5, "vel_kp": 0.9, "vel_ki": 0.002,
            "vel_kd": 0.0, "vel_i_limit": 0.05, "vel_output_limit": 1.0,
            "max_v": 1.0, "max_a": 0.8, "mass": 3.0, "drag": 2.0},
}

DT = 0.01


def _run(core, steps, nav=(0, 0, 0, 0, 0, 0), vel=(0, 0, 0, 0, 0, 0)):
    core.update_nav(nav, vel)
    forces = [core.step() for _ in range(steps)]
    return forces


def test_forces_in_bounds():
    """缺省状态(无目标)分力都在 [-1,1] 且为 0。"""
    core = Zit6Controller(CHASSIS)
    core.update_nav((0, 0, 0, 0, 0, 0), (0, 0, 0, 0, 0, 0))
    for _ in range(10):
        f = core.step()
        assert all(-1.0 <= v <= 1.0 for v in f)


def test_position_forward_produces_positive_fx():
    """绝对世界 +x 位置目标(control_key=0 POSITION),稳态 Fx 为正且收敛。"""
    core = Zit6Controller(CHASSIS)
    core.update_setpoint(0, (1.0, 0.0, 0.0, 0.0, 0.0, 0.0), 0, False, False)
    assert core.control_level == 1  # POSITION
    forces = _run(core, 2000)
    last = forces[-1]
    # 位置误差导致 +x 力;Fy/Fz 应接近 0
    assert last[0] > 0.0
    assert abs(last[1]) < 1e-2
    assert abs(last[2]) < 1e-2


def test_position_yaw_positive_myaw():
    """绝对世界 yaw 目标 π/2(朝东),稳态 Myaw 为正后收敛。"""
    core = Zit6Controller(CHASSIS)
    core.update_setpoint(0, (0.0, 0.0, 0.0, 0.0, 0.0, math.pi / 2), 0, False, False)
    assert core.control_level == 1
    forces = _run(core, 2000)
    last = forces[-1]
    # yaw 误差 → 正向 Mz(朝东 NED yaw 顺时针为正)
    assert last[5] > 0.0


def test_velocity_mode_level():
    """VELOCITY 模式(control_key=1)应映射为 control_level=2。"""
    core = Zit6Controller(CHASSIS)
    core.update_setpoint(1, (0.5, 0.0, 0.0, 0.0, 0.0, 0.0), 0, False, False)
    assert core.control_level == 2  # VELOCITY


def test_actuator_passthrough():
    """ACTUATOR(control_key=2) 直接推力模式:核心输出约等于 thrust_body 设定。

    同时验证 control_key 解码:2 -> ControlLevel::ACTUATOR(3)。
    """
    core = Zit6Controller(CHASSIS)
    core.update_setpoint(2, (0.5, 0.0, 0.0, 0.0, 0.0, 0.0), 0, True, False)
    assert core.control_level == 3  # ACTUATOR
    f = core.step()
    assert abs(f[0] - 0.5) < 1e-3
    assert abs(f[1]) < 1e-3
    assert abs(f[3]) < 1e-3
