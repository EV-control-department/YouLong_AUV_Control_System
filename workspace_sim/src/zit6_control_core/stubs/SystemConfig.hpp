#ifndef __SYSTEM_CONFIG_HPP
#define __SYSTEM_CONFIG_HPP

/**
 * 宿主(Host) SystemConfig 桩 — 提供 verbatim CascadeController.hpp 需要的
 * auv::config::ChassisConfig/AxisConfig 结构体(与固件 SystemConfig.hpp 同构)。
 * 固件那份由 config.json 生成且带注册表;宿主只需纯数据结构,由 Python 传入参数。
 */

#include <cstdint>

namespace auv {
namespace config {

struct AxisConfig {
  float pos_kp = 0.0f;
  float pos_ki = 0.0f;
  float pos_kd = 0.0f;
  float pos_i_limit = 0.0f;
  float pos_output_limit = 0.0f;
  float vel_kp = 0.0f;
  float vel_ki = 0.0f;
  float vel_kd = 0.0f;
  float vel_i_limit = 0.0f;
  float vel_output_limit = 0.0f;
  float max_v = 0.0f;
  float max_a = 0.0f;
  float mass = 0.0f;
  float drag = 0.0f;
};

struct ChassisConfig {
  bool planner_enabled = false;
  AxisConfig x;
  AxisConfig y;
  AxisConfig z;
  AxisConfig roll;
  AxisConfig pitch;
  AxisConfig yaw;
};

} // namespace config
} // namespace auv

#endif // __SYSTEM_CONFIG_HPP
