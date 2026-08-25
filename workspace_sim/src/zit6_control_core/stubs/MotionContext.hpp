#ifndef __MOTION_CONTEXT_HPP
#define __MOTION_CONTEXT_HPP

/**
 * 宿主(Host) MotionContext 桩 — 与固件 UserApp/Common/MotionContext.hpp 同构,
 * 提供 verbatim CascadeController/SetpointRouter 编译所需的类型与全局单例。
 * 与固件差异:移除 FreeRTOS 队列/外设 include,LockedField 为无锁版。
 */

#include "LockedField.hpp"
#include <array>
#include <cmath>
#include <cstdint>

namespace auv {
namespace motion {

enum class ControlLevel : uint8_t { NONE = 0, POSITION = 1, VELOCITY = 2, ACTUATOR = 3 };

/** 6-DOF 导航状态。索引 [x,y,z,roll,pitch,yaw],角度弧度,世界系 NED / 机体系 FRD。 */
struct NavState {
  std::array<float, 6> pos_world = {0, 0, 0, 0, 0, 0};
  std::array<float, 6> vel_body = {0, 0, 0, 0, 0, 0};
};

/** 级联控制各层级目标设定值(6-DOF,弧度)。 */
struct TargetSetpoint {
  std::array<float, 6> pos_world = {0, 0, 0, 0, 0, 0};
  std::array<float, 6> vel_body = {0, 0, 0, 0, 0, 0};
  std::array<float, 6> thrust_body = {0, 0, 0, 0, 0, 0};
};

struct HomeOffset {
  bool active = false;
  std::array<float, 6> offset = {0, 0, 0, 0, 0, 0};
};

struct Constants {
  static constexpr float CONTROL_FREQ = 100.0f;
  static constexpr uint32_t CONTROL_PERIOD_MS = 10;
  static constexpr float DEG2RAD = 0.0174532925f;
  static constexpr float RAD2DEG = 57.2957795f;
};

class MotionContext {
public:
  static float wrapAngle(float angle);

  // 线程安全字段(LockedField 宿主无锁,退化为读写)
  LockedField<NavState> nav_state_{};
  LockedField<TargetSetpoint> current_setpoint_{};
  LockedField<float> last_dt_ms_{0.0f};
  LockedField<float> last_exec_ms_{0.0f};
  LockedField<uint32_t> control_overrun_count_{0};
  LockedField<uint32_t> thrust_tx_fail_count_{0};
  LockedField<uint32_t> last_received_seq_{0};
  LockedField<std::array<float, 6>> last_output_forces_{};

  // SITL/USBL 队列 — 宿主驱动不走它们,保留为 void*(stubs/queue.h)
  void *sitl_nav_queue = nullptr;
  void *usbl_topic_queue = nullptr;

  LockedField<HomeOffset> home_offset_{};

  void setHomeOffset(const std::array<float, 6> &offset);
  void clearHomeOffset();
};

extern MotionContext motion_context;

} // namespace motion
} // namespace auv

#endif // __MOTION_CONTEXT_HPP
