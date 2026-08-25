#include "MotionContext.hpp"

/**
 * 宿主(Host) motion_context 全局单例定义。
 * 与固件 UserApp/Common/MotionContext.cpp 行为一致(仅 wrapAngle + home offset),
 * 去掉 FreeRTOS 队列/日志依赖。宿主桥通过 nav_state_/current_setpoint_ 注入状态,
 * 喂给 verbatim CascadeController::update()。
 */

namespace auv {
namespace motion {

MotionContext motion_context{};

float MotionContext::wrapAngle(float angle) {
  constexpr float kPi = 3.14159265358979323846f;
  constexpr float kTwoPi = 6.28318530717958647692f;
  if (angle > kPi || angle < -kPi) {
    angle = std::fmod(angle + kPi, kTwoPi);
    if (angle < 0.0f)
      angle += kTwoPi;
    angle -= kPi;
  }
  return angle;
}

void MotionContext::setHomeOffset(const std::array<float, 6> &offset) {
  HomeOffset h;
  h.active = true;
  h.offset = offset;
  home_offset_.set(h);
}

void MotionContext::clearHomeOffset() {
  HomeOffset h;
  home_offset_.set(h);
}

} // namespace motion
} // namespace auv
