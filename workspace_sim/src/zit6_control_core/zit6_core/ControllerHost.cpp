#include "ControllerHost.hpp"
#include "MathUtils.hpp"

namespace auv {
namespace host {

ControllerHost::ControllerHost() : cascade_() {}

ControllerHost::ControllerHost(const auv::config::ChassisConfig &cfg)
    : cascade_(cfg) {}

void ControllerHost::applyConfig(const auv::config::ChassisConfig &cfg) {
  cascade_.applyConfig(cfg);
}

void ControllerHost::updateSetpoint(auv::motion::ControlLevel level,
                                    const float val[6], uint32_t mask,
                                    bool is_body, bool is_inc) {
  // 与固件 ChassisManager::updateSetpoint 一致:route 读/写 motion_context 单例
  // 并返回目标层级,再 setControlLevel(固件内部读单例 nav 做无扰动对齐)。
  auto lv = router_.route(cascade_.getControlLevel(), level, val, mask,
                          is_body, is_inc);
  cascade_.setControlLevel(lv);
}

void ControllerHost::updateNav(const auv::motion::NavState &nav) {
  // 先应用解锁原点(Home Offset),再写入 motion_context 单例,
  // 供 verbatim CascadeController::update() 读取。复刻固件
  // ControlTask::updateNavigation 的 offset 平移+旋转。
  auv::motion::NavState nav_out = nav;
  const auto home = auv::motion::motion_context.home_offset_.get();
  if (home.active) {
    const auto &off = home.offset;
    float diff[6];
    for (int i = 0; i < 6; i++)
      diff[i] = nav.pos_world[i] - off[i];
    // offset.roll/pitch 恒 0(固件 executeArm 强制),故仅按 offset.yaw 旋转
    auv::algorithm::math::applyRotationToBody(diff, nav_out.pos_world.data(),
                                              off[3], off[4], off[5]);
    for (int i = 3; i < 6; i++)
      nav_out.pos_world[i] = auv::motion::MotionContext::wrapAngle(nav_out.pos_world[i]);
  }
  auv::motion::motion_context.nav_state_.set(nav_out);
}

void ControllerHost::setHomeOffset(const float pos6[6]) {
  // 复刻固件 SafetyMonitor::executeArm: 当前位姿写入 home_offset, roll/pitch 强制 0
  auv::motion::HomeOffset h;
  h.active = true;
  h.offset[0] = pos6[0];
  h.offset[1] = pos6[1];
  h.offset[2] = pos6[2];
  h.offset[3] = 0.0f;  // Roll 强制 0
  h.offset[4] = 0.0f;  // Pitch 强制 0
  h.offset[5] = pos6[5];  // Yaw 正常记录
  auv::motion::motion_context.home_offset_.set(h);
}

void ControllerHost::clearHomeOffset() {
  // 复刻固件 forceDisarmWithNeutralLevel::clearHomeOffset
  auv::motion::HomeOffset h;
  auv::motion::motion_context.home_offset_.set(h);
}

bool ControllerHost::hasHomeOffset() const {
  return auv::motion::motion_context.home_offset_.get().active;
}

void ControllerHost::setControlLevel(auv::motion::ControlLevel lvl) {
  cascade_.setControlLevel(lvl);
}

std::array<float, 6> ControllerHost::step() {
  // verbatim CascadeController::update() 读取 motion_context 单例(nav + setpoint)
  return cascade_.update();
}

void ControllerHost::configurePID(int axis, bool is_pos_ring, float kp,
                                  float ki, float kd, float i_limit,
                                  float out_limit) {
  cascade_.configurePID(axis, is_pos_ring, kp, ki, kd, i_limit, out_limit);
}

void ControllerHost::configureProfile(int axis, float max_v, float max_a) {
  cascade_.configureProfile(axis, max_v, max_a);
}

} // namespace host
} // namespace auv
