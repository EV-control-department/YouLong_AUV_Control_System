#include "ControllerHost.hpp"
#include "MathUtils.hpp"

namespace auv {
namespace host {

ControllerHost::ControllerHost() : cascade_() {
  auv::motion::motion_context.resetNavigation();
}

ControllerHost::ControllerHost(const auv::config::ChassisConfig &cfg)
    : cascade_(cfg) {
  auv::motion::motion_context.resetNavigation();
}

void ControllerHost::applyConfig(const auv::config::ChassisConfig &cfg) {
  cascade_.applyConfig(cfg);
}

void ControllerHost::updateSetpoint(auv::motion::ControlLevel level,
                                    const float val[6], uint32_t mask,
                                    bool is_body, bool is_inc) {
  // 与固件 ChassisManager 一致：按轴保留现有控制层级，路由返回更新后的层级。
  auto levels = router_.route(cascade_.getAxisControlLevels(), level, val, mask,
                              is_body, is_inc);
  cascade_.setAxisControlLevels(levels);
}

void ControllerHost::updateNav(const auv::motion::NavState &nav,
                               uint32_t timestamp_ms, bool valid) {
  auv::motion::motion_context.updateNavigationSnapshot(nav, timestamp_ms, valid);
}

auv::motion::OdomSnapshot ControllerHost::getOdomSnapshot() const {
  return auv::motion::motion_context.getOdomSnapshot();
}

bool ControllerHost::trySetOrigin(uint32_t now_ms, uint32_t max_age_ms,
                                  auv::motion::OriginCommit &commit) {
  return auv::motion::motion_context.trySetOrigin(now_ms, max_age_ms, commit);
}

void ControllerHost::setHomeOffset(const float pos6[6]) {
  std::array<float, 6> origin;
  for (int i = 0; i < 6; ++i) origin[i] = pos6[i];
  auv::motion::motion_context.setHomeOffset(origin);
}

void ControllerHost::clearHomeOffset() {
  auv::motion::motion_context.clearHomeOffset();
}

bool ControllerHost::hasHomeOffset() const {
  return auv::motion::motion_context.getOdomSnapshot().origin_initialized;
}

void ControllerHost::setControlLevel(auv::motion::ControlLevel lvl) {
  cascade_.setControlLevel(lvl);
}

void ControllerHost::resetSetpoints() {
  auv::motion::motion_context.current_setpoint_.set(auv::motion::TargetSetpoint{});
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
