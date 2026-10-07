#include "MotionContext.hpp"

namespace auv {
namespace motion {

MotionContext motion_context{};

float MotionContext::wrapAngle(float angle) {
  constexpr float pi = 3.14159265358979323846f;
  constexpr float two_pi = 2.0f * pi;
  if (angle > pi || angle < -pi) {
    angle = std::fmod(angle + pi, two_pi);
    if (angle < 0.0f) angle += two_pi;
    angle -= pi;
  }
  return angle;
}

void MotionContext::resetNavigation() {
  std::lock_guard<std::mutex> guard(navigation_mutex_);
  raw_nav_ = NavSnapshot{};
  odom_ = OdomSnapshot{};
  nav_state_.set(NavState{});
  current_setpoint_.set(TargetSetpoint{});
  home_offset_.set(HomeOffset{});
}

NavState MotionContext::applyOrigin(const NavState &nav) const {
  NavState result = nav;
  const auto home = home_offset_.get();
  if (home.active) {
    const float dx = nav.pos_world[0] - home.offset[0];
    const float dy = nav.pos_world[1] - home.offset[1];
    const float cy = std::cos(home.offset[5]);
    const float sy = std::sin(home.offset[5]);
    result.pos_world[0] = cy * dx + sy * dy;
    result.pos_world[1] = -sy * dx + cy * dy;
    result.pos_world[2] = nav.pos_world[2] - home.offset[2];
    result.pos_world[5] = wrapAngle(nav.pos_world[5] - home.offset[5]);
  }
  return result;
}

void MotionContext::updateNavigationSnapshot(const NavState &nav,
                                             uint32_t timestamp_ms, bool valid) {
  std::lock_guard<std::mutex> guard(navigation_mutex_);
  for (size_t i = 0; i < 6; ++i)
    valid = valid && std::isfinite(nav.pos_world[i]) && std::isfinite(nav.vel_body[i]);
  raw_nav_.raw_nav = nav;
  raw_nav_.nav_timestamp_ms = timestamp_ms;
  raw_nav_.nav_valid = valid;
  raw_nav_.have_sample = raw_nav_.have_sample || valid;
  odom_.nav_state = applyOrigin(nav);
  odom_.nav_timestamp_ms = timestamp_ms;
  odom_.nav_valid = valid;
  nav_state_.set(odom_.nav_state);
}

OdomSnapshot MotionContext::getOdomSnapshot() const {
  std::lock_guard<std::mutex> guard(navigation_mutex_);
  return odom_;
}

bool MotionContext::trySetOrigin(uint32_t now_ms, uint32_t max_age_ms,
                                OriginCommit &commit) {
  std::lock_guard<std::mutex> guard(navigation_mutex_);
  if (!raw_nav_.have_sample || !raw_nav_.nav_valid ||
      now_ms - raw_nav_.nav_timestamp_ms > max_age_ms)
    return false;
  for (float value : raw_nav_.raw_nav.pos_world)
    if (!std::isfinite(value)) return false;
  for (float value : raw_nav_.raw_nav.vel_body)
    if (!std::isfinite(value)) return false;
  HomeOffset home;
  home.active = true;
  home.offset = raw_nav_.raw_nav.pos_world;
  home.offset[3] = home.offset[4] = 0.0f;
  home_offset_.set(home);
  odom_.origin_initialized = true;
  ++odom_.origin_generation;
  if (odom_.origin_generation == 0) ++odom_.origin_generation;
  odom_.nav_state = applyOrigin(raw_nav_.raw_nav);
  nav_state_.set(odom_.nav_state);
  current_setpoint_.set(TargetSetpoint{});
  commit.origin_nav = home.offset;
  commit.nav_timestamp_ms = raw_nav_.nav_timestamp_ms;
  commit.origin_generation = odom_.origin_generation;
  return true;
}

void MotionContext::setHomeOffset(const std::array<float, 6> &offset) {
  std::lock_guard<std::mutex> guard(navigation_mutex_);
  HomeOffset home;
  home.active = true;
  home.offset = offset;
  home.offset[3] = home.offset[4] = 0.0f;
  home_offset_.set(home);
  odom_.origin_initialized = true;
  ++odom_.origin_generation;
  if (odom_.origin_generation == 0) ++odom_.origin_generation;
  odom_.nav_state = applyOrigin(raw_nav_.raw_nav);
  nav_state_.set(odom_.nav_state);
}

void MotionContext::clearHomeOffset() {
  std::lock_guard<std::mutex> guard(navigation_mutex_);
  home_offset_.set(HomeOffset{});
  odom_.origin_initialized = false;
  odom_.origin_generation = 0;
  odom_.nav_state = raw_nav_.raw_nav;
  nav_state_.set(odom_.nav_state);
}

} // namespace motion
} // namespace auv
