#ifndef __CONTROLLER_HOST_HPP
#define __CONTROLLER_HOST_HPP

/**
 * @file ControllerHost.hpp
 * @brief 宿主(Host)控制核门面：SetpointRouter + CascadeController 的封装。
 *
 * 对应固件 ChassisManager:状态注入到全局 auv::motion::motion_context 单例,
 * 然后调用固件原样(verbatim)的 CascadeController::update() 读取单例产生分力。
 * 单线程、无 ROS/固件平台依赖;由 Python 桥以 100Hz 驱动。
 */

#include "CascadeController.hpp"
#include "MotionContext.hpp"
#include "SetpointRouter.hpp"
#include <array>

namespace auv {
namespace host {

class ControllerHost {
public:
  ControllerHost();
  explicit ControllerHost(const auv::config::ChassisConfig &cfg);

  void applyConfig(const auv::config::ChassisConfig &cfg);

  /**
   * @brief 更新设定点（对应固件 ChassisManager::updateSetpoint）
   * @param level   目标控制层级（firmware 侧 new_level）
   * @param val     设定值数组 [X,Y,Z,Roll,Pitch,Yaw]（radian）
   * @param mask    轴掩码
   * @param is_body 是否机体系
   * @param is_inc  是否增量
   */
  void updateSetpoint(auv::motion::ControlLevel level, const float val[6],
                      uint32_t mask, bool is_body, bool is_inc);

  /** @brief 更新导航状态（写入 motion_context 单例;世界系 NED + 机体系 FRD，radian） */
  void updateNav(const auv::motion::NavState &nav);

  /**
   * @brief 设置解锁原点(Home Offset) — 复刻固件 SafetyMonitor::executeArm。
   * @param pos6 当前位置 [x,y,z,roll,pitch,yaw];内部 roll/pitch 强制 0。
   * 之后 updateNav 会先把输入减去该原点并按 offset.yaw 旋转(理正姿态)。
   */
  void setHomeOffset(const float pos6[6]);

  /** @brief 清除解锁原点(复刻 forceDisarmWithNeutralLevel::clearHomeOffset)。 */
  void clearHomeOffset();

  /** @brief 是否已设置解锁原点 */
  bool hasHomeOffset() const;

  /** @brief 强制切换控制层级（带无扰动对齐，固件内部读单例 nav） */
  void setControlLevel(auv::motion::ControlLevel lvl);

  /** @brief 100Hz 演进一次，返回 6-DOF 归一化力/力矩 [Fx,Fy,Fz,Mroll,Mpitch,Myaw] */
  std::array<float, 6> step();

  auv::motion::ControlLevel getControlLevel() const { return cascade_.getControlLevel(); }

  // 运行时参数写通（/zit6/update_params 用）
  void configurePID(int axis, bool is_pos_ring, float kp, float ki, float kd,
                    float i_limit, float out_limit);
  void configureProfile(int axis, float max_v, float max_a);

private:
  auv::component::SetpointRouter router_{};
  auv::component::CascadeController cascade_{};
};

} // namespace host
} // namespace auv

#endif // __CONTROLLER_HOST_HPP
