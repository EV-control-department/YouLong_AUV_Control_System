#!/usr/bin/env bash
# 校验/刷新 zit6_core 到固件(AUV_zit6_cmake)的软链接。
#
# 架构:仿真打桩 = 固件控制核原封不动(软链),宿主用 stubs/ 打桩喂状态。
# 固件 submodule 改动后,纯数学/控制核文件经软链自动跟随,无需复制。
set -euo pipefail

PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FIRM_REL=../../../../third_party/AUV_zit6_cmake/UserApp
CORE="$PKG_DIR/zit6_core"

if [ ! -d "$CORE/../stubs" ]; then
  echo "!! 缺少宿主桩目录: 应在 $PKG_DIR/stubs (FreeRTOS/task/queue/MotionContext/SystemConfig/LockedField)" >&2
  exit 1
fi

# (重)建软链: 纯数学 + 控制核全部指向固件,保证"原封不动 + 自动同步"
links=(
  "Algorithm/MathUtils.hpp"
  "Algorithm/PID_Controller.hpp"
  "Algorithm/PID_Controller.cpp"
  "Algorithm/KinematicProfile.hpp"
  "Algorithm/KinematicProfile.cpp"
  "Component/Chassis/CascadeController.hpp"
  "Component/Chassis/CascadeController.cpp"
  "Component/Chassis/SetpointRouter.hpp"
  "Component/Chassis/SetpointRouter.cpp"
)
for rel in "${links[@]}"; do
  name="$(basename "$rel")"
  ln -sf "$FIRM_REL/$rel" "$CORE/$name"
done

echo "zit6_core 已全部软链到固件:$FIRM_REL"
echo
echo "说明:"
echo "  - 纯数学(Algorithm/*)与控制核(Chassis/*)逐字等于固件,零剥离、零重构。"
echo "  - 宿主桩在 stubs/:FreeRTOS/task/queue(空临界区)、LockedField(无锁)、"
echo "    MotionContext(全局单例 motion_context)、SystemConfig(ChassisConfig)。"
echo "  - ControllerHost(纯桩)把 Python 桥喂的 nav/setpoint 写进 motion_context 单例,"
echo "    调 verbatim CascadeController::update() 取分力。"
