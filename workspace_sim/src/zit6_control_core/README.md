# zit6_control_core

ZIT6 固件**控制核的宿主(host)编译版**——把 STM32/FreeRTOS 固件里的级联 PID 控制器(级联位置→速度→力)、运动学规划、位置/速度/力路由用 pybind11 编译成 Python 可导入的模块,供仿真桥(`uv_sim/sim_bridge.py`)在进程内调用。

> **为什么:** `uv_sim/sim_bridge.py` 原先用 Python 自己抄了一份控制逻辑,会和真机固件悄悄漂移。本包让仿真直接跑**固件真实 C++ 源码**,消除逻辑偏差。

## 架构:固件原封不动 + 宿主打桩

- **`zit6_core/` 里的控制核文件全部是软链,逐字等于固件 `third_party/AUV_zit6_cmake`** —— 不剥离、不重构、不复制。固件升级自动跟随。
- **`stubs/` 是宿主打桩点**:提供 `FreeRTOS.h`/`task.h`/`queue.h`(空临界区)、`LockedField.hpp`(无锁)、`MotionContext.hpp`(全局单例 `motion_context`)、`SystemConfig.hpp`(`ChassisConfig`)。include 路径 `stubs/` 放最前,shadow 掉固件平台头,使 verbatim 控制核宿主可编译。
- **`ControllerHost`**(纯桩)把 Python 桥喂的 nav/setpoint **写进 `motion_context` 单例**,然后调 verbatim `CascadeController::update()`(内部读单例)取 6-DOF 分力。与固件 `ChassisManager` 的驱动方式一致。

## 目录

```
zit6_control_core/
  CMakeLists.txt              # ament_cmake: 编 zit6_core 静态库 + pybind11 模块
  package.xml
  src/pybind_control_core.cpp # pybind11 绑定: 暴露 Zit6Controller 类
  stubs/                      # 宿主打桩点(软链固件头的 shadow 替代)
    FreeRTOS.h task.h queue.h # 空临界区/任务/队列
    LockedField.hpp           # 无锁版(单线程宿主)
    MotionContext.hpp         # 全局单例 motion_context + NavState/TargetSetpoint
    SystemConfig.hpp          # ChassisConfig/AxisConfig
  zit6_core/                  # 全部软链到固件(verbatim),+ 两个宿主桩源
    MathUtils.hpp             #  (软链) 6-DOF 旋转矩阵/变换
    PID_Controller.*          #  (软链) 增量式 PID, 外部微分项
    KinematicProfile.*        #  (软链) 影子平滑器
    CascadeController.*       #  (软链) 级联控制器, 读 motion_context 单例
    SetpointRouter.*          #  (软链) 路由, 读/写 motion_context 单例
    motion_context_host.cpp   # 宿主: 定义全局 motion_context + wrapAngle
    ControllerHost.*          # 宿主桩: 写单例 -> update() -> 取分力
  scripts/sync_core.sh        # 校验/重建软链到固件
  test/test_core.py           # 单元测试: 分力合理性
```

## 构建前置

```bash
sudo apt install python3-pybind11   # CMake find_package(pybind11 REQUIRED)
# Eigen 3.4+ :  一般预装(sudo apt install libeigen3-dev 有则忽略)
```

系统 python 是 externally-managed,**不要** `pip install pybind11`(会要求 `--break-system-packages`)。

## 构建 & 测试

```bash
cd workspace_auv
colcon build --packages-select zit6_control_core
source install/setup.bash

# 单元测试
python3 -c "import zit6_control_core; print(zit6_control_core.__file__)"
python3 -m pytest src/zit6_control_core/test/
```

运行后 `uv_sim/sim_bridge.py` 直接:
```python
from zit6_control_core import Zit6Controller
core = Zit6Controller(chassis_config_dict)   # 取固件 config.json 的 chassis 段
core.update_setpoint(1, [x,y,z,roll,pitch,yaw], 0, is_body, is_inc)
core.update_nav(pos_world6, vel_body6)        # 弧度
forces6 = core.step()                          # [Fx,Fy,Fz,Mroll,Mpitch,Myaw] 归一化 [-1,1]
```

## 与固件同步

`zit6_core/` 全部是**软链**,固件 `third_party/AUV_zit6_cmake` 改动后无需复制,自动跟随。若软链缺失或被误替换,重建:

```bash
cd workspace_auv/src/zit6_control_core && ./scripts/sync_core.sh
```

脚本会重新把纯数学 + 控制核软链指回固件,并检查 `stubs/` 桩目录存在。宿主 `stubs/` 与 `motion_context_host.cpp` 是只在本包维护的桩,不属于固件。

## 已知事项 / 坑

- **单位**: `/zit6/*` 的 pos/yaw 与 `NavState`/`TargetSetpoint` 都是**弧度(rad)**,不要做度↔弧度转换。
- **ControlLevel**: NONE=0, POSITION=1, VELOCITY=2, ACTUATOR=3,与 `ZitStatus.msg` 的 LEVEL_* 一致。
- **dt 硬编码 0.01s**: `step()` 不带 dt,宿主桥必须严格 100Hz 调用(专用线程 + ≥10ms 护栏)。
- **增益量级**: 控制核用固件 `config.json` 的增益(如 `pos.x kp=0.5`、`vel.x kp=6.0`,planner off),和旧 Python `_Pid`(`pos.x kp=800`)完全不同。仿真会变慢/需重新调参,这是忠于固件的预期。
- **不包含推力混合**: 核输出 6-DOF 机体分力,不做 6 推进器混合(真实中在外接电机板)。混合由 `uv_sim/thrust_mixer.py` 完成。
