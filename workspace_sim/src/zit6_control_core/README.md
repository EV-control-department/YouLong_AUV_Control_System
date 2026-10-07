# zit6_control_core

ZIT6 固件**控制核的宿主(host)编译版**——把 STM32/FreeRTOS 固件里的级联 PID 控制器(级联位置→速度→力)、运动学规划、位置/速度/力路由用 pybind11 编译成 Python 可导入的模块,供仿真桥(`uv_sim_bridge/sim_bridge.py`)在进程内调用。

> **为什么:** `uv_sim_bridge/sim_bridge.py` 原先用 Python 自己抄了一份控制逻辑,会和真机固件悄悄漂移。本包让仿真直接跑**固件真实 C++ 源码**,消除逻辑偏差。

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
    MotionContext.hpp         # 全局单例 motion_context + NavState/TargetSetpoint + 原子 raw/odom 快照
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
cd workspace_sim
colcon build --packages-select zit6_control_core
source install/setup.bash

# 单元测试
python3 -c "import zit6_control_core; print(zit6_control_core.__file__)"
python3 -m pytest src/zit6_control_core/test/
```

运行后 `uv_sim_bridge/sim_bridge.py` 直接:
```python
from zit6_control_core import Zit6Controller
core = Zit6Controller(chassis_config_dict)   # 取固件 config.json 的 chassis 段
core.update_setpoint(1, [x,y,z,roll,pitch,yaw], 0, is_body, is_inc)
core.update_nav(raw_nav6, vel_body6, timestamp_ms=now_ms, valid=True)  # 米/弧度
commit = core.try_set_origin(now_ms, max_age_ms=200)  # 显式设置，ARM 不调用
odom = core.get_odom_snapshot()                     # 与 MCU typed odom 一致
forces6 = core.step()                          # [Fx,Fy,Fz,Mroll,Mpitch,Myaw] 归一化 [-1,1]
```

## 与固件同步

`zit6_core/` 全部是**软链**,固件 `third_party/AUV_zit6_cmake` 改动后无需复制,自动跟随。若软链缺失或被误替换,重建:

```bash
cd workspace_sim/src/zit6_control_core && ./scripts/sync_core.sh
```

脚本会重新把纯数学 + 控制核软链指回固件,并检查 `stubs/` 桩目录存在。宿主 `stubs/` 与 `motion_context_host.cpp` 是只在本包维护的桩,不属于固件。

## 已知事项 / 坑

- **单位**: `/zit6/*` 的 pos/yaw 与 `NavState`/`TargetSetpoint` 都是**弧度(rad)**,不要做度↔弧度转换。
- **ControlLevel**: NONE=0, POSITION=1, VELOCITY=2, ACTUATOR=3,与 `ZitStatus.msg` 的 LEVEL_* 一致。
- **dt 硬编码 0.01s**: `step()` 不带 dt,宿主桥必须严格 100Hz 调用(专用线程 + ≥10ms 护栏)。
- **增益量级**: 控制核用固件 `config.json` 的增益(如 `pos.x kp=0.5`、`vel.x kp=6.0`,planner off),和旧 Python `_Pid`(`pos.x kp=800`)完全不同。仿真会变慢/需重新调参,这是忠于固件的预期。
- **不包含推力混合**: 核输出 6-DOF 机体分力,不做 6 推进器混合(真实中在外接电机板)。混合由 `uv_sim_bridge/thrust_mixer.py` 完成。

## 原点与 SIL/HIL 导航

`update_nav` 接收持续的 nav 位姿与 body 速度，宿主 `MotionContext` 一次性提交 raw nav、采用 MCU 原点计算的 odom 和 `origin_generation`。`try_set_origin` 仅接受有效、新鲜的 raw nav，响应原点 `[x,y,z,0,0,yaw]`；重复设置从 raw nav 取样，roll/pitch 和 body twist 保留。宿主快照使用 mutex；仿真桥用同一 core lock 串行运行导航、控制和 service 回调。

SIL 仿真桥直接提供 canonical `setorigin`、`ZitOdom` 与 ARM 心跳入口，上电为 disarmed。ARM 在成功设置过原点后还需要至少 10 次心跳、持续至少 1 秒；1 秒心跳超时上锁并保留原点。localization 只适配 ZIT6 odom，BasicMotion 只发送该 odom 目标。

DVL/IMU 的 bootstrap 积分位于仿真后端 `RawNavigation`，其 nav 连续，不因作业原点重设而清零。HIL 经 hw_manager 向 MCU 发布同一 raw nav，并在 disarmed 时设置 `simulation.hitl_enabled=false`、`simulation.sitl_enabled=true`，选择外部 Stonefish 导航。SIL/HIL 自动任务等待导航、原点服务和 BasicMotion 都就绪后再启动。
