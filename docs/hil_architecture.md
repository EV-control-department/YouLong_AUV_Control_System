# 硬件在环 (HIL) 仿真系统

> ## ⏸ 当前阶段：只做软件在环（SIL）
>
> 本文档描述 HIL 的既有设计与实现，**保留备用**，但**不参与当前验收**。
> 话题/类型的权威已迁至 `docs/interfaces/topic_contract.md`（§8 为 HIL 专属行，标 ⏸）。
> 本文档只保留 HIL 独有的内容：micro-ROS 链路、控制数据流、MCU 固件注意事项、启动与验证。
> 待 HIL 重新纳入范围时，再按 `roadmap.md` 的 T7 恢复逐项核对。

## 概述

硬件在环仿真 (HIL, Hardware-in-the-Loop) 是在算法在环仿真 (SIL) 的基础上，将 PID 控制、推力混合、ZIT6 状态机等逻辑从 Host PC 上的仿真 MCU（T5 之前是单体 `sim_bridge`，现在是 `uv_sim` 的 `mcu_sim`）迁移至真实的 STM32 MCU 执行。MCU 通过 **micro-ROS** 串行链路与 ROS 2 系统通信，运行与实机完全相同的固件。

SIL 和 HIL 共享同一套上层控制栈（`uv_control` → `uv_perception` → `uv_nav` → `uv_task`），两者的区别仅在硬件抽象层。

---

## 一、系统组成

```
┌──────────────────────────────────────────────────────────────────┐
│                     HOST PC (Ubuntu + ROS 2 Jazzy)               │
│                                                                   │
│  ┌────────────┐  ┌────────────┐  ┌──────────┐  ┌──────────┐     │
│  │ uv_control │  │uv_perception│  │ uv_nav   │  │ uv_task  │     │
│  │ basic_mtn  │  │ vision/pos │  │ navigator│  │ runner   │     │
│  └─────┬──────┘  └─────┬──────┘  └────┬─────┘  └────┬─────┘     │
│        │               │              │              │            │
│        │ /zit6/cmd/    │ /auv/front_  │              │            │
│        │   setpoint    │   cam/*      │              │            │
│        ▼               ▲              │              │            │
│  ┌─────────────┐  ┌─────────────┐     │              │            │
│  │ micro-ROS   │  │ camera_relay│     │              │            │
│  │ Agent       │  │ + mcu_sim   │     │              │            │
│  │MicroXRCEAgent│ │ (推力转发)   │     │              │            │
│  └──────┬──────┘  └──────┬──────┘     │              │            │
│         │                │            │              │            │
│         │ serial (UART)  │            │              │            │
│         │ /dev/ttyUSB0   │            │              │            │
│         │ 921600 baud    │            │              │            │
│         │                │            │              │            │
│  ┌──────┴────────────────┴────────────┴──────────────┴──────┐    │
│  │                   Stonefish Simulator                     │    │
│  │                                                           │    │
│  │  发布:                                                    │    │
│  │    /auv/odometry      (Odometry, 30Hz)                    │    │
│  │    /auv/imu            (Imu, 200Hz)                       │    │
│  │    /auv/dvl            (DVL, 20Hz)                        │    │
│  │    /auv/pressure       (FluidPressure, 5Hz)               │    │
│  │    /sim/front_cam/left/image_color  (Image)              │    │
│  │    /sim/front_cam/right/image_color (Image)              │    │
│  │    /sim/down_cam/left/image_color   (Image)              │    │
│  │    /sim/down_cam/right/image_color  (Image)              │    │
│  │                                                           │    │
│  │  订阅:                                                    │    │
│  │    /auv/thrusters_cmd (Float64MultiArray[6], 60Hz)       │    │
│  └───────────────────────────────────────────────────────────┘    │
│                                                                   │
└──────────────────────────────────────────────────────────────────┘
          │                              ▲
          │   串口 (UART / USB CDC)       │
          │   XRCE-DDS 协议              │
          ▼                              │
┌──────────────────────────────────────────────────────────────────┐
│                       MCU (STM32F4/F7/H7)                        │
│                                                                   │
│  micro-ROS Client (rmw_micro_xrcedds + FreeRTOS)                 │
│                                                                   │
│  订阅的 ROS 2 主题:                                               │
│    /zit6/cmd/setpoint   (ZitSetpoint)  — 运动设定值             │
│    /auv/odometry        (Odometry)      — 位姿 + 世界系速度     │
│    /auv/imu             (Imu)           — 角速度                │
│    /auv/dvl             (DVL)           — 机体速度              │
│    /zit6/cmd/agxhbt     (UInt32)        — 心跳/布防             │
│    /zit6/cmd/ins        (UInt8)         — INS 控制              │
│    /zit6/cmd/light      (UInt8)         — LED 控制              │
│                                                                   │
│  发布的 ROS 2 主题:                                               │
│    /auv/thrusters_cmd   (Float64MultiArray[6]) — 推力指令       │
│    /zit6/state/status   (ZitStatus)          — 系统状态          │
│    /zit6/state/pos      (Float32MultiArray[6]) — 位姿            │
│    /zit6/state/vel      (Float32MultiArray[6]) — 机体速度        │
│    /zit6/state/thr      (Float32MultiArray[6]) — 当前力          │
│    /zit6/state/zithbt   (UInt32)              — 心跳             │
│                                                                   │
│  提供的服务:                                                       │
│    /zit6/get_params     (GetParams)    — 读取 PID 参数          │
│    /zit6/update_params  (UpdateParams) — 在线更新 PID 参数      │
│                                                                   │
└──────────────────────────────────────────────────────────────────┘
```

---

## 二、通信接口详解

### 2.1 micro-ROS 架构

micro-ROS 采用 **Client-Agent** 拓扑：

- **Client** (`rmw_micro_xrcedds`) 运行在 MCU 上，通过 UART 收发 XRCE-DDS 消息
- **Agent** (`MicroXRCEAgent`) 运行在 Host PC 上，桥接串口与 DDS 网络

启动 Agent（由 launch 文件自动执行）：

```bash
MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600
```

### 2.2 下行通信 (ROS 2 → MCU)

**话题清单已收敛到唯一事实源**：见 `docs/interfaces/topic_contract.md` §3（控制与执行机构）
与 §1（仿真传感器输入）。MCU 端经 micro-ROS 订阅的就是那张表里标 `SIL`/`⏸HIL` 的下行话题：
`/zit6/cmd/{setpoint,agxhbt,ins,light,servo}` 与传感器侧 `/auv/{odometry,imu,dvl,pressure}`。

（此前这里维护过一张 7 行的话题表，与本文件外的文档重复；2026-09-16 T3a 删除，改为链接。）

**带宽估算**: Odometry ~200B×30 + IMU ~150B×200 + DVL ~200B×20 ≈ **40 KB/s**
（IMU 200 Hz 后约为原来 3 倍；921600 baud 理论上限 ≈92 KB/s，但未计 micro-ROS 每帧开销。
若 HIL 实测拥塞，可在 T5 的 `imu_sim` 侧对 MCU 通道降采样。）

### 2.3 上行通信 (MCU → ROS 2)

同样见 `docs/interfaces/topic_contract.md` §4（状态反馈）。

**带宽估算**: thrusters_cmd ~50B×60 + status ~100B×10 + pos ~60B×30 + vel ~60B×50 ≈ **8 KB/s**

**总带宽**: 上下行合计约 22 KB/s（早期版本按 IMU 20 Hz 计算）至 **~48 KB/s**（IMU 200 Hz 后），
均低于 921600 baud 的理论上限；链路余量需在 HIL 会话中用实测帧率复核。

---

## 三、控制数据流

### 3.1 闭环控制流程 (位置模式为例)

```
uv_control/basic_motion
    │
    │  /zit6/cmd/setpoint (ZitSetpoint)
    │  control_key=0 (CK_POS), type_mask=0
    │  x, y, z, yaw (目标位姿, NED 地图坐标系)
    │
    ▼
micro-ROS Agent ──── serial ────→ MCU micro-ROS Client
                                      │
                                      │  控制回调更新 target_pos
                                      ▼
                              control_task (100Hz)
                                │
                                │  世界坐标系误差 → 机体坐标系误差 (yaw 旋转)
                                │
                                ├── pid_yaw: eyaw → target_yaw_rate (级联外环)
                                ├── pid_yaw_rate: 角速度误差 → Mz
                                ├── pid_x: ex_body → Fx
                                ├── pid_y: ey_body → Fy
                                ├── pid_z: ez (带浮力前馈) → Fz
                                │
                                │  推力混合 (xunyun 6 推进器):
                                │    T0 =  (Fx + Fy + Mz×0.5) / MAX_T  (艉右舷)
                                │    T1 =  (Fx - Fy - Mz×0.5) / MAX_T  (艉左舷)
                                │    T2 =   Fz / MAX_T                  (垂荡艏)
                                │    T3 =   Fz / MAX_T                  (垂荡艉)
                                │    T4 = -(Fx - Fy + Mz×0.5) / MAX_T  (艏右舷)
                                │    T5 = -(Fx + Fy - Mz×0.5) / MAX_T  (艏左舷)
                                │
                                ▼
                              /auv/thrusters_cmd (Float64MultiArray[6])
                                      │
MCU micro-ROS Client ──── serial ────→ micro-ROS Agent
                                          │
                                          ▼
                                   Stonefish Simulator
                                     │  推进器动力学 → 水动力计算 → 新位姿
                                     │
                                     ▼
                              /auv/odometry, /auv/imu, /auv/dvl
                                          │
micro-ROS Agent ──── serial ────→ MCU (传感器回调更新缓存)
                                      │
                              同时 MCU 发布:
                                /zit6/state/pos   ──→ uv_control 读取位置反馈
                                /zit6/state/vel   ──→ uv_control 读取速度反馈
                                /zit6/state/status ──→ uv_control 读取状态
```

### 3.2 控制模式

MCU 通过 `ZitSetpoint.control_key & 0x03` 解析控制模式：

| control_key | 模式           | 说明                                                                         |
| ----------- | -------------- | ---------------------------------------------------------------------------- |
| `0`       | POS (位置模式) | 目标位姿 → 级联 PID (yaw 外环 → yaw_rate 内环) → 机体坐标系力 → 推力混合 |
| `1`       | VEL (速度模式) | 目标机体速度 → 速度环 PID → 机体坐标系力 → 推力混合                       |
| `2`       | FORCE (力模式) | 直接 4-DOF 力 → 推力混合 (无 PID)                                           |

`type_mask` 位掩码语义 (**逆逻辑：1 = 忽略该轴**):

| 位   | 轴  | 置 1 时含义     |
| ---- | --- | --------------- |
| bit0 | x   | 忽略 x 设定值   |
| bit1 | y   | 忽略 y 设定值   |
| bit2 | z   | 忽略 z 设定值   |
| bit3 | yaw | 忽略 yaw 设定值 |

---

## 四、与 SIL 的对比

> ⚠️ **已知缺口 — navigator 的位姿源**：`navigator.py` 目前订阅 `uv_msgs/AuvState` 的 `/auv/state`，
> 但该话题在 SIL 与 HIL 下**都没有发布者**（真值里程计已改走 `/zit6/state/pos` 与 `/zit6/sim/nav`）。
> 因此在两种模式下导航器都拿不到自身位姿，A* 规划不可用。规划中的修法是改用
> `/basic_motion/pose_info`（仿真/实机通用）。在此之前，`real_bringup.py` 虽默认 `enable_nav:=true`，
> 也只是拉起一个空转节点。

| 组件          | SIL（`mcu_sim` 正常模式 + `nav_sim`）    | HIL（`hil_mode:=true`）             |
| ------------- | ------------------------------------------ | ----------------------------------- |
| PID 控制      | Python `uv_sim/mcu_control.py` 的 `Pid`   | STM32 C 实现                        |
| 推力混合      | `uv_sim/mixer.py` 的 `mix_thruster_forces()` | STM32 固件                        |
| ZIT6 状态发布 | `nav_sim` 发 pos/vel、`mcu_sim` 发 status/thr/zithbt | MCU 发布**全部** `/zit6/state/*` |
| INS 对准      | 模拟（开机 1.5s 自动完成）                  | MCU 实际执行                        |
| ARM/安全      | 心跳超时检查未实现（始终布防）              | MCU 实际执行心跳 + 布防逻辑         |
| PID 参数服务  | `mcu_sim` 提供 `/zit6/get_params`          | MCU 提供 (micro-ROS service)        |
| 相机转发      | `camera_relay`                             | `camera_relay`（相同）              |
| 上层接口      | `/zit6/cmd/setpoint` + `/zit6/state/*` | **完全相同 — 上层透明**      |

> HIL 下 `dvl_sim` / `pressure_sim` **不启动**（真 MCU 直接订阅 `/auv/dvl`、`/auv/pressure`），
> `nav_sim` 只发 `/zit6/sim/nav`（`publish_state:=false`），`mcu_sim` 只订阅
> `/zit6/state/thr` 并转发成 `/auv/thrusters_cmd`。见 `docs/interfaces/topic_contract.md` §8。

---

## 五、MCU 固件注意事项

### 5.1 HIL 模式编译

MCU 固件应通过编译宏区分数据源，使 HIL 与实机共用同一份代码：

```c
#ifdef HIL_MODE
  // 从 micro-ROS 订阅获取传感器数据
  // /auv/odometry → pos + vel
  // /auv/imu → angular_velocity
  // /auv/dvl → body_velocity
#else
  // 从物理传感器 (SPI/I2C/UART) 读取
  // IMU (ICM-20948), DVL, 深度计等
#endif
```

### 5.2 坐标系

- 所有通信使用 **NED** (North-East-Down) 坐标系
- 位置主题 `/zit6/state/pos` 为**地图坐标系**
- 速度主题 `/zit6/state/vel` 为**机体坐标系**
- yaw = 0 指向北，**顺时针为正**
- 内部角度用**度**存储，ROS 消息用**弧度**

### 5.3 安全机制

- MCU 须在检测到心跳超时后自动 Disarm（SIL 侧 `uv_sim/mcu_control.py` 里此检查未实现：始终布防）
- 串口断开 → 通信超时 → 安全状态 (推力归零)
- 重连后需重新发送布防命令

---

## 六、启动方式

### SIL 模式 (不变)

```bash
cd sim_ws && source install/setup.bash
ros2 launch uv_sim_bringup sim_bringup.py
```

### HIL 模式

```bash
cd sim_ws && source install/setup.bash
ros2 launch uv_sim_bringup hil_bringup.py serial_dev:=/dev/ttyUSB0
```

Launch 参数:

| 参数            | 默认值                    | 说明               |
| --------------- | ------------------------- | ------------------ |
| `serial_dev`  | `/dev/ttyUSB0`          | MCU 串口设备路径（转发给 `uv_hm/hm_launch.py`） |
| `serial_baud` | `921600`                | 串口波特率         |
| `enable_agent` | `true`                 | 启动 micro-ROS agent |
| `enable_ai`   | `false`                 | 启用感知节点       |
| `enable_nav`  | `false`                 | 启用导航节点       |
| `enable_task` | `false`                 | 启用任务执行器     |
| `enable_motion` | `false`               | 启用 basic_motion  |
| `scenario_desc` | `underwater_xunyun.scn` | 场景文件名（在 `Data/` 内；**T4 起真正生效**） |
| `headless`    | `false`                 | `true` = console 引擎（无渲染、无相机图像） |
| `simulation_rate` | `100.0`             | 物理步进率；⚠️ **必须给正数**——给 `0` 会让仿真循环停住、一条话题都不发（2026-09-16 实测更正，早先这里写反了） |

> ⚠️ HIL 默认上层节点为 `false`（launch 只拉起 MCU 链路与相机），需要显式 `enable_ai:=true` 等开启。
> T4 修正（**静态核查通过，运行时未验证**）：`scenario_desc` 以前声明了却没传进仿真器
> `arguments`（改它不换场景），现在会真正传下去；`headless`/`simulation_rate` 取代了原先写死的
> GPU 版与 `100.0`；micro-ROS agent 与 `hw_manager` 统一由 `uv_hm/hm_launch.py` 提供
> （因此 HIL 会起 `hw_manager`——真 MCU 需要 `/zit6/cmd/agxhbt` 心跳才会解锁）。
> T5 变化（**同样只做静态核查**）：`hil_mode` 现在是 `bridge_launch.py` 的**组合开关**，
> HIL 起 5 个节点（`odom_sim`/`imu_sim`/`nav_sim`/`mcu_sim`/`camera_relay`），
> 参数含义见本文件"与 SIL 的对比"一节与 `docs/interfaces/launch_entries.md`。

---

## 七、验证步骤

1. **通信链路**: 启动 HIL 栈，确认 Agent 端输出 `session established`；`ros2 topic list` 确认 MCU 发布/订阅的主题齐全
2. **传感器数据**: `ros2 topic echo /zit6/state/status` 确认 MCU 正确转发状态
3. **闭环控制**: 通过 action 发送目标位置，观察 AUV 到位：
   ```bash
   ros2 action send_goal /basic_motion uv_msgs/action/BasicMotion \
     "{cmd_type: 3, target: [5.0, 0.0, -2.0, 0.0]}"
   ```
4. **SIL/HIL 对比**: 相同初始条件和设定值，对比两条轨迹。差异应在浮点精度范围内
5. **异常恢复**: 拔掉串口 → MCU disarm → 插件串口 → 重新布防 → 恢复正常控制
