# 硬件在环 (HIL) 仿真系统

## 概述

硬件在环仿真 (HIL, Hardware-in-the-Loop) 是在算法在环仿真 (SIL) 的基础上，将 PID 控制、推力混合、ZIT6 状态机等逻辑从 Host PC 上的 `sim_bridge` 节点迁移至真实的 STM32 MCU 执行。MCU 通过 **micro-ROS** 串行链路与 ROS 2 系统通信，运行与实机完全相同的固件。

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
│  │ micro-ROS   │  │  sim_bridge │     │              │            │
│  │ Agent       │  │  HIL 模式   │     │              │            │
│  │MicroXRCEAgent│ │  仅相机转发  │     │              │            │
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
│  │    /auv/imu            (Imu, 20Hz)                        │    │
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
│    /zit6/state/pos      (Float32MultiArray[4]) — 位置            │
│    /zit6/state/vel      (Float32MultiArray[4]) — 速度            │
│    /zit6/state/thr      (Float32MultiArray[4]) — 当前力          │
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

MCU 通过 micro-ROS 订阅以下主题，获取仿真传感器数据和控制指令：

| 主题                   | 消息类型                        | 频率  | 说明                                                                                       |
| ---------------------- | ------------------------------- | ----- | ------------------------------------------------------------------------------------------ |
| `/zit6/cmd/setpoint` | `zit6_interfaces/ZitSetpoint` | 按需  | 运动设定值；`control_key` 决定 POS/VEL/FORCE 模式；`type_mask` 决定忽略哪些轴 (1=忽略) |
| `/auv/odometry`      | `nav_msgs/Odometry`           | 30 Hz | Stonefish 真值里程计；`pose` = NED 世界坐标系位姿；`twist` = 世界坐标系线速度 + 角速度 |
| `/auv/imu`           | `sensor_msgs/Imu`             | 20 Hz | 角速度 (用于替代真实 IMU 的 gyro 数据)                                                     |
| `/auv/dvl`           | `stonefish_ros2/DVL`          | 20 Hz | 4 波束 DVL 机体坐标系速度                                                                  |
| `/zit6/cmd/agxhbt`   | `std_msgs/UInt32`             | 10 Hz | 心跳 + 布防：`1` = 正常布防 (需 INS 就绪)，`3` = 强制布防 (绕过 INS)                   |
| `/zit6/cmd/ins`      | `std_msgs/UInt8`              | 按需  | INS 控制:`1` = 启用 DVL，`2` = 禁用 DVL，`3` = 重新对准                              |
| `/zit6/cmd/light`    | `std_msgs/UInt8`              | 按需  | LED:`1` = 红，`2` = 黄，`3` = 绿                                                     |

**带宽估算**: Odometry ~200B×30 + IMU ~150B×20 + DVL ~200B×20 ≈ **14 KB/s**

### 2.3 上行通信 (MCU → ROS 2)

MCU 通过 micro-ROS 发布以下主题：

| 主题                   | 消息类型                       | 频率  | 说明                                                                     |
| ---------------------- | ------------------------------ | ----- | ------------------------------------------------------------------------ |
| `/auv/thrusters_cmd` | `std_msgs/Float64MultiArray` | 60 Hz | 6 路归一化推力值`[-1.0, 1.0]`，直接驱动 Stonefish 推进器               |
| `/zit6/state/status` | `zit6_interfaces/ZitStatus`  | 10 Hz | 布防状态、控制模式、INS 状态、力输出、电池电压、错误标志                 |
| `/zit6/state/pos`    | `std_msgs/Float32MultiArray` | 30 Hz | 地图坐标系位置`[x, y, z, yaw_rad]`；`uv_control` 依赖此话题          |
| `/zit6/state/vel`    | `std_msgs/Float32MultiArray` | 60 Hz | 机体坐标系速度`[vx, vy, vz, vyaw_rate_rad]`；`uv_control` 依赖此话题 |
| `/zit6/state/thr`    | `std_msgs/Float32MultiArray` | 30 Hz | 当前施加的 4-DOF 力`[Fx, Fy, Fz, Mz]`                                  |
| `/zit6/state/zithbt` | `std_msgs/UInt32`            | 10 Hz | 心跳序列号，用于上层心跳监控                                             |

**带宽估算**: thrusters_cmd 约 50B×60 + status 约 100B×10 + pos 约 50B×30 + vel 约 50B×60 ≈ **8 KB/s**

**总带宽**: 上下行合计约 22 KB/s，远低于 921600 baud (≈92 KB/s) 的理论上限，串口链路充足。

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

| 组件          | SIL (`sim_bridge` 正常模式)              | HIL (`sim_bridge hil_mode:=True`) |
| ------------- | ------------------------------------------ | ----------------------------------- |
| PID 控制      | Python`_Pid` 数据类                      | STM32 C 实现                        |
| 推力混合      | `_publish_thrust_from_4dof()`            | STM32 固件                          |
| ZIT6 状态发布 | sim_bridge 发布`/zit6/state/*`           | MCU 发布`/zit6/state/*`           |
| INS 对准      | 模拟 (开机 1.5s 自动完成)                  | MCU 实际执行                        |
| ARM/安全      | 注释跳过 (始终布防)                        | MCU 实际执行心跳 + 布防逻辑         |
| PID 参数服务  | sim_bridge`/zit6/get_params`             | MCU 提供 (micro-ROS service)        |
| 相机转发      | sim_bridge                                 | sim_bridge (相同)                   |
| 上层接口      | `/zit6/cmd/setpoint` + `/zit6/state/*` | **完全相同 — 上层透明**      |

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

- MCU 须在检测到心跳超时后自动 Disarm (当前 `sim_bridge` 中此检查被注释)
- 串口断开 → 通信超时 → 安全状态 (推力归零)
- 重连后需重新发送布防命令

---

## 六、启动方式

### SIL 模式 (不变)

```bash
cd sim_ws && source install/setup.bash
ros2 launch uv_bringup sim_bringup.py
```

### HIL 模式

```bash
cd sim_ws && source install/setup.bash
ros2 launch uv_bringup hil_bringup.py serial_dev:=/dev/ttyUSB0
```

Launch 参数:

| 参数            | 默认值                    | 说明               |
| --------------- | ------------------------- | ------------------ |
| `serial_dev`  | `/dev/ttyUSB0`          | MCU 串口设备路径   |
| `serial_baud` | `921600`                | 串口波特率         |
| `enable_ai`   | `true`                  | 启用感知节点       |
| `enable_nav`  | `true`                  | 启用导航节点       |
| `enable_task` | `false`                 | 启用任务执行器     |
| `scenario`    | `underwater_xunyun.scn` | Stonefish 场景文件 |

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
