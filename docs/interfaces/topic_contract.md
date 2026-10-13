# 话题契约（唯一事实源）

> **本文件是话题/消息类型的唯一权威。** 其他文档（`hil_architecture.md`、`node_parameters.md`、
> `docs/architecture/*`）**不得**再复制话题表，只能链接到这里。
>
> 校验：`python3 scripts/check_docs_contract.py --check`（不需要 ROS，纯源码/场景比对）
> ＋ `test_scene_publisher_types.py`（场景声明发布者的**类型**对 C++ 证据，T3b 加）
>
> 当前阶段：**只做软件在环（SIL）**。HIL/实机行保留在册但标 ⏸，不参与 SIL 验收。
>
> **相邻事实源**（别在本文件里复制）：
> - 频率与 QoS → `docs/interfaces/topic_rates.md`（实测，带快照模式）
> - 服务/动作的**字段语义** → `docs/interfaces/service_action_contract.md`
> - "哪个入口起哪些节点" → `docs/interfaces/launch_entries.md`

## 表格格式（脚本按此解析，勿改列名/顺序）

```markdown
| 话题 | 消息类型 | 方向 | 发布者 | 订阅者 | 模式 | 证据 | 说明 |
```

- **方向**：`发布` / `订阅` / `服务`（一行只描述一个方向；同一话题双向就写两行）
- **模式**：`SIL` / `HIL` / `两者` / `⏸HIL`
- **证据**：`相对路径:create_publisher|create_subscription|create_service|ros_publisher`，
  多个用 `;` 分隔。**不用行号**（会漂）。脚本校验：该文件里确实出现「该话题串」与「该消息类型符号」。
- **话题模板**：代码里是 f-string 的（`/perception/detection/{ch}`），本表**展开为具体话题**；
  脚本对 `{ch}` / `{cam}` 做反查（已知 4 通道固定为 `front_left/front_right/down_left/down_right`，
  task_runner 只订阅下视两路）。
- **场景来源**：Stonefish 的 `<ros_publisher topic=...>` 声明在 `.scn` 里；相机话题场景写作
  `/sim/front_cam/left`，而 ROS 实际话题带 `/image_color` 后缀，脚本对此外缀白名单放行。

## 1. SIL：仿真传感器输入（Stonefish → 转发层 → 栈）

> **T5 起有转发层**：`/auv/*` 是**引擎**的话题（引擎/场景变它就会变），`/sim/sensor/*` 是
> **我们自己的**话题（T6 的 AQUA-SLAM 与 MCU 契约都消费它）。`odom_sim` / `imu_sim` /
> `dvl_sim` / `pressure_sim` 只做「引擎话题 → 我们的话题」的转发与类型适配，
> **不改时间戳**（转发不该伪造新样本）。

| 话题 | 消息类型 | 方向 | 发布者 | 订阅者 | 模式 | 证据 | 说明 |
|---|---|---|---|---|---|---|---|
| `/auv/odometry` | `nav_msgs/Odometry` | 发布 | `stonefish_simulator` | — | 两者 | `workspace_sim/src/stonefish_ros2/Data/xunyun_fixed.scn:ros_publisher` | 真值里程计；`pose` 在 `world_ned`，**`twist` 在机体系** |
| `/auv/odometry` | `nav_msgs/Odometry` | 订阅 | — | `odom_sim` | 两者 | `workspace_sim/src/uv_sim/uv_sim/odom_sim.py:create_subscription` | 原样转发（含 header） |
| `/sim/sensor/odom` | `nav_msgs/Odometry` | 发布 | `odom_sim` | — | 两者 | `workspace_sim/src/uv_sim/uv_sim/odom_sim.py:create_publisher` | 转发层输出 |
| `/sim/sensor/odom` | `nav_msgs/Odometry` | 订阅 | — | `nav_sim` | 两者 | `workspace_sim/src/uv_sim/uv_sim/nav_sim.py:create_subscription` | 状态/注入的位姿源 |
| `/sim/sensor/odom` | `nav_msgs/Odometry` | 订阅 | — | `mcu_sim` | SIL | `workspace_sim/src/uv_sim/uv_sim/mcu_sim.py:create_subscription` | PID 反馈（**不读自己的 `/zit6/state/*`**） |
| `/auv/imu` | `sensor_msgs/Imu` | 发布 | `stonefish_simulator` | — | 两者 | `workspace_sim/src/stonefish_ros2/Data/xunyun_fixed.scn:ros_publisher` | 200 Hz，含噪声；输出比力 |
| `/auv/imu` | `sensor_msgs/Imu` | 订阅 | — | `imu_sim` | 两者 | `workspace_sim/src/uv_sim/uv_sim/imu_sim.py:create_subscription` | 原样转发 |
| `/sim/sensor/imu` | `sensor_msgs/Imu` | 发布 | `imu_sim` | — | 两者 | `workspace_sim/src/uv_sim/uv_sim/imu_sim.py:create_publisher` | |
| `/sim/sensor/imu` | `sensor_msgs/Imu` | 订阅 | — | `nav_sim` | 两者 | `workspace_sim/src/uv_sim/uv_sim/nav_sim.py:create_subscription` | 角速度（200 Hz 覆盖 odom 的 30 Hz） |
| `/sim/sensor/imu` | `sensor_msgs/Imu` | 订阅 | — | `mcu_sim` | SIL | `workspace_sim/src/uv_sim/uv_sim/mcu_sim.py:create_subscription` | 偏航速率环反馈 |
| `/auv/dvl` | `stonefish_ros2/DVL` | 发布 | `stonefish_simulator` | — | 两者 | `workspace_sim/src/stonefish_ros2/Data/xunyun_fixed.scn:ros_publisher` | 4 波束，**机体系**速度（θ=30° 离竖直）；`beams` 恒空 |
| `/auv/dvl` | `stonefish_ros2/DVL` | 订阅 | — | `dvl_sim` | SIL | `workspace_sim/src/uv_sim/uv_sim/dvl_sim.py:create_subscription` | |
| `/sim/sensor/dvl` | `uv_msgs/Dvl` | 发布 | `dvl_sim` | — | SIL | `workspace_sim/src/uv_sim/uv_sim/dvl_sim.py:create_publisher` | **AQUA-SLAM 适配器**：透传 + 合成 4 波束；速度是 **DVL 处**的值（含 `ω×r_dvl`），与 `/zit6/state/vel` 的**原点**值不同（B19） |
| `/auv/dvl_altitude` | `sensor_msgs/Range` | 发布 | `stonefish_simulator` | — | 两者 | `workspace_sim/src/stonefish_ros2/Data/xunyun_fixed.scn:ros_publisher` | 20 Hz 底距（`sensor_msgs/Range`，`radiation_type=ULTRASOUND`、`field_of_view=2·beam_angle`；`ROS2Interface.cpp:PublishDVLAltitude`）。**当前无订阅者**（`/auv/dvl.altitude` 已带同一信息）；2026-10-10 实测更正：此前本表写 `std_msgs/Float32` |
| `/auv/pressure` | `sensor_msgs/FluidPressure` | 发布 | `stonefish_simulator` | — | 两者 | `workspace_sim/src/stonefish_ros2/Data/xunyun_fixed.scn:ros_publisher` | 5 Hz，`variance` 由场景噪声模型填 |
| `/auv/pressure` | `sensor_msgs/FluidPressure` | 订阅 | — | `pressure_sim` | SIL | `workspace_sim/src/uv_sim/uv_sim/pressure_sim.py:create_subscription` | |
| `/sim/sensor/pressure` | `sensor_msgs/FluidPressure` | 发布 | `pressure_sim` | — | SIL | `workspace_sim/src/uv_sim/uv_sim/pressure_sim.py:create_publisher` | 当前无订阅者（见 §9） |

### 1.1 SIL：线激光链路（Stonefish → laser_sim → 栈）

场景把线激光声明为 Stonefish 的 `type="multibeam"`（传感器名 `line_laser`）：**一个平面上的射线扇形 = 海床上的一根线**。
它是纯 CPU 射线求交，所以 console(无头) 模式也能跑（相机不能）。外参见 `docs/frames/extrinsics_reference.md`。

| 话题 | 消息类型 | 方向 | 发布者 | 订阅者 | 模式 | 证据 | 说明 |
|---|---|---|---|---|---|---|---|
| `/auv/laser_scan` | `sensor_msgs/LaserScan` | 发布 | `stonefish_simulator` | — | 两者 | `workspace_sim/src/stonefish_ros2/Data/xunyun_fixed.scn:ros_publisher` | 30 Hz，960 条射线（`fov=87.19°`，`steps=959`）；`ranges[i]=inf` + `intensities[i]=0` 表示该射线没打到（引擎的 `rangeMax` 约定），`frame_id` 是**传感器名** `line_laser` |
| `/auv/laser_scan` | `sensor_msgs/LaserScan` | 订阅 | — | `laser_sim` | 两者 | `workspace_sim/src/uv_sim/uv_sim/laser_sim.py:create_subscription` | |
| `/sim/sensor/laser_points` | `sensor_msgs/PointCloud2` | 发布 | `laser_sim` | — | 两者 | `workspace_sim/src/uv_sim/uv_sim/laser_sim.py:create_publisher` | 激光条纹的**三维点**，传感器（激光平面）系、`z≡0`：结构化光测量本身就是"二维剖面 + 已知平面"。丢弃未命中的射线；`attenuation_c>0` 时再按水质随机丢弃远端（见 `node_parameters.md` §laser_sim）。消费见 §7.5 |

## 2. SIL：相机链路（Stonefish → camera_relay → 栈）

| 话题 | 消息类型 | 方向 | 发布者 | 订阅者 | 模式 | 证据 | 说明 |
|---|---|---|---|---|---|---|---|
| `/sim/front_cam/left/image_color` | `sensor_msgs/Image` | 发布 | `stonefish_simulator` | — | SIL | `workspace_sim/src/stonefish_ros2/Data/xunyun_fixed.scn:ros_publisher` | 场景声明为 `/sim/front_cam/left`，ROS 话题带 `/image_color` |
| `/sim/front_cam/right/image_color` | `sensor_msgs/Image` | 发布 | `stonefish_simulator` | — | SIL | `workspace_sim/src/stonefish_ros2/Data/xunyun_fixed.scn:ros_publisher` | 同上 |
| `/sim/down_cam/left/image_color` | `sensor_msgs/Image` | 发布 | `stonefish_simulator` | — | SIL | `workspace_sim/src/stonefish_ros2/Data/xunyun_fixed.scn:ros_publisher` | 同上 |
| `/sim/down_cam/right/image_color` | `sensor_msgs/Image` | 发布 | `stonefish_simulator` | — | SIL | `workspace_sim/src/stonefish_ros2/Data/xunyun_fixed.scn:ros_publisher` | 同上 |
| `/sim/front_cam/left/image_color` | `sensor_msgs/Image` | 订阅 | — | `camera_relay` | SIL | `workspace_sim/src/uv_sim/uv_sim/camera_relay.py:create_subscription` | |
| `/sim/front_cam/right/image_color` | `sensor_msgs/Image` | 订阅 | — | `camera_relay` | SIL | `workspace_sim/src/uv_sim/uv_sim/camera_relay.py:create_subscription` | |
| `/sim/down_cam/left/image_color` | `sensor_msgs/Image` | 订阅 | — | `camera_relay` | SIL | `workspace_sim/src/uv_sim/uv_sim/camera_relay.py:create_subscription` | |
| `/sim/down_cam/right/image_color` | `sensor_msgs/Image` | 订阅 | — | `camera_relay` | SIL | `workspace_sim/src/uv_sim/uv_sim/camera_relay.py:create_subscription` | |
| `/auv/front_cam/left` | `sensor_msgs/Image` | 发布 | `camera_relay` | — | 两者 | `workspace_sim/src/uv_sim/uv_sim/camera_relay.py:create_publisher` | 单目拆分图（原样转发） |
| `/auv/front_cam/right` | `sensor_msgs/Image` | 发布 | `camera_relay` | — | 两者 | `workspace_sim/src/uv_sim/uv_sim/camera_relay.py:create_publisher` | |
| `/auv/down_cam/left` | `sensor_msgs/Image` | 发布 | `camera_relay` | — | 两者 | `workspace_sim/src/uv_sim/uv_sim/camera_relay.py:create_publisher` | |
| `/auv/down_cam/right` | `sensor_msgs/Image` | 发布 | `camera_relay` | — | 两者 | `workspace_sim/src/uv_sim/uv_sim/camera_relay.py:create_publisher` | |
| `/auv/front_cam/stitched` | `sensor_msgs/Image` | 发布 | `camera_relay` | — | 两者 | `workspace_sim/src/uv_sim/uv_sim/camera_relay.py:create_publisher` | 左右并排拼接（宽 = 2×单目宽）；**T5 起每帧只拼一次**（见 `stonefish_notes.md`） |
| `/auv/front_cam/stitched` | `sensor_msgs/Image` | 订阅 | — | `vision` | SIL | `workspace_auv/src/uv_perception/uv_perception/vision.py:create_subscription` | 仅 `sim_mode:=true` 时订阅 |
| `/auv/down_cam/stitched` | `sensor_msgs/Image` | 发布 | `camera_relay` | — | 两者 | `workspace_sim/src/uv_sim/uv_sim/camera_relay.py:create_publisher` | |
| `/auv/down_cam/stitched` | `sensor_msgs/Image` | 订阅 | — | `vision` | SIL | `workspace_auv/src/uv_perception/uv_perception/vision.py:create_subscription` | 仅 `sim_mode:=true` 时订阅 |

## 3. 控制与执行机构

| 话题 | 消息类型 | 方向 | 发布者 | 订阅者 | 模式 | 证据 | 说明 |
|---|---|---|---|---|---|---|---|
| `/auv/thrusters_state` | `stonefish_ros2/ThrusterState` | 发布 | `stonefish_simulator` | — | SIL | `workspace_sim/src/stonefish_ros2/Data/xunyun_fixed.scn:ros_publisher` | 推进器实际状态（场景 `<ros_publisher thrusters=...>`；`ROS2ScenarioParser.cpp:442` 建 `create_publisher<stonefish_ros2::msg::ThrusterState>`）；**实测 99.8 Hz**（= 物理步长），**当前无订阅者**。2026-10-10 实测更正：此前本表写 `std_msgs/Float64MultiArray` |
| `/zit6/cmd/setpoint` | `zit6_interfaces/ZitSetpoint` | 发布 | `basic_motion` | — | SIL | `workspace_auv/src/uv_control/uv_control/basic_motion.py:create_publisher` | 运动设定值；`type_mask` 1=忽略该轴 |
| `/zit6/cmd/setpoint` | `zit6_interfaces/ZitSetpoint` | 订阅 | — | `mcu_sim` | SIL | `workspace_sim/src/uv_sim/uv_sim/mcu_sim.py:create_subscription` | HIL 下由 MCU 经 micro-ROS 订阅 |
| `/auv/thrusters_cmd` | `std_msgs/Float64MultiArray` | 发布 | `mcu_sim` | — | 两者 | `workspace_sim/src/uv_sim/uv_sim/mcu_sim.py:create_publisher` | 6 推进器归一化 [−1, 1]，直接驱动 Stonefish（SIL 由 PID 算；HIL 转发 MCU 的力） |
| `/zit6/cmd/light` | `std_msgs/UInt8` | 发布 | `task_runner` | — | 两者 | `workspace_auv/src/uv_task/uv_task/task_runner.py:create_publisher` | `1`=红 **`2`=绿 `3`=黄**（2026-10-13 更正：此前写 `2`=黄 `3`=绿，与 `task_runner.LIGHT_GREEN=2` / `LIGHT_YELLOW=3` 及实机相反）。**灯语与状态机见 `interfaces/indicator_light.md`** |
| `/zit6/cmd/light` | `std_msgs/UInt8` | 订阅 | — | `mcu_sim` | SIL | `workspace_sim/src/uv_sim/uv_sim/mcu_sim.py:create_subscription` | SIL 下仅记账，不驱动仿真 |
| `/zit6/cmd/servo` | `std_msgs/Float32` | 发布 | `task_runner` | — | 两者 | `workspace_auv/src/uv_task/uv_task/task_runner.py:create_publisher` | 舵机/云台 |
| `/zit6/cmd/servo` | `std_msgs/Float32` | 订阅 | — | `mcu_sim` | SIL | `workspace_sim/src/uv_sim/uv_sim/mcu_sim.py:create_subscription` | |
| `/zit6/cmd/pushrod` | `zit6_interfaces/ZitPushrod` | 发布 | `task_runner` | — | 两者 | `workspace_auv/src/uv_task/uv_task/task_runner.py:create_publisher` | 取样推杆 |
| `/zit6/cmd/agxhbt` | `std_msgs/UInt32` | 发布 | `hw_manager` | — | ⏸HIL | `workspace_auv/src/uv_hm/uv_hm/hw_manager.py:create_publisher` | 心跳 + 布防（`1` 正常 / `3` 强制） |
| `/zit6/cmd/agxhbt` | `std_msgs/UInt32` | 订阅 | — | `mcu_sim` | SIL | `workspace_sim/src/uv_sim/uv_sim/mcu_sim.py:create_subscription` | SIL 下只影响 `arm_mode`（始终布防） |
| `/zit6/cmd/ins` | `std_msgs/UInt8` | 订阅 | — | `mcu_sim` | SIL | `workspace_sim/src/uv_sim/uv_sim/mcu_sim.py:create_subscription` | `1`=启用 DVL `2`=禁用 `3`=重对准 |

## 4. 状态反馈（MCU 契约；SIL 由 `nav_sim` + `mcu_sim` 扮演）

> **单一发布者规则（不变式 I2）**：SIL 下 `/zit6/state/{pos,vel}` 归 `nav_sim`，
> `/zit6/state/{thr,status,zithbt}` 归 `mcu_sim`；HIL/实机下**全部**归真 MCU，
> 此时 `nav_sim` 以 `publish_sim_nav:=true, publish_state:=false` 运行（不创建这两个发布者），
> `mcu_sim` 以 `hil_mode:=true` 运行（只做推力转发，不创建任何 `/zit6/state/*` 发布者）。

| 话题 | 消息类型 | 方向 | 发布者 | 订阅者 | 模式 | 证据 | 说明 |
|---|---|---|---|---|---|---|---|
| `/zit6/state/pos` | `std_msgs/Float32MultiArray` | 发布 | `nav_sim` | — | SIL | `workspace_sim/src/uv_sim/uv_sim/nav_sim.py:create_publisher` | `[x, y, z, roll_rad, pitch_rad, yaw_rad]`（HIL/实机：MCU ⏸） |
| `/zit6/state/pos` | `std_msgs/Float32MultiArray` | 订阅 | — | `basic_motion` | 两者 | `workspace_auv/src/uv_control/uv_control/basic_motion.py:create_subscription` | 兼容 4/6 元素 |
| `/zit6/state/vel` | `std_msgs/Float32MultiArray` | 发布 | `nav_sim` | — | SIL | `workspace_sim/src/uv_sim/uv_sim/nav_sim.py:create_publisher` | `[vx, vy, vz, vroll, vpitch, vyaw]`，**机体系**；线性部分 = **机体原点**处的速度（odometry twist，**不含** DVL 杆臂，见 §9 的保真差距） |
| `/zit6/state/vel` | `std_msgs/Float32MultiArray` | 订阅 | — | `basic_motion` | 两者 | `workspace_auv/src/uv_control/uv_control/basic_motion.py:create_subscription` | |
| `/zit6/state/thr` | `std_msgs/Float32MultiArray` | 发布 | `mcu_sim` | — | SIL | `workspace_sim/src/uv_sim/uv_sim/mcu_sim.py:create_publisher` | 6-DOF 力/力矩（Mx/My 恒 0） |
| `/zit6/state/thr` | `std_msgs/Float32MultiArray` | 订阅 | — | `hw_manager` | ⏸HIL | `workspace_auv/src/uv_hm/uv_hm/hw_manager.py:create_subscription` | 推力饱和监控 |
| `/zit6/state/thr` | `std_msgs/Float32MultiArray` | 订阅 | — | `mcu_sim` | ⏸HIL | `workspace_sim/src/uv_sim/uv_sim/mcu_sim.py:create_subscription` | HIL：转发 MCU 力 → `/auv/thrusters_cmd` |
| `/zit6/state/status` | `zit6_interfaces/ZitStatus` | 发布 | `mcu_sim` | — | SIL | `workspace_sim/src/uv_sim/uv_sim/mcu_sim.py:create_publisher` | 布防/模式/电池/错误标志 |
| `/zit6/state/status` | `zit6_interfaces/ZitStatus` | 订阅 | — | `basic_motion` | 两者 | `workspace_auv/src/uv_control/uv_control/basic_motion.py:create_subscription` | |
| `/zit6/state/status` | `zit6_interfaces/ZitStatus` | 订阅 | — | `hw_manager` | ⏸HIL | `workspace_auv/src/uv_hm/uv_hm/hw_manager.py:create_subscription` | |
| `/zit6/state/status` | `zit6_interfaces/ZitStatus` | 订阅 | — | `task_runner` | 两者 | `workspace_auv/src/uv_task/uv_task/task_runner.py:create_subscription` | |
| `/zit6/state/zithbt` | `std_msgs/UInt32` | 发布 | `mcu_sim` | — | SIL | `workspace_sim/src/uv_sim/uv_sim/mcu_sim.py:create_publisher` | 心跳序号 |
| `/zit6/state/zithbt` | `std_msgs/UInt32` | 订阅 | — | `hw_manager` | ⏸HIL | `workspace_auv/src/uv_hm/uv_hm/hw_manager.py:create_subscription` | 看门狗 |

## 5. 位姿与感知

| 话题 | 消息类型 | 方向 | 发布者 | 订阅者 | 模式 | 证据 | 说明 |
|---|---|---|---|---|---|---|---|
| `/basic_motion/pose_info` | `uv_msgs/PoseInfo` | 发布 | `basic_motion` | — | 两者 | `workspace_auv/src/uv_control/uv_control/basic_motion.py:create_publisher` | 30 Hz，含 roll/pitch（deg），**无 frame 字段** |
| `/basic_motion/pose_info` | `uv_msgs/PoseInfo` | 订阅 | — | `position` | 两者 | `workspace_auv/src/uv_perception/uv_perception/position.py:create_subscription` | 射线原点/姿态 |
| `/basic_motion/pose_info` | `uv_msgs/PoseInfo` | 订阅 | — | `navigator` | 两者 | `workspace_auv/src/uv_nav/uv_nav/navigator.py:create_subscription` | 规划前的就绪条件 |
| `/basic_motion/pose_info` | `uv_msgs/PoseInfo` | 订阅 | — | `task_runner` | 两者 | `workspace_auv/src/uv_task/uv_task/task_runner.py:create_subscription` | |
| `/perception/detection/front_left` | `uv_msgs/DetectionArray` | 发布 | `vision` | — | 两者 | `workspace_auv/src/uv_perception/uv_perception/vision.py:create_publisher` | 4 通道之一（`{ch}` 展开） |
| `/perception/detection/front_right` | `uv_msgs/DetectionArray` | 发布 | `vision` | — | 两者 | `workspace_auv/src/uv_perception/uv_perception/vision.py:create_publisher` | |
| `/perception/detection/down_left` | `uv_msgs/DetectionArray` | 发布 | `vision` | — | 两者 | `workspace_auv/src/uv_perception/uv_perception/vision.py:create_publisher` | |
| `/perception/detection/down_right` | `uv_msgs/DetectionArray` | 发布 | `vision` | — | 两者 | `workspace_auv/src/uv_perception/uv_perception/vision.py:create_publisher` | |
| `/perception/detection/front_left` | `uv_msgs/DetectionArray` | 订阅 | — | `position` | 两者 | `workspace_auv/src/uv_perception/uv_perception/position.py:create_subscription` | |
| `/perception/detection/front_right` | `uv_msgs/DetectionArray` | 订阅 | — | `position` | 两者 | `workspace_auv/src/uv_perception/uv_perception/position.py:create_subscription` | |
| `/perception/detection/down_left` | `uv_msgs/DetectionArray` | 订阅 | — | `position` | 两者 | `workspace_auv/src/uv_perception/uv_perception/position.py:create_subscription` | |
| `/perception/detection/down_right` | `uv_msgs/DetectionArray` | 订阅 | — | `position` | 两者 | `workspace_auv/src/uv_perception/uv_perception/position.py:create_subscription` | |
| `/perception/detection/down_left` | `uv_msgs/DetectionArray` | 订阅 | — | `task_runner` | 两者 | `workspace_auv/src/uv_task/uv_task/task_runner.py:create_subscription` | 任务内联检测（`{cam}` 展开，仅下视两路） |
| `/perception/detection/down_right` | `uv_msgs/DetectionArray` | 订阅 | — | `task_runner` | 两者 | `workspace_auv/src/uv_task/uv_task/task_runner.py:create_subscription` | |
| `/perception/line/front_left` | `uv_msgs/LineState` | 订阅 | — | `task_runner` | 两者 | `workspace_auv/src/uv_task/uv_task/line_follower.py:create_subscription` | 巡线任务；订阅在 `LineFollower(self, p)` 里建立（`task_runner.py:1011`） |
| `/perception/line/down_left` | `uv_msgs/LineState` | 订阅 | — | `task_runner` | 两者 | `workspace_auv/src/uv_task/uv_task/line_follower.py:create_subscription` | 同上 |
| `/perception/aruco/ids` | `std_msgs/Int32MultiArray` | 订阅 | — | `task_runner` | 两者 | `workspace_auv/src/uv_task/uv_task/arrow_surfacer.py:create_subscription` | 上浮任务；订阅在 `ArrowSurfacer(self, p)` 里建立（`task_runner.py:1019`） |
| `/perception/line/front_left` | `uv_msgs/LineState` | 发布 | `vision` | — | 两者 | `workspace_auv/src/uv_perception/uv_perception/vision.py:create_publisher` | 巡线状态（`{ch}` 展开） |
| `/perception/line/front_right` | `uv_msgs/LineState` | 发布 | `vision` | — | 两者 | `workspace_auv/src/uv_perception/uv_perception/vision.py:create_publisher` | |
| `/perception/line/down_left` | `uv_msgs/LineState` | 发布 | `vision` | — | 两者 | `workspace_auv/src/uv_perception/uv_perception/vision.py:create_publisher` | |
| `/perception/line/down_right` | `uv_msgs/LineState` | 发布 | `vision` | — | 两者 | `workspace_auv/src/uv_perception/uv_perception/vision.py:create_publisher` | |
| `/perception/aruco/ids` | `std_msgs/Int32MultiArray` | 发布 | `vision` | — | 两者 | `workspace_auv/src/uv_perception/uv_perception/vision.py:create_publisher` | ArUco ID 列表 |
| `/perception/objects` | `uv_msgs/ObjectPositionArray` | 发布 | `position` | — | 两者 | `workspace_auv/src/uv_perception/uv_perception/position.py:create_publisher` | 3D 世界坐标（含 `instance_id`） |
| `/perception/objects` | `uv_msgs/ObjectPositionArray` | 订阅 | — | `navigator` | 两者 | `workspace_auv/src/uv_nav/uv_nav/navigator.py:create_subscription` | |
| `/perception/objects` | `uv_msgs/ObjectPositionArray` | 订阅 | — | `task_runner` | 两者 | `workspace_auv/src/uv_task/uv_task/task_runner.py:create_subscription` | |

## 6. 任务与导航

| 话题 | 消息类型 | 方向 | 发布者 | 订阅者 | 模式 | 证据 | 说明 |
|---|---|---|---|---|---|---|---|
| `/task/status` | `uv_msgs/TaskStatus` | 发布 | `task_runner` | — | 两者 | `workspace_auv/src/uv_task/uv_task/task_runner.py:create_publisher` | 1 Hz，任务进度 |
| `/nav/path` | `uv_msgs/WaypointPath` | 发布 | `navigator` | — | 两者 | `workspace_auv/src/uv_nav/uv_nav/navigator.py:create_publisher` | 规划出的航点路径 |

## 7. 服务与动作

| 名称 | 类型 | 方向 | 服务端 | 客户端 | 模式 | 证据 | 说明 |
|---|---|---|---|---|---|---|---|
| `/basic_motion` | `uv_msgs/action/BasicMotion` | 动作 | `basic_motion` | `task_runner`、`check_dvl_frame.py` | 两者 | `workspace_auv/src/uv_control/uv_control/basic_motion.py:ActionServer` | 唯一运动入口；START 必须先发 |
| `/zit6/get_params` | `zit6_interfaces/srv/GetParams` | 服务 | `mcu_sim` | — | SIL | `workspace_sim/src/uv_sim/uv_sim/mcu_sim.py:create_service` | 返回全部 PID 通道（HIL：MCU 提供 ⏸） |
| `/zit6/update_params` | `zit6_interfaces/srv/UpdateParams` | 服务 | `mcu_sim` | — | SIL | `workspace_sim/src/uv_sim/uv_sim/mcu_sim.py:create_service` | **只改运行时，不写回 `sim_pid.yaml`** |
| `/nav/navigate_to` | `uv_msgs/srv/RunTask` | 服务 | `navigator` | — | 两者 | `workspace_auv/src/uv_nav/uv_nav/navigator.py:create_service` | 请求执行导航任务 |
| `/task/run` | `uv_msgs/srv/RunTask` | 服务 | `task_runner` | — | 两者 | `workspace_auv/src/uv_task/uv_task/task_runner.py:create_service` | 启动任务/任务列表 |
| `/task/stop` | `std_srvs/srv/Trigger` | 服务 | `task_runner` | — | 两者 | `workspace_auv/src/uv_task/uv_task/task_runner.py:create_service` | 中断当前任务 |
| `/task/exec` | `uv_msgs/srv/ExecTask` | 服务 | `task_runner` | — | 两者 | `workspace_auv/src/uv_task/uv_task/task_runner.py:create_service` | 单条任务执行（**仅 `debug_mode:=true`**） |

> **字段语义**（请求/响应每个字段是什么意思、什么条件下失败）不在本表，见
> `docs/interfaces/service_action_contract.md`（T3b 建立，含引擎自带服务的登记与
> 定义了但无人使用的接口清单）。

> 服务/动作的**请求-响应字段语义**属于 `service_action_contract.md`（T3b 已建立），本表只固定名称与类型。

## 7.5 栈外消费者：可视化 GUI（`visualization/auv_visualizer.py`）

> 该 GUI **不是** colcon 包、没有 entry point，用 `python3 visualization/auv_visualizer.py` 直接跑；
> 登记在此是因为它订阅/发布的话题必须与契约一致。

| 话题 | 消息类型 | 方向 | 发布者 | 订阅者 | 模式 | 证据 | 说明 |
|---|---|---|---|---|---|---|---|
| `/basic_motion/pose_info` | `uv_msgs/PoseInfo` | 订阅 | — | GUI | 两者 | `visualization/auv_visualizer.py:create_subscription` | 2D 地图位姿 |
| `/perception/objects` | `uv_msgs/ObjectPositionArray` | 订阅 | — | GUI | 两者 | `visualization/auv_visualizer.py:create_subscription` | 物体与射线 |
| `/sim/sensor/laser_points` | `sensor_msgs/PointCloud2` | 订阅 | — | GUI | 两者 | `visualization/auv_visualizer.py:create_subscription` | 线激光条纹：2D 地图上画跨航向条纹（含历史留痕）+ 右侧海床剖面（机体系 y ↔ 深度 z） |
| `/task/status` | `uv_msgs/TaskStatus` | 订阅 | — | GUI | 两者 | `visualization/auv_visualizer.py:create_subscription` | 任务进度 |
| `/perception/annotated/front_left` | `sensor_msgs/Image` | 订阅 | **无（缺口）** | GUI | 两者 | `visualization/auv_visualizer.py:create_subscription` | 4 通道；**当前无发布者**，见 §9 |
| `/perception/annotated/front_right` | `sensor_msgs/Image` | 订阅 | **无（缺口）** | GUI | 两者 | `visualization/auv_visualizer.py:create_subscription` | 同上 |
| `/perception/annotated/down_left` | `sensor_msgs/Image` | 订阅 | **无（缺口）** | GUI | 两者 | `visualization/auv_visualizer.py:create_subscription` | 同上 |
| `/perception/annotated/down_right` | `sensor_msgs/Image` | 订阅 | **无（缺口）** | GUI | 两者 | `visualization/auv_visualizer.py:create_subscription` | 同上 |
| `/zit6/cmd/setpoint` | `zit6_interfaces/ZitSetpoint` | 发布 | GUI | — | 两者 | `visualization/auv_visualizer.py:create_publisher` | 遥控面板（与 `basic_motion` 争用同一话题） |

## 8. HIL 专属（⏸ 暂缓，当前阶段不做验收）

| 话题 | 消息类型 | 方向 | 发布者 | 订阅者 | 模式 | 证据 | 说明 |
|---|---|---|---|---|---|---|---|
| `/zit6/sim/nav` | `std_msgs/Float32MultiArray` | 发布 | `nav_sim` | — | ⏸HIL | `workspace_sim/src/uv_sim/uv_sim/nav_sim.py:create_publisher` | 仿真位姿 → MCU：12 float = `pos_world[6]` + `vel_body[6]`（rad / m·s⁻¹ / rad·s⁻¹）；固件 `onSimNav → sitl_nav_queue` |

> micro-ROS 串口链路的上下行全貌（含 MCU 端订阅清单）见 `docs/hil_architecture.md`；
> 该文档在 SIL 阶段仅作参考，**不参与当前验收**。

## 9. 缺口登记（已知且尚未修复，勿当成不存在）

| 缺口 | 现状 | 归属 |
|---|---|---|
| `/perception/annotated/{front,down}_{left,right}` | **有订阅者（GUI `visualization/auv_visualizer.py`）但全局无发布者** | B5，待定 |
| `/auv/state`（`uv_msgs/AuvState`） | **无发布者、无订阅者**（navigator 2026-09-13 已迁到 `PoseInfo`）；消息只剩定义 | 清理待定 |
| `/auv/dvl_altitude` | 有发布者、**无订阅者**（`/auv/dvl.altitude` 已带同一信息） | 待定（T6 可能需要） |
| `/auv/thrusters_state` | 有发布者、**无订阅者**（可用于混控校验，目前没人用） | 待定 |
| `/sim/sensor/dvl` | 有发布者（`dvl_sim`）、**当前无订阅者** | 有意为之：T6 的 AQUA-SLAM 是它的消费者 |
| `/sim/sensor/pressure` | 有发布者（`pressure_sim`）、**当前无订阅者** | 有意为之：补全 `/sim/sensor/*` 面，T6 可能用（深度先验） |
| `/sim/sensor/laser_points` | 有发布者（`laser_sim`）、有订阅者（GUI `visualization/auv_visualizer.py` 做可视化）；**栈内仍无消费者** | 有意为之：线激光海床重建的输入，重建/建图节点由使用者自己写（场景侧 `line_laser` 见 §1.1） |
| `/tf` | **有发布者（引擎 `stonefish_simulator*`），但实测 0 条消息**：场景未声明 `<ros_base_link_transform publish="true"/>`，广播被那个 `if` 守卫关着（2026-10-10 快照实测） | B18，已文档化 |
| `/tf_static` | **话题根本不存在**（不是"存在但没消息"；上述快照里没有它） | B18 的措辞更正 |
| `/basic_motion/pose_info` 的频率 | 设计 30 Hz（`create_timer(1.0/30.0)`），**完整栈里实测 ≈15.6 Hz**（单独跑 `basic_motion` 则 29.9 Hz）⇒ 完整栈下定时器丢一半节拍。影响 `position`/`navigator`/`task_runner` 的位姿新鲜度 | **B20**，见 `topic_rates.md` §2 与 `roadmap.md` |
| 引擎自带服务 `/stonefish_ros2/{enable_currents,disable_currents,respawn_robot}` | 有服务端（引擎），**本表未列**（本表的服务行要求 `create_service` 证据）；登记在 `service_action_contract.md` §9 | 引擎范围 |
| 定义了但无人使用的接口 | `uv_msgs/{AuvState,DeviceCommand,MotionCommand,PidGains,PidState}` 与 `srv/GetState`（0 处引用，2026-10-10 全量 grep 审计）；清单在 `service_action_contract.md` §10 | 清理待定 |
| `/perception/line/{front,down}_left` | 有发布者、有订阅者（`line_follower.py`，仅 `follow_line` 任务运行期间建立）；`front_right`/`down_right` 两路**无订阅者** | 待定 |
| `/zit6/state/vel` 的**物理定义与实机不同**（保真差距，不是 bug） | SIL 下它是 `nav_sim` 从 odometry 取的**机体原点**速度；实机 MCU 的同一话题来自惯导（含 DVL/杆臂与滤波），会漂移也会更平滑 | 有意为之：SIL 控制环不引入 DVL；T6 的 SLAM 走 `/sim/sensor/*` |
