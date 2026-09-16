# 话题契约（唯一事实源）

> **本文件是话题/消息类型的唯一权威。** 其他文档（`hil_architecture.md`、`node_parameters.md`、
> `docs/architecture/*`）**不得**再复制话题表，只能链接到这里。
>
> 校验：`python3 scripts/check_docs_contract.py --check`（不需要 ROS，纯源码/场景比对）
>
> 当前阶段：**只做软件在环（SIL）**。HIL/实机行保留在册但标 ⏸，不参与 SIL 验收。

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

## 1. SIL：仿真传感器输入（Stonefish → 栈）

| 话题 | 消息类型 | 方向 | 发布者 | 订阅者 | 模式 | 证据 | 说明 |
|---|---|---|---|---|---|---|---|
| `/auv/odometry` | `nav_msgs/Odometry` | 发布 | `stonefish_simulator` | — | 两者 | `workspace_sim/src/stonefish_ros2/Data/xunyun_fixed.scn:ros_publisher` | 真值里程计；`pose` 在 `world_ned`，**`twist` 在机体系** |
| `/auv/odometry` | `nav_msgs/Odometry` | 订阅 | — | `sim_bridge` | 两者 | `workspace_sim/src/uv_sim/uv_sim/sim_bridge.py:create_subscription` | SIL 状态估计 / HIL 位姿聚合 |
| `/auv/imu` | `sensor_msgs/Imu` | 发布 | `stonefish_simulator` | — | 两者 | `workspace_sim/src/stonefish_ros2/Data/xunyun_fixed.scn:ros_publisher` | 200 Hz，含噪声；输出比力 |
| `/auv/imu` | `sensor_msgs/Imu` | 订阅 | — | `sim_bridge` | 两者 | `workspace_sim/src/uv_sim/uv_sim/sim_bridge.py:create_subscription` | |
| `/auv/dvl` | `stonefish_ros2/DVL` | 发布 | `stonefish_simulator` | — | 两者 | `workspace_sim/src/stonefish_ros2/Data/xunyun_fixed.scn:ros_publisher` | 4 波束，**机体系**速度（θ=30° 离竖直） |
| `/auv/dvl` | `stonefish_ros2/DVL` | 订阅 | — | `sim_bridge` | 两者 | `workspace_sim/src/uv_sim/uv_sim/sim_bridge.py:create_subscription` | |
| `/auv/dvl_altitude` | `std_msgs/Float32` | 发布 | `stonefish_simulator` | — | 两者 | `workspace_sim/src/stonefish_ros2/Data/xunyun_fixed.scn:ros_publisher` | 底距；**当前无订阅者** |
| `/auv/pressure` | `sensor_msgs/FluidPressure` | 发布 | `stonefish_simulator` | — | 两者 | `workspace_sim/src/stonefish_ros2/Data/xunyun_fixed.scn:ros_publisher` | |
| `/auv/pressure` | `sensor_msgs/FluidPressure` | 订阅 | — | `sim_bridge` | 两者 | `workspace_sim/src/uv_sim/uv_sim/sim_bridge.py:create_subscription` | |

## 2. SIL：相机链路（Stonefish → sim_bridge → 栈）

| 话题 | 消息类型 | 方向 | 发布者 | 订阅者 | 模式 | 证据 | 说明 |
|---|---|---|---|---|---|---|---|
| `/sim/front_cam/left/image_color` | `sensor_msgs/Image` | 发布 | `stonefish_simulator` | — | SIL | `workspace_sim/src/stonefish_ros2/Data/xunyun_fixed.scn:ros_publisher` | 场景声明为 `/sim/front_cam/left`，ROS 话题带 `/image_color` |
| `/sim/front_cam/right/image_color` | `sensor_msgs/Image` | 发布 | `stonefish_simulator` | — | SIL | `workspace_sim/src/stonefish_ros2/Data/xunyun_fixed.scn:ros_publisher` | 同上 |
| `/sim/down_cam/left/image_color` | `sensor_msgs/Image` | 发布 | `stonefish_simulator` | — | SIL | `workspace_sim/src/stonefish_ros2/Data/xunyun_fixed.scn:ros_publisher` | 同上 |
| `/sim/down_cam/right/image_color` | `sensor_msgs/Image` | 发布 | `stonefish_simulator` | — | SIL | `workspace_sim/src/stonefish_ros2/Data/xunyun_fixed.scn:ros_publisher` | 同上 |
| `/sim/front_cam/left/image_color` | `sensor_msgs/Image` | 订阅 | — | `sim_bridge` | SIL | `workspace_sim/src/uv_sim/uv_sim/sim_bridge.py:create_subscription` | |
| `/sim/front_cam/right/image_color` | `sensor_msgs/Image` | 订阅 | — | `sim_bridge` | SIL | `workspace_sim/src/uv_sim/uv_sim/sim_bridge.py:create_subscription` | |
| `/sim/down_cam/left/image_color` | `sensor_msgs/Image` | 订阅 | — | `sim_bridge` | SIL | `workspace_sim/src/uv_sim/uv_sim/sim_bridge.py:create_subscription` | |
| `/sim/down_cam/right/image_color` | `sensor_msgs/Image` | 订阅 | — | `sim_bridge` | SIL | `workspace_sim/src/uv_sim/uv_sim/sim_bridge.py:create_subscription` | |
| `/auv/front_cam/left` | `sensor_msgs/Image` | 发布 | `sim_bridge` | — | 两者 | `workspace_sim/src/uv_sim/uv_sim/sim_bridge.py:create_publisher` | 单目拆分图 |
| `/auv/front_cam/right` | `sensor_msgs/Image` | 发布 | `sim_bridge` | — | 两者 | `workspace_sim/src/uv_sim/uv_sim/sim_bridge.py:create_publisher` | |
| `/auv/down_cam/left` | `sensor_msgs/Image` | 发布 | `sim_bridge` | — | 两者 | `workspace_sim/src/uv_sim/uv_sim/sim_bridge.py:create_publisher` | |
| `/auv/down_cam/right` | `sensor_msgs/Image` | 发布 | `sim_bridge` | — | 两者 | `workspace_sim/src/uv_sim/uv_sim/sim_bridge.py:create_publisher` | |
| `/auv/front_cam/stitched` | `sensor_msgs/Image` | 发布 | `sim_bridge` | — | 两者 | `workspace_sim/src/uv_sim/uv_sim/sim_bridge.py:create_publisher` | 左右并排拼接 |
| `/auv/front_cam/stitched` | `sensor_msgs/Image` | 订阅 | — | `vision` | SIL | `workspace_auv/src/uv_perception/uv_perception/vision.py:create_subscription` | 仅 `sim_mode:=true` 时订阅 |
| `/auv/down_cam/stitched` | `sensor_msgs/Image` | 发布 | `sim_bridge` | — | 两者 | `workspace_sim/src/uv_sim/uv_sim/sim_bridge.py:create_publisher` | |
| `/auv/down_cam/stitched` | `sensor_msgs/Image` | 订阅 | — | `vision` | SIL | `workspace_auv/src/uv_perception/uv_perception/vision.py:create_subscription` | 仅 `sim_mode:=true` 时订阅 |

## 3. 控制与执行机构

| 话题 | 消息类型 | 方向 | 发布者 | 订阅者 | 模式 | 证据 | 说明 |
|---|---|---|---|---|---|---|---|
| `/auv/thrusters_state` | `std_msgs/Float64MultiArray` | 发布 | `stonefish_simulator` | — | SIL | `workspace_sim/src/stonefish_ros2/Data/xunyun_fixed.scn:ros_publisher` | 推进器实际状态（场景 `<ros_publisher thrusters=...>`）；**当前无订阅者** |
| `/zit6/cmd/setpoint` | `zit6_interfaces/ZitSetpoint` | 发布 | `basic_motion` | — | SIL | `workspace_auv/src/uv_control/uv_control/basic_motion.py:create_publisher` | 运动设定值；`type_mask` 1=忽略该轴 |
| `/zit6/cmd/setpoint` | `zit6_interfaces/ZitSetpoint` | 订阅 | — | `sim_bridge` | SIL | `workspace_sim/src/uv_sim/uv_sim/sim_bridge.py:create_subscription` | HIL 下由 MCU 经 micro-ROS 订阅 |
| `/auv/thrusters_cmd` | `std_msgs/Float64MultiArray` | 发布 | `sim_bridge` | — | 两者 | `workspace_sim/src/uv_sim/uv_sim/sim_bridge.py:create_publisher` | 6 推进器归一化 [−1, 1]，直接驱动 Stonefish |
| `/zit6/cmd/light` | `std_msgs/UInt8` | 发布 | `task_runner` | — | 两者 | `workspace_auv/src/uv_task/uv_task/task_runner.py:create_publisher` | `1`=红 `2`=黄 `3`=绿 |
| `/zit6/cmd/light` | `std_msgs/UInt8` | 订阅 | — | `sim_bridge` | SIL | `workspace_sim/src/uv_sim/uv_sim/sim_bridge.py:create_subscription` | SIL 下仅记账，不驱动仿真 |
| `/zit6/cmd/servo` | `std_msgs/Float32` | 发布 | `task_runner` | — | 两者 | `workspace_auv/src/uv_task/uv_task/task_runner.py:create_publisher` | 舵机/云台 |
| `/zit6/cmd/servo` | `std_msgs/Float32` | 订阅 | — | `sim_bridge` | SIL | `workspace_sim/src/uv_sim/uv_sim/sim_bridge.py:create_subscription` | |
| `/zit6/cmd/pushrod` | `zit6_interfaces/ZitPushrod` | 发布 | `task_runner` | — | 两者 | `workspace_auv/src/uv_task/uv_task/task_runner.py:create_publisher` | 取样推杆 |
| `/zit6/cmd/agxhbt` | `std_msgs/UInt32` | 发布 | `hw_manager` | — | ⏸HIL | `workspace_auv/src/uv_hm/uv_hm/hw_manager.py:create_publisher` | 心跳 + 布防（`1` 正常 / `3` 强制） |
| `/zit6/cmd/agxhbt` | `std_msgs/UInt32` | 订阅 | — | `sim_bridge` | ⏸HIL | `workspace_sim/src/uv_sim/uv_sim/sim_bridge.py:create_subscription` | |
| `/zit6/cmd/ins` | `std_msgs/UInt8` | 订阅 | — | `sim_bridge` | ⏸HIL | `workspace_sim/src/uv_sim/uv_sim/sim_bridge.py:create_subscription` | `1`=启用 DVL `2`=禁用 `3`=重对准 |

## 4. 状态反馈（MCU 契约，SIL 由 sim_bridge 扮演）

| 话题 | 消息类型 | 方向 | 发布者 | 订阅者 | 模式 | 证据 | 说明 |
|---|---|---|---|---|---|---|---|
| `/zit6/state/pos` | `std_msgs/Float32MultiArray` | 发布 | `sim_bridge` | — | 两者 | `workspace_sim/src/uv_sim/uv_sim/sim_bridge.py:create_publisher` | `[x, y, z, roll_rad, pitch_rad, yaw_rad]` |
| `/zit6/state/pos` | `std_msgs/Float32MultiArray` | 订阅 | — | `basic_motion` | 两者 | `workspace_auv/src/uv_control/uv_control/basic_motion.py:create_subscription` | 兼容 4/6 元素 |
| `/zit6/state/vel` | `std_msgs/Float32MultiArray` | 发布 | `sim_bridge` | — | 两者 | `workspace_sim/src/uv_sim/uv_sim/sim_bridge.py:create_publisher` | `[vx, vy, vz, vroll, vpitch, vyaw]`，**机体系** |
| `/zit6/state/vel` | `std_msgs/Float32MultiArray` | 订阅 | — | `basic_motion` | 两者 | `workspace_auv/src/uv_control/uv_control/basic_motion.py:create_subscription` | |
| `/zit6/state/thr` | `std_msgs/Float32MultiArray` | 发布 | `sim_bridge` | — | 两者 | `workspace_sim/src/uv_sim/uv_sim/sim_bridge.py:create_publisher` | 6-DOF 力/力矩 |
| `/zit6/state/thr` | `std_msgs/Float32MultiArray` | 订阅 | — | `hw_manager` | ⏸HIL | `workspace_auv/src/uv_hm/uv_hm/hw_manager.py:create_subscription` | 推力饱和监控 |
| `/zit6/state/thr` | `std_msgs/Float32MultiArray` | 订阅 | — | `sim_bridge` | ⏸HIL | `workspace_sim/src/uv_sim/uv_sim/sim_bridge.py:create_subscription` | HIL：转发 MCU 力 → `/auv/thrusters_cmd` |
| `/zit6/state/status` | `zit6_interfaces/ZitStatus` | 发布 | `sim_bridge` | — | 两者 | `workspace_sim/src/uv_sim/uv_sim/sim_bridge.py:create_publisher` | 布防/模式/电池/错误标志 |
| `/zit6/state/status` | `zit6_interfaces/ZitStatus` | 订阅 | — | `basic_motion` | 两者 | `workspace_auv/src/uv_control/uv_control/basic_motion.py:create_subscription` | |
| `/zit6/state/status` | `zit6_interfaces/ZitStatus` | 订阅 | — | `hw_manager` | ⏸HIL | `workspace_auv/src/uv_hm/uv_hm/hw_manager.py:create_subscription` | |
| `/zit6/state/status` | `zit6_interfaces/ZitStatus` | 订阅 | — | `task_runner` | 两者 | `workspace_auv/src/uv_task/uv_task/task_runner.py:create_subscription` | |
| `/zit6/state/zithbt` | `std_msgs/UInt32` | 发布 | `sim_bridge` | — | 两者 | `workspace_sim/src/uv_sim/uv_sim/sim_bridge.py:create_publisher` | 心跳序号 |
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
| `/zit6/get_params` | `zit6_interfaces/srv/GetParams` | 服务 | `sim_bridge` | — | SIL | `workspace_sim/src/uv_sim/uv_sim/sim_bridge.py:create_service` | 返回全部 PID 通道 |
| `/zit6/update_params` | `zit6_interfaces/srv/UpdateParams` | 服务 | `sim_bridge` | — | SIL | `workspace_sim/src/uv_sim/uv_sim/sim_bridge.py:create_service` | **只改运行时，不写回 `sim_pid.yaml`** |
| `/nav/navigate_to` | `uv_msgs/srv/RunTask` | 服务 | `navigator` | — | 两者 | `workspace_auv/src/uv_nav/uv_nav/navigator.py:create_service` | 请求执行导航任务 |
| `/task/run` | `uv_msgs/srv/RunTask` | 服务 | `task_runner` | — | 两者 | `workspace_auv/src/uv_task/uv_task/task_runner.py:create_service` | 启动任务/任务列表 |
| `/task/stop` | `std_srvs/srv/Trigger` | 服务 | `task_runner` | — | 两者 | `workspace_auv/src/uv_task/uv_task/task_runner.py:create_service` | 中断当前任务 |
| `/task/exec` | `uv_msgs/srv/ExecTask` | 服务 | `task_runner` | — | 两者 | `workspace_auv/src/uv_task/uv_task/task_runner.py:create_service` | 单条任务执行 |

> 服务/动作的**请求-响应字段语义**属于 `service_action_contract.md`（T3b），本表只固定名称与类型。

## 7.5 栈外消费者：可视化 GUI（`visualization/auv_visualizer.py`）

> 该 GUI **不是** colcon 包、没有 entry point，用 `python3 visualization/auv_visualizer.py` 直接跑；
> 登记在此是因为它订阅/发布的话题必须与契约一致。

| 话题 | 消息类型 | 方向 | 发布者 | 订阅者 | 模式 | 证据 | 说明 |
|---|---|---|---|---|---|---|---|
| `/basic_motion/pose_info` | `uv_msgs/PoseInfo` | 订阅 | — | GUI | 两者 | `visualization/auv_visualizer.py:create_subscription` | 2D 地图位姿 |
| `/perception/objects` | `uv_msgs/ObjectPositionArray` | 订阅 | — | GUI | 两者 | `visualization/auv_visualizer.py:create_subscription` | 物体与射线 |
| `/task/status` | `uv_msgs/TaskStatus` | 订阅 | — | GUI | 两者 | `visualization/auv_visualizer.py:create_subscription` | 任务进度 |
| `/perception/annotated/front_left` | `sensor_msgs/Image` | 订阅 | **无（缺口）** | GUI | 两者 | `visualization/auv_visualizer.py:create_subscription` | 4 通道；**当前无发布者**，见 §9 |
| `/perception/annotated/front_right` | `sensor_msgs/Image` | 订阅 | **无（缺口）** | GUI | 两者 | `visualization/auv_visualizer.py:create_subscription` | 同上 |
| `/perception/annotated/down_left` | `sensor_msgs/Image` | 订阅 | **无（缺口）** | GUI | 两者 | `visualization/auv_visualizer.py:create_subscription` | 同上 |
| `/perception/annotated/down_right` | `sensor_msgs/Image` | 订阅 | **无（缺口）** | GUI | 两者 | `visualization/auv_visualizer.py:create_subscription` | 同上 |
| `/zit6/cmd/setpoint` | `zit6_interfaces/ZitSetpoint` | 发布 | GUI | — | 两者 | `visualization/auv_visualizer.py:create_publisher` | 遥控面板（与 `basic_motion` 争用同一话题） |

## 8. HIL 专属（⏸ 暂缓，当前阶段不做验收）

| 话题 | 消息类型 | 方向 | 发布者 | 订阅者 | 模式 | 证据 | 说明 |
|---|---|---|---|---|---|---|---|
| `/zit6/sim/nav` | `std_msgs/Float32MultiArray` | 发布 | `sim_bridge` | — | ⏸HIL | `workspace_sim/src/uv_sim/uv_sim/sim_bridge.py:create_publisher` | 仿真位姿 → MCU（`vel` 为**机体系**速度） |

> micro-ROS 串口链路的上下行全貌（含 MCU 端订阅清单）见 `docs/hil_architecture.md`；
> 该文档在 SIL 阶段仅作参考，**不参与当前验收**。

## 9. 缺口登记（已知且尚未修复，勿当成不存在）

| 缺口 | 现状 | 归属 |
|---|---|---|
| `/perception/annotated/{front,down}_{left,right}` | **有订阅者（GUI `visualization/auv_visualizer.py`）但全局无发布者** | B5，待定 |
| `/auv/state`（`uv_msgs/AuvState`） | **无发布者、无订阅者**（navigator 2026-09-13 已迁到 `PoseInfo`）；消息只剩定义 | 清理待定 |
| `/auv/dvl_altitude` | 有发布者、**无订阅者** | 待定（T6 可能需要） |
| `/auv/thrusters_state` | 有发布者、**无订阅者**（可用于混控校验，目前没人用） | 待定 |
| `/tf`、`/tf_static` | 话题存在但**无任何消息**：场景未声明 `<ros_base_link_transform publish="true"/>` | B18，已文档化 |
| `/perception/line/{front,down}_left` | 有发布者、有订阅者（`line_follower.py`，仅 `follow_line` 任务运行期间建立）；`front_right`/`down_right` 两路**无订阅者** | 待定 |
