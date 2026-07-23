# WUURC 2026 AUV 赛道任务说明

本文档对应 `uv_task/task_runner.py` 中的 WUURC 通用任务逻辑。JSON 只安排
已有的移动任务（例如 `wtravelxy`、`setz`、`setrz`），只有视觉对正、上浮、
ArUco、动态 sector 选择和设备命令使用专用任务点。

## 当前任务执行情况

当前默认任务列表位于：

```text
/home/laurie/AUV2026/YouLong_AUV_Control_System/workspace_auv/src/uv_task/config/tasks.json
```

文件当前顺序为：

1. `start` 和 `wait`：初始化 odom 并等待传感器位姿稳定。
2. `sync_ins_pose`：将任务层命令位姿同步为 `/basic_motion/pose_info` 的 INS
   位姿，防止巡线或碰撞后的历史命令积分漂移。
3. `wtravelxy`：直接移动至 basket、获取框、ArUco 观测位置、动态目标 sector
   和 start。
4. `align_downward`：完成 arrow、目标 sector 和 start 的下视对正。
5. `surface_attempt`：完成获取框及终点的 0.5 m 上浮尝试。
6. `recognize_aruco` 和 `select_aruco_sector`：识别 4x4_1000 ID 并生成动态
   `wtravelxy(target=selected_sector)` 目标。当前 Stonefish 场景加载
   `aruco_4x4_id4.png`，因此 JSON 明确设置了仿真兜底 `fallback_id: 4`，ID4
   对应绿色 sector；真实比赛配置应删除该字段。
7. `take_water_sample`、`show_selected_sector_led`、`drop_beacon`：发布采样、
   指示灯和投放设备命令。

因此，当前代码已经实现的完整流程是：

| 阶段 | 状态 | 实现位置 |
|---|---|---|
| 管道巡检、识别 triangle/square、basket 结束 | 已有实现，未重写 | `uv_task/line_follower.py` |
| basket、获取框、ArUco、sector、start 位移 | 已实现 | JSON `wtravelxy` |
| 环境采样 | 已实现话题下发 | `/task/device_command` |
| 获取框 arrow 下视对正 | 已实现 | `task_runner.py::_align_downward` |
| 获取框上浮 0.5 m、失败回退 | 已实现 | `task_runner.py::_try_surface_and_restore` |
| ArUco 4x4_1000 ID 识别 | 已实现 | `uv_task/aruco_decoder.py` + 50 帧投票 |
| ID 到 yellow/green/red sector 映射 | 已实现 | `task_runner.py::_sector_for_aruco` |
| sector 下视对正和 beacon 投放命令 | 已实现 | `align_downward`、`drop_beacon` |
| 返回 start、start 标志下视对正、上浮尝试 | 已实现 | JSON `wtravelxy`、`align_downward` |

任务层不使用 `position.py` 的世界定位结果，也不依赖占位的
`uv_hm/hw_manager.py`。长距离移动全部通过已有的 `wtravelxy` 封装，视觉
微调通过 FRD 机体系 `BMOVE xy` 完成。

## 坐标系和场景坐标

控制栈使用 NED 约定：X 为北，Y 为东，Z 向下为正，yaw 顺时针为正，单位为
米和度。`START` 之后，BasicMotion 的 odom 原点是 AUV 当时的位置，odom 的
0 度是 AUV 当时的朝向。当前 JSON 使用了已经在仿真中实际观测到的世界坐标；
实际比赛前应根据 START 后 `/basic_motion/pose_info` 的 odom 坐标重新测量并
调整 JSON 中的 `wtravelxy` 目标。

默认场景 `workspace_sim/src/stonefish_ros2/Data/wuurc_murc_2026_auv.scn`
中的起始位姿是 `(10.50, -2.80, yaw=90°)`，任务代码的默认点如下：

| 对象 | 当前 JSON 世界 `(x,y)` |
|---|---:|
| start / start 标志 | `(10.50,-2.80)` |
| basket 框中心 | `(5.88,2.80)` |
| 获取框和 arrow | `(3.50,2.80)` |
| ArUco 观测位置 | `(0.85,3.15)` |
| yellow sector | `(1.75,-2.80)` |
| green sector | `(3.50,-2.80)` |
| red sector | `(5.25,-2.80)` |

固定场景点直接在 `config/tasks.json` 中调整；动态 sector 的坐标只保留在
`_sector_for_aruco`，因为它依赖识别出的 ArUco ID。

`recognize_aruco`、`select_aruco_sector`、动态 sector 移动、sector 对正和投放
任务均设置了 `stop_on_failure`。任何一个关键阶段失败时任务列表会暂停在当前
位置，不会误执行后面的“返回 start”移动；可检查相机、话题和位姿后从对应任务
重新执行。

### 巡线交接位姿保护

巡线期间 `LineFollower` 会根据每个 BMOVE 成功结果积分一个命令位姿，但控制器
在碰撞、墙面约束或动作容差内提前返回时，命令位姿可能比真实机器人位置大很多。
本次实际日志中，真实位姿约为 `(6.53,2.63)`，历史命令位姿却是
`(29.60,-1.94)`，导致 basket 目标被误算为 `(-24.00,6.56)`，机器人向池外
移动并卡墙。

现在每一个关键场景移动前都会由 JSON 的 `sync_ins_pose` 读取 `/basic_motion/pose_info` 的
`robot_x/y/z/yaw`，用最新 INS 位姿重置 task runner 的命令跟踪，再计算
`wtravelxy` 增量。若位姿话题缺失或超过 `wuurc_pose_max_age_s`（默认 1 秒），
该场景移动会拒绝发送，避免再次产生大幅错误位移。

## WUURC 任务流程

### 1. Basket 和环境采样

先执行 `sync_ins_pose`，再使用 `wtravelxy` 回到 basket 中心，消除巡线结束时的
视觉定位误差。随后 `take_water_sample` 发布
一个 `uv_msgs/msg/DeviceCommand`：

```text
话题：/task/device_command
类型：uv_msgs/msg/DeviceCommand
device_type=DEVICE_ARM(1), command=1, value=1
```

下位机负责真正的采样机构动作，任务层不调用 placeholder hardware manager，
也不等待采样器的内部反馈。

### 2. 获取任务和上浮

机器人先到获取框中心，再使用 `down_left` 相机的 YOLO 检测结果寻找类别 3
（`arrow`）。检测框中心与图像中心的误差被转换为 FRD 机体系的微小 `BMOVE`：

- 图像 X 误差修正 body-Y（右方向）；
- 图像 Y 误差修正 body-X（前方向）；
- 每次修正受 `wuurc_align_max_step_m` 限制；
- 置信度、像素容差、比例和方向符号均可通过 ROS 参数调整。

对正完成后尝试沿 NED Z 轴上浮 0.5 m。若动作被控制器拒绝，则重新发送原始
深度并继续后续流程；若动作成功，停留一小段时间后重新下潜到原始深度。

### 3. ArUco 识别和 sector 投放

机器人移动到 ArUco 墙牌的测量观测位置，设置观测深度和朝向，随后从
`/auv/front_cam/stitched` 的左右完整半幅图像使用 `aruco/build/detect_aruco.py`
中已经验证的 OpenCV 逻辑解码。识别不使用 YOLO 框，也不使用
`position.py`。任务等待 50 个不同图像帧，每帧检测到的合法 ID 计数一次，最终
选择出现次数最多的 ID；平票时选择较小的 ID。只接受 ID 1 到 6：

| ArUco ID | 目标 sector | YOLO 类别 | LED 命令 |
|---|---|---:|---:|
| 1、2 | yellow | 0 | 1 |
| 3、4 | green | 2 | 2 |
| 5、6 | red | 1 | 3 |

识别成功后先发布对应 LED 命令 3 秒，再回到工作深度并到达对应 sector。使用
同一个 `down_left` 检测通道对正 sector 框，最后只发布一个 beacon 投放命令：

```text
device_type=DEVICE_SERVO(2), command=2, value=ArUco_ID
```

如果 50 帧中没有获得合法 ArUco ID，`recognize_aruco` 会失败并因
`stop_on_failure` 暂停任务，不会盲选 sector。当前 Stonefish 配置中的
`fallback_id: 4` 仅用于已知 ID4 的仿真场景；真实比赛应删除该参数。

### 4. 返回起点

投放后回到 odom `(0,0)`，使用类别 4（`start`）进行下视对正，最后尝试上浮
0.5 m。当前仿真模型如果拒绝完全上浮，代码会保留已对正的深度位置，并在日志
中明确记录失败原因。

## 视觉模型

`vision.py` 默认从以下位置加载模型：

```text
workspace_auv/src/uv_perception/resource/box_best.pt
workspace_auv/src/uv_perception/resource/seg_best.pt
```

其中 box 模型类别为：`yellow_sector=0`、`red_sector=1`、`green_sector=2`、
`arrow=3`、`start=4`、`triangle=5`、`square=6`、`basket=7`、
`aruco_tag=8`；seg 模型类别 0 为 pipe。box 模型只负责确认 ArUco 板存在，
具体 ID 由任务层的 OpenCV ArUco 解码完成，因为 `DetectionArray` 不包含 ID
字段。

## 可调 ROS 参数

常用参数如下，全部声明在 `task_runner`：

| 参数 | 默认值 | 用途 |
|---|---:|---|
| `wuurc_surface_delta_z` | `0.50` | 上浮距离 |
| `wuurc_align_pixel_tolerance` | `35` px | 视觉对正容差 |
| `wuurc_align_m_per_pixel_x/y` | `0.00055` | 像素到米的比例 |
| `wuurc_align_body_x_sign` | `-1` | 图像 Y 到 body-X 的方向 |
| `wuurc_align_body_y_sign` | `-1` | 图像 X 到 body-Y 的方向 |
| `wuurc_align_timeout_s` | `25` s | 单次视觉对正超时 |
| `wuurc_aruco_timeout_s` | `12` s | ArUco 50 帧采样最长等待时间 |
| `wuurc_aruco_sample_count` | `50` | ArUco 投票帧数 |
| `wuurc_led_duration_s` | `3` s | LED 指示时间 |

第一次下水调试时，应先固定机器人，观察一帧检测框，确认图像向右/向下的误差
是否会产生正确的 FRD 修正，再调整两个 sign 参数和比例参数。

## Humble 环境运行步骤

以下命令假设仓库路径为 `/home/laurie/AUV2026/YouLong_AUV_Control_System`，
并且当前系统安装的是 ROS 2 Humble。

### 1. 构建控制侧

```bash
cd /home/laurie/AUV2026/YouLong_AUV_Control_System/workspace_auv
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select uv_msgs uv_control uv_perception uv_task uv_bringup
source install/setup.bash
```

如果系统没有预装依赖，需要先用 Humble 的 rosdep 安装依赖；模型文件不需要
额外下载，确认两个 `.pt` 文件存在即可。

### 2. 构建仿真侧

```bash
cd /home/laurie/AUV2026/YouLong_AUV_Control_System/workspace_sim
source /opt/ros/humble/setup.bash
source ../workspace_auv/install/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 3. 先运行感知和控制、暂不自动执行任务

建议第一次先关闭 task runner，检查仿真图像和检测话题：

```bash
ros2 launch uv_bringup sim_bringup.py \
  scenario_desc:=wuurc_murc_2026_auv.scn \
  enable_ai:=true enable_task:=false enable_nav:=false \
  publish_annotated:=true
```

另开终端重新 source 两个工作区，然后检查：

```bash
ros2 topic list | grep -E 'perception|basic_motion|auv/front_cam'
ros2 topic echo /perception/detection/down_left
ros2 topic echo /perception/detection/front_left
```

应能看到 `down_left`、`front_left` 的 `DetectionArray`。如果没有检测结果，
先检查 `vision` 启动日志是否显示 box 和 segment 模型加载成功。

### 4. 运行完整自动任务

停止上一步 launch 后，执行：

```bash
ros2 launch uv_bringup sim_bringup.py \
  scenario_desc:=wuurc_murc_2026_auv.scn \
  enable_ai:=true enable_task:=true enable_nav:=false \
  debug_mode:=false
```

`enable_task:=true` 会让 `task_runner` 自动加载并依次执行上面所述的
`workspace_auv/src/uv_task/config/tasks.json`。观察任务状态和设备命令：

```bash
ros2 topic echo /task/status
ros2 topic echo /task/device_command
```

### 5. 单步调试

如果不想自动执行任务，可启动：

```bash
ros2 launch uv_bringup sim_bringup.py \
  scenario_desc:=wuurc_murc_2026_auv.scn \
  enable_ai:=true enable_task:=true debug_mode:=true
```

此模式不会自动加载 `tasks.json`。调用单任务服务前必须已经执行过 START，
否则 BasicMotion 会拒绝所有运动目标：

```bash
ros2 service call /task/exec uv_msgs/srv/ExecTask \
  "{task_name: start, params_json: '{}', timeout: 20.0}"
ros2 service call /task/exec uv_msgs/srv/ExecTask \
  "{task_name: sync_ins_pose, params_json: '{}', timeout: 10.0}"
ros2 service call /task/exec uv_msgs/srv/ExecTask \
  "{task_name: wtravelxy, params_json: '{\"x\": 5.88, \"y\": 2.80}', timeout: 120.0}"
ros2 service call /task/exec uv_msgs/srv/ExecTask \
  "{task_name: align_downward, params_json: '{\"class_id\": 3, \"label\": \"arrow\"}', timeout: 60.0}"
ros2 service call /task/exec uv_msgs/srv/ExecTask \
  "{task_name: recognize_aruco, params_json: '{}', timeout: 60.0}"
ros2 service call /task/exec uv_msgs/srv/ExecTask \
  "{task_name: select_aruco_sector, params_json: '{}', timeout: 10.0}"
```

停止任务：

```bash
ros2 service call /task/stop std_srvs/srv/Trigger
```

### 6. Humble 下的构建和测试检查

```bash
cd /home/laurie/AUV2026/YouLong_AUV_Control_System/workspace_auv
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select uv_perception uv_task --symlink-install
colcon test --packages-select uv_task
colcon test-result --verbose
```

当前仓库验证结果为：`uv_perception` 和 `uv_task` 构建成功，任务包的 10 个
ArUco 映射及动态目标测试全部通过。完整 Stonefish 仿真还依赖本机的图形环境、
Stonefish 原生库和 DDS 网络权限；如果启动窗口或 DDS 失败，应先单独检查
`stonefish_ros2` 和 `uv_sim`，不应把它误判为任务代码错误。

## 可视化器调试

启动仿真后另开终端，按相同顺序 source 环境后运行：

```bash
cd /home/laurie/AUV2026/YouLong_AUV_Control_System
source /opt/ros/humble/setup.bash
source workspace_auv/install/setup.bash
source workspace_sim/install/setup.bash
python3 visualization/auv_visualizer.py
```

可视化器优先使用 `/basic_motion/pose_info` 显示 odom 位姿；若该话题暂时没有
数据，则自动回退到 `/auv/state`。图像优先显示 YOLO 标注图；即使未设置
`publish_annotated:=true`，也会从 `/auv/front_cam/stitched` 和
`/auv/down_cam/stitched` 自动分离左右图像显示。

若界面仍没有数据，请在同一终端执行：

```bash
ros2 topic hz /basic_motion/pose_info
ros2 topic hz /auv/state
ros2 topic hz /auv/front_cam/stitched
ros2 topic hz /auv/down_cam/stitched
```

四个话题均为当前可视化器的有效输入。若 `ros2 topic hz` 没有输出，先确认
仿真和 visualizer 使用相同的 `ROS_DOMAIN_ID`，并且两个工作区都已 source。

source /opt/ros/humble/setup.bash
source /home/laurie/AUV2026/YouLong_AUV_Control_System/workspace_sim/install/setup.bash
source /home/laurie/AUV2026/YouLong_AUV_Control_System/workspace_auv/install/setup.bash
舍弃使用yolo框的逻辑，直接使用cv在整个图像范围内识别aruco tag

ros2 launch uv_bringup sim_bringup.py   scenario_desc:=wuurc_murc_2026_auv.scn   enable_ai:=true   enable_nav:=false     task_auto_start:=true   task_file:=/home/laurie/AUV2026/YouLong_AUV_Control_System/workspace_auv/src/uv_task/config/tasks.json   model_path:=/home/laurie/AUV2026/YouLong_AUV_Control_System/workspace_auv/src/uv_perception/resource/box_best.pt   segment_model_path:=/home/laurie/AUV2026/YouLong_AUV_Control_System/workspace_auv/src/uv_perception/resource/seg_best.pt quite:=true
