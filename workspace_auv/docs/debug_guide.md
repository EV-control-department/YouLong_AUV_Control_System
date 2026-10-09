# 调试指南

## 三项大任务的状态灯

`task_runner` 对建图（含遍历）、抓海参、转盘统一发布
`/zit6/cmd/light`，消息类型 `std_msgs/msg/UInt8`：

| 数值 | 颜色 | 状态 |
| --- | --- | --- |
| 3 | 黄 | 当前大任务执行中，包括定位、识别、移动和接触动作 |
| 2 | 绿 | 当前大任务返回成功 |
| 1 | 红 | 返回失败、主动停止或初始化/执行/清理异常 |
| 0 | 灭 | 既有关闭灯光接口，不用于大任务正常结束 |

结束颜色保持到下一任务或其他灯光命令覆盖。绿灯只表示软件流程成功；
建图降级默认结果、抓取效果、转盘实际转角仍需结合报告或录像验证。
请关闭rqt/GUI中并行的灯光发布，避免覆盖状态。该指示不会改变心跳、
解锁条件或运动安全判定。复制代码后构建uv_task并重启task_runner生效。

## START 与心跳统一管理（真机新版）

`basic_motion` 是 `/zit6/cmd/agxhbt` 的唯一心跳发送者；`uv_hm` 默认只监控
（`enable_heartbeat:=false`）。请停止旧版 uv_hm、rqt、GUI、CLI 心跳发布器，
构建后重新启动节点，旧进程不会自动加载新代码。

START 不再按心跳发布器数量拒绝初始化：rqt 创建了发布器但未发送消息时，
不会阻断 START。仍然检查 MCU 是否实际处于上锁状态；如果其他节点持续发送
解锁心跳使 MCU 无法上锁，START 会等待超时失败，不绕过固件限制。

BasicMotion 中文诊断日志：启动时列出配置和接口；运行中每3秒汇报 status/odom
接收年龄、导航有效性、原点代数、心跳启用状态和累计发送包数。
START 的 `[1/6]` 到 `[6/6]` 分别表示等待上锁、查找服务、发送请求、
确认新代odom、初始化上位机原点并发布位姿、启用心跳并返回成功。
等待服务/odom/移动解锁时每秒打印实际条件。没有START日志表示请求尚未到达执行回调；
odom累计为0需检查MCU发布和DDS通信；心跳未启用时看START最后停在哪一步。
心跳累计增加只证明本地publish成功，不证明MCU收到或已解锁。

START 顺序：暂停自身心跳 → 等待 MCU 上锁（最多3秒）→ 调用 `setorigin`
→ 确认有效且代数一致的新 odom → 初始化上位机原点并发布位姿 → 开始15Hz心跳。
之后运动会等待 MCU 确认解锁；START 失败时不启用心跳，也不执行后续抓取。
因此无需先手动解锁再 START；这正是旧流程被 `requires disarmed state` 拒绝的原因。
再次 START 也会先暂停心跳、等待上锁，不绕过固件保护。节点退出后心跳停止，
由 MCU 的心跳超时保护上锁；务必保证没有第二个心跳源。

```bash
cd /home/nvidia/YouLong_AUV_Control_System/workspace_auv
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --packages-select uv_control uv_hm uv_task
source install/setup.bash
# 独立终端，均加载上述环境；agent、视觉节点仍按各任务要求启动
ros2 run uv_control basic_motion
ros2 run uv_hm hw_manager --ros-args -p enable_heartbeat:=false
# 确认现场路径、抓取深度和下压参数后，在另一终端启动
ros2 run uv_task task_runner --ros-args \
  -p mission_file:="$PWD/src/uv_task/config/missions/grab_sea_cucumber.yaml"
```

海参下压参数位于 `src/uv_task/config/tasks/grab_sea_cucumber.yaml`。
下压采用现场配置的速度/时长连续发布body-z速度，不拆分位置小步，也不恢复
已删除的旧速度/行程限幅；只拒绝非有限值和非正速度/时长。兼容字段
`max_press_distance_m` 不再生效，实际行程和触底需现场验证，程序没有接触传感器。
上浮已改为直接通过 `/zit6/cmd/setpoint` 发布机体负z速度，不再使用BMOVE。
`ascent.speed_mps: 0.01` 表示1cm/s上限，近目标减速；发布周期0.05s。
停止、异常、到位或超时均发零速度，不主动发向下位置纠偏。回位预算默认300秒，
仍受整项任务600秒预算约束，长距离慢上浮需据实增加预算。

这只绕过位置环，MCU速度环仍可制动、反向输出；零速度不等于零推力。
后续运输也会切回位置控制。若要求绝不产生向下垂推，必须在固件速度/推力控制层
实现方向限制或专用抓取模式，不能靠此任务保证。

舵机任务配置已采用 `gripper.pickup_angle_deg: 0.0` 和
`gripper.release_angle_deg: 90.0`。按最新真机rqt实测，ZitServo.angle单位为度，
发布值是0和90。任务内部set_servo接口仍接受弧度以兼容旧任务，出口统一转换为度。
本地旧固件副本的rad注释不适用于当前真机；未修改或烧录该固件副本。

海参下视视觉伺服的物理安装方向：任务配置 `down_camera_mount_yaw_deg: 180.0`
表示下视双目整体绕机体yaw旋转180°。camera的 `down_rotate_180` 是显示/推理
方向处理，发布检测掩膜时已还原到原始标定像素；因此任务还需补偿物理安装角，
不是重复旋转图像。此补偿反转水平body x/y修正，再按艇体yaw转为odom增量；
夹爪偏置、升沉和建图外参不改。恢复原安装时任务参数填0。

海参任务视觉降级：水平对准超时、抓前无可信计数或暂时为0，均等待1秒后重新
扫描，不再单次失败就退出。连续视觉失败使用 `max_failed_attempts` 重试预算，
仍受总超时约束。计数超时但已有至少2个新鲜帧时使用保守计数；旧帧和单帧不兜底。
已完成抓取并回位但抓后计数未知时，先运往收集区释放可能携带的海参，再返回复查；
不增加 `confirmed_removed`，不能把未知投放算作完成。位姿丢失、运动失败、
上浮超调及下压配置限制仍会停止，不继续盲目运动。

本节替代下文旧版“uv_hm持续解锁心跳”的启动方式；手动心跳仅用于独立维护，
不可与 BasicMotion 同时启用。

## 真机完整复制后的建图部署检查（Foxy / Jetson）

唯一目标目录为 `/home/nvidia/YouLong_AUV_Control_System/workspace_auv`，
不是 `/home/nvidia/dev/YouLong_AUV_Control_System/workspace_auv`。
复制源码、模型和标定配置，不复制本机的 `build/`、`install/`、`log/`、
`.venv/` 或 `__pycache__/`；不同架构及 ROS 发行版的产物不能混用。
还需保留仓库根目录的 `scripts/convert_yolo_inference.py`。
先停止旧工作区的 camera、task_runner、basic_motion 等节点，避免两版节点共存。
确认 `.bashrc` 没有自动 source `~/dev/.../install/setup.bash`。

### 重新生成兼容权重和构建

`last.pt` 是自己训练的可信 YOLO11n-seg 权重；不要对不可信模型执行转换，
因为 PyTorch checkpoint 使用 pickle。转换只移除训练对象，不替换网络层。
已有 `last_inference.pt` 时跳过转换；脚本拒绝覆盖任何已有输出。

```bash
cd /home/nvidia/YouLong_AUV_Control_System
python3 scripts/convert_yolo_inference.py \
  workspace_auv/src/uv_camera/resource/last.pt --device cuda:0

cd /home/nvidia/YouLong_AUV_Control_System/workspace_auv
source /opt/ros/foxy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 pkg prefix uv_task
ros2 pkg prefix uv_camera
ros2 pkg prefix uv_msgs
python3 -c 'from uv_msgs.msg import MappingObservationArray, PoseInfo; from uv_msgs.action import BasicMotion; import uv_task.task_runner, uv_camera.composed; print("入口及消息导入通过")'
```

三个 prefix 都应落在本目标工作区 `install/` 中。
若已有从另一台机器复制来的构建产物，先把本工作区的 `build`、`install`、
`log` 移至源码之外的备份目录，再构建；不要带旧缓存继续增量构建。
如果出现 future timestamp / Clock skew，先核对两台机器的系统时间，
时间同步后再执行干净构建，否则配置和消息生成可能仍被跳过。

### 各终端使用相同环境

每个新终端先执行以下命令。`ROS_DOMAIN_ID` 需与 MCU agent、上位机一致；
示例使用 0，如现有系统不是 0，所有终端统一改为实际值。
RMW 实现也要保持现有部署一致，不在调试过程中混用。

```bash
cd /home/nvidia/YouLong_AUV_Control_System/workspace_auv
source /opt/ros/foxy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
export UV_MODEL_MAPPING_FILE="$PWD/src/uv_camera/weights/real_last.yaml"
```

终端一：先按原有方式启动真机 MCU 通信/agent，确认 `/zit6/state/pos`、
`/zit6/state/status` 持续更新，再启动运动控制服务。此命令只启动节点，
不要在路径与深度未确认时发送运动目标。

```bash
ros2 run uv_control basic_motion --ros-args -p basic_motion_action:=/basic_motion
```

终端二：开启视觉和推流。双目拼接均为 `1280x480`（每目 `640x480`）。
采集尺寸不满足时本地新版会拒绝使用错误内参，不再默默继续定位。

```bash
ros2 run uv_camera uv_camera --ros-args \
  --params-file "$PWD/src/uv_camera/config/profiles/real_default.yaml" \
  -p enable_ai:=true -p enable_mapping_vision:=true \
  -p enable_turntable_vision:=false \
  -p enable_front_camera:=true -p enable_down_camera:=true \
  -p front_cam_path:=/dev/video0 -p down_cam_path:=/dev/video2 \
  -p model_path:="$PWD/src/uv_camera/resource/last_inference.pt" \
  -p device:=cuda:0 -p confidence:=0.35 -p inference_fps:=10.0
```

终端三：先检查，不急于运行任务：

```bash
ros2 action info /basic_motion
ros2 topic echo /basic_motion/pose_info
ros2 topic info /perception/mapping/observations
ros2 topic echo /perception/mapping/observations
ros2 topic info /zit6/cmd/servo --verbose
```

应有一个动作服务端，位姿持续更新，camera 发布建图观测；无可见目标时
观测可能为空，需同时核对 camera 加载成功日志和帧处理日志。
完成 `config/tasks/mapping_grid.json` 中真实九宫格坐标、Tag 坐标、池底深度、
巡航深度和相机外参复核，确认运动区域安全后才运行：

```bash
ros2 run uv_task task_runner --ros-args \
  -p target_id:=mapping_grid -p basic_motion_action:=/basic_motion \
  -p mission_file:="$PWD/src/uv_task/config/missions/mapping_grid.json"
```

该命令会自动 START 然后执行含 AprilTag 的建图链，不能仅用于导入检查。
`target_id` 只是目标元数据，不负责选择任务链，任务链由 `mission_file` 指定。
默认动作名改为绝对路径 `/basic_motion`，不随 `/auv` 节点命名空间改变；
如需旧 `/auv/basic_motion`，在控制与任务两端同时设置该参数。

### 本轮日志的解释

* `/auv/perception/model_classes` / `ModelClassMapping` / `ObjectTrackArray`
  属于现场另一版管线。本工作区使用 `uv_camera/weights/real_last.yaml`
  和 `MappingObservationArray`，不依赖上述接口。再次看到对应 FATAL，
  应检查加载路径、旧进程和 overlay，而不是凭空补充另一版消息。
* `uv_msgs/action/BasicMotion_FeedbackMessage` 是 action 内部反馈类型。
  日志显示 rqt 的 msg 类型解析器把它当作普通 msg 解析而失败；不能据此
  判断动作服务不存在。用 `ros2 action info /basic_motion` 检查服务，
  用 `/basic_motion/pose_info` 和 `/task/status` 观察业务状态。
* `/zit6/cmd/servo` 应唯一为 `std_msgs/msg/Float32`，单位 rad，与本地 MCU
  订阅一致。若同名有多个类型，查看 `--verbose` 给出的节点并停止或重映射
  异类型的旧节点；不要随意把本版消息类型改成另一种。
* 当前本地 task_runner 没有日志中那条 model_classes 等待逻辑，也没有
  TF listener 后台线程。旧节点退出后的 TF 析构异常不能靠修改本版
  去隐藏；先完成整套源码和干净产物的统一，再定位仍能复现的问题。

## 构建

### 构建单个包（快速迭代）

```bash
# AUV 工作空间
cd workspace_auv
colcon build --packages-select uv_control uv_task  --symlink-install
source install/setup.bash

# 仿真工作空间（必须先 source auv_ws）
cd workspace_sim
colcon build --packages-select uv_sim  --symlink-install
source install/setup.bash
```

`--symlink-install` 让 Python 节点的改立即生效，无需重新构建。

### 完整构建

```bash
cd workspace_auv && colcon build && source install/setup.bash
cd workspace_sim && colcon build && source install/setup.bash
```

## ros2 CLI 调试

### 打开 ZIT6 GUI 上位机

在图形桌面终端中从仓库根目录启动。GUI 采用包内相对导入，因此要用 Python 模块方式运行，不能直接执行 `gui.py` 文件：

```bash
code
```

如果需要在仿真中查看实时数据，先在另一个终端启动仿真/相关 ROS 节点，并确保 GUI 与节点使用相同的 ROS_DOMAIN_ID 和 RMW 配置。GUI 需要可用的桌面显示环境；若提示缺少 PyQt5/PySide6 或 ROS Python 模块，应在当前 ROS 环境中安装/配置对应依赖后再启动。

### 检查节点是否运行

```bash
ros2 node list
ros2 node info /vision
ros2 node info /position
ros2 node info /basic_motion
ros2 node info /task_runner
ros2 node info /sim_bridge
```

### 检查话题

```bash
# 列出所有活跃话题
ros2 topic list

# 查看话题信息（类型、发布者、订阅者）
ros2 topic info /topic_name

# 实时查看话题内容
ros2 topic echo /basic_motion/pose_info
ros2 topic echo /zit6/cmd/setpoint
ros2 topic echo /zit6/state/status
ros2 topic echo /task/status
ros2 topic echo /auv/odometry

# 感知系统
ros2 topic echo /perception/objects
ros2 topic echo /perception/detection/front_left
ros2 topic echo /perception/detection/front_right
ros2 topic echo /perception/detection/down_left
ros2 topic echo /perception/detection/down_right

# 查看话题频率
ros2 topic hz /basic_motion/pose_info
ros2 topic hz /perception/objects
ros2 topic hz /zit6/cmd/setpoint
ros2 topic hz /auv/odometry
```

### 检查服务

```bash
# 列出所有服务
ros2 service list

# 调用 task_runner 服务
ros2 service call /task/stop std_srvs/srv/Trigger
```

### 发送 BasicMotion Action 测试

```bash
# 检查 action server
ros2 action list
ros2 action info /basic_motion

# 发送 START goal（初始化 odom 原点）
ros2 action send_goal /basic_motion uv_msgs/action/BasicMotion \
  "{cmd_type: 6, axes: '', target: [0, 0, 0, 0], timeout: 5.0}"

# SET — 下潜到 z=3.0
ros2 action send_goal /basic_motion uv_msgs/action/BasicMotion \
  "{cmd_type: 3, axes: '', target: [0, 0, 3.0, 0], timeout: 30.0}"

# SET — 转到 yaw=50°
ros2 action send_goal /basic_motion uv_msgs/action/BasicMotion \
  "{cmd_type: 3, axes: '', target: [0, 0, 0, 50.0], timeout: 30.0}"

# BMOVE — 向前 2m
ros2 action send_goal /basic_motion uv_msgs/action/BasicMotion \
  "{cmd_type: 2, axes: '', target: [2.0, 0, 0, 0], timeout: 60.0}"

# SET — 回到原点
ros2 action send_goal /basic_motion uv_msgs/action/BasicMotion \
  "{cmd_type: 3, axes: '', target: [0, 0, 0, 0], timeout: 60.0}"

# WTRAVEL — 向世界系 (3,4) 方向直线前进（先转向，再直走）
ros2 action send_goal /basic_motion uv_msgs/action/BasicMotion \
  "{cmd_type: 4, axes: '', target: [3.0, 4.0, 0.0, 0.0], timeout: 60.0}"

# BTRAVEL — 沿当前机头方向直线前进 3m（body→world 后再转向+直走）
ros2 action send_goal /basic_motion uv_msgs/action/BasicMotion \
  "{cmd_type: 5, axes: '', target: [3.0, 0.0, 0.0, 0.0], timeout: 60.0}"
```

给 action 发送 goal 时加上 `--feedback` 可以看实时反馈：

```bash
ros2 action send_goal --feedback /basic_motion uv_msgs/action/BasicMotion \
  "{cmd_type: 2, axes: '', target: [2.0, 0, 0, 0], timeout: 60.0}"
```

### 检查参数

```bash
# 列出节点参数
ros2 param list /sim_bridge

# 获取参数值
ros2 param get /sim_bridge hil_mode
```

### 日志

所有节点的 `output='screen'`，运行时直接看终端输出。也可以单独看某个节点的日志：

```bash
# 设置日志级别（DEBUG 最详细）
ros2 run uv_control basic_motion --ros-args --log-level debug
```

## 仿真调试

### 完整执行 RoboCup 任务

下面的命令会重新构建两个工作空间，并启动当前 `robocup_enbody` 分支的具身智能建图任务链。仿真起点固定为 `(0, 0, 0.10)`；任务启动不等待 YOLO 首次推理，避免 CPU 推理把任务释放延后数十秒。

```bash
REPO_ROOT=/home/laurie/AUV_2026_Robocup/YouLong_AUV_Control_System

source /opt/ros/humble/setup.bash

# 准备工作空间 Python 运行时（首次运行或 NumPy 缺失时安装）
if ! "$REPO_ROOT/workspace_auv/.venv/bin/python" -c 'import numpy' >/dev/null 2>&1; then
  bash "$REPO_ROOT/scripts/setup_workspace_python.sh"
fi
export PATH="$REPO_ROOT/workspace_auv/.venv/bin:$PATH"

# 构建仿真栈（先构建其独立依赖）
cd "$REPO_ROOT/workspace_sim"
colcon build --symlink-install
source install/setup.bash

# 构建 AUV 控制栈
cd "$REPO_ROOT/workspace_auv"
colcon build --symlink-install
source install/setup.bash

# 启动当前默认的转盘任务链；此仿真场景无真机转盘，仅用于失败路径冒烟测试
cd "$REPO_ROOT"
mkdir -p /tmp/auv_ros_log
ROS_LOG_DIR=/tmp/auv_ros_log \
ROS_LOCALHOST_ONLY=1 \
LIBGL_ALWAYS_SOFTWARE=1 \
ros2 launch uv_bringup sim.launch.py \
  profile:=sim_dev \
  gpu:=true \
  gpu_backend:=software \
  enable_ai:=true \
  enable_nav:=false \
  enable_task:=true \
  turntable_mode:=true \
  turntable_model_path:="$REPO_ROOT/workspace_auv/src/uv_camera/resource/last.pt" \
  enable_preview:=false \
  wait_for_detections:=false \
  scenario_desc:=water_embodied_intelligence_random.scn \
  mission_file:="$REPO_ROOT/workspace_auv/src/uv_task/config/missions/mapping_grid.json"
```

任务启动后可在另一个终端查看任务状态：

```bash
source /opt/ros/humble/setup.bash
source /home/laurie/AUV_2026_Robocup/YouLong_AUV_Control_System/workspace_auv/install/setup.bash
source /home/laurie/AUV_2026_Robocup/YouLong_AUV_Control_System/workspace_sim/install/setup.bash
ros2 topic echo /task/status
```

按 `Ctrl+C` 会停止整套仿真和任务节点。


### 快速启动仿真

```bash
cd workspace_sim
source install/setup.bash
ros2 launch uv_bringup sim.launch.py
```

按需启用/禁用组件：

```bash
ros2 launch uv_bringup sim.launch.py enable_ai:=false enable_nav:=true enable_task:=true
```

- `enable_ai:=false` — 关闭视觉
- `enable_nav:=true` — 开启导航
- `enable_task:=true` — 开启任务执行器

仿真 bringup 将 Stonefish、sim_bridge、控制、视觉、定位、导航和任务节点统一托管在当前终端中；所有节点的 stdout/stderr 都可以直接查看，关闭 bringup（`Ctrl+C`）时整套进程会一起退出。无图形界面或 CI 环境只需关闭预览：

```bash
ros2 launch uv_bringup sim.launch.py enable_preview:=false
```

如果启动日志在 `Generating ocean waves...` 后出现两个
`Failed to link program!`，先检查是否误用了集成显卡。bringup 默认
`gpu_backend:=auto`：检测到 `/dev/nvidia0` 时会自动选择 NVIDIA PRIME
offload，否则自动选择 Mesa `llvmpipe` 软件 OpenGL。也可以显式指定：

```bash
ros2 launch uv_bringup sim.launch.py gpu_backend:=nvidia
```

强制 NVIDIA 但设备不可用时，启动会立即报出驱动检查提示。没有可用的
NVIDIA 驱动时可以使用 `gpu_backend:=software`。需要保留系统
OpenGL 选择时使用 `gpu_backend:=system`。`gpu:=false` 是 Stonefish 的
无 GPU 可执行项，不适用于包含相机的完整场景。

### 仿真桥（sim_bridge）状态输出

sim_bridge 用 `output='screen'`，所有 PID 计算、推进器混合、状态更新都会打印到终端。

### PID 调参

sim_bridge 的 8 个 PID 控制器运行时可调，无需改代码重启：

| 参数路径 | 控制器 | 级联层级 |
|---|---|---|
| `chassis.pid.pos.x` | X 位置环 | 外环 |
| `chassis.pid.pos.y` | Y 位置环 | 外环 |
| `chassis.pid.pos.z` | Z 位置环 | 外环 |
| `chassis.pid.pos.yaw` | Yaw 位置环 | 外环 |
| `chassis.pid.vel.x` | X 速度环 | 内环 |
| `chassis.pid.vel.y` | Y 速度环 | 内环 |
| `chassis.pid.vel.z` | Z 速度环 | 内环 |
| `chassis.pid.vel.yaw` | Yaw 速度环 | 内环 |

每个控制器有 4 个增益：kp（比例）、ki（积分）、kd（微分）、i_limit（积分限幅）。

**查看当前值：**

```bash
ros2 service call /zit6/get_params zit6_interfaces/srv/GetParams "{paths: []}"
```

返回全部 8 个 PID 的完整配置 JSON。

**运行时修改 PID 参数：**

```bash
# 改单个 PID 的所有增益（推荐）
ros2 service call /zit6/update_params zit6_interfaces/srv/UpdateParams \
  "{json: '{\"chassis.pid.pos.z\": {\"kp\": 800.0, \"ki\": 50.0, \"kd\": 600.0, \"i_limit\": 5000.0}}'}"

# 只改 kp（传单个数值）
ros2 service call /zit6/update_params zit6_interfaces/srv/UpdateParams \
  "{paths: ['chassis.pid.pos.z'], values: ['800.0']}"
```

**永久改参数**：编辑 `sim_bridge.py`，改 `_init_full()` 中的 `_pid_params` 字典和 `_Pid` 构造参数（两个地方都需要改）。建议运行时调参试出好值后再写死。

### 查看 Stonefish 传感器数据

```bash
# 压力传感器
ros2 topic echo /auv/pressure

# IMU
ros2 topic echo /auv/imu

# DVL
ros2 topic echo /auv/dvl

# 里程计
ros2 topic echo /auv/odometry
```

### 查看 ZIT6 协议层

```bash
# 下位机设定值
ros2 topic echo /zit6/cmd/setpoint

# 下位机反馈状态
ros2 topic echo /zit6/state/status
ros2 topic echo /zit6/state/thr
ros2 topic echo /zit6/state/pos
ros2 topic echo /zit6/state/vel
```

## task_runner 调试

### task_runner 状态

task_runner 以 1Hz 发布 `/task/status`：

```bash
ros2 topic echo /task/status
```

输出示例：

```
current: 0, total: 6, name: "start", status: "running"
current: 1, total: 6, name: "setz", status: "running"
current: 1, total: 6, name: "setz", status: "succeeded"
...
current: 5, total: 6, name: "wait", status: "succeeded"
status: "ALL_DONE"
```

停止正在执行的任务：

```bash
ros2 service call /task/stop std_srvs/srv/Trigger
```

### task_runner 日志级别

task_runner 有详细的 INFO 日志，包括每个任务的执行状态和结果：

- 发送 goal 时打印命令类型、目标值、超时
- goal 被接受/拒绝时打印结果
- 每个任务完成时打印 success + message
- 异常时打印 error traceback

### task_runner 支持的任务列表

| 任务名 | 命令类型 | params | 说明 |
|---|---|---|---|
| `start` | START | `{}` | 初始化 odom 原点 |
| `setx` | SET | `{x}` | 绝对定位 X 轴 |
| `sety` | SET | `{y}` | 绝对定位 Y 轴 |
| `setz` | SET | `{z}` | 绝对定位 Z 轴 |
| `setrz` | SET | `{rz}` | 绝对定位偏航 |
| `setxy` | SET | `{x, y}` | 绝对定位 XY |
| `setxyz` | SET | `{x, y, z}` | 绝对定位 XYZ |
| `setxyzrz` | SET | `{x, y, z, rz}` | 绝对定位 XYZ+偏航 |
| `setxyrz` | SET | `{x, y, rz}` | 绝对定位 XY+偏航 |
| `bmovex` | BMOVE | `{dx}` | 机体系步进 X |
| `bmovey` | BMOVE | `{dy}` | 机体系步进 Y |
| `bmovez` | BMOVE | `{dz}` | 机体系步进 Z |
| `bmoverz` | BMOVE | `{drz}` | 机体系步进偏航 |
| `bmovexy` | BMOVE | `{dx, dy}` | 机体系步进 XY |
| `bmovexyz` | BMOVE | `{dx, dy, dz}` | 机体系步进 XYZ |
| `wmovex` | WMOVE | `{dx}` | 世界系步进 X |
| `wmovey` | WMOVE | `{dy}` | 世界系步进 Y |
| `wmovez` | WMOVE | `{dz}` | 世界系步进 Z |
| `wmoverz` | WMOVE | `{drz}` | 世界系步进偏航 |
| `wmovexy` | WMOVE | `{dx, dy}` | 世界系步进 XY |
| `wmovexyz` | WMOVE | `{dx, dy, dz}` | 世界系步进 XYZ |
| `wtravelx` | WTRAVEL | `{dx}` | 世界系直线 X |
| `wtravely` | WTRAVEL | `{dy}` | 世界系直线 Y |
| `wtravelz` | WTRAVEL | `{dz}` | 世界系直线 Z |
| `wtravelxy` | WTRAVEL | `{dx, dy}` | 世界系直线 XY |
| `wtravelxyz` | WTRAVEL | `{dx, dy, dz}` | 世界系直线 XYZ |
| `btravelx` | BTRAVEL | `{dx}` | 机体系直线 X |
| `btravely` | BTRAVEL | `{dy}` | 机体系直线 Y |
| `btravelz` | BTRAVEL | `{dz}` | 机体系直线 Z |
| `btravelxy` | BTRAVEL | `{dx, dy}` | 机体系直线 XY |
| `btravelxyz` | BTRAVEL | `{dx, dy, dz}` | 机体系直线 XYZ |
| `navigate` | SET | `{x, y, z, rz}` | 绝对定位，超时 120s |
| `wait` | — | `{duration}` | 等待 N 秒 |

## 网络/通信调试

### 检查 DDS 发现

```bash
# 列出所有参与的 DDS 节点
ros2 topic list
ros2 node list
```

如果有节点频繁掉线，检查 DDS 配置：

```bash
# 查看 DDS 调试信息（仅限调试）
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

### micro-ROS（HIL/实物）

HIL 模式下 micro-ROS agent 连接 STM32 MCU：

```bash
# HIL 启动
ros2 launch uv_bringup hil.launch.py serial_dev:=/dev/ttyUSB0 serial_baud:=921600

# 检查 micro-ROS agent 是否收到数据（agent log 有 -v 4 的详细输出）
```

micro-ROS agent 的输出直接打印到终端。如果 MCU 离线，agent 会报错：

```
[micro_ros_agent]: [ERROR] [1234567890.123] - Client lost
```

## rqt 可视化

```bash
# 安装 rqt
sudo apt install ros-jazzy-rqt ros-jazzy-rqt-common-plugins

# 启动 rqt
rqt

# 或直接用 rqt_graph 查看节点/话题拓扑
rqt_graph
```

## 常见问题排查

### "odom origin not set"

**问题**：发送运动命令前没有发 START。
**解决**：先发 START goal：

```bash
ros2 action send_goal /basic_motion uv_msgs/action/BasicMotion \
  "{cmd_type: 6, axes: '', target: [0, 0, 0, 0], timeout: 5.0}"
```

### Action server 不接受连接

**问题**：basic_motion 节点没启动或崩溃。
**解决**：检查节点列表和日志：

```bash
ros2 node list | grep basic_motion
# 如果有输出说明节点在运行
```

### 仿真启动后黑屏

Stonefish 在无 GPU 的环境（SSH、WSL）会挂。确保：

1. 使用低负载渲染模式（`sim.launch.py` 默认 `render_quality:=low`；需要更清晰画面时再显式改为 `medium` 或 `high`）
2. 或者用 VNC 连接桌面环境

### 机器人收到定深指令后转圈

**已知 bug**：sim_bridge.py 第 616 行 yaw 极性反转。定位方法：

```bash
# 看 setpoint 的值，yaw 分量是否正？
ros2 topic echo /zit6/cmd/setpoint
```

## 快速参考卡片

| 场景 | 命令 |
|---|---|
| 构建一个包 | `colcon build --packages-select uv_control  --symlink-install` |
| 查看节点 | `ros2 node list` |
| 查看话题 | `ros2 topic list` |
| 看话题内容 | `ros2 topic echo /topic` |
| 看话题频率 | `ros2 topic hz /topic` |
| 发 action goal | `ros2 action send_goal /basic_motion uv_msgs/action/BasicMotion "{...}"` |
| 发 action + 反馈 | `ros2 action send_goal --feedback /basic_motion uv_msgs/action/BasicMotion "{...}"` |
| 调服务 | `ros2 service call /task/stop std_srvs/srv/Trigger` |
| 列参数 | `ros2 param list /node_name` |
| 看日志 | 终端输出（所有节点 `output='screen'`） |
| 关视觉 | `ros2 launch uv_bringup sim.launch.py enable_ai:=false` |
| 开任务 | `ros2 launch uv_bringup sim.launch.py enable_task:=true` |
| 停止任务 | `ros2 service call /task/stop std_srvs/srv/Trigger` |

source /opt/ros/foxy/setup.bash
source /home/nvidia/YouLong_AUV_Control_System/workspace_auv/install/setup.bash

ros2 run uv_record record \
  --record-mode go2rtc \
  --host 127.0.0.1 \
  --port 1984 \
  --go2rtc-stream-mode unannotated \
  --go2rtc-video-format ts \
  --video-codec libx264 \
  --segment-duration 60 \
  --bag-duration 300 \
  --bag-storage sqlite3 \
  --use-sim-time false \
  --topic-regex '^/(zit6/state/.*|(auv/)?basic_motion/pose_info|task/mapping/map|diagnostics)$' \
  --output-root /home/nvidia/YouLong_AUV_Control_System/records/sessions

## 最新 MCU 工程：独立心跳与零点初始化（2026-10-07）

**后续更新：BasicMotion START 已接入 setorigin；通常不再提前手动调用。**
真机 `reset_mcu_origin_on_start` 默认true：START单次请求MCU原点，等待同代有效
`ZitOdom` 后初始化上位机原点。服务拒绝、超时或缺少反馈都返回START失败，不发送移动。
服务超时时请求可能已执行，不应盲目重试；检查MCU原点代数。真机位姿改由
`/zit6/state/odom` 提供，旧`/zit6/state/pos`不再混用。后续移动最多等待5秒确认
新鲜状态中MCU已解锁且导航就绪。它不发心跳，仍由唯一的uv_hm负责。

新启动顺序：连接agent，确认未解锁且导航稳定、关闭其他心跳源 → 启动BasicMotion和
视觉 → 启动唯一uv_hm → 启动包含START的任务链。初次原点未设置时uv_hm不会解锁；
START成功设置原点后，MCU重新积累至少1秒心跳再解锁。若原点已经设置且MCU已解锁，
必须停止任务/控制输出、关闭所有心跳并确认未解锁后，才能再次START。
不能自动停用其他节点的心跳，也不绕过固件解锁状态检查。

```bash
cd /home/nvidia/YouLong_AUV_Control_System/workspace_auv
source /opt/ros/foxy/setup.bash
colcon build --packages-select zit6_interfaces uv_control uv_bringup --symlink-install
source install/setup.bash
ros2 run uv_control basic_motion
# 已在安全未解锁条件下准备好时，可以显式测试START（会重置坐标）：
ros2 action send_goal /basic_motion uv_msgs/action/BasicMotion \
  '{cmd_type: 6, axes: "", target: [0.0, 0.0, 0.0, 0.0], timeout: 0.0}'
```

`sim.launch.py` 显式传入 `reset_mcu_origin_on_start:=false`，保留原仿真START行为。
其他旧固件/仿真若直接run可传 `-p reset_mcu_origin_on_start:=false`，但不能将此用作
新真机固件服务失败时的绕过办法。下面手动setorigin命令保留用于服务诊断，不要在
任务运行或解锁状态执行。

对照 `AUV_zit6_cmake-master`，心跳没有换接口：`/zit6/cmd/agxhbt`，类型
`std_msgs/msg/UInt32`。`uv_hm` 的 `hw_manager` 默认 15 Hz，模式 1 是正常导航模式；
模式 3 是遥控模式（绕过导航解锁检查，不绕过原点要求），不是“强制推力模式”。
固件解锁要求原点已设置、至少 10 个心跳且持续至少 1 秒；正常模式还要求导航有效。
心跳中断超过 1 秒固件会上锁。`watchdog_timeout` 的 7/3 秒仅是上位机反馈告警时间，
不会修改固件的 1 秒安全期限。不要同时在 rqt、GUI 或 heartbeat CLI 发送心跳。

新“重置零点”接口实际为服务 `/zit6/cmd/setorigin`，类型
`zit6_interfaces/srv/SetOrigin`，空请求。只能在未解锁且导航样本有效、新鲜（200 ms）时成功；
返回 `success`、`message`、原始导航原点和 `origin_generation`。
`/zit6/state/odom`（`ZitOdom`）可检查 `origin_initialized`、`nav_valid` 和代数。
接口定义已按最新工程补齐；`ZitStatus`、`ZitSetpoint` 与当前工作区一致，无需替换。

构建（Foxy 真机，新终端）：

```bash
cd /home/nvidia/YouLong_AUV_Control_System/workspace_auv
source /opt/ros/foxy/setup.bash
colcon build --packages-select zit6_interfaces uv_hm --symlink-install
source install/setup.bash
```

建议启动顺序：micro-ROS agent 已连接，停止任务和控制输出，关闭全部心跳源，确认机器人处于
安全、未解锁状态且导航正常。**设置零点会改变坐标系，不能在运行中调用。**
先由操作者执行并检查 `success: true`：

```bash
ros2 service call /zit6/cmd/setorigin zit6_interfaces/srv/SetOrigin '{}'
ros2 topic echo /zit6/state/odom
```

仅在停止 BasicMotion 的独立维护诊断中，才可启动唯一的维护心跳节点
（本操作会请求解锁，不是纯监控）：

```bash
cd /home/nvidia/YouLong_AUV_Control_System/workspace_auv
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 run uv_hm hw_manager --ros-args \
  --params-file "$PWD/src/uv_hm/config/profiles/real_default.yaml" \
  -p enable_heartbeat:=true
```

另一个终端做只读检查：

```bash
ros2 topic info /zit6/cmd/agxhbt --verbose  # 应只有一个发布器
ros2 topic hz /zit6/cmd/agxhbt              # 预期约15 Hz
ros2 topic echo /zit6/state/status         # 查看is_armed/navigation_ready/error_flags
```

以上手动流程仅用于服务诊断。新版任务 `START` 会再次调用 MCU setorigin，
不要同时运行维护心跳和新版 BasicMotion；请停止维护节点，再按文档开头的新顺序启动。
最新固件 `/zit6/state/pos` 已是 MCU odom 坐标，BasicMotion 可把它视作稳定的底层坐标系
再定义任务原点；若运行中重新 setorigin，上位机原点就会失效。
`real.launch.py` 已包含 hw_manager，不要在 launch 之外重复启动一份。
uv_hm 只增加参数检查、重复心跳源告警和 MCU 原点显示，不自动调用服务、不发送位移指令。
