# YouLong AUV Control System

无人自主水下航行器（AUV）控制系统，兼容 ROS 2 Foxy 和 Jazzy，面向 SAUVC
竞赛。Edge 设备保持 Foxy，开发/仿真环境可使用 Jazzy。

## 架构

### 系统栈（自底向上）

| 层 | 包 | 说明 |
|---|---|---|
| 协议 | `auv_protocol` | `/auv` canonical topic/service/action 注册表 |
| 描述 | `auv_description` | real 车辆 URDF 与 PDF 机械几何、静态 TF |
| 硬件管理 | `uv_hm` | ZIT6 adapter、heartbeat、状态监控与 watchdog |
| 运动控制 | `uv_control` | BasicMotion：SET / WMOVE / BMOVE / TRAVEL / BODY_VELOCITY |
| 相机/感知 | `uv_camera`, `uv_perception` | 相机采集、标定、YOLO 和目标观测 |
| 定位 | `uv_localization` | `/auv/state/*` 估计状态边界（当前 bootstrap） |
| 规划/导航 | `uv_planning`, `uv_nav` | planning 边界与 A* 兼容后端 |
| 任务 | `uv_task` | YAML mission 与竞赛任务顺序执行 |
| 仿真 | `uv_sim_description`, `uv_sim_bridge`, `stonefish_ros2` | Stonefish 世界、传感器 adapter 和 SIL/HIL |
| 实验 | `uv_sim_degradation`, `uv_sim_evaluation`, `experiments/` | 退化注入、ATE/RPE 和结果目录 |

### 工作区结构

双工作区分治，仿真与控制解耦：

```
YouLong_AUV_Control_System/
├── workspace_auv/        # AUV 控制栈（无仿真依赖）
│   └── src/
│       ├── auv_protocol/   # /auv 接口注册表
│       ├── auv_description/# real URDF、机械几何与静态 TF
│       ├── uv_msgs/        # canonical 消息、服务和 Action
│       ├── uv_control/     # 运动控制
│       ├── uv_hm/          # 真机硬件 adapter
│       ├── uv_camera/      # 相机 IO、标定和视觉兼容层
│       ├── uv_perception/  # 感知接口边界
│       ├── uv_localization/# 状态估计边界
│       ├── uv_planning/    # 规划接口边界
│       ├── uv_nav/         # A* 过渡后端
│       ├── uv_task/        # 任务执行
│       ├── uv_bringup/     # real/通用启动
│       └── zit6_interfaces/# ZIT6 固件协议定义
├── workspace_sim/        # 仿真覆盖层（依赖 workspace_auv）
│   └── src/
│       ├── uv_sim_description/ # 仿真 URDF 与静态 TF
│       ├── uv_sim_bridge/     # Stonefish canonical adapters
│       ├── uv_sim_degradation/# 传感器退化
│       ├── uv_sim_evaluation/ # 真值评测
│       ├── uv_sim_bringup/    # SIM/HIL launch
│       ├── stonefish_ros2/    # Stonefish 仿真器
│       └── zit6_control_core/ # 固件控制核 host adapter
├── docs/          # 设计文档
└── datas/         # 模型权重、标定数据、参数
```

## 构建与运行

```bash
# 首次使用仿真环境时，创建工作空间本地 Python 运行时
bash scripts/setup_workspace_python.sh

# AUV 控制栈
cd workspace_auv
colcon build --symlink-install && source install/setup.bash
ros2 launch uv_bringup real.launch.py

# 仿真栈（需要先 source workspace_auv）
cd workspace_sim
colcon build && source install/setup.bash
ros2 launch uv_sim sim.launch.py \
  world:=guoshui_2026/cruise_seeded vehicle:=youlong
```

`uv_sim_assets` 是 Stonefish 资源的唯一维护入口：`vehicles/` 保存车辆，
`worlds/` 保存环境，`objects/` 和 `textures/` 保存可复用比赛物体。旧脚本仍可
调用 `uv_sim_bringup sim.launch.py scenario_desc:=...`；旧的场景 basename 会映射到
迁移后的维护场景或 `worlds/examples/`，其余旧 fixture 从 `legacy_data/` 运行。新的入口同时指定 `world:=` 和
`scenario_desc:=` 会直接报错，避免场景选择歧义。

常用 world 名称：

```text
guoshui_2026/cruise
guoshui_2026/cruise_seeded
sauvc_2026/finals
sauvc_2026/qualification
sauvc_2026/pool
```

仿真 Python 节点会自动使用 `workspace_auv/.venv`。依赖文件会按 Python 版本
选择 NumPy 1.x：Foxy/Python 3.8 使用 `<1.25`，Jazzy 使用 `1.26.4`，以避免
`cv_bridge` 的 ABI 不匹配。

任务配置已经按 mission 拓扑和 task 参数拆分：

```text
workspace_auv/src/uv_task/config/
├── missions/robocup_26.yaml
└── tasks/*.yaml
```

未指定时默认自动加载 `robocup_26.yaml`。`mission_file` 同时支持任务链
YAML 和单个 task YAML；例如直接执行过门任务：

```bash
ros2 launch uv_sim_bringup sim.launch.py enable_task:=true \
  mission_file:=/home/doc049/dev/UUV/YouLong_AUV_Control_System/workspace_auv/src/uv_task/config/tasks/26rb_gate_task.yaml
```

自定义任务链仍可传入 `missions/*.yaml` 文件。

正式运行入口按模式划分，`profile` 是标准 ROS 2 参数文件预设：

```bash
# SIL 仿真：默认 sim_dev；CI/headless 显式关闭桌面观测
ros2 launch uv_sim_bringup sim.launch.py profile:=sim_dev
ros2 launch uv_sim_bringup sim.launch.py profile:=sim_ci enable_preview:=false

# 竞赛 world 预设；显式 world:= 会覆盖 profile 选择
ros2 launch uv_sim sim.launch.py profile:=sauvc_finals
ros2 launch uv_sim sim.launch.py profile:=guoshui_cruise_seeded

# 混合显卡机器：默认自动使用容器中的 NVIDIA；需要系统 OpenGL 时显式指定
ros2 launch uv_sim_bringup sim.launch.py gpu_backend:=auto
ros2 launch uv_sim_bringup sim.launch.py gpu_backend:=nvidia
ros2 launch uv_sim_bringup sim.launch.py gpu_backend:=system

# HIL：串口参数直接传给 micro-ROS agent
ros2 launch uv_sim_bringup hil.launch.py profile:=hil_lab \
  serial_dev:=/dev/ttyUSB0 serial_baud:=921600

# 真机：real_safe 使用更保守的功能参数
ros2 launch uv_bringup real.launch.py profile:=real_safe
```

场景 seed 会生成到独立的临时 Data 目录，不会改写源码场景；请使用
`sim.launch.py`、`hil.launch.py`、`real.launch.py` 和 `core_sim.launch.py`
这四个正式入口。

需要并行运行多个 seed 时，为每个 launch 使用不同的 DDS domain：

```bash
ROS_DOMAIN_ID=41 ros2 launch uv_sim_bringup sim.launch.py scene_seed:=1
ROS_DOMAIN_ID=42 ros2 launch uv_sim_bringup sim.launch.py scene_seed:=2
```

## 一键部署到 AUV 电脑

项目提供了基于 `rsync` 的部署脚本，默认同步到 `nvidia@192.168.16.10:~/YouLong_AUV_Control_System`：

```bash
# 首次使用前确认本机和目标机已安装 ssh、rsync，并配置好 SSH 登录
chmod +x scripts/deploy.sh
./scripts/deploy.sh
```

为了真正做到一键执行，建议先配置 SSH 公钥登录；部署脚本不会保存 SSH 密码：

```bash
ssh-copy-id nvidia@192.168.16.10
ssh nvidia@192.168.16.10
```

脚本默认不会同步 `.git`、ROS 2 的 `build/install/log` 和 Python 缓存。建议首次部署先执行预览：

```bash
./scripts/deploy.sh --dry-run
```

如果只想按文件内容 checksum 检查并部署指定文件：

```bash
./scripts/deploy.sh --checksum --dry-run \
  workspace_auv/src/uv_camera/uv_camera/composed.py
./scripts/deploy.sh --checksum \
  workspace_auv/src/uv_camera/uv_camera/composed.py
```

也可以同时指定多个文件或目录：

```bash
./scripts/deploy.sh --checksum \
  workspace_auv/src/uv_camera/uv_camera/composed.py \
  workspace_auv/src/uv_camera/config
```

`--checksum` 会让 rsync 读取本地和远端文件内容进行比较，只传输内容不同的文件。
指定路径时不能同时使用 `--delete`。

也可以直接让 Git 生成部署文件列表。部署当前工作区相对 `HEAD` 的修改：

```bash
./scripts/deploy.sh --git-changed --checksum --dry-run
./scripts/deploy.sh --git-changed --checksum
```

部署某个提交范围内的文件：

```bash
./scripts/deploy.sh --git-range HEAD~1..HEAD --checksum
```

Git 删除的文件不会被选择性部署删除；如需删除远端多余文件，请确认后对整个项目使用 `--delete`。

如需让远端目录与本地完全一致，可显式启用删除模式：

```bash
./scripts/deploy.sh --delete
```

目标地址、目录、SSH 端口和私钥可通过环境变量覆盖：

```bash
DEPLOY_HOST=192.168.16.10 \
DEPLOY_PATH='~/YouLong_AUV_Control_System' \
SSH_KEY=~/.ssh/auv \
./scripts/deploy.sh
```

## 当前重构状态

- `workspace_auv` 不依赖 `uv_sim` 或 `stonefish_ros2`，可以独立构建并启动
  `uv_bringup/real.launch.py`。
- 所有新接口统一使用 `/auv/...`；旧 `/task/*`、`/basic_motion*`、`/zit6/*`
  仅作为兼容入口。
- `/auv/state/odom`、`/auv/state/twist` 是控制、规划和任务的估计状态入口；
  `/auv/sim/ground_truth/*` 只交给 `uv_sim_evaluation`。
- 当前定位后端是可替换的 bootstrap estimator，不宣称已经实现 FGO、SLAM 或
  Active SLAM；这些功能可在保持 canonical 接口的前提下继续接入。
- real 端机械位置以
  [`docs/auv元件说明和标定数据_第四版.pdf`](docs/auv元件说明和标定数据_第四版.pdf)
  为准。PDF 未给出 DVL 安装外参，因此 real URDF 不发布未经测量的
  `dvl_link`。

详细协议、坐标系、启动图和真值隔离见 [`docs/architecture/`](docs/architecture/)。

## 坐标系约定

- **NED**（北-东-地）。偏航 0° = 北，顺时针为正
- 算法和 canonical 接口内部使用 **rad**；历史 `PoseInfo` 字段仍为度，
  仅在兼容消息边界转换。距离为 m、速度为 m/s、角速度为 rad/s。

## 依赖

- ROS 2 Foxy（Edge）或 ROS 2 Jazzy（开发/仿真）
- Python 3
- Stonefish 1.6（仅仿真，https://github.com/patrykcieslak/stonefish）
- PyTorch + YOLOv8（仅感知）

核心 ROS 镜像不会强制安装 PyTorch，因为 GPU/CUDA wheel 与机器相关；YOLO
推理、自动标注和训练需要时执行：

```bash
INSTALL_WORKSPACE_AI=true bash scripts/setup_workspace_python.sh
```

对应依赖记录在 `requirements-ai.txt`。不安装它时，`uv_camera` 会保留相机
和 ROS 控制功能，但 AI 检测会按代码设计自动禁用。
