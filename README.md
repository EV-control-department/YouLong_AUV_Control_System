# YouLong AUV Control System

无人自主水下航行器（AUV）控制系统，基于 ROS 2 Jazzy 构建，面向 SAUVC 竞赛。

## 架构

### 系统栈（自底向上）

| 层 | 包 | 说明 |
|---|---|---|
| 仿真 | `stonefish_ros2` (C++) | Stonefish 1.6 水下物理仿真，无 GPU 模式 |
| 硬件管理 | `uv_hm` | SIM：级联 PID + 推力分配 → 6 推进器；实机：hw_manager 占位 |
| 运动控制 | `uv_control` | 运动 API：SET / WMOVE / BMOVE / TRAVEL |
| 感知 | `uv_camera` | YOLO 目标检测 + 单目射线求交 3D 定位 |
| 导航 | `uv_nav` | A* 路径规划 + 避障路径跟踪 |
| 任务 | `uv_task` | YAML mission 竞赛任务顺序执行 |

### 工作区结构

双工作区分治，仿真与控制解耦：

```
YouLong_AUV_Control_System/
├── workspace_auv/        # AUV 控制栈（无仿真依赖）
│   └── src/
│       ├── uv_control/     # 运动控制
│       ├── uv_hm/          # 硬件管理
│       ├── uv_camera/       # 感知
│       ├── uv_nav/         # 导航
│       ├── uv_task/        # 任务执行
│       ├── uv_bringup/     # 启动文件
│       ├── uv_msgs/        # 自定义消息格式
│       └── zit6_interfaces/# ZIT6 协议定义
├── workspace_sim/        # 仿真覆盖层（依赖 workspace_auv）
│   └── src/
│       └── stonefish_ros2/ # Stonefish 仿真器
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
ros2 launch uv_bringup sim.launch.py
```

仿真 Python 节点会自动使用 `workspace_auv/.venv`，其中固定了
`numpy==1.26.4`，以匹配 ROS 2 Jazzy 的 `cv_bridge`。

任务配置已经按 mission 拓扑和 task 参数拆分：

```text
workspace_auv/src/uv_task/config/
├── missions/robocup_26.yaml
└── tasks/*.yaml
```

默认任务链会自动加载 `robocup_26.yaml`。切换自定义任务链时，直接传入
YAML 文件：

```bash
ros2 launch uv_bringup sim.launch.py enable_task:=true \
  mission_file:=/path/to/custom_mission.yaml
```

正式运行入口按模式划分，`profile` 是标准 ROS 2 参数文件预设：

```bash
# SIL 仿真：默认 sim_dev；CI/headless 显式关闭桌面观测
ros2 launch uv_bringup sim.launch.py profile:=sim_dev
ros2 launch uv_bringup sim.launch.py profile:=sim_ci enable_preview:=false

# 混合显卡机器：auto 会在检测到 NVIDIA 设备时自动启用 PRIME offload
ros2 launch uv_bringup sim.launch.py gpu_backend:=auto

# HIL：串口参数直接传给 micro-ROS agent
ros2 launch uv_bringup hil.launch.py profile:=hil_lab \
  serial_dev:=/dev/ttyUSB0 serial_baud:=921600

# 真机：real_safe 使用更保守的功能参数
ros2 launch uv_bringup real.launch.py profile:=real_safe
```

场景 seed 会生成到独立的临时 Data 目录，不会改写源码场景；请使用
`sim.launch.py`、`hil.launch.py`、`real.launch.py` 和 `core_sim.launch.py`
这四个正式入口。

需要并行运行多个 seed 时，为每个 launch 使用不同的 DDS domain：

```bash
ROS_DOMAIN_ID=41 ros2 launch uv_bringup sim.launch.py scene_seed:=1
ROS_DOMAIN_ID=42 ros2 launch uv_bringup sim.launch.py scene_seed:=2
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

## 项目状态

> ⚠️ **本项目处于早期开发阶段。以下内容反映当前已知状态，不完整且可能过时。**

### ✅ 已确认可用

- **basic_motion 节点** — 运动控制核心逻辑已审查和修正（单位一致性、掩码系统清理），内部使用 Action Server 对外暴露接口

### ❓ 待审查/待完善

- **sim_bridge (uv_hm)** — 仿真桥接逻辑未审查
- **hw_manager (uv_hm)** — STM32 MCU 通信仅占位
- **PID 控制参数** — 参数未调优
- **thrust_mixer** — 推力分配矩阵未验证
- **uv_camera** — YOLO 检测与多帧单目射线交会 3D 定位，支持 sim/real 模式
- **astar / navigator (uv_nav)** — 路径规划与避障未审查
- **task_runner (uv_task)** — 竞赛任务执行器未审查
- **stonefish 场景和物理参数** — 仿真场景（`underwater_xunyun.scn` 等）未验证
- **坐标系单位一致性** — 除 basic_motion 外，其他节点的角度/坐标单位未审查
- **测试** — 无单元测试或集成测试

## 坐标系约定

- **NED**（北-东-地）。偏航 0° = 北，顺时针为正
- 角度内部存储单位：**度**；仅在与 ZIT6 协议交互的边界处转换弧度（`set_map` / `_pos_cb`）

## 依赖

- ROS 2 Jazzy
- Python 3
- Stonefish 1.6（仅仿真，https://github.com/patrykcieslak/stonefish）
- PyTorch + YOLOv8（仅感知）
