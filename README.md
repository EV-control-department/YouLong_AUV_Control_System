# YouLong AUV Control System

无人自主水下航行器（AUV）控制系统，基于 ROS 2 Jazzy 构建，面向 SAUVC 竞赛。

## 架构

### 系统栈（自底向上）

| 层 | 包 | 说明 |
|---|---|---|
| 仿真 | `stonefish_ros2` (C++) | Stonefish 1.6 水下物理仿真，无 GPU 模式 |
| 硬件管理 | `uv_hm` | SIM：级联 PID + 推力分配 → 6 推进器；实机：hw_manager 占位 |
| 运动控制 | `uv_control` | 运动 API：SET / WMOVE / BMOVE / TRAVEL |
| 感知 | `uv_perception` | YOLO 目标检测 + 单目射线求交 3D 定位 |
| 导航 | `uv_nav` | A* 路径规划 + 避障路径跟踪 |
| 任务 | `uv_task` | JSON 竞赛任务顺序执行 |

### 工作区结构

双工作区分治，仿真与控制解耦：

```
YouLong_AUV_Control_System/
├── auv_ws/        # AUV 控制栈（无仿真依赖）
│   └── src/
│       ├── uv_control/     # 运动控制
│       ├── uv_hm/          # 硬件管理
│       ├── uv_perception/  # 感知
│       ├── uv_nav/         # 导航
│       ├── uv_task/        # 任务执行
│       ├── uv_bringup/     # 启动文件
│       ├── uv_msgs/        # 自定义消息格式
│       └── zit6_interfaces/# ZIT6 协议定义
├── sim_ws/        # 仿真覆盖层（依赖 auv_ws）
│   └── src/
│       └── stonefish_ros2/ # Stonefish 仿真器
├── docs/          # 设计文档
└── datas/         # 模型权重、标定数据、参数
```

## 构建与运行

```bash
# AUV 控制栈
cd auv_ws
colcon build --symlink-install && source install/setup.bash
ros2 launch uv_bringup real_bringup.py

# 仿真栈（需要先 source auv_ws）
cd sim_ws
colcon build && source install/setup.bash
ros2 launch uv_bringup sim_bringup.py
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
- **vision (uv_perception)** — YOLO 检测流程未审查
- **position (uv_perception)** — 单目 3D 定位未审查
- **astar / navigator (uv_nav)** — 路径规划与避障未审查
- **task_runner (uv_task)** — 竞赛任务执行器未审查
- **stonefish 场景和物理参数** — 仿真场景（`underwater_xunyun.scn` 等）未验证
- **坐标系单位一致性** — 除 basic_motion 外，其他节点的角度/坐标单位未审查
- **启动配置** — `sim_bringup.py` 中仍引用已移除的 `minimal_control` 节点
- **测试** — 无单元测试或集成测试

## 坐标系约定

- **NED**（北-东-地）。偏航 0° = 北，顺时针为正
- 角度内部存储单位：**度**；仅在与 ZIT6 协议交互的边界处转换弧度（`set_map` / `_pos_cb`）

## 依赖

- ROS 2 Jazzy
- Python 3
- Stonefish 1.6（仅仿真，https://github.com/patrykcieslak/stonefish）
- PyTorch + YOLOv8（仅感知）
