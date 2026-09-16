# 系统架构（SIL 视角）

> **当前阶段：只做软件在环（SIL）**。HIL/实机路径保留在册但标 ⏸，不参与验收。
> 话题与消息类型不在本文件维护，见 `docs/interfaces/topic_contract.md`。
> 参数见 `docs/node_parameters.md`。约定与坐标系见 `docs/frames/`。
>
> 本文的事实来源：各包 `setup.py` 的 `console_scripts`、`package.xml`、以及各节点源码
> （2026-09-16 核对）。校验：`python3 scripts/check_docs_contract.py --check`。

## 1. 分层

```
层  包                 角色                                       语言/构建
0   stonefish_ros2     Stonefish 1.6 物理引擎封装（含 GPU/无头两种仿真器）   C++ / ament_cmake
1   uv_sim             仿真硬件桥：SIL 下扮演 MCU（PID + 推力混控 + 状态）    Python / ament_python
    uv_hm              实机硬件管理：心跳 + 状态监控（SIL 下不启动）         Python / ament_python
2   uv_control         运动 API：唯一动作入口 basic_motion                   Python / ament_python
3   uv_perception      YOLO 检测（vision）+ 多射线 3D 定位（position）        Python / ament_python
4   uv_nav             A* 路径规划 + 避障                                   Python / ament_python
5   uv_task            JSON 任务执行器（task_runner）                        Python / ament_python
—   uv_bringup         launch 文件集合（**无 entry point**）                 Python / ament_python
—   uv_msgs            自定义 msg/srv/action                                ament_cmake
—   zit6_interfaces    MCU 协议消息与服务                                    ament_cmake
—   visualization/     栈外 PySide6 GUI（非 colcon 包，直接 python3 运行）     Python
```

**依赖方向是单向的**：上层依赖下层的接口（话题/服务/动作），下层**不知道**上层的存在。
唯一的例外是 `uv_control` 通过 `/zit6/cmd/setpoint` 与 `/zit6/state/*` 与"MCU"对话——
而 SIL 里那个"MCU"就是 `uv_sim`，这正是 SIL/HIL 可切换的关键抽象。

## 2. 可执行节点（9 个）

| 可执行 | 包 | 工作区 | 职责 | SIL 是否启动 |
|---|---|---|---|---|
| `stonefish_simulator` | `stonefish_ros2` | workspace_sim | GPU 渲染仿真器（物理 + 传感器 + 渲染） | ✅（需 X/GPU） |
| `stonefish_simulator_nogpu` | `stonefish_ros2` | workspace_sim | 无头仿真器（**拒绝含相机的场景**） | 仅离线物理验证 |
| `sim_bridge` | `uv_sim` | workspace_sim | SIL：PID + 混控 + 状态发布；HIL：相机转发 + 推力转发 | ✅ |
| `basic_motion` | `uv_control` | workspace_auv | 运动动作服务端（START/SET/WMOVE/BMOVE/WTRAVEL/BTRAVEL） | ✅ |
| `vision` | `uv_perception` | workspace_auv | YOLO 检测（`sim_mode` 订阅仿真图像） | 可选（`enable_ai`） |
| `position` | `uv_perception` | workspace_auv | 多射线交会 3D 定位（位姿源 = `PoseInfo`） | 可选（`enable_ai`） |
| `navigator` | `uv_nav` | workspace_auv | A* 规划 + `navigate_to` 服务 | 可选（`enable_nav`） |
| `task_runner` | `uv_task` | workspace_auv | JSON 任务执行（启动即跑 `tasks.json`） | 可选（`enable_task`） |
| `hw_manager` | `uv_hm` | workspace_auv | 实机心跳/监控（依赖真实 MCU 存在） | ❌ ⏸ |

## 3. SIL 数据流（闭环）

```
                 ┌─────────────── 控制下行 ───────────────┐
用户/GUI ──动作──▶ basic_motion ──/zit6/cmd/setpoint──▶ sim_bridge
（task_runner）        ▲                                    │
                       │                           串级 PID + 6 推进器混控
                       │                                    │
                       │                          /auv/thrusters_cmd
                       │                                    ▼
                       │                            stonefish_simulator
                       │                                    │
                       │                        /auv/{odometry,imu,dvl,pressure}
                       │                                    │
                       └──── /zit6/state/{pos,vel,thr,…} ◀──┘
                              （sim_bridge 聚合，~60 Hz）
```

关键点：

1. **`sim_bridge` 是 MCU 的替身**：仿真传感器进、力/力矩出；PID 与混控都在它内部。
   换成真 MCU 时，同一套 `/zit6/*` 接口不变（这就是 HIL，⏸）。
2. **`/auv/odometry` 的 `twist` 是机体系**（`pose` 才是 `world_ned`）；唯一解码点是
   `sim_bridge.body_twist_from_odometry()`，见 `docs/frames/frame_convention.md` §3。
3. **位姿源是 `PoseInfo`**（`basic_motion` 发布 30 Hz）：`position` 用它做射线原点与姿态，
   `navigator` 用它做规划前置，`task_runner` 用它做本地坐标记账。
   `/auv/state`（`AuvState`）已无发布者，属历史。

### 感知支路（可选）

```
/sim/*_cam/*/image_color ─▶ sim_bridge（拼接）─▶ /auv/{front,down}_cam/stitched
                                                        │  (sim_mode:=true)
                                                        ▼
                                                      vision
                                                        │ /perception/detection/*
                                                        ▼
                                                     position ─▶ /perception/objects
                                                        ▲              │
                                     /basic_motion/pose_info         navigator / task_runner
```

## 4. 两条入口（当前都可用，但 plan 里要拆分）

| 入口 | 位置 | 说明 |
|---|---|---|
| `ros2 launch uv_bringup sim_bringup.py` | `workspace_auv` | 完整 SIL：仿真器 + sim_bridge + 控制/感知/（可选）nav/task |
| `ros2 launch uv_bringup sim_core.py` | `workspace_auv` | 极简：仿真器 + sim_bridge |

⚠️ **这两个文件位于 `workspace_auv` 却直接引用 `uv_sim`/`stonefish_ros2`**（违反"workspace_auv
不依赖仿真"的分层，登记为 B1），T4 会把它们迁到仿真侧的 bringup 包。
`uv_bringup/package.xml` **没有**声明 `<exec_depend>uv_sim</exec_depend>` /
`<exec_depend>stonefish_ros2</exec_depend>`（已实测），因此依赖关系在 manifest 层也是缺的——
`check_docs_contract.py` 的依赖检查会报出来。

## 5. 不在本文件维护的内容

| 内容 | 去哪看 |
|---|---|
| 话题、消息类型、发布者/订阅者、缺口登记 | `docs/interfaces/topic_contract.md` |
| 服务与动作的字段语义 | `docs/interfaces/service_action_contract.md`（T3b） |
| 各节点参数与默认值 | `docs/node_parameters.md` |
| 坐标系、外参、TF 现状 | `docs/frames/` |
| SIL/HIL/REAL 边界与所有权 | `docs/architecture/sim_real_boundary.md` |
| 包依赖图与启动顺序 | `docs/architecture/node_dependency.md` |
| 运动动作的完整用法 | `workspace_auv/docs/basic_motion_action.md` |
| 感知实现细节 | `workspace_auv/docs/perception.md` |
| 仿真引擎行为与坑 | `workspace_sim/stonefish_notes.md` |
