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
1   uv_sim             仿真侧 7 节点：传感器转发层（odom/imu/dvl/pressure）+    Python / ament_python
                       nav_sim + mcu_sim（仿真 MCU）+ camera_relay
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
而 SIL 里那个"MCU"就是 `uv_sim` 的 `mcu_sim`，这正是 SIL/HIL 可切换的关键抽象。

## 2. 可执行节点（15 个）

| 可执行 | 包 | 工作区 | 职责 | SIL 是否启动 |
|---|---|---|---|---|
| `stonefish_simulator` | `stonefish_ros2` | workspace_sim | GPU 渲染仿真器（物理 + 传感器 + 渲染） | ✅（需 X/GPU） |
| `stonefish_simulator_nogpu` | `stonefish_ros2` | workspace_sim | 无头仿真器（**拒绝含相机的场景**） | 仅离线物理验证 |
| `odom_sim`/`imu_sim`/`dvl_sim`/`pressure_sim` | `uv_sim` | workspace_sim | `/auv/*` → `/sim/sensor/*` 转发层（`dvl_sim` 兼 AQUA-SLAM 适配器） | ✅（HIL 下只起前两个） |
| `nav_sim` | `uv_sim` | workspace_sim | 导航解组装：`/sim/sensor/*` → `/zit6/state/{pos,vel}`（SIL）或 `/zit6/sim/nav`（HIL） | ✅ |
| `mcu_sim` | `uv_sim` | workspace_sim | 仿真 MCU：setpoint→PID→混控→推力 + 状态 + 参数服务；HIL 只做推力转发 | ✅ |
| `camera_relay` | `uv_sim` | workspace_sim | 相机转发 + 左右目拼接 | ✅ |
| `laser_sim` | `uv_sim` | workspace_sim | 线激光适配：`/auv/laser_scan` → `/sim/sensor/laser_points`（条纹三维点） | ✅（无消费者） |
| `basic_motion` | `uv_control` | workspace_auv | 运动动作服务端（START/SET/WMOVE/BMOVE/WTRAVEL/BTRAVEL） | ✅ |
| `vision` | `uv_perception` | workspace_auv | YOLO 检测（`sim_mode` 订阅仿真图像） | 可选（`enable_ai`） |
| `position` | `uv_perception` | workspace_auv | 多射线交会 3D 定位（位姿源 = `PoseInfo`） | 可选（`enable_ai`） |
| `navigator` | `uv_nav` | workspace_auv | A* 规划 + `navigate_to` 服务 | 可选（`enable_nav`） |
| `task_runner` | `uv_task` | workspace_auv | JSON 任务执行（启动即跑 `tasks.json`） | 可选（`enable_task`） |
| `hw_manager` | `uv_hm` | workspace_auv | 实机心跳/监控（依赖真实 MCU 存在） | ❌ ⏸ |

## 3. SIL 数据流（闭环）

```
                 ┌─────────────── 控制下行 ───────────────┐
用户/GUI ──动作──▶ basic_motion ──/zit6/cmd/setpoint──▶ mcu_sim
（task_runner）        ▲                                    │
                       │                           串级 PID + 6 推进器混控
                       │                                    │
                       │                          /auv/thrusters_cmd
                       │                                    ▼
                       │                            stonefish_simulator
                       │                                    │
                       │                        /auv/{odometry,imu,dvl,pressure}
                       │                                    │
                       │                   转发层 ×4（odom/imu/dvl/pressure_sim）
                       │                                    │
                       │                            /sim/sensor/*
                       │                          ┌─────────┴─────────┐
                       │                          ▼                   ▼
                       │                       nav_sim             mcu_sim
                       │                          │              （PID 反馈）
                       └──── /zit6/state/{pos,vel} ◀──┘   （30 / 60 Hz）
```

关键点：

1. **`mcu_sim` 是 MCU 的替身**：仿真传感器进、力/力矩出；PID 与混控都在它内部。
   换成真 MCU 时，同一套 `/zit6/*` 接口不变（这就是 HIL，⏸）。
2. **`/auv/odometry` 的 `twist` 是机体系**（`pose` 才是 `world_ned`）；唯一解码点是
   `uv_sim/odometry.py`（`nav_sim` 与 `mcu_sim` 共用），见 `docs/frames/frame_convention.md` §3。
3. **`/zit6/state/*` 单发布者（I2）**：SIL 由 `nav_sim`(pos/vel) + `mcu_sim`(status/thr/zithbt)
   发布；HIL 下全部归真 MCU，此时 `nav_sim` 只发 `/zit6/sim/nav`、`mcu_sim` 只转发推力。
4. **转发层（`/sim/sensor/*`）是我们自己的命名空间**：引擎话题变它不变，
   而且 T6 的 AQUA-SLAM 消费的是它（`/sim/sensor/dvl` 带 4 路合成波束）。
5. **位姿源是 `PoseInfo`**（`basic_motion` 发布 30 Hz）：`position` 用它做射线原点与姿态，
   `navigator` 用它做规划前置，`task_runner` 用它做本地坐标记账。
   `/auv/state`（`AuvState`）已无发布者，属历史。

### 感知支路（可选）

```
/sim/*_cam/*/image_color ─▶ camera_relay（拼接）─▶ /auv/{front,down}_cam/stitched
                                                        │  (sim_mode:=true)
                                                        ▼
                                                      vision
                                                        │ /perception/detection/*
                                                        ▼
                                                     position ─▶ /perception/objects
                                                        ▲              │
                                     /basic_motion/pose_info         navigator / task_runner
```

## 4. 入口：实机 / 仿真各一个（T4 起分层干净）

| 入口 | 所在工作区 | 说明 |
|---|---|---|
| `ros2 launch uv_bringup real_bringup.py` | `workspace_auv` | **唯一实机入口**：`uv_hm`（agent + hw_manager）+ 控制/感知/（可选）nav/task |
| `ros2 launch uv_sim_bringup sim_bringup.py` | `workspace_sim` | 完整 SIL：Stonefish + `uv_sim` 的 7 个节点 + 控制/感知/（可选）nav/task |
| `ros2 launch uv_sim_bringup sim_bringup.py enable_motion:=false enable_ai:=false` | `workspace_sim` | 等价于已删除的 `sim_core`：只有引擎 + 仿真 MCU（7 个仿真节点仍在） |
| `ros2 launch uv_sim_bringup hil_bringup.py` | `workspace_sim` | HIL ⏸：Stonefish + 真 MCU |

规则：**节点定义归各包，系统预设只做编排**。预设文件里没有 `Node(...)`，只有 include +
条件；每个包（`uv_control`/`uv_perception`/`uv_nav`/`uv_task`/`uv_hm`/`uv_sim`）各有一个
`launch/` 文件声明自己的节点，Stonefish 直接复用引擎自带的 launch。

因此 `workspace_auv` **不含任何仿真依赖**（不变式 I6）：干净环境只 source `/opt/ros/jazzy`
与 `workspace_auv/install` 即可 `launch real_bringup.py`；入口清单与新旧映射见
`docs/interfaces/launch_entries.md`。这两条都由 `check_docs_contract.py` 的
`launch-targets` / `launch-refs` 检查与 `test_launch_contract.py` 强制。

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
