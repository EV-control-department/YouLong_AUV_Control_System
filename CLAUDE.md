# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Workspace Structure

Two colcon workspaces in this repo:

```
YouLong_AUV_Control_System/
  workspace_auv/       # AUV control stack (no sim dependency)
  workspace_sim/       # Simulation overlay (sources workspace_auv, adds stonefish_ros2)
```

### Build & Run — AUV Workspace (`workspace_auv/`)

```bash
cd workspace_auv
colcon build                          # build all packages
colcon build --packages-select uv_control  # build one package
colcon build --symlink-install        # symlink install (Python + launch changes take effect without rebuild)
source install/setup.bash             # source the workspace overlay BEFORE any ros2 command

ros2 launch uv_bringup real_bringup.py   # real AUV stack
```

**修改后是否需要重新编译：**
- **启动文件 (launch/)** → 需要 `colcon build`（除非用了 `--symlink-install`）
- **Python 源码** → 需要 `colcon build`（除非用了 `--symlink-install`）
- **msg/srv/action 定义** → 必须 `colcon build`（ament_cmake，symlink 无效）
- **C++ 源码** → 必须 `colcon build`
- **JSON 配置文件** → 无需编译，直接生效

**优先使用 `--packages-select` 只编译修改的包：**
```bash
# 只编译你改过的包，比全量 colcon build 快得多
colcon build --packages-select uv_bringup    # 改了 launch 文件
colcon build --packages-select uv_control    # 改了 control 代码
colcon build --packages-select uv_msgs       # 改了 msg/srv/action 定义（依赖它的包也要重新编译）
```

### Build & Run — Sim Workspace (`workspace_sim/`)

```bash
cd workspace_sim
colcon build                          # builds stonefish_ros2 only
source install/setup.bash             # also sources workspace_auv underlay

ros2 launch uv_bringup sim_bringup.py   # full simulation stack
```

Note: `workspace_sim` uses colcon overlay — source `workspace_auv/install/setup.bash` first, then `workspace_sim/install/setup.bash`.

## Architecture

This is a **ROS 2 Jazzy** project for the **YouLong AUV** (autonomous underwater vehicle), designed for the SAUVC competition. It uses colcon with mixed ament_python (application logic) and ament_cmake (custom messages, Stonefish wrapper) packages.

### 6-Layer Stack (bottom-up)

| Layer | Package | Role |
|---|---|---|
| 0 | `stonefish_ros2` (C++) | Stonefish 1.6 marine physics simulator. GPU mode runs `stonefish_simulator`, headless runs `stonefish_simulator_nogpu`. Lives in `workspace_sim/` |
| 1 | `uv_hm` + `uv_sim` | Cascaded PID + thruster mixing → 6 thrusters. `sim_bridge` (package **`uv_sim`**, **`workspace_sim/`**) for SIL; `hw_manager` (package `uv_hm`, `workspace_auv/`) keeps the STM32 MCU armed and monitors `/zit6/state/*` |
| 2 | `uv_control` | Motion API: SET/WMOVE/BMOVE/TRAVEL. Only entry point is `basic_motion` |
| 3 | `uv_perception` | 4 通道双目 YOLO 检测 + 多帧单目射线交会 3D 定位。sim/real 模式切换，发布 PoseInfo 位姿源 |
| 4 | `uv_nav` | A* pathfinding + obstacle avoidance waypoint following |
| 5 | `uv_task` | JSON-based competition task runner |

### Action Server — `basic_motion` (`/basic_motion`)

`basic_motion` exposes a `uv_msgs/action/BasicMotion` Action Server. This is the primary entry point for all motion control — nodes send action goals and block/wait for completion.

#### Action 定义

```
# Goal
uint8 cmd_type      # SET=3, WMOVE=1, BMOVE=2, WTRAVEL=4, BTRAVEL=5, START=6
string axes          # 生效轴 "x"/"y"/"z"/"rz" 任意组合，空=全部
float32[] target     # [x, y, z, yaw] — 含义取决于 cmd_type
float32 timeout      # 超时秒数，≤0 默认 60s
---
# Result
bool success
string message
---
# Feedback
float32 distance_remaining   # 3D 欧氏距离
```

#### 坐标系约定

- **odom 系**：以 START 时 AUV 位置为原点 (0,0,0)，初始朝向为 yaw=0°
- **yaw 单位**：度（°），NED 系顺时针为正
- **NED 轴**：x=北(N), y=东(E), z=下(D, 正数更深)

#### 6 种命令类型

| 命令 | 值 | target 含义 | 运动模式 | 行为 |
|---|---|---|---|---|
| **START** | 6 | 忽略 | — | 初始化 odom 原点。必须在所有运动命令之前发送一次 |
| **SET** | 3 | 绝对坐标 `[x, y, z, yaw]` | `set_world` + `wait_reached` | 直接发往目标，等到达。简单直接 |
| **WMOVE** | 1 | 世界系偏移 `[dx, dy, dz, dyaw]` | `step_move_world` | 拆分长距离为动态步进段。横向误差大时自动减小步长 |
| **BMOVE** | 2 | 机体系偏移 `[dx, dy, dz, dyaw]` | `body_to_world` + `step_move_world` | 先将 body 偏移旋转到世界系，再走 WMOVE |
| **WTRAVEL** | 4 | 世界系偏移 `[dx, dy, dz]` | 转向 + 直线 step_move | 先转向目标方向 (atan2)，再沿 body-X 前进 |
| **BTRAVEL** | 5 | 机体系偏移 `[dx, dy, dz]` | `body_to_world` + WTRAVEL | 先转 body→world，再走 WTRAVEL |

#### START 命令

```python
goal = BasicMotion.Goal()
goal.cmd_type = BasicMotion.Goal.START
# 其他字段忽略
```

- 记录当前 map 位姿为 odom 原点，之后所有坐标相对此原点
- odom 0° = AUV 初始朝向（yaw 也做偏移）
- 例：AUV 朝东 (map 90°) → START → odom 0° = 东，`SET(5,0,0,0)` 向东走 5m
- 只发一次，重复发送是无操作（log 提示已设置）

#### SET 命令

```python
goal.cmd_type = BasicMotion.Goal.SET
goal.target = [5.0, 0.0, -2.0, 90.0]   # odom x=5, y=0, z=-2, yaw=90°(朝东)
```

- **绝对定位**：直接发往 odom 系坐标，阻塞等待到达
- 容差：x/y/z=0.1m, yaw=5°
- 超时默认 60s
- 适用于已知精确目标位置的场景

#### WMOVE 命令

```python
goal.cmd_type = BasicMotion.Goal.WMOVE
goal.target = [2.0, 1.0, 0.0, 0.0]     # 世界系：向北 2m，向东 1m
```

- **世界系步进**：从当前位置加上偏移量为目标
- 使用动态步进算法：长距离拆成小段，每步根据横向误差动态调整步长
- 步进参数：STEP_X=0.6m, STEP_Y=0.2m（椭圆模型），STEP_PERIOD=0.3s
- 收敛判据：预估剩余时间 ≤ 0.15s，或前向误差 < 步长×0.3

#### BMOVE 命令

```python
goal.cmd_type = BasicMotion.Goal.BMOVE
goal.target = [2.0, 0.0, 0.0, 0.0]     # 机体系：向前 2m（当前朝向）
```

- **机体系步进**：参数为机体系偏移量，内部用 `body_to_world` 转世界系
- body-x = 向前（AUV 机头方向），body-y = 向右
- BMOVE (dx=2) 与 BMOVE (dx=-2) 结合可实现前进后退
- 旋转后走 WMOVE 的步进逻辑

#### WTRAVEL 命令

```python
goal.cmd_type = BasicMotion.Goal.WTRAVEL
goal.target = [3.0, 4.0, 0.0, 0.0]     # 世界系：向 (3,4) 方向直线移动
```

- **世界系直线移动**：先转向目标方向，再沿 body-X 前进
- 第一步：在当前原地转向至 `atan2(dy, dx)`
- 第二步：沿 body-X 轴直线前进到目标
- yaw 被忽略（由方向决定）
- 适用于需要直线轨迹的任务（过门、巡线等）

#### BTRAVEL 命令

```python
goal.cmd_type = BasicMotion.Goal.BTRAVEL
goal.target = [3.0, 0.0, 0.0, 0.0]     # 机体系：向前直线 3m
```

- **机体系直线移动**：body 偏移转 world 后走 WTRAVEL
- 先转 body→world，再转向+前进

#### 结果与反馈

```python
# Result
result.success    # True=成功, False=超时/拒绝/取消
result.message    # 空="成功", "cancelled", "motion timeout", 或其他错误

# Feedback (0.5Hz, 仅运动中有值)
feedback.distance_remaining   # 到目标的 3D 欧氏距离（不含 yaw 误差）
```

#### 使用示例

```python
import rclpy
from rclpy.action import ActionClient
from uv_msgs.action import BasicMotion

rclpy.init()
node = Node('my_client')
client = ActionClient(node, BasicMotion, 'basic_motion')

# 1. 等待 server 就绪
client.wait_for_server()

# 2. START（初始化 odom 原点）
goal = BasicMotion.Goal()
goal.cmd_type = BasicMotion.Goal.START
send_future = client.send_goal_async(goal)
rclpy.spin_until_future_complete(node, send_future)
gh = send_future.result()
result_future = gh.get_result_async()
rclpy.spin_until_future_complete(node, result_future)
print('START:', result_future.result().result)

# 3. SET 绝对定位
goal = BasicMotion.Goal()
goal.cmd_type = BasicMotion.Goal.SET
goal.target = [5.0, 0.0, -2.0, 90.0]   # x=5, y=0, z=-2, yaw=90°
goal.timeout = 30.0
send_future = client.send_goal_async(goal)
rclpy.spin_until_future_complete(node, send_future)
gh = send_future.result()
result_future = gh.get_result_async()
rclpy.spin_until_future_complete(node, result_future)
result = result_future.result().result
print('SET:', result.success, result.message)
```

#### 注意事项

1. **必须先 START**：任何 SET/WMOVE/BMOVE/TRAVEL 之前必须发送 START。如果 odom 原点未设置，server 会 reject goal 并返回 `"odom origin not set, call start() first"`
2. **target 必须 4 元素**：就算 WTRAVEL/BTRAVEL 忽略 yaw，也必须传 4 个值。否则 server abort
3. **timeout ≤ 0** 自动用 60s
4. **axes 字段**当前仅用于日志记录，server 内部未使用（始终全轴生效）
5. **MultiThreadedExecutor**：basic_motion 内部使用多线程执行器，action 回调在独立线程运行，不阻塞主循环

### task_runner (`uv_task`)

`workspace_auv/src/uv_task/uv_task/task_runner.py` is the competition task executor. It reads a JSON task list (`config/tasks.json`) and executes tasks sequentially using BasicMotion ActionClient.

**Key behaviors:**
- **Auto-start**: on launch, if `tasks.json` has entries, a daemon thread immediately starts executing them (no service call needed)
- **Task map**: JSON `name` field maps to a method via the `task_map` dict in `task_runner.py`. Beyond the SET/WMOVE/BMOVE/WTRAVEL/BTRAVEL axis variants (e.g. `setxyzrz`, `bmovexy`, `wtravelx`), plus `start`, `navigate`, `wait`, it exposes competition-specific actions: `follow_line`, `arrow_surface`, `drop_beacon`, `take_water_sample`, `get_sample`, `release_sampler`. **`task_runner.py`'s `task_map` is the authoritative list**; `docs/debug_guide.md` lags behind it.
- **ROS parameters**: `debug_mode`, `down_image_width`, `down_image_height`, `align_ema_alpha` (see `docs/node_parameters.md`)
- **Stop service** (`/task/stop`): sets `self.stopped = True` to interrupt the current goal and abort the task list
- **Status publisher** (`/task/status`, 1Hz): publishes current task index, total count, task name, status string

**ActionClient pattern (SingleThreadedExecutor + daemon thread):**

```python
# Daemon thread sends goals, main thread runs rclpy.spin()
# Future polling (NOT spin_until_future_complete):
send_future = client.send_goal_async(goal)
while rclpy.ok() and not self.stopped and not send_future.done():
    time.sleep(0.01)
```

The main thread runs `rclpy.spin(node)` with SingleThreadedExecutor; the work thread polls futures in a loop. `spin_until_future_complete` is NOT used (creates a second executor → conflicts).

**Position tracking:** task_runner tracks commanded position locally (`_cmd_x/_cmd_y/_cmd_z/_cmd_yaw`) rather than subscribing to AuvState, because AuvState is only published by the simulator, not by the real AUV hardware.

### AuvState — Sim-Only Topic

`uv_msgs/msg/AuvState` is intended for simulation only, and **nothing publishes `/auv/state`** any more (the old `stonefish_ros2` publisher is gone; sim_bridge publishes `/zit6/state/*` and `/zit6/sim/nav`). Real AUV hardware never publishes it. Nodes running on the real AUV must not depend on it.

For pose feedback, both perception and navigation use `/basic_motion/pose_info` (`uv_msgs/PoseInfo`, 30Hz) published by `basic_motion`. This works in both simulation and real-hardware modes — `navigator.py` was migrated to it (2026-09-13), and it refuses to plan until a pose has arrived.

The `AuvState` message itself is now unused by any node; only `uv_msgs` still defines it.

### sim_bridge (`uv_sim`, workspace_sim)

`sim_bridge` lives at `workspace_sim/src/uv_sim/uv_sim/sim_bridge.py` (package `uv_sim`, **not** `uv_hm`). It plays the role of the MCU in SIL mode: `ZitSetpoint` in → PID + thruster mixing → `/auv/thrusters_cmd`, and publishes `/zit6/state/{pos,vel,thr,status,zithbt}`. With `hil_mode:=True` it only forwards camera images and relays `/zit6/state/thr` → thrusters, leaving PID/mixing to the real MCU.

Historical note: the yaw-polarity bug documented in earlier revisions (`cmd_rz = -float(self.pid_yaw_rate.step(...))`) has been **fixed** — the negation is gone (now `cmd_rz = float(...)`). Do not re-apply that fix.

**Sim sensor geometry (2026-09-15, aligned with the real vehicle).** The XUNYUN DVL in `workspace_sim/src/stonefish_ros2/Data/xunyun_fixed.scn` is mounted flush with the body (`origin rpy="0 0 0" xyz="-0.375 0 0.1"`) with `<specs beam_angle="30.0" beam_positive_z="true"/>`, matching the real DVL (yaw=0 mount, beams offset 45°, 30° off vertical, z≈+0.1 m). Consequences worth remembering:
- `/auv/dvl` therefore carries **body-frame** velocity. The pre-2026-09-15 scene used `rpy=(3.1416 0 -0.785)` with the default `beam_positive_z=false`, which made Stonefish publish velocity in a frame rotated 45° from the body with y/z flipped; `sim_bridge` forwards that straight into `/zit6/state/vel`, so `basic_motion`'s along-track speed became `V·cos(2α+45°)` (≈0 at α=22.5°). Regression pinned by `workspace_sim/src/uv_sim/test/test_scene_sensors.py`; end-to-end check in `workspace_sim/tools/check_dvl_frame.py`.
- IMU is 200 Hz with injected noise (σ = density × √rate) to match AQUA-SLAM's `NoiseGyro=1e-3` / `NoiseAcc=1e-2`.
- Headless note: the console simulator rejects any scene containing cameras and its ROS publishers only fire in real-time mode (`simulation_rate:=0`). See `workspace_sim/stonefish_notes.md` for the workaround.
- **PID gains live in one external file**: `workspace_sim/src/uv_sim/config/sim_pid.yaml` (9 channels: `chassis.pid.pos.{x,y,z,yaw}` + the yaw-cascade inner loop `chassis.pid.pos.yaw_rate` + `chassis.pid.vel.{x,y,z,yaw}`). The same data builds both the `/zit6/get_params` view and the live `_Pid` objects, so the service can never report gains that differ from the running controllers (before 2026-09-15 they were two hardcoded copies that had drifted: `pos.z.ki` 30 vs 80, `vel.x.kp` 300 vs 600, and the yaw-rate loop was invisible to the service). Override the file at runtime with `-p pid_config_file:=/path/to/tuned.yaml`; `/zit6/update_params` changes only the running process, never the file. Control structure: position mode = x/y/z single loops + **yaw two-level cascade**; velocity mode = four single loops.
- Build mode matters for Python edits: `colcon build` (no flag) **copies** the package into `install/`, while `colcon build --symlink-install` keeps `build/uv_sim/uv_sim -> src/uv_sim/uv_sim` live. If a source edit seems ignored, check the layout: `ls -l workspace_sim/install/uv_sim/lib/python3.12/site-packages/` (egg-link = live, a real `uv_sim/` directory = stale copy).

### 6-DOF State Feedback (migrated 2026-07-20, MCU side complete)

控制栈保持 4-DOF，状态反馈升级为 6-DOF 以携带完整姿态，用于纠正 position 节点的位置估计。

**统一数据顺序：** `[x, y, z, roll, pitch, yaw]`

| 话题 | 格式 | 发布者 |
|---|---|---|
| `/zit6/state/pos` | `[x, y, z, roll_rad, pitch_rad, yaw_rad]` | sim_bridge (SIL) / MCU (HIL·实机) |
| `/zit6/state/vel` | `[vx, vy, vz, vroll_rad, vpitch_rad, vyaw_rad_s]` | sim_bridge (SIL) / MCU (HIL·实机) |
| `/zit6/state/thr` | `[Fx, Fy, Fz, Mx, My, Mz]`（sim_bridge 的 Mx/My 恒为 0） | sim_bridge (SIL) / MCU (HIL·实机) |
| `PoseInfo` | 含 `robot_roll`, `robot_pitch` (deg) | basic_motion 30Hz |

**MCU 固件状态（`third_party/AUV_zit6_cmake`，`MicroRosPublisher.cpp`）：** 已完成 4→6 元素升级，`pos/vel/thr` 的 `data.size = capacity = 6`，`pos_world[3..5]` = roll/pitch/yaw。devlog #2 记录的「协议层截断 roll/pitch」问题已不存在。

**数据流：**
```
IMU → sim_bridge._odom_cb (全 RPY 提取)
  → /zit6/state/pos [6elem] → basic_motion._pos_cb → self.pose.rx/ry
  → PoseInfo (robot_roll/pitch) → position._pose_cb → _euler_to_rotation_matrix(roll,pitch,yaw)
  → 射线方向 v_world = R_robot @ v_body (完整姿态)
```

**注意：** `_vel_cb` 和 `_pos_cb` 都兼容旧 4 元素格式（通过 `len(msg.data) >= 6` 检查）；`hw_manager._thr_cb` 也用同一守卫，4 元素来源会让推力饱和监控静默失效。

### Coordinate System & Conventions

- **NED** (North-East-Down) throughout. Yaw 0 = North, **clockwise positive**.
- Angles stored in **degrees** internally, converted to radians for ROS messages.
- `Coordinate` class (`uv_control/coordinate.py`): 6-DOF NED with 4×4 homogeneous transform matrices.

- **NED**：`x = North(北)`，`y = East(东)`，`z = Down(下，深度)`。**x 轴指向正北**。Stonefish 世界系本身就是这个 NED 系。
  - `z=0` 为水面，z 越大越深；**负 z 表示在水面之上（空气里）**。
  - 证据 1（重力）：`SimulationManager` 中 `g=9.81` 且 `dynamicsWorld->setGravity(Vector3(0,0,g))` → 重力沿 **+Z**，即 Z 轴朝下。
  - 证据 2（轴定义）：`NED::__nRe__()` 给出的 ECEF→NED 矩阵三行依次为 North / East / Down；`GPS.cpp` 把世界系位置 `gpsPos` 的 `x,y` 直接当 `north,east` 传给 `Ned2Geodetic(x, y, 0, ...)` → 世界系 `x=北, y=东`。
  - 机体坐标系同理：`x=前(艏)`、`y=右舷`、`z=下`；`yaw=0`（机头朝 +x）时 x 才与正北重合。
  - `<environment><ned latitude longitude/>` 只是 NED 原点的大地坐标，**不是**坐标系切换。
  - 3D 软件按 z-up 导出的网格在 NED 下是**上下颠倒**的，必须给正的 z 偏移把它压到水下：`canyon.obj` 原始 `z∈[-5.73, 0.34]`，`world_transform z=+8.0` 后谷缘≈2.27 m 深、谷底≈8.34 m 深。
  - 新写 `.scn` 时切勿按 z-up 推深度，否则会把 AUV 放到水面之上（`canyon_vision_dvl.scn` 初版就踩过这个坑）。

### Control Modes (MotionCommand.type_mask)

The `type_mask` bitmask controls which axes are active. **CRITICAL — inverse logic: 0 = axis enabled, 1 = axis disabled.**

| Bit | Axis | Meaning when set (1) |
|---|---|---|
| bit0 | x | ignore x |
| bit1 | y | ignore y |
| bit2 | z | ignore z |
| bit3 | yaw | ignore yaw |

`0x0F` means **ALL axes disabled**, not enabled. `0x00` means all axes enabled.

9 control modes: `POS_WORLD`, `POS_BODY`, `VEL_WORLD`, `VEL_BODY`, `FORCE_BODY`, and step variants of each.

## Package Types

- **ament_python**: `uv_control`, `uv_hm`, `uv_nav`, `uv_perception`, `uv_task`, `uv_bringup`
- **ament_cmake**: `uv_msgs`, `zit6_interfaces`, `stonefish_ros2` (define custom `.msg`/`.srv` files)

## Key Files

| Path | Purpose |
|---|---|
| `workspace_auv/src/uv_task/config/tasks.json` | Competition task list (JSON array of `{name, params}`) |
| `workspace_auv/src/uv_task/uv_task/task_runner.py` | Task executor, BasicMotion action client |
| `workspace_auv/src/uv_control/uv_control/basic_motion.py` | Motion action server (cmd_type dispatch) |
| `workspace_sim/src/uv_sim/uv_sim/sim_bridge.py` | Simulation hardware bridge (SIL PID + thruster mixing / HIL passthrough); package `uv_sim` |
| `workspace_auv/src/uv_perception/uv_perception/vision.py` | Vision node: 4-channel YOLO, sim/real switch, dataset capture. Camera calibration is intentionally **empty** (new cameras, not yet calibrated) → undistortion skipped; sim images have no distortion anyway. `*_CAPTURE_RESOLUTION` are V4L2 **request** values; actual frame size is read back and a mismatch is warned. front 2560×960 / down 3840×1080 stitched (left+right eyes). Cameras deliver **MJPEG over USB** (`CAP_PROP_FOURCC='MJPG'`), so the "raw" BGR buffer is already JPEG-decoded. Dataset capture saves the **full stitched frame before split/undistort** (`front_*.jpg` / `down_*.jpg`, JPEG q95, 5 fps per camera), hence independent of calibration parameters and of model loading; default `dataset_dir` is `<repo>/datasets` (repo root found by walking up to `.git`, **not** by counting `__file__` levels — that lands in `build/`), overridable via `-p dataset_dir:=`, falls back to `~/.ros/auv_dataset` when no repo root is found |
| `workspace_auv/src/uv_perception/uv_perception/position.py` | Position node: multi-ray 3D localization (now uses full RPY for ray casting). `CAM_PARAMS` per-eye geometry: front 1280×960 @ hfov 57.19°, down **1920×1080** @ hfov 87.19°; intrinsics are derived from HFOV, not from any calibration file |
| `visualization/auv_visualizer.py` | PySide6 GUI: 2D map, objects/rays, node list, task status, force remote control |
| `workspace_auv/docs/basic_motion_action.md` | BasicMotion action interface reference |
| `workspace_auv/docs/perception.md` | Perception system architecture + data flow |
| `workspace_auv/docs/debug_guide.md` | Debugging guide (ros2 CLI, action test, simulation) |

## Conventions

- **Entry points** defined in `console_scripts` of `setup.py`; `launch/` dir at package root
- **Imports order**: stdlib → rclpy → ROS msgs → local packages (blank-line separated)
- **Architecture**: thin ROS nodes that import core logic from sibling modules. Each `package/__init__.py` is empty; actual code is in `package/module.py`.
- `.pt` YOLO weights under `src/datas/` are **gitignored** — models must be provided separately
- `stonefish_ros2` depends on external https://github.com/patrykcieslak/stonefish library
- **Tests**: only `uv_sim` has them so far (25 cases: `workspace_sim/src/uv_sim/test/` — scene sensor contract, odometry frame conventions, PID-config single-source, entry-point integrity). Run with `python3 -m pytest workspace_sim/src/uv_sim/test/ -q` after sourcing both workspaces; keep it green.
