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

ros2 launch uv_sim_bringup sim_bringup.py   # full simulation stack
```

**入口分层（T4 起）**：`workspace_auv` 只有一个实机入口 `uv_bringup real_bringup.py`，
**不含任何仿真依赖**；仿真/HIL 入口在 `workspace_sim` 的新包 `uv_sim_bringup`。
原则是「**节点定义归各包，系统预设只做编排**」：预设文件里没有 `Node(...)`，只有 include +
条件，所以增删节点不用改 bringup。入口清单与新旧映射见 `docs/interfaces/launch_entries.md`。

Note: `workspace_sim` uses colcon overlay — source `workspace_auv/install/setup.bash` first, then `workspace_sim/install/setup.bash`.

## Architecture

This is a **ROS 2 Jazzy** project for the **YouLong AUV** (autonomous underwater vehicle), designed for the SAUVC competition. It uses colcon with mixed ament_python (application logic) and ament_cmake (custom messages, Stonefish wrapper) packages.

### 6-Layer Stack (bottom-up)

| Layer | Package | Role |
|---|---|---|
| 0 | `stonefish_ros2` (C++) | Stonefish 1.6 marine physics simulator. GPU mode runs `stonefish_simulator`, headless runs `stonefish_simulator_nogpu`. Lives in `workspace_sim/` |
| 1 | `uv_hm` + `uv_sim` | Cascaded PID + thruster mixing → 6 thrusters. Since T5 `uv_sim` (**`workspace_sim/`**) is **seven** nodes: sensor relays (`odom_sim`/`imu_sim`/`dvl_sim`/`pressure_sim` → `/sim/sensor/*`), `nav_sim` (state/injection), `mcu_sim` (the simulated MCU), `camera_relay`; `hw_manager` (package `uv_hm`, `workspace_auv/`) keeps the STM32 MCU armed and monitors `/zit6/state/*` |
| 2 | `uv_control` | Motion API: SET/WMOVE/BMOVE/TRAVEL. Only entry point is `basic_motion` |
| 3 | `uv_perception` | 4 通道双目 YOLO 检测 + 多帧单目射线交会 3D 定位。sim/real 模式切换，发布 PoseInfo 位姿源 |
| 4 | `uv_nav` | A* pathfinding + obstacle avoidance waypoint following |
| 5 | `uv_task` | JSON-based competition task runner |

### Action Server — `basic_motion` (`/basic_motion`)

`basic_motion` exposes a `uv_msgs/action/BasicMotion` Action Server. This is the primary entry point for all motion control — nodes send action goals and block/wait for completion.

#### Action 定义

```
# Goal
uint8 cmd_type      # WMOVE=1 BMOVE=2 SET=3 WTRAVEL=4 BTRAVEL=5 START=6
                    # WVELOCITY=7 BVELOCITY=8 WFORCE=9 BFORCE=10 STAY=11 (2026-10-12 扩充)
string axes          # 生效轴 "x"/"y"/"z"/"rz" 任意组合，空=全部
float32[] target     # [x, y, z, yaw] — 含义取决于 cmd_type；速度命令 yaw=度/秒
float32 timeout      # 超时秒数，≤0 默认 60s
float32 duration      # 秒，仅速度/推力类：保持这么久后自动收尾为原地保持
---
# Result
bool success
string message
---
# Feedback
float32 distance_remaining   # 3D 欧氏距离（位置类；速度/推力类恒 0）
float32[] velocity_current   # 机体实测 [vx,vy,vz,vyaw_rate]，m/s 与 度/秒
float32[] velocity_target    # 机体目标速度；非速度命令为全 0
uint8 control_level          # 回读 MCU 层级 0=NONE 1=POS 2=VEL 3=FORCE
```

#### 坐标系约定

- **odom 系**：`x`/`y`/`z`/`yaw` **四轴全部**以 START 时的 AUV 位姿为原点。
  `start()` 的 `_origin` 取当前 map 位姿的**完整 4-DOF**（`x/y/z/rz`，rx/ry 恒为 0）。
  所以 `SET(x,y,3,0)` 是「**比 START 时再深 3 m**」，**不是**「下潜到 3 m 绝对深度」。
  2026-10-12 SIL console 实测确认：START 在 map z=6.89 处 → `origin_z=6.90` → `SET(z=1)`
  到达 map z=**7.79**（≈6.89+1，继续下潜），而不是上浮到 1 m。
  ⚠️ **此改动翻转了 z 的语义**：2026-09-16 定的是旧行为（`_origin.z` 写死 `0.0` ⇒ odom z =
  绝对深度、水面 0）。改成 4-DOF 原点的原因是让"原点"名副其实 —— 旧写法下 `start()` 把
  `self.pose.z` 清零、下一个 `_pos_cb`（33 ms 后）又把它变成绝对深度，那一帧是自相矛盾的。
  **影响**：z 命令的含义随 START 深度漂移；**重复发送 START 会移动深度基准**
  （下潜到 3 m 后重发 START，之后 `setz: 3` 就到 6 m 绝对深度）。凡是有绝对深度意图的
  竞赛任务都要相应调整；`task_runner` 的 `setz`/`wmovez`/`bmovez` 同理
- **yaw 单位**：度（°），NED 系顺时针为正
- **NED 轴**：x=北(N), y=东(E), z=下(D, 正数更深)

#### 11 种命令类型

| 命令 | 值 | target 含义 | 运动模式 | 行为 |
|---|---|---|---|---|
| **START** | 6 | 忽略 | — | 初始化 odom 原点。必须在所有运动命令之前发送一次 |
| **SET** | 3 | 绝对坐标 `[x, y, z, yaw]` | `set_world` + `wait_reached` | 直接发往目标，等到达。简单直接 |
| **WMOVE** | 1 | 世界系偏移 `[dx, dy, dz, dyaw]` | `step_move_world` | 拆分长距离为动态步进段。横向误差大时自动减小步长 |
| **BMOVE** | 2 | 机体系偏移 `[dx, dy, dz, dyaw]` | `body_to_world` + `step_move_world` | 先将 body 偏移旋转到世界系，再走 WMOVE |
| **WTRAVEL** | 4 | 世界系偏移 `[dx, dy, dz]` | 转向 + 直线 step_move | 先转向目标方向 (atan2)，再沿 body-X 前进 |
| **BTRAVEL** | 5 | 机体系偏移 `[dx, dy, dz]` | `body_to_world` + WTRAVEL | 先转 body→world，再走 WTRAVEL |
| **WVELOCITY** | 7 | 世界系速度 `[vx,vy,vz,vyaw(度/秒)]` | 速度闭环（MCU） | 保持 `duration` 秒后**自动收尾为原地保持** |
| **BVELOCITY** | 8 | 机体系速度 | 速度闭环（MCU） | 同上；`vx` 恒指艇头方向 |
| **WFORCE** | 9 | 世界系推力 `[Fx,Fy,Fz,Mz]` | 直接推力（MCU） | 同上 |
| **BFORCE** | 10 | 机体系推力 | 直接推力（MCU） | 同上 |
| **STAY** | 11 | 忽略 | 位置（目标=当前位姿） | 锁住当前位置，**立即返回**（纯 latch，无 duration） |

**速度/推力的两条硬规则**（2026-10-12）：
- **世界系/机体系由 `control_key` 的 frame 位表达，转换在 MCU 侧做**（固件与仿真都按
  `R_ZYX^T`）。`basic_motion` 只填位，自己不算。位置类是历史例外（`BMOVE`/`BTRAVEL` 在
  `basic_motion` 内转世界后统一走 `0x00`）。
- **必须收尾**：MCU 不会忘记最后一条速度/推力目标。到时、被 cancel、被抢占，三条路径
  都会先发一条「目标=当前位姿」再返回；节点退出（Ctrl+C/SIGTERM）发推力归零。

#### START 命令

```python
goal = BasicMotion.Goal()
goal.cmd_type = BasicMotion.Goal.START
# 其他字段忽略
```

- 记录当前 map 位姿为 odom 原点，之后所有坐标相对此原点
- odom 0° = AUV 初始朝向（yaw 也做偏移）
- 例：AUV 朝东 (map 90°) → START → odom 0° = 东，`SET(5,0,0,0)` 向东走 5m
- **重复发送 START 不是无操作**：`start()` 每次都重写原点（先打
  `odom origin already set, updating to current map pose`，再把当前 map 位姿记为原点）。
  2026-09-16 实测确认（见 `devlog/2026-09-16_status.md` 的 L2 一节）；此前文档写「无操作」是错的

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
- 步进参数：STEP_X=0.6m, STEP_Y=0.4m（椭圆模型），STEP_PERIOD=0.2s（2026-10-12 与 `basic_motion.py` 常量对拍更正；此前写 0.2/0.3 是错的）
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
result.success    # True=成功, False=超时/被取消/被抢占/被拒绝
result.message    # 空="成功"; "duration elapsed"=速度/推力到时收尾;
                  # "cancelled"; "superseded"=被新 goal 抢占; "motion timeout"

# Feedback (0.5Hz, 仅命令执行中有值)
feedback.distance_remaining   # 位置类：3D 欧氏距离（不含 yaw）；速度/推力类恒 0
feedback.velocity_current     # 机体实测 [vx,vy,vz,vyaw_rate]，m/s 与 度/秒
feedback.velocity_target      # 机体目标速度；非速度命令为全 0
feedback.control_level        # 回读 MCU 层级 0=NONE 1=POS 2=VEL 3=FORCE
```

**打断语义**（2026-10-12）：任何命令被 cancel 或被更新的 goal **抢占**时，都先发一条
「目标=当前位姿」的位置 setpoint 再返回 —— 只"停止下发"会让 MCU 继续执行上一条
（速度/推力下尤其危险）。抢占**不是排队**：新 goal 立即被接受，旧命令在 ≤100 ms 内让位。

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
4. **`axes` 会真正生效**（2026-10-10 实测更正：此前写"仅用于日志"是错的）。
   对 **SET/WMOVE/WTRAVEL/WVELOCITY/BVELOCITY/WFORCE/BFORCE**，只有列在 `axes` 里的轴更新为
   目标里的新值，其余轴**沿用上一目标**（速度/推力类沿用**上一条速度/推力目标**，与位置目标
   不是同一份状态）；对 **BMOVE/BTRAVEL/START/STAY** 完全不参与。
   极性与 `ZitSetpoint.type_mask` **相反**（那里 1=忽略）。
   实现是 `fuse_target_axes()`（2026-10-12 抽出，此前在派发里复制三份）
5. **MultiThreadedExecutor**：basic_motion 内部使用多线程执行器，action 回调在独立线程运行，不阻塞主循环
6. **并发命令会被抢占，不会排队**（2026-10-12）：ActionServer 的 execute_callback 由
   `executor.create_task` 驱动，**并发 goal 会真的同时执行**（实测两条 SET 同写
   `/zit6/cmd/setpoint`）。现在用 `_exec_lock` 串行化 + 单调递增代号抢占：新 goal 赢，
   旧命令收尾让位。
7. **必须收尾**：MCU 不会忘记最后一条速度/推力目标。到时/被 cancel/被抢占都会先发原地保持；
   节点退出（Ctrl+C 或 SIGTERM）发推力归零（`control_key=0x12` 全 0）。

### task_runner (`uv_task`)

`workspace_auv/src/uv_task/uv_task/task_runner.py` is the competition task executor. It reads a JSON task list (`config/tasks.json`) and executes tasks sequentially using BasicMotion ActionClient.

**Key behaviors:**
- **Auto-start**: on launch, if `tasks.json` has entries, a daemon thread immediately starts executing them
  (no service call needed). **`debug_mode:=true` skips auto-start** so `/task/exec` owns the vehicle
- **Task-list validation** (2026-10-12): `load_tasks` both parses and validates — JSON syntax, root shape,
  every `name` present in `task_map`, `params` is an object. Failure ⇒ returns `[]` **and refuses to start**,
  logging each problem. Never raises: it is called from a service callback, where an exception would mean
  the client gets no response at all. Rationale: `run_task_list` deliberately does **not** abort on a failed
  task, so a typo'd name used to be one `warn` and then the AUV silently skipped a step with every later
  command running from a stale `_cmd_*` estimate
- **Shutdown** (2026-10-12): `main()` mirrors `basic_motion` — uninstalls rclpy's signal handlers, installs
  its own SIGINT/SIGTERM handler, and `request_shutdown()` in `finally` cancels the active goal and waits
  (≤3 s) for it to end. It deliberately does **not** publish a thrust-zero: `/zit6/cmd/setpoint` has exactly
  one writer (`basic_motion`), which zeroes thrust on its own exit
- **Task map**: JSON `name` field maps to a method via the `task_map` dict in `task_runner.py`. Beyond the SET/WMOVE/BMOVE/WTRAVEL/BTRAVEL axis variants (e.g. `setxyzrz`, `bmovexy`, `wtravelx`), plus `start`, `navigate`, `wait`, and since 2026-10-12 the velocity/force latch tasks `bvelocity`/`wvelocity`/`bforce`/`wforce`/`stay` (params carry `duration`; they deliberately do **not** update `_cmd_x/_cmd_y/_cmd_z/_cmd_yaw`, so do not mix them with position tasks that rely on that tracking), it exposes competition-specific actions: `follow_line`, `arrow_surface`, `drop_beacon`, `take_water_sample`, `get_sample`, `release_sampler`. **`task_runner.py`'s `task_map` is the authoritative list**; `docs/debug_guide.md` lags behind it.
- **ROS parameters**: `debug_mode`, `down_image_width`, `down_image_height`, `align_ema_alpha` (see `docs/node_parameters.md`)
- **Stop service** (`/task/stop`): sets `self.stopped = True` **and cancels the running action goal**
  (2026-10-12 修 B23：此前调 `ActionClient.async_cancel_goal` —— rclpy 的 `ActionClient`
  **没有**取消方法，取消只能在 goal handle 上 `cancel_goal_async()`；`AttributeError` 被
  服务回调吞掉，所以它一直是「停列表但不停船」)
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

`uv_msgs/msg/AuvState` is intended for simulation only, and **nothing publishes `/auv/state`** any more (the old `stonefish_ros2` publisher is gone; `uv_sim`'s `nav_sim`/`mcu_sim` publish `/zit6/state/*` and `nav_sim` publishes `/zit6/sim/nav`). Real AUV hardware never publishes it. Nodes running on the real AUV must not depend on it.

For pose feedback, both perception and navigation use `/basic_motion/pose_info` (`uv_msgs/PoseInfo`, 30Hz) published by `basic_motion`. This works in both simulation and real-hardware modes — `navigator.py` was migrated to it (2026-09-13), and it refuses to plan until a pose has arrived.

The `AuvState` message itself is now unused by any node; only `uv_msgs` still defines it.

### uv_sim — the simulation-side nodes (T5: seven nodes, one responsibility each)

`uv_sim` lives at `workspace_sim/src/uv_sim/` (package `uv_sim`, **not** `uv_hm`). Until T5 it was one
919-line node called `sim_bridge`; it is now seven entry points, all launched by
`uv_sim/launch/bridge_launch.py`:

| Node | Role |
|---|---|
| `odom_sim` / `imu_sim` / `pressure_sim` | pure relays `/auv/*` → `/sim/sensor/*` (message and stamp forwarded unchanged) |
| `dvl_sim` | `/auv/dvl` → `/sim/sensor/dvl` (`uv_msgs/Dvl`): AQUA-SLAM's adapter; passes the DVL-frame velocity through and **synthesizes the four along-beam speeds** (the engine never fills `beams`) |
| `nav_sim` | assembles `/sim/sensor/*` into `/zit6/state/{pos,vel}` (SIL) **or** `/zit6/sim/nav` (HIL). Two mutually exclusive params `publish_state` / `publish_sim_nav`; both true ⇒ startup `RuntimeError` (invariant I2) |
| `mcu_sim` | the simulated MCU: `ZitSetpoint` in → PID + mixing → `/auv/thrusters_cmd`, plus `/zit6/state/{status,thr,zithbt}` and `/zit6/{get,update}_params`. With `hil_mode:=true` it is a **thrust relay only** (no PID, no state publishing, no services -- the real MCU owns those) |
| `camera_relay` | `/sim/*_cam/*` → `/auv/*_cam/*` + stitched; one stitch per frame pair on a 100 Hz timer with a dirty flag (pre-T5 both callbacks stitched, doubling the topic rate). `log_rates:=true` prints writer-side rates |

**Where the maths lives**: `uv_sim/mcu_control.py` (`McuController`: setpoint state machine, cascaded
PID, force mode, ARM/INS, parameter read/write -- ROS-free), `uv_sim/mixer.py` (4-DOF → 6 thrusters;
yaw ×1.0, heave ×0.8), `uv_sim/pid_config.py` (the gain file), `uv_sim/odometry.py` (the one
`/auv/odometry` decoder shared by `nav_sim` and `mcu_sim`), `uv_sim/runner.py` (the `main()` body).
`test/data/thrust_golden.json` + `test_mcu_control.py` exist to prove the T5 refactor did not change the
control maths (recorded from the pre-split monolith, replayed bit-exactly).

**SIL vs HIL is one launch argument.** `bridge_launch.py` declares `hil_mode` (+ `pid_config_file`) and
derives `nav_sim`'s role from it; `hil_mode:=true` starts **five** nodes (`odom_sim`, `imu_sim`,
`nav_sim`, `mcu_sim`, `camera_relay`) and skips `dvl_sim`/`pressure_sim`, because a real MCU subscribes
`/auv/*` itself. Do not add a second mode knob: two switches that must agree is exactly how I2 breaks.

**`/auv/dvl` carries the DVL-frame velocity** (velocity at the DVL acoustic centre), while
`/zit6/state/vel` carries the **body-origin** velocity. They differ by `omega x r_dvl`
(`r_dvl = (-0.375, 0, 0.1)`), up to ~0.13 m/s while yawing. That was bug B19 (two writers on one topic
with different meanings); T5 fixed it by **separation** -- `/zit6/state/vel` has exactly one writer
(`nav_sim`, origin twist) and the DVL value has its own topic. Do not "helpfully" add the lever arm to
the state topic, and do not read `/zit6/state/*` back into `mcu_sim`.

Historical note: the yaw-polarity bug documented in earlier revisions (`cmd_rz = -float(self.pid_yaw_rate.step(...))`) has been **fixed** — the negation is gone (now `cmd_rz = float(...)`). Do not re-apply that fix.

**Sim sensor geometry (2026-09-15, aligned with the real vehicle).** The XUNYUN DVL in `workspace_sim/src/stonefish_ros2/Data/xunyun_fixed.scn` is mounted flush with the body (`origin rpy="0 0 0" xyz="-0.375 0 0.1"`) with `<specs beam_angle="30.0" beam_positive_z="true"/>`, matching the real DVL (yaw=0 mount, beams offset 45°, 30° off vertical, z≈+0.1 m). Consequences worth remembering:
- `/auv/dvl` therefore carries **body-frame** velocity. The pre-2026-09-15 scene used `rpy=(3.1416 0 -0.785)` with the default `beam_positive_z=false`, which made Stonefish publish velocity in a frame rotated 45° from the body with y/z flipped; the old monolith forwarded that straight into `/zit6/state/vel`, so `basic_motion`'s along-track speed became `V·cos(2α+45°)` (≈0 at α=22.5°). Regression pinned by `workspace_sim/src/uv_sim/test/test_scene_sensors.py`; end-to-end check in `workspace_sim/tools/check_dvl_frame.py`.
- IMU is 200 Hz with injected noise (σ = density × √rate) to match AQUA-SLAM's `NoiseGyro=1e-3` / `NoiseAcc=1e-2`.
- Headless note: the console simulator rejects any scene containing cameras, and `simulation_rate` must stay **positive** (0 stops the simulation loop and publishes nothing). Since T4 this is a first-class parameter path — `ros2 launch uv_sim_bringup sim_bringup.py headless:=true` — see `workspace_sim/stonefish_notes.md` and `docs/interfaces/launch_entries.md`.
- **PID gains live in one external file**: `workspace_sim/src/uv_sim/config/sim_pid.yaml` (9 channels: `chassis.pid.pos.{x,y,z,yaw}` + the yaw-cascade inner loop `chassis.pid.pos.yaw_rate` + `chassis.pid.vel.{x,y,z,yaw}`). The same data builds both the `/zit6/get_params` view and the live `Pid` objects (`uv_sim/pid_config.build_pids` → `McuController.pids`), so the service can never report gains that differ from the running controllers (before 2026-09-15 they were two hardcoded copies that had drifted: `pos.z.ki` 30 vs 80, `vel.x.kp` 300 vs 600, and the yaw-rate loop was invisible to the service). Override the file at runtime with `-p pid_config_file:=/path/to/tuned.yaml`; `/zit6/update_params` changes only the running process, never the file. Control structure: position mode = x/y/z single loops + **yaw two-level cascade**; velocity mode = four single loops.
- Build mode matters for Python edits: `colcon build` (no flag) **copies** the package into `install/`, while `colcon build --symlink-install` keeps `build/uv_sim/uv_sim -> src/uv_sim/uv_sim` live. If a source edit seems ignored, check the layout: `ls -l workspace_sim/install/uv_sim/lib/python3.12/site-packages/` (egg-link = live, a real `uv_sim/` directory = stale copy).

### 6-DOF State Feedback (migrated 2026-07-20, MCU side complete)

控制栈保持 4-DOF，状态反馈升级为 6-DOF 以携带完整姿态，用于纠正 position 节点的位置估计。

**统一数据顺序：** `[x, y, z, roll, pitch, yaw]`

| 话题 | 格式 | 发布者 |
|---|---|---|
| `/zit6/state/pos` | `[x, y, z, roll_rad, pitch_rad, yaw_rad]` | `nav_sim` (SIL) / MCU (HIL·实机) |
| `/zit6/state/vel` | `[vx, vy, vz, vroll_rad, vpitch_rad, vyaw_rad_s]` | `nav_sim` (SIL) / MCU (HIL·实机) |
| `/zit6/state/thr` | `[Fx, Fy, Fz, Mx, My, Mz]`（`mcu_sim` 的 Mx/My 恒为 0） | `mcu_sim` (SIL) / MCU (HIL·实机) |
| `PoseInfo` | 含 `robot_roll`, `robot_pitch` (deg) | basic_motion 30Hz |

**MCU 固件状态（`third_party/AUV_zit6_cmake`，`MicroRosPublisher.cpp`）：** 已完成 4→6 元素升级，`pos/vel/thr` 的 `data.size = capacity = 6`，`pos_world[3..5]` = roll/pitch/yaw。devlog #2 记录的「协议层截断 roll/pitch」问题已不存在。

**数据流：**
```
/auv/odometry + /auv/imu → odom_sim/imu_sim → /sim/sensor/*
  → nav_sim 解码（全 RPY + 机体 twist，唯一编码点 uv_sim/odometry.py）
  → /zit6/state/pos [6elem] → basic_motion._pos_cb → self.pose.rx/ry
  → PoseInfo (robot_roll/pitch) → position._pose_cb → _euler_to_rotation_matrix(roll,pitch,yaw)
  → 射线方向 v_world = R_robot @ v_body (完整姿态)
```

**注意：** `_vel_cb` 和 `_pos_cb` 都兼容旧 4 元素格式（通过 `len(msg.data) >= 6` 检查）；`hw_manager._thr_cb` 也用同一守卫，4 元素来源会让推力饱和监控静默失效。

### Coordinate System & Conventions

**文档体系（T3a 起）**：`docs/README.md` 是索引与维护规则。事实源各自唯一：
话题/类型 → `docs/interfaces/topic_contract.md`；参数 → `docs/node_parameters.md`；
外参数值 → `docs/frames/extrinsics_reference.md`；坐标系约定 → `docs/frames/frame_convention.md`。
改接口/参数必须同一次改动里改对应文档，由源码级校验强制（不需要跑栈）：

```bash
python3 scripts/check_docs_contract.py --check     # 参数/launch 参数/节点/话题/依赖/README + 两类 launch 一致性（共 8 类）
```

**权威文档（T2 起）**：`docs/frames/frame_convention.md` 是约定的唯一权威，
`docs/frames/extrinsics_reference.md` 是数值的唯一权威（含仿真表 / 实机表 / DVL 波束几何 /
AQUA-SLAM 对照）。下面只是速查，冲突时以那两篇为准。可执行校验：

```bash
python3 workspace_sim/tools/check_dvl_geometry.py          # 波束几何 + α 敏感度（无 ROS）
python3 workspace_sim/tools/derive_extrinsics.py --check   # 外参表 ↔ 场景对拍
python3 workspace_sim/tools/dump_tf_tree.py --self-test    # TF 报告逻辑（无 ROS）
```

- **NED** (North-East-Down) throughout. Yaw 0 = North, **clockwise positive**.
- Angles stored in **degrees** internally, converted to radians for ROS messages.
  **例外**：`.scn` 的 `<origin rpy>` 是弧度，但 `<specs beam_angle>` 是**度**。
- `Coordinate` class (`uv_control/coordinate.py`): 6-DOF NED with 4×4 homogeneous transform matrices.
- `<origin rpy>` 复合顺序是 `R = Rz(yaw)·Ry(pitch)·Rx(roll)`；代码里只用
  `uv_sim.geometry.euler_rpy_to_matrix()`，别手写 `Rx·Ry·Rz`。
- DVL 角度三个记号别混：**θ**=离竖直（场景 `beam_angle`=30°）、**α**=离水平
  （AQUA-SLAM `mE`，=60°）、**ψ**=艇航向。α 填错是静默的水平尺度错误
  （出厂默认 67.5° → +30.7%）。
- TF：**仿真当前完全没有 TF**（`/tf` 为空）。广播是默认关闭的，需要场景在 `<robot>` 里写
  `<ros_base_link_transform publish="true"/>`（`ROS2SimulationManager.cpp` 那行外面套着 `if` 守卫，
  别把它当"TF 存在"的证据）；15 个场景都没写。外参读
  `workspace_sim/src/uv_sim/config/extrinsics/xunyun.yaml`（由场景生成，勿手改）。
  在线核对：`python3 workspace_sim/tools/dump_tf_tree.py`（console 模式即可，不必 GPU；
  当前健康状态就是 exit 1 + "no TF received"）。
- **NED 细则**（为什么 z 朝下、z-up 网格的坑）：见本节末尾。

#### NED 细则与证据

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

- **ament_python**: `uv_control`, `uv_hm`, `uv_nav`, `uv_perception`, `uv_task`, `uv_bringup`, `uv_sim`, `uv_sim_bringup`
- **ament_cmake**: `uv_msgs`, `zit6_interfaces`, `stonefish_ros2` (define custom `.msg`/`.srv` files)

## Key Files

| Path | Purpose |
|---|---|
| `workspace_auv/src/uv_task/config/tasks.json` | Competition task list (JSON array of `{name, params}`) |
| `workspace_auv/src/uv_task/uv_task/task_runner.py` | Task executor, BasicMotion action client |
| `workspace_auv/src/uv_control/uv_control/basic_motion.py` | Motion action server (cmd_type dispatch) |
| `workspace_sim/src/uv_sim/uv_sim/` | The seven simulation nodes (`odom_sim`/`imu_sim`/`dvl_sim`/`pressure_sim`/`nav_sim`/`mcu_sim`/`camera_relay`) plus the ROS-free core they share (`mcu_control.py`, `mixer.py`, `pid_config.py`, `pid.py`, `odometry.py`, `mathutil.py`, `runner.py`) and `geometry.py`; package `uv_sim` |
| `workspace_auv/src/uv_perception/uv_perception/vision.py` | Vision node: 4-channel YOLO, sim/real switch, dataset capture. Camera calibration is intentionally **empty** (new cameras, not yet calibrated) → undistortion skipped; sim images have no distortion anyway. `*_CAPTURE_RESOLUTION` are V4L2 **request** values; actual frame size is read back and a mismatch is warned. front 2560×960 / down 3840×1080 stitched (left+right eyes). Cameras deliver **MJPEG over USB** (`CAP_PROP_FOURCC='MJPG'`), so the "raw" BGR buffer is already JPEG-decoded. Dataset capture saves the **full stitched frame before split/undistort** (`front_*.jpg` / `down_*.jpg`, JPEG q95, 5 fps per camera), hence independent of calibration parameters and of model loading; default `dataset_dir` is `<repo>/datasets` (repo root found by walking up to `.git`, **not** by counting `__file__` levels — that lands in `build/`), overridable via `-p dataset_dir:=`, falls back to `~/.ros/auv_dataset` when no repo root is found |
| `workspace_auv/src/uv_perception/uv_perception/position.py` | Position node: multi-ray 3D localization (now uses full RPY for ray casting). `CAM_PARAMS` per-eye geometry: front 1280×960 @ hfov 57.19°, down **1920×1080** @ hfov 87.19°; intrinsics are derived from HFOV, not from any calibration file |
| `visualization/auv_visualizer.py` | PySide6 GUI: 2D map, objects/rays, node list, task status, force remote control |
| `workspace_auv/docs/basic_motion_action.md` | BasicMotion action interface reference |
| `workspace_auv/docs/perception.md` | Perception system architecture + data flow |
| `workspace_auv/docs/debug_guide.md` | Debugging guide (ros2 CLI, action test, simulation) |
| `docs/frames/frame_convention.md` | **坐标系与约定的唯一权威**（NED、单位分界、rpy 顺序、TF 树现状、DVL θ/α/ψ 术语、相机光学系） |
| `docs/frames/extrinsics_reference.md` | **外参数值的唯一权威**（仿真表 + 实机表 + `T_b_c/T_gyro_c/T_dvl_c` 推导 + α 敏感度 + T6 必改清单）；含机器可读 fenced YAML 块 |
| `workspace_sim/src/uv_sim/uv_sim/geometry.py` | 几何唯一真相点（rpy→R、刚体求逆、DVL 波束 `E` 与反演、`.scn` 解析）；纯 numpy 无 ROS |
| `workspace_sim/tools/derive_extrinsics.py` | 从场景推导外参；`--emit/--out/--check`（`--check` 与文档对拍，漂移即非 0 退出） |
| `workspace_sim/tools/check_dvl_geometry.py` | DVL 波束几何数值校验 + α 敏感度表（无 ROS 可跑） |
| `workspace_sim/tools/dump_tf_tree.py` | 在线 TF 树核对 + `--self-test` |
| `docs/README.md` | **文档索引 + 维护规则**（标记含义、禁止复制事实、跨文档待补清单） |
| `docs/interfaces/topic_contract.md` | **话题/消息类型唯一事实源**（SIL/HIL/服务动作/栈外 GUI/缺口登记，每行带代码证据） |
| `docs/interfaces/launch_entries.md` | **启动入口清单 + 新旧映射 + 编排规则**（T4 起；改 launch 先看这里） |
| `workspace_auv/src/uv_bringup/uv_bringup/launch_common.py` | 编排助手 `arg()` / `include(when=…)`：预设文件只 include，不声明节点 |
| `workspace_sim/src/uv_sim_bringup/uv_sim_bringup/stonefish_common.py` | Stonefish 侧助手：`Data/` 目录解析、场景名检查、引擎 include（GPU/console 分支） |
| `docs/architecture/{system_architecture,sim_real_boundary,node_dependency}.md` | 分层与节点、SIL/HIL/REAL 边界、包依赖与启动顺序（含依赖缺口登记） |
| `scripts/check_docs_contract.py` | **源码级文档契约校验**（参数/launch 参数/节点/话题/依赖/README + launch 目标与文档入口一致性，共 8 类；无需跑栈）；`--update` 生成库存 |

## Conventions

- **Entry points** defined in `console_scripts` of `setup.py`; `launch/` dir at package root
- **Imports order**: stdlib → rclpy → ROS msgs → local packages (blank-line separated)
- **Architecture**: thin ROS nodes that import core logic from sibling modules. Each `package/__init__.py` is empty; actual code is in `package/module.py`.
- `.pt` YOLO weights under `src/datas/` are **gitignored** — models must be provided separately
- `stonefish_ros2` depends on external https://github.com/patrykcieslak/stonefish library
- **Tests**: three suites. `uv_sim` (**171 cases**: `workspace_sim/src/uv_sim/test/` — scene sensor
  contract, odometry frame conventions (now via `nav_sim`/`mcu_sim`), PID-config single-source + fallback
  structure, entry-point integrity (derived from `setup.py`), frame/beam geometry, scene↔YAML↔doc
  contract, camera-offset (N1), TF (B18), documentation-contract (T3a), launch-contract / layering (T4),
  and since T5: the thrust golden regression (`test_mcu_control.py`), the node set + topic-contract
  immutability + I2 ownership (`test_node_split.py`), `nav_sim` role switch (`test_nav_sim.py`),
  DVL beams/validity (`test_dvl_sim.py`), stitching (`test_camera_relay.py`), the line laser
  (`test_laser_sim.py`) and the PID-config path walk (`test_pid_config.py`)), and since 2026-10-12
  `uv_control` (**37 cases**: `workspace_auv/src/uv_control/test/` — `cmd_type` names derived from
  `BasicMotion.action`, the `axes` target-fusion rule, the step constants vs their doc table, command
  serialisation + preemption + interrupt-to-position-hold, the velocity/force/STAY/duration/feedback
  behaviour added 2026-10-12, and the **non-zero-origin** coordinate checks that the zero-origin tests
  could not make), and since 2026-10-12 `uv_task` (**13 cases**: `workspace_auv/src/uv_task/test/` —
  task-list validation (typo'd names, malformed JSON/root/params) and the cancel-API shape).
  The `uv_control` suite exists because that package had **no tests at all**, which is how a
  `STEP_Y`/`STEP_PERIOD` doc drift and a triplicated `cmd_type` name map went unnoticed.
  Run after sourcing both workspaces; keep them green:
  ```bash
  python3 -m pytest workspace_sim/src/uv_sim/test/ -q
  python3 -m pytest workspace_auv/src/uv_control/test/ -q
  python3 -m pytest workspace_auv/src/uv_task/test/ -q
  ```
