# 服务与动作契约（T3b）

> **本文件只装"字段语义"**：每个请求/响应字段是什么意思、什么条件下失败、边界值如何解释。
> **名称与类型**的唯一事实源是 `docs/interfaces/topic_contract.md` §7（那份由
> `scripts/check_docs_contract.py` 校验）；命令级的完整用法（6 种运动命令、示例、注意事项）
> 在 `workspace_auv/docs/basic_motion_action.md`，本文件不复制。
>
> 校验方式：`.srv`/`.action` 的**字段表**由源码定义（`zit6_interfaces`、`uv_msgs`），
> 语义由本文件描述并与实现对齐（每节给出代码锚点）；`ros2 interface show <type>` 可现场核对。
> 约定的单位分界见 `docs/frames/frame_convention.md`（ROS 消息用弧度，YAML/参数/PoseInfo 用度）。

## 1. 总览

| 名称 | 类型 | 服务端 | 模式 | 一句话 |
|---|---|---|---|---|
| `/basic_motion` | `uv_msgs/action/BasicMotion` | `basic_motion` | 两者 | 唯一运动入口，阻塞到到位/超时 |
| `/zit6/get_params` | `zit6_interfaces/srv/GetParams` | `mcu_sim`（SIL）/ MCU ⏸ | SIL | 读 PID 增益（9 通道） |
| `/zit6/update_params` | `zit6_interfaces/srv/UpdateParams` | `mcu_sim`（SIL）/ MCU ⏸ | SIL | 只改运行时增益，不落盘 |
| `/nav/navigate_to` | `uv_msgs/srv/RunTask` | `navigator` | 两者 | 让导航器去一组目标 |
| `/task/run` | `uv_msgs/srv/RunTask` | `task_runner` | 两者 | 启动/停止任务列表 |
| `/task/stop` | `std_srvs/srv/Trigger` | `task_runner` | 两者 | 中断当前任务 |
| `/task/exec` | `uv_msgs/srv/ExecTask` | `task_runner` | 两者 | 执行单条任务（**仅 debug 模式**） |

同一 `RunTask` 类型被两个服务端复用（`/nav/navigate_to`、`/task/run`），字段语义**不同**，
所以下面分开写，不要互相套用。

## 2. 动作 `/basic_motion`（`uv_msgs/action/BasicMotion`）

**Goal**

| 字段 | 类型 | 语义 |
|---|---|---|
| `cmd_type` | `uint8` | 命令类型常量：位置类 `WMOVE=1`、`BMOVE=2`、`SET=3`、`WTRAVEL=4`、`BTRAVEL=5`、`START=6`；速度类 `WVELOCITY=7`、`BVELOCITY=8`；推力类 `WFORCE=9`、`BFORCE=10`；`STAY=11`（2026-10-12 扩充，此前只有 1–6） |
| `axes` | `string` | 生效轴：`"x"`/`"y"`/`"z"`/`"rz"` 任意组合，空=全部。**对 SET/WMOVE/WTRAVEL/WVELOCITY/BVELOCITY/WFORCE/BFORCE 真正生效**（2026-10-10 实测更正：此前多处文档写"仅用于日志"）：只更新列出的轴，其余轴**沿用上一目标**（速度/推力类沿用**上一条速度/推力目标**，与位置目标不是同一份状态）；**对 BMOVE/BTRAVEL/START/STAY 完全不参与**。极性与 `ZitSetpoint.type_mask` **相反**（那里 1=忽略） |
| `target` | `float32[]` | 含义随 `cmd_type` 变（见下表）。**必须 ≥4 个元素**，即使 WTRAVEL/BTRAVEL 忽略 yaw。**yaw 一律是度**（速度为**度/秒**；推力是 `Mz`，不作单位换算） |
| `timeout` | `float32` | 秒；`≤0` ⇒ 默认 60 s。位置类=到位等待上限 |
| `duration` | `float32` | 秒；**只有速度/推力类使用**：保持这么久后自动收尾为原地保持。取值为 `duration > timeout > DEFAULT_DURATION(10 s)`。其它 `cmd_type` 传非 0 会被**忽略并告警**（不静默） |

**`target` 的含义（按 `cmd_type`）**

| cmd_type | `target` | 坐标系 |
|---|---|---|
| `SET` | `[x, y, z, yaw]` | odom（世界）系，米与度；**绝对目标** |
| `WMOVE` | `[x, y, z, yaw]` | odom 系；**绝对目标**（⚠️ 2026-10-12 更正：此前本表写 `[dx, dy, dz, dyaw]`「偏移」，与实现不符 —— 实现一直是绝对的，见下方"位置类语义") |
| `WTRAVEL` | `[x, y, z, yaw]`（yaw 忽略） | odom 系；**绝对目标**（同上更正；`_travel_world` 用 `target - 当前位姿` 算位移） |
| `BMOVE` | `[dx, dy, dz, dyaw]` | **机体系偏移**，`basic_motion` 内转世界 |
| `BTRAVEL` | `[dx, dy, dz]` | **机体系偏移**，`basic_motion` 内转世界 |
| `WVELOCITY` / `BVELOCITY` | `[vx, vy, vz, vyaw_rate]` | m/s 与**度/秒**；世界/机体由 `cmd_type` 决定 |
| `WFORCE` / `BFORCE` | `[Fx, Fy, Fz, Mz]` | 与 `force_4dof` 同量纲（调好增益下的单位） |
| `STAY` | 忽略 | 目标 = **当前位姿** |

> **位置类语义（2026-10-12 实测厘清）**：`SET`/`WMOVE`/`WTRAVEL` 的 `target` 都是
> **odom 系绝对目标**；只有 `BMOVE`/`BTRAVEL` 是**机体系偏移**。
> 实测：在 odom x=5 发 `WTRAVEL target=[2.0,...]`，basic_motion 日志为
> `WTRAVEL: 目标=(2.00, 0.00, 0.00), 方向角=180.0°` → `直线移动 第二阶段(前进): 距离=3.00m`
> → 结束于 x=2.00（**掉头往回走**），而不是前进到 x=7。`WMOVE` 同理（从 x=4.94 到 x=2.00）。
> 此前 action 文件与 4 处 md 把它写成「偏移量」，照那个写会让船朝**反方向**走。
| `BMOVE` / `BTRAVEL` | `[dx, dy, dz, dyaw]` | 机体系；`basic_motion` 内转世界系 |
| `WVELOCITY` / `BVELOCITY` | `[vx, vy, vz, vyaw_rate]` | m/s 与**度/秒**；世界/机体由 `cmd_type` 决定 |
| `WFORCE` / `BFORCE` | `[Fx, Fy, Fz, Mz]` | 与 `force_4dof` 同量纲（调好增益下的单位）；世界/机体由 `cmd_type` 决定 |
| `STAY` | 忽略 | 目标 = **当前位姿** |

**世界系与机体系在哪转换**：**在 MCU 侧**，由 `control_key` 的 frame 位（`0x10`）表达，
固件与仿真都按当前姿态的 `R_ZYX^T` 转（`SetpointRouter` / `mcu_control.world_to_body_vector`，
见 `docs/roadmap.md` B21）。`basic_motion` 只填 frame 位，自己不做转换 —— 位置类例外（历史行为：
`BMOVE`/`BTRAVEL` 在 `basic_motion` 内转世界后统一走 `0x00`）。

> ⚠️ **mask/axes 指机体轴**：世界系命令是**先转换、再按机体轴套 mask**（与固件顺序一致）。
> 所以在 `ZitSetpoint` 层面，"世界系北向速度 + 忽略机体 y"= 什么都不动。`basic_motion`
> 的 `axes` 走的是目标融合而非 mask，不受这条影响，但直接用 `ZitSetpoint` 时要注意。

**Result**

| 字段 | 类型 | 语义 |
|---|---|---|
| `success` | `bool` | `True`=到位；`False`=超时/被取消/被抢占/被拒绝 |
| `message` | `string` | 空字符串=成功；`"cancelled"`；`"superseded"`（被更新的 goal 抢占）；`"motion timeout"`；未初始化原点时 `"odom origin not set, call start() first"` |

**打断语义（2026-10-12 新增）**：一条命令被 **cancel** 或被**更新的 goal 抢占**时，都会
**先把目标设成当前位姿（原地保持）再返回**，然后才轮到新命令下发自己的目标。原因：仅仅
「停止下发」会让 MCU 继续执行上一条指令（速度/推力模式下尤其危险）。
`"cancelled"` 与 `"superseded"` 的区别在于是谁发起的：前者是客户端 `cancel_goal`，后者是新 goal 抢占。
**超时不算打断**，不主动 hold（沿用既有行为，MCU 会继续走向原目标）。

**抢占不是排队**：新 goal 立刻被接受（`goal_callback` 不拒绝），正在执行的命令在下一次
轮询（≤100 ms，`_wait_reached` 的 sleep 周期）让位。实测：长 SET 执行中发第二条 SET，
前者 0.05 s 内以 `"superseded"` 结束、并先发出一条「目标=当前位姿」的位置 setpoint，
后者随后正常完成。

**Feedback**

| 字段 | 类型 | 语义 |
|---|---|---|
| `distance_remaining` | `float32` | 位置类：目标的三维欧氏距离（**不含 yaw 误差**）。速度/推力类**恒为 0**（没有位置目标，不拿上一次目标充数）。0.5 Hz，**仅命令执行中有值** |
| `velocity_current` | `float32[]` | 机体**实测**速度 `[vx, vy, vz, vyaw_rate]`，m/s 与**度/秒**（来自 `/zit6/state/vel`；那里角速度是 rad/s，这里已换算，以与 `target` 的约定一致）。无数据时为空数组 |
| `velocity_target` | `float32[]` | 机体**目标**速度，单位同上；仅速度命令有意义，其它命令为全 0（不是"沿用上一条"） |
| `control_level` | `uint8` | 回读 MCU 控制层级：`0=NONE 1=POSITION 2=VELOCITY 3=FORCE`（来自 `/zit6/state/status`） |

> `control_level` **无法区分世界系与机体系**（WVELOCITY 与 BVELOCITY 都报 `2`）。要判断
> frame 位是否生效，只能看船实际往哪个方向动 —— 测试就是这么做的（见 `docs/roadmap.md` B21）。

**拒绝与边界**（代码锚点 `workspace_auv/src/uv_control/uv_control/basic_motion.py`）

1. **必须先 `START`**：任何 SET/WMOVE/BMOVE/WTRAVEL/BTRAVEL 之前要发一次 `START`；未发则 goal 被
   reject（message 见上）。`START` 无视 `axes`/`target`/`timeout`。
2. **重复 `START` 不是无操作**：每次都重写 odom 原点（先打 `odom origin already set, updating to
   current map pose`），当前 map 位姿成为新的 (0,0,0,0)。
3. **容差**：x/y/z 0.1 m，yaw 5°。
4. **单位**：`target` 的位置用米、yaw 用**度**（NED，顺时针为正）。
5. **odom 系 vs map 系**：`basic_motion` 只认 odom 系；`PoseInfo` 同时给 `origin_*` 与 `robot_*`，
   两者相减即可回到 map 系（`odom 0° = 初始朝向`）。**四轴全部相对**：`start()` 的 `_origin`
   取当前 map 位姿的 `x/y/z/rz`（rx/ry 恒 0），所以 odom z ≠ map z，`SET(x,y,3,0)` 是
   「比 START 时再深 3 m」。2026-10-12 实测：START 在 map z=6.89 → `SET(z=1)` → map z=7.79。
   ⚠️ 因此**重复 `START` 会移动深度基准**；2026-09-16 之前是"原点 z 写死 0 ⇒ odom z = 绝对深度"，
   那次结论已被本次改动取代。

## 3. 服务 `/zit6/get_params`（`zit6_interfaces/srv/GetParams`）

**请求**

| 字段 | 类型 | 语义 |
|---|---|---|
| `paths` | `string[]` | 点路径数组（如 `chassis.pid.pos.z`）；**空数组 = 返回全部通道** |

**响应**

| 字段 | 类型 | 语义 |
|---|---|---|
| `success` | `bool` | SIL 实现恒为 `True`（读操作不会失败） |
| `message` | `string` | 失败说明；SIL 实现为空 |
| `config_json` | `string` | JSON：`{路径: {kp, ki, kd, i_limit[, z_thrust_ff]}}` |

**语义与边界**（锚点 `workspace_sim/src/uv_sim/mcu_sim.py` → `uv_sim/mcu_control.py`）

- 9 个通道：`chassis.pid.pos.{x,y,z,yaw}` + 串级内环 `chassis.pid.pos.yaw_rate` +
  `chassis.pid.vel.{x,y,z,yaw}`；`pos.z`/`vel.z` 额外带 `z_thrust_ff`（浮力前馈）。
- **只读一个来源**：返回值与实际运行的 `Pid` 对象由同一份
  `config/sim_pid.yaml` 构造（`uv_sim/pid_config.py`），因此"服务报的值 ≠ 生效的值"这一历史缺陷
  （B14）不可能复发。请求不存在的路径 ⇒ 该键**不出现**在 JSON 里（不是报错）。
- HIL/实机 ⏸：由 MCU 经 micro-ROS 提供同一服务。

## 4. 服务 `/zit6/update_params`（`zit6_interfaces/srv/UpdateParams`）

**请求**（两种写法互斥，`json` 优先）

| 字段 | 类型 | 语义 |
|---|---|---|
| `json` | `string` | JSON 对象；**非空时忽略** `paths`/`values` |
| `paths` | `string[]` | 点路径（用 `paths`/`values` 写法时） |
| `values` | `string[]` | 与 `paths` 一一对应的**字符串化**值 |

**响应**

| 字段 | 类型 | 语义 |
|---|---|---|
| `success` | `bool` | `False` 时 `message` 是异常字符串（如 JSON 解析失败） |
| `message` | `string` | `"Parameters updated"` / 错误说明 |

**语义与边界**

- **值的三态**：字典值只覆盖它显式给出的字段（如只给 `kp`）；**单个数值 = 只改 `kp`**；
  未知路径/未知字段**静默忽略**。
- **只改运行时，永不写回 `sim_pid.yaml`**；调好后要手工落盘（`docs/node_parameters.md`）。
- 越界值不报错（`i_limit` 只影响积分钳位），非法数字会让 `success=false`。
- HIL/实机 ⏸：由 MCU 决定处理方式（`trigger_replan`/`persist` 字段已从 `.srv` 移除）。

## 5. 服务 `/nav/navigate_to`（`uv_msgs/srv/RunTask`，服务端 `navigator`）

| 字段 | 方向 | 语义 |
|---|---|---|
| `task_name` | 请求 | **逗号分隔的坐标**：`"x,y"` 或 `"x,y,z"`（`navigator._navigate_to_cb` 里 `split(',')` 后 `float()`），**不是任务名** |
| `start` | 请求 | **未使用**（服务名已表达语义；`RunTask` 类型只是被两个服务端共用） |
| `success` | 响应 | 规划并开始执行成功 |
| `message` | 响应 | 成功 `'Navigation planned'`；失败 `'Navigation failed'`；解析异常时是异常字符串 |

**就绪前置**：未收到 `/basic_motion/pose_info` 位姿前 `navigate_to()` **拒绝规划**
（`navigator.py:82` 打 `navigate_to rejected: no pose yet on /basic_motion/pose_info
(is basic_motion running and has START been sent?)`）。注意**响应里看不到这个原因**：它返回
`success=false, message='Navigation failed'`，具体原因只在该节点的日志里——排查
"服务说不成功但不知道为啥"时先看 `navigator` 的日志。
（另一半常见原因是**没发 START**：`PoseInfo` 有值但 odom 原点未设时下游动作会被
`basic_motion` 拒绝。）

## 6. 服务 `/task/run`（`uv_msgs/srv/RunTask`，服务端 `task_runner`）

| 字段 | 方向 | 语义 |
|---|---|---|
| `task_name` | 请求 | 任务名，**或** `tasks.json` 的路径（`start=true` 时用它装载） |
| `start` | 请求 | `true`=启动任务列表；`false`=等价于停止 |
| `success` | 响应 | 装载/启动是否成功 |
| `message` | 响应 | `Started N tasks` / `No tasks loaded` / `Stopped` |

> `task_runner` **启动即自动跑** `config/tasks.json`（非 debug 模式）——不调本服务它也会动，
> 这是已知的"意外运动"来源（见 `docs/debug_guide.md`）。

## 7. 服务 `/task/stop`（`std_srvs/srv/Trigger`）

无请求字段。响应 `success=true`、`message='Tasks stopped'`（`std_srvs/Trigger` 的固定形状）。
效果：置内部 `stopped` 标志（列表不再发新命令）**并取消当前正在执行的 action goal**；不改变 `tasks.json`。

**2026-10-12 修**：此前取消用的是 `self._action_client.async_cancel_goal(handle)`，而 rclpy
Jazzy 的 `ActionClient` **没有这个方法**（客户端侧根本没有取消 API，取消只能在 **goal handle**
上调 `cancel_goal_async()`）。`AttributeError` 被服务回调吞掉，所以 `/task/stop` 一直是
**「停列表但不停船」**：`stopped=True` 让列表不再发新命令，可当前那条 goal 会继续跑到目标。
现在实测能停：`basic_motion` 日志出现 `Action cancel requested, accepting` →
`等待到达: 被让位` → `打断收尾: 目标设为当前位置`。

**另注**：`stopped` 标志会被 `/task/run`（`run_task_list` 开头）与 `/task/exec`
（`_debug_exec_single` 开头）重置，所以 `/task/stop` 之后仍可继续手动发单条任务。

## 8. 服务 `/task/exec`（`uv_msgs/srv/ExecTask`）

| 字段 | 方向 | 语义 |
|---|---|---|
| `task_name` | 请求 | `task_runner.task_map` 里的名字（`wtravelxy`、`setx`、`start`…，**`task_map` 是权威清单**） |
| `params_json` | 请求 | 该任务的参数字典，JSON 字符串（如 `{"dx": 5.0, "dy": 0.0}`） |
| `timeout` | 请求 | 覆盖 action goal 的超时（秒）；`≤0` 用默认 |
| `success` / `message` | 响应 | 成功或具体原因 |

**唯一前置**：**仅 `debug_mode:=true` 时可用**；否则
`success=false, message='ExecTask service only available in debug mode'`。
未知任务名会返回合法清单；`params_json` 解析失败也有专门 message。

## 9. 引擎自带服务（**栈外提供者**，登记备查）

实测（快照 `2026-10-10_sil-console/services.txt`）由 `stonefish_ros2` 提供，不属于本栈代码，
因此在 `topic_contract.md` §7 里没有行（那份表要求 `create_service` 证据）：

| 名称 | 类型 | 用途 |
|---|---|---|
| `/stonefish_ros2/enable_currents` | `std_srvs/srv/Trigger` | 打开场景海流 |
| `/stonefish_ros2/disable_currents` | `std_srvs/srv/Trigger` | 关闭场景海流 |
| `/stonefish_ros2/respawn_robot` | `stonefish_ros2/srv/Respawn` | 复位机器人（跑崩后不用重启整栈） |

## 10. 定义了但没有任何节点使用的接口（**死定义登记**，勿当成可用接口）

审计方式：在 `workspace_auv/src`、`workspace_sim/src`、`visualization/` 全量 grep 类型名
（2026-10-10）。用途不明的定义不要删，但用之前要知道它**没有实现**：

| 定义 | 现状 |
|---|---|
| `uv_msgs/srv/GetState` | **无服务端**（响应是 `AuvState`，而 `/auv/state` 早已无发布者，见契约 §9） |
| `uv_msgs/msg/AuvState` | 无发布者、无订阅者（契约 §9 已登记） |
| `uv_msgs/msg/MotionCommand` | 0 处引用；轴掩码语义现在由 `zit6_interfaces/ZitSetpoint.type_mask` 承担 |
| `uv_msgs/msg/DeviceCommand` | 0 处引用 |
| `uv_msgs/msg/PidGains`、`uv_msgs/msg/PidState` | 0 处引用；PID 增益的事实源是 `uv_sim/config/sim_pid.yaml` |

> `stonefish_ros2` 自带的其它消息（`INS`、`BeaconInfo`、`EventArray`、`DebugPhysics`…）不在本表：
> 它们服务的是别的传感器/场景，属引擎文档范围。
