# BasicMotion Action Server 接口文档

## 概述

`basic_motion` 节点（`uv_control` 包）提供 `uv_msgs/action/BasicMotion` Action Server，是 YouLong AUV 所有运动控制的统一入口。其他节点（`uv_task`、`uv_nav` 等）通过 ROS 2 Action Client 发送 goal 来执行运动。

- Action 名称：`basic_motion`
- Action 类型：`uv_msgs/action/BasicMotion`

---

## Action 定义

文件：`uv_msgs/action/BasicMotion.action`

```
uint8 WMOVE=1       # 世界系步进, target=[dx, dy, dz, dyaw]
uint8 BMOVE=2       # 机体系步进, target=[dx, dy, dz, dyaw]
uint8 SET=3         # 绝对定位, target=[x, y, z, yaw]
uint8 WTRAVEL=4     # 世界系直线, target=[dx, dy, dz]
uint8 BTRAVEL=5     # 机体系直线, target=[dx, dy, dz]
uint8 START=6       # 初始化里程计原点
uint8 WVELOCITY=7   # 世界系速度, target=[vx, vy, vz, vyaw_rate(度/秒)]   (2026-10-12 新增)
uint8 BVELOCITY=8   # 机体系速度, target=[vx, vy, vz, vyaw_rate(度/秒)]   (2026-10-12 新增)
uint8 WFORCE=9      # 世界系推力, target=[Fx, Fy, Fz, Mz]                 (2026-10-12 新增)
uint8 BFORCE=10     # 机体系推力, target=[Fx, Fy, Fz, Mz]                 (2026-10-12 新增)
uint8 STAY=11       # 原地保持：目标=当前位姿，立即返回                    (2026-10-12 新增)
---
uint8 cmd_type
string axes         # 生效轴: "x"/"y"/"z"/"rz" 任意组合, 空=全部
float32[] target    # 4 元素；含义随 cmd_type，yaw 一律**度**（速度为度/秒）
float32 timeout     # 秒, ≤0=默认60s。位置类=到位等待上限
float32 duration    # 秒, 仅速度/推力类：保持这么久后自动收尾为原地保持
---
# Result
bool success
string message      # ""/"cancelled"/"superseded"/"motion timeout"/"duration elapsed"
---
# Feedback
float32 distance_remaining   # 位置类：到目标的 3D 欧氏距离；速度/推力类恒 0
float32[] velocity_current   # 机体实测 [vx, vy, vz, vyaw_rate]，m/s 与 度/秒
float32[] velocity_target    # 机体目标，同上；非速度命令为全 0
uint8 control_level          # 回读 MCU 层级 0=NONE 1=POS 2=VEL 3=FORCE
```

---

## 坐标系约定

| 项目        | 约定                                             |
| ----------- | ------------------------------------------------ |
| 坐标系      | **NED**（North-East-Down）                 |
| Yaw 0°     | 北（N）                                          |
| Yaw 正方向  | **顺时针**                                 |
| 角度单位    | **度**（°），内部使用，ROS 消息层也使用度 |
| odom 原点   | START 时 AUV 的位置，START 后 `x`/`y`/`yaw` 为相对值；**`z` 例外：原点 z 固定为 0 ⇒ odom z = 绝对深度（水面 0）** |
| 机体系 body | body-x = 机头方向，body-y = 右侧                 |

---

## 前置条件：START

**任何运动命令之前必须先发送 START。** 如果 odom 原点未设置，server 会拒绝 goal。

```python
from uv_msgs.action import BasicMotion

goal = BasicMotion.Goal()
goal.cmd_type = BasicMotion.Goal.START
# target/timeout/axes 均被忽略
```

START 的行为：

- 记录当前 map 位姿为 odom 原点
- odom (0,0,0,0) = AUV 在 START 时刻的位置和朝向（**z 除外**：`start()` 里 `_origin.z` 被写死为
  `0.0`，所以 odom z 等于 map z，即**绝对深度**；`SET(x,y,3,0)` = 下潜到 3 m 而不是「比开始时深 3 m」）
- 例：AUV 朝东（map 90° 方向）→ START → odom 0° = 朝东
  - 之后 `SET(5, 0, 0, 0)` 表示向东走 5m
- **重复发送 START 不是无操作**：`start()` 会覆盖原点（源码里先打
  `odom origin already set, updating to current map pose`，随后更新 x/y/z/yaw）
- 证据/实测：`uv_control/basic_motion.py:start()`；2026-09-16 console SIL 实测
  `SET(0.5, 0.0, 3.0, 0.0)` 从水面到 map z≈3.12（见 `devlog/2026-09-16_status.md`）

---

## 6 种命令类型

### SET — 绝对定位

```
cmd_type: BasicMotion.Goal.SET
target:   [x, y, z, yaw]    # odom 系绝对坐标
```

- 直接发送目标位置到 odom 系，阻塞等待到达
- 容差：x/y/z = 0.1m, yaw = 5°
- 适用于已知精确目标位置的场景

**示例：**

```python
goal.cmd_type = BasicMotion.Goal.SET
goal.target = [5.0, 0.0, -2.0, 90.0]
# 结果：AUV 移动到 odom x=5, y=0, z=-2, 朝东(yaw=90°)
```

### WMOVE — 世界系步进

```
cmd_type: BasicMotion.Goal.WMOVE
target:   [dx, dy, dz, dyaw]    # 世界系偏移量
```

- 从当前位置加上偏移量为目标
- 使用动态步进算法：长距离拆成小段逐段发送
- 每步根据当前横向误差动态调整步长
- 横向误差大时步长自动减小，提高抗干扰能力

**步进参数：**

| 参数           | 值   | 说明                 |
| -------------- | ---- | -------------------- |
| STEP_X         | 0.6m | X 方向椭圆半轴       |
| STEP_Y         | 0.4m | Y 方向椭圆半轴       |
| STEP_PERIOD    | 0.2s | 收敛时间阈值         |
| LATERAL_LAMBDA | 2.0  | 横向误差指数衰减系数 |

> 上表与 `basic_motion.py` 的 `STEP_X/STEP_Y/STEP_PERIOD/LATERAL_LAMBDA` 常量对拍。
> 2026-10-12 更正：此前本表写 `STEP_Y=0.2m` / `STEP_PERIOD=0.3s`，与代码（`0.4` / `0.2`）不符。

**示例：**

```python
goal.cmd_type = BasicMotion.Goal.WMOVE
goal.target = [2.0, 1.0, 0.0, 0.0]
# 结果：AUV 向北 2m, 向东 1m（相对于当前位置）
```

### BMOVE — 机体系步进

```
cmd_type: BasicMotion.Goal.BMOVE
target:   [dx, dy, dz, dyaw]    # 机体系偏移量
```

- 参数为机体系偏移量，内部用 `body_to_world` 旋转到世界系
- body-x = 向前（机头方向），body-y = 向右
- 旋转后走 WMOVE 的步进逻辑

**示例：**

```python
goal.cmd_type = BasicMotion.Goal.BMOVE
goal.target = [2.0, 0.0, 0.0, 0.0]
# 结果：AUV 向前（当前朝向）2m
```

```python
goal.cmd_type = BasicMotion.Goal.BMOVE
goal.target = [0.0, 1.0, 0.0, 45.0]
# 结果：AUV 向右 1m 且右转 45°，先旋转再在 body 系移动
```

### WTRAVEL — 世界系直线移动

```
cmd_type: BasicMotion.Goal.WTRAVEL
target:   [dx, dy, dz, 0]       # yaw 被忽略
```

- 两步执行：先原地转向目标方向 → 再沿 body-X 直线前进
- 转向角 = `atan2(dy, dx)`
- 适用于需要直线轨迹的任务（过门、巡线等）

**示例：**

```python
goal.cmd_type = BasicMotion.Goal.WTRAVEL
goal.target = [3.0, 4.0, 0.0, 0.0]
# 结果：AUV 先转向 atan2(4,3)≈53° 方向，再向前走 5m
```

### BTRAVEL — 机体系直线移动

```
cmd_type: BasicMotion.Goal.BTRAVEL
target:   [dx, dy, dz, 0]       # yaw 被忽略
```

- body 偏移转 world 后走 WTRAVEL
- 两步执行：先转向目标方向 → 再沿 body-X 直线前进

**示例：**

```python
goal.cmd_type = BasicMotion.Goal.BTRAVEL
goal.target = [3.0, 0.0, 0.0, 0.0]
# 结果：AUV 先转向当前朝向方向，再向前走 3m
```

### 命令对比

| 命令    | 坐标系 | 目标含义 | 运动模式           | 典型场景         |
| ------- | ------ | -------- | ------------------ | ---------------- |
| SET     | odom   | 绝对坐标 | 直发 + 等到达      | 明确目标点       |
| WMOVE   | odom   | 偏移量   | 动态步进           | 向指定方向走一段 |
| BMOVE   | body   | 偏移量   | 旋转 + 动态步进    | 相对当前姿态移动 |
| WTRAVEL | odom   | 偏移量   | 转向 + 直线        | 过门、直线轨迹   |
| BTRAVEL | body   | 偏移量   | 旋转 + 转向 + 直线 | 沿当前方向直线   |
| START   | —     | —       | 初始化原点         | 开始作业         |
| WVELOCITY | **world** | 速度 | 世界系速度闭环 | 沿岸/沿画面前进 |
| BVELOCITY | **body**  | 速度 | 机体系速度闭环 | 巡线、边走边搜 |
| WFORCE  | **world** | 力/力矩 | 直接推力       | 手动遥控、顶流 |
| BFORCE  | **body**  | 力/力矩 | 直接推力       | 手动遥控、顶流 |
| STAY    | —     | —       | 原地保持       | 停下等待、收尾 |

---

## 速度 / 推力 / STAY（2026-10-12 新增）

### WVELOCITY / BVELOCITY — 速度闭环

```python
goal = BasicMotion.Goal()
goal.cmd_type = BasicMotion.Goal.BVELOCITY   # 机体系；WVELOCITY 是世界系
goal.target = [0.4, 0.0, 0.0, 0.0]           # 前进 0.4 m/s，不转
goal.duration = 3.0                          # 保持 3 秒
```

- **不阻塞到某一点**，而是"保持这个速度 `duration` 秒"
- `target = [vx, vy, vz, vyaw_rate]`：m/s 与**度/秒**（正=顺时针，NED 约定）
- 到时（或被打断）**自动收尾为原地保持** —— MCU 不会自己忘掉最后一条速度目标，
  不收尾就是"命令结束了但船还在走"
- `duration ≤ 0` 时依次回退 `timeout`、`DEFAULT_DURATION`(10 s)。速度命令**必须有界**
- 世界系 vs 机体系：`BVELOCITY` 的 `vx` 永远指艇头方向；`WVELOCITY` 的 `vx` 指北，
  **不随艇头转**。转换在 MCU 侧按当前姿态做（`R_ZYX^T`），`basic_motion` 只填 frame 位

### WFORCE / BFORCE — 直接推力

```python
goal.cmd_type = BasicMotion.Goal.BFORCE
goal.target = [300.0, -150.0, 420.0, 90.0]   # [Fx, Fy, Fz, Mz]
goal.duration = 2.0
```

- 绕过位置/速度环，直接给 4-DOF 推力（`mcu_sim` 的 `force_4dof`），量纲是"调好增益下的单位"
- 同样有 `duration`、到期收尾；GUI 的遥控面板走的就是 `control_key=0x12`
- 世界系/机体系同上

### STAY — 原地保持

```python
goal.cmd_type = BasicMotion.Goal.STAY
# target / duration 均忽略
```

- 把位置目标设为**当前位姿**后**立即返回**（不阻塞、无 duration），成功态
- 它就是"打断收尾"用的同一个动作（`_hold_current_position`），所以语义完全一致

### 打断语义（速度/推力尤其重要）

任何命令被 **cancel** 或被**更新的 goal 抢占**时，都会**先把目标设成当前位姿
（原地保持）再返回**，然后才轮到新命令下发自己的目标。所以"边走边看、看到就换命令"
是安全的：旧的速度命令不会在你切走之后继续推着船跑。

`message` 区分三种非成功终态：`"cancelled"`（客户端取消）、`"superseded"`（被新 goal
抢占）、`"motion timeout"`（位置类超时；**超时不主动 hold**，沿用既有行为）。
抢占**不是排队**：新 goal 立刻被接受，旧命令在下一次轮询（≤100 ms）让位。

> ⚠️ `control_level` 回读**分不出**世界系与机体系（WVELOCITY/BVELOCITY 都报 `2`）。
> 判断 frame 位是否生效只能看船实际往哪个方向动。

---

## PoseInfo 话题

basic_motion 同时以 30Hz 发布 `/basic_motion/pose_info` (类型 `uv_msgs/PoseInfo`)，供感知系统获取机器人位姿：

```
builtin_interfaces/Time stamp    # 时间戳
float32 origin_x/y/z/yaw         # odom 原点在 map 系中的位姿 (yaw: 度)
float32 robot_x/y/z/yaw          # 机器人当前 odom 系位姿 (yaw: 度)
float32 target_x/y/z/yaw         # 当前运动目标 odom 系位姿 (yaw: 度)
```

`position` 节点 (uv_perception) 订阅此话题作为射线原点，不再依赖仅仿真可用的 `/auv/state`。

---

## 结果与反馈

### Result

| 字段        | 类型   | 说明                                                                |
| ----------- | ------ | ------------------------------------------------------------------- |
| `success` | bool   | True = 成功到达目标（速度/推力类=跑满 duration）                    |
| `message` | string | 空 = 成功, "duration elapsed" = 速度/推力到时收尾, "motion timeout" = 超时, "cancelled" = 取消, "superseded" = 被新 goal 抢占, 其他 = 错误 |

终态（第 3 项为 2026-10-12 新增）：

| 状态 | success | message          | goal_handle    | 收尾 |
| ---- | ------- | ---------------- | -------------- | ---- |
| 成功（位置） | True | "" | `succeed()` | 已在目标点 |
| 成功（速度/推力） | True | "duration elapsed" | `succeed()` | **原地保持** |
| 超时 | False   | "motion timeout" | `abort()`    | 不动（位置模式继续走向原目标） |
| 取消 | False   | "cancelled"      | `canceled()` | **原地保持** |
| 被抢占 | False   | "superseded"     | `abort()`    | **原地保持** |

### Feedback

| 字段                   | 类型    | 频率  | 说明                                  |
| ---------------------- | ------- | ----- | ------------------------------------- |
| `distance_remaining` | float32 | 0.5Hz | 位置类：到目标的 3D 欧氏距离（不计 yaw）；速度/推力类恒 0 |
| `velocity_current` | float32[] | 0.5Hz | 机体实测 `[vx, vy, vz, vyaw_rate]`，m/s 与度/秒 |
| `velocity_target` | float32[] | 0.5Hz | 机体目标速度，单位同上；非速度命令为全 0 |
| `control_level` | uint8 | 0.5Hz | 回读 MCU 层级 0=NONE 1=POS 2=VEL 3=FORCE |

仅在命令执行中有反馈值，空闲时不发。

---

## 完整使用示例

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from uv_msgs.action import BasicMotion


def main():
    rclpy.init()
    node = Node('motion_client')
    client = ActionClient(node, BasicMotion, 'basic_motion')

    if not client.wait_for_server(timeout_sec=5.0):
        node.get_logger().error('basic_motion server not available')
        return

    # 1. START — 初始化 odom 原点
    goal = BasicMotion.Goal()
    goal.cmd_type = BasicMotion.Goal.START
    send_future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_future)
    gh = send_future.result()
    result_future = gh.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)
    result = result_future.result().result
    node.get_logger().info(f'START: {result.success} {result.message}')

    # 2. SET — 下潜到 z=-1m
    goal = BasicMotion.Goal()
    goal.cmd_type = BasicMotion.Goal.SET
    goal.target = [0.0, 0.0, -1.0, 0.0]
    goal.timeout = 30.0
    send_future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_future)
    gh = send_future.result()
    result_future = gh.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)
    result = result_future.result().result
    node.get_logger().info(f'SET z=-1: {result.success} {result.message}')

    # 3. BMOVE — 向前 3m
    goal = BasicMotion.Goal()
    goal.cmd_type = BasicMotion.Goal.BMOVE
    goal.target = [3.0, 0.0, 0.0, 0.0]
    goal.timeout = 60.0
    send_future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_future)
    gh = send_future.result()
    result_future = gh.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)
    result = result_future.result().result
    node.get_logger().info(f'BMOVE: {result.success} {result.message}')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## 在 task_runner 中使用

task_runner 通过同样的 ActionClient 接口调用 basic_motion。JSON 任务配置文件中的任务名映射到对应命令：

完整任务列表（SET/WMOVE/BMOVE/WTRAVEL/BTRAVEL 单轴到多轴共 25+ 个任务）见 `docs/debug_guide.md` 的 task_runner 章节。

---

## 注意事项

1. **必须先 START**：任何 SET/WMOVE/BMOVE/TRAVEL 之前必须发送 START。server 拒绝并返回 `"odom origin not set, call start() first"`
2. **target 必须 4 元素**：即使 WTRAVEL/BTRAVEL 忽略 yaw，也必须传 4 个值
3. **timeout ≤ 0** 自动使用 60s
4. **`axes` 会真正生效**（2026-10-10 实测更正；此前本文与 `CLAUDE.md` 都写"仅用于日志"）：
   对 **SET / WMOVE / WTRAVEL**，只有出现在 `axes` 里的轴会用 goal 的新值更新，
   其余轴**沿用上一目标**（代码里 `axes = req.axes or 'xyzrz'`，逐个 `if 'x' in axes: …`）；
   对 **BMOVE / BTRAVEL / START**，`axes` 完全不参与（始终全轴）。`axes` 空 = `'xyzrz'` = 全轴。
   注意这与 `zit6_interfaces/ZitSetpoint.type_mask` **极性相反**（那里 1=忽略该轴）。
5. **yaw 单位**：始终是**度**，不是弧度
6. **MultiThreadedExecutor**：basic_motion 内部使用多线程执行器，action 回调在独立线程运行
