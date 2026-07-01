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
---
uint8 cmd_type
string axes         # 生效轴: "x"/"y"/"z"/"rz" 任意组合, 空=全部
float32[] target    # [x, y, z, yaw]
float32 timeout     # 秒, ≤0=默认60s
---
# Result
bool success
string message
---
# Feedback
float32 distance_remaining   # 到目标的 3D 欧氏距离
```

---

## 坐标系约定

| 项目        | 约定                                             |
| ----------- | ------------------------------------------------ |
| 坐标系      | **NED**（North-East-Down）                 |
| Yaw 0°     | 北（N）                                          |
| Yaw 正方向  | **顺时针**                                 |
| 角度单位    | **度**（°），内部使用，ROS 消息层也使用度 |
| odom 原点   | START 时 AUV 的位置，START 后所有坐标为相对值    |
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
- odom (0,0,0,0) = AUV 在 START 时刻的位置和朝向
- 例：AUV 朝东（map 90° 方向）→ START → odom 0° = 朝东
  - 之后 `SET(5, 0, 0, 0)` 表示向东走 5m
- 重复发送 START 是无操作（log 提示已设置）

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
| STEP_PERIOD    | 0.3s | 收敛时间阈值         |
| LATERAL_LAMBDA | 2.0  | 横向误差指数衰减系数 |

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

---

## 结果与反馈

### Result

| 字段        | 类型   | 说明                                                                |
| ----------- | ------ | ------------------------------------------------------------------- |
| `success` | bool   | True = 成功到达目标                                                 |
| `message` | string | 空 = 成功, "motion timeout" = 超时, "cancelled" = 取消, 其他 = 错误 |

三种终态：

| 状态 | success | message          | goal_handle    |
| ---- | ------- | ---------------- | -------------- |
| 成功 | True    | ""               | `succeed()`  |
| 超时 | False   | "motion timeout" | `abort()`    |
| 取消 | False   | "cancelled"      | `canceled()` |

### Feedback

| 字段                   | 类型    | 频率  | 说明                                  |
| ---------------------- | ------- | ----- | ------------------------------------- |
| `distance_remaining` | float32 | 0.5Hz | 到目标的 3D 欧氏距离（不计 yaw 误差） |

仅在运动进行中有反馈值，空闲时不发。

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
4. **axes 字段**当前仅用于日志记录，server 内部未实现（始终全轴生效）
5. **yaw 单位**：始终是**度**，不是弧度
6. **MultiThreadedExecutor**：basic_motion 内部使用多线程执行器，action 回调在独立线程运行
