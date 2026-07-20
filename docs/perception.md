# 感知系统文档

## 概述

感知系统位于 `uv_perception` 包，由两个节点组成两级流水线：

- **vision** — YOLO 目标检测，将双目拼接图拆分为左右独立通道分别推理
- **position** — 单目多帧射线交会，从 2D 检测结果反算 3D 世界坐标

两个节点通过 `DetectionArray` 消息解耦：vision 只关心"图像中有什么"，position 只关心"目标在 3D 空间中的位置"。

---

## 架构

```
sim_bridge                          vision                              position
─────────                          ──────                              ────────
front_cam/left ──┐                 ┌─ front_left  (1280×960) ── YOLO ──→ /perception/detection/front_left
front_cam/right ─┼─ hstack(2560×960)┤                                         │
                 └─ /auv/front_cam/stitched ──┤                             │
                                              ├─ split ──┤                  │
                 ┌─ /auv/down_cam/stitched  ──┤           │                  │
down_cam/left  ─┼─ hstack(2560×960)          │  └─ front_right (1280×960) ──→ /perception/detection/front_right
down_cam/right ─┘                            │                              │
                                             │  ┌─ down_left  (1280×960) ──→ /perception/detection/down_left
                                             └─┤                              │
                                                └─ down_right (1280×960) ──→ /perception/detection/down_right
                                                                             │
basic_motion ── /basic_motion/pose_info (30Hz) ─────────────────────────────┘
                                                                             │
                                                              射线累积 + 交会
                                                                             │
                                                                             ↓
                                                              /perception/objects (10Hz)
                                                                             │
                                                              ┌──────────────┤
                                                              ↓              ↓
                                                         NavigatorNode  TaskRunnerNode
```

---

## VisionNode (`vision`)

**节点名**: `vision`
**源文件**: `uv_perception/vision.py`

### 功能

订阅 sim_bridge 发布的拼接双目图像，从中线切开为左右独立图像（各 1280×960），分别运行 YOLO 检测，每个通道独立发布检测结果。

### 订阅

| 主题 | 类型 | 说明 |
|------|------|------|
| `/auv/front_cam/stitched` | `sensor_msgs/Image` | 前视双目拼接图 (2560×960, bgr8) |
| `/auv/down_cam/stitched` | `sensor_msgs/Image` | 下视双目拼接图 (2560×960, bgr8) |

### 发布

| 主题 | 类型 | 说明 |
|------|------|------|
| `/perception/detection/front_left` | `DetectionArray` | 前视左目检测结果 |
| `/perception/detection/front_right` | `DetectionArray` | 前视右目检测结果 |
| `/perception/detection/down_left` | `DetectionArray` | 下视左目检测结果 |
| `/perception/detection/down_right` | `DetectionArray` | 下视右目检测结果 |
| `/perception/image/front_left` | `sensor_msgs/Image` | 前视左目图像 (仅 publish_images:=true) |
| `/perception/image/front_right` | `sensor_msgs/Image` | 前视右目图像 (仅 publish_images:=true) |
| `/perception/image/down_left` | `sensor_msgs/Image` | 下视左目图像 (仅 publish_images:=true) |
| `/perception/image/down_right` | `sensor_msgs/Image` | 下视右目图像 (仅 publish_images:=true) |

### 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `model_path` | `str` | `""` | YOLO 模型 .pt 文件路径。为空时自动搜索默认路径 |
| `publish_images` | `bool` | `false` | 是否转发拆分后的原始图像到 `/perception/image/*` |

### 图像拆分

拼接图由 sim_bridge 通过 `np.hstack` 水平拼接左右目（各 1280×960）得到 2560×960。vision 从正中间切开：

```python
h, w = cv_img.shape[:2]
mid = w // 2
left_img = cv_img[:, :mid]    # 1280×960
right_img = cv_img[:, mid:]   # 1280×960
```

左右目各跑一次 YOLO，推理结果独立发布。camera_name 字段标记为 `"front_left"`, `"front_right"`, `"down_left"`, `"down_right"`。

### 检测结果格式

每条检测 (`Detection` 消息) 包含：

| 字段 | 含义 |
|------|------|
| `class_id` | YOLO 类别 ID |
| `confidence` | 置信度 (0~1) |
| `pixel_x`, `pixel_y` | 边界框中心像素坐标 |
| `bbox_x1`, `bbox_y1`, `bbox_x2`, `bbox_y2` | 边界框四角 |

---

## PositionNode (`position`)

**节点名**: `position`
**源文件**: `uv_perception/position.py`

### 功能

订阅 4 个通道的检测结果和机器人位姿，通过**多帧单目射线交会**反算目标的 3D 世界坐标，以 10Hz 发布被跟踪物体的位置列表。

### 订阅

| 主题 | 类型 | 说明 |
|------|------|------|
| `/perception/detection/front_left` | `DetectionArray` | 前视左目检测 |
| `/perception/detection/front_right` | `DetectionArray` | 前视右目检测 |
| `/perception/detection/down_left` | `DetectionArray` | 下视左目检测 |
| `/perception/detection/down_right` | `DetectionArray` | 下视右目检测 |
| `/basic_motion/pose_info` | `PoseInfo` | 机器人位姿 (30Hz) |

### 发布

| 主题 | 类型 | 频率 | 说明 |
|------|------|------|------|
| `/perception/objects` | `ObjectPositionArray` | 10Hz | 被跟踪物体的 3D 世界坐标列表 |

### 相机参数

| 参数 | 前视 | 下视 |
|------|------|------|
| 分辨率 (单目) | 1280×960 | 1280×960 |
| 水平视场角 (HFOV) | 34.19° | 32.18° |
| 焦距 fx=fy | 2158.4 | 2307.6 |
| 光心 cx, cy | (640, 480) | (640, 480) |
| 机体安装偏移 (x, y, z) | (0.25, -0.02825, 0.25) m | (0.0, 0.03765, 0.21) m |

### 算法：多帧单目射线交会

PositionNode 利用 AUV 运动过程中从不同位置观察同一目标的多个视角，通过射线交会实现单目三角定位。

**1. 像素 → 相机射线**

利用针孔相机模型，将检测到的像素中心映射为相机坐标系中的射线方向：

```
v_cam = [(px - cx) / fx, (py - cy) / fy, 1.0]
v_cam = normalize(v_cam)
```

**2. 相机射线 → 世界射线**

经过两级旋转变换：

```
v_body = optical_to_body @ v_cam     # 相机系 → 机体 NED
v_world = R_robot(yaw) @ v_body      # 机体 NED → 世界 NED
ray_origin = robot_pos + R_robot @ camera_offset
```

注：当前仅使用 yaw 旋转（pitch/roll 假定为 0）。

**3. 射线累积与剪枝**

- 按 `(class_id, camera_pair)` 分组维护射线历史（最多 `max_history`，默认 30 条）
- 新射线与上一条的基线距离须 ≥ `MIN_BASELINE=0.1m` 才加入（保证足够视差）
- 超限时按冗余度评分剪枝：高余弦相似度 + 近距离的射线优先删除

**4. 配对交会（主方法）**

对历史中所有射线对求公垂线中点，经过多重质量过滤：

| 过滤条件 | 阈值 |
|----------|------|
| 近线平行 | cos(angle) < 0.9995 |
| 前向交点 | t1 > 0, t2 > 0 |
| 垂距 | < 5.0m |

所有有效交点取**中值**作为最终位置（抗离群值）。

**5. 最小二乘回退**

若配对法无有效解，用 ≥ 3 条射线解超定线性方程组：

```
Σ(I - d_i·d_i^T) · p = Σ(I - d_i·d_i^T) · o_i
```

条件数 < 1e5 才接受解。

**6. 位置验证**

- 物体距离 > `MAX_DISTANCE=50m` 时丢弃
- 验证通过的位置写入 `object_memory`，按 class_id 索引

### 物体位置输出

`ObjectPosition` 消息字段：

| 字段 | 含义 |
|------|------|
| `class_id` | 物体类别 ID |
| `class_name` | 类别名称 (如 blue_drump, gate_green_stick 等) |
| `world_x`, `world_y`, `world_z` | 世界 NED 坐标 |
| `confidence` | 最近一次检测置信度 |
| `num_observations` | 用于定位的射线数量 |

---

## 机器人位姿源 (`PoseInfo`)

感知系统通过 `basic_motion` 节点获取机器人位姿，不再依赖仅仿真可用的 `/auv/state`。

**主题**: `/basic_motion/pose_info`
**类型**: `uv_msgs/msg/PoseInfo`
**频率**: 30Hz

```
builtin_interfaces/Time stamp    # 位姿对应的时间戳
float32 origin_x/y/z/yaw         # odom 原点在 map 系中的位姿 (yaw: 度)
float32 robot_x/y/z/yaw          # 机器人当前 odom 系位姿 (yaw: 度)
float32 target_x/y/z/yaw         # 当前运动目标 odom 系位姿 (yaw: 度)
```

PositionNode 使用 `robot_x/y/z` 和 `robot_yaw`（度）作为射线原点。

---

## 坐标系

- **odom 系**: 以 START 时 AUV 位置为原点(0,0,0)，初始朝向为 yaw=0°。position 的 `robot_pos` 和输出 `world_*` 均在此坐标系
- **map 系**: MCU/仿真器上报的原始坐标系。PoseInfo 中的 `origin_*` 记录了 odom 原点在 map 系中的位姿
- **NED**: x=北(N), y=东(E), z=下(D)，yaw 顺时针为正，度为单位

---

## 启动

### 独立启动

```bash
ros2 launch uv_perception perception_launch.py
```

### 作为仿真/实车的一部分

```bash
# 仿真 (默认 enable_ai:=true)
ros2 launch uv_bringup sim_bringup.py

# 关闭感知
ros2 launch uv_bringup sim_bringup.py enable_ai:=false

# 开启图像转发 (调试用)
ros2 run uv_perception vision --ros-args -p publish_images:=true
```

---

## 下游消费者

| 节点 | 包 | 用途 |
|------|-----|------|
| `NavigatorNode` | `uv_nav` | 将 ObjectPosition 作为障碍物加入 A* 路径规划 (膨胀半径 2m) |
| `TaskRunnerNode` | `uv_task` | 存储 `self.objects`，任务 handler 可引用感知数据做决策 |

---

## 已知局限

1. 仅使用 yaw 旋转，忽略 pitch/roll — AUV 大姿态倾斜时定位精度下降
2. 相机内参硬编码在 `position.py` 中，`config/*.npz` 立体校准文件未被使用
3. 需要 AUV 运动产生视差才能定位（静态悬停时无法更新位置）
4. 最大感知距离 50m，超过此范围的目标被丢弃
