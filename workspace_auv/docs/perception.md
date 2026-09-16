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
down_cam/left  ─┼─ hstack(3840×1080)         │  └─ front_right (1280×960) ──→ /perception/detection/front_right
down_cam/right ─┘                            │                              │
                                             │  ┌─ down_left  (1920×1080) ──→ /perception/detection/down_left
                                             └─┤                              │
                                                └─ down_right (1920×1080) ──→ /perception/detection/down_right
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

订阅 sim_bridge 发布的拼接双目图像，从**中线**切开为左右独立图像（前视各 1280×960、下视各 1920×1080），分别运行 YOLO 检测，每个通道独立发布检测结果。切分位置是 `mid = width // 2`，不依赖固定分辨率。

### 订阅

| 主题 | 类型 | 说明 |
|------|------|------|
| `/auv/front_cam/stitched` | `sensor_msgs/Image` | 前视双目拼接图 (2560×960, bgr8) |
| `/auv/down_cam/stitched` | `sensor_msgs/Image` | 下视双目拼接图 (**3840×1080**, bgr8) |

### 发布

| 主题 | 类型 | 说明 |
|------|------|------|
| `/perception/detection/front_left` | `DetectionArray` | 前视左目检测结果 |
| `/perception/detection/front_right` | `DetectionArray` | 前视右目检测结果 |
| `/perception/detection/down_left` | `DetectionArray` | 下视左目检测结果 |
| `/perception/detection/down_right` | `DetectionArray` | 下视右目检测结果 |

vision 不再发布 `/perception/image/*` 或 `/perception/annotated/*` 图像话题。
实机模式下图像始终留在进程内，通过本地 MJPEG 源交给 go2rtc，避免大图像进入 DDS。

go2rtc 视频流（默认端口 `1984`）：

| 流名称 | 说明 |
|------|------|
| `front` / `down` | 前视/下视原始拼接视频 |
| `front_annotated` / `down_annotated` | 前视/下视 YOLO 识别后带框拼接视频 |

### 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `model_path` | `str` | `""` | YOLO 模型 .pt 文件路径。为空时自动搜索默认路径 |
| `sim_mode` | `bool` | `false` | `true` 从 `/auv/*/stitched` 取图；`false` 从 V4L2 直接取图 |
| `enable_gortc` | `bool` | `true` | 启用本地 MJPEG 服务和 go2rtc |
| `mjpeg_port` | `int` | `8090` | vision 本地 MJPEG 端口 |
| `gortc_http_port` | `int` | `1984` | go2rtc HTTP/WebRTC 端口 |
| `stream_annotated` | `bool` | `true` | 是否通过 go2rtc 转发带检测框的视频 |

### 图像拆分

拼接图由 sim_bridge 通过 `np.hstack` 水平拼接左右目得到（前视 1280×960×2 → 2560×960，
下视 1920×1080×2 → 3840×1080）。vision 从正中间切开：

```python
h, w = cv_img.shape[:2]
mid = w // 2
left_img = cv_img[:, :mid]    # 前视 1280×960 / 下视 1920×1080
right_img = cv_img[:, mid:]   # 同上
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

### 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `max_history` | `int` | `30` | 每条射线历史最大保留数量 |
| `angle_match_threshold` | `float` | `50.0` | 多实例关联角度阈值（度），射线方向夹角小于此值才归入同一 `instance_id` |

位姿历史缓冲：保留 `2.0s`（30Hz 上限 150 条），为编译期常量，不是 ROS 参数。

### 相机参数

`position.py` 的 `CAM_PARAMS` 与仿真场景 `xunyun_fixed.scn` 一致，且分辨率已与**实机实测录像**对齐；
`fx` 由 HFOV 现算（`fx = width / (2·tan(HFOV/2))`，假定方形像素即 `fy = fx`）：

| 项 | 前视 (front_L/R) | 下视 (down_L/R) |
|------|------|------|
| 分辨率 (单目) | 1280×960 | **1920×1080** |
| 拼接后 stitched | 2560×960 | **3840×1080** |
| 水平视场角 (HFOV) | 57.19° | 87.19° |
| 焦距 fx=fy（现算） | 1174.09 | **1008.40** |
| 光心 cx, cy | (640, 480) | **(960, 540)** |
| 机体安装偏移 (x, y, z) | (0.23, ∓0.05, 0.076) m | (-0.13, ∓0.05, 0.0645) m |

`optical_to_body` 前视与下视各一套（见源码 CAM_PARAMS）。

**分辨率来源**：实机录像实测（`/home/origin/recordings`，2026-09-12）——front 推流 2560×960、
down 推流 3840×1080，均为左右目并排，故单目分别为 1280×960 与 1920×1080。
仿真场景 `xunyun_fixed.scn` 的四路相机已按此改为 front 1280×960、down 1920×1080。
Stonefish 的 `horizontal_fov` **严格作用于宽度**，垂直视场由宽高比推出
（`OpenGLRealCamera`: `fovy = 2·atan(height/width · tan(fovx/2))`），所以改宽度不影响水平视场。

> ⚠️ **去畸变当前为空**：换了新摄像头后 `vision.py` 的
> `FRONT/DOWN_CAMERA_MATRIX` 与 `*_DIST_COEFFS` 已清空（未标定），
> 实机下会跳过去畸变并打印一次告警；仿真模式下 Stonefish 渲染本身无畸变，
> 因此整条链路目前都不做去畸变，像素坐标直接进入上述针孔模型。
> 旧的 `config/*.npz`（Girona 时代标定）已删除，不可再用。

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
v_body = optical_to_body @ v_cam          # 相机系 → 机体 NED
v_world = R_robot(roll, pitch, yaw) @ v_body   # 机体 NED → 世界 NED
ray_origin = robot_pos + R_robot @ camera_offset
```

`R_robot` 为完整 3D 欧拉旋转矩阵（`_euler_to_rotation_matrix`）。
姿态取自**与检测帧时间戳对齐**的历史位姿（见下节「时间戳对齐」），不再是「当前」姿态。

### 时间戳对齐

PoseInfo 的 `stamp` 是 ZIT6 位姿**到达时刻**，position 用它做二分查找：

```
position._pose_buffer (deque, 2.0s / 150 条) ← PoseInfo 入队
检测帧到达 (header.stamp) → _lookup_pose() → bisect 取最近位姿
                          → 用对齐后的 roll/pitch/yaw 算射线
```

避免了「用当前姿态处理历史检测帧」在运动中的定位偏差。

**3. 射线累积与剪枝**

- 按 `(class_id, camera_pair, instance_id)` 分组维护射线历史（最多 `max_history`，默认 30 条）
- 新射线与上一条的基线距离须 ≥ `MIN_BASELINE=0.1m` 才加入（保证足够视差）
- 超限时按冗余度评分剪枝：高余弦相似度 + 近距离的射线优先删除（评分含像素权重惩罚）

**4. 像素权重**

检测框中心越靠画面边缘，射线方向越不可靠，按高斯权重降权：

```
weight = exp(-0.5 * (dist / sigma)^2)   # 下限 0.1
```

权重参与后续 WLS 交汇与剪枝评分。

**5. 配对交会（主方法）**

对历史中所有射线对求公垂线中点，经过多重质量过滤：

| 过滤条件 | 阈值 |
|----------|------|
| 近线平行 | cos(angle) < 0.9995 |
| 前向交点 | t1 > 0, t2 > 0 |
| 垂距 | < 5.0m |

对权重取 `w1 × w2`，先用中位数粗筛，剔除偏离中位数 > 1.5m 的离群点，再对 inlier 做加权平均。

**6. 加权最小二乘（WLS）回退**

若配对法无有效解，用 ≥ 3 条射线解加权超定线性方程组：

```
A += w · (I - d_i·d_i^T) ;  b += w · (I - d_i·d_i^T) · o_i
```

条件数 < 1e5 才接受解。

**7. EMA 平滑**

仅对**已存在**的物体做位置平滑，抑制抖动：

```
new_pos = old_pos * 0.7 + position * 0.3
```

**8. 位置验证**

- 物体距离 > `MAX_DISTANCE=50m` 时丢弃
- 验证通过的位置写入 `object_memory`

### 多实例数据关联

同一画面可能出现多个同类物体（两根箭头、两个三角等）。对 `MULTI_INSTANCE_CLASSES = {arrow, triangle, square}`
启用按 `instance_id` 分组，四阶段关联：

```
Phase 1: 尚无任何实例 → instance 0（冷启动）
Phase 2: 与已有实例位置方向匹配（夹角 < angle_match_threshold）
Phase 3: 填入不完整实例（方向匹配或射线数最少者优先）
Phase 4: 全部实例已满且都不匹配 → 新建实例
```

其余类别（sector / start / basket / aruco_tag）恒为 `instance_id = 0`。

### 物体位置输出

`ObjectPosition` 消息字段：

| 字段 | 含义 |
|------|------|
| `class_id` | 物体类别 ID |
| `instance_id` | 同类物体内的实例序号（0, 1, 2, …），用于区分同画面多个同类目标 |
| `class_name` | 类别名称 |
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
builtin_interfaces/Time stamp    # 位姿对应的时间戳（ZIT6 位姿到达时刻）
float32 origin_x/y/z/yaw         # odom 原点在 map 系中的位姿 (yaw: 度)
float32 robot_x/y/z              # 机器人当前 odom 系位置
float32 robot_roll/pitch/yaw     # 机器人当前姿态 (度)，roll/pitch 用于射线 3D 旋转
float32 target_x/y/z/yaw         # 当前运动目标 odom 系位姿 (yaw: 度)
```

PositionNode 使用 `robot_x/y/z`、`robot_roll/pitch/yaw` 和 `stamp`（时间对齐）做射线计算。

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
ros2 launch uv_sim_bringup sim_bringup.py

# 关闭感知
ros2 launch uv_sim_bringup sim_bringup.py enable_ai:=false

# 开启图像转发 (调试用)
ros2 run uv_perception vision --ros-args -p stream_annotated:=false
```

---

## 下游消费者

| 节点 | 包 | 用途 |
|------|-----|------|
| `NavigatorNode` | `uv_nav` | 将 ObjectPosition 作为障碍物加入 A* 路径规划 (膨胀半径 2m) |
| `TaskRunnerNode` | `uv_task` | 存储 `self.objects`，任务 handler 可引用感知数据做决策 |

---

## 已知局限

1. **新摄像头尚未标定**：`vision.py` 的内参矩阵与畸变系数留空，去畸变被跳过；旧的 `config/*.npz` 已删除
2. 实机内参与 `position.py` 由 HFOV 现算的几何没有交叉校验；`hfov` 目前直接沿用仿真场景值（前 57.19° / 下 87.19°），**实机真实视场角待标定确认**
3. 采集分辨率常量（`*_CAPTURE_RESOLUTION`）只是请求值，设备可能给出别的尺寸；启动时会回读并 WARN
4. 需要 AUV 运动产生视差才能定位（静态悬停时无法更新位置）
5. 最大感知距离 50m，超过此范围的目标被丢弃
6. 多实例关联的 `instance_id` 尚未在实机/多物体场景下验证
7. `self._confidence` 仍是模块常量，未参数化
