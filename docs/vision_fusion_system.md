# 当前视觉融合系统工作原理

> 本文描述当前仓库中的实际实现，主要依据
> [`object_localizer.py`](../workspace_auv/src/uv_camera/uv_camera/object_localizer.py)。
> 文中的“当前”指现行 `uv_camera` 组合节点、`object_localizer` 和
> `sim_bringup.py` 的行为，不以旧版 `position.py` 或历史设计文档为准。

## 1. 系统定位

当前视觉系统的目标是：

1. 从前视、下视相机图像中得到目标检测框；前视门框优先提取稳定图像特征点；
2. 结合左右目相机标定和机器人位姿，把检测转换为 `odom/NED` 坐标系中的三维观测；
3. 用多帧、多视角、协方差和空间聚类抑制偶发误差；
4. 为 GUI、任务和导航分别提供兼容输出、丰富目标状态和原始观测历史。

系统并不是一个“所有相机结果共用一个滤波器”的全局融合器。当前实现有两个独立的定位空间：

```text
前视检测 ──┬─ 左右目双目三角化 ───────────────┐
           └─ 单目射线 + 不同位姿交会 ──────────┤
                                               ├─ 前视观测池
下视检测 ──┬─ 已知高度平面交点 ───────────────┤    └─ 前视实例/滤波状态
           └─ 可选下视双目三角化 ──────────────┤
                                               └─ 下视观测池
                                                    └─ 下视实例/滤波状态
```

前视和下视会通过相同的语义类别命名规则表达同一个物理类别，但目前不会把一个前视目标和一个下视目标直接合并成一个三维 Kalman 状态。两者在消息中通过 `estimate_source=front/down` 区分。

## 2. 数据流总览

### 2.1 仿真模式

```text
Stonefish 左/右目图像
  /sim/front_cam/left/image_color
  /sim/front_cam/right/image_color
  /sim/down_cam/left/image_color
  /sim/down_cam/right/image_color
             │
             ▼
      CameraPassthrough
      左右时间配对、hstack 拼接
             │
             ├─ /auv/front_cam/stitched
             └─ /auv/down_cam/stitched
                         │
                         ▼
             uv_camera::uv_sensor
             ROS Image → BGR numpy
                         │
                         ▼
             内存 FrameGate
             丢弃旧帧，避免堆积
                         │
                         ▼
             uv_camera::uv_ai
             YOLO 左目/右目分别检测
                         │
                         ├─ /perception/detection/front_left
                         ├─ /perception/detection/front_right
                         ├─ /perception/detection/down_left
                         └─ /perception/detection/down_right
```

仿真图像的左右拼接由
[`camera_passthrough.py`](../workspace_sim/src/uv_sim/uv_sim/camera_passthrough.py)
完成。当前仿真前视左右图像允许最多约 `0.12 s` 的时间差，下视允许默认约
`0.04 s` 的时间差。拼接输出使用左图的 header 时间戳，不会把不满足时间条件的左右图像强行混合。
同时发布轻量的 `/auv/{front,down}_cam/stereo_info`，携带原始左右图像时间戳和
`stereo_pair_id`。`uv_camera` 用这个元数据给左右检测分别恢复真实时间，并把同一对
检测写入相同的配对 ID；如果运行的是旧版接口，则退回到拼接图的单一时间戳。

### 2.2 实机模式

实机模式由 `uv_sensor` 直接读取 V4L2 设备，例如前视 `/dev/video2`、下视
`/dev/video0`。读取到的 BGR 帧一方面更新原始预览，另一方面通过内存
`FrameGate` 交给 `uv_ai`，不经过 ROS Image 二次传输，也不在 sensor 与 AI
之间编码 JPEG。

### 2.3 AI 检测

[`ai.py`](../workspace_auv/src/uv_camera/uv_camera/ai.py) 对拼接图像的左半幅和右半幅分别运行 YOLO：

- 左右图像分别发布 `DetectionArray`；
- 每个 `Detection` 包含 `class_id`、置信度、bbox 中心和边界框；
- 前视 `gate_front` 还可通过 `feature_type`、`feature_pixel_x/y` 携带门框开口锚点：
  当前 `auto` 模式优先使用分割轮廓的有向包络中心，其次尝试红色门框管线的中心线范围；
  提取失败时回退到 bbox 中心；
- 前视和下视使用独立的 detector label；
- `xxx_front` 只能进入前视处理，`xxx_down` 只能进入下视处理；
- 没有 `_front` 或 `_down` 后缀的标签不受该相机方向过滤；
- 当前仿真启动默认 AI 推理频率为 `3 Hz`，具体值由 `ai_inference_fps` 控制。

`uv_ai` 还会发布管线 `LineState` 和 ArUco 结果，但它们不参与
`object_localizer` 的静态目标三维位置融合。

## 3. 时间、坐标系和标定

### 3.1 坐标约定

所有定位结果发布在 `odom` 坐标系，并采用项目的 NED 约定：

- `x`：North，前向水平轴，单位 m；
- `y`：East，右向水平轴，单位 m；
- `z`：Down，向下为正，单位 m；
- 角度输入 `PoseInfo` 中为 degree；
- ROS 控制消息中的 yaw 通常为 rad，但这不改变定位节点内部对 `PoseInfo` 的 degree 读取方式。

### 3.2 位姿时间对齐

`object_localizer` 订阅 `/basic_motion/pose_info`，把最近的最多 300 个位姿样本保存到
`_pose_buffer`。对每一帧检测，节点会：

1. 取检测消息 header 的时间戳；
2. 在位姿缓存中查找前后样本；
3. 对位置线性插值，对姿态使用最短路径的 SO(3) 四元数 SLERP；
4. 记录字段中的 yaw 仍使用最短角度差，避免日志显示跨越 ±180° 时跳变；
5. 如果位姿距离检测时间超过 `pose_max_age_sec=0.08 s`，丢弃这次几何处理。

位姿协方差随时间年龄放大。也就是说，位姿不是只作为一个确定性的坐标变换使用，它的不确定度会传递到目标三维协方差中。
仿真左右目如果带有 `stereo_pair_id`，前视/下视几何还会分别按左、右原始采样时间
查找位姿，并用两条各自时刻的世界射线做异步双目三角化；不会因为拼接图只有一个
header 就把运动中的两个相机硬当成同一时刻。旧消息没有配对元数据时，才退回单一时间戳。

### 3.3 双目标定

标定来源由 `calibration_source` 选择：

- `npz`：加载前视、下视各自的 `.npz` 标定文件；
- `sim_camera_info`：从 Stonefish 左右目 `CameraInfo` 和启动参数中的相机安装位姿构造双目模型。

每个双目模型包含：

- 左右相机内参 `K`；
- 左右畸变参数 `D`；
- 左右相对旋转和平移；
- 立体校正矩阵 `R1/R2`；
- 校正后的投影矩阵 `P1/P2`；
- 重投影矩阵 `Q`；
- 双目基线长度。

像素首先通过 `cv2.undistortPoints` 去畸变并映射到校正平面，然后才能用于极线误差、射线和三角化计算。

## 4. `object_localizer` 的处理阶段

一个左右目检测对从接收到发布，依次经过以下阶段：

```text
检测消息接收
    │
    ▼
按前/下视分别进入 pending 队列
    │
    ▼
左右检测按时间配对
    │
    ├─ 配对成功：走双目/平面双目流程
    └─ 超时未配对：走单目回退流程
    │
    ▼
左右检测匹配或单目射线生成
    │
    ▼
几何有效性检查
    │
    ▼
得到三维点或射线
    │
    ├─ 前视：原始 bearing 进入类别级射线观测池
    │       （不先分配给 front track）
    └─ 下视：交点进入对应下视观测池
    │
    ▼
前视候选射线交会 + 几何密度聚类
    │
    ▼
前视软概率关联 + bearing LM/Huber 融合
下视按原有池化聚类和窗口滤波
    │
    ▼
发布目标状态和观测历史
```

这里必须区分四个概念：

| 概念 | 含义 |
|---|---|
| 左右目匹配 | 决定哪一个左目框和哪一个右目框尝试组成一对；它不决定目标实例归属 |
| 几何有效 | 决定该匹配是否能形成有效的三维点或射线 |
| 进入观测池 | 前视原始 bearing 或下视三维点被保存到对应滚动观测池 |
| 生成/更新实例 | 前视池批量分组后形成目标假设并更新其 bearing 后验；下视仍由池聚类决定实例 |

例如，一条前视射线可以已经进入 bearing 池，但还没有足够视角基线形成三维后验；它不会因此被强行绑定到某个已有实例。

### 4.1 观测入池条件速查

| 来源 | 直接进入三维观测池的条件 | 不会直接进入三维观测池的情况 |
|---|---|---|
| 前视双目 | 左右检测先尝试按同类、极线和框形状配对；每个有效左右像素分别转换成 bearing 并入前视射线池；双目交会只用于候选初始化，不额外作为独立 XYZ 因子 | 单个像素无法生成有限射线、位姿过期或标定不可用；左右配对失败时，默认仍分别保留为单目 bearing |
| 前视单目 | 有效像素、位姿和标定可用即可入前视射线池；是否属于哪个物体由后续批量几何聚类和软概率关联决定 | 只有像素/位姿/标定本身无效时才不入池；单条射线、近似平行射线不会被删除，只是不能独立初始化稳定三维位置 |
| 下视 known-height | 类别有目标高度配置；射线和平面正向交点有效；入射程度 ≥ 0.15；左右结果（若都有）差异 ≤ 0.12 m；到机器人距离在 0.05～2 m | `gate` 等忽略类别、未知高度类别、射线近平行、交点在后方、左右结果不一致、距离超限 |
| 下视 stereo | 左右同类框匹配；极线误差 ≤ 8 px；视差 ≥ 2 px；深度在 0.05～2 m；三维点有效 | 未配对、极线误差过大、视差不足、深度越界或三角化失败 |

“进入观测池”仍不等于“已经生成目标实例”：前视池会先保存所有有效 bearing，再由候选交会、几何聚类、软关联和 bearing 后验决定是否产生/更新实例。实例上限只作用于最终发布的假设，不作用于原始射线入池。

## 5. 前视定位

前视有两种观测形式：

1. `FORM_FRONT_STEREO=1`：左右目同一时刻三角化得到三维点；
2. `FORM_FRONT_MULTI_VIEW=2`：单目检测形成射线，机器人移动后由多条射线交会得到三维点。

### 5.1 左右检测如何匹配

`_match_detections()` 的匹配规则如下：

1. 左右 `class_id` 必须完全相同；
2. 对前视 `gate_front` 优先取 `feature_pixel_x/y`，其他类别或旧消息取 bbox 中心，
   再将左右图像特征点去畸变并校正；
3. 计算校正后左右特征点的极线方向误差；前视超过 `8 px` 的候选直接不作为双目匹配；
4. 分别计算两个检测框的宽高比：

   ```text
   left_aspect  = left_width  / left_height
   right_aspect = right_width / right_height
   ```

5. 用两个宽高比的对数差作为框形状代价：

   ```text
   aspect_cost = |log(left_aspect / right_aspect)|
   cost = epipolar_error + 5 × aspect_cost
   ```

6. 所有候选进入允许“未匹配”的全局一对一分配，先最大化匹配数量，再最小化总代价；
   不会让一个低质量右框抢走另一个更合理的匹配。

前视匹配有两个候选级硬门槛：极线误差不能超过
`front_epipolar_error_px=8 px`，左右框宽高比的相互比例不能超过
`front_bbox_aspect_ratio_max=1.6`。除此之外没有额外的总 cost 硬阈值；通过候选门槛后，
仍要由后面的框边缘、三角化和点前向检查决定是否形成三维观测。下视 known-height
为了保留左右独立的平面射线，使用不带这两个前视硬门槛的宽松匹配；下视 stereo
在三角化阶段再执行极线和视差硬检查。

这里的 1.6 不是目标必须接近长方体的假设。它表示左右两个检测框的“框形状比例”最多允许相差约 1.6：

```text
max(left_aspect / right_aspect,
    right_aspect / left_aspect) <= 1.6
```

它只是在判断左右目是否可能看到了同一个目标，不是在判断目标本身是不是规则几何体。

### 5.2 前视双目如何入 bearing 观测池

左右检测的配对仍然需要极线和框形状信息，但这只是“左框和右框是否尝试组成一对”，
不是“这两个框属于哪个世界目标”。完成配对后，左右像素分别生成两条普通
`BearingObservation`，写入同一个类别的前视 bearing 池：

```text
left_t  ──→ bearing observation
right_t ──→ bearing observation
```

如果左右配对失败，默认仍把两个单独有效的检测分别写入 bearing 池。这样极线误差、边缘
框或视差异常不会使原始单目证据消失。双目三角化只可作为候选点 `X0` 的初始化，不能再把
同一对像素得到的 XYZ 当成第三条独立观测。

左右框的宽高比比较仍是：

```text
left_aspect  = left_width  / left_height
right_aspect = right_width / right_height
max(left_aspect / right_aspect,
    right_aspect / left_aspect) <= 1.6
```

它只影响双目配对候选，不影响单目 bearing 是否入池。位姿、标定或像素本身无法生成有限
射线时，才不产生该 bearing。

### 5.3 前视双目不确定度策略

当前仿真和实际经验表明，前视双目在 `0.5～2.5 m` 的距离区间内更可靠。代码采用“区间内信任、区间外降权”的策略：

- `0.5～2.5 m`：不额外放大距离因素；
- 小于 `0.5 m` 或大于 `2.5 m`：保留有限三维点，但放大协方差；
- 范围外的距离放大倍率最多为 `6.0`；
- `front_stereo_trusted_range_only` 默认 `false`，所以不会因为超出该区间而全部丢弃；
- `front_stereo_out_of_range_as_ray` 默认 `false`，也不会仅因为距离超出区间就强制转成射线。

极线误差和视差会参与质量缩放：

- 极线误差越大，协方差越大；
- 视差越小，协方差越大；
- 基础前视双目噪声倍率为 `front_stereo_noise_scale=1.8`。

因此，远距离或小视差观测仍可以帮助观测池和多帧估计，但对最终位置的影响会明显小于高质量观测。

### 5.4 前视双目和多视角的统一 bearing 求解

前视左右目和不同时间的单目观测现在统一为：

```text
left_0, right_0, left_1, right_1, ... → 多条 bearing factor
```

每条观测保存像素、相机、采样时刻、相机位姿、世界射线、bearing 协方差和原始检测 ID。
双目只意味着同一时刻有两个相机中心；机器人移动则提供时间基线。两者不再维护两套最终
位置估计。

每条 bearing 的切平面残差为：

```text
e_i(X) = B_iᵀ · (X - o_i) / ||X - o_i||
```

其中 `B_i` 是观测 bearing 的 3×2 tangent basis。节点用像素协方差传播得到 2×2 bearing
协方差，再使用 Huber 鲁棒 LM 最小化所有观测的加权残差。最终协方差来自鲁棒 Hessian
的逆，而不是多个 XYZ 点的简单平均。

双目 `XYZ`、两射线最近点只用于提供初值。原始像素只贡献一次，避免“像素约束 + 由这些
像素得到的 XYZ”重复计算同一份信息。

对于门框，当前实现不假设目标是长方体，也不把 bbox 形状当作目标形状。检测模型仍
提供 bbox 作为可见范围/边缘保护依据；前视几何优先使用稳定开口锚点：若有分割轮廓，
取其有向包络中心；当前检测模型没有 mask 时，在 bbox 内提取红色管线且要求红色像素
同时覆盖水平、垂直方向，再取管线范围的中心；条件不足时才回退到 bbox 中心。该锚点
会同时用于左右目匹配极线误差、双目三角化、单目射线和统一重投影。检测结果中可通过
`feature_id` 区分来源；它是稳定的几何代表点，不等同于“已经恢复了某个不可见角点”。

### 5.5 双目失败后的单目回退

下列情况会让已经匹配的前视左右对无法形成双目点：

- 边缘框；
- 无效框；
- 框形状比例差异超过 1.6；
- 三角化异常；
- 三维点在相机后方；
- 世界坐标点非有限值；
- 机器人位姿不可用或过期。

前视左右配对失败时，左右检测默认分别保留为单目原始 bearing。两条近似平行的射线不会
作为候选初值的有效基线，但它们仍然留在 bearing 池中，后续可以在新的运动基线下参与
LM 融合。左右未匹配检测也会分别走单目 bearing 流程；不会把派生 XYZ 重复喂给滤波器。
`use_rejected_front_pairs_for_multiview` 保留用于旧启动文件兼容，实时 raw bearing 路径
的目标是保留所有有效单目证据。

## 6. 前视单目多视角融合

### 6.1 射线生成

对于一个前视单目检测，节点使用：

1. 检测中心像素；
2. 左/右相机内参与畸变模型；
3. 左/右相机到机体的外参；
4. 检测时刻的机器人位姿；

生成世界坐标系下的射线：

```text
X(λ) = origin_world + λ × direction_world
```

射线方向会归一化，并根据像素框大小、相机焦距和检测置信度估计角度标准差 `sigma_angle`。

### 6.2 射线是否进入观测池

现在前视射线直接进入按语义类别划分的 `_front_bearing_pool`，不经过
`_associate_front_ray()`，也不受 `25°/32°` 目标关联门槛和前视实例上限影响。
实例上限只在聚类结果准备发布时作为安全上限使用。

```text
raw pixel → world bearing → _front_bearing_pool[class]
                         ↓
              pairwise seed / 几何密度聚类
                         ↓
              soft membership + bearing LM
                         ↓
              _front_tracks / TargetPosition
```

只要像素、位姿和标定能够生成有限射线，射线就可以入池。它暂时没有目标实例编号，
或者同时拥有多个 cluster membership 概率。无法形成新的基线、近似平行或暂时没有
三维交点的射线也保留在池中，不会被删除；它们会等待后续视角提供初始化。

当前的 `front_multi_view_max_rays=20` 只限制每个已发布实例缓存的 rays 数量，不再
限制原始类别池。原始池容量由 `front_observation_pool_size=300` 控制。

### 6.3 射线交会算法

射线两两最近点只用于生成候选初值。设第 `i` 条射线起点为 `o_i`、方向为 `d_i`，
首先用：

```text
P_i = I - d_i d_i^T
```

```text
X0 = (Σ wi Pi)^-1 Σ wi Pi oi
```

初始化得到的 `X0` 不作为最终观测，而是 bearing LM 的初值。最终估计直接使用每条
原始 bearing 的切平面残差：

```text
di_pred = (X - oi) / ||X - oi||
ei      = Biᵀ di_pred
```

最终目标函数为：

```text
min_X Σ wi · Huber(eiᵀ Ri^-1 ei)
```

其中 `Bi` 是观测 bearing 的 3×2 tangent basis，`Ri` 是由像素协方差传播得到的
2×2 bearing covariance，并叠加当前类别的视角模型误差。LM 迭代同时计算：

```text
Hi = Jiᵀ Ri^-1 Ji
H  = Σ wi Hi
P  = H^-1
```

因此输出的是位置后验近似：

```text
X ~ N(position, covariance)
```

同时保存软关联概率；错误或互相冲突的射线可以进入 clutter/其他 cluster，而不是
强迫当前目标接受它。

求解时使用：

- 检测置信度；
- pixel → bearing 的 2×2 协方差；
- 视角模型误差；
- Huber 鲁棒权重。

多视角交点不使用 `0.05～2 m` 的机器人距离硬限制。`0.5～2.5 m` 仍可作为双目
协方差标定的高可信区间，但不会把范围外的有限 bearing 或候选结果全部丢弃。

```text
front_multi_view_min_angle_deg=5°
```

这个参数只用于避免近似平行射线作为候选种子，不是射线入池门槛。

只有以下情况会让某条原始 bearing 不入池：

- 像素、位姿或标定不可用；
- 射线方向无法归一化；
- 结果包含非有限值。

如果所有射线都近似平行，目标会保持没有三维位置的观测池状态，而不是生成一个
虚假的深度值。

多视角交点没有 `0.05～2 m` 的机器人距离硬限制。即使目标离当前机器人较远，只要射线几何条件良好，也可以进入前视观测池。多视角基础噪声倍率为 `front_multi_view_noise_scale=1.5`。

## 7. 下视定位

### 7.1 默认 `known_height` 模式

仿真启动时通过 `sim_bringup.py` 将下视模式设为 `known_height`。每个物理类别可以配置一个目标场景深度，例如：

| 语义类别 | 默认/仿真目标场景深度 m |
|---|---:|
| `guide_line` | 1.294 |
| `target_rack` | 1.000 |
| `collection_frame` | 0.940 |
| `yellow_golf` | 0.964 |
| `pink_golf` | 0.964 |
| `red_ring` | 0.925 |

仿真中 `down_scene_origin_z_m=0.12`，用于将场景绝对深度转换为局部 `odom` 深度。真实系统如果位姿已经是绝对深度，应将该偏移设为 0。

### 7.2 射线和平面交点

下视检测像素经过标定后形成世界射线：

```text
X(λ) = origin + λ × direction
```

对于平面：

```text
n^T X + c = 0
```

交点参数为：

```text
λ = -(n^T origin + c) / (n^T direction)
```

下视交点必须满足：

1. 射线不接近平面平行；
2. 交点在相机前方；
3. 交点为有限值；
4. 入射程度不低于 `down_min_plane_incidence=0.15`；
5. 最终目标点到当前机器人位姿的距离在 `0.05～2.0 m` 内。

协方差会考虑像素误差、位姿误差、目标高度误差和相机外参误差。

### 7.3 下视左右目配对行为

当左右目都能形成下视平面交点时：

- 比较左右交点距离；
- 差异超过 `down_plane_consistency_m=0.12 m` 时拒绝这次观测；
- 差异合格时使用左目交点作为代表；
- 右目结果只作为一致性检查，不作为第二个独立 Kalman 因子；
- 当前没有将左右平面交点取平均的逻辑。

如果只有一侧检测能形成合法交点，则可以使用单目平面交点。下视 known-height 模式因此不要求每次左右检测都同时存在。

### 7.4 下视可选双目模式

如果 `down_geometry_mode=stereo`，下视不再使用已知高度平面，而是使用下视双目三角化。此时左右配对需要满足通用双目几何条件：

- 极线误差不超过 `front_epipolar_error_px=8.0 px`；
- 视差不小于 `min_disparity_px=2.0 px`；
- 深度在 `0.05～2.0 m` 内；
- 三维点有限且在相机前方。

默认配置仍然优先使用 known-height，因为固定场景下目标高度约束通常比下视远距离双目视差更稳定。

### 7.5 下视忽略类别

默认 `down_ignored_classes=["gate"]`。类别名会先去除 `_front/_down` 后缀，并将 `door` 归一化为 `gate`，所以 `gate_front`、`gate_down`、`door` 等都会归入 gate 语义类别。

## 8. 语义类别和实例空间

### 8.1 语义归一化

原始 detector label 仍然保存在消息的 `class_id/class_name` 中，但关联时会生成物理语义类别：

```text
collection_frame_front → collection_frame
collection_frame_down  → collection_frame
gate_front             → gate
gate_down              → gate
door                   → gate
```

这样可以让不同 detector head 的标签表达同一物理类别。但“语义相同”不等于“前后视状态共享”：

```text
physical_class_name = red_ring
estimate_source      = front  → 前视独立 track
estimate_source      = down   → 下视独立 track
```

### 8.2 实例上限

当前默认实例上限为：

| 语义类别 | 最大实例数 |
|---|---:|
| 普通类别 | 1 |
| `guide_line` | 6 |
| `gate` | 4 |

射线或三维点找不到合适已有 track 时会尝试新建实例。达到上限后：

- 前视射线通常被拒绝并计入 `instance_limit_rejected`；
- 前视三维观测仍可以进入前视观测池，但可能暂时没有实例；
- 下视有效三维观测先进入下视观测池，之后由聚类决定是否能生成实例。

## 9. 前视和下视观测池

### 9.1 池的定义

节点维护两个完全独立的观测体系：下视三维观测池，以及前视原始 bearing 观测池。
前视实例只保存聚类结果和成员缓存：

```python
_front_bearing_pool:     semantic_class -> deque[RayObservation]
# 旧版直接 XYZ 调用的兼容池，不是实时 raw bearing 的主路径
_front_observation_pool: semantic_class -> deque[FrontPositionObservation]
_down_observation_pool:  semantic_class -> deque[DownDirectObservation]
```

前视 bearing 池默认每个语义类别保存 `300` 条原始观测。它保存射线原点、方向、像素和
协方差，而不是先验分配后的三维点：

- 前视双目的左、右像素分别进入同一个 bearing 池；
- 前视单目像素直接进入 bearing 池；
- 左右配对失败不会自动删除两个仍然有效的单目 bearing；
- bearing 只有在批量聚类和 LM 求解后才形成前视实例位置；
- 下视有效平面交点或下视双目点可以进下视池。

### 9.2 前视几何聚类与软关联

前视不再使用 K-means，也不在观测到达时按射线方向关联已有 track。每次类别池更新后，
节点执行：

1. 使用有足够视差的射线对生成最近点/中点候选；
2. 使用空间哈希和候选点距离组成几何初始簇；
3. 对所有池内射线计算每个候选位置的 bearing 似然；
4. 使用带 clutter 分量的 soft membership，而不是一次性硬分配；
5. 对每个簇执行 Huber/LM bearing 优化；
6. 仅把通过秩、支持度和前向深度检查的后验提升为前视实例。

一条观测可以暂时属于多个候选簇：

```text
observation_i → {cluster_0: 0.72, cluster_1: 0.19, clutter: 0.09}
```

`25°/32°` 不再参与前视 raw bearing 入池。`front_bearing_track_match_distance_m` 只用于
聚类完成后把新的后验簇映射到旧的 instance ID，避免 ID 跳变，不会阻止观测参与其他簇。

### 9.3 近距离簇合并

几何候选中心如果在 bearing 误差尺度内接近，会被初始聚类合并：

- 普通类别默认使用 `front_bearing_cluster_radius_m=0.35 m`；
- `gate` 至少使用 `front_gate_duplicate_merge_distance_m=0.50 m`；
- 这个半径只用于生成初始候选簇，不会删除 raw bearing。

这一步用于防止检测抖动被误分成多个物理目标。

## 10. 实例状态和滑动窗口滤波

### 10.1 前视实例

前视 track 不参与原始 bearing 的入池关联。前视池每次批量重建候选簇，并对每个簇执行
soft membership 和鲁棒 bearing LM。只有一个簇具有至少两条有效观测、满秩法方程和前向
几何时才生成三维位置；观测不足时仍留在池中，不创建虚假深度。

前视最终状态不是“多个 XYZ 点的 Kalman 平均”，而是：

```text
X ~ N(position, covariance)
```

其中 `covariance` 来自 bearing Hessian 的逆，并叠加类别模型误差下限。独立观测会增加
信息、降低随机误差；共同的标定/位姿误差不会因为观测数量变多而消失。Huber 权重使
视角变化造成的异常 bbox 中心成为低权重参考，而不是直接把整个目标状态推走。

聚类完成后才执行 cluster-to-track 的位置匹配，以保持 instance ID 连续。这个匹配只
作用于已经形成的后验簇，不会用 `25°/32°` 阻止任何 raw bearing 参与其他簇。一次聚类
未命中旧 track 时，旧 track 不立即删除；长时间无新观测后才通过年龄变为 `STALE`。

### 10.2 下视实例

下视每次把有效观测放入对应语义类别的池，然后重新聚类并更新实例：

- 每个簇绑定一个下视实例 ID；
- 每个实例使用该簇最近 `50` 条观测做 Kalman 窗口滤波；
- 当下视可靠观测重新锚定一个原先只有前视证据的假设时，会清理旧的前视-only 证据；
- 当前一次聚类未选中的已有下视定位 track 不会立即被移除，而是保留并按正常年龄规则变为 `STALE`；只有明确的重复下视目标合并才会删除 loser track；
- 发布前还会按 N/E 距离执行一次重复下视 track 合并；
- 合并时保留较小 instance ID，并优先复制下视证据更强的状态。

### 10.3 当前代码中的直接下视队列说明

代码中保留了 `_accept_down_direct_in_window()` 这一类旧的下视窗口门控辅助函数，但当前正常的 `FORM_DOWN_DIRECT` 路径会先把有效点写入下视观测池，再执行下视类别重聚类。因此说明当前行为时，应以“下视池 + 重聚类 + 每簇 50 点滤波”为准，而不能把该辅助函数当作唯一的实际入池门槛。

## 11. 不确定度和置信度

### 11.1 协方差来源

前视 bearing 估计的三维协方差来自 bearing LM 的鲁棒 Hessian：

```text
P ≈ (Σ Jiᵀ Ri^-1 Ji)^-1
```

其中 `Ri` 由像素协方差经过 pixel → bearing 的 Jacobian 传播得到，并加入
`front_ray_model_sigma_m` 或 `front_gate_model_sigma_m` 对应的视角模型误差。当前 V1
按设计把采集时的机器人位姿和相机外参视为准确值；`front_bearing_include_geometry_uncertainty`
默认关闭，待 bearing 模型验证后再接入共同位姿/外参误差。

因此，前视协方差保留各向异性：单方向观测通常沿视线深度方向较大；不同相机位置和
不同机器人姿态提供互补约束后，Hessian 的最小特征值会增大，误差椭球可能变得更圆。
视角模型误差作为每条观测的噪声参与融合，不会因为“不是规则物体”就删除该观测。

下视 known-height 仍然通过数值 Jacobian 传播像素、位姿、外参和目标高度误差；它与
前视 bearing estimator 目前保持独立。

主要默认误差参数如下：

| 参数 | 默认值 | 作用 |
|---|---:|---|
| `pixel_sigma_fraction` | 0.08 | 像素标准差约为框尺寸的比例 |
| `pixel_sigma_min_px` | 1.0 px | 像素误差下限 |
| `pixel_sigma_max_px` | 12.0 px | 像素误差上限 |
| `calibration_pixel_sigma_px` | 0.5 px | 标定像素误差 |
| `pose_position_sigma_m` | 0.03 m | 位姿位置标准差 |
| `pose_angle_sigma_deg` | 1.0° | 位姿姿态标准差 |
| `extrinsic_position_sigma_m` | 0.005 m | 相机安装位置误差 |
| `extrinsic_angle_sigma_deg` | 0.5° | 相机安装姿态误差 |
| `shared_error_scale` | 1.0 | 跨帧共同几何误差底座倍率 |

检测置信度越低，像素误差也会被放大。最终不同观测形式还会乘以各自的噪声倍率：

| 观测形式 | 默认倍率 |
|---|---:|
| 前视双目 | `front_stereo_noise_scale=1.8` |
| 前视多视角 | `front_multi_view_noise_scale=1.5` |
| 下视直接观测 | `down_direct_noise_scale=0.75` |

`shared_error_scale=1.0` 控制跨帧共同误差底座。位姿、相机安装外参、下视已知高度/
平面参考等误差不会被当作完全独立的每帧噪声；窗口滤波会保留一个不随观测数量无限
下降的协方差底座。它不是完整的联合 SLAM 协方差，但能避免“看得越久置信度反而
虚高”。设为 `0` 可关闭这层保守底座，实战建议保留默认值。

### 11.2 目标置信度

目标发布置信度使用当前 track 的最大检测置信度和协方差迹：

```text
geometry_confidence = exp(-trace(covariance) / 0.25)
confidence = clip(
    max(last_detection_confidence, 0.05) × geometry_confidence,
    0, 1)
```

因此，观测数量多不一定意味着置信度高；如果几何协方差很大，最终 confidence 仍然会比较低。

### 11.3 状态判定

`TargetPosition.status` 的判定是：

| 状态 | 条件 |
|---|---|
| `UNINITIALIZED` | 没有位置或协方差 |
| `ESTIMATING` | 有位置，但未达到稳定条件 |
| `STABLE` | 观测数不少于 `minimum_stable_observations=2`，且协方差迹不超过 `stable_covariance_trace_m2=0.04` |
| `STALE` | 最近观测年龄超过 `track_timeout_sec=2.0` |

前视 gate 还有额外发布条件：观测数至少 `3`，且置信度不低于
`front_min_publish_confidence=0.15`。不满足时，前视 gate 不进入
`/perception/target_positions`，但相关观测仍可在观测池/历史中看到。

## 12. ROS 接口

### 12.1 输入话题

| 话题 | 类型 | 用途 |
|---|---|---|
| `/perception/detection/front_left` | `uv_msgs/DetectionArray` | 前视左目检测 |
| `/perception/detection/front_right` | `uv_msgs/DetectionArray` | 前视右目检测 |
| `/perception/detection/down_left` | `uv_msgs/DetectionArray` | 下视左目检测 |
| `/perception/detection/down_right` | `uv_msgs/DetectionArray` | 下视右目检测 |
| `/basic_motion/pose_info` | `uv_msgs/PoseInfo` | 检测时刻的机器人位姿 |
| `/sim/front_cam/left/camera_info` | `sensor_msgs/CameraInfo` | 仿真前视左目标定输入 |
| `/sim/front_cam/right/camera_info` | `sensor_msgs/CameraInfo` | 仿真前视右目标定输入 |
| `/sim/down_cam/left/camera_info` | `sensor_msgs/CameraInfo` | 仿真下视左目标定输入 |
| `/sim/down_cam/right/camera_info` | `sensor_msgs/CameraInfo` | 仿真下视右目标定输入 |

`object_localizer` 不直接消费拼接图像，它消费 AI 已经按左、右目拆开的检测元数据。

### 12.2 仿真图像话题

| 话题 | 类型 | 用途 |
|---|---|---|
| `/sim/front_cam/left/image_color` | `sensor_msgs/Image` | Stonefish 前视左目原图 |
| `/sim/front_cam/right/image_color` | `sensor_msgs/Image` | Stonefish 前视右目原图 |
| `/sim/down_cam/left/image_color` | `sensor_msgs/Image` | Stonefish 下视左目原图 |
| `/sim/down_cam/right/image_color` | `sensor_msgs/Image` | Stonefish 下视右目原图 |
| `/auv/front_cam/stitched` | `sensor_msgs/Image` | 前视左右拼接图，供 `uv_camera` 使用 |
| `/auv/down_cam/stitched` | `sensor_msgs/Image` | 下视左右拼接图，供 `uv_camera` 使用 |
| `/auv/front_cam/stereo_info` | `uv_msgs/StereoFrameInfo` | 仿真前视左右真实采集时间和配对 ID |
| `/auv/down_cam/stereo_info` | `uv_msgs/StereoFrameInfo` | 仿真下视左右真实采集时间和配对 ID |

### 12.3 输出话题

| 话题 | 类型 | 内容和使用方 |
|---|---|---|
| `/perception/objects` | `uv_msgs/ObjectPositionArray` | 兼容接口；当前只输出未过期的下视估计，供任务/导航等旧消费者使用 |
| `/perception/target_positions` | `uv_msgs/TargetPositionArray` | 丰富目标状态；包含前视和下视独立估计、协方差、来源、观测计数和状态 |
| `/perception/target_observations` | `uv_msgs/TargetObservationArray` | 最近的直接三维观测和多视角射线历史 |

三个输出的 header frame 都是 `odom`。

### 12.4 观测形式消息语义

| 常量 | 值 | `TargetObservation` 含义 |
|---|---:|---|
| `FORM_FRONT_STEREO` | 1 | 前视左右目三角化的直接三维点，`has_position=true` |
| `FORM_FRONT_MULTI_VIEW` | 2 | 前视单目射线；未交会时 `has_position=false`，交会产生三维点后可带位置 |
| `FORM_DOWN_DIRECT` | 4 | 下视平面交点或可选下视双目直接点，`has_position=true` |

`TargetPosition` 还会发布：

- 原始代表 `class_id/class_name`；
- 归一化后的 `physical_class_name`；
- 所有曾经观察到的 `observed_class_ids`；
- `estimate_source=front/down`；
- `position_covariance`；
- `front_stereo_count`、`front_multi_view_count`、`down_direct_count`；
- `observation_form_mask` 和最近一次 `last_observation_form`；
- 最近观测时间和年龄。

`TargetObservation` 还会发布 `source_raw_observation_ids` 和 `feature_id`，用于判断一个
派生三维点依赖哪些原始图像特征；同一批射线重算时，派生记录可以更新，但不会伪造
新的原始证据。

### 12.5 观测历史中的“未分配实例”

观测池中的三维点在实例聚类完成前，可能还没有稳定 instance ID。此时记录使用
`UNASSIGNED_INSTANCE_ID = 2^32 - 1`。随后前视/下视重聚类会尽量把历史记录 remap 到实际实例 ID。

## 13. 当前关键参数

### 13.1 同步与几何参数

| 参数 | 节点默认值 | 作用 |
|---|---:|---|
| `stereo_sync_slop_sec` | 0.04 s | detection 左右目配对时间容差 |
| `stereo_pending_timeout_sec` | 0.15 s | 未配对检测转入单目回退前的等待时间 |
| `pose_max_age_sec` | 0.08 s | 检测可使用的最大位姿年龄 |
| `front_edge_margin_px` | 8 px | 前视框边缘保护 |
| `front_edge_margin_ratio` | 0.02 | 前视框边缘保护比例 |
| `front_edge_ray_noise_scale` | 3.0 | 边缘截断框保留为单目射线时的角度噪声倍率 |
| `front_bbox_aspect_ratio_max` | 1.6 | 左右框宽高比的相互比例上限 |
| `min_depth_m` | 0.05 m | 三维点最小有效距离 |
| `max_depth_m` | 2.0 m | 下视直接三维点最大距离；不是前视双目的硬上限 |
| `front_epipolar_error_px` | 8 px | 前视候选匹配的极线硬门槛，同时参与已配对点的协方差放大 |
| `min_disparity_px` | 2 px | 下视双目硬条件，以及前视质量缩放参考 |

仿真左右图像的检测消息还带有 `DetectionArray.stereo_pair_id`。非零时，定位器优先
按相同配对 ID 关联左右消息，而不是用两只相机的时间差猜测配对；这允许仿真保留
Stonefish 左右相机的真实渲染时间，同时避免相邻帧串配。旧消息或实机未填写该字段
时仍使用 `stereo_sync_slop_sec` 的时间配对。

### 13.2 前视多视角参数

| 参数 | 默认值 | 作用 |
|---|---:|---|
| `use_rejected_front_pairs_for_multiview` | true | 被拒绝前视双目对是否回退为单目射线 |
| `front_multi_view_max_rays` | 20 | 每个已发布实例缓存的射线数；不限制原始类别池 |
| `front_multi_view_min_angle_deg` | 5° | 仅用于候选初始化时排除近似平行射线 |
| `front_multi_view_max_line_error_m` | 1.0 m | 候选射线对的最近点最大间隙；不删除原始 bearing |
| `front_bearing_seed_max_rays` | 80 | 生成候选种子时使用的最近射线数 |
| `front_bearing_seed_max_pairs` | 2400 | 候选射线对数量上限 |
| `front_bearing_cluster_radius_m` | 0.35 m | 候选交点几何初始聚类半径 |
| `front_bearing_min_cluster_rays` | 2 | 后验簇的最小有效观测支持数 |
| `front_bearing_clutter_likelihood` | 0.08 | soft membership 中的杂波/错误检测分量 |
| `front_bearing_lm_iterations` | 10 | 每个 bearing 簇的 LM 最大迭代次数 |
| `front_bearing_lm_initial_damping` | 1e-3 | LM 初始阻尼 |
| `front_bearing_track_match_distance_m` | 2.0 m | 后验簇映射旧 instance ID 的位置距离 |
| `front_bearing_rebuild_period_sec` | 0.50 s | 合并 raw 到达后执行 batch 重建的周期 |
| `front_bearing_include_geometry_uncertainty` | false | V1 是否加入位姿/外参共同 bearing 误差 |
| `ray_association_angle_deg` | 25° | 旧无 raw 像素兼容函数的参数；不参与实时 bearing 入池 |
| `gate_ray_association_angle_deg` | 32° | 旧无 raw 像素兼容函数的参数；不参与实时 bearing 入池 |
| `ray_gate_chi2` | 16.0 | 旧三维兼容更新的 Mahalanobis 门限 |

### 13.3 观测池、实例和状态参数

| 参数 | 默认值 | 作用 |
|---|---:|---|
| `front_observation_pool_size` | 300 | 前视每个语义类别的滚动池容量 |
| `down_observation_pool_size` | 300 | 下视每个语义类别的滚动池容量 |
| `front_direct_queue_size` | 50 | 前视派生三维点的初始化/稳健窗口长度 |
| `front_raw_observation_window_size` | 100 | 每个前视实例保留的原始图像特征观测数 |
| `down_direct_queue_size` | 50 | 下视每实例滤波窗口长度 |
| `front_duplicate_merge_distance_m` | 0.25 m | 前视普通簇/目标合并半径 |
| `down_duplicate_merge_distance_m` | 0.25 m | 下视普通目标合并半径 |
| `guide_line_min_spacing_m` | 0.5 m | 管线最小间距和合并半径 |
| `front_gate_min_cluster_observations` | 3 | 前视 gate 新簇最小支持数 |
| `front_min_publish_confidence` | 0.15 | 前视 gate 最低发布置信度 |
| `track_timeout_sec` | 2.0 s | track 变为 STALE 的年龄阈值 |
| `minimum_stable_observations` | 2 | 稳定状态最小观测数 |
| `stable_covariance_trace_m2` | 0.04 | 稳定状态协方差迹阈值 |
| `observation_history_size` | 500 | 节点默认观测历史容量 |

### 13.4 仿真启动覆盖值

[`sim_bringup.py`](../workspace_auv/src/uv_bringup/launch/sim_bringup.py) 对定位节点覆盖了部分默认值：

| 参数 | 仿真值 |
|---|---:|
| `calibration_source` | `sim_camera_info` |
| `down_geometry_mode` | `known_height` |
| `down_scene_origin_z_m` | 0.12 |
| `down_target_z_sigma_m` | 0.01 |
| `down_ignored_classes` | `gate` |
| `down_min_plane_incidence` | 0.15 |
| `front_stereo_noise_scale` | 1.8 |
| `front_stereo_trusted_min_range_m` | 0.5 |
| `front_stereo_trusted_max_range_m` | 2.5 |
| `front_stereo_out_of_range_noise_scale` | 6.0 |
| `front_stereo_trusted_range_only` | false |
| `front_stereo_out_of_range_as_ray` | false |
| `use_rejected_front_pairs_for_multiview` | true |
| `front_observation_pool_size` | 300 |
| `down_observation_pool_size` | 300 |
| `front_direct_queue_size` | 50 |
| `down_direct_queue_size` | 50 |
| `publish_period_sec` | 0.2 s |
| `observation_history_size` | 100 |
| `front_gate_min_cluster_observations` | 3 |
| `front_min_publish_confidence` | 0.15 |
| `gate_feature_mode` | `auto` |

这些是“仿真 launch 的实际值”，不是 `object_localizer` 在脱离 launch 单独启动时的所有默认值。

### 13.5 标定、平面和实例命名参数

下面列出不影响日常调参、但会改变坐标解释或实例命名的参数；数组参数按
`object_localizer.py` 中的默认值记录：

| 参数 | 节点默认值 | 作用 |
|---|---:|---|
| `front_calibration_file` | `""` | `npz` 模式的前视标定文件；空值时搜索包内 `config/front.npz` |
| `down_calibration_file` | `""` | `npz` 模式的下视标定文件；空值时搜索包内 `config/down.npz` |
| `front_image_width` / `front_image_height` | `1280` / `960` | 前视检测图像尺寸 |
| `down_image_width` / `down_image_height` | `1280` / `960` | 下视检测图像尺寸 |
| `front_left_camera_info_topic` | `/sim/front_cam/left/camera_info` | 仿真前视左目 CameraInfo |
| `front_right_camera_info_topic` | `/sim/front_cam/right/camera_info` | 仿真前视右目 CameraInfo |
| `down_left_camera_info_topic` | `/sim/down_cam/left/camera_info` | 仿真下视左目 CameraInfo |
| `down_right_camera_info_topic` | `/sim/down_cam/right/camera_info` | 仿真下视右目 CameraInfo |
| `front_left_translation` / `front_right_translation` | `[0.23,-0.05,0.076]` / `[0.23,0.05,0.076]` | 前视左右相机机体外参平移（m） |
| `down_left_translation` / `down_right_translation` | `[-0.13,-0.05,0.0645]` / `[-0.13,0.05,0.0645]` | 下视左右相机机体外参平移（m） |
| `front_left_rotation` / `front_right_rotation` | `[0,0,1,-1,0,0,0,-1,0]` | 前视左右相机机体外参旋转矩阵（行优先） |
| `down_left_rotation` / `down_right_rotation` | `[0,-1,0,1,0,0,0,0,1]` | 下视左右相机机体外参旋转矩阵（行优先） |
| `down_plane_enabled` | `false` | 兼容开关；为 true 时强制选择旧的任意平面模式 |
| `down_plane_normal` / `down_plane_c` | `[0,0,1]` / `0.0` | 任意平面模式的法向量和常数项 |
| `down_plane_sigma_m` | `0.03` | 任意平面参考误差（m） |
| `down_plane_consistency_m` | `0.12` | 左右下视平面交点一致性门槛（m） |
| `down_default_target_z_m` | `1.294` | 未配置类别高度时的兼容默认场景深度 |
| `down_target_z_json` | `""` | 按语义类别覆盖目标场景深度的 JSON 对象 |
| `down_direct_queue_gate_chi2` | `16.0` | 下视窗口的创新门控 |
| `down_direct_reanchor_chi2` | `9.0` | 下视可靠观测重新锚定前视-only 假设的门控 |
| `position_gate_chi2` | `16.0` | 下视/旧直接三维兼容路径的关联门控 |
| `huber_delta` | `2.5` | 前视 bearing LM 和旧兼容更新的 Huber 转折值 |
| `max_instances_default` | `1` | 普通语义类别最大实例数 |
| `max_instances_guide_line` | `6` | `guide_line` 最大实例数 |
| `max_instances_gate` | `4` | `gate` 最大实例数 |
| `class_names` | 见 `DEFAULT_CLASS_NAMES` | 检测类别顺序；必须与训练数据类别 ID 一致 |

## 14. 启动时序

当前仿真启动入口是
[`sim_bringup.py`](../workspace_auv/src/uv_bringup/launch/sim_bringup.py)：

```text
Stonefish
  + sim_bridge
  + basic_motion
        │
        ▼
wait_for_sim(sensors)
        │ 传感器真正开始发布后
        ▼
uv_camera + object_localizer + annotated_preview
        │
        ▼
wait_for_sim(perception)
        │ 前/下视真正产生 AI 检测后
        ▼
可选 navigator + task_runner
```

关键点：

- `object_localizer` 只有在收到有效标定后才能进行几何定位；
- 仿真标定来自四个 `CameraInfo` 话题和启动时配置的相机外参；
- `uv_camera` 和 `object_localizer` 在传感器 readiness 之后启动；
- task/nav 在 perception readiness 之后才启动；
- 预览的原图和带框图像属于显示链路，不等同于定位观测池。

仿真性能相关的默认值目前是：Stonefish `render_quality=low`、显示刷新 `30 Hz`、相机拼接 `5 Hz`、AI 推理每相机 `3 Hz`、PyTorch 推理线程数 `2`。这几个频率影响检测进入定位节点的速度，但不改变定位算法的几何规则。

## 15. 日志和排障

### 15.1 周期摘要

`object_localizer` 会周期打印类似：

```text
localizer: down_tracks=... front_tracks=... \
down_pool={...} front_pool={...} front_bearing_pool={...} \
front_stereo_accepted=... front_stereo_rejected=... \
front_multi_view_rays=... front_multi_view_points=... \
front_bearing_clusters=... front_bearing_optimizations=... \
down_direct_accepted=... down_direct_rejected=...
```

含义如下：

| 计数/字段 | 含义 |
|---|---|
| `down_tracks` | 当前下视实例数，包含状态管理中的目标 |
| `front_tracks` | 当前已经由 bearing 后验生成的前视实例数；原始未成簇射线不占实例槽 |
| `down_pool` | 各下视语义类别当前保留的三维观测数 |
| `front_pool` | 旧直接 XYZ 兼容路径各类别保留的三维观测数 |
| `front_bearing_pool` | 实时前视各语义类别保留的原始 bearing 数；判断射线是否消失应先看这里 |
| `front_stereo_accepted` | 前视双目左右检测完成处理并把左右 bearing 写入池的次数 |
| `front_stereo_rejected` | 前视匹配对无法形成双目点的次数 |
| `front_epipolar_match_rejected` | 前视候选因极线误差超过 8 px 被拒绝的次数 |
| `front_aspect_match_rejected` | 前视候选因左右框宽高比相互比例超过 1.6 被拒绝的次数 |
| `front_bbox_match_rejected` | 前视候选因任一框宽度/高度无效被拒绝的次数 |
| `front_multi_view_rays` | 写入实时前视 bearing 池的原始射线数（兼容路径也会计数） |
| `front_multi_view_points` | batch bearing 后验成功生成前视位置的次数 |
| `front_unified_optimizations` | 前视实例用原始图像特征成功完成统一重投影优化的次数 |
| `front_bearing_pool_added` | 新 raw bearing 写入类别池的次数 |
| `front_bearing_seed_candidates` | 本次累计生成的射线对候选初值数量 |
| `front_bearing_clusters` | batch 聚类后形成的后验簇累计数量 |
| `front_bearing_optimizations` | 成功完成 bearing LM 的簇数量 |
| `front_bearing_noise_observations` | 当前没有形成候选时被视为噪声统计的观测次数；原始 bearing 仍在池中 |
| `front_bearing_rank_deficient` | bearing Hessian 不满秩，暂时不能给出三维后验的次数 |
| `front_bearing_soft_reassignments` | 保留字段；当前 soft membership 由每次 batch 重算 |
| `front_pool_observations` | 旧直接 XYZ 兼容池新建槽的次数 |
| `front_multi_view_angle_rejected` | 旧 track 兼容路径的 5° 基线拒绝次数；不代表实时 raw bearing 被拒绝 |
| `front_cluster_support_rejected` | 旧三维池聚类支持不足，暂未创建实例的次数 |
| `down_direct_accepted` | 下视有效三维观测写入下视池并成功完成实例聚类/关联的次数 |
| `down_direct_rejected` | 下视几何、距离、一致性或实例流程拒绝的次数 |
| `down_direct_queue_rejected` | 旧窗口门控计数；需结合当前池化路径理解，不能单独视为所有下视拒绝来源 |
| `association_rejected` | 已有 track 的 Mahalanobis 更新门控拒绝 |
| `instance_limit_rejected` | 达到类别实例上限，无法新建 track |
| `camera_label_rejected` | 检测 label 的 `_front/_down` 后缀与实际相机不兼容 |

### 15.2 常见现象和定位方向

#### 有射线但没有三维目标

检查：

- `front_multi_view_rays` 是否增加；
- `front_multi_view_points` 是否为 0；
- `front_bearing_pool` 是否增加；
- `front_bearing_seed_candidates` 是否增加；
- `front_bearing_rank_deficient` 是否持续增加；
- 机器人是否移动到足够产生有效几何基线；
- 后验簇是否只有两条边缘/低置信度射线；
- 是否已经达到最终前视实例上限。

#### 前视双目观测数量少

检查：

- 左右 `class_id` 是否一致；
- 框是否接触边缘；
- 框宽度/高度是否为正；
- 两个框的 `width/height` 宽高比是否相差超过 1.6；
- 是否缺少位姿或位姿年龄超过 0.08 s；
- 标定是否成功；
- 三角化点是否在相机前方。

不要仅因为距离超出 0.5～2.5 m 就认为观测一定被丢弃。当前策略是保留有限结果并增大协方差。

#### 观测池有数据但没有目标实例

检查：

- 类别是否为 gate，且前视簇是否少于 3 条；
- 是否已经达到实例上限；
- 几何初始簇是否被 `front_bearing_cluster_radius_m` 合并；
- bearing Hessian 是否不满秩，或软关联是否主要落在 clutter；
- 目标是否被前视发布置信度条件过滤；
- 消费者是否订阅了正确的话题：前视应看 `target_positions`，不能只看兼容的 `objects`。

#### 目标“消失”

先区分三种情况：

1. 目标不再更新但仍在 `target_positions` 中，状态为 `STALE`；
2. 前视 gate 未达到最小支持数或最低置信度，未进入 rich target 输出；
3. 下视 track 仍会保留并转为 `STALE`；只有重复目标合并才会删除 loser，检查
   `down_duplicate_merged` 是否异常增长。

应同时检查 `target_positions`、`target_observations` 和摘要日志，不要只根据 `/perception/objects` 判断视觉系统是否还有观测。

### 15.3 建议的 ROS 检查命令

```bash
ros2 node info /object_localizer
ros2 topic hz /perception/detection/front_left
ros2 topic hz /perception/detection/front_right
ros2 topic echo /perception/target_positions
ros2 topic echo /perception/target_observations
ros2 topic echo /perception/objects
ros2 topic hz /basic_motion/pose_info
```

如果是仿真启动问题，还应检查：

```bash
ros2 topic hz /sim/front_cam/left/image_color
ros2 topic hz /sim/front_cam/right/image_color
ros2 topic hz /auv/front_cam/stitched
ros2 topic hz /auv/down_cam/stitched
ros2 topic echo /sim/front_cam/left/camera_info --once
```

## 16. 当前实现的边界和注意事项

1. 当前系统是静态目标定位器，假设场景目标在世界坐标中基本静止；
2. 前视内部的左右目/跨位姿观测已经统一到同一个目标求解器，但前视和下视仍是独立 track 空间，不是跨相机的统一三维滤波器；
3. 同一物理目标可能同时发布两个 `TargetPosition`，分别标记 `estimate_source=front` 和 `estimate_source=down`；
4. 前视左右框的 1.6 判断是检测框形状一致性，不是目标长方体约束；
5. 前视 0.5～2.5 m 是高可信度区间，不是当前的绝对可见范围；
6. 前视双目没有 2 m 的硬上限，只有非正深度和最小机器人距离等硬条件；
7. 多视角交点没有机器人距离上限，但必须有有效视角基线和前向交点；
8. 原始射线不是三维观测，必须先交会成功才能进入前视三维池；
9. 下视 known-height 依赖类别目标高度配置，未知类别不会自动假设与默认平面相同；
10. 下视左右平面结果当前是“左目代表 + 右目一致性检查”，不是两点平均；
11. `/perception/objects` 是兼容接口，当前只提供未过期下视结果；
12. 预览视频存在并不代表检测或定位观测已经进入池，图像链路和几何链路需要分别检查。
13. bbox-only 模型无法从框本身恢复被遮挡目标的真实物理锚点；当前使用框中心，并把
    框尺寸、检测置信度、边缘截断和几何质量反映到协方差中。需要更准确的目标特定
    锚点时，必须改变消息/标注契约，不能靠假设所有目标都是长方体解决。

## 17. 实战数据集：只需要大框标注

目标定位链路只读取 YOLO 的 `result.boxes`，定位器只需要类别、置信度、框中心和
框尺寸；可选 mask 只用于管线辅助显示/线状态，不参与目标三维定位。因此实战数据集
不需要分割 mask、关键点、姿态标注或 3D 标注；每个可见目标画一个尽量完整的大框即可。
大框的含义是覆盖目标主体，而不是只框一个角或一小块
可见区域。目标被图像边缘截断时仍可用于单目方向搜索，但前视双目会拒绝该框，单目
射线会自动增大角度噪声。

LabelMe 矩形转 YOLO detection 的工具是
[`scripts/labelme2yolo_bbox.py`](../scripts/labelme2yolo_bbox.py)：

```bash
cd /home/doc049/dev/UUV/YouLong_AUV_Control_System
python scripts/labelme2yolo_bbox.py \
  --labelme datas/down_dataset/labelme \
  --image-root datas/down_dataset/images \
  --classes datas/down_dataset/classes.txt \
  --output-root datas/down_bbox_dataset \
  --val-ratio 0.2
```

转换器会保留无目标负样本，支持矩形，并兼容把已有 polygon/circle 转成外接框；默认
用软链接避免重复占用录制图片空间，需要独立拷贝时加 `--copy-images`。如果原始
LabelMe 图像是左右拼接的一张整图，直接增加 `--split-side-by-side`，脚本会生成
`_left`/`_right` 半幅图并自动裁剪、平移大框坐标；运行时 `uv_ai` 始终对左右半幅分别
推理，不能把整幅拼接图中的整图坐标直接当作单目训练坐标。

训练 detection 模型使用
[`scripts/train_bbox.py`](../scripts/train_bbox.py)：

```bash
python scripts/train_bbox.py \
  --weights yolov8n.pt \
  --data datas/down_bbox_dataset/data.yaml \
  --imgsz 640 --epochs 100 --batch 8
```

必须提供 detection checkpoint；训练出的 `best.pt` 可直接作为 `uv_camera` 的
`model_path`。目标检测和三维定位链路没有强制访问 `result.masks`，因此 bbox-only 模型
可以覆盖仿真和实机，只要类别顺序与 `classes.txt`/启动参数保持一致。若任务还要求
`follow_line` 的精确管线方向/中心线，当前 `LineState` 的高精度辅助仍来自可选 mask；
这不影响门、球、框等目标的 bbox-only 定位。

## 18. 与旧版实现的区别

旧版 [`position.py`](../workspace_auv/src/uv_camera/uv_camera/position.py) 是历史定位实现，不能用来推断当前 `object_localizer` 的全部行为。当前节点的主要变化包括：

- 前视和下视分开维护独立实例空间；
- 增加前视双目三角化和多视角射线交会；
- 增加 `0.5～2.5 m` 前视双目可信区间和协方差降权；
- 增加下视 known-height 平面交点；
- 增加按语义类别的 raw bearing 滚动观测池、几何候选聚类和软关联；
- 增加 bearing LM/Huber 后验、Hessian 协方差、实例后验匹配和历史观测消息；
- 增加 `TargetPosition`/`TargetObservation` 丰富接口；
- 保留 `/perception/objects` 作为任务和旧消费者的兼容输出。

因此，调试当前视觉融合时，优先查看：

1. [`object_localizer.py`](../workspace_auv/src/uv_camera/uv_camera/object_localizer.py)；
2. [`ai.py`](../workspace_auv/src/uv_camera/uv_camera/ai.py)；
3. [`camera_passthrough.py`](../workspace_sim/src/uv_sim/uv_sim/camera_passthrough.py)；
4. [`sim_bringup.py`](../workspace_auv/src/uv_bringup/launch/sim_bringup.py)；
5. [`uv_msgs/msg`](../workspace_auv/src/uv_msgs/msg)。
