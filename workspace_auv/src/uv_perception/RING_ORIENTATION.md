# 下视 red_ring 图像方向

方向随 `/auv/perception/detections` 中的下视 `red_ring` 发布，由该检测
对应的同一幅单目 BGR 图像计算。CV 模块不依赖 ROS 或 YOLO，也不会发起额外推理。
本版提供方向感知，巡游中的转向、夹取动作后续接入。

## 消息和 Task 边界

`Detection` 新增：

| 字段 | 含义 |
|---|---|
| `orientation_valid` | 本帧有可靠主轴方向 |
| `orientation_axis_deg` | 原始单目图像的主轴角 `[0,180)`，右方 0°、向下 90° |
| `orientation_quality` | PCA 各向异性 `(λ大−λ小)/(λ大+λ小)`，不是 YOLO 置信度 |

方向沿用外层消息的 `header`、`camera_name`、`capture_id` 和 `stereo_pair_id`。
其他类别、前视相机、功能关闭以及方向失败时，三个字段为 `false/0/0`，目标检测仍发布。
门框原有 `feature_type/feature_pixel_*` 的意义独立于这些方向字段。

Task 从现有订阅选中目标后检查类别、方向有效性和本帧新鲜度，只消费数据。
无效字段里的零角不表示水平环，也不能拿来执行对齐。图像法向是
`(orientation_axis_deg + 90) % 180`；后续控制层要用相机标定、安装 TF 和实际姿态
转换到抓取 yaw，再处理夹爪的安装偏角。图像角不能直接当作机体 yaw。

此输出是二维投影轴，存在 180° 正反歧义。推算水平法向的适用假设是圆环近似竖直、
相机接近正上方观察；不表达完整三维圆面法向。

## 算法和参数

bbox 中心扩展 1.2 倍 → 两个红色 HSV 区间 → 一次 3×3 闭运算
→ 红色连通区域选择 → PCA 主轴。

选择原 bbox 内红色像素最多的连通区域，并列时选中心最近的区域。扩展区里的
大块背景不能仅凭面积赢得选择。原 bbox 触及单目图像边缘时方向无效；只裁掉
扩展留白时允许继续计算。原图和检测框坐标不会被修改。

默认要求至少 24 个像素、主轴投影 5%–95% 长度至少 12 像素、质量至少 0.8。
近圆形、过短、颜色不足、输入错误或 CV 异常均返回无效结果。

检测节点声明四个启动参数，可通过 ROS 参数文件或 `--ros-args -p` 配置：

| 参数 | 默认值 | 范围 |
|---|---|---|
| `ring_orientation_enabled` | `true` | 布尔值 |
| `ring_roi_scale` | `1.2` | 有限数，≥1 |
| `ring_min_pixels` | `24` | 整数，≥2 |
| `ring_min_quality` | `0.8` | 有限数，0–1 |

HSV 阈值沿用现有红色门管道的两区间：H=0–18 / 165–180，S≥45，V≥30。
参数在启动时读取，实际水下颜色和阈值需要用实拍图验证。

## 构建和切换版本

`Detection` 的消息定义已经改变，所有发布/订阅节点必须使用同一套新消息类型。
在工作空间构建并加载环境后，重新启动检测、定位器、Task，以及订阅检测的
启动管理器、等待节点和视频流节点。构建命令：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to uv_perception uv_task uv_control uv_bringup uv_stream --symlink-install
source install/setup.bash
```

本次隔离构建安装目录为 `/tmp/colcon-ring-20261008-install`，可以加载它来检查消息。
当前工作机没有正在运行的上述节点；实艇的运行进程需在该机器加载新构建后重启。

## 验收和 CPU 耗时

测试覆盖水平、竖直、斜线、0°/180° 边界、两红色 H 区间、1 像素细线、小断裂、
近圆形、邻近红色干扰、ROI 留白裁剪、目标截断、参数和异常。发布测试覆盖左右目
独立计算、同帧元数据、语义类别映射、门框特征、消息序列化和每目一次 YOLO 调用。
ROS 传输测试使用隔离域 185 的合成图像，不连接实艇。

```bash
ROS_LOG_DIR=/tmp/ros-ring-test-log ROS_LOCALHOST_ONLY=1 \
python3 -m pytest -p no:cacheprovider \
  src/uv_perception/test/test_ring_orientation.py \
  src/uv_perception/test/test_detector_ring_orientation.py \
  src/uv_perception/test/test_ring_orientation_ros.py -q
```

2026-10-08 在 AMD Ryzen 9 7945HX、OpenCV 4.13.0、NumPy 2.5.2 上测量。
输入单目 1920×1080、约 500 像素长红色细线；每角度预热 30 次、测量 200 次。
表中只含 CV 方向处理，不含 YOLO、读图或 ROS 发布：

| 角度 | 中位耗时 ms | P95 ms |
|---|---:|---:|
| 0° | 0.363 | 0.388 |
| 30° | 1.728 | 1.953 |
| 45° | 1.643 | 1.823 |
| 90° | 0.543 | 0.578 |
| 135° | 1.641 | 1.808 |

以上不是实艇 NVIDIA 平台的性能承诺，也未验证真实水下图像的方向准确率。
