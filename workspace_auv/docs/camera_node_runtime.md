# `uv_camera` 节点运行逻辑

本文说明当前仿真与实机共用的视觉节点、输入输出接口，以及任务启动时的检查方法。

## 1. 总体数据流

```text
Stonefish 四路相机
  /sim/front_cam/{left,right}/image_color
  /sim/down_cam/{left,right}/image_color
              |
              v
uv_sim/sim_bridge::CameraPassthrough
  左右图时间匹配、横向拼接、发布 StereoFrameInfo
              |
              v
  /auv/front_cam/stitched   /auv/down_cam/stitched
              |
              v
uv_camera::Sensor
  ROS Image -> BGR；更新预览；送入进程内 FrameGate
              |
              v
uv_camera::Ai
  左右图拆分 -> 标定/去畸变 -> YOLO 推理 -> DetectionArray
              |
              +--> /perception/detection/{front_left,front_right,down_left,down_right}
              +--> /perception/line/{front,down}
              +--> /perception/aruco/ids
              |
              v
object_localizer
  结合 CameraInfo、里程计/位姿与检测结果发布目标世界坐标
              |
              v
wait_for_sim_perception / uv_task / 导航任务
```

`uv_camera` 是一个组合节点：`Sensor` 和 `Ai` 在同一个 ROS 进程中运行。图像在二者之间通过内存中的 `FrameGate` 传递，不经过第二次 ROS 图像传输，也不经过 JPEG 编解码。MJPEG/go2rtc 仅用于观察，不参与任务控制。

## 2. 仿真模式

`sim_bridge` 订阅 Stonefish 的四路原始图像。左右图只有在时间戳差值不超过仿真双目窗口时才会组成一帧，避免把相邻时刻的左右图错误配对。

Stonefish 会串行渲染相机，因此对应左右相机的时间戳可能相差约 100 ms。当前 front/down 均使用 `0.12 s` 的仿真匹配窗口；普通同步函数的默认窗口仍为 `0.04 s`，不会改变其它调用方的行为。拼接结果为左图在左、右图在右，`uv_camera` 后续再按宽度一分为二。

仿真模式下 `Sensor` 还会接收 `/auv/{front,down}_cam/stereo_info`。由于图像和元数据使用 BEST_EFFORT QoS，节点会短暂缓存图像，等待元数据到达；元数据缺失时仍可按图像头时间戳使用兼容路径。

## 3. AI 推理与消息

模型由参数 `model_path` 指定，当前任务默认使用：

```text
workspace_auv/src/uv_camera/resource/best.pt
```

类别映射由 `class_mapping_path` 指定，当前为 `weights/robotcup20260901.yaml`。当前模型是 segmentation 模型；检测消息除边界框外还可携带 `mask_x`、`mask_y`，供后续按掩膜提取 SGBM 深度的合理峰值。推理节流由 `inference_fps` 控制，默认值通常为 5 Hz。

AI 输出空的 `DetectionArray` 也属于有效输出，因此“没有检测到目标”和“没有收到图像”需要区分：前者仍会有 detection 消息，后者会使感知就绪检查一直等待。

实机模式不订阅仿真 ROS 图像，而是分别打开前视、下视 V4L2 设备，完成分辨率/FourCC 设置和首帧预检后，再由采集线程送入同一个 `FrameGate`。

## 4. 预览服务

启动时 `uv_camera` 默认创建本地 MJPEG 服务，当前端口为 `8090`，提供 `/front`、`/down` 及 annotated 版本。仓库已准备 `third_party/go2rtc/go2rtc`，启动时会自动转发到 `1984`；如果未下载该二进制，`go2rtc not found` 只表示没有启动外部转发服务，不影响推理和任务执行。

原始流 `/front`、`/down` 只经过图像编码，延迟最低；`/front_annotated`、`/down_annotated` 必须等待 YOLO 分割推理，在 CPU 仿真下可能明显滞后。检查相机传输时应优先观察原始流和 `camera passthrough` 计数，不要用 annotated 流的延迟判断 DDS 图像链路。

## 5. 启动与诊断

建议先重新构建仿真工作空间，确保场景和 `CameraPassthrough` 使用最新源码：

```bash
cd ~/AUV_2026_Robocup/YouLong_AUV_Control_System
source /opt/ros/humble/setup.bash
source workspace_auv/install/setup.bash
cd workspace_sim
colcon build --symlink-install --packages-select stonefish_ros2 uv_sim
source workspace_sim/install/setup.bash
cd ..
```

启动后依次检查四层接口：

```bash
ros2 topic list | grep -E 'sim/.*/image_color|auv/.*/stitched|perception/detection'
ros2 topic hz /sim/front_cam/left/image_color
ros2 topic hz /auv/front_cam/stitched
ros2 topic hz /auv/down_cam/stitched
ros2 topic hz /perception/detection/down_left
ros2 topic echo /perception/detection/down_left --once
```

判断方法：

1. 原始 `/sim/.../image_color` 没有频率：检查 Stonefish 场景相机配置。
2. 原始图像有频率、`/auv/.../stitched` 没有频率：检查左右时间戳和双目匹配窗口。
3. stitched 有频率、detection 没有频率：检查 `uv_camera` 订阅、模型加载、推理异常和 `enable_ai` 参数。
4. detection 有频率但 `wait_for_sim_perception` 仍等待：继续检查 `object_localizer` 的标定、位姿和 `target_positions`。

启动日志中的 `uv_sensor started (sim mode: ROS stitched topics)` 与 `YOLO model loaded` 只能证明节点初始化成功，不能证明已经收到图像；最终应以 stitched 和 detection 的实际消息频率为准。

`sim_bridge` 每 5 秒还会输出一次 `camera passthrough: raw callbacks ...; stitched=...`。如果 raw 计数为 0，是 Stonefish 原始图像/DDS 链路问题；如果 raw 增长而 stitched 不增长，是左右时间戳匹配问题；如果 stitched 增长而 detection 不增长，则继续检查 `uv_camera` 的 FrameGate、模型推理或 Python 异常。
