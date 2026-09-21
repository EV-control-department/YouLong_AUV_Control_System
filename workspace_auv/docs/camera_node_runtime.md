# `uv_camera` 节点运行逻辑

本文说明当前仿真与实机共用的视觉节点、输入输出接口，以及任务启动时的检查方法。

## 2026-09-18 低帧率排查记录（尚未完全解决）

## DDS 图像边界检查

当前实现尚未满足“DDS 不承载图像、上位机只取推流”的目标。仿真模式的实际链路是：

```text
Stonefish
  -> /sim/front_cam/{left,right}/image_color   (DDS sensor_msgs/Image)
  -> sim_bridge
  -> /auv/{front,down}_cam/stitched            (DDS sensor_msgs/Image)
  -> uv_camera / mapping task                  (DDS sensor_msgs/Image)
```

Stonefish 在 `ROS2ScenarioParser.cpp` 中通过 `image_transport::advertise()` 创建
`/sim/.../image_color` 发布者；`ROS2SimulationManager::ColorCameraImageReady()`
填充并发布 `sensor_msgs/msg/Image`。`camera_passthrough.py` 又将四路输入拼接后
发布 `/auv/front_cam/stitched` 和 `/auv/down_cam/stitched`。因此这些话题都会占用
DDS 带宽，不能称为“只在进程内传输”。

当前两个上位机程序也仍直接订阅 DDS 图像：

- `visualization/mapping_visualizer.py` 订阅 `/auv/down_cam/stitched`；
- `visualization/auv_visualizer.py` 订阅 `/perception/annotated/front_left`、
  `/perception/annotated/front_right`、`/perception/annotated/down_left`、
  `/perception/annotated/down_right`。

`uv_camera` 自身的上位机预览才是推流链路：进程内 JPEG 缓存 -> 本地 MJPEG
`8090` -> go2rtc HTTP/WebRTC `1984`。`annotated_preview.py` 使用 MJPEG；但两个
PySide 上位机尚未切换到该链路。当前只能说“预览服务已具备”，不能说“上位机已只用推流”。

检查命令（必须与仿真使用同一个 `ROS_DOMAIN_ID`）：

```bash
ros2 topic list --no-daemon | grep -E '(/sim/.*/image|/auv/.*/stitched|/perception/annotated)'
ros2 topic info -v /sim/down_cam/left/image_color
ros2 topic info -v /auv/down_cam/stitched
curl -I http://127.0.0.1:8090/down
curl http://127.0.0.1:1984/api/streams
```

在没有运行仿真时，`ros2 topic` 没有输出是正常的；源码检查仍可确认发布者存在。
要真正实现 DDS 零图像，需要把 Stonefish 到 `uv_camera` 的仿真输入从 ROS Image
改为共享内存/本地 socket 或在同一进程内直连，不能仅修改上位机订阅者。当前任务
控制和建图仍依赖 `/sim`、`/auv` 图像话题，贸然关闭发布会使 AI 和 SGBM 失去输入。

### GitHub 历史结论

远程仓库历史中最容易混淆的是提交 `a99aa25`（2026-05-18）。它的提交说明是
“彻底剥离 image 图像的 dds 下发”，实际改动范围是 `uv_perception/vision.py`：
删除视觉节点发布 `/perception/image/*` 和 `/perception/annotated/*` 的 ROS Image
发布者，实机 V4L2 帧改为进程内处理并通过 MJPEG/go2rtc 输出。

但是该提交的父版本已经保留了仿真链路：Stonefish 发布 `/sim/.../image_color`，
`sim_bridge` 订阅这些图像并发布 `/auv/*/stitched`；该部分没有被 `a99aa25` 删除。
因此“图像不再经 DDS”从未对仿真模式全局成立。

当前远程 `robocup_enbody` 的最新提交 `8495831`（2026-09-18）仅改造建图上位机，
并新增了对 `/auv/down_cam/stitched` 的 DDS Image 订阅；它没有移除 Stonefish 的
图像发布者，也没有把该上位机切换到 MJPEG/go2rtc。到目前为止没有找到一个后续提交
真正完成仿真图像的 DDS 零传输改造。

相关提交：

- `a99aa25`：实机视觉输出改为 MJPEG/go2rtc，仿真 DDS 输入仍保留；
- `972c816`（2026-06-14）：工作空间合并后继续保留 Stonefish 图像 DDS 发布；
- `8495831`（2026-09-18）：当前 `robocup_enbody` 上位机改造，仍订阅 DDS 图像。

在独立 ROS 域运行当前随机场景，NVIDIA 渲染、AI 开启、任务及预览关闭，
已复现低帧率；因此任务后台建图与 go2rtc 并非该次复现的必要条件。

已修复 `common.bgr_to_image_msg` 的大图消息赋值开销：
原先将 `bytes` 赋给 ROS `Image.data`，触发 Python 逐元素校验。
7372800 字节的本机微基准赋值约 349 ms，改为 `array('B')` 后赋值约
0.001 ms（不含数组构造）。实际拼接、转换、发布合计从约 350–395 ms
降到约 3–19 ms；消息类型、像素、时间戳与话题保持不变。

但修复后原图接收仍偏低，短测拼接发布约 0–1.2 Hz，camera 接收更低。
YOLO 实际设备为 CPU，预热后单眼调用约 23–36 ms；不能将推理等待图像
误判为推理耗时。`gpu_backend` 只选择 Stonefish OpenGL，不选择 YOLO 设备。
低帧率的剩余原因仍需区分 Stonefish 图像生成、DDS 发布/传输及订阅接收；
物理 real_time_factor 接近 1 不代表摄像机帧率正常。

### 显卡与链路对照实测

在同一随机场景、同一窗口和 100 Hz 物理参数下，分别使用
`gpu_backend:=nvidia` 与 `gpu_backend:=software`，并关闭 AI、任务和预览。
Stonefish 侧新增了相机回调和发布耗时统计，结果如下：

| 环节 | NVIDIA | llvmpipe 软件 OpenGL |
| --- | ---: | ---: |
| Stonefish 下视单眼回调 | 约 10 Hz | 约 10 Hz |
| Stonefish 前视单眼回调 | 约 5 Hz | 约 5 Hz |
| Stonefish 单帧发布调用 | 约 0.8–1.2 ms | 约 0.8–1.2 ms |
| `sim_bridge` 原图接收（前/下，左右合计） | 约 1.2–2.6 / 3.0–5.8 Hz | 约 0.8–2.6 / 1.0–3.2 Hz |
| real-time factor | 约 1.0 | 约 1.0 |

因此显卡驱动不是当前卡顿的主因：硬件和软件渲染的 Stonefish 回调结果
基本一致，且渲染端发布调用没有出现数百毫秒阻塞。问题发生在 Stonefish
发布之后、`sim_bridge` 收到之前，属于大尺寸 ROS Image 的 DDS/`rclpy`
接收链路。每张 1280×960 `bgr8` 图像约 3.69 MB，四路图像合计约
110 MB/s 原始数据；`sim_bridge` 使用单线程 `rclpy.spin()`，同时承载四路
大图反序列化、控制回调和拼接。BEST_EFFORT/depth=1 会丢弃旧帧，表现为
窗口低帧率而不是持续延迟。此前出现的 Fast DDS `Failed init_port` 共享内存
错误也会进一步削弱该链路，但不是 NVIDIA 驱动错误。

当前最有价值的下一步是降低 DDS 图像负载或绕过 DDS：仿真相机应通过共享内存/
本地 socket 直接交给 `uv_camera`，上位机继续只使用 MJPEG/go2rtc；临时验证
可将仿真相机降分辨率或改为压缩传输。仅切换 NVIDIA/llvmpipe、提高 YOLO
线程数或安装 go2rtc 都不能解决 Stonefish 到 `sim_bridge` 的丢帧。

当前每 5 秒限频输出的调试点：

- `相机管线`：前/下原图接收 Hz（每组左右眼合计）、拼接发布 Hz、最近拼接发布耗时。
- `camera输入`：进入 camera 管线的 Hz、解码/预览/入队耗时、图像字节数及采集戳。
- `YOLO管线`：实际推理设备、模型锁等待、推理调用耗时及输入尺寸。

没有使用暂停线程的断点，也没有通过扩大时间同步容差来掩盖丢帧。
camera 输入统计只有收到图像才输出；完全断流时应对照 bridge 的定时日志。

分支对比使用本地存在的 `robotcup`（并非名为 `robocup` 的分支）：
相机仍为四路 1280×960、各 10 Hz，sensor、FrameGate 与原始预览结构基本相同；
主要差异为新分割模型、所有类别掩膜发布、下视配对时间窗。
上述 `bytes` 赋值也存在于 `robotcup`，不能将其归为建图分支独有回归；
队友环境是否使用同一提交、Python 优化选项、DDS、场景和模型仍需核对。

附带修复：显式选择 NVIDIA 时清除遗留 llvmpipe 设置。
验证包括图像非连续数组往返、字节数组类型与双目配对等 23 项测试，
另有 5 项 launch 合约测试通过。短测结束由 timeout 主动发 SIGINT，
退出日志不代表任务运行中崩溃；本次未运行完整建图任务。

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

当前仿真相机的单眼分辨率为 `640x480`，左右拼接话题的尺寸为 `1280x480`。
`uv_camera` 会再次按中线拆成两个 `640x480` 视图，因此 CameraInfo、YOLO
输入和双目几何计算使用的是单眼尺寸。

仿真开发配置将 YOLO 设备设为 `cuda:0`。启动时必须看到类似下面的日志，
才能确认没有回退到 CPU：

```text
YOLO CUDA enabled: NVIDIA GeForce RTX 4060 Laptop GPU (cuda:0)
YOLO管线[down_left] device=cuda:0 ... 输入=640x480
```

当前机器的 PyTorch 环境为 `torch 2.11.0+cu128`、`torchvision 0.26.0+cu128`，
使用 CUDA 12.8 wheel。重新配置运行环境时可执行：

```bash
python3 -m pip install --user --upgrade \
  --index-url https://download.pytorch.org/whl/cu128 \
  torch==2.11.0+cu128 torchvision==0.26.0+cu128
python3 - <<'PY'
import torch
assert torch.cuda.is_available()
print(torch.__version__, torch.version.cuda, torch.cuda.get_device_name(0))
PY
```

没有 CUDA 版 PyTorch 时，代码会记录警告并回退到 CPU；这不会阻止节点启动，
但应视为性能降级而不是正常配置。

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
