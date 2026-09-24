# 感知系统

当前图像链路分成四层：

```text
uv_camera
  └─ iceoryx2: youlong/camera/front|down (2560×960 BGR8)
       ├─ uv_perception/object_detector
       ├─ uv_dataset/dataset_recorder
       └─ uv_stream/camera_streamer
                              └─ MPEG-TS/H264 stdout → go2rtc :1984
```

ROS 2 / DDS 只承载 CameraInfo、检测、测量、跟踪和 TF，不承载任何图像像素。
系统不再启用本地 MJPEG/RTSP 中间端口。

## 包和节点

| 包 | 节点 | 职责 |
|---|---|---|
| `uv_camera` | `uv_camera` | V4L2/Stonefish、双目拼接、时间戳、CameraInfo、iceoryx2 publisher |
| `uv_perception` | `object_detector` | iceoryx2 → 左右目拆分 → YOLO → DetectionArray |
| `uv_perception` | `object_localizer` | DetectionArray + CameraInfo + TF → 几何测量 |
| `uv_perception` | `object_estimator` | 关联、滤波、track 状态和后端扩展 → ObjectTrackArray |
| `uv_dataset` | `dataset_recorder` | 直接记录 iceoryx2 原图和帧元数据 |
| `uv_stream` | `camera_streamer` | 显示缩放、抽帧、300 ms 匹配缓存、raw/annotated H264 |

## ROS 接口

```text
/auv/sensors/camera/front/left/camera_info
/auv/sensors/camera/front/right/camera_info
/auv/sensors/camera/downward/left/camera_info
/auv/sensors/camera/downward/right/camera_info
/auv/camera/status

/auv/perception/detections
/auv/perception/measurements
/auv/perception/tracks
/auv/tf
/auv/tf_static
```

`DetectionArray` 的 `camera_name` 为 `front_left`、`front_right`、
`down_left` 或 `down_right`，并带有 `capture_id`、`stereo_pair_id` 和原图时间戳。
因此标注流可以把框准确画回对应的双目半幅。

## 推流

`camera_streamer` 收到原图后：

1. 将 2560×960 缩放为 1280×960；
2. 每三帧保留一帧；
3. 按 `front`/`down` 分别保留约 300 ms 缓存；
4. raw 模式直接编码，annotated 模式按检测元数据绘制框后编码。

go2rtc 配置为 `uv_stream/config/go2rtc.yaml`，只监听 1984：

```text
http://设备IP:1984/stream.html?src=front
http://设备IP:1984/stream.html?src=down
http://设备IP:1984/stream.html?src=front_annotated
http://设备IP:1984/stream.html?src=down_annotated
```

没有 perception 进程时，`front` 和 `down` 仍然可用；没有匹配检测时，
annotated 流回退为 raw 画面。客户端断开不会阻塞相机、检测或数据集记录。

## 数据集

正式母版通过下列命令记录，不依赖 go2rtc：

```bash
ros2 run uv_dataset dataset_recorder --output records/datasets/session_001
```

记录内容为原始分辨率、原始帧率、`capture_id`、`stereo_pair_id`、时间戳、
CameraInfo 版本和 `manifest.jsonl`。1280×960 显示流、三帧抽一帧的流、H264
录像和远程屏幕录制都不能替代正式数据集母版。

## 构建与启动

Compose 会自动初始化 `third_party/iceoryx2` 子模块、构建并安装仓库内官方
iceoryx2 Python binding，再构建两个 ROS 工作空间。`uv_camera`、
`uv_perception`、`uv_stream` 和 `uv_dataset` 都在同一进程内直接调用它，
不再启动额外 native bridge：

```bash
git submodule update --init --recursive
./scripts/compose_up.sh build auv
./scripts/compose_up.sh run --rm auv sim \
  world:=guoshui_2026/cruise_seeded vehicle:=youlong
```

运行时不需要额外 helper、8090 或 8554；高带宽图像仍只存在于
iceoryx2 data plane。`INSTALL_WORKSPACE_AI=false` 可跳过 YOLO 依赖，
此时仍可运行原图推流和数据记录。
