# 感知系统

当前图像链路分成四层：

```text
uv_camera
  └─ iceoryx2: youlong/camera/front|down (2560×960 BGR8)
       ├─ uv_perception/object_detector
       ├─ uv_record/record
       └─ uv_stream/camera_streamer
                              └─ MPEG-TS/H.264 stdout → go2rtc exec
                                   ├─ HTTP 播放器、信令、MP4 API :1984
                                   └─ WebRTC 媒体 TCP/UDP :8555
```

ROS 2 / DDS 只承载 CameraInfo、检测、测量、跟踪和 TF，不承载任何图像像素。
相机帧通过 iceoryx2 到达推流器；不存在独立的本地 MJPEG 相机服务器。H.264
显示流由 go2rtc 通过 HTTP 播放器/MSE、WebRTC 和 MP4 API 输出；HTTP API 与 WebRTC
信令使用 1984，WebRTC 媒体使用 8555。当前 H.264 源不提供 MJPEG，RTSP 8554 在应用配置中关闭。
## 包和节点

| 包 | 节点 | 职责 |
|---|---|---|
| `uv_camera` | `uv_camera` | V4L2/Stonefish、双目拼接、时间戳、CameraInfo、iceoryx2 publisher |
| `uv_perception` | `object_detector` | iceoryx2 → 左右目拆分 → YOLO → DetectionArray |
| `uv_perception` | `object_localizer` | DetectionArray + CameraInfo + TF → 几何测量 |
| `uv_perception` | `object_estimator` | 关联、滤波、track 状态和后端扩展 → ObjectTrackArray |
| `uv_record` | `record` | 统一 session 记录；raw 原始帧或 go2rtc 视频可选 |
| `uv_stream` | `camera_streamer` | 显示缩放、抽帧、300 ms 匹配缓存、raw/annotated H264 |

## 目标位置估计

object_localizer 把每次检测转换成检测时刻 odom 坐标系下的单目方位射线；左右目
同一帧中类别相同且两条射线几何一致的检测，还会生成一个瞬时双目三维位置
（FORM_FRONT_STEREO / FORM_DOWN_STEREO）。瞬时位置只用于观测显示，不进入全局射线估计。
object_estimator 按前视/下视和物理类别分别存储单目射线；默认在节点本次运行期间保留全部
观测，observation_pool_size 设为正数时才按条数限制。
候选位置由历史和最新射线对生成，随后用每条观测的角度、位姿、外参、类别锚点误差
及检测置信度计算似然，进行带杂波项的软关联和 Huber 鲁棒位置优化。

每次有新射线时，估计器重新使用池中全部射线拟合候选。单实例目标的新证据即使让
估计位置移动超过轨迹关联距离，也会修正原轨迹；多实例目标则可用当前受支持的
新候选替换名额已满但未匹配的旧轨迹。轨迹的“稳定/暂定”状态由观测支持和位置
协方差决定，不随距上次观测的时间自动变为“过期/丢失”；
last_measurement_stamp 仍提供最后一次观测时间。前视与下视目前各自估计，不会合并为一个位置。RViz 的 perception measurements
MarkerArray 会用洋红色大球显示当前帧的瞬时双目位置（约 0.6 秒），池化估计轨迹
仍由 perception tracks 单独显示。匹配最大射线间距和最小视差角可通过
stereo_max_ray_gap_m、stereo_min_parallax_deg 调整。

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

1. 将 2560×960 等比例缩放为 1280×480（水平、垂直均缩小一半）；
2. 按相机时间戳门限抽帧，输出帧率不超过目标帧率；
3. 按 `front`/`down` 分别保留约 300 ms 缓存；
4. raw 模式直接编码，annotated 模式按检测元数据绘制框后编码。

go2rtc 配置为 `uv_stream/config/go2rtc.yaml`。HTTP 播放器/API 和 WebRTC 信令使用
1984；WebRTC 媒体使用 TCP/UDP 8555。当前可用地址：

```text
http://设备IP:1984/stream.html?src=front
http://设备IP:1984/stream.html?src=down
http://设备IP:1984/stream.html?src=front_annotated
http://设备IP:1984/stream.html?src=down_annotated
http://设备IP:1984/stream.html?src=front&mode=webrtc
http://设备IP:1984/api/stream.mp4?src=front
```

`upper_examples` 录像从 `/api/stream.mp4` 获取 H.264 编码流；预览传输方式不改变
录制输入。go2rtc 的 `/api/stream.mjpeg` 只适用于 MJPEG/JPEG 编码源，本项目的 H.264
流不会由该接口自动转码。RTSP/TCP 8554 已从当前应用配置中关闭。
没有 perception 进程时，`front` 和 `down` 仍然可用；没有匹配检测时，
annotated 流回退为 raw 画面。客户端断开不会阻塞相机、检测或数据集记录。

## 数据集

raw 母版通过统一录制器直接记录，不依赖 go2rtc：

```bash
ros2 run uv_record record --record-mode raw --output-root records/sessions
```

输出 session 下的 `camera/raw/{front,down}` 保存 PNG 与逐帧 JSONL；索引保留源时间戳、
`capture_id`、`stereo_pair_id`、尺寸、stride 和 CameraInfo 版本。1280×960 显示流、三帧抽一帧的流、H264
录像和远程屏幕录制都不能替代正式数据集母版。

## 构建与启动

Compose 会自动初始化 `third_party/iceoryx2` 子模块、构建并安装仓库内官方
iceoryx2 Python binding，再构建两个 ROS 工作空间。`uv_camera`、
`uv_perception`、`uv_stream` 和 `uv_record` 都在同一进程内直接调用它，
不再启动额外 native bridge：

```bash
git submodule update --init --recursive
./scripts/compose_up.sh build auv
./scripts/compose_up.sh run --rm auv sim \
  world:=guoshui_2026/cruise_seeded vehicle:=youlong
```

运行时由 `uv_stream` 启动 go2rtc 与 `camera_streamer`；高带宽原图只存在于
iceoryx2 data plane，HTTP 输出和 WebRTC 只承载编码后的显示流。
`INSTALL_WORKSPACE_AI=false` 可跳过 YOLO 依赖，此时仍可运行原图推流和数据记录。
