# go2rtc 与 iceoryx2 视频流

当前像素数据不经过 ROS 2/DDS 图像话题，也不走旧的本地 MJPEG 相机服务器：

```text
uv_camera.driver（真机 MJPG 无损旋转；仿真 BGR 编码 quality 95）
  └─ uv_image_transport (iceoryx2 Python API)
       └─ iceoryx2 JPEG: youlong/camera/front|down
            ├─ uv_perception 解码一次 BGR 后切分左右眼
            ├─ uv_record raw 直接写入逐帧 JPG
            └─ uv_stream/camera_streamer（选帧后解码 JPEG）
                 └─ stdout MPEG-TS/H.264
                      └─ go2rtc exec
                           ├─ HTTP 播放器、API 和 WebRTC 信令 :1984
                           │    └─ 编码录像 /api/stream.mp4?src=front
                           └─ WebRTC 媒体 TCP/UDP :8555
```

RTSP 输出端口 8554 在当前配置中关闭。它不是这套相机数据链路的输入或录制依赖。

`camera_streamer` 将 2560×960 双目原图等比例缩放为 1280×480 显示流，每只眼输出 640×480，保持原始 4:3 画面比例；并根据相机时间戳估算帧率、丢弃重复或过密帧；时间戳不可用时改用本机到达时间。H.264 编码帧率随输入调整，IDR 间隔约为两秒。标注模式维护约 300 ms 的匹配缓存，另外订阅
`/auv/perception/detections`，按照 `camera_name`、`capture_id` 和
`stereo_pair_id` 将框绘制回左右半幅。没有 perception 时 raw 流仍然可用。

go2rtc 配置位于 `uv_stream/config/go2rtc.yaml`。网页播放器/API 和 WebRTC 信令使用
HTTP 1984；WebRTC 媒体使用 TCP/UDP 8555。常用地址如下：

```text
http://设备IP:1984/stream.html?src=front
http://设备IP:1984/stream.html?src=down
http://设备IP:1984/stream.html?src=front_annotated
http://设备IP:1984/stream.html?src=down_annotated
http://设备IP:1984/stream.html?src=front&mode=webrtc
http://设备IP:1984/api/stream.mp4?src=front
```

`upper_examples` 的两路录像都从 HTTP MP4 API 取 H.264 编码流，因此预览选 WebRTC
或 MSE 不会改变录像数据路径。录像先无损复制 H.264 到临时容器，停止时将首个媒体时间戳归零，再导出 MKV 或 MP4；
若存在音轨，MP4 会转为 AAC。录制器先检查 go2rtc 中是否配置该源，再等待 FFmpeg
收到媒体包，最后用 ffprobe 确认文件含有可解码视频帧。编码器每两秒发送 IDR，新接收端可以较快开始解码。

`uv_record` session recorder 走另一条 HTTP 接口：从 `/api/stream.ts?src=<stream>` 读取
MPEG-TS/H.264。默认 `jpeg` 模式解码为带源帧时间戳映射的 JPEG 帧归档；`ts` 模式重新编码
H.264，并输出 MPEG-TS 分段及 HLS `.m3u8` 清单。两条录制路径均不依赖 RTSP 8554。
go2rtc 的 `/api/stream.mjpeg` 只输出 MJPEG/JPEG 编码源；本项目的 H.264 源不会自动转码为 MJPEG。

实机相机连接具有单路容错：`uv_camera` 为 front 和 down 各启动一个采集线程。启动时某一路
打不开、运行中 `read()` 连续失败或返回无效帧时，只会记录该路错误并按间隔重新打开；另一
路继续发布 iceoryx2 帧。设备重新插入并收到有效帧后，会记录恢复日志并发布恢复状态。
默认无有效帧 5 秒后判定为掉线，每 1 秒重试一次，可通过
`camera_startup_timeout_sec` 和 `camera_reconnect_interval_sec` 调整。

raw 录制使用统一入口并直接保存 iceoryx2 的 JPEG payload 为 JPG，不解码或重新编码，不依赖推流：

```bash
ros2 launch uv_sim_bringup sim.launch.py \
  record_session:=true record_mode:=raw enable_stream:=false
```

go2rtc 录像使用 `record_mode:=go2rtc`。`uv_stream` 发布输出帧 PTS 到源帧时间戳的映射；
`uv_record` 将映射写入归档索引。未找到映射的帧会标记为未对齐，会话状态变为 degraded，
本机接收时间只用于诊断。两种录制方式均记录在统一 session 中。

部署时由 Compose 自动构建官方 Python binding 和 ROS 包：

```bash
git submodule update --init --recursive
./scripts/compose_up.sh build auv
./scripts/compose_up.sh run --rm auv sim \
  world:=guoshui_2026/cruise_seeded vehicle:=youlong
```

只运行视频展示而不启动 Stonefish 时，可以先让 Compose 服务保持空闲，
再在另一个终端启动相机/感知 bringup：

```bash
./scripts/compose_up.sh up -d auv
./scripts/compose_up.sh exec auv ros2 launch uv_bringup real.launch.py
```

仿真侧使用 `sim` 命令；实机侧使用 `YOULONG_RUNTIME=real`，它会额外映射
`/dev/video0`、`/dev/video2` 和默认的 `/dev/ttyUSB0`：

```bash
YOULONG_RUNTIME=real ./scripts/compose_up.sh run --rm auv real \
  enable_hardware:=true
```

如果设备枚举不同，可以覆盖 `CAMERA_FRONT_DEVICE`、`CAMERA_DOWN_DEVICE` 和
`HARDWARE_SERIAL_DEVICE`。

## JPEG 采集及部署

真机明确使用 OpenCV V4L2 + MJPG，并设置 `CAP_PROP_CONVERT_RGB=0`。
读取的压缩帧先通过 libjpeg-turbo `tjTransform` 在 DCT 系数域旋转整幅图像 180°，
恢复原有方向及左/右眼顺序，再以 JPEG 写入 Ice。每路工作线程复用自己的变换句柄，
不生成 BGR，也不逐帧启动子进程。使用 `TJXOPT_PERFECT` 拒绝无法完整变换的 MCU
边缘，不裁剪图像；移除 EXIF 等附加标记以避免查看器再次旋转。默认 2560×960 满足
常见 JPEG MCU 的完整旋转要求。无效 JPEG、尺寸不符或旋转失败按单路掉线逻辑报告和重连。

仿真在拼接 BGR 后编码一次 JPEG（quality 95），不旋转。感知及推流都在消费端解码；
推流仍然编码为 MPEG-TS/H.264，网络侧的编码参数保持不变。主要优化的是 Ice 上的
共享内存数据量及 raw 的 PNG 编码/写入开销，实际收益应在目标设备测量。

Docker 已增加 `libturbojpeg0-dev`；直接运行真机节点时先安装该依赖：

```bash
sudo apt-get install libturbojpeg0-dev
```

Foxy/Python 3.8 与 Jazzy 共用稳定的 TurboJPEG 2.x C API。部署需重建镜像/包并
统一重启相机、感知、录制、推流及 go2rtc exec 消费进程，避免旧消费者收到 JPEG。

## 感知去畸变与推流坐标

相机继续发布方向修正后的原始 JPEG，不执行全图去畸变。感知解码后按每眼 K/D 校正；普通流和 raw JPG 保留原图，标注流用相同标定校正底图后叠框。详见 [感知链路说明](../uv_perception/README.md)。
