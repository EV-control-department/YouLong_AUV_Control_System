# go2rtc 与 iceoryx2 视频展示

新的图像链路不经过 DDS 图像话题或本地中间端口：

```text
uv_camera
  └─ iceoryx2: youlong/camera/front|down
       └─ camera_streamer
            └─ stdout MPEG-TS/H264
                 └─ go2rtc exec
                      └─ HTTP/WebRTC :1984
```

`camera_streamer` 从 2560×960 原图生成 1280×960 显示流，每三帧保留一帧，
并维护约 300 ms 的匹配缓存。标注进程另外订阅
`/auv/perception/detections`，按照 `camera_name`、`capture_id` 和
`stereo_pair_id` 将框绘制回左右半幅。没有 perception 时 raw 流仍然可用。

go2rtc 配置位于 `uv_stream/config/go2rtc.yaml`，只有 1984 对外监听：

```text
http://设备IP:1984/stream.html?src=front
http://设备IP:1984/stream.html?src=down
http://设备IP:1984/stream.html?src=front_annotated
http://设备IP:1984/stream.html?src=down_annotated
```

正式数据集由 `uv_dataset/dataset_recorder` 直接记录 iceoryx2 原图，不能
使用显示缩放、三帧抽一帧或 H264 录像作为数据集母版。

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
