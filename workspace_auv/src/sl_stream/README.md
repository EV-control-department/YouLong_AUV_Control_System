# sl_stream 动态 H.265 摄像头推流

`sl_stream` 订阅 `sl_camera` 发布的 `sl_msgs/msg/CameraCatalog`，为每个已启用摄像头创建 `sl_<id>` go2rtc 流。摄像头图像仍由 `sl_camera` 通过 Iceoryx2 发布 JPEG；只有有预览或录像客户端订阅该流时，go2rtc 才启动 H.265 编码进程。最后一个客户端断开后，go2rtc 会回收推流进程。

流保留源分辨率、原始方向及双目拼接布局。默认帧率跟随清单中的实测/协商速率，最高 30 fps；目标码率为每路 2 Mbps，峰值 3 Mbps。Jetson 上自动选择 `nvv4l2h265enc`，没有 Jetson 硬件编码器时使用 GStreamer `x265enc` 软件编码。编码器和清单话题可通过 ROS 参数覆盖。

## 启动

先按 [sl_camera 文档](../sl_camera/README.md) 独立启动采集，再启动视频服务：

```bash
source /opt/ros/jazzy/setup.bash
source workspace_auv/install/setup.bash
ros2 launch sl_stream stream.launch.py
```

常用参数：

```bash
ros2 launch sl_stream stream.launch.py \
  catalog_topic:=/auv/sl_camera/cameras \
  max_fps:=30 bitrate:=2000000 max_bitrate:=3000000 encoder:=auto
```

`encoder` 可选 `auto`、`jetson` 或 `software`。go2rtc 的 HTTP API 默认监听 1984；更改此值请同时修改上位机的 go2rtc 地址。`sl_stream` 与 `uv_stream` 不能同时占用同一 HTTP 端口。

摄像头清单启动后会立即提供对应的 `sl_<id>` 路由。预览和录制地址例如 `http://<艇端地址>:1984/api/stream.ts?src=sl_1`。路由随摄像头启用项动态增删；采集进程重启和清单过期会使旧推流停止。

路由保存在服务独占的临时 go2rtc 配置中。go2rtc 会拒绝通过 REST 接口创建 `exec:` 源，因此摄像头集合、配置代次、Iceoryx2 服务或尺寸变化时，服务会更新该运行时配置并重启自己的 go2rtc 实例。普通在线状态及帧率统计更新不会重启实例。

## 上位机预览

`upper_examples image_viewer` 与主 GUI 自动订阅 `/auv/sl_camera/cameras`，两个预览窗口可分别选择在线摄像头或选择“不预览”。预览从 go2rtc 获取 H.265 MPEG-TS，由上位机的 FFmpeg 本地解码。录制通过 HTTP MP4 接口复制原始 H.265 码流。

上位机需安装 `ffmpeg`。Jetson 自动硬件编码依赖 JetPack 的 GStreamer `nvv4l2h265enc` 与 `nvvidconv` 插件；目标机可运行 `gst-inspect-1.0 nvv4l2h265enc` 检查。软件编码依赖 GStreamer `x265enc` 插件。

构建时 `colcon build` 会自动发现 `workspace_auv/src/sl_stream`。工作区的常规打包入口是 `scripts/prepare_workspace.sh`。
