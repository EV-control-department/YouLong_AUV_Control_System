# uv_log 录制策略

默认录制 `front` 和 `down` 原始画面，而不是 `front_annotated` 和
`down_annotated`。标注流跟随 YOLO 推理频率，不能作为连续视频的时间基准。

默认采用方案 2：直接保存相机 HTTP MJPEG 中已经编码好的 JPEG 帧，播放时
才用 OpenCV 解码。录制进程不启动 H.264 编码器，因此不会把图像消息写入
rosbag，也不会因为编码器积压造成录制卡顿。

仿真启动时，录制 FPS 默认跟随 `camera_stitch_fps`：

```bash
ros2 launch uv_bringup sim_bringup.py \
  record_session:=true \
  record_video_mode:=raw \
  open_annotated_windows:=false \
  stream_annotated:=false
```

需要同时保存标注流时使用：

```bash
ros2 launch uv_bringup sim_bringup.py \
  record_session:=true \
  record_video_mode:=both
```

视频按默认 2 秒短段保存为 `chunk_XXXXXX.mjpg`，旁边的
`chunk_XXXXXX.jsonl` 保存时间戳、序号、偏移、长度和 CRC32。每帧带有自描述
头，写入顺序是先帧数据、后索引；完成短段会先 fsync，坏尾会被扫描并截断，
之前的短段仍可播放。断电时可能
损失正在写入的当前短段（默认约 2 秒），但不会把损坏尾暴露给播放器。启动
后如需整理未正常结束的 session：

```bash
ros2 run uv_log recover --session-dir sessions/YYYYMMDD_HHMMSS
ros2 run uv_log player sessions/YYYYMMDD_HHMMSS
```

`record_video_fps` 应与 `camera_stitch_fps` 保持一致。默认仿真值为 5 FPS；
提高 `camera_stitch_fps` 时，录制 FPS 会自动跟随，除非显式覆盖
`record_video_fps`。

`record_video_format` 默认为 `jpeg`。如需兼容旧的 TS 播放链路，可设置
`record_video_format:=ts`，此时恢复 H.264 转码路径，但 CPU 和编码延迟会更高。

仿真启动默认使用 `record_use_sim_time:=false`，录制时间戳采用稳定的墙上时钟，
不依赖 `/clock`。录制默认会自动打开 MJPEG 端点；不录制时预览默认关闭以节省
CPU。需要预览时显式设置 `enable_preview:=true`，如手动执行 `record`，也应加上
`--use-sim-time false`。

rosbag 只保存状态、控制、检测和其他元数据等小消息。录制器会无条件排除
`sensor_msgs/msg/Image`、`sensor_msgs/msg/CompressedImage` 和
`stereo_msgs/msg/DisparityImage`；旧参数 `record_image_topics` 仅为兼容保留，
即使设为 `true` 也不会把图像写入 bag。播放器会将剩余的 rosbag 话题按同一
时间轴反序列化并发布；拖动进度条时也会重放到对应时刻。

如果只需要快速录制原始 MJPEG，不需要 ROS session，可以使用：

```bash
GORTC_HOST=127.0.0.1 GORTC_PORT=8090 \
GORTC_STREAMS="front down" VIDEO_FPS=10 \
./scripts/record_go2rtc.sh
```

该独立脚本仍使用 2 秒 TS 分段，并只同步已经稳定的分段文件；session 录制
建议使用上面的 `record_session` 入口，以获得 JPEG 归档和恢复工具支持。
