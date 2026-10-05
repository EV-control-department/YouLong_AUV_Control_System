# uv_record 统一录制

`uv_record` 将 raw 原图、go2rtc 视频、非图像 rosbag、运行日志、恢复和回放放入同一个 session。
每个 session 只选择一种图像模式，默认 `raw`。

## raw：Iceoryx2 源帧

不需要启动 go2rtc；仿真中可关闭推流：

```bash
ros2 launch uv_sim_bringup sim.launch.py \
  record_session:=true record_mode:=raw enable_stream:=false
```

直接运行录制器也可使用：

```bash
ros2 run uv_record record --record-mode raw --output-root records/sessions
```

帧以无损 PNG 保存到 `camera/raw/front/`、`camera/raw/down/`，对应的
`frames.jsonl` 保留 Iceoryx2 `FrameHeader.timestamp_ns`、`capture_id`、
`stereo_pair_id`、图像尺寸、stride、编码、CameraInfo 版本和本机接收时间。

## go2rtc：编码视频

需要可用的 go2rtc HTTP API 和对应的相机流：

```bash
ros2 launch uv_sim_bringup sim.launch.py \
  record_session:=true record_mode:=go2rtc \
  go2rtc_stream_mode:=unannotated
```

`go2rtc_stream_mode` 可选 `unannotated`、`annotated`、`both`；
`go2rtc_video_format` 可选 `jpeg` 或 `ts`。`uv_stream` 为每个实际编码帧发布
流实例、帧序号、PTS、源帧时间戳和配对 ID；录制器按 PTS 将归档帧关联回源帧。
映射写入 `video/<stream>/frame_alignment.jsonl`。映射缺失时保存接收时间及未对齐标记，
不以接收时间冒充源时间戳；session 的
`timestamp_alignment.status` 会标为 `degraded`。

## 会话管理

按 Ctrl+C 正常停止时，录制器会先关闭相机和 rosbag 写入器，再尝试生成
session.mcap。这个文件合并 ROS 话题、相机压缩帧和逐帧对齐元数据；
相机帧位于 /auv/record/camera/<流名>/compressed，元数据位于对应的
frame_metadata 话题。raw 使用 PNG，go2rtc JPEG 归档保持 JPEG；TS 模式
同时将 MPEG-TS 分段和逐帧对齐记录写入 MCAP。导出完成后，player 会优先读取这个
合并后的 MCAP。ROS 2 Jazzy 等带有 `rosbag2_py` 和
`rosbag2_storage_mcap` 时会生成该文件；ROS 2 Foxy 没有这些可选能力时，
录制器自动使用 SQLite3，保留 `bag/part_*/`、相机文件和日志，并在
`manifest.json` 中记录 `session.mcap` 未生成的原因，不会因此导致录制失败。

默认输出位置为 `records/sessions/<时间戳>/`，包含 `bag/`、`video/`、
`camera/raw/`（仅 raw 模式）、`logs/`、`metadata/` 和 `manifest.json`。
`recover` 与 `player` 保留对旧 `uv_log` session 格式的读取能力：

```bash
ros2 run uv_record recover --session-dir records/sessions/YYYYMMDD_HHMMSS
ros2 run uv_record player records/sessions/YYYYMMDD_HHMMSS
```

rosbag 继续排除 Image、CompressedImage 和 DisparityImage，避免重复保存图像像素。
