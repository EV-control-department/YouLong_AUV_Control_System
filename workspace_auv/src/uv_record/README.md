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

JPEG 帧直接保存到 `camera/raw/front/`、`camera/raw/down/` 下的逐帧 `.jpg` 文件，
文件字节与 Iceoryx2 payload 完全一致。真机 JPEG 已在相机端无损旋转 180°，
因此单独打开 JPG 时方向也正确；仿真 JPEG 保持仿真方向。raw 不执行解码或重新编码。
对应的 `frames.jsonl` 保留 Iceoryx2 `FrameHeader.timestamp_ns`、`capture_id`、
`stereo_pair_id`、尺寸、`stride=0`、`encoding=JPEG`、CameraInfo 版本及本机接收时间。
源 JPEG 已有的压缩保留；这里的 raw 表示源帧录制，不表示无损像素图。

raw 性能探针为版本 2：`jpeg_write_ms` 记录直接写 JPEG 的耗时，`file_size_bytes`
记录该帧字节数，`total_jpeg_bytes` 记录累计字节数；不再产生 BGR 转换或 PNG 编码指标。
`camera_receive_unix_ns` 为收到帧的时间，`file_write_complete_unix_ns` 为文件写入返回时间；
兼容字段 `receive_time_unix_ns` 仍表示写入完成时间。继续记录读帧等待/间隔、源帧间隔、
`capture_id_delta`、索引刷新及约每秒一次的索引 fsync 耗时。

每路相机仍由独立线程顺序读帧、写 JPG 和帧索引，没有额外应用层队列。每 5 秒写入的
`metadata/performance.jsonl` 包含最近 300 帧的均值、P95、最大值、当前阶段及持续时间，
以及系统和磁盘 I/O、剩余空间及 inode。写入异常会在终端和 `events.jsonl` 记录相机、
阶段、帧号、错误和文件系统状态。文件与索引不保证掉电时最后一帧持久化；recover 会
剔除缺失、截断、尺寸不符的 JPG 和不完整索引尾部。
停止录制后请提供以下文件（或整个 session 目录）：

- `manifest.json` 和 `events.jsonl`
- `metadata/performance.jsonl`
- `camera/raw/front/frames.jsonl`、`camera/raw/down/frames.jsonl`（实际启用的相机）
- `logs/` 下启动与录制控制台日志

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
frame_metadata 话题。新 raw 使用 JPEG，旧 raw PNG 导出仍标记 PNG；go2rtc JPEG 归档保持 JPEG；TS 模式
同时将 MPEG-TS 分段和逐帧对齐记录写入 MCAP。导出完成后，player 会优先读取这个
合并后的 MCAP。ROS 2 Jazzy 等带有 `rosbag2_py` 和
`rosbag2_storage_mcap` 时会生成该文件；ROS 2 Foxy 没有这些可选能力时，
录制器自动使用 SQLite3，保留 `bag/part_*/`、相机文件和日志，并在
`manifest.json` 中记录 `session.mcap` 未生成的原因，不会因此导致录制失败。

默认输出位置为 `records/sessions/<时间戳>/`，包含 `bag/`、`video/`、
`camera/raw/`（仅 raw 模式）、`logs/`、`metadata/` 和 `manifest.json`。
`recover` 与 `player` 支持新 raw JPG、旧 raw PNG 和旧 `uv_log` session 格式：

```bash
ros2 run uv_record recover --session-dir records/sessions/YYYYMMDD_HHMMSS
ros2 run uv_record player records/sessions/YYYYMMDD_HHMMSS
```

rosbag 继续排除 Image、CompressedImage 和 DisparityImage，避免重复保存图像像素。

## 检测坐标与标定版本

raw JPG 为 Ice payload 原字节，保留镜头畸变；DetectionArray schema 2 使用去畸变坐标，校正前后 CameraInfo 位于 `/auv/perception/camera/{front,downward}/{left,right}/calibration`。默认录制包含这些小型主题，manifest 的 perception 段明确两种坐标系。回放/MCAP 导出将旧 DetectionArray 转成 RAW，并保留旧 JPG/PNG 支持；新 schema 截断消息直接报错。标定与类别表回放使用持久化 QoS。详见 [感知链路说明](../uv_perception/README.md)。
