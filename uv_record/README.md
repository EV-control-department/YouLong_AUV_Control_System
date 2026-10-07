# uv_record 统一录制

`uv_record` 将摄像头视频、非图像 rosbag、轨迹位姿、ROS日志、建图快照和回放放入同一个 session。
默认使用 `competition` 比赛低占用模式；`dataset` 用于完整分辨率的数据集采样。

## 当前工作区的构建与运行

仓库根目录的 `uv_record` 是唯一源码；`workspace_auv/src/uv_record` 是相对符号链接。
部署时必须同时复制根目录包并保留链接，不能只复制工作区导致链接悬空。
在已有 ROS/FFmpeg/OpenCV 环境中无需新增 Python 库。录像必须有 FFmpeg：

```bash
cd /home/nvidia/YouLong_AUV_Control_System/workspace_auv
source /opt/ros/foxy/setup.bash
source install/setup.bash
command -v ffmpeg
colcon build --packages-select uv_record --symlink-install
source install/setup.bash
```

相机节点须已运行、front/down 均开启。本包不启动机器人任务，也不发送运动指令。
比赛模式直接读取同机 `http://127.0.0.1:8090/front` 和 `/down`，不依赖 go2rtc
或 Iceoryx2，不从 DDS 录制图像像素。

```bash
# 比赛：完整双摄视野，低画质，默认60分钟后结束
ros2 run uv_record record --profile competition \
  --output-root /home/nvidia/YouLong_AUV_Control_System/records/sessions

# 可选：还保存front_annotated和down_annotated；须先启用标注流和AI
ros2 run uv_record record --profile competition --go2rtc-stream-mode both

# 数据集：原拼接分辨率、JPEG归档，每路最多2FPS（非无损PNG）
ros2 run uv_record record --profile dataset --video-fps 2 --duration-minutes 30

# 数据集导出：前/下视分别切成左右目，保留帧元数据，拒绝覆盖已有目录
ros2 run uv_record export_frames /绝对路径/session \
  --output /绝对路径/新数据集目录 --split-stereo

# 本地回放：前视、下视、轨迹/航向/锥桶融合位置及近期观测点、ROS日志
ros2 run uv_record player /绝对路径/session
```

比赛默认值：每路最多8FPS、拼接宽度最多960（保持比例、完整视野不裁剪）、
H.264 `libx264` 每路目标/最高码率1200kbps、视频60秒分段、bag请求300秒分段。
`--video-fps` 现在实际选择源帧，不补帧；源帧不足时不会凭空达到指定帧率。
FFmpeg使用两个编码线程，并采用Foxy常见版本支持的 `-vsync 0`。
双路视频按码率估算约1.08GB/小时，四路约2.16GB/小时；另加TS封装、bag、索引及日志。
这是预算估算，不是已完成真机60分钟实测的结论。

默认4.8GB会话阈值、剩余空间不足1GB、或满60分钟时正常收尾，并在
会话根目录 `events.jsonl` 记录停止原因；容量单位为十进制GB。
空间检查每0.5秒、目录大小检查每5秒，关闭期间仍有少量落盘，所以4.8GB留有余量，
不是逐字节硬配额。极端日志洪泛或异常话题流量可能提前结束；不能同时无限保存
异常流量并保证完整60分钟。录制器不删除历史会话、不自动循环覆盖。
比赛配置不要扩大topic正则到所有DDS内容；`--max-session-gb` 可调整容量保护阈值。
dataset默认不限制会话大小，但仍保留60分钟时限及1GB剩余磁盘保护。

### 保存内容和同步边界

- `video/front/`、`video/down/`：比赛为TS/HLS视频；数据集为JPEG分块和帧索引。
- `metadata/trajectory.jsonl`：5Hz位姿，含xyz、roll/pitch/yaw（度）、消息戳和接收时间。
- `metadata/mapping.jsonl`：1Hz精简地图，类别、融合位置、状态、真假默认图标记及每格最近20条测量。
- `logs/rosout.jsonl`：从开始订阅起收到的ROS日志；不包含启动前日志或未发布到rosout的普通stdout。
- `bag/`：ROS日志、位姿、底层状态、建图地图/事件、任务状态；排除图像和控制指令。
- `logs/nodes/`：录制子进程日志；`metadata/performance.jsonl`：性能和剩余磁盘诊断。

直接MJPEG模式没有采集戳映射接口时，视频按FFmpeg PTS和本机接收时间做近似时间轴，
明确标记 `timestamp_alignment=degraded`，不冒充精确的图像/惯导采集同步。
因此回放用于展示和一般故障分析，不能据此证明毫秒级同步。
采用带 `CameraStreamFrameInfo` 的go2rtc链路时，可通过 `--video-source go2rtc --port 1984`
保留源采集戳映射；该消息缺失不阻止当前直接MJPEG录制。

player默认完全离线，不发布任何DDS消息。若确实需要给另一套GUI回放ROS状态，
仅在隔离域运行 `ROS_DOMAIN_ID=77 ros2 run uv_record player SESSION --publish-telemetry`，
GUI也使用域77；命令和action话题始终排除。无需启动BasicMotion或task_runner。
播放器优先PySide6，缺失时使用rqt常见的PyQt5。

## 旧架构可选接口

以下Iceoryx2接口属于旧架构，需要额外的auv_protocol/uv_image_transport包；
当前MJPEG比赛/数据集模式不需要它们。`--profile legacy` 恢复旧默认参数。

## raw：Iceoryx2 源帧

不需要启动 go2rtc；仿真中可关闭推流：

```bash
ros2 launch uv_sim_bringup sim.launch.py \
  record_session:=true record_mode:=raw enable_stream:=false
```

直接运行录制器也可使用：

```bash
ros2 run uv_record record --profile legacy --record-mode raw --output-root records/sessions
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

默认输出位置为 `records/sessions/<时间戳>/`，包含 `bag/`、`video/`、
`camera/raw/`（仅 raw 模式）、`logs/`、`metadata/` 和 `manifest.json`。
`recover` 与 `player` 保留对旧 `uv_log` session 格式的读取能力：

```bash
ros2 run uv_record recover --session-dir records/sessions/YYYYMMDD_HHMMSS
ros2 run uv_record player records/sessions/YYYYMMDD_HHMMSS
```

rosbag 继续排除 Image、CompressedImage 和 DisparityImage，避免重复保存图像像素。
