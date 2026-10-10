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
H.264 `libx264` 每路目标/最高码率1200kbps、视频5秒分段、bag请求300秒分段。
`--video-fps` 现在实际选择源帧，不补帧；源帧不足时不会凭空达到指定帧率。
FFmpeg使用两个编码线程，并采用Foxy常见版本支持的 `-vsync 0`。
`competion` 拼写作为 `competition` 的兼容别名。启动必须等到两路均实际写入
完整视频分段或 JPEG，解码日志的一帧不再视为录制成功。FFmpeg 错误保存在
`logs/nodes/video_*.log` 和 `video/*/status.json`；异常重启采用最多30秒退避。
H.264 编码失败时比赛模式自动退回 JPEG 分块（相同帧率和宽度），保留
`ts_failure.json`、`fallback_reason` 和实际 `output_format`。JPEG 无法保证1200kbps，
会话容量保护继续生效，异常情况下可能提前结束。正常停止通过 FFmpeg 专用
命令管道发送 `q`，等待清单和尾段完整写入；异常 `.ts.tmp` 保留供检查。
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
- `metadata/mapping.jsonl`：通常1Hz精简地图；状态变化、最终结果立即保存，避免漏掉最终地图。
- `metadata/mapping_observations.jsonl`：5Hz建图观测及拒绝原因，包含 AprilTag 世界坐标、深度和位姿年龄。
- `metadata/apriltag.jsonl`：在线 AprilTag 解码/ID过滤/未解码候选/深度失败诊断。
- `metadata/tasks.jsonl`：任务状态变化，不丢失短任务的切换事件。
- `logs/rosout.jsonl`：从开始订阅起收到的ROS日志；不包含启动前日志或未发布到rosout的普通stdout。
- `bag/`：ROS日志、位姿、原始 odom、底层状态、建图地图/观测/事件、目标位置/观测和任务状态；
  还保存 setpoint/servo/light 指令供故障分析。比赛默认不录全量分割 mask 话题，以控制容量；
  图像类型仍排除；player 回放始终不发送指令。
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

### 离线 Python 解读与可视化

```bash
# 构建后使用 ROS 包入口；无需启动任何节点或连接机器人
ros2 run uv_record analyze /绝对路径/session --output /绝对路径/新报告目录

# 仓库根目录直接运行；已有 JSONL 时不需要 ROS 环境
PYTHONPATH=uv_record python3 -m uv_record.analyze /绝对路径/session \
  --output /绝对路径/新报告目录 --no-bag
```

打开报告目录中的 `report.html`，包含建图格子和证据来源、可拖动时间轴的 XY 路径、
前/下视视频播放器、各轮任务开始/结束/下压/释放等阶段时间戳，以及 AprilTag
在线诊断、世界坐标和录像离线重识别的角点标注图片。导出 `maps.json`、每轮
`map_*.svg/csv`、`trajectory.svg/csv`、`tasks.svg/csv`、`task_events.csv`、
`videos.json`、`videos/*.mp4`、`apriltag.json/csv` 和 `apriltag/*.jpg`。
SVG 可独立打开和导出，HTML 不需要网络。导出目录必须为空且位于 session 外；
脚本不会修改原录制内容。视频清单保留原文件的链接，播放副本位于报告目录。

默认使用 FFmpeg 将各 HLS 清单分别无转码封装为 MP4；不同重启不拼成连续视频。
JPEG 归档生成8FPS的实时回看副本，间隙重复前一帧，原帧索引继续作为证据。
`--ffmpeg /路径/ffmpeg` 可指定二进制；没有 FFmpeg 时仍输出全部遥测和源视频清单。
默认 AprilTag 字典为 `DICT_APRILTAG_16h5`，每秒采样1帧，最多保存200张识别成功图片；
可设置 `--tag-dictionary`、`--tag-sample-fps`、`--max-tag-images`，或 `--no-detect-tags`。
离线像素解码需要含 `cv2.aruco` 的 OpenCV；缺失会提示，在线识别报告仍可生成。
离线识别明确标记 `offline_video_redetection`，不当作当时在线任务的识别结果。

默认显示 UTC+8，`--timezone-offset` 可调整。路径按接收时钟显示，保留源消息戳；
源戳重复、漂移修正或任务重启均不会伪装成真实游动。没有明确任务成功结束日志时，
结束时间由下一任务开始推断并标记 `end_inferred`；录制结束时尚未结束的任务标为
`incomplete_at_recording_end`。缺失 JSONL 可在 source 对应 ROS 环境后从 SQLite bag
补读，先将 DB/WAL/SHM 复制到临时目录；MCAP-only 会话依赖 JSONL 索引。
旧地图缺少最终快照但日志含“建图最终结果”时，恢复最终类别并标注
`final_assignment_source=task_result_log`；位置和协方差仍来自最后过程快照，
不会补造最终位置或 `verified_complete`。原始快照类别保存在 `snapshot_label`。

旧 session 里只有解码索引、没有 `.ts` 或 `.mjpg` 时，图像像素未保存，无法还原
视频或 AprilTag 角点。报告会明确显示缺失，仍展示已有地图、路径、任务和在线诊断。

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
