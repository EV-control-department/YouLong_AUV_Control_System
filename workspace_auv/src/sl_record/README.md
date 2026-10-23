# sl_record 结构光与下视录制

复用 `uv_record` 的会话、rosbag、日志、恢复及可用时的 MCAP 导出。
分别订阅 `youlong/camera/sl` 和 `youlong/camera/down`，哪路有帧就保存哪路，
支持两路同时录制、只有一路，以及录制中途新启动的相机。
默认每路图像录制上限 **30 fps**；低帧率不补帧，高帧率抽帧后保留源时间戳。
原有 `uv_record` 继续默认录制 front/down，默认上限 5 fps。

按 `sl_camera/README.md` 构建后，编辑共享 profile：
`workspace_auv/src/sl_camera/config/profiles/default.yaml`。
默认启动结构光 `/dev/video0` 和下视双目 `/dev/video3`。一起启动采集和录制：

```bash
ros2 launch sl_record record.launch.py
# 自定义共享 profile：
ros2 launch sl_record record.launch.py profile:=/absolute/path/collection.yaml
```

profile 中的 `cameras.sl.enabled`、`cameras.down.enabled` 控制是否启动对应采集节点；
已在外部启动的采集节点无需再次启动，可将相应 enabled 设为 false。
录制器仍然等待两路 Iceoryx2 数据，开关不限制外部数据的录制。

录制默认值也放在同一个 profile：

```yaml
recording:
  output_root: ""       # 空值使用仓库 records/sl_sessions
  video_fps: 30.0
  bag_duration: 10.0
  bag_storage: auto
```

launch 参数 `output_root`、`record_video_fps`、`bag_duration`、`bag_storage`
可覆盖 profile 中相应值。例如：

```bash
ros2 launch sl_record record.launch.py \
  profile:=/absolute/path/collection.yaml \
  output_root:=/data/sl_sessions record_video_fps:=30.0
```

相机已单独启动时，在另一个已加载工作区的终端只启动录制：

```bash
ros2 run sl_record record
# 同样读取 profile 中的录制参数，但不会启动摄像头：
ros2 run sl_record record --profile /absolute/path/collection.yaml
# 可选：--output-root /data/sl_sessions --video-fps 30 --bag-storage sqlite3
```

默认目录为仓库下的 `records/sl_sessions/<时间戳>/`：

```text
camera/raw/sl/frame_00000000000000000000.jpg
camera/raw/sl/frames.jsonl
camera/raw/down/frame_00000000000000000000.jpg
camera/raw/down/frames.jsonl
bag/part_*/
logs/
metadata/
manifest.json
events.jsonl
session.mcap                 # 安装相应 MCAP 能力时生成
```

JPEG 文件保留相机发布的原字节，不叠加文字或重新编码。`frames.jsonl` 中的
`timestamp_ns`、`source_timestamp_ns` 是来自相机帧头的采集时间，`capture_id` 是相机帧号，
`frame_sequence` 是会话内的连续图片序号。另有尺寸、时间戳 epoch 和接收/写入诊断字段。
相机重启或时间戳回退不会覆盖会话内已保存的图片。
没有帧的通道保留空索引并等待数据，不阻塞另一通道或 rosbag。
两路都没有帧时状态为 waiting；只有一路正常到帧时仍可正常对齐，
不会仅因另一台摄像头未启用而标为 degraded。缺失源时间戳仍会标为 degraded。
manifest/heartbeat 记录每路帧数、available_cameras 和 waiting_for_cameras。

rosbag 的话题正则、图像类型排除、分段及存储选择全部沿用 `uv_record`；
Image、CompressedImage、DisparityImage 像素不重复进入 bag。
Ctrl+C 正常停止会关闭写入并尝试汇总 `session.mcap`，相机话题为
`/auv/record/camera/{sl,down}/compressed` 和对应的 `frame_metadata`；
没有实际录到帧的相机不会加入 MCAP 图像流。
Foxy 等缺少可选 MCAP 能力的环境保留 SQLite3 bag 和逐帧图片。

异常中断后复用现有恢复命令：

```bash
ros2 run uv_record recover --session-dir /absolute/path/records/sl_sessions/YYYYMMDD_HHMMSS
```

恢复器检查图片完整性并清理无效索引尾部。一同启动时会保存相机与录制进程的输出、
ROS 日志和生命周期事件；分别启动时，相机进程日志由其启动终端或外部启动器管理。
