# sl_record 结构光录制

复用 `uv_record` 的会话、rosbag、日志、恢复及可用时的 MCAP 导出，图像只订阅
`youlong/camera/sl`。默认图像录制上限 **30 fps**；低帧率不补帧，高帧率抽帧后保留源时间戳。
原有 `uv_record` 继续默认录制 front/down，默认上限 5 fps。

按 `sl_camera/README.md` 构建后，先填写相机 YAML 的 `device`。一起启动相机和录制：

```bash
ros2 launch sl_record record.launch.py config:=/absolute/path/sl_camera.yaml
```

可选参数：`output_root`、`record_video_fps`（默认 30.0）、`bag_duration`（默认 10.0 秒）、
`bag_storage`（默认 auto）。例如：

```bash
ros2 launch sl_record record.launch.py \
  config:=/absolute/path/sl_camera.yaml \
  output_root:=/data/sl_sessions record_video_fps:=30.0
```

相机已单独启动时，在另一个已加载工作区的终端只启动录制：

```bash
ros2 run sl_record record
# 可选：--output-root /data/sl_sessions --video-fps 30 --bag-storage sqlite3
```

默认目录为仓库下的 `records/sl_sessions/<时间戳>/`：

```text
camera/raw/sl/frame_00000000000000000000.jpg
camera/raw/sl/frames.jsonl
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
相机重启或时间戳回退不会覆盖会话内已保存的图片。未收到相机帧时 bag 仍继续录制，
会话的图像时间对齐状态会标为 degraded；可查看相机状态和性能诊断定位原因。

rosbag 的话题正则、图像类型排除、分段及存储选择全部沿用 `uv_record`；
Image、CompressedImage、DisparityImage 像素不重复进入 bag。
Ctrl+C 正常停止会关闭写入并尝试汇总 `session.mcap`，其中仅有结构光归档图像，话题为
`/auv/record/camera/sl/compressed` 和 `/auv/record/camera/sl/frame_metadata`。
Foxy 等缺少可选 MCAP 能力的环境保留 SQLite3 bag 和逐帧图片。

异常中断后复用现有恢复命令：

```bash
ros2 run uv_record recover --session-dir /absolute/path/records/sl_sessions/YYYYMMDD_HHMMSS
```

恢复器检查图片完整性并清理无效索引尾部。一同启动时会保存相机与录制进程的输出、
ROS 日志和生命周期事件；分别启动时，相机进程日志由其启动终端或外部启动器管理。
