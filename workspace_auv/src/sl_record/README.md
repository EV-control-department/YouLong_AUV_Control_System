# sl_record 自动发现录制

订阅 `sl_msgs/msg/CameraCatalog` DDS 清单，为每个启用的描述项建立独立 Iceoryx2 reader。
没有固定相机数量、名称或服务名，也不从 profile 的摄像头定义推断订阅源。
录制器与采集器可以任意先后启动；采集器重启后，按新清单增删 reader。
某路离线时等待数据，其他图像与 rosbag 继续录制。

构建见 [sl_camera](../sl_camera/README.md)。共享设置文件为
`workspace_auv/src/sl_camera/config/profiles/default.yaml`。
录制器只读取其中的 `catalog_topic` 和 `recording`：

```yaml
recording:
  output_root: ""       # 空值使用仓库 records/sl_sessions
  video_fps: 30.0       # 每路图像上限
  bag_duration: 10.0
  bag_storage: auto    # auto / sqlite3 / mcap
```

## 启动

同时启动时只有一个 `sl_camera` 管理进程：

```bash
ros2 launch sl_record record.launch.py
ros2 launch sl_record record.launch.py \
  profile:=/absolute/path/collection.yaml \
  output_root:=/data/sl_sessions record_video_fps:=30.0
```

还支持 `bag_duration`、`bag_storage` launch 参数。
已有采集器时，只启动录制器，不要再同时启动第二个相机管理进程：

```bash
ros2 run sl_record record
ros2 run sl_record record --profile /absolute/path/collection.yaml
# 命令行覆盖：
ros2 run sl_record record --catalog-topic /auv/sl_camera/cameras \
  --output-root /data/sl_sessions --video-fps 30 --bag-storage sqlite3
```

默认每路图像上限 30 fps；低帧率不补帧，高帧率抽帧后保留源采集时间戳。
相机的 `enabled: false` 表示不订阅，`online: false` 表示已配置但当前等待数据。
默认一套管理进程负责全部相机；不支持多个管理器并发争用同一个清单话题。

## 归档与时间戳

默认输出 `records/sl_sessions/<时间戳>/`：

```text
camera/raw/camera_1_sl/frame_00000000000000000000.jpg
camera/raw/camera_1_sl/frames.jsonl
camera/raw/camera_2_down/...
metadata/camera_catalog.json
metadata/camera_catalogs.jsonl
bag/part_*/
logs/
manifest.json
events.jsonl
session.mcap                 # 可选 MCAP 能力可用时汇总
```

目录名由 ID 和名称共同组成；没有实际保存过帧的相机不创建图片目录。
JPEG 直接写入相机原始字节，不叠字、不旋转、不重新编码。
每条索引包含：

- 源采集时间 `timestamp_ns/source_timestamp_ns`、源帧号 `capture_id`。
- 会话图片序号 `frame_sequence`、路径、尺寸、格式及接收/写入诊断。
- `camera_id/camera_name/frame_id`、`source_instance_id`、`config_version`、`catalog_sequence`。

`metadata/camera_catalog.json` 保存最新清单；`camera_catalogs.jsonl` 保存完整历史。
索引用 `catalog_sequence` 关联对应快照，快照先持久化，再保存引用它的帧。
仅接受 ID、配置版本、尺寸及编码均匹配的帧，旧版本残留帧不会归入新参数。
相机重启后源帧号可以重置，会话图片序号继续递增；已有 JPEG 不覆盖。
时间戳回退增加 `timestamp_epoch`，保留源时间；缺少源时间戳标记为未对齐，
限流此时使用主机单调时钟，不伪造采集时间。
显式复用会话目录时清单序号与图片序号也会延续。

无清单或没有任何图片时会话状态为 waiting，缺帧允许继续录制。
磁盘写入失败、reader 异常或线程退出超时会记录失败并将会话标记 FAILED。
Ctrl+C 正常停止关闭 reader、DDS 线程和 rosbag 写入器。

## rosbag、MCAP 与恢复

只读调用现有 `uv_record` 的会话、日志、rosbag 命令、恢复和 MCAP 工具，
SL 发现与图像写入逻辑全部位于 `sl_*` 包。
rosbag 默认话题正则、Image/CompressedImage/DisparityImage 类型排除、
分段与存储后端选择保持 UV 原有规则；自定义清单话题也会额外纳入。
DDS 清单进入 bag，像素通过 Ice 写到 JPEG，避免在 bag 中重复录制 ROS 图像。

正常停止时，MCAP 能力可用则合并 bag 与实际保存的图像，图像话题为
`/auv/record/camera/camera_<ID>_<name>/compressed` 和对应的 `frame_metadata`。
采集器的 `frame_id` 和原始时间戳保存在逐帧 metadata 中。
共享导出器的 CompressedImage header.frame_id 沿用归档目录名，
遇到时间戳回退时共享导出器可能平移 MCAP 回放时间线；源时间仍完整保留在索引及 metadata。
Foxy 等缺少可选能力时保留原始 bag 和图片。

异常中断后运行：

```bash
ros2 run sl_record recover --session-dir /absolute/path/sl_sessions/YYYYMMDD_HHMMSS
# 或扫描 SL 默认根目录：
ros2 run sl_record recover
```

恢复器跳过仍在运行或已正常结束的会话，清理损坏图片的索引及不完整清单历史尾部，
恢复最新清单快照，并标记 RECOVERED。恢复不会补拍图片或生成原本没有采集的数据。
联合启动时保存管理进程与录制器的输出及生命周期日志；独立启动的采集器日志由其终端管理。

## 模拟验证

构建并加载工作区后可运行：

```bash
python3 -m pytest -q src/sl_camera/test src/sl_record/test
```

其中原生联调在 iceoryx2、rosbag2_py 和 MCAP 可用时运行，否则跳过。
测试用模拟采集输入验证 1/2/3 路、DDS 晚订阅、离线重连、管理器重启增删、
原始 JPEG、版本关联、限流、时间戳回退、恢复、rosbag 参数一致及 MCAP 汇总。
实机仍需在目标主机确认设备权限、可用 MJPG 模式及目标帧率。
