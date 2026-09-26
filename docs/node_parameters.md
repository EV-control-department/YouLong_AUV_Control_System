# 节点参数与启动参数文档

## 所有可执行节点

| 可执行文件 | 包 | 工作区 | 说明 |
|---|---|---|---|
| `stonefish_simulator` | `stonefish_ros2` | workspace_sim | GPU 渲染仿真器 |
| `stonefish_simulator_nogpu` | `stonefish_ros2` | workspace_sim | 无头模式仿真器 |
| `sim_bridge` | `uv_sim` | workspace_sim | 仿真硬件桥接 |
| `hw_manager` | `uv_hm` | workspace_auv | 实车硬件管理 |
| `basic_motion` | `uv_control` | workspace_auv | 运动控制 |
| `uv_camera` | `uv_camera` | workspace_auv | 采集、CameraInfo、iceoryx2 原图发布 |
| `object_detector` | `uv_perception` | workspace_auv | YOLO 检测 |
| `object_localizer` | `uv_perception` | workspace_auv | 双目/射线几何定位 |
| `object_estimator` | `uv_perception` | workspace_auv | 多帧关联与跟踪 |
| `camera_streamer` | `uv_stream` | workspace_auv | raw/annotated H264 推流适配 |
| `record` | `uv_record` | workspace_auv | raw Iceoryx2 或 go2rtc 视频、非图像 rosbag 和运行日志会话记录 |
| `navigator` | `uv_nav` | workspace_auv | A* 路径规划 + 避障 |
| `task_runner` | `uv_task` | workspace_auv | YAML 任务执行器 |

---

## uv_camera 节点参数

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `sim_mode` | bool | `false` | `true` 从 Stonefish POSIX 共享内存读取；`false` 从真机 V4L2 采集 |
| `enable_front` | bool | `true` | 是否启用前视相机 |
| `enable_down` | bool | `true` | 是否启用下视相机 |
| `camera_config_profile` | str | `auto` | `auto` 按 sim_mode 选择 `sim` 或 `real` 配置 |
| `camera_config_dir` | str | `` | 可选相机 YAML 配置目录 |
| `camera_startup_timeout_sec` | float | `5.0` | 等待相机源就绪的超时 |
| `camera_info_version` | int | `1` | 写入 iceoryx2 帧头的相机标定版本 |

Camera YAML 保存设备路径与内参。图像经 `uv_image_transport` 的 iceoryx2 API 发布到 `youlong/camera/front` 和 `youlong/camera/down`；不发布 DDS 图像话题。视频由 `uv_stream` 转码。iceoryx2 Python binding 由仓库现有脚本构建和安装。

## uv_perception 节点参数

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `confidence` | float | `0.5` | YOLO 最低置信度 |
| `model_path` | str | 自动搜索 | YOLO 权重路径 |
| `stereo_baseline_m` | float | `0.10` | 双目基线，米 |
| `world_frame` | str | `odom` | 几何结果转换到的 TF 坐标系 |
| `association_distance_m` | float | `2.0` | estimator 最近邻关联距离 |

---

## sim_bridge 节点参数

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `hil_mode` | bool | `false` | `true`: HIL 模式（相机直通 + 推力混合）<br>`false`: SIL 全仿真（PID + ZIT6 状态机） |

---

## 无参数节点

`basic_motion`、`navigator`、`hw_manager` 不接受 ROS2 参数。

## task_runner 节点参数

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `mission_file` | string | `config/missions/robocup_26.yaml` | 启动时加载的 YAML 任务流程或单个任务文件 |
| `target_id` | string | `yellow_golf` | 比赛目标元数据 |
| `debug_mode` | bool | `false` | 开启后跳过任务流程自动执行，仅允许 `/auv/mission/execute` |

---

## 启动文件参数

### sim.launch.py

| 参数 | 默认值 | 说明 |
|---|---|---|
| `enable_ai` | `true` | 启用 uv_perception 检测、定位和跟踪节点；相机采集由相机启动项管理 |
| `enable_motion` | `true` | 启用 basic_motion |
| `enable_nav` | `false` | 启用 navigator |
| `enable_task` | `false` | 启用 task_runner |
| `enable_stream` | `true` | 启动 go2rtc；raw-only 录制可设为 `false` |
| `record_session` | `false` | 创建并记录统一 session |
| `record_mode` | `raw` | 图像录制方式：`raw` 或 `go2rtc`，每个 session 选一种 |
| `go2rtc_stream_mode` | `unannotated` | go2rtc 模式的流选择：`unannotated`、`annotated` 或 `both` |
| `go2rtc_video_format` | `jpeg` | go2rtc 视频归档格式：`jpeg` 或 `ts` |
| `mission_file` | `config/missions/robocup_26.yaml` | YAML 任务流程或单个任务文件路径 |
| `scenario_desc` | `guoshui_2026_cruise_seeded.scn` | Stonefish 场景文件 |
| `scene_seed` | `0` | 生成场景使用的整数 seed，运行目录隔离 |
**用法：**
```bash
ros2 launch uv_sim_bringup sim.launch.py profile:=sim_dev enable_ai:=true
```

任务配置由 `mission_file` 指定。它可以指向描述任务顺序的任务流程文件，
也可以直接指向 `config/tasks/*.yaml` 执行单个任务。任务流程条目的
`params` 可以覆盖任务默认值。

### real.launch.py

| 参数 | 默认值 | 说明 |
|---|---|---|
| `enable_ai` | `true` | 启用 `uv_perception` 检测、定位观测和跟踪节点；相机采集由相机启动项管理 |
| `enable_motion` | `true` | 启用 basic_motion |
| `enable_nav` | `true` | 启用 navigator |
| `enable_task` | `false` | 启用 task_runner |
**用法：**
```bash
ros2 launch uv_bringup real.launch.py profile:=real_default
```

### hil.launch.py

| 参数 | 默认值 | 说明 |
|---|---|---|
| `enable_ai` | `false` | 是否启动 uv_perception 检测、定位和跟踪节点 |
| `enable_nav` | `false` | 启用 navigator |
| `enable_task` | `false` | 启用 task_runner |
| `enable_motion` | `false` | 启用 basic_motion |
| `scenario_desc` | `underwater_xunyun.scn` | Stonefish 场景文件 |
| `serial_dev` | `/dev/ttyUSB0` | MCU 串口设备 |
| `serial_baud` | `921600` | 串口波特率 |
**用法：**
```bash
ros2 launch uv_sim_bringup hil.launch.py enable_ai:=true
```

### 分包启动

单独启动组件时分别使用所属包的 launch；完整真机或仿真图由 `uv_bringup` / `uv_sim_bringup` 编排。

```bash
ros2 launch uv_camera camera_launch.py sim_mode:=false
ros2 launch uv_perception perception_launch.py
ros2 launch uv_stream stream_launch.py
```

## 话题参考

### uv_perception 检测话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/auv/perception/detections` | `DetectionArray` | 汇总检测结果；消息内 `camera_name` 标识 front_left、front_right、down_left 或 down_right |
uv_camera 通过 CameraInfo 发布标定元数据，并在 iceoryx2 服务 `youlong/camera/front`、`youlong/camera/down` 发送图像帧；它不发布 DDS 图像话题。请通过 go2rtc 的 `front`、`down`、
`front_annotated`、`down_annotated` 流查看视频。

go2rtc 视频流：

| 流名称 | 说明 |
|---|---|
| `front` / `down` | 前视/下视原始拼接视频 |
| `front_annotated` / `down_annotated` | 前视/下视 YOLO 识别后带框拼接视频 |

### object_localizer 发布话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/auv/perception/measurements` | `ObjectMeasurementArray` | 由检测、CameraInfo 和 TF 生成的几何测量 |

### object_estimator 发布话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/auv/perception/tracks` | `ObjectTrackArray` | 多帧关联后的目标状态 |
