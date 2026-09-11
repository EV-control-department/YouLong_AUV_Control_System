# 节点参数与启动参数文档

## 所有可执行节点

| 可执行文件 | 包 | 工作区 | 说明 |
|---|---|---|---|
| `stonefish_simulator` | `stonefish_ros2` | workspace_sim | GPU 渲染仿真器 |
| `stonefish_simulator_nogpu` | `stonefish_ros2` | workspace_sim | 无头模式仿真器 |
| `sim_bridge` | `uv_sim` | workspace_sim | 仿真硬件桥接 |
| `hw_manager` | `uv_hm` | workspace_auv | 实车硬件管理 |
| `basic_motion` | `uv_control` | workspace_auv | 运动控制 |
| `uv_camera` | `uv_camera` | workspace_auv | YOLO 目标检测 |
| `object_localizer` | `uv_camera` | workspace_auv | 3D 目标定位 |
| `navigator` | `uv_nav` | workspace_auv | A* 路径规划 + 避障 |
| `task_runner` | `uv_task` | workspace_auv | YAML mission 任务执行器 |

---

## uv_camera 节点参数

### 运行模式

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `sim_mode` | bool | `false` | `true`: 订阅 `/auv/*/stitched` ROS 话题<br>`false`: 使用 V4L2 设备直接采集 |
| `front_cam_path` | str | `/dev/video0` | 前视摄像头 V4L2 设备路径（仅 real 模式） |
| `down_cam_path` | str | `/dev/video2` | 下视摄像头 V4L2 设备路径（仅 real 模式） |

### 图像发布

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `enable_gortc` | bool | `true` | 启用本地 MJPEG 源和 go2rtc 子进程 |
| `mjpeg_port` | int | `8090` | uv_camera 本地 MJPEG 服务端口 |
| `gortc_http_port` | int | `1984` | go2rtc HTTP/WebRTC 服务端口 |
| `stream_annotated` | bool | `true` | 通过 go2rtc 提供带检测框的视频流 `front_annotated/down_annotated` |

### 模型与数据集

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `model_path` | str | `""` | YOLO 模型 .pt 文件路径。为空时自动查找默认路径 |
| `save_dataset` | bool | `false` | 是否保存采集的图像到 `img/` 目录 |

### 相机标定

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `front_camera_matrix` | float[9] | 单位矩阵 | 前视相机内参 3x3 |
| `front_dist_coeffs` | float[5] | 全零 | 前视相机畸变系数 (k1,k2,p1,p2,k3) |
| `down_camera_matrix` | float[9] | 单位矩阵 | 下视相机内参 3x3 |
| `down_dist_coeffs` | float[5] | 全零 | 下视相机畸变系数 (k1,k2,p1,p2,k3) |

## object_localizer 节点参数

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `observation_history_size` | int | `500` | 定位观测历史最大保留数量 |

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
| `mission_file` | string | `config/missions/robocup_26.yaml` | 启动时加载的 YAML mission |
| `target_id` | string | `yellow_golf` | 比赛目标元数据 |
| `debug_mode` | bool | `false` | 开启后跳过 mission 自动执行，仅允许 `/task/exec` |

---

## 启动文件 Launch Arguments

### sim.launch.py

| 参数 | 默认值 | 说明 |
|---|---|---|
| `enable_ai` | `true` | 启用 uv_camera + object_localizer |
| `enable_motion` | `true` | 启用 basic_motion |
| `enable_nav` | `false` | 启用 navigator |
| `enable_task` | `false` | 启用 task_runner |
| `mission_file` | `config/missions/robocup_26.yaml` | YAML mission 文件路径 |
| `scenario_desc` | `guoshui_2026_cruise_seeded.scn` | Stonefish 场景文件 |
| `scene_seed` | `0` | 生成场景使用的整数 seed，运行目录隔离 |
**用法：**
```bash
ros2 launch uv_bringup sim.launch.py profile:=sim_dev enable_ai:=true
```

任务配置由 `mission_file` 指定。mission 只描述任务顺序，具体参数位于
`config/tasks/*.yaml`，mission 条目的 `params` 可以覆盖任务默认值。

### real.launch.py

| 参数 | 默认值 | 说明 |
|---|---|---|
| `enable_ai` | `true` | 启用 uv_camera + object_localizer |
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
| `enable_ai` | `false` | 启用 uv_camera + object_localizer |
| `enable_nav` | `false` | 启用 navigator |
| `enable_task` | `false` | 启用 task_runner |
| `enable_motion` | `false` | 启用 basic_motion |
| `scenario_desc` | `underwater_xunyun.scn` | Stonefish 场景文件 |
| `serial_dev` | `/dev/ttyUSB0` | MCU 串口设备 |
| `serial_baud` | `921600` | 串口波特率 |
**用法：**
```bash
ros2 launch uv_bringup hil.launch.py enable_ai:=true
```

### uv_camera/launch/perception_launch.py

该启动文件不再提供图像发布参数；视频通过 go2rtc 查看。

**用法：**
```bash
ros2 launch uv_camera perception_launch.py
```

## 话题参考

### uv_camera 发布话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/perception/detection/front_left` | `DetectionArray` | 前视左检测结果 |
| `/perception/detection/front_right` | `DetectionArray` | 前视右检测结果 |
| `/perception/detection/down_left` | `DetectionArray` | 下视左检测结果 |
| `/perception/detection/down_right` | `DetectionArray` | 下视右检测结果 |
uv_camera 不发布图像 DDS 话题；请通过 go2rtc 的 `front`、`down`、
`front_annotated`、`down_annotated` 流查看视频。

go2rtc 视频流：

| 流名称 | 说明 |
|---|---|
| `front` / `down` | 前视/下视原始拼接视频 |
| `front_annotated` / `down_annotated` | 前视/下视 YOLO 识别后带框拼接视频 |

### object_localizer 发布话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/perception/objects` | `ObjectPositionArray` | 3D 物体世界坐标 |
