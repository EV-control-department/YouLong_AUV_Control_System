# sl_camera 可配置多摄像头采集

一个 `sl_camera` 进程管理 profile 中全部启用的摄像头，每路使用独立采集线程和 Iceoryx2 发布器。
DDS 发布完整摄像头清单，`sl_record` 据此发现图像源。此链路不启动或读取 `uv_camera`。

默认 profile：`config/profiles/default.yaml`。

| ID | 名称 | 设备 | 完整图像尺寸 | 布局 | Iceoryx2 服务 |
|---|---|---|---|---|---|
| 1 | sl | /dev/video0 | 1280×720 | mono | youlong/sl_camera/sl |
| 2 | down | /dev/video3 | 2560×960 | stereo_sbs | youlong/sl_camera/down |

两路默认请求 30 fps。下视双目仍是一张完整的左右拼接图像，**保留设备原始方向**。
不旋转、不拆分、不去畸变、不重新编码。设备必须支持配置的 MJPG 分辨率和帧率。

## 构建与启动

在仓库根目录执行，ROS 发行版按目标机选择：

```bash
source /opt/ros/jazzy/setup.bash
cd workspace_auv
colcon build --symlink-install --packages-up-to sl_record
source install/setup.bash
cd ..
source scripts/source_workspace.sh
```

运行环境需要兼容当前 Python 的 iceoryx2 binding，参见仓库环境准备说明。
单独启动采集：

```bash
ros2 launch sl_camera camera.launch.py
ros2 launch sl_camera camera.launch.py profile:=/absolute/path/collection.yaml
# 同一个节点的直接入口：
ros2 run sl_camera sl_camera --ros-args -p profile:=/absolute/path/collection.yaml
```

`profile` 可以是安装的 profile 名称（默认 `default`）或 YAML 路径。
修改配置后重启管理进程生效。旧版 `config:=`、固定 `cameras.sl/down` 配置不再使用。
同时启动采集与录制见 [sl_record](../sl_record/README.md)。

## 增删摄像头

复制默认 profile，按需增删 `cameras` 列表。每项的分辨率都在本文件中直接设置：

```yaml
schema_version: 2
catalog_topic: /auv/sl_camera/cameras
cameras:
  - id: 1
    name: sl
    enabled: true
    device: /dev/video0
    ice_service: youlong/sl_camera/sl
    frame_id: sl_camera_optical_frame
    width: 1280
    height: 720
    fps: 30.0
    pixel_format: MJPG
    layout: mono
    startup_timeout_sec: 5.0
    reconnect_interval_sec: 1.0
    calibrations: []
recording:
  output_root: ""
  video_fps: 30.0
  bag_duration: 10.0
  bag_storage: auto
```

ID 必须是非零 uint32；名称只用字母、数字、下划线。ID、名称、Ice 服务名必须唯一，
启用的设备也必须唯一。可用 `/dev/v4l/by-id/...` 稳定设备选择。
`enabled: false` 的项仍在 DDS 清单中，但不打开设备，也不会被录制器订阅。
空列表合法；无需修改代码即可配置第三台或更多摄像头。

`layout` 支持 `mono` 和 `stereo_sbs`。后者 width 为整幅拼接图像宽度且须为偶数。
`frame_id` 只提供坐标系标识；本次不进行 TF 变换。
`calibrations: []` 表示未标定，不生成单位矩阵，也不读取 UV 标定。
将来可填 `eye: mono/left/right` 和标准 `sensor_msgs/CameraInfo` 字段的 `camera_info`；
这些字段只随清单传递，不改变图像。

## DDS 与 Ice 帧的关联

默认清单话题：`/auv/sl_camera/cameras`，类型 `sl_msgs/msg/CameraCatalog`。
QoS 为 Reliable、Transient Local、深度 1；启动立即发布，之后每秒及状态变化时更新。
清单包含禁用、启动中、在线和离线的全部配置项，以及数量、状态、错误描述和标定数组。

`requested_fps` 是请求值；`negotiated_fps` 是设备报告的协商值；
`actual_fps` 是最近最多 31 帧的成功发布速率，尚不能测量或离线时为 0（未知）。
它不会用请求值冒充实测速率。每次启动生成新的 `source_instance_id` 和非零 `config_version`。

```bash
ros2 topic echo /auv/sl_camera/cameras sl_msgs/msg/CameraCatalog \
  --qos-reliability reliable --qos-durability transient_local
```

沿用通用 Ice 帧头：`camera_group` 为配置 ID，`camera_info_version` 为本次配置版本；
后者与“是否标定”无关。成功读取后立即从 ROS 时钟取纳秒时间，
这是主机软件采集时间，不是硬件曝光时间。每路源帧号独立递增。

打开失败时报告具体路径、权限或 errno；连续无有效帧达到默认 5 秒后重开，
默认重试间隔 1 秒。一台离线不影响其他台。Ctrl+C 等待各采集线程退出；
设备驱动阻塞导致退出超时会明确报错。

`No config file was loaded` 是 Iceoryx 默认配置提示，不代表摄像头 profile 没有加载。
V4L2 打开失败时在目标机检查：

```bash
ls -l /dev/video0 /dev/video3
id
fuser -v /dev/video0 /dev/video3
v4l2-ctl -d /dev/video0 --all --list-formats-ext
v4l2-ctl -d /dev/video3 --all --list-formats-ext
```

确认设备支持 Video Capture 和目标 MJPG 模式、用户有读写权限、设备未被其他进程占用。
容器还需要映射实际设备。不要让其他 UV 采集进程同时打开同一设备。
