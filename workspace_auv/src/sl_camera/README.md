# sl_camera 结构光与下视双目采集

`camera.launch.py` 按共享 profile 分别启动结构光单目和下视双目，默认两路都启用。
结构光由 `sl_camera` 节点采集，下视直接复用 `uv_camera`（仅启用 down）。
两路各自发布 Iceoryx2 JPEG，设备打不开时各自重试。

| 相机 | 默认设备 | 默认图像 | Iceoryx2 服务 |
|---|---|---|---|
| 结构光 | `/dev/video0` | 1280×720，请求 30 fps，保留原方向 | `youlong/camera/sl` |
| 下视双目 | `/dev/video3` | 2560×960 左右拼接，每眼 1280×960 | `youlong/camera/down` |

下视尺寸、标定、CameraInfo 和 JPEG 无损旋转 180° 全部沿用
`uv_camera/stereos/down.yaml` 与现有驱动，设备路径在本 profile 中覆盖。
相机需要支持配置的 MJPG 尺寸；实际采集帧率由设备能力决定。

在仓库根目录构建并加载环境（按实际 ROS 版本选择 Foxy 或 Jazzy）：

```bash
source /opt/ros/jazzy/setup.bash
cd workspace_auv
colcon build --symlink-install --packages-up-to sl_record
source install/setup.bash
cd ..
source scripts/source_workspace.sh
```

运行环境需按仓库 README 准备 iceoryx2 Python binding；`source_workspace.sh` 会将
工作区 venv 中的 binding 加入当前 Python 路径。无需为结构光采集启动 go2rtc。

所有启动开关、设备路径、结构光采集参数和录制参数统一放在：
`workspace_auv/src/sl_camera/config/profiles/default.yaml`。
可直接编辑，也可复制为外部文件后传入绝对路径。主要配置为：

```yaml
cameras:
  sl:
    enabled: true
    device: /dev/video0
    width: 1280
    height: 720
    fps: 30.0
    startup_timeout_sec: 5.0
    reconnect_interval_sec: 1.0
  down:
    enabled: true
    device: /dev/video3
    camera_config_dir: ""
    startup_timeout_sec: 5.0
    reconnect_interval_sec: 1.0
```

外部文件还应保留原 profile 的 `schema_version: 1` 和 `recording` 配置。
`enabled: false` 只控制是否在这次 launch 中启动对应采集节点；
已由其他进程发布的 Iceoryx2 帧仍可被 `sl_record` 录制。
推荐在设备枚举不稳定时将路径改为 `/dev/v4l/by-id/...-video-index0`。

只启动采集、不录制：

```bash
ros2 launch sl_camera camera.launch.py
ros2 launch sl_camera camera.launch.py profile:=/absolute/path/collection.yaml
```

只启动结构光节点的旧 ROS 参数 YAML 用法仍然支持：

```bash
ros2 run sl_camera sl_camera --ros-args --params-file /absolute/path/sl_camera.yaml
```

`config/default.yaml` 是上述单节点入口的旧参数模板，不是共享 profile。
launch 的 `config:=...` 也保留为结构光节点参数覆盖；建议新配置统一使用 `profile:=...`。

每次成功读取后立即从 ROS 时钟取纳秒时间，写入 `FrameHeader.timestamp_ns`；
这是主机软件采集时间，不是相机硬件曝光时间。`capture_id` 在节点生命周期内递增，
`camera_group=3`、`stereo_pair_id=0`、`camera_info_version=0`。

状态发布在 `/auv/camera/sl/status`，包含帧数和距最后一帧的时长。打开失败会重试；
连续无有效帧达到默认 5 秒后重新打开，默认重试间隔 1 秒。Ctrl+C 关闭采集和发布器。
如果驱动阻塞在不可中断的读取中，节点在关闭等待超时后报告错误。

采集与录制一起启动的用法见 `sl_record/README.md`。若某路已由其他程序启动，
将本 profile 中该路的 enabled 设为 false，避免重复打开设备。

## V4L2 打开失败排查

`No config file was loaded` 是 Iceoryx 默认配置提示，不代表相机 profile 没有加载；
启动日志会打印实际 profile 路径和两路开关。`cannot open camera /dev/video0` 才是采集错误。
新日志会区分路径不存在、非字符设备、读写权限和系统 errno；设备存在但 OpenCV 仍打不开时，
需在目标 NVIDIA 主机检查：

```bash
ls -l /dev/video0 /dev/video3
id
fuser -v /dev/video0 /dev/video3
v4l2-ctl -d /dev/video0 --all --list-formats-ext
v4l2-ctl -d /dev/video3 --all --list-formats-ext
```

确认是 Video Capture 节点、支持目标 MJPG 模式、当前用户有读写权限，且没有旧采集程序占用。
原 `uv_camera` 下视 registry 的 device 是 `/dev/video0`，本采集 profile 已将其覆盖为
`/dev/video3`；单独启动其他旧配置时仍需检查该配置的设备路径。
容器中还必须显式映射这两个实际设备；profile 只能选择设备，不能替代 Docker 的设备映射。
