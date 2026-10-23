# sl_camera 结构光单目采集

独立打开一台用于观察结构光的 V4L2 单目摄像头。默认请求 **1280×720、30 fps、MJPG**，
通过 Iceoryx2 服务 `youlong/camera/sl` 发布完整 JPEG，不解码、不旋转、不切分左右目。
相机必须支持配置的 MJPG 尺寸；实际采集帧率受设备能力影响。

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

编辑 `workspace_auv/src/sl_camera/config/default.yaml` 的 `device`，例如 `/dev/video4`，
推荐使用稳定的 `/dev/v4l/by-id/...-video-index0`。默认留空并报错，避免误开其他相机。
支持 `width`、`height`、`fps`、`startup_timeout_sec`、`reconnect_interval_sec` 参数。
参数类型遵循示例，帧率和超时使用浮点数。可以另存一份 YAML，再指定其绝对路径：

```bash
ros2 launch sl_camera camera.launch.py config:=/absolute/path/sl_camera.yaml
```

也可直接运行节点：

```bash
ros2 run sl_camera sl_camera --ros-args --params-file /absolute/path/sl_camera.yaml
```

每次成功读取后立即从 ROS 时钟取纳秒时间，写入 `FrameHeader.timestamp_ns`；
这是主机软件采集时间，不是相机硬件曝光时间。`capture_id` 在节点生命周期内递增，
`camera_group=3`、`stereo_pair_id=0`、`camera_info_version=0`。

状态发布在 `/auv/camera/sl/status`，包含帧数和距最后一帧的时长。打开失败会重试；
连续无有效帧达到默认 5 秒后重新打开，默认重试间隔 1 秒。Ctrl+C 关闭采集和发布器。
如果驱动阻塞在不可中断的读取中，节点在关闭等待超时后报告错误。

采集与录制一起启动的用法见 `sl_record/README.md`。容器运行时还需将 YAML 中的设备
映射进容器，并保证当前用户拥有读写权限；现有真机 Compose 的两路相机映射不会自动新增该设备。
