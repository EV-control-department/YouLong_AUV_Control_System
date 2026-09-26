# 启动关系图（当前分层图）

启动文件按 workspace 边界组织。相机采集、感知、推流在 real、SIM、HIL 中由
独立包启动；完整系统由 `uv_bringup` / `uv_sim_bringup` 编排。单独调试时分别启动
`uv_camera/camera_launch.py`、`uv_perception/perception_launch.py` 和 `uv_stream/stream_launch.py`。

```text
workspace_auv
  uv_bringup/real.launch.py
    ├── auv_description/description.launch.py
    ├── uv_hm/hardware_launch.py
    ├── uv_localization/localization_launch.py
    ├── uv_control/control_launch.py
    ├── uv_camera/camera_launch.py
    ├── uv_perception/perception_launch.py (可选)
    ├── uv_stream/stream_launch.py
    ├── uv_planning/planning_launch.py
    └── uv_task/task_launch.py

workspace_sim
  uv_sim/sim.launch.py (public world/vehicle entry)
    └── uv_sim_bringup/sim.launch.py
        ├── uv_sim_description/description.launch.py
        ├── stonefish_ros2/*
        ├── uv_sim_bridge/bridge.launch.py
        ├── uv_sim_degradation/degradation.launch.py (optional)
        ├── uv_localization/localization_launch.py
        ├── uv_camera/camera_launch.py
        ├── uv_perception/perception_launch.py (可选)
        ├── uv_stream/stream_launch.py
        └── 与 AUV 相同的控制/规划/任务启动文件

  uv_sim_bringup/hil.launch.py
    ├── Stonefish + uv_sim_bridge (只桥接控制/小型传感器元数据)
    ├── uv_camera/camera_launch.py
    ├── uv_perception/perception_launch.py (可选)
    ├── uv_stream/stream_launch.py
    └── MicroXRCEAgent + 上层控制/规划/任务
```

图像像素沿 `Stonefish/V4L2 → uv_camera → iceoryx2` 传递，不经过 ROS 2/DDS
图像话题。`uv_stream/camera_streamer` 从 iceoryx2 读取原图并编码 H.264，go2rtc
在 HTTP/API `:1984` 提供网页播放器、MSE/WebRTC 信令和 MP4 编码流，WebRTC
媒体使用 `:8555`；当前 H.264 源不提供 MJPEG，RTSP/TCP `:8554` 在应用配置中关闭。
统一录制由 `uv_record/record` 管理；`record_mode:=raw` 直接保存 iceoryx2 源帧，`record_mode:=go2rtc` 保存对齐到源帧时间戳的视频。

`hil.launch.py` 使用仿真描述和场景、真实 ZIT6 传输/代理，以及相同的 AUV
下游软件包。`uv_bringup` 不包含 Stonefish 或 `uv_sim` 运行时依赖。
`uv_sim_bringup` 保留旧 `scenario_desc` 转发入口；新的用户启动命令使用
`uv_sim sim.launch.py world:=... vehicle:=youlong`。

## TF 归属

`robot_state_publisher` 是唯一的固定变换发布者，并重映射到
`/auv/tf_static`。`uv_localization` 是唯一的动态
`odom -> base_link` 发布者，并重映射到 `/auv/tf`。
