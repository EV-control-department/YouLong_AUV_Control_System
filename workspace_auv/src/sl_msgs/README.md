# sl_msgs

SL 多摄像头链路的独立 DDS 接口。像素通过 Iceoryx2 传输，DDS 只传摄像头清单。

- `CameraCalibration`：`eye`（mono/left/right）与标准 `sensor_msgs/CameraInfo`。
- `CameraDescriptor`：ID、名称、设备、Ice 服务、frame_id、完整图像及每目尺寸、
  请求/协商/观测帧率、MJPG/JPEG 格式、布局、启用/在线状态、状态描述、帧计数和标定。
- `CameraCatalog`：ROS 发布时间、发布实例 UUID、非零配置版本、配置/启用/在线数量及全部描述项。

默认话题 `/auv/sl_camera/cameras`，Reliable + Transient Local，深度 1。
启动立即发布，此后每秒及状态变化发布。禁用项和离线项仍包含在清单中。
`actual_fps=0` 表示未知或离线，`negotiated_fps=0` 表示设备未报告。
初始 `calibrations=[]`、`calibrated=false`，不构造假标定。

每次管理器启动使用新的 `source_instance_id` 和随机非零 uint32 `config_version`。
Ice 通用帧头的 `camera_group` 对应描述项 ID，`camera_info_version` 对应配置版本。
配置版本与是否已标定相互独立。录制器须校验版本，不把上一配置的残留帧与新参数混合。
不保证跨多个同时运行的管理器发现；默认一套管理器负责全部配置。

构建：`colcon build --packages-select sl_msgs`；
查看：`ros2 interface show sl_msgs/msg/CameraCatalog`。
