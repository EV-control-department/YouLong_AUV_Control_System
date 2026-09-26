# 当前视觉数据链路

本文说明当前源码中的相机、传输、感知和视频职责。详细定位算法方案见“目标物三维位置估计”文档，但其中早期参数和算法草案不代表现行实现。

## 数据流

1. uv_camera.driver 通过 V4L2 读取实机图像，或从仿真共享内存读取图像；相机配置与 CameraInfo 由 uv_camera 管理。
2. uv_image_transport.iceoryx2 将 BGR8 拼接帧写入 youlong/camera/front 和 youlong/camera/down。帧头包含采集 ID、时间戳、双目配对 ID、相机标识、尺寸、stride 和标定版本。
3. uv_perception.object_detector 读取两路 iceoryx2 服务，发布检测结果、管线状态和 ArUco ID。
4. uv_perception.object_localizer 结合左右目检测、CameraInfo 和 TF 生成几何测量；uv_perception.object_estimator 将测量关联为多帧目标状态。
5. uv_stream.camera_streamer 读取原始帧并编码视频；标注流匹配 uv_perception 的检测结果。uv_record 统一管理 session；raw 模式直接记录原始帧，go2rtc 模式记录视频并映射源帧时间戳。

相机图像不通过 ROS 2/DDS Image 话题传输。CameraInfo 和检测/测量/目标状态仍通过 ROS 话题传递。

## 包职责

- uv_camera：相机采集、仿真/实机输入适配、相机 YAML、CameraInfo 和 TF 工具。
- uv_image_transport：iceoryx2 相机帧读写；不创建 ROS 节点，也不是通用传输框架。
- uv_perception：检测、几何测量、目标关联与 GUI；模型权重和类别映射也由该包安装。
- uv_stream：原始/标注视频流。
- uv_record：session、非图像 rosbag、进程日志和图像归档；每个 session 选 raw 源帧或 go2rtc 视频。

## 启动

单独启动组件：

```bash
ros2 launch uv_camera camera_launch.py sim_mode:=false
ros2 launch uv_perception perception_launch.py
ros2 launch uv_stream stream_launch.py
ros2 run uv_perception perception_gui
```

完整系统应通过 uv_bringup 或 uv_sim_bringup 启动。

## 主要 ROS 接口

- 相机内参：/auv/sensors/camera/{front,downward}/{left,right}/camera_info。
- 检测：/auv/perception/detections。
- 几何测量：/auv/perception/measurements。
- 多帧目标状态：/auv/perception/tracks。
- iceoryx2 图像服务：youlong/camera/front、youlong/camera/down。

相机及感知启动参数以 docs/node_parameters.md 和各包 launch 文件为准。
