# workspace_auv 关键包职权：当前状态

盘点日期：2026-09-26。本文根据源码、package manifest 和真机/SIM/HIL launch 配置描述当前实现；不代表当前 ROS graph 中的实时进程状态。

## uv_hm

当前 executable 是 `hw_manager`。真机启动 `uv_bringup/real.launch.py` 在 `enable_hardware=true` 时会拉起它。当前职责是：

- 定时向 ZIT6 发布心跳；默认参数为 15 Hz。
- 默认订阅固件旧 `/zit6/state/*` 话题，并转发到 `/auv/hardware/zit6/state/*`；包含状态、位置、速度、推进器、心跳和 USBL。
- 监测 MCU 心跳、状态超时、电池电压、错误标志和推进器饱和，输出诊断日志。
- 通过 `legacy_state_topics` 选择订阅旧固件话题或 canonical 话题。

它当前不运行级联 PID、不做推进器混控，也不实现仿真 bridge。运动设定点由 `uv_control/basic_motion.py` 发布；仿真 bridge 在 `workspace_sim`。

`config/pid_parameters.json` 和 `config/pid_params.yaml` 仍被 `setup.py` 安装，但本仓库的运行节点没有读取它们；它们很可能是旧控制器/bridge 的遗留配置。`cycle_time_warn_threshold` 参数也只声明、没有运行时读取。模块注释曾写心跳为 10 Hz，与代码默认 15 Hz 不符；本次已将注释改成可配置、默认 15 Hz。

**当前职权：** ZIT6 状态适配与健康监控。暂时保留 `uv_hm` 包名；先确认 PID 文件没有固件部署或仓库外消费者，再处理其归属。

## uv_planning 与 uv_nav

`uv_planning` 是对外规划启动边界；`uv_nav` 已标记为弃用的过渡后端，但目前仍由 `uv_planning` 启动，因此暂时保留运行能力。新代码应依赖 `uv_planning`，不要新增对 `uv_nav` 的直接依赖。真机 launch 默认 `enable_nav=true`；仿真和 HIL launch 默认 `enable_nav=false`，但允许显式传入 true。

启用后，`navigator` 会订阅 `/auv/state/odom` 和 `/auv/perception/tracks`，以 0.5 m 栅格和 2.0 m 障碍安全半径运行 A*，发布 `/auv/control/trajectory`，并提供 `/auv/planning/navigate_to` 服务。

当前它还不是运动执行闭环：节点将起点和 A* 选出的一个下一航点写入路径消息，但没有调用 `BasicMotion` 跟随航点。若 A* 找不到路径，`AStarPlanner` 会返回目标点，服务仍可能报告规划成功。同时，`uv_task` 的 `navigate` 处理函数直接发送 `BasicMotion.SET`，因此 mission 运动会绕过规划节点。

**当前职权：** `uv_planning` 持有公开规划边界；`uv_nav` 是弃用但仍被调用的 A* 迁移实现。迁移完成前不要移除或停用 `uv_nav`。后续应先明确路径规划/跟随责任并迁走实现，再确认直接消费者已迁移后删除该包。

## uv_camera 与 uv_image_transport

当前采集入口是 `uv_camera.driver`：真机从 V4L2 采集前视/下视双目图像，仿真从 POSIX shared memory 读取；参数和内参由相机 YAML 提供，并发布各眼 `CameraInfo`。图像帧通过独立的 `uv_image_transport` Python 包接入 iceoryx2 服务，服务名仍由 `auv_protocol.topics` 定义。当前不发布 DDS `sensor_msgs/Image`。

**职责边界：** `uv_camera` 负责相机采集、仿真/真机输入适配、相机配置、CameraInfo 和原始帧发布；`uv_image_transport` 只负责 iceoryx2 图像帧读写，不是 ROS 节点或通用 transport 框架。检测、当前定位和跟踪由 `uv_perception` 启动；视频显示由 `uv_stream` 处理。`uv_record` 管理 session、非图像 rosbag、运行日志和图像归档；每个会话只选 raw 源帧或 go2rtc 视频一种图像路径。

旧传输实现、相机聚合入口和组合感知/单目定位链路均已移除；`uv_perception.object_localizer` 是现用定位节点，仍保留。模型类别映射和 YOLO 权重归属 `uv_perception`；相机注册表与 `CameraExtrinsicsProvider` 仍由 `uv_camera` 提供。

**依赖方向：** `uv_camera`、`uv_perception`、`uv_record` 和 `uv_stream` 均直接依赖 `uv_image_transport`，传输实现通过新包共享。迁移没有保留旧 Python 导入兼容层或相机包 GUI 命令别名。

## uv_mapping

`uv_mapping` 已标记为弃用。它只有 `contracts.py`、package/setup 文件和一个 topic 前缀检查；`contracts.py` 只是从 `auv_protocol` 重新导出三个 mapping topic 常量。包内没有 mapping 节点、launch、消息定义或运行入口；本仓库内没有其他运行代码依赖它。

**当前职权：** 无运行时职权，是应迁除的 topic 常量兼容 shim。新代码直接从 `auv_protocol.topics` 导入常量。确认仓库外没有消费者后即可移除本包；topic 名称仍由 `auv_protocol` 管理。

## 后续整理顺序

1. 更新 `uv_hm/package.xml` 和旧架构说明，使描述符合当前适配/监控职责。
2. 确认旧 PID 配置没有外部用途，再从 `uv_hm` 安装清单中移除或迁到实际控制器归属包。
3. 迁移 `uv_nav` 的 A* 实现及其直接调用者，完成路径执行闭环定义后再移除兼容后端。
4. 检查下游工作区和部署脚本是否依赖 `uv_mapping`；若没有，移除 shim，保留 `auv_protocol` 中的 topic 常量。