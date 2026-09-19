# YouLong AUV ROS 2 话题协议 V1

所有新建的公开 ROS 2 话题、服务和 Action 都必须位于 `/auv` 命名空间下。
此前缀属于接口定义的一部分，而不是启动时临时约定的命名空间。

## 标准接口

| 层级 | 接口 |
| --- | --- |
| 相机 | `/auv/sensors/camera/front/{left,right}/image_raw` 和 `camera_info` |
| 相机 | `/auv/sensors/camera/downward/{left,right}/image_raw` 和 `camera_info` |
| 相机调试 | `/auv/sensors/camera/{front,downward}/image_stitched` |
| 传感器 | `/auv/sensors/imu/data`、`/auv/sensors/dvl/{velocity,altitude}`、`/auv/sensors/pressure`、`/auv/sensors/usbl/measurement` |
| 感知 | `/auv/perception/detections/{front/left,front/right,downward/left,downward/right}` |
| 感知 | `/auv/perception/lines/*`、`/auv/perception/observations`、`/auv/perception/targets` |
| 状态 | `/auv/state/odom`、`/auv/state/twist`、`/auv/state/health`、`/auv/state/reset` |
| TF | `/auv/tf`、`/auv/tf_static` |
| 控制 | `/auv/control/motion_command`、`/auv/control/trajectory`、`/auv/control/status`、`/auv/basic_motion` |
| 任务流程 | `/auv/mission/{run,stop,status,execute}` |
| 硬件 | `/auv/hardware/zit6/cmd/*` 和 `/auv/hardware/zit6/state/*` |
| 仿真 | `/auv/sim/raw/*`、`/auv/sim/ground_truth/*`、`/auv/sim/actuators/{thruster_command,thruster_state}` |
| 仿真运行时 | `/auv/sim/performance`（Stonefish real-time factor）、`/auv/sim/control_performance`（控制环频率/抖动） |
| 仿真退化 | `/auv/sim/degraded/*`、`/auv/sim/degradation/events` |
| 评测 | `/auv/evaluation/{metrics,events}` |

`/auv/state/odom` 和 `/auv/state/twist` 是正式控制、规划和 Mission 代码唯一
允许消费的估计状态接口。当前 `PoseInfo` 仍是 odom 的兼容消息类型（其 yaw
字段为度）；`uv_localization` 内部保持 SI 单位，后续可替换为带协方差的
正式状态消息而不改变话题名称。`/auv/state/reset` 由 START 动作触发，用于
建立 odom 原点。

## 兼容接口

旧的 `/task/*`、`/basic_motion*` 和 `/zit6/*` 接口只能由兼容适配器暴露。
新代码必须从 `auv_protocol.topics` 导入接口名称，不得引入未加 `/auv`
前缀的话题。

## 坐标系与单位

AUV 机体坐标系采用 FRD（`x` 向前、`y` 向右、`z` 向下）。内部角度统一使用
弧度；现有 ZIT6 `PoseInfo` 兼容字段在该消息淘汰前仍使用角度制。
