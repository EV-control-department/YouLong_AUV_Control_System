# ROS 工作空间参数清单

本文件按当前源码整理仓库 ROS 2 包的 launch 参数、节点参数、录制器命令行选项和主要 YAML 配置入口。固件工程内部的私有配置不属于 ROS 参数。

## 弃用状态说明

| 标记 | 含义 |
|---|---|
| 否 | 源码没有标记弃用，且参数当前有实际用途。 |
| 是 | 源码明确标记为 deprecated / 已弃用；新配置不应依赖它。 |
| 兼容项 | 为旧配置保留；可能无效果，或只在新参数缺省时作为回退值。 |
| 未接线 | 没有弃用标记，但当前实现没有消费它，修改不会产生预期效果。 |

launch 参数通常以字符串传入；节点参数按声明的类型解释。

## 包覆盖范围

| 包 | 工作区 | 参数入口 / 状态 |
|---|---|---|
| auv_description | workspace_auv | description.launch.py：use_sim_time；模型由 URDF 提供 |
| auv_protocol | workspace_auv | 话题和服务常量；无运行参数 |
| uv_bringup | workspace_auv | real、readiness、observability 启动参数 |
| uv_camera | workspace_auv | 相机节点参数和相机 YAML |
| uv_control | workspace_auv | basic_motion 节点和 launch 参数 |
| uv_hm | workspace_auv | hw_manager 节点参数、实车 profile |
| uv_image_transport | workspace_auv | 图像传输库；无运行参数 |
| uv_localization | workspace_auv | estimator 节点和 launch 参数 |
| uv_mapping | workspace_auv | 已弃用的 topic shim；无运行参数 |
| uv_msgs | workspace_auv | 消息和服务定义；无运行参数 |
| uv_nav | workspace_auv | **包已弃用**；A* 过渡实现，建议从 uv_planning 启动 |
| uv_perception | workspace_auv | 检测、定位、跟踪和 GUI 参数 |
| uv_planning | workspace_auv | 导航开关和参数文件入口；当前委托给 uv_nav |
| uv_record | workspace_auv | record 可执行程序命令行选项 |
| uv_stream | workspace_auv | go2rtc 可执行文件路径 |
| uv_task | workspace_auv | task_runner 节点、任务和 mission YAML |
| stonefish_ros2 | workspace_sim | 仿真场景启动参数；render_fps 节点参数 |
| uv_sim | workspace_sim | 对外仿真入口；部分继承参数当前没有转发 |
| uv_sim_assets | workspace_sim | 仿真车辆和传感器静态配置；无 ROS 节点参数 |
| uv_sim_bridge | workspace_sim | sim_bridge 节点和相机适配参数 |
| uv_sim_bringup | workspace_sim | SIL、HIL、实验和传感器退化 launch 参数 |
| uv_sim_degradation | workspace_sim | 相机、DVL、IMU、USBL 退化节点参数 |
| uv_sim_description | workspace_sim | description.launch.py：use_sim_time |
| uv_sim_evaluation | workspace_sim | evaluator 节点参数 |
| zit6_control_core | workspace_sim | 控制算法库；无 ROS 节点参数 |
| zit6_interfaces | third_party/AUV_zit6_cmake | 消息/服务接口；无运行参数 |
| upper_examples | third_party/AUV_zit6_cmake | 本机 config.json 存在时才构建；未发现 ROS 参数声明 |

prepare_workspace.sh 会检查子模块里的 micro_ros_stmcube 工程，但该工程不在脚本的 colcon 构建路径中。upper_examples 依赖本机 UserApp/Config/config.json；这里不展开固件私有配置字段。

## workspace_auv

### uv_bringup：真机入口 real.launch.py

| 参数 | 默认值 | 含义 | 弃用 |
|---|---|---|---|
| profile | real_default | 实车参数组，可选 real_default、real_safe；加载 uv_hm 和 uv_camera 对应 YAML | 否 |
| mission_file | uv_task/config/missions/robocup_26.yaml | task_runner 使用的 mission 流程或单项任务 YAML | 否 |
| enable_ai | true | 启动感知检测、定位和跟踪 | 否 |
| enable_nav | true | 启动导航 | 否 |
| enable_task | false | 启动任务执行器 | 否 |
| enable_motion | true | 启动 basic_motion | 否 |
| enable_hardware | true | 启动硬件管理器 | 否 |
| enable_perception_gui | false | 启动感知观测 GUI | 否 |
| enable_stream | true | 启动 uv_stream/go2rtc 视频流 | 否 |
| camera_config_dir | 空 | 覆盖相机 YAML 目录；目录中应有 front.yaml、down.yaml | 否 |

### uv_bringup：观测、预览和录制参数

以下参数由 real.launch.py 和仿真入口共用；独立运行 observability.launch.py 时也可设置。

| 参数 | 默认值 | 含义 | 弃用 |
|---|---|---|---|
| stream_annotated | true | 名义上控制是否生成带检测标注的视频流；当前源码没有消费此值 | 未接线 |
| enable_preview | true | 历史预览开关；当前不起作用。视频服务启动由 enable_stream 控制 | 兼容项 |
| annotated_max_width | 1280 | 名义上限制标注画面的最大宽度；当前源码没有消费此值 | 未接线 |
| preview_port | 1984 | 传给 uv_record 的 go2rtc 访问端口；不修改 go2rtc 服务自身监听配置 | 否 |
| gortc_http_port | 1984 | 历史 HTTP 端口参数；当前没有代码读取此参数 | 未接线 |
| record_session | false | 是否创建统一录制 session 并启动 uv_record | 否 |
| record_root | 仓库根目录下 records/sessions | session 输出根目录 | 否 |
| record_mode | raw | 图像录制来源：raw 或 go2rtc | 否 |
| go2rtc_stream_mode | unannotated | go2rtc 录制流：unannotated、annotated 或 both | 否 |
| go2rtc_video_format | jpeg | go2rtc 归档格式：jpeg 或 ts | 否 |
| camera_stitch_fps | 5.0 | recorder 默认视频输出帧率；record_video_fps 未指定时采用此值 | 否 |
| record_video_fps | camera_stitch_fps | 视频归档帧率；建议与相机拼接输出帧率一致 | 否 |
| record_video_codec | libx264 | FFmpeg 编码器；主要供旧 ts/H.264 录制路径使用 | 兼容项 |
| record_image_topics | false | 请求将图像消息写入 rosbag；当前始终排除图像消息，此参数被忽略 | 是 |
| video_segment_seconds | 2.0 | 视频分段时长，秒 | 否 |
| bag_segment_seconds | 10.0 | rosbag 分段时长，秒 | 否 |
| record_bag_storage | auto | rosbag 存储后端：auto、sqlite3 或 mcap | 否 |
| record_use_sim_time | false | recorder 是否使用仿真时间 | 否 |

go2rtc 的实际流和监听端口配置位于 uv_stream/config/go2rtc.yaml。enable_preview:=false 不会关闭视频流；支持该开关的入口应使用 enable_stream:=false。

### uv_bringup：readiness.launch.py

| 参数 | 默认值 | 含义 | 弃用 |
|---|---|---|---|
| phase | 必填 | 就绪检查阶段：backend、control、sensors 或 perception | 否 |
| require_ai | true | 检查阶段是否要求 AI 感知节点就绪 | 否 |
| timeout | 120.0 | 等待就绪的最大时间，秒 | 否 |

### uv_camera

| 参数 | 默认值 | 含义 | 弃用 |
|---|---|---|---|
| sim_mode | false | true 从仿真共享内存取图；false 使用真机 V4L2 设备 | 否 |
| enable_front | true | 启用前视相机 | 否 |
| enable_down | true | 启用下视相机 | 否 |
| camera_config_profile | auto | auto 根据 sim_mode 选 sim/real；也可显式选择 profile | 否 |
| camera_config_dir | 空 | 覆盖相机配置 YAML 目录 | 否 |
| front_camera_device | 空 | 真机前视 V4L2 设备路径覆盖项；空值使用 YAML 中的 device | 否 |
| down_camera_device | 空 | 真机下视 V4L2 设备路径覆盖项；空值使用 YAML 中的 device | 否 |
| camera_startup_timeout_sec | 5.0 | 等待相机源就绪的超时，秒 | 否 |
| camera_info_version | 1 | 写入 iceoryx2 图像帧头的标定版本 | 否 |

### uv_control 和 uv_hm

| 包 / 参数 | 默认值 | 含义 | 弃用 |
|---|---|---|---|
| uv_control.enable_motion | true | control_launch.py 是否启动 basic_motion | 否 |
| uv_control.sim_mode | false | basic_motion 按真机或仿真模式运行 | 否 |
| uv_control.profile_params | 空 | 可选 ROS 参数 YAML 路径；当前 real/SIL 入口传空值 | 否 |
| uv_hm.enable_hardware | true | hardware_launch.py 是否启动 hw_manager | 否 |
| uv_hm.profile_params | 空 | 可选 ROS 参数 YAML 路径；real.launch.py 根据 profile 自动传入 | 否 |
| uv_hm.heartbeat_rate | 15.0 | 向控制板发送心跳的频率，Hz | 否 |
| uv_hm.watchdog_timeout | 7.0 | MCU 心跳看门狗超时，秒 | 否 |
| uv_hm.arm_mode | 1 | 控制板 arm 模式；1 为 normal，3 为 force | 否 |
| uv_hm.battery_low_threshold | 14.0 | 低电量阈值 | 否 |
| uv_hm.cycle_time_warn_threshold | 100.0 | 控制周期告警阈值，毫秒 | 否 |
| uv_hm.legacy_state_topics | true | 接收旧版 /zit6/state/* MCU 状态话题 | 否（兼容接口；源码未标记弃用） |

real_safe profile 将 watchdog_timeout 设为 3.0、battery_low_threshold 设为 15.0；real_default 对应 7.0 和 14.0。profile 文件位于 uv_hm/config/profiles/。

### uv_localization

| 参数 | 默认值 | 含义 | 弃用 |
|---|---|---|---|
| sim_mode | false | 选择真机或仿真状态处理路径 | 否 |
| publish_tf | true | 是否发布定位 TF | 否 |
| estimator | bootstrap | 定位后端；当前实现只支持 bootstrap | 否 |
| dvl_topic | /auv/sensors/dvl/velocity | DVL 速度输入重映射 | 否 |
| imu_topic | /auv/sensors/imu/data | IMU 输入重映射 | 否 |
| usbl_topic | /auv/sensors/usbl/measurement | USBL 输入重映射 | 否 |
| publish_rate | 30.0 | 状态发布频率，Hz | 否 |
| position_timeout | 2.0 | 判定位置输入超时的时长，秒 | 否 |

dvl_topic、imu_topic、usbl_topic 是 launch 层的话题重映射，不是节点参数。

### uv_perception：perception_launch.py 参数

| 参数 | 默认值 | 含义 | 弃用 |
|---|---|---|---|
| model_path | 空（自动搜索权重） | YOLO 权重文件路径 | 否 |
| confidence | 0.5 | 检测最低置信度 | 否 |
| stereo_baseline_m | 0.10 | 三角测量使用的左右相机光心间距，米。相机内参不包含该间距；当前实现直接使用此参数，应按双目标定或 TF 外参配置 | 否 |
| association_distance_m | 1.5 | 对 `multi_instance: true` 类别做多实例轨迹关联时，新观测与已有轨迹均有位置的最大欧氏距离，米；低于阈值才会匹配。单实例静态类别不使用此门限 | 否 |
| bearing_association_distance_m | 0.35 | 方位测量关联距离，米 | 否 |
| world_frame | odom | 定位结果和目标几何输出使用的 TF 坐标系。相机坐标结果会按观测时间通过 TF 转到该坐标系；估计器也要求使用同一坐标系。通常 odom 表示局部连续坐标，不代表地理绝对坐标 | 否 |
| edge_margin_px | 8.0 | 检测框边缘的像素安全边距 | 否 |
| edge_margin_ratio | 0.02 | 检测框边缘的比例边距 | 否 |
| stereo_epipolar_tolerance_px | 10.0 | 左右目垂直极线误差容限，像素 | 否 |
| max_stereo_range_m | 30.0 | 双目定位最大距离，米 | 否 |
| min_parallax_deg | 5.0 | 多视角定位所需最小视差角，度 | 否 |
| huber_delta | 2.5 | 鲁棒估计器的 Huber 阈值 | 否 |
| pose_translation_sigma_m | 0.03 | 机器人平移不确定度，米 | 否 |
| pose_rotation_sigma_deg | 1.0 | 机器人旋转不确定度，度 | 否 |
| extrinsic_translation_sigma_m | 0.005 | 相机外参平移不确定度，米 | 否 |
| extrinsic_rotation_sigma_deg | 0.5 | 相机外参旋转不确定度，度 | 否 |
| enable_gui | false | 是否启动感知 GUI | 否 |

#### 感知节点声明的其它参数

下列参数用于节点构造或可通过节点参数覆盖；它们没有全部暴露为 perception_launch.py 的 launch 参数。

| 节点 / 参数 | 默认值 | 含义 | 弃用 |
|---|---|---|---|
| object_detector.aruco_fps | 10.0 | ArUco 处理频率上限，Hz | 否 |
| object_detector.guide_line_class_id | 4 | 导引线模型类别 ID | 否 |
| object_detector.gate_front_class_id | 3 | 正面门框模型类别 ID | 否 |
| object_detector.gate_feature_mode | auto | 门框特征识别模式 | 否 |
| object_detector.device | 空 | 推理设备；空值交由运行环境选择 | 否 |
| object_localizer.pair_timeout_s | 0.05 | 等待左右目检测配对的时间，秒 | 否 |
| object_localizer.down_ground_z | 0.0 | 下视投影使用的地面 Z 坐标，米 | 否 |
| object_estimator.stale_after_s | 0.5 | 目标观测多旧后标记为过期，秒 | 否 |
| object_estimator.lost_after_s | 2.0 | 目标多旧后标记为丢失，秒 | 否 |
| perception_gui.refresh_period_ms | 150 | GUI 刷新周期，毫秒 | 否 |
| perception_gui.measurement_history_limit | 500 | GUI 保留的测量历史条数 | 否 |
| perception_gui.association_distance_m | 1.5 | GUI 将观测对应到已有轨迹以保持显示标签时使用的最大距离，米 | 否 |

### uv_task：task_launch.py / task_runner

| 参数 | 默认值 | 含义 | 弃用 |
|---|---|---|---|
| enable_task | true（独立 task_launch） | 是否启动 task_runner；总控入口会覆写默认值 | 否 |
| profile_params | 空 | 可选 ROS 参数 YAML 路径；当前总控入口传空值 | 否 |
| camera_config_profile | auto | 相机配置 profile；总控入口按 real/sim 传入 | 否 |
| camera_config_dir | 空 | 覆盖相机 YAML 目录 | 否 |
| mission_file | uv_task/config/missions/robocup_26.yaml | mission 流程或单项任务配置文件 | 否 |
| debug_mode | false | true 时启动后不自动加载并运行 mission_file，且启用 `/auv/mission/execute` 单任务调试服务；false 时启动时自动加载并运行 mission_file。`/auv/mission/run` 服务仍可手动加载并运行任务列表 | 否 |
| camera_base_frame | base_link | 任务视觉换算使用的机体坐标系 | 否 |
| camera_tf_timeout_sec | 5.0 | 等待相机 TF 的最长时间，秒 | 否 |
| camera_tf_retry_period_sec | 0.1 | 重试查找相机 TF 的间隔，秒 | 否 |

#### `/auv/mission` 接口

`/auv/mission` 是任务执行器的 ROS 命名空间前缀，不是单个参数或服务。`/auv` 是机器人系统根路径，`/mission` 下按用途放置启动、停止、单任务调试和状态接口。

| 名称 | ROS 类型 | 用途 |
|---|---|---|
| `/auv/mission/run` | `uv_msgs/srv/RunTask` | `start=true` 时加载并运行请求指定的任务/mission 配置；`start=false` 时请求停止任务列表 |
| `/auv/mission/stop` | `std_srvs/srv/Trigger` | 紧急停止当前活动任务和运动 |
| `/auv/mission/execute` | `uv_msgs/srv/ExecTask` | `debug_mode=true` 时执行一个指定任务，可传 JSON 参数和超时 |
| `/auv/mission/status` | `uv_msgs/msg/TaskStatus` 话题 | 发布当前任务状态，例如 idle、running、paused、done、error |

目标类别等行为由具体任务 YAML 参数决定，例如 `26rb_grab_ball` 的 `ball_color`；无需单独的 `target_id` launch 参数。

mission 根字段为 mission，任务序列位于 mission.tasks。每项可包含 name、config、initial、on_failure。initial.pose 可指定运动命令、坐标轴和目标 [x,y,z,yaw]；initial.params 覆盖该任务配置。on_failure 可按失败代码覆盖参数或目标位姿。

单项任务 YAML 使用 task 和 params 两个根字段。任务名及 params 字段类型由 [config_loader.py](../workspace_auv/src/uv_task/uv_task/config_loader.py) 中的 TASK_SCHEMAS 校验；当前各任务文件中的注释说明每项具体作用：

- [start.yaml](../workspace_auv/src/uv_task/config/tasks/start.yaml)
- [return_origin.yaml](../workspace_auv/src/uv_task/config/tasks/return_origin.yaml)
- [btravelx.yaml](../workspace_auv/src/uv_task/config/tasks/btravelx.yaml)
- [setz.yaml](../workspace_auv/src/uv_task/config/tasks/setz.yaml)
- [26rb_hit_balls.yaml](../workspace_auv/src/uv_task/config/tasks/26rb_hit_balls.yaml)
- [26rb_gate_task.yaml](../workspace_auv/src/uv_task/config/tasks/26rb_gate_task.yaml)
- [26rb_find_collection_frame.yaml](../workspace_auv/src/uv_task/config/tasks/26rb_find_collection_frame.yaml)
- [26rb_grab_ball.yaml](../workspace_auv/src/uv_task/config/tasks/26rb_grab_ball.yaml)
- [26rb_drop_ball_target_rack.yaml](../workspace_auv/src/uv_task/config/tasks/26rb_drop_ball_target_rack.yaml)

已知任务兼容/弃用字段：

| 字段 | 含义与当前行为 | 弃用 |
|---|---|---|
| mission 项中的 params | 历史初始覆盖写法；建议迁移至 initial.params。两者都提供时 initial.params 优先 | 兼容项 |
| 26rb_gate_task.min_gate_extent_fraction | 旧门框候选筛选阈值；当前不再用于拒绝候选 | 兼容项 |
| 26rb_gate_task.search.stop_height_fraction | 旧搜索提前停止阈值；当前流程会完成扫描后选择门框 | 兼容项 |
| 26rb_drop_ball_target_rack 的 horizontal_servo_* / horizontal_command_timeout | 旧水平伺服参数；对应 down_visual_servo_* 未提供时作为回退值 | 兼容项 |

### uv_record：record 命令行选项

record 是受 launch 管理的进程，不是 ROS 节点。launch 会转发其中一部分选项。

| 选项 | 默认值 | 含义 | 弃用 |
|---|---|---|---|
| --session-dir | 空 | 继续写入指定 session 目录；优先于 output-root | 否 |
| --output-root | 仓库根目录下 records/sessions | 新 session 输出根目录 | 否 |
| --host | 127.0.0.1 | 连接 go2rtc 的主机地址 | 否 |
| --port | 1984 | 连接 go2rtc 的 HTTP 端口 | 否 |
| --record-mode | raw | 图像录制路径：raw 源帧或 go2rtc 视频 | 否 |
| --segment-duration | 2.0 | 视频分段时长，秒 | 否 |
| --bag-duration | 10.0 | rosbag 分段时长，秒 | 否 |
| --bag-storage | auto | rosbag 存储插件：auto、sqlite3 或 mcap | 否 |
| --go2rtc-video-format | jpeg | go2rtc 归档格式；ts 保留旧 H.264 转码路径 | 兼容项 |
| --video-fps | 10.0（独立运行）；launch 默认跟随 camera_stitch_fps | 视频输出帧率 | 否 |
| --go2rtc-stream-mode | unannotated | 录制 unannotated、annotated 或 both 流 | 否 |
| --topic-regex | 内置元数据 topic 正则 | rosbag 纳入的话题过滤表达式 | 否 |
| --record-image-topics | false | 图像消息总是从 rosbag 排除，此选项被忽略 | 是 |
| --use-sim-time | false | 是否按仿真时钟记录 | 否 |
| --video-codec | libx264 | FFmpeg 编码器；用于 ts 路径 | 兼容项 |
| --ffmpeg | ffmpeg | FFmpeg 可执行文件路径 | 否 |

### uv_record：player 和 recover 子命令

| 子命令 / 参数 | 默认值 | 含义 | 弃用 |
|---|---|---|---|
| player.session_dir | 必填 | 要播放的录制 session 目录或可解析的 session 名称 | 否 |
| recover.--root | 仓库根目录下 records/sessions | 扫描并恢复该根目录下可恢复的 session | 否 |
| recover.--session-dir | 空 | 只恢复指定 session；指定后不扫描 root | 否 |

### uv_stream：go2rtc 和 camera_streamer

| 参数 | 默认值 | 含义 | 弃用 |
|---|---|---|---|
| go2rtc_executable | 自动查找仓库内二进制；否则使用 PATH 中的 go2rtc | go2rtc 服务程序路径 | 否 |
| camera_streamer.--camera | 必填：front 或 down | 选择相机输入 | 否 |
| camera_streamer.--mode | raw | 输出 raw 或 annotated 视频 | 否 |
| camera_streamer.--output-fps | 10.0 | 输出视频帧率上限，Hz | 否 |
| go2rtc api.listen | :1984 | go2rtc HTTP API / WebRTC signaling 监听地址 | 否 |
| go2rtc webrtc.listen | :8555 | WebRTC 媒体监听地址 | 否 |

四条默认 go2rtc 流由 uv_stream/config/go2rtc.yaml 定义：front/down 使用 raw，front_annotated/down_annotated 使用 annotated。当前 YAML 未给 camera_streamer 指定 output-fps，因此采用 10.0。

### 其它 workspace_auv 包

| 包 / 参数 | 默认值 | 含义 | 弃用 |
|---|---|---|---|
| auv_description.use_sim_time | false | 真机描述节点是否使用仿真时钟 | 否 |
| auv_description.robot_description | 从包内 URDF 文件读取 | 机器人描述节点启动时载入的 URDF 内容 | 否 |
| uv_planning.enable_nav | true | 是否启动 navigator | 否 |
| uv_planning.profile_params | 空 | 可选 navigator ROS 参数文件 | 否 |
| uv_nav.enable_nav | true | 旧 navigation_launch.py 是否启动 navigator | 兼容项 |
| uv_nav.profile_params | 空 | 旧导航启动器的参数文件路径 | 兼容项 |

uv_nav 包及 nav_launch.py 已标记弃用；新集成应使用 uv_planning。当前 navigator 节点没有 declare_parameter 声明。uv_msgs、auv_protocol、uv_image_transport 没有 ROS 参数；uv_mapping 是已弃用的兼容重导出包。

### 相机 YAML 配置字段

文件为 uv_camera/config/cameras/front.yaml 和 down.yaml。顶层包含 schema_version、camera 和 profiles.sim / profiles.real。

| 字段 | 含义 | 弃用 |
|---|---|---|
| capture_resolution | 左右目拼接后采集分辨率 [宽,高] | 否 |
| eye_resolution | 单目分辨率 [宽,高] | 否 |
| image_topic | 拼接图像逻辑 topic | 否 |
| eye_image_topics | 左右目输入 topic | 否 |
| stereo_info_topic | 双目标定信息 topic | 否 |
| camera_info_topics | 左右目 CameraInfo topic | 否 |
| device | 真机 V4L2 设备路径；sim profile 可为 null | 否 |
| calibration_source | yaml 或 sim_camera_info | 否 |
| intrinsics.left/right.matrix | 左右目 3×3 相机内参矩阵 K | 否 |
| intrinsics.left/right.distortion | 左右目畸变参数 D | 否 |
| calibration_npz | 旧 npz 标定文件 | 是；加载器拒绝 |
| calibration_source=npz | 旧 npz 标定来源 | 是；加载器拒绝 |
| extrinsics、intrinsics.translation、intrinsics.optical_to_body | 旧相机安装外参；现从 URDF/TF 获取 | 是；加载器拒绝 |

## workspace_sim

### uv_sim_bringup：SIL 仿真启动参数

| 参数 | 默认值 | 含义 | 弃用 |
|---|---|---|---|
| profile | sim_dev | 参数组：sim_dev 或 sim_ci | 否 |
| mission_file | robocup_26.yaml | 任务流程或单项任务配置 | 否 |
| enable_ai | true | 启动感知节点 | 否 |
| enable_motion | true | 启动 basic_motion | 否 |
| enable_nav | false | 启动导航 | 否 |
| enable_task | false | 启动任务执行器 | 否 |
| scenario_desc | worlds/guoshui_2026/guoshui_2026_cruise_seeded.scn | Stonefish 场景路径；公开入口通常使用 world 名称 | 否（实现层仍在用） |
| scene_seed | 0 | 场景生成 seed | 否 |
| simulation_rate | 100.0 | Stonefish 仿真频率，Hz | 否 |
| sim_window_width | 960 | 仿真窗口宽度，像素 | 否 |
| sim_window_height | 540 | 仿真窗口高度，像素 | 否 |
| render_quality | low | GPU 渲染质量 | 否 |
| render_fps | 30.0 | Stonefish 窗口刷新帧率上限 | 否 |
| gpu | true | true 使用 GPU Stonefish；false 使用无 GPU 版本 | 否 |
| gpu_backend | auto | OpenGL 供应方：auto、nvidia 或 system | 否 |
| camera_stitch_fps | 5.0 | 旧桥接帧率值会被相机适配器忽略；同时作为 recorder 的 record_video_fps 默认值 | 兼容项 |
| publish_raw_camera_topics | false | 将逐目图像发布为 DDS topic；当前架构使用共享内存，参数已弃用 | 是 |
| ai_inference_fps | 3.0 | 名义上的感知推理频率上限；当前仿真 launch 没有传给检测节点 | 未接线 |
| inference_threads | 2 | 名义上的 PyTorch CPU 线程上限；当前没有设置到运行环境 | 未接线 |
| ai_confidence | 0.8 | 仿真总控传给检测器的置信度阈值 | 否 |
| gate_feature_mode | auto | 名义上的门框特征模式；当前仿真 launch 没有传给检测节点 | 未接线 |
| startup_timeout | 120.0 | 名义上的启动就绪检查超时；当前仿真 launch 没有消费此值 | 未接线 |
| enable_perception_gui | false | 是否启动感知 GUI | 否 |
| enable_stream | true | 是否启动 uv_stream/go2rtc | 否 |
| enable_evaluation | true | 是否启动仿真评估器 | 否 |
| estimator | bootstrap | 定位算法；当前只支持 bootstrap | 否 |
| enable_degradation | false | 是否插入传感器退化节点 | 否 |
| evaluation_output_dir | 空 | 评估结果目录 | 否 |
| evaluation_run_id | 空 | 评估运行 ID | 否 |
| camera_config_dir | 空 | 覆盖相机 YAML 目录 | 否 |

仿真入口也接受前文观测和录制参数。

### uv_sim：公开 world / vehicle 启动入口

| 参数 | 默认值 | 含义 | 弃用 |
|---|---|---|---|
| profile | sim_dev | 参数组；还可选 guoshui、guoshui_cruise、guoshui_cruise_seeded、sauvc_finals、sauvc_qualification 场景预设 | 否 |
| world | 空 | 公开世界名或 uv_sim_assets 中的相对场景路径 | 否 |
| vehicle | youlong | 车辆名；当前只接受 youlong | 否 |
| scenario_desc | 空 | 直接指定 Stonefish 场景；旧接口，不能与 world 同时指定 | 是 |

uv_sim/launch/bridge.launch.py 另有一组与 uv_sim_bridge.launch.py 相同的参数：hil_mode=false、camera_stitch_fps=10.0、publish_raw_camera_topics=false、profile_params=空；camera_stitch_fps 是兼容项，publish_raw_camera_topics 已弃用。

uv_sim/launch/sim.launch.py 目前消费 profile、world、vehicle 和 scenario_desc，并只把解析后的 profile 与场景转发到 uv_sim_bringup。它还声明 mission_file、enable_ai、enable_nav、enable_task、公共仿真参数（scene_seed、simulation_rate、窗口尺寸、render_quality、render_fps、gpu、gpu_backend、camera_stitch_fps、publish_raw_camera_topics、AI 参数、startup_timeout）和前文的观测/录制参数，但没有把这些参数转发给实现层；从该公开入口设置它们不会产生预期作用。这里没有声明 enable_motion，也没有声明 sim.launch.py 专有的 enable_stream、enable_evaluation、enable_degradation 等参数。继承参数默认值与上方 uv_sim_bringup / uv_bringup 对应表一致。需要这些开关时可直接运行 uv_sim_bringup/launch/sim.launch.py，或补齐 wrapper 转发。

### uv_sim_bringup：退化注入 degradation.launch.py

这些参数可直接传给 degradation.launch.py。sim.launch.py 只转发其中一部分；未转发项在总控仿真中使用默认值。

| 参数 | 默认值 | 含义 | 弃用 |
|---|---|---|---|
| enable_degradation | false | 是否启动全部退化节点 | 否 |
| degradation_seed | 0 | 退化噪声随机数 seed | 否 |
| dvl_dropout_probability | 0.0 | 丢弃 DVL 样本的概率 | 否 |
| dvl_beam_loss_probability | 0.0 | DVL 单束失效概率 | 否 |
| dvl_noise_stddev | 0.0 | DVL 速度噪声标准差 | 否 |
| dvl_altitude_noise_stddev | 0.0 | DVL 高度噪声标准差 | 否 |
| dvl_bottom_lock_loss_probability | 0.0 | DVL bottom-lock 丢失概率 | 否 |
| dvl_delay_ms | 0.0 | DVL 消息延迟，毫秒 | 否 |
| imu_dropout_probability | 0.0 | 丢弃 IMU 样本的概率 | 否 |
| imu_accelerometer_noise_stddev | 0.0 | 加速度计噪声标准差 | 否 |
| imu_gyroscope_noise_stddev | 0.0 | 陀螺仪噪声标准差 | 否 |
| imu_accelerometer_bias | [0,0,0] | 加速度计三轴偏置 | 否 |
| imu_gyroscope_bias | [0,0,0] | 陀螺仪三轴偏置 | 否 |
| imu_random_walk_stddev | 0.0 | IMU 偏置随机游走标准差 | 否 |
| imu_delay_ms | 0.0 | IMU 消息延迟，毫秒 | 否 |
| visual_dropout_probability | 0.0 | 丢弃相机帧的概率 | 否 |
| visual_brightness_scale | 1.0 | 图像亮度乘数 | 否 |
| visual_noise_stddev | 0.0 | 图像高斯噪声标准差 | 否 |
| visual_blur_kernel | 0 | 模糊卷积核尺寸；0 表示关闭 | 否 |
| visual_delay_ms | 0.0 | 图像消息延迟，毫秒 | 否 |
| usbl_dropout_probability | 0.0 | 丢弃 USBL 测量的概率 | 否 |
| usbl_position_noise_stddev | 0.0 | USBL 位置噪声标准差 | 否 |
| usbl_outlier_probability | 0.0 | USBL 离群测量概率 | 否 |
| usbl_outlier_stddev | 0.0 | USBL 离群位置偏差标准差 | 否 |
| usbl_delay_ms | 0.0 | USBL 消息延迟，毫秒 | 否 |

sim.launch.py 只把 DVL 全部选项，以及 IMU dropout/random_walk、视觉 dropout/brightness/noise/blur、USBL dropout/outlier/outlier_stddev 等部分值转发。IMU 噪声和偏置、IMU delay、visual_delay_ms、USBL 位置噪声及 delay 未从总控入口转发。

### uv_sim_bringup：HIL 与实验入口

| 入口 / 参数 | 默认值 | 含义 | 弃用 |
|---|---|---|---|
| hil.launch.py profile | hil_lab | HIL 参数组 | 否 |
| hil.launch.py scenario_desc | underwater_xunyun.scn | Stonefish HIL 场景 | 否 |
| hil.launch.py enable_ai | false | 是否启动感知节点 | 否 |
| hil.launch.py enable_nav | false | 是否启动导航 | 否 |
| hil.launch.py enable_task | false | 是否启动任务执行器 | 否 |
| hil.launch.py enable_motion | false | 是否启动 basic_motion | 否 |
| hil.launch.py camera_config_dir | 空 | 覆盖相机 YAML 目录 | 否 |
| hil.launch.py serial_dev | /dev/ttyUSB0 | micro-ROS agent 使用的 MCU 串口 | 否 |
| hil.launch.py serial_baud | 921600 | MCU 串口波特率 | 否 |
| hil.launch.py agent_executable | 自动查找 micro_ros_agent；缺省使用 PATH 中同名程序 | micro-ROS agent 可执行文件 | 否 |
| experiment.launch.py world | guoshui_2026 cruise_seeded 场景 | 实验场景路径 | 否 |
| experiment.launch.py seed | 0 | 场景和退化随机 seed | 否 |
| experiment.launch.py degradation | nominal | 退化预设：nominal、dvl_loss、visual_loss、dvl_visual | 否 |
| experiment.launch.py estimator | bootstrap | 定位算法 | 否 |
| experiment.launch.py results_dir | results/current | 实验记录和评估输出目录 | 否 |
| experiment.launch.py run_id | 空 | 实验运行 ID | 否 |
| experiment.launch.py duration | 60.0 | 自动结束时间，秒 | 否 |
| experiment.launch.py record_bag | true | 是否录制实验 rosbag | 否 |
| experiment.launch.py gpu | false | 是否使用 GPU Stonefish | 否 |

HIL 还接受 Stonefish 公共参数；默认窗口 1280×720、渲染质量 high、camera_stitch_fps 10.0。core_sim.launch.py 不声明用户参数，使用固定值启动最小仿真。

### uv_sim_bridge

| 参数 | 默认值 | 含义 | 弃用 |
|---|---|---|---|
| hil_mode | false | true 选择 HIL 相机直通和推力混控；false 运行 SIL 控制桥 | 否 |
| camera_stitch_fps | 10.0 | 旧拼接帧率参数；CameraPassthrough 当前直接忽略 | 兼容项 |
| publish_raw_camera_topics | false | 旧逐目 DDS 图像发布开关；已标记弃用。总控入口固定传 false | 是 |
| profile_params | 空 | 可选 ROS 参数文件路径 | 否 |

sim_dev / sim_ci profile 为 hil_mode=false、publish_raw_camera_topics=false；hil_lab 为 hil_mode=true、publish_raw_camera_topics=false。profile 中的 camera_stitch_fps 值仍会被相机适配器忽略。

### uv_sim_degradation 节点参数

| 节点 / 参数 | 默认值 | 含义 | 弃用 |
|---|---|---|---|
| camera_degradation.camera | front | 区分 front 或 downward 摄像头实例 | 否 |
| camera_degradation.input_topic | front stitched topic | 输入图像 topic | 否 |
| camera_degradation.output_topic | degraded front stitched topic | 输出图像 topic | 否 |
| camera_degradation.dropout_probability | 0.0 | 帧丢失概率 | 否 |
| camera_degradation.brightness_scale | 1.0 | 图像亮度乘数 | 否 |
| camera_degradation.gaussian_noise_stddev | 0.0 | 图像高斯噪声标准差 | 否 |
| camera_degradation.blur_kernel | 0 | 模糊卷积核尺寸 | 否 |
| camera_degradation.delay_ms | 0.0 | 图像延迟，毫秒 | 否 |
| camera_degradation.seed | 0 | 随机数 seed | 否 |
| dvl_degradation.input_topic | DVL velocity topic | 输入 DVL 速度 topic | 否 |
| dvl_degradation.output_topic | degraded DVL velocity topic | 输出 DVL 速度 topic | 否 |
| dvl_degradation.altitude_input_topic | DVL altitude topic | 输入 DVL 高度 topic | 否 |
| dvl_degradation.altitude_output_topic | degraded DVL altitude topic | 输出 DVL 高度 topic | 否 |
| dvl_degradation.dropout_probability | 0.0 | DVL 样本丢失概率 | 否 |
| dvl_degradation.beam_loss_probability | 0.0 | DVL 波束丢失概率 | 否 |
| dvl_degradation.noise_stddev | 0.0 | 速度噪声标准差 | 否 |
| dvl_degradation.altitude_noise_stddev | 0.0 | 高度噪声标准差 | 否 |
| dvl_degradation.bottom_lock_loss_probability | 0.0 | bottom-lock 丢失概率 | 否 |
| dvl_degradation.delay_ms | 0.0 | 消息延迟，毫秒 | 否 |
| dvl_degradation.seed | 0 | 随机数 seed | 否 |
| imu_degradation.input_topic | IMU topic | 输入 IMU topic | 否 |
| imu_degradation.output_topic | degraded IMU topic | 输出 IMU topic | 否 |
| imu_degradation.dropout_probability | 0.0 | IMU 样本丢失概率 | 否 |
| imu_degradation.accelerometer_noise_stddev | 0.0 | 加速度计噪声标准差 | 否 |
| imu_degradation.gyroscope_noise_stddev | 0.0 | 陀螺仪噪声标准差 | 否 |
| imu_degradation.accelerometer_bias | [0,0,0] | 加速度计三轴偏置 | 否 |
| imu_degradation.gyroscope_bias | [0,0,0] | 陀螺仪三轴偏置 | 否 |
| imu_degradation.random_walk_stddev | 0.0 | IMU 偏置随机游走标准差 | 否 |
| imu_degradation.delay_ms | 0.0 | 消息延迟，毫秒 | 否 |
| imu_degradation.seed | 0 | 随机数 seed | 否 |
| usbl_degradation.input_topic | USBL topic | 输入 USBL topic | 否 |
| usbl_degradation.output_topic | degraded USBL topic | 输出 USBL topic | 否 |
| usbl_degradation.dropout_probability | 0.0 | 测量丢失概率 | 否 |
| usbl_degradation.position_noise_stddev | 0.0 | 位置噪声标准差 | 否 |
| usbl_degradation.outlier_probability | 0.0 | 离群测量概率 | 否 |
| usbl_degradation.outlier_stddev | 0.0 | 离群位置偏差标准差 | 否 |
| usbl_degradation.delay_ms | 0.0 | 消息延迟，毫秒 | 否 |
| usbl_degradation.seed | 0 | 随机数 seed | 否 |

### stonefish_ros2 与 uv_sim_description

| 包 / 参数 | 默认值 | 含义 | 弃用 |
|---|---|---|---|
| stonefish_ros2.simulation_data | 空 | Stonefish 仿真资源目录 | 否 |
| stonefish_ros2.scenario_desc | 空 | Stonefish 场景文件名或绝对路径 | 否 |
| stonefish_ros2.simulation_rate | 100.0 | 物理仿真更新频率，Hz | 否 |
| stonefish_ros2.window_res_x | 800（独立 launch；总控默认 960） | GPU 仿真窗口宽度，像素 | 否 |
| stonefish_ros2.window_res_y | 600（独立 launch；总控默认 540） | GPU 仿真窗口高度，像素 | 否 |
| stonefish_ros2.rendering_quality | high（独立 launch；总控默认 low） | GPU 渲染质量 | 否 |
| stonefish_ros2.render_fps | 30.0 | stonefish_simulator ROS 参数：窗口刷新频率上限 | 否 |
| uv_sim_description.use_sim_time | true | 仿真机器人描述节点使用仿真时间 | 否 |
| uv_sim_description.robot_description | 从包内 URDF 文件读取 | 机器人描述节点启动时载入的 URDF 内容 | 否 |

### uv_sim_evaluation 节点

| 参数 | 默认值 | 含义 | 弃用 |
|---|---|---|---|
| output_dir | 空 | 指标和事件输出目录 | 否 |
| run_id | 空 | 本次评估运行标识 | 否 |
| publish_period | 1.0 | 指标发布周期，秒 | 否 |
| seed | 0 | 场景/退化随机 seed 元数据 | 否 |

### uv_sim_assets 静态车辆配置

uv_sim_assets/config 下的 YAML 是生成/校验 Stonefish 车辆资源的静态输入，不是 ROS 节点参数。

| 文件 | 主要字段 | 含义 | 弃用 |
|---|---|---|---|
| vehicle_geometry.yaml | visual_meshes、physical_meshes、physical_mesh.path、collision、hydrodynamics、scale、coordinate_frame、units | 视觉/物理网格、碰撞和水动力网格、缩放及坐标约定 | 否 |
| vehicle_inertial.yaml | mass、inertia_kg_m2、center_of_mass、center_of_buoyancy、calibration_status | 质量、惯量和质心/浮心来源；当前注明 uncalibrated | 否 |
| vehicle_hydrodynamics.yaml | physics_mode、buoyancy、added_mass、drag_geometry、collision_geometry、thrust_model | 水下物理模式、浮力、附加质量、阻力/碰撞网格和推力模型 | 否 |
| vehicle_sensors.yaml | sensors、dvl_mount.position_body、dvl_mount.orientation_rpy、camera_frames | 传感器 topic、DVL 安装位姿及相机坐标帧 | 否 |
| vehicle_thrusters.yaml | count、command_topic、state_topic、ordering、position_body、direction_body、allocation_matrix | 推进器数量、topic、编号顺序、位置方向及推力分配矩阵 | 否 |
| vehicle_baseline.yaml | mesh、source_mesh、topology_repair、runtime_regression、calibration_status | 网格拓扑修复和几何回归记录；不代表物理标定结果 | 否 |

## 尚未发现参数的包与参数文件提示

- auv_protocol、uv_msgs、uv_image_transport 和 zit6_interfaces 主要提供 topic 常量、消息、服务或传输 API，没有 ROS launch 参数或节点参数声明。
- uv_mapping 已标记弃用；它只是兼容重导出包，没有参数。
- uv_nav 已标记弃用；当前 navigator 节点没有参数声明。uv_planning 是对外规划边界。
- uv_hm/config/pid_params.yaml 和 pid_parameters.json 随包安装，但当前 ROS 节点和 launch 源码没有读取它们；不要把它们当成 hw_manager 当前生效参数。
- auv_description/config/real_components.yaml 保存结构和安装位置参考数据，当前 launch 由 URDF 提供 robot_description；该 YAML 不是 launch 参数文件。
- Stonefish 场景文件还可包含各自的对象和传感器设置；它们由场景路径选择，不属于统一的 ROS 节点参数集合。
