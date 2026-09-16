# 节点参数与启动参数文档

> 本文件是各节点 ROS 参数与 launch 参数的汇总入口。**参数真值以源码 `declare_parameter` 为准**，
> 修改参数后若与本文件冲突，请同步本文件。
>
> 最后核对：**2026-09-16**（T3a：逐条对照 `declare_parameter` 与 `DeclareLaunchArgument` 源码）

## 阅读须知：本文件的完整性状态

T3a 只做**不随 T4/T5 变化**的部分；涉及 launch 结构与 `sim_bridge` 拆分的内容留待回填。
各节标题后的标记含义：

| 标记 | 含义 |
|---|---|
| ✅ | 已与代码逐条核对，当前准确 |
| ⏳ | **已知不完整或即将失效**，等 T4/T5 落地后回填（见 `roadmap.md`） |
| 🔗 | 事实源已迁移到别处，本节只留链接 |

**待补清单（T3a 已知缺口，别当成"已完成"）**

| # | 缺口 | 何时补 |
|---|---|---|
| 1 | **launch 入口表**：`sim_bringup`/`sim_core`/`hil_bringup` 位于 `workspace_auv` 却直接引用仿真包（违反分层，B1），T4 会重排；包级 launch `uv_control/launch/control_launch.py`、`uv_hm/launch/hm_launch.py` 当前引用**不存在的入口**（B2） | T4 |
| 2 | **`sim_bridge` 参数与话题归属**：T5 将把它拆成 7 个节点，§sim_bridge 每一行都会变 | T5 |
| 3 | **话题频率 / QoS 列**：需实时快照（`ros2 topic info -v` / `hz`）；本沙箱仅 console 模式可用且无相机（B13） | T3b |
| 4 | **话题表去重**：`hil_architecture.md` §2.2/2.3 与本文件话题参考仍有重复，将收敛到 `docs/interfaces/topic_contract.md` | T3b |
| 5 | `docs/README.md` 索引与 `tools/check_docs_contract.py`（参数/入口/依赖三方对拍） | T3a 后续 |

> **维护规则（T3a 起）**：改任何 ROS 参数默认值或 launch 参数，**必须同一次改动里更新本文件**；
> 本轮新增的 `pid_config_file` 就属此类。校验工具落地后由 `check_docs_contract.py` 强制。

## 所有可执行节点

| 可执行文件 | 包 | 所在工作区 | 源码 | 说明 |
|---|---|---|---|---|
| `stonefish_simulator` | `stonefish_ros2` | workspace_sim | C++ | GPU 渲染仿真器（需 X Display / `xvfb-run`） |
| `stonefish_simulator_nogpu` | `stonefish_ros2` | workspace_sim | C++ | 无头模式仿真器（**拒绝含相机的场景**，见 `workspace_sim/stonefish_notes.md`） |
| `sim_bridge` | **`uv_sim`** | **workspace_sim** | `src/uv_sim/uv_sim/sim_bridge.py` | 仿真硬件桥接（SIL 跑 PID；HIL 仅转发） |
| `hw_manager` | `uv_hm` | workspace_auv | `uv_hm/hw_manager.py` | 实车硬件管理（心跳 + 状态监控） |
| `basic_motion` | `uv_control` | workspace_auv | `uv_control/basic_motion.py` | 运动控制 Action Server |
| `vision` | `uv_perception` | workspace_auv | `uv_perception/vision.py` | YOLO 目标检测 |
| `position` | `uv_perception` | workspace_auv | `uv_perception/position.py` | 3D 目标定位 |
| `navigator` | `uv_nav` | workspace_auv | `uv_nav/navigator.py` | A* 路径规划 + 避障 |
| `task_runner` | `uv_task` | workspace_auv | `uv_task/task_runner.py` | JSON 任务执行器 |

注意 `uv_sim` 只存在于 `workspace_sim/src/`，不在 `workspace_auv/src/` 中。

> ✅ 已与各包 `setup.py` 的 `console_scripts` 核对：Python 入口 **7 个**（上面除两个仿真器外的 7 行）。
> `uv_bringup` 自身没有 entry point，只有 launch 文件；**谁启动谁**属 launch 层，见待补清单 #1。

### 包级 launch 现状 ⏳（B2 的具体化）

| launch 文件 | 引用的入口 | 入口是否存在 | 状态 |
|---|---|---|---|
| `uv_hm/launch/hm_launch.py` | `uv_hm/sim_bridge` + `hw_manager` | ❌ `sim_bridge` 不在 `uv_hm`（在 `uv_sim`/workspace_sim） | **失效**，等 T4 重排 |
| `uv_control/launch/control_launch.py` | `minimal_control` + `basic_motion` | ❌ `minimal_control` 无此入口 | **失效**，等 T4 重排 |
| `uv_nav/launch/nav_launch.py` | `navigator` | ✅ | 可用 |
| `uv_perception/launch/perception_launch.py` | `vision` + `position` | ✅ | 可用（无图像发布参数，视频走 go2rtc） |
| `uv_task/launch/task_launch.py` | `task_runner` | ✅ | 可用 |
| `uv_bringup/launch/{sim,real,hil}_bringup.py`、`sim_core.py` | 多个 | ✅ | 可用，但 `sim_*`/`hil_*` 位于 `workspace_auv` 却引用仿真包（违反分层，B1），T4 拆分 |

> 上面 `uv_bringup` 的 4 个文件就是待补清单 #1 说的"T4 会重排"的对象；
> 本表只记录**现状**，迁移后的入口映射表在 T4 落地时补。

---

## vision 节点参数 ✅

### 运行模式

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `sim_mode` | bool | `false` | `true`: 订阅 `/auv/*/stitched` ROS 话题<br>`false`: 使用 V4L2 设备直接采集 |

> `sim_bringup.py` / `hil_bringup.py` 在启动 vision 时传 `sim_mode: True`；
> `real_bringup.py` 传 `sim_mode: False`。直接 `ros2 run` 时默认 `false`。

### 相机使能与设备

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `enable_front_camera` | bool | `true` | 启用前视相机通道 |
| `enable_down_camera` | bool | `true` | 启用下视相机通道 |
| `front_cam_path` | str | `/dev/video0` | 前视摄像头 V4L2 设备路径（仅 real 模式声明） |
| `down_cam_path` | str | `/dev/video2` | 下视摄像头 V4L2 设备路径（仅 real 模式声明） |

### 图像发布 / go2rtc

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `enable_gortc` | bool | `true` | 启用本地 MJPEG 源和 go2rtc 子进程 |
| `gortc_executable` | str | `go2rtc` | go2rtc 可执行文件名/路径 |
| `mjpeg_port` | int | `8090` | vision 本地 MJPEG 服务端口（1–65535，越界直接抛错） |
| `gortc_http_port` | int | `1984` | go2rtc HTTP/WebRTC 服务端口（1–65535，越界直接抛错） |
| `stream_annotated` | bool | `true` | 通过 go2rtc 额外提供带检测框的视频流 `front_annotated/down_annotated` |

### 巡线（line follower）

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `line_contour_min_area` | int | `200` | 轮廓最小面积阈值 |
| `line_filter_process_noise` | float | `1.0` | 巡线滤波器过程噪声 |
| `line_filter_measurement_noise` | float | `3.0` | 巡线滤波器测量噪声 |

### 模型与数据集

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `model_path` | str | `""` | YOLO 模型 .pt 文件路径。为空时自动查找默认路径 |
| `save_dataset` | bool | **`false`** | 是否保存采集的图像（2026-09-16 更正：常量 `SAVE_DATASET` 已改为 `False`，避免误抓数据集） |
| `dataset_dir` | str | `<仓库根>/datasets` | 数据集落盘目录，支持 `~` 展开。空字符串按默认值处理 |

> **默认位置与解析方式**：默认写到**仓库根下的 `datasets/`**，该目录已加入 `.gitignore`。
>
> 仓库根是**从 `vision.py` 逐级向上找 `.git`** 得到的，**不是**按 `__file__` 固定上溯
> 层数 —— colcon 会把 `vision.py` 拷进 `build/uv_perception/uv_perception/`，固定上溯
> 三层会得到 `workspace_auv/build/img`，一次 `rm -rf build` 或 colcon 清理就把数据集删掉。
> 逐级找 `.git` 在 `src/`、`build/`、`install/` 三种布局下都能正确回到仓库根。
> 找不到仓库根时（装到系统路径、或源码被单独拷走）退回 `~/.ros/auv_dataset`。
>
> 运行时可用 `-p dataset_dir:=<path>` 覆盖：
>
> ```bash
> ros2 launch uv_bringup real_bringup.py \
>   --ros-args -p dataset_dir:=/data/auv_capture
> ```
>
> **数据集内容**：保存的是**整幅拼接原图**（左右目并排，**未切分、未去畸变**），
> 文件名 `front_*.jpg` / `down_*.jpg`，JPEG **q95**，每相机限 5 帧/秒。
> 保存点在去畸变**之前**，因此与 `K`/`D` 是否已标定解耦 —— 标定前后采的数据
> 几何一致，可以直接混用。需要单目图时在训练前按 `mid = width // 2` 切分
> （切分点必须与 `vision.py` 的 `mid` 一致）。
> 保存不依赖 YOLO 模型是否加载成功，可用于采集验证集。
> ⚠️ 这不是传感器原始像素：上游已过「摄像头内 JPEG → USB/MJPEG → OpenCV 解码」，
> 落盘时再做一次 JPEG q95。体积约 **9.5 MB/s ≈ 34 GB/小时**，正式跑前确认磁盘余量。

### 相机标定

> ⚠️ **当前全部留空（未标定）**。换了新摄像头后旧内参作废，`vision.py` 里这 4 个常量
> 均为空元组 `()`；空值表示「未标定」，vision 会跳过去畸变并打印一次告警，**不是错误**。
> 旧的 `config/*.npz`（Girona 时代标定）已从仓库删除。
> 仿真模式下 Stonefish 渲染的图像本身无畸变，永远不需要去畸变。

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `front_camera_matrix` | float[9] | **空 `()`** | 前视相机内参 3x3（行优先），待重新标定 |
| `front_dist_coeffs` | float[5] | **空 `()`** | 前视相机畸变系数 (k1,k2,p1,p2,k3)，待重新标定 |
| `down_camera_matrix` | float[9] | **空 `()`** | 下视相机内参 3x3，待重新标定 |
| `down_dist_coeffs` | float[5] | **空 `()`** | 下视相机畸变系数，待重新标定 |

标定后填入 K(9 个值) 与 D(5 个值) 即可自动生效；只填 K 不填 D 也允许（D 为空表示无畸变）。

模块级常量（非 ROS 参数，改需重编译）：`CONFIDENCE`（检测置信度）、`PIPE_CLASS_ID` / `PIPE_CONFIDENCE`（pipe 单独高阈值）、
`ENABLE_UNDISTORT`、`DEFAULT_MODEL_FILENAME`。

分辨率常量（均为非 ROS 参数）：

| 常量 | 值 | 含义 |
|---|---|---|
| `FRONT_CAMERA_RESOLUTION` | `(1280, 960)` | 前视**单目**尺寸，说明用；实际取自输入图像 shape |
| `DOWN_CAMERA_RESOLUTION` | `(1920, 1080)` | 下视**单目**尺寸，说明用 |
| `FRONT_CAPTURE_RESOLUTION` | `(2560, 960)` | 前视 V4L2 采集**请求值**（stitched 双目图，实机实测一致） |
| `DOWN_CAPTURE_RESOLUTION` | `(3840, 1080)` | 下视 V4L2 采集**请求值**（stitched 双目图，实机实测一致） |

> ⚠️ `*_CAPTURE_RESOLUTION` 只是传给 `cap.set(CAP_PROP_FRAME_WIDTH/HEIGHT)` 的请求值，
> 驱动不保证满足（曾出现常量写 `2560x960`、实机实际输出 `3840x1080` 的静默偏差）。
> `vision.py` 现在会在每个相机首次收到帧时回读实际 `shape`，与常量不一致就打 WARN。
> 实测来源：`/home/origin/recordings`（2026-09-12，拉的是 `:8090/front|down`）。

---

## position 节点参数 ✅

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `max_history` | int | `30` | 每条射线历史最大保留数量 |
| `angle_match_threshold` | float | `50.0` | 多实例关联的角度匹配阈值（度），夹角小于此值视为同一实例 |

编译期常量（非 ROS 参数）：`MIN_BASELINE=0.1m`、`MAX_DISTANCE=50m`、EMA 平滑系数 `alpha=0.3`。
位姿历史缓冲时长固定为 `2.0s` / 上限 150 条，**没有** `pose_buffer_duration` 这个 ROS 参数。

---

## sim_bridge 节点参数 ⏳

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `hil_mode` | bool | `false` | `true`: HIL 模式（相机直通 + 转发 MCU 推力）<br>`false`: SIL 全仿真（PID + ZIT6 状态机 + 状态发布） |
| `pid_config_file` | string | `''` | PID 增益文件路径。留空 = 自动解析（源码树 `src/uv_sim/config/sim_pid.yaml` 优先，其次包内 `share/uv_sim/config/sim_pid.yaml`）；可指向 `/tmp` 下的调参副本，无需重编译 |

> ⏳ **本节将在 T5 重写**：`sim_bridge` 会被拆成 `odom_sim` / `imu_sim` / `dvl_sim` /
> `pressure_sim` / `nav_sim` / `mcu_sim` / `camera_relay` 七个节点，参数与话题归属随之变化。
> 当前内容已与代码核对，可放心使用到 T5 落地为止。

**PID 增益来源（2026-09-15 起）**：`workspace_sim/src/uv_sim/config/sim_pid.yaml` 是唯一事实源，9 个通道
（`chassis.pid.pos.{x,y,z,yaw}` + yaw 串级内环 `chassis.pid.pos.yaw_rate` + `chassis.pid.vel.{x,y,z,yaw}`）。
同一份数据既构造 `/zit6/get_params` 的返回值、也构造实际运行的 PID 对象，二者不会再漂移；
`/zit6/update_params` 只改运行时、不写回文件。启动日志会打印实际加载路径。
此前 `uv_hm/config/pid_params.yaml`（已删除，无人读取且数值已落后）与 `_init_full()` 内的硬编码副本均已废弃。

---

## hw_manager 节点参数 ✅

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `heartbeat_rate` | float | `8.0` | 向 `/zit6/cmd/agxhbt` 发布心跳的频率（Hz） |
| `watchdog_timeout` | float | `7.0` | 心跳/状态超时阈值（秒），超时进入 DEGRADED |
| `arm_mode` | int | `1` | 心跳携带的布防值：`1`=正常布防，`3`=强制布防 |
| `battery_low_threshold` | float | `14.0` | 低电压告警阈值（V） |
| `cycle_time_warn_threshold` | float | `100.0` | MCU 周期时间告警阈值（ms） |

---

## task_runner 节点参数 ✅

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `debug_mode` | bool | `false` | 调试模式（输出更详细日志） |
| `down_image_width` | float | `1280.0` | 下视图像宽度，用于像素→角度换算 |
| `down_image_height` | float | `960.0` | 下视图像高度 |
| `align_ema_alpha` | float | `0.08` | 对准控制的 EMA 平滑系数（越小越平滑、越滞后） |

---

## 无参数节点 ✅

`basic_motion`、`navigator` 不声明任何 ROS 参数。

---

## 启动文件 Launch Arguments ⏳

### sim_bringup.py

| 参数 | 默认值 | 说明 |
|---|---|---|
| `enable_ai` | `true` | 启用 vision + position |
| `enable_nav` | `false` | 启用 navigator |
| `enable_task` | `false` | 启用 task_runner |
| `scenario_desc` | `wuurc_murc_2026_auv.scn` | Stonefish 场景文件 |

**用法：**
```bash
ros2 launch uv_bringup sim_bringup.py enable_ai:=true
```

### sim_core.py

只启动 Stonefish + sim_bridge，不带任何上层节点。

| 参数 | 默认值 | 说明 |
|---|---|---|
| `scenario` | `underwater_xunyun.scn` | ⚠️ 仅用于日志打印，仿真器实际加载的场景名硬编码在文件内 |
| `gpu` | `true` | ⚠️ **当前未被使用**，仿真器固定使用 GPU 版 `stonefish_simulator` |

### real_bringup.py

| 参数 | 默认值 | 说明 |
|---|---|---|
| `enable_ai` | `true` | 启用 vision + position |
| `enable_nav` | **`false`** | 启用 navigator（2026-09-16 更正：文档此前写 `true`，源码 `default_value='false'`） |
| `enable_task` | `false` | 启用 task_runner |

**用法：**
```bash
ros2 launch uv_bringup real_bringup.py
```

> ✅ 关于 navigator 的位姿源（2026-09-13 已迁移，此前文档描述过时）：
> `navigator` 现在订阅 `/basic_motion/pose_info`（`uv_msgs/PoseInfo`），**不再**使用
> `/auv/state`（该话题已无任何发布者，`AuvState` 消息也只剩定义）。`navigator` 在收到
> 第一个位姿前会拒绝规划并打印提示。
>
> ⚠️ `real_bringup` 的 `enable_nav` 默认 `false`，因此实机默认**不**启动导航；
> 需要导航时显式 `enable_nav:=true`，且必须同时有 `basic_motion` 在跑（位姿源）。

### hil_bringup.py

| 参数 | 默认值 | 说明 |
|---|---|---|
| `enable_ai` | `false` | 启用 vision + position |
| `enable_nav` | `false` | 启用 navigator |
| `enable_task` | `false` | 启用 task_runner |
| `enable_motion` | `false` | 启用 basic_motion |
| `scenario` | `underwater_xunyun.scn` | ⚠️ 仅用于日志打印，仿真器实际加载的场景名硬编码在文件内 |
| `serial_dev` | `/dev/ttyUSB0` | MCU 串口设备 |
| `serial_baud` | `921600` | 串口波特率 |

**用法：**
```bash
ros2 launch uv_bringup hil_bringup.py enable_ai:=true
```

### perception_launch.py

该启动文件不再提供图像发布参数；视频通过 go2rtc 查看。

**用法：**
```bash
ros2 launch uv_perception perception_launch.py
```

---

## 话题参考 ⏳

### vision ✅

| 方向 | 话题 | 类型 | 说明 |
|---|---|---|---|
| 发布 | `/perception/detection/{front_left,front_right,down_left,down_right}` | `DetectionArray` | 4 通道检测结果 |
| 发布 | `/perception/line/{front_left,down_left,…}` | `LineState` | **巡线状态（此前文档漏记）** |
| 发布 | `/perception/aruco/ids` | `Int32MultiArray` | ArUco 标记 ID |
| 订阅 | `/auv/{front,down}_cam/stitched` | `sensor_msgs/Image` | **仅 `sim_mode:=true`** 时订阅（实机走 V4L2，不订阅） |

> ⚠️ 此前文档写「vision 不发布图像 DDS 话题」——对**实机**成立（视频走 go2rtc），
> 但 `sim_mode` 下 vision 是**订阅**方，图像由 `sim_bridge` 发布（见下）。
> go2rtc 流：`front` / `down`（原始拼接）、`front_annotated` / `down_annotated`（YOLO 带框）。

### position ✅

| 方向 | 话题 | 类型 | 说明 |
|---|---|---|---|
| 订阅 | `/perception/detection/{front_left,front_right,down_left,down_right}` | `DetectionArray` | 4 通道检测 |
| 订阅 | `/basic_motion/pose_info` | `uv_msgs/PoseInfo` | 位姿源（含 roll/pitch） |
| 发布 | `/perception/objects` | `ObjectPositionArray` | 3D 物体世界坐标（含 `instance_id`） |

### basic_motion ✅

| 方向 | 名称 | 类型 | 说明 |
|---|---|---|---|
| 动作 | `/basic_motion` | `uv_msgs/action/BasicMotion` | 唯一运动入口（START/SET/WMOVE/BMOVE/WTRAVEL/BTRAVEL） |
| 发布 | `/basic_motion/pose_info` | `uv_msgs/PoseInfo` | 机器人位姿 30 Hz（含 roll/pitch，**无 frame 字段**） |
| 订阅 | `/zit6/state/{pos,vel}` | `Float32MultiArray[6]` | 里程计状态反馈 |
| 订阅 | `/zit6/state/status` | `zit6_interfaces/ZitStatus` | MCU 状态 |
| 发布 | `/zit6/cmd/setpoint` | `zit6_interfaces/ZitSetpoint` | 发往 MCU / sim_bridge 的设定值 |

### navigator ✅

| 方向 | 名称 | 类型 | 说明 |
|---|---|---|---|
| 订阅 | `/basic_motion/pose_info` | `uv_msgs/PoseInfo` | **位姿源**（2026-09-13 已从 `/auv/state` 迁移） |
| 订阅 | `/perception/objects` | `ObjectPositionArray` | 障碍 / 目标 |
| 发布 | `/nav/path` | `nav_msgs/Path` | 规划出的航点路径 |
| 服务 | `/nav/navigate_to` | `uv_msgs/srv/RunTask` | 请求运行任务（已核对：`create_service(RunTask, '/nav/navigate_to', …)`） |

### hw_manager ✅（实机 / HIL）

| 方向 | 话题 | 类型 | 说明 |
|---|---|---|---|
| 发布 | `/zit6/cmd/agxhbt` | `std_msgs/UInt32` | 心跳（`heartbeat_rate` Hz，携带 `arm_mode`） |
| 订阅 | `/zit6/state/status` | `zit6_interfaces/ZitStatus` | MCU 状态汇总 |
| 订阅 | `/zit6/state/zithbt` | `std_msgs/UInt32` | MCU 心跳序列号（看门狗） |
| 订阅 | `/zit6/state/thr` | `Float32MultiArray[6]` | 推力饱和监控（4 元素来源会让监控静默失效） |

### task_runner ✅

| 方向 | 名称 | 类型 | 说明 |
|---|---|---|---|
| 订阅 | `/perception/objects` | `ObjectPositionArray` | 目标位置 |
| 订阅 | `/perception/detection/{cam}` | `DetectionArray` | 任务内联检测 |
| 订阅 | `/basic_motion/pose_info` | `uv_msgs/PoseInfo` | 位姿 |
| 订阅 | `/zit6/state/status` | `zit6_interfaces/ZitStatus` | MCU 状态 |
| 发布 | `/task/status` | `uv_msgs/TaskStatus` | 任务进度（1 Hz） |
| 发布 | `/zit6/cmd/{light,servo,pushrod}` | `UInt8` / `Float32` / `ZitPushrod` | 执行机构 |
| 服务 | `/task/run`、`/task/stop`、`/task/exec` | `uv_msgs/srv/RunTask` / `std_srvs/Trigger` / `uv_msgs/srv/ExecTask` | 任务控制 |

### sim_bridge / MCU 状态话题 ⏳

**SIL 模式**（`hil_mode:=false`，PID 在 sim_bridge 内）：

| 方向 | 话题 | 类型 | 说明 |
|---|---|---|---|
| 订阅 | `/auv/{odometry,imu,dvl,pressure}` | Stonefish 传感器消息 | 仿真传感器输入 |
| 订阅 | `/zit6/cmd/{setpoint,servo,light,agxhbt,ins}` | 见各消息 | 来自控制栈 / 心跳 |
| 发布 | `/zit6/state/{pos,vel,thr,status,zithbt}` | `Float32MultiArray[6]` / `ZitStatus` / `UInt32` | MCU 状态仿真 |
| 发布 | `/auv/thrusters_cmd` | `Float64MultiArray` | 6 推进器归一化设定点 [−1, 1] |
| 发布 | `/auv/{front,down}_cam/{left,right,stitched}` | `sensor_msgs/Image` | 拼接后的双目图像（供 vision `sim_mode`） |
| 服务 | `/zit6/{get_params,update_params}` | `GetParams` / `UpdateParams` | PID 参数读写（只改运行时，**不写回文件**） |

**HIL 模式**（`hil_mode:=true`，PID 交给实机 MCU）：

| 方向 | 话题 | 类型 | 说明 |
|---|---|---|---|
| 发布 | `/zit6/sim/nav` | `Float32MultiArray` | 仿真位姿 → MCU（`vel` 为**机体系**速度） |
| 订阅 | `/zit6/state/thr` | `Float32MultiArray[6]` | 转发 MCU 推力到 `/auv/thrusters_cmd` |
| 发布 | 相机图像（同上） | `sensor_msgs/Image` | |

> **已确认的缺口（登记在案，T3b 收敛进 `docs/interfaces/topic_contract.md`）**
>
> | 缺口 | 现状 |
> |---|---|
> | `/perception/annotated/{cam}` | **有订阅者（GUI `visualization/auv_visualizer.py`）但全局无发布者**（B5） |
> | `/auv/state`（`uv_msgs/AuvState`） | **无发布者、也无订阅者**（navigator 已迁到 `PoseInfo`）；消息本身只剩定义 |
> | `/auv/dvl_altitude` | 由 Stonefish DVL 传感器发布，**无人订阅** |
> | `/sim/{front,down}_cam/*` | 仿真相机原始话题，由 `sim_bridge` 订阅后合成 `/auv/*_cam/*` |
