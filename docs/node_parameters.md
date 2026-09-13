# 节点参数与启动参数文档

> 本文件是各节点 ROS 参数与 launch 参数的汇总入口。**参数真值以源码 `declare_parameter` 为准**，
> 修改参数后若与本文件冲突，请同步本文件。
>
> 最后核对：2026-09-13（对照 `vision.py` / `position.py` / `sim_bridge.py` / `hw_manager.py` / `task_runner.py`）

## 所有可执行节点

| 可执行文件 | 包 | 所在工作区 | 源码 | 说明 |
|---|---|---|---|---|
| `stonefish_simulator` | `stonefish_ros2` | workspace_sim | C++ | GPU 渲染仿真器（需 X Display / `xvfb-run`） |
| `stonefish_simulator_nogpu` | `stonefish_ros2` | workspace_sim | C++ | 无头模式仿真器 |
| `sim_bridge` | **`uv_sim`** | **workspace_sim** | `src/uv_sim/uv_sim/sim_bridge.py` | 仿真硬件桥接（SIL 跑 PID；HIL 仅转发） |
| `hw_manager` | `uv_hm` | workspace_auv | `uv_hm/hw_manager.py` | 实车硬件管理（心跳 + 状态监控） |
| `basic_motion` | `uv_control` | workspace_auv | `uv_control/basic_motion.py` | 运动控制 Action Server |
| `vision` | `uv_perception` | workspace_auv | `uv_perception/vision.py` | YOLO 目标检测 |
| `position` | `uv_perception` | workspace_auv | `uv_perception/position.py` | 3D 目标定位 |
| `navigator` | `uv_nav` | workspace_auv | `uv_nav/navigator.py` | A* 路径规划 + 避障 |
| `task_runner` | `uv_task` | workspace_auv | `uv_task/task_runner.py` | JSON 任务执行器 |

注意 `uv_sim` 只存在于 `workspace_sim/src/`，不在 `workspace_auv/src/` 中。

---

## vision 节点参数

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
| `save_dataset` | bool | `true` | 是否保存采集的图像 |
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

## position 节点参数

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `max_history` | int | `30` | 每条射线历史最大保留数量 |
| `angle_match_threshold` | float | `50.0` | 多实例关联的角度匹配阈值（度），夹角小于此值视为同一实例 |

编译期常量（非 ROS 参数）：`MIN_BASELINE=0.1m`、`MAX_DISTANCE=50m`、EMA 平滑系数 `alpha=0.3`。
位姿历史缓冲时长固定为 `2.0s` / 上限 150 条，**没有** `pose_buffer_duration` 这个 ROS 参数。

---

## sim_bridge 节点参数

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `hil_mode` | bool | `false` | `true`: HIL 模式（相机直通 + 转发 MCU 推力）<br>`false`: SIL 全仿真（PID + ZIT6 状态机 + 状态发布） |

---

## hw_manager 节点参数

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `heartbeat_rate` | float | `8.0` | 向 `/zit6/cmd/agxhbt` 发布心跳的频率（Hz） |
| `watchdog_timeout` | float | `7.0` | 心跳/状态超时阈值（秒），超时进入 DEGRADED |
| `arm_mode` | int | `1` | 心跳携带的布防值：`1`=正常布防，`3`=强制布防 |
| `battery_low_threshold` | float | `14.0` | 低电压告警阈值（V） |
| `cycle_time_warn_threshold` | float | `100.0` | MCU 周期时间告警阈值（ms） |

---

## task_runner 节点参数

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `debug_mode` | bool | `false` | 调试模式（输出更详细日志） |
| `down_image_width` | float | `1280.0` | 下视图像宽度，用于像素→角度换算 |
| `down_image_height` | float | `960.0` | 下视图像高度 |
| `align_ema_alpha` | float | `0.08` | 对准控制的 EMA 平滑系数（越小越平滑、越滞后） |

---

## 无参数节点

`basic_motion`、`navigator` 不声明任何 ROS 参数。

---

## 启动文件 Launch Arguments

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
| `enable_nav` | `true` | 启用 navigator |
| `enable_task` | `false` | 启用 task_runner |

**用法：**
```bash
ros2 launch uv_bringup real_bringup.py
```

> 注意：`navigator` 订阅的 `/auv/state` 目前无任何发布者，实机默认打开 `enable_nav` 会得到一个拿不到位姿的导航节点。

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

## 话题参考

### vision 发布话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/perception/detection/front_left` | `DetectionArray` | 前视左检测结果 |
| `/perception/detection/front_right` | `DetectionArray` | 前视右检测结果 |
| `/perception/detection/down_left` | `DetectionArray` | 下视左检测结果 |
| `/perception/detection/down_right` | `DetectionArray` | 下视右检测结果 |

vision 不发布图像 DDS 话题；请通过 go2rtc 的 `front`、`down`、
`front_annotated`、`down_annotated` 流查看视频。

go2rtc 视频流：

| 流名称 | 说明 |
|---|---|
| `front` / `down` | 前视/下视原始拼接视频 |
| `front_annotated` / `down_annotated` | 前视/下视 YOLO 识别后带框拼接视频 |

### position 发布话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/perception/objects` | `ObjectPositionArray` | 3D 物体世界坐标（含 `instance_id`） |

### basic_motion 接口

| 名称 | 类型 | 说明 |
|---|---|---|
| `/basic_motion` | `uv_msgs/action/BasicMotion` | 运动 Action Server |
| `/basic_motion/pose_info` | `uv_msgs/PoseInfo` | 机器人位姿（30Hz，含 roll/pitch） |
| `/zit6/cmd/setpoint` | `zit6_interfaces/ZitSetpoint` | 发往 MCU / sim_bridge 的运动设定值 |

### sim_bridge / MCU 状态话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/zit6/state/pos` | `Float32MultiArray[6]` | `[x, y, z, roll_rad, pitch_rad, yaw_rad]` |
| `/zit6/state/vel` | `Float32MultiArray[6]` | `[vx, vy, vz, vroll, vpitch, vyaw]`（机体系） |
| `/zit6/state/thr` | `Float32MultiArray[6]` | `[Fx, Fy, Fz, Mx, My, Mz]` |
| `/zit6/state/status` | `zit6_interfaces/ZitStatus` | MCU 状态汇总 |
| `/zit6/state/zithbt` | `std_msgs/UInt32` | MCU 心跳序列号 |

> `/auv/state`（`uv_msgs/AuvState`）当前**没有任何发布者**，`navigator` 仍订阅它，待迁移。
