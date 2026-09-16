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
| 1 | ~~**launch 入口表 / 分层**：`sim_*`/`hil_*` 位于 `workspace_auv` 却引用仿真包（B1）；包级 launch 引用不存在的入口（B2）~~ | ✅ **T4 已完成**：仿真预设迁到 `workspace_sim/uv_sim_bringup`，`workspace_auv` 只留 `real_bringup.py`；入口清单与新旧映射见 `interfaces/launch_entries.md` |
| 2 | **`sim_bridge` 参数与话题归属**：T5 将把它拆成 7 个节点，§sim_bridge 每一行都会变 | T5 |
| 3 | **话题频率 / QoS 列**：需实时快照（`ros2 topic info -v` / `hz`）；本沙箱仅 console 模式可用且无相机（B13） | T3b |
| 4 | ~~话题表去重~~ | ✅ T3a 已完成：话题行收敛到 `docs/interfaces/topic_contract.md`，本文件与 `hil_architecture.md` 改为链接 |
| 5 | 本文件里仍是 **C++ 仿真器与 HIL 相关**的条目（`hil_bringup` 段）——当前阶段 SIL 优先，按其原文保留但**不参与验收** | 待 HIL 重新纳入（T7） |
| 7 | **引擎自带 launch 参数未收录**：`stonefish_simulator.launch.py` 有 6 个（`simulation_data`、`scenario_desc`、`simulation_rate`、`window_res_x`、`window_res_y`、`rendering_quality`），`stonefish_simulator_nogpu.launch.py` 有 3 个（前三个）。校验工具已显式跳过这两个文件（`UNCHECKED_LAUNCH_FILES`） | 待补（引擎文档，非 SIL 栈） |

> **维护规则（T3a 起）**：改任何 ROS 参数默认值或 launch 参数，**必须同一次改动里更新本文件**；
> 改话题名/消息类型，必须更新 `docs/interfaces/topic_contract.md`。
> 两者都已由 `python3 scripts/check_docs_contract.py --check` 强制（源代码级校验，无需跑栈）。

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
> `uv_bringup` 与 `uv_sim_bringup` 自身没有 entry point，只有 launch 文件；
> **谁启动谁**属 launch 层，入口清单见 `interfaces/launch_entries.md`。

### 包级 launch 现状 ✅（T4 重排后）

| launch 文件 | 引用的入口 | 状态 |
|---|---|---|
| `uv_hm/launch/hm_launch.py` | `hw_manager` + micro-ROS agent | ✅ T4 修复（原先引用不存在的 `uv_hm/sim_bridge`，B2） |
| `uv_control/launch/control_launch.py` | `basic_motion` | ✅ T4 修复（原先引用不存在的 `minimal_control`，B2） |
| `uv_perception/launch/perception_launch.py` | `vision` + `position` | ✅ 可用（T4 新增 `sim_mode` 参数） |
| `uv_sim/launch/bridge_launch.py` | `sim_bridge` | ✅ T4 新增（`hil_mode` / `pid_config_file`） |
| `uv_nav/launch/nav_launch.py` | `navigator` | ✅ 可用 |
| `uv_task/launch/task_launch.py` | `task_runner` | ✅ 可用 |

> 规则（T4 起）：**节点定义归各包，系统预设只做编排**。各包 launch 只声明自己的节点与
> 节点级选项；`uv_bringup/real_bringup.py`（实机）与 `uv_sim_bringup/{sim,hil}_bringup.py`
> （仿真）只 include 它们，并用 `condition` 决定起不起。因此 `enable_*` 门控**只存在于预设**，
> 包级 launch 里没有同名开关。这条由 `scripts/check_docs_contract.py` 的 `launch-targets`
> 检查强制：launch 里出现的 `executable` 必须在对应包 `setup.py`/`CMakeLists.txt` 里存在。

---

## vision 节点参数 ✅

### 运行模式

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `sim_mode` | bool | `false` | `true`: 订阅 `/auv/*/stitched` ROS 话题<br>`false`: 使用 V4L2 设备直接采集 |

> `uv_sim_bringup` 的 `sim_bringup.py` / `hil_bringup.py` 在启动 vision 时传 `sim_mode: True`
> （经 `uv_perception` 的 `perception_launch.py`）；`real_bringup.py` 传 `sim_mode: False`。
> 直接 `ros2 run` 时默认 `false`。

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

## 启动文件 Launch Arguments ✅（T4 重排后）

> 入口的分工与新旧映射见 `docs/interfaces/launch_entries.md`。本节只列**参数**。
> 规则：`enable_*` 门控只在**系统预设**里（`real_bringup` / `sim_bringup` / `hil_bringup`）；
> 包级 launch 只声明自己的节点级选项，不带同名开关。

### real_bringup.py（包 `uv_bringup`，实机唯一入口）

| 参数 | 默认值 | 说明 |
|---|---|---|
| `enable_agent` | `true` | 启动 micro-ROS agent（转发给 `hm_launch.py`） |
| `enable_motion` | `true` | 启用 `basic_motion` |
| `enable_ai` | `true` | 启用 vision + position（以 `sim_mode:=false` 启动） |
| `enable_nav` | `false` | 启用 navigator |
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

### hm_launch.py（包 `uv_hm`）

`hw_manager` + micro-ROS agent（真 MCU 的串口链路；agent 是 transport，`hw_manager`
负责 `/zit6/cmd/agxhbt` 心跳）。REAL 与 HIL 都 include 它。

| 参数 | 默认值 | 说明 |
|---|---|---|
| `enable_agent` | `true` | 启动 micro-ROS agent |
| `agent_bin` | 自动探测 | agent 可执行文件路径；优先 `~/micro_ros_agent_ws/install/...`，其次 `$PATH` |
| `serial_dev` | `/dev/ttyUSB0` | MCU 串口设备 |
| `serial_baud` | `921600` | 串口波特率 |

**用法：**
```bash
ros2 launch uv_hm hm_launch.py enable_agent:=false     # 只跑 hw_manager
```

### control_launch.py（包 `uv_control`）

声明 `basic_motion`，**无 launch 参数**（起不起由预设的 `enable_motion` 决定）。

### perception_launch.py（包 `uv_perception`）

| 参数 | 默认值 | 说明 |
|---|---|---|
| `sim_mode` | `false` | `true`: 订阅 Stonefish 相机话题 `/auv/*_cam/stitched`；`false`: V4L2 直采（下发给 vision 节点） |

**用法：**
```bash
ros2 launch uv_perception perception_launch.py
```

### bridge_launch.py（包 `uv_sim`，仿真侧）

| 参数 | 默认值 | 说明 |
|---|---|---|
| `hil_mode` | `false` | `true`: 只转发相机 + 透传推力（真 MCU 负责 PID）；`false`: 仿真 MCU |
| `pid_config_file` | 空 | 空 = 用包内 `config/sim_pid.yaml` |

**用法：**
```bash
ros2 launch uv_sim bridge_launch.py hil_mode:=true
```

### sim_bringup.py（包 `uv_sim_bringup`，SIL）

| 参数 | 默认值 | 说明 |
|---|---|---|
| `scenario_desc` | `wuurc_murc_2026_auv.scn` | Stonefish 场景文件名（在 `Data/` 内，可给绝对路径） |
| `headless` | `false` | `true` = console 仿真器 `stonefish_simulator_nogpu`（无渲染窗口、**无相机图像**） |
| `simulation_rate` | `100.0` | 物理步进率（固定步长）；**必须为正**——`0` 会让仿真循环停住且不发任何话题（2026-09-16 实测更正，见 `stonefish_notes.md`） |
| `window_res_x` | `2560` | 窗口宽（仅 GPU 模式） |
| `window_res_y` | `1440` | 窗口高（仅 GPU 模式） |
| `rendering_quality` | `high` | `low`/`medium`/`high`（仅 GPU 模式） |
| `enable_mcu` | `true` | 启用 `sim_bridge`（仿真 MCU） |
| `enable_motion` | `true` | 启用 `basic_motion` |
| `enable_ai` | `true` | 启用 vision + position（`sim_mode:=true`） |
| `enable_nav` | `false` | 启用 navigator |
| `enable_task` | `false` | 启用 task_runner |

**用法：**
```bash
ros2 launch uv_sim_bringup sim_bringup.py enable_ai:=true
# 等价于旧 sim_core.py（只有引擎 + 仿真 MCU）：
ros2 launch uv_sim_bringup sim_bringup.py enable_motion:=false enable_ai:=false
# 无图形环境（console 物理仿真）：
ros2 launch uv_sim_bringup sim_bringup.py headless:=true simulation_rate:=100.0
```

### hil_bringup.py（包 `uv_sim_bringup`）⏸

HIL：Stonefish + **真 MCU**。当前阶段 SIL 优先，**运行时验收暂缓**（静态核查通过）。

| 参数 | 默认值 | 说明 |
|---|---|---|
| `scenario_desc` | `underwater_xunyun.scn` | Stonefish 场景文件名 |
| `headless` | `false` | 同 `sim_bringup.py`（console 模式无相机 ⇒ 相机转发链路不可用） |
| `simulation_rate` | `100.0` | 物理步进率（固定步长，必须为正） |
| `window_res_x` | `2560` | 窗口宽（仅 GPU 模式） |
| `window_res_y` | `1440` | 窗口高（仅 GPU 模式） |
| `rendering_quality` | `high` | 渲染质量（仅 GPU 模式） |
| `enable_mcu` | `true` | 启用 `sim_bridge`（以 `hil_mode:=true`） |
| `enable_motion` | `false` | 启用 `basic_motion` |
| `enable_ai` | `false` | 启用 vision + position |
| `enable_nav` | `false` | 启用 navigator |
| `enable_task` | `false` | 启用 task_runner |
| `enable_agent` | `true` | 启动 micro-ROS agent（转发给 `hm_launch.py`） |
| `serial_dev` | `/dev/ttyUSB0` | MCU 串口设备 |
| `serial_baud` | `921600` | 串口波特率 |

**用法：**
```bash
ros2 launch uv_sim_bringup hil_bringup.py enable_ai:=true
```

> ⚠️ T4 变化（**未做运行时验证**）：agent 与 `hw_manager` 现在来自 `uv_hm/hm_launch.py`，
> 因此 HIL 会起 `hw_manager`。这是有意的——真 MCU 需要心跳才会解锁——但证据只到静态核查。

---

## 话题参考 🔗

**话题/消息类型的权威已迁至 `docs/interfaces/topic_contract.md`**（2026-09-16 T3a 建立，
逐行带代码/场景证据，`scripts/check_docs_contract.py --check` 可校验）。

契约文件按用途分节：

| 节 | 内容 |
|---|---|
| §1–§2 | SIL 仿真传感器与相机链路 |
| §3–§4 | 控制与执行机构、状态反馈（MCU 契约） |
| §5–§6 | 位姿与感知、任务与导航 |
| §7 / §7.5 | 服务与动作、栈外 GUI 消费者 |
| §8 | HIL 专属（⏸ 当前暂缓） |
| §9 | **缺口登记**（有订阅无发布等已知问题） |

参数与话题的关系：本文件只负责**参数**（谁有什么旋钮、默认值多少），
话题归属与消息类型一律看契约文件，此处不再复制——避免两处漂移
（此前这两个文件各维护一份话题表，已出现不一致）。
