# 话题频率与 QoS（T3b，实测）

> **本文件只装频率与 QoS。** 话题名/消息类型/方向/发布者/订阅者/证据的唯一事实源仍是
> `docs/interfaces/topic_contract.md`（那份由 `scripts/check_docs_contract.py` 校验）；
> 这里回答的是契约表格答不了的两个问题：**它跑多快**、**它的 QoS 是什么**。
>
> 数据来源：`snapshots/2026-10-10_sil-console/`（由 `scripts/dump_interfaces.sh --rates` 生成）。
> **快照必须带模式**：`sil-console` 没有相机图像、`sil-gpu` 有、HIL 的 `/zit6/state/*` 归真 MCU
> 且少两个转发节点——不带模式的频率数字没有意义。目前入库的只有 SIL console 一份，
> **sil-gpu 与 HIL 的快照仍是欠账**（见 `launch_entries.md` §5）。

## 0. 口径与陷阱（先读这段，否则数字会被误读）

1. **`ros2 topic hz` 是订阅侧测量，会低估**。大图 RELIABLE 话题上尤其严重
   （实测 1.93 Hz 对 9.5 Hz/眼，见 `workspace_sim/stonefish_notes.md`）。本表里的数字都是
   这个工具给的，只在**小消息**上可信；相机那几行在 console 模式下本来就无数据。
2. **"设计值"来自代码/契约，"实测值"来自快照**，两者不一致时以实测为准并**登记差异**
   （本轮就抓到一条，见 §2 的 B20）。
3. **`-` 的含义是"该模式下没有发布者或没有数据"**，不是"没测"。（`rates.txt` 的注释行同义。）
4. **QoS 的 depth 读不到**：本栈（Fast DDS + `ros2 topic info -v`）对 KEEP_LAST 深度一律报
   `UNKNOWN`，所以只能断言 reliability/durability，不能断言队列长度。
5. **场景声明的发布者（相机、DVL、IMU…）的"设计值"是场景 `rate=`**，不是我们的代码；
   改场景就会改这些数字。

## 1. SIL console 实测（2026-10-10，`canyon_vision_dvl_headless.scn`，`enable_ai:=false`）

| 话题 | 类型 | 实测 Hz | 设计/场景 | 归属 | 备注 |
|---|---|---|---|---|---|
| `/auv/odometry` | `nav_msgs/Odometry` | **30.111** | 30 | 引擎 | 跟物理步长 |
| `/auv/imu` | `sensor_msgs/Imu` | **99.899** | 场景写 200 | 引擎 | 受 100 Hz 物理步长上限压住（要给 200 得 `simulation_rate:=200`） |
| `/auv/dvl` | `stonefish_ros2/DVL` | **19.976** | 20 | 引擎 | |
| `/auv/dvl_altitude` | `sensor_msgs/Range` | **19.990** | 20 | 引擎 | 类型 2026-10-10 更正 |
| `/auv/pressure` | `sensor_msgs/FluidPressure` | **4.981** | 5 | 引擎 | |
| `/auv/thrusters_state` | `stonefish_ros2/ThrusterState` | **99.825** | 物理步长 | 引擎 | 类型 2026-10-10 更正；无订阅者 |
| `/sim/sensor/odom` | `nav_msgs/Odometry` | **30.013** | 同源 | `odom_sim` | 转发不重新采样 |
| `/sim/sensor/imu` | `sensor_msgs/Imu` | **99.861** | 同源 | `imu_sim` | |
| `/sim/sensor/dvl` | `uv_msgs/Dvl` | **20.115** | 同源 | `dvl_sim` | |
| `/sim/sensor/pressure` | `sensor_msgs/FluidPressure` | **5.015** | 同源 | `pressure_sim` | |
| `/zit6/state/pos` | `std_msgs/Float32MultiArray` | **29.949** | 30 | `nav_sim` | 60 Hz 定时器门控 |
| `/zit6/state/vel` | `std_msgs/Float32MultiArray` | **59.875** | 60 | `nav_sim` | |
| `/zit6/state/status` | `zit6_interfaces/ZitStatus` | **9.981** | 10 | `mcu_sim` | |
| `/zit6/state/thr` | `std_msgs/Float32MultiArray` | **29.943** | 30 | `mcu_sim` | |
| `/zit6/state/zithbt` | `std_msgs/UInt32` | **10.000** | 10 | `mcu_sim` | |
| `/basic_motion/pose_info` | `uv_msgs/PoseInfo` | **15.58** | 30（`create_timer(1.0/30.0)`） | `basic_motion` | **与设计值差一半 ⇒ B20（未修）**，见 §2 |

**该模式下端点存在但无数据**（`-`，不是缺失）：`/auv/{front,down}_cam/{left,right,stitched}`
与 `/sim/*_cam/*/image_color`（console 引擎不出图）、`/auv/thrusters_cmd`/`/zit6/cmd/setpoint`
（未发运动指令时 `mcu_sim`/`basic_motion` 不发）、`/zit6/cmd/{servo,light}`（`task_runner` 未启动）、
`/zit6/cmd/{agxhbt,ins}`（`hw_manager` 未启动 / 无发布者）、`/tf`（**有发布者，0 消息**，见 §3）、
`/rosout`、`/parameter_events`（ROS 自带，无业务数据）。

**未启动因而未实测**（`enable_ai/enable_nav/enable_task` 默认关）：`/perception/detection/*`、
`/perception/line/*`、`/perception/aruco/ids`、`/perception/objects`、`/nav/path`、`/task/status`、
`/zit6/cmd/pushrod`；HIL 专属的 `/zit6/sim/nav` 也不在 SIL 出现。
其中 `/perception/annotated/*` 是**全局无发布者**的缺口（GUI 在等它，`topic_contract.md` §9）。

## 2. 与设计值不一致的一条：`/basic_motion/pose_info`（B20）

`basic_motion._publish_pose_info` 由 `create_timer(1.0/30.0, ...)` 驱动（`basic_motion.py:159`），
契约与多份文档都写 "PoseInfo 30 Hz"，但实测**只有一半**：

| 场景 | `ros2 topic hz` | 独立 depth-1 计数 | 结论 |
|---|---|---|---|
| 完整 SIL console 栈 | 10.3~19.9（多次，抖动） | **15.58 Hz** | 发布侧确实 ≈15.6 Hz |
| **只跑 `basic_motion`**（无引擎、无 uv_sim） | — | **29.93 Hz** | 代码路径本身没问题 |
| 同窗口对照 `/zit6/state/pos` | 29.949 | **29.77 Hz** | 测量方法与机器都能跑 30 Hz |

所以这不是测量误差，而是**在完整栈里 `basic_motion` 的 30 Hz 定时器丢了一半节拍**
（单独跑就正常）。影响面：`position` / `navigator` / `task_runner` 都把它当 30 Hz 位姿源
（新鲜度减半）。登记为 **B20**（见 `roadmap.md`）。

**第一次修复尝试：无效，已如实记录。** 假设是"`_pose_cb`（30 Hz 状态订阅）与
`_publish_pose_info`（30 Hz 定时器）共用默认互斥回调组、互相串行"，于是给定时器单独开了
`MutuallyExclusiveCallbackGroup`。复测（同口径、完整栈）：**13.6 Hz**（改动前 15.6），
同窗口 `/zit6/state/pos` **26.8 Hz**（此前 29.8）⇒ 差异落在系统噪声内，**没有改善**。
机器有 **28 核、load ≈1.7**，`MultiThreadedExecutor` 也确实按 CPU 亲和数开了线程，
所以既不是"回调组互斥"也不是 CPU 饥饿；更像是**该进程内回调的延迟/GIL**：
`_vel_cb`(60 Hz) + `_pos_cb`(30 Hz) + 定时器(30 Hz) 挤在一个 Python 进程里，而 rclpy 定时器
**不补拍**，错过就丢。改动本身保留了（周期性发布与高频订阅分开是常规结构、代价为零），
但**它没有修好 B20**。下一步候选：① 在 `_pose_cb` 里直接发（去掉第二个 30 Hz 源）；
② 先给定时器加计数日志，判定"定时器没触发"还是"消息没送达"。
**在修好之前，任何"30 Hz 位姿"的说法都应带这个实测限定。**

## 3. QoS（实测）

- **发布者：全部 `RELIABLE` + `VOLATILE`**，包括 Stonefish 的 4 路相机图像话题
  （`create_publisher<Image>(topic, 10)`）与我们所有 Python 节点。**没有任何一条 best-effort**。
  这是大图话题在弱消费者下丢帧/积压的根因：RELIABLE 会重传，重传不完就积压。
- **订阅者：同样 `RELIABLE` + `VOLATILE`**（`basic_motion`、`nav_sim`、`mcu_sim`、`vision` 等）。
- **`/tf` 的发布者是引擎**（`stonefish_simulator_nogpu` 建了端点），但**实测 0 条消息**——
  场景没有 `<ros_base_link_transform publish="true"/>`，广播被那个 `if` 守卫关着。
  `/tf_static` **不存在**（不是"存在但没消息"）。B18 的结论不变，措辞按实测收紧。
- **depth 读不到**（`UNKNOWN`），别在文档里断言队列深度。

## 4. 维护方式

```bash
# 起栈（console SIL）
ros2 launch uv_sim_bringup sim_bringup.py headless:=true \
  scenario_desc:=canyon_vision_dvl_headless.scn enable_ai:=false
# 另一终端
scripts/dump_interfaces.sh --rates --mode sil-console
```

新增/改动节点后重跑一份快照，并**用新目录名**（日期 + 模式）入库；旧快照不覆盖，
它们是"当时确实这样"的记录。改本表时同一次改动里更新 `docs/README.md` 与
`roadmap.md` 的 T3b 状态。
