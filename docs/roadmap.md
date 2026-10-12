# YouLong AUV 大计划：仿真传感器对齐 · 文档体系 · bringup 分离 · sim_bridge 拆分 · AQUA-SLAM 移植

> **文档定位**：这是**大计划**（workstream 级），只定义目标、边界、任务切分、验收与依赖。
> **用法**：每个 `T*` 任务在下一次进入 plan mode 时展开为**小计划**（文件级改动 + 测试命令），实现完成后回到本文档回填状态列。
> **维护规则**：任务的验收口径变更必须改本文档；实现细节不进本文档（进对应小计划或代码注释）。

| 状态图例 | 含义 |
|---|---|
| ☐ | 未开始 |
| ◐ | 部分完成 |
| ☑ | 已完成并通过验收 |

---

## 0. 一句话总览

先让**仿真与实机的传感器几何一致**（T1），再固化**坐标系/外参/接口契约**（T2、T3），然后**把 bringup 与 sim_bridge 拆成可维护的结构**（T4、T5），最后在这个干净地基上**移植 AQUA-SLAM**（T6），并预留实机接入（T7）。

---

## 1. 已确认事实与关键决定

### 1.1 硬件/仿真事实（本次核实）

| 项 | 事实 | 依据 |
|---|---|---|
| 实机 DVL 安装 | 安装 yaw = 0（与艇体系对齐），z ≈ +0.1 m | 用户确认 |
| 实机 DVL 波束方位 | **45°（DVL 斜置）**：四波束方位 45/135/225/315，即相对艇体系为 45° 斜向 | 用户确认 |
| 实机 DVL 波束倾角 | **30°（离竖直）** ⇒ `beam_angle=30`，AQUA-SLAM `α = 60°` | 用户确认 |
| 仿真 DVL 现状 | `xyz=(-0.375, 0, 0.2)`、`rpy=(π, 0, −0.785)`、`beam_angle=30°`、`beam_positive_z` 默认 false | `workspace_sim/src/stonefish_ros2/Data/xunyun_fixed.scn:277-291` |
| Stonefish 波束方位约定 | 传感器系内固定 `45° + i·90°` | `stonefish/Library/src/sensors/scalar/DVL.cpp:84` |
| Stonefish 波束倾角约定 | `beamAngle` = 与传感器 **z 轴**夹角；`dir = z·cos + (x·cosα + y·sinα)·sin`，`dirFactor = beamPosZ ? +1 : −1` | `DVL.cpp:85-89`；解析 `ScenarioParser.cpp:3520-3528` |
| Stonefish DVL 速度 | `v = R_sensor←worldᵀ · v_point`，**传感器系内**，且**不因 beamPosZ 变号** | `DVL.cpp:155`（dirFactor 只作用于射线方向） |
| Stonefish `<origin rpy>` | `R = Rz(yaw)·Ry(pitch)·Rx(roll)` | `stonefish/CMakeLists.txt:70`（`BT_EULER_DEFAULT_ZYX`）→ `btQuaternion.h:93-104`；解析 `ScenarioParser.cpp:5189` |
| 仿真 IMU | 20 Hz，无噪声配置 | `xunyun_fixed.scn:293-299` |
| 仿真前视双目 | 1280×960 @10 Hz，hfov 57.19°，基线 0.10 m | `xunyun_fixed.scn:301-315` |
| DVL 被谁消费 | 仅 `sim_bridge._dvl_cb` | `workspace_sim/src/uv_sim/uv_sim/sim_bridge.py:166` |

### 1.2 由此确定的决定

| 决定 | 内容 | 理由 |
|---|---|---|
| **D1** | 仿真 DVL 安装改为 **与实机一致**：`xyz=(-0.375, 0, 0.1)`、`rpy=(0, 0, 0)`，并显式 `beam_positive_z="true"` | 实机 DVL 为「yaw=0 安装 + 45° 斜置波束」，故仿真须让传感器系与机体系对齐、波束在艇体系内呈 45° 斜向；这样速度输出即机体系速度（同时见 §1.3） |
| **D2** | AQUA-SLAM 的 DVL 参数：**β = (45°,135°,225°,315°)**（与 Stonefish 及 A50 传感器系约定一致）；**α = 60°**（= 90° − 30°，波束离竖直 30° 已确认） | 仓库 YAML 现用论文 A50 的 α=67.5°（离竖直 22.5°）；用错会让水平速度放大 `cos60/cos67.5 = 1.307`（+31%）、垂直速度偏低 6%。几何依据：`AQUA-SLAM/src/DVLGroPreIntegration.cpp:54-58`（`mE` 行向量 = 波束方向）、反解 `:552`；DVL↔IMU 跨系关系 `AQUA-SLAM/src/ImuTypes.cc:545` |
| **D3** | 每平台**独立外参 YAML**（sim / real 各一份），严禁共用 | DVL 安装 yaw/z、IMU 噪声、相机内参都不同；共用会把一台的几何错误带进另一台 |
| **D4** | 仿真中 α 的「自洽性」不构成正确性证明：仿真用 `v_beam = E(α,β)·v` 合成再反解，α 取任何值都能跑对；**只有实机会暴露 α 错误** | 因此 α/β 必须按硬件真值填写，不能靠仿真调出来 |

> **D4 的重要性**：这是本项目最容易「仿真全绿、实机崩掉」的地方，T2 会把它写成显式验收项。

### 1.3 本次核实中发现的**现存缺陷**（T1 顺带修复）

**缺陷：仿真 DVL 的传感器系相对机体系旋转 45°（且 y、z 轴取反），已实际污染控制栈。**

- 现状 `rpy=(π, 0, −45°)` ⇒ `R_bs = Rz(−45°)·Rx(180°)`（Stonefish 约定 `R = Rz·Ry·Rx`）
- `/auv/dvl` 发布的是 **DVL 系**速度：`v_s = R_bsᵀ·v_b = Rx(180°)·Rz(45°)·v_b`
  → 水平分量被旋转 +45° 且 y 取反；heave 取反（`w → −w`）
- 沿航向稳态运动（目标方向在机体系内为 α）时，`basic_motion` 的沿航向速度变为
  **`v_along = V·cos(2α + 45°)`**（推导：`v_along = (cos α, sin α)·v_s`）
  → α=0° 时仅 `0.707V`；**α=22.5° 时为 0**；α=67.5° 时为 **`−V`（符号翻转）**
- 传播链：`/auv/dvl` → `sim_bridge._dvl_cb`（`sim_bridge.py:671-674`）→ `/zit6/state/vel`
  （`sim_bridge.py:708-716`）→ `basic_motion._vel_cb`（`basic_motion.py:143,191-198`）
  → `v_along`（`basic_motion.py:479-484`）→ **WMOVE/WTRAVEL 步进收敛判据**
  （`converging = (e_along > 0 and v_along > 0) or …`；`t_remaining = e_along / v_eff`）

| 影响面 | 现状 | T1 之后 |
|---|---|---|
| WMOVE 步进收敛（`v_along` / `converging` / `t_remaining`） | **受污染**，误差随航向在 −100%~+100% 间变化 | 正确（`R_bs = I`） |
| SIL 速度模式控制（`_velocity_pid_step` 用 `self.vel['x'/'y'/'z']`，`sim_bridge.py:620-631`） | **潜在缺陷**（当前 `basic_motion` 只用 `CK_POS` 位置模式，尚未触发） | 正确 |
| heave 符号（`/zit6/state/vel[2]`） | 反号 | 正确 |

> 因此 T1 不是「美观问题」而是**功能修复**，同时也是 AQUA-SLAM 外参正确性的前提。
> **独立佐证（B11）**：既有权威文档 `docs/hil_architecture.md:112` 早已把 `/auv/dvl` 写成「4 波束 DVL **机体坐标系速度**」（`:245` 的 MCU 固件注释亦然），而代码一直没做到；T1 让两者一致。

---

## 2. 现状基线（实测证据）

| # | 现状 | 证据 |
|---|---|---|
| B1 | 入口：`uv_bringup`（workspace_auv）内含仿真入口，违反「workspace_auv 不依赖仿真」 | `launch/sim_bringup.py`、`launch/sim_core.py`、`launch/hil_bringup.py` 直接引用 `uv_sim`/`stonefish_ros2` |
| B2 | 各包 launch 文件陈旧失效 | `uv_control/launch/control_launch.py` 引用不存在的 `minimal_control`；`uv_hm/launch/hm_launch.py` 引用 `uv_hm/sim_bridge`（实际在 `uv_sim` 且无该 entry point）。实测 entry points 仅 `basic_motion/hw_manager/navigator/vision/position/task_runner` |
| B3 | bringup 内联 `Node(...)`，与各包 launch 重复定义 | 三个 bringup 文件均如此 |
| B4 | ~~`sim_bridge.py` 单节点 765 行承担 5 类职责~~ **已修（T5）** | 原状：MCU 仿真 + 传感器聚合 + 相机转发 + HIL 分支 + 状态发布挤在一个文件里。现拆成 7 个单职责节点（`odom_sim`/`imu_sim`/`dvl_sim`/`pressure_sim`/`nav_sim`/`mcu_sim`/`camera_relay`）+ 5 个 ROS-free 共享模块，控制数学有拆分前后的数值黄金回归 |
| B5 | 接口无权威文档，已存在契约漂移 | GUI 订阅 `/perception/annotated/<cam>`（`visualization/auv_visualizer.py:206`）**全局无发布者** |
| B6 | 记忆/agent 文档绑定旧结构 | `CLAUDE.md`(11 处)、`workspace_auv/docs/debug_guide.md`(14)、`.claude/commands/auv-sim-master.md`(8)、`.github/agents/auv_master.md`(18)，另加根 `docs/node_parameters.md`（记录 `sim_bridge` 参数、`sim_core.py`、旧 launch 参数） |
| B7 | 参考架构已存在于 `robotcup` 分支（可借鉴，不可直接合并：Foxy、包名 `uv_camera`、落后 17 提交） | robotcup: `launch/{real,sim,hil,core_sim,readiness,observability}.launch.py`、`uv_sim/{camera_passthrough.py,thrust_mixer.py,coordinate_convention.py,config/profiles/*,test/*}` |
| B8 | 磁盘：**已腾出**，`/` 剩余 **12 GB**（原 2.6 GB） | `df -h /`；满足 T6 的 ≥3 GB 需求 |
| B9 | 磁盘外环境就绪 | ROS 2 Jazzy；GCC 13.3（无 clang）；CMake 3.28；OpenCV 4.6；Eigen 3.4；Boost 1.83；fmt 9.1；PCL 1.14；`ros-jazzy-{octomap-msgs,pcl-ros,sophus}` 可 apt 安装 |
| B10 | **仓库根 `docs/` 已有入库的权威文档**（此前遗漏，T3 必须并入而非另起一套） | `docs/node_parameters.md`（293 行：全部节点/参数/launch 参数/话题参考）、`docs/hil_architecture.md`（312 行：micro-ROS、上下行话题表、SIL/HIL 对比、启动与验证）、`docs/auv-architecture.{html,json,png}`（archify 架构图产物）。被 `CLAUDE.md:243`、`.claude/commands/auv-sim-master.md:9`、`.github/agents/{.agent,stonefish_master.agent,auv_master,ros2-uuv-optimizer}*.md`、`workspace_auv/docs/debug_guide.md:324`、`workspace_sim/stonefish_notes.md:63,374` 引用为「权威来源」 |
| B11 | **既有文档已写下「正确契约」，但代码没做到**（T1 的独立佐证） | `docs/hil_architecture.md:112` 把 `/auv/dvl` 描述为「4 波束 DVL **机体坐标系速度**」，`:245` 的 MCU 固件注释同样写 `// /auv/dvl → body_velocity`；而 §1.3 证明当前仿真发布的是旋转 45° 的 DVL 系速度 |
| B12 | ~~**odometry twist 被二次旋转**~~ **已修（2026-09-15）** | 根因：Stonefish 的 `/auv/odometry` twist 本来就是机体系（`Odometry.cpp:59,65`：`v = R_sensor←worldᵀ · v_point`；`PublishOdometry` 把 header 设为 `world_ned`、child 设为传感器名），而文档曾写它是世界系。两处代码照错的假设又乘了一次 yaw（= `R(−ψ)`）：① `_sim_nav_odom_cb` → HIL 的 `/zit6/sim/nav`（固件契约要求 `vel_body`，`third_party/AUV_zit6_cmake/docs/protocols/与AGX.md:301-307`）；② `_odom_cb` → SIL 的 `/zit6/state/vel`（60 Hz 发布，`self.vel.x/y/z` 被 `_dvl_cb` 20 Hz 与 `_odom_cb` 30 Hz 竞争写）。**修复**：新增 `sim_bridge.body_twist_from_odometry()` 作为唯一编码处，两条回调直接取 twist 分量，删除错误的 yaw 旋转与死状态 `vel_world`。**验证**：`test_odometry_frame.py`（5 例，ψ=90° 合成消息）——改动前 **2 failed / 3 passed**（`vel['x']` 实测 −0.19999 而非 0.3，正是 `R(−90°)·(0.3,−0.2)`），改动后 **5 passed**。T5 的 `nav_sim` 只需继承该契约（`body_twist_from_odometry` 可直接复用） |
| B14 | ~~**PID 参数多源、且服务值与生效值不一致**~~ **已修（2026-09-15）** | 原状：① `_init_full()` 内**两份硬编码**——`_pid_params` 字典（`/zit6/get_params` 返回它）与 `_Pid` 对象（真正生效）；两者漂移：`pos.z.ki` 报 30 实为 80、`vel.x.kp` 报 300 实为 600；② yaw 串级内环 `pid_yaw_rate` **既不在字典也不在服务面** → 读不到也调不了；③ `uv_hm/config/pid_params.yaml` 自称 sim_bridge 默认参数却**无人读取**（值还跟错的那份一致）；④ `uv_hm/config/pid_parameters.json` 是 MCU/AGX 的另一套参数体系（p/i/d + output_limit，8 轴），同样无人读取。**修复**：新增 `workspace_sim/src/uv_sim/config/sim_pid.yaml` 作为唯一事实源（9 通道，含 yaw_rate 与 `z_thrust_ff`），同一份数据既建服务视图又建 `_Pid` 对象；`pid_config_file` 参数可指向调参副本；删除死文件 `uv_hm/config/pid_params.yaml`。**验证**：`test_pid_config.py`（6 例）钉住"文件=服务=生效对象"三者一致，并证明换文件真的改变增益 |
| B13 | **沙箱限制（2026-09-16 重写）** | **相机图像**仍需 GPU/X 会话（无 `/dev/dri` → GPU 版 `XGLXCreateContext BadValue`）。但"必须有 GPU 才能验证控制闭环"**是错的**：`strip_vision_sensors.py` 去相机 + **正的** `simulation_rate`（默认 100；`0` 会让仿真停住、一条话题都不发）后，console 模式实测 `/auv/odometry` 30.0 Hz、`/auv/imu` 100.2 Hz、`/zit6/state/{pos,vel}` 30/60 Hz；BasicMotion `START`+`SET(0.5,0,3.0,0)` 端到端到位（x/y/z 误差 ≤0.05 m）；`check_dvl_frame.py` 静态段与四段航向均采到有效样本。旧结论两处错误：①「`<rate> 0`（实时模式）」写反（根因见 `stonefish_notes.md` §7）；②「话题实测在线」当时只看到话题列表、并无数据流。**仍受限**：无相机图像 ⇒ `vision`/`position` 链路、GPU 渲染路径与 TF（B18）未验证 |
| B15 | ~~**相机 offset 与场景差 0.2 m（N1）**~~ **已决（2026-09-15）** | `position.py` 的 `offset` z（0.076/0.0645）与 `line_follower.py`、`arrow_surfacer.py`、`docs/perception.md` 四处一致（4:1），场景 `xunyun_fixed.scn` 的 `0.276`/`0.2645` 只此一处——**以代码为准，改场景**（前视 `0.276→0.076`、下视 `0.2645→0.0645`）。**验证**：`derive_extrinsics.py --check` 绿、`test_frames_geometry.py` 新增两例钉住（场景 z == 权威 offset；三模块 offset 一致）、pytest 47 passed。残留下限：这组数字本身仍来自早期估算，未经实机复测；若日后量出别的值，改四处 + 场景再重跑生成器即可 |
| B16 | **AQUA-SLAM 参考检出的外参与波束角是 blue_gx5 的（N2）** | `AQUA-SLAM/data/underwater_orbslam3_blue_gx5_{short,long}.yaml`：`alpha=[1.1780]×4`(=67.5°)、`beta=[0.7854]×4`(=45°，且是标量不是逐波束)、`Tbc=(0.4,0,0.1)`；`dvl_model/dvl.cpp:508` 另硬编码 `ALPHA=67.5°`。与 XUNYUN（α=60°、β=45/135/225/315、相机 `(0.23,−0.05,0.076)`）不符。**后果**：水平速度 +30.7%（`cos67.5/cos60=1.3066`）。**处理**：不改参考检出，改在 `extrinsics_reference.md` §5 列成 T6 逐项必改清单 |
| B17 | **`alpha` 一词两义（N3）** | `check_dvl_frame.py` 曾用 `alpha` 表示艇的**航向**，而 AQUA-SLAM 与 `docs/frames/` 用 α 表示**波束离水平仰角**——这正是 B16 那类错误的高危来源。**已修**：统一 ψ=航向、θ=离竖直、α=离水平；脚本变量与日志改名，行为不变 |
| B18 | **TF 从未开启，而文档曾断言它存在** | `ROS2SimulationManager.cpp` 的 TF 广播套着 `if (publishBaseLinkTransform_)`，该标志由 `ROS2ScenarioParser.cpp` 从 `<robot>` 子元素 `<ros_base_link_transform publish=...>` 读取，默认 `false`（`ROS2SimulationManager.h`）；本仓库 15 个 `.scn` **一个都没写** → 仿真**完全没有 TF**（`/tf` 实测为空，`dump_tf_tree.py` exit 1）。**T2 文档初稿误称"唯一 TF `world_ned → Vehicle/base_link`"**，并把 `ROS2SimulationManager.cpp:395` 当证据（忽略了 `if` 守卫）。**已修**：`frame_convention.md` §3 改写为"当前无 TF"+开启方式+守卫坑；`extrinsics_reference.md` 删"并广播 TF"；`dump_tf_tree.py` 改为读场景 `<robot name>` 且区分"没开 TF"与"开了没数据"；新增 2 例测试钉住（无场景声明 TF、工具从场景读名字）。**附带发现**：Canyon 场景把艇命名为 `XUNYUN`（非 `Vehicle`），工具原先硬编码 `Vehicle`，一开 TF 即误报 |
| B19 | ~~**`/zit6/state/vel` 有两个写者，且两者物理含义不同（T1 验收卡在这一条上）**~~ **已修（T5，由"分开"而不是"改公式"）** | 原状：`sim_bridge._odom_cb` 写的是**机体系原点处**的 odometry twist（30 Hz），`_dvl_cb` 写的是**DVL 位置处**的速度（20 Hz，= 原点速度 + `ω×r_dvl`，`r_dvl=(-0.375,0,0.1)`）——两者在**有角速度时相差 `0.375·\|ω_z\|`**。实测：原地 90° 回转时 `dvl.y − odom.y = +0.037` 而 `ω_z=−0.099`，`−0.375·ω_z = +0.037`（三位小数吻合）；按 `MAX_YAW_RATE=0.35` 折算最大可差 **0.13 m/s**，正是 `check_dvl_frame.py` 里 `stv err` 报的 0.033~0.150。**T5 修复**：`/zit6/state/vel` 只剩 `nav_sim` 一个写者、线性部分 = **原点** twist；DVL 处的值走 `/sim/sensor/dvl.velocity`（`dvl_sim`，含杆臂），并另带 4 路合成波束。同轮还修掉该脚本的 `along ratio` 度量 bug：它把 DVL 速度投影到**未做杆臂修正**的 `linear` 上，于是转得越快的腿比值越偏（psi=90° 实测 1.117），改为投影到 `expected` |
| B20 | **`/basic_motion/pose_info` 的频率只有设计值的一半**（2026-10-10 T3b 发现，**未修**） | 设计：`basic_motion.py:159` `create_timer(1.0/30.0, self._publish_pose_info)`，`PoseInfo` 被 `position`/`navigator`/`task_runner` 当作 30 Hz 位姿源。实测：完整 SIL console 栈里 **15.58 Hz**（独立 depth-1 计数器，10 s 窗口），同窗口对照 `/zit6/state/pos` **29.77 Hz**（`ros2 topic hz` 给 29.949）⇒ 不是测量方法问题；**只跑 `basic_motion`（无引擎/无 uv_sim）时 29.93 Hz** ⇒ 代码路径本身能跑满，是**在完整栈里丢了一半节拍**。怀疑方向：`MultiThreadedExecutor` 下 30 Hz 的 `_pose_cb`（订阅 `/zit6/state/pos`）与 30 Hz 的 `_publish_pose_info` 定时器**共用默认互斥回调组**，互相串行 + 负载下 GIL 抖动，而 rclpy 定时器不排队 ⇒ 丢的就是丢了。影响：三个下游节点的位姿新鲜度减半。**第一次修复尝试（2026-10-10）无效并已如实记录**：给定时器单独开 `MutuallyExclusiveCallbackGroup` 后复测 13.6 Hz（改动前 15.6），同窗口 `/zit6/state/pos` 26.8（此前 29.8）⇒ 落在系统噪声内；机器 28 核、load≈1.7、`MultiThreadedExecutor` 按 CPU 亲和数开线程，所以既不是回调组互斥也不是 CPU 饥饿，更像**该进程内回调延迟/GIL**（`_vel_cb` 60 Hz + `_pos_cb` 30 Hz + 定时器 30 Hz，而 rclpy 定时器不补拍）。下一步：① 在 `_pose_cb` 里直接发（去掉第二个 30 Hz 源）；② 先给定时器加计数日志，判定"定时器没触发"还是"消息没送达"。改完必须在**完整栈**里复测而非单节点 |
| B21 | ~~**仿真忽略 `control_key` 的 frame 位（速度/推力），与固件不一致**~~ **已修（2026-10-12）** | 原状：`mcu_control.apply_setpoint` 把 `frame = control_key & 0x10` 算出来后**只在 mode==0（位置）分支里读**（`mcu_control.py:181`），所以速度（mode 1）与推力（mode 2）**完全忽略 frame 位** —— `0x01`（世界系速度）与 `0x11`（机体系速度）行为完全一样。而**真实固件是支持的**：`third_party/AUV_zit6_cmake/UserApp/Component/Chassis/SetpointRouter.cpp` 对 VELOCITY 调 `applyRotationToBody(world6, body6, roll, pitch, yaw)`、对 ACTUATOR 调 `applyWrenchToBody(...)`，两者都取 `R_ZYX^T`（`MathUtils.hpp`）。影响：SIL 里调好的世界系速度/推力指令上实机是另一种行为 —— 正是这套栈存在的意义所要防的静默 SIL/实机分歧。**修复**：`mcu_control` 新增 `world_to_body_vector()`（用 `uv_sim.geometry.euler_rpy_to_matrix` 取转置，仓库规定 rpy 复合顺序的唯一实现），速度/推力分支在 `frame==0` 时先转换**再**逐轴套 mask（与固件顺序一致：固件是先 `converted_val` 再按 body 轴套 mask）。**为什么原来的黄金回归抓不到**：`test_mcu_control.py` 在调 `apply_setpoint` 前把状态清零（`controller.pos.update(ZERO_POSE)`，三个角全 0），level 姿态下 `R^T = I`，两坐标系数值相同 ⇒ 该用例对 frame 位**完全不可见**。**新增 8 例测试**用非水平、带偏航的姿态钉住：世界系速度按 `R^T` 旋转且与 `euler_rpy_to_matrix` 对拍、机体系直传不旋转、世界系推力同理、mask 在旋转**之后**生效（mask 指机体轴）。旧代码下 5 例失败（yaw=0 那例正确通过，因为两系重合）。**实测（SIL console，yaw≈95°）**：世界系 x=+0.35 让机体速度落在 **−y**（-0.042），机体系 x=+0.35 落在 **+x**（+0.080）；世界位移分别为北 +0.147 m / 东 +0.331 m —— 修复前两者都会朝机头走 |


---

## 3. 任务分解

### T1 ◐ 仿真传感器对齐（DVL / IMU / 前视双目）

> **状态（2026-10-10 更新）**：场景改动、静态/解析式验证（V1）已完成；**V2/V3 端到端验证已于
> 2026-09-16 在 GPU 会话与 console 两种模式下跑过**（`dvl err` 2.5~2.7 mm/s、四段
> `along ratio` 0.997~0.999 ⇒ A1-2/A1-3/A1-4 通过）。`stv err` 当时偏高是 **B19 双写者**，
> **B19 已由 T5 修复**（2026-10-10 复测 `stv err` 0.003~0.021，四段全通过）。
> 仍欠：**A1-5 只验证了 2/4 个场景**（其余场景可用 console 补跑）。

**目标**：让仿真传感器几何与实机一致，并把为 AQUA-SLAM 所需的采样率/噪声补齐。

**改动点**
1. `workspace_sim/src/stonefish_ros2/Data/xunyun_fixed.scn` DVL 块（精确 diff 见附录 A）
   - `origin rpy="3.1416 0 -0.785003"` → `rpy="0 0 0"`
   - `origin xyz="-0.375 0 0.2"` → `xyz="-0.375 0 0.1"`
   - `<specs beam_angle="30.0"/>` → `<specs beam_angle="30.0" beam_positive_z="true"/>`
   - 更新块内中文注释（当前注释描述的是旧的 π 翻转 + −45° 偏航，会误导后人）
2. 同文件 IMU 块：`rate="20.0"` → `"200.0"`，并补 `<noise>`（见下）
3. 同文件前视双目：`rate="10.0"` → `"20.0"`（与论文数据集帧率一致，利于 ORB 时序假设）
4. 更新受影响场景的头部说明：`canyon_vision_dvl.scn:19-21`（DVL 行）等

**噪声注入换算**（AQUA-SLAM 的 `IMU.NoiseGyro/NoiseAcc` 是连续时间噪声密度，Stonefish `<noise>` 取**标准差**）：
`σ = 密度 × √rate` → `NoiseGyro=1e-3`、`NoiseAcc=1e-2`、rate=200 Hz ⇒
`angular_velocity σ = 1e-3×√200 = 0.0141`、`linear_acceleration σ = 1e-2×√200 = 0.1414`：
```xml
<noise angle="0 0 0" angular_velocity="0.0141" linear_acceleration="0.1414"/>
```
（语法依据：`stonefish/Library/src/core/ScenarioParser.cpp:3456-3511`，`setNoise` 见 `IMU.cpp:104-116`）

**验收**
- A1-1 仿真入口启动 `scenario_desc:=canyon_vision_dvl.scn` 后：
  - `/auv/dvl` 频率 ≈20 Hz，`status` 为有效底锁，`altitude` 比改动前**增加约 0.1 m**（DVL 上移 0.1 m 所致）
  - `/auv/imu` 频率 ≈200 Hz
- A1-2 `/zit6/state/vel` 线性分量与 `/auv/odometry` 的机体速度一致（误差 < 5%）
- A1-3 **航向扫描回归（专捕 §1.3 缺陷）**：分别沿机体系 α = 0° / **22.5°** / 45° / 90° 做匀速平移，
  `v_along = (cos α, sin α)·v_body` 应≈真值速度（误差 <5%）。
  *改动前 α=22.5° 时 `v_along ≈ 0`、α=67.5° 时符号翻转，该测试必然失败——这正是它的价值。*
- A1-4 静止 30 s：`/auv/dvl` 速度 ≈0，无虚假横向分量
- A1-5 `canyon_vision_dvl.scn` 等 4 个包含 `xunyun_fixed.scn` 的场景均可正常加载（`sauvc_2026_finals.scn`、`underwater_xunyun.scn`、`wuurc_murc_2026_auv.scn`）

**依赖**：无（可立即开始）
**预估**：0.5 人日
**风险**：① `beam_positive_z="true"` 的解析/行为需实测确认（若射线方向反了，症状是 DVL 无底锁、`status=3`）。回退方案 A：确认后修 Stonefish 的 `DVL` 解析（我们有源码 `/home/origin/stonefish`）；回退方案 B：保留 `rpy=(π,0,0)`（传感器 +z 朝上、y/z 取反），在 `dvl_sim`/`nav_sim` 与 `T_dvl_c` 中**显式**编码 `diag(1,−1,−1)` 翻转——数学上等价，但多一层易错环节，必须在 `extrinsics_reference.md` 写明。② IMU 提到 200 Hz 会增加仿真与话题负载，若 CPU 紧张可先只改 DVL、把 IMU 放在 T6 前一并处理。

**改动与验证记录（2026-09-15）**

| 项 | 结果 |
|---|---|
| 场景改动 | `xunyun_fixed.scn`：DVL `rpy="0 0 0"`+`xyz="-0.375 0 0.1"`+`beam_positive_z="true"`；IMU `rate=200` + `<noise angular_velocity=0.0141 linear_acceleration=0.1414>`；前视双目 `rate=20`。`canyon_vision_dvl.scn` 头部说明同步（含 DVL 底距 1.8→1.9 m） |
| V1 改动前 | `pytest workspace_sim/src/uv_sim/test/test_scene_sensors.py`：**7 failed / 4 passed**（DVL z=0.2、无 `beam_positive_z`、IMU 20 Hz 无噪声、前视 10 Hz；解析式两条也失败：`R_body←sensor ≠ I`、波束方位 0/90/180/270） |
| V1 改动后 | **11 passed**。其中两条解析式断言直接证明本次核心：`R_body←sensor = I`（⇒ `/auv/dvl` 即机体系速度）、波束在艇体系内为下视 30° 倾角 + 45/135/225/315 方位 |
| V2/V3 端到端 | **未执行**（沙箱限制见 B13）。命令与判据：`python3 workspace_sim/tools/check_dvl_frame.py`（需 SIL 在跑；预期静态 30 s 后 4 个航向段全 PASS，`along ratio ≈ 1.0`；改动前签名是 α=0° 时 0.707、α=22.5° 时 ≈0）。V3 快检：`ros2 topic hz /auv/imu`≈200、`/auv/dvl`≈20 且 `altitude>0` |
| 顺带修复 | **B12**（odometry twist 二次旋转）：新增 `body_twist_from_odometry()` 统一编码，`_odom_cb` 与 `_sim_nav_odom_cb` 去掉多余 yaw 旋转，删除死状态 `vel_world`；`SimBridgeNode.__init__` 支持 `node_name/**kwargs` 以便测试。回归测试 `test_odometry_frame.py`：改动前 **2 failed / 3 passed**（`vel['x']` = −0.19999 而非 0.3），改动后 **5 passed** |
| 文档同步 | `workspace_sim/stonefish_notes.md`（话题表速率、DVL 安装说明、无头模式两个坑、odometry twist 约定）、`docs/hil_architecture.md`（IMU 200 Hz、带宽 40 KB/s、**顺手修正 odometry twist 坐标系错误**）、`CLAUDE.md`（Sim sensor geometry 段落） |
| 事故与恢复 | 2026-09-15 21:56 `uv_sim/sim_bridge.py` 被写成 **0 字节**（非本会话的编辑操作：其前一次成功写入后 `grep`/`py_compile`/测试均正常，21:56 那次写入来源未定位——若有编辑器打开过该文件，优先怀疑编辑器保存）。症状：启动仿真 `ImportError: cannot import name 'main'`。处置：`git checkout HEAD -- <file>` 恢复 765 行基线并**按原 5 处改动重放**（`git diff --stat` = 44+/28−，与首版一致），测试全绿。**加固**：新增 `test_entry_points.py`（3 例）——模块 <20 kB 或缺 `class SimBridgeNode`/`def main(`/`def body_twist_from_odometry(` 即失败；已用 0 字节副本实测能命中 |
| 新增资产 | `workspace_sim/src/uv_sim/test/{test_scene_sensors.py（V1，11 例）,test_odometry_frame.py（B12 回归，5 例）,test_entry_points.py（入口点守卫，3 例）,test_pid_config.py（B14 回归，6 例）}`、`workspace_sim/src/uv_sim/config/sim_pid.yaml`（PID 单一事实源）、`workspace_sim/tools/check_dvl_frame.py`（V2，待跑）、`workspace_sim/tools/strip_vision_sensors.py`（去相机变体，无头跑物理用）。**本仓测试共 25 例，`python3 -m pytest workspace_sim/src/uv_sim/test/ -q` 应全绿**。构建模式提醒：`colcon build --symlink-install` 才是"源码改动即时生效"（`build/uv_sim/uv_sim → src/uv_sim/uv_sim`）；不带该参数会把包**拷贝**进 `install/`，此后必须重新 build（判据：`ls -l workspace_sim/install/uv_sim/lib/python3.12/site-packages/` 有 `uv-sim.egg-link` 才是软链模式） |

---

### T2 ◐ 坐标系与外参基线

> **状态（2026-10-10 更新）**：静态部分已完成并全部可复跑（脚本 + 单测，现随全套 143 例绿）；
> `dump_tf_tree.py` 的在线核对**已在 console 模式跑过**（健康状态就是 exit 1 + "no TF received"，
> 2026-10-10 快照进一步确认 `/tf` 有发布者但 0 消息、`/tf_static` 不存在）。
> 仍欠：**实机外参的实测复测（N1，待硬件）**。

**目标**：把「谁在哪个系、角度单位是什么、外参怎么推」固化成可核对的文件，并把 DVL 数学写成可执行校验。

**已交付**
- `docs/frames/frame_convention.md`：四套坐标系、度/弧度分界表（`.scn` rpy=rad 但 `beam_angle`=deg）、rpy = `Rz·Ry·Rx`、TF 树现状（**只有** `world_ned → Vehicle/base_link`，无传感器静态 TF）、odom 系、`type_mask` 反逻辑、相机光学系、`/auv/thrusters_cmd ∈ [−1,1]`；每条约定带**文件+符号**锚点（不用行号区间）
- `docs/frames/extrinsics_reference.md`：仿真表（8 传感器）+ 实机表（带**验证状态列**）+ `T_b_c/T_gyro_c/T_dvl_c` 推导 + **与 AQUA-SLAM `Tbc` 的逐元素对照**（证明记号没搞反）+ θ/α/β/ψ 术语表 + α 敏感度表 + **D3 每平台独立 YAML** + **T6 移植必改清单**（N2）+ 两项 T6 前置关卡的判据
- `workspace_sim/src/uv_sim/uv_sim/geometry.py`：几何唯一真相点（`euler_rpy_to_matrix`、`invert_rigid`、`dvl_beam_directions`、`beam_velocity_to_body`、`beam_inversion_sensitivity`、`parse_scn_sensors`）；纯 numpy，无 ROS 依赖
- `workspace_sim/tools/derive_extrinsics.py`：从 `.scn` 推导全部外参 + `--emit/--out/--check`；`--check` **解析文档里的 fenced YAML 块并与场景对拍**（文档漂移即非 0 退出）
- `workspace_sim/tools/check_dvl_geometry.py`：`(EᵀE)⁻¹Eᵀ(E v) = v`（实测 `1.1e-16`）、θ→α 换算、`beam_positive_z` 回归、α 敏感度表（含 67.5° 的 +30.66%）
- `workspace_sim/tools/dump_tf_tree.py`：在线 TF 树核对（`--self-test` 无 ROS 可跑）
- `workspace_sim/src/uv_sim/config/extrinsics/xunyun.yaml`：机器可读外参表（生成物，禁止手改），`setup.py` 已安装
- `workspace_sim/src/uv_sim/test/test_frames_geometry.py`（19 例）：数学 + 契约（场景↔YAML↔文档三方一致）
- `check_dvl_frame.py`：杠杆臂改为读外参 YAML（消除第二份硬编码），航向变量 `alpha`→`psi`（N3）

**脚本位置修正**：原计划写 `workspace_auv/scripts/`，实际落在 `workspace_sim/tools/`——
`derive_extrinsics.py` 要读 `workspace_sim` 里的 `.scn`，放 `workspace_auv` 会造成跨工作区反向依赖；
`workspace_auv/scripts/` 目录此前并不存在。

**两项静态校验（写进文档，作为 T6 的前置关卡）**
1. 艇体静止 30 s：SLAM 输出的相机系速度 ≈ 0（判据 `|v| < 3σ_dvl = 0.0045 m/s`）
2. 给定已知方向运动：SLAM 首帧位移方向与 `/auv/odometry` 真值夹角 < 5°，尺度比 ∈ [0.95, 1.05]

**阻塞项**：N1（相机 offset z 差 0.2 m）需要实机测量才能定权威值；确认后改一处 + 重跑
`derive_extrinsics.py --out` + `pytest` 即闭环。

**验收**：`check_dvl_geometry.py` PASS、`derive_extrinsics.py --check` 退出 0、44 例单测全绿。

> **T1 端到端实测（2026-09-16）** —— **DVL 帧正确性成立（A1-2/A1-3/A1-4 通过）**
>
> 桌面 GPU 会话（`sim_bringup scenario_desc:=canyon_vision_dvl.scn`）：
>
> | block | samples | dvl err | stv err | along ratio |
> |---|---|---|---|---|
> | psi=0° | 153 | 0.0026 | 0.0376 | **0.998** |
> | psi=22.5° | 110 | 0.0027 | 0.0417 | **0.999** |
> | psi=45° | 91 | 0.0025 | 0.0327 | **0.997** |
> | psi=90° | 115 | 0.0026 | 0.0352 | **0.998** |
>
> - `dvl err` 全程 **2.5~2.7 mm/s**，`along ratio` 四段 **0.997~0.999**（修复前 22.5° 那档约为 0，
>   这正是本任务的判别点）⇒ **`/auv/dvl` 确为机体系速度**，杆臂项 `ω×r_dvl` 也对
> - 整体仍 `exit 1` 只因 `stv err`（0.033~0.042）：那是 **B19** —— `/zit6/state/vel` 被
>   `_odom_cb`（原点 twist）与 `_dvl_cb`（DVL 处速度，含 `0.375·ω_z` 杆臂）双写；
>   0.375×|ω_z|≈0.375×0.09~0.11 = 0.034~0.041 与实测逐段吻合。**归 T5，不放宽阈值**
> - 同轮修掉该脚本的两个工具 bug：① 时间戳取在 `send_motion()` 之后（block 永远采不到动样本）；
>   ② `along ratio` 投影到未做杆臂修正的 `linear`（转得越快越偏，曾报 1.117）。两者都已被
>   `--static-seconds 15 --timeout 40` 的复跑验证（上表）
> - ⚠️ **静态段的坑**：`start()` 之后艇没有 setpoint 时会因微正浮力上浮，靠近水面时
>   `median |v|` 会超限（实测一次 0.0856 > 0.05）。复现前先给一个定深指令
>   （如 `SET(0,0,6,0)` 等它稳定），再跑静态段，即可稳定通过（沙箱里 0.0021、GPU 会话里 0.0022）
> - **A1-5 仍部分**：`canyon_vision_dvl.scn` 与去相机变体已验证；`sauvc_2026_finals` /
>   `underwater_xunyun` / `wuurc_murc_2026_auv` 三个场景尚未逐个启动确认
>
**依赖**：T1（外参表要反映新安装）
**预估**：0.5~1 人日

---

### T3 ☑ 文档体系（接口即契约）— **T3a + T3b 均已交付（2026-09-16 / 2026-10-10）**

> **拆分（2026-09-16）**：T3 的接口契约里，一部分不随 T4/T5 变化、现在就能冻结并机检，
> 另一部分必须等 launch 与 `sim_bridge` 结构定下来。故拆成：
>
> - **T3a**：`node_parameters.md` 逐条核对（✅）、`docs/README.md` 索引、
>   `tools/check_docs_contract.py`（**源码级**对拍，无需跑栈，绕开 B13 的 GPU 限制）、
>   `docs/architecture/` 三篇、话题表去重
> - **T3b**：话题频率/QoS、实时快照入库、服务/动作字段语义（✅ 2026-10-10）；
>   ~~launch 入口表~~ 与 ~~`node_parameters.md` 按新结构改写~~ 已随 **T4** 一起交付
>
> **T3b 交付（2026-10-10）**：
>
> | 交付物 | 状态 |
> |---|---|
> | `scripts/dump_interfaces.sh` + `interfaces/snapshots/2026-10-10_sil-console/` | ✅ 快照带日期+模式（sil-console / sil-gpu / hil 端点集不同）；sil-gpu / HIL 快照仍欠 |
> | `docs/interfaces/topic_rates.md`（频率 + QoS，含测量陷阱与口径） | ✅ （没塞进契约表格：那张表已被 checker 按列解析） |
> | `docs/interfaces/service_action_contract.md`（7 服务 + 1 动作字段语义、失败条件、引擎自带服务、死定义登记） | ✅ |
> | `test_scene_publisher_types.py`（9 例：场景声明发布者的类型对 C++ 证据） | ✅ 补上了 checker 的结构性盲区 |
>
> **T3b 顺带查出的问题**（详见 `devlog/2026-10-10_status.md`）：
> ① `/auv/dvl_altitude` 与 `/auv/thrusters_state` 两条契约行的**消息类型写错**了
> （分别是 `sensor_msgs/Range`、`stonefish_ros2/ThrusterState`）——场景声明发布者的类型
> 一直是"自由声明"，现在被证据钉住；
> ② **B20（新登记）**：`/basic_motion/pose_info` 设计 30 Hz，**完整栈里实测 ≈15.6 Hz**；
> ③ `axes` 字段**会真正生效**（文档此前写"仅用于日志"是错的）；
> ④ `/tf` 其实**有发布者**（0 消息）、`/tf_static` **不存在**；`GetState`/`MotionCommand`/
> `DeviceCommand`/`PidGains`/`PidState` 是**无人使用的死定义**（已登记）。
>
> **T3a 交付（2026-09-16）**：
>
> | 交付物 | 状态 |
> |---|---|
> | `docs/node_parameters.md` 逐条核对（3 处错误 + 1 处遗漏 + 状态标记 + 待补清单） | ✅ |
> | `docs/interfaces/topic_contract.md`（**话题唯一事实源**：SIL/HIL/服务动作/栈外 GUI/缺口登记） | ✅ |
> | `docs/architecture/{system_architecture,sim_real_boundary,node_dependency}.md` | ✅ |
> | `docs/README.md`（索引 + 维护规则 + 标记含义 + 跨文档待补清单） | ✅ |
> | `scripts/check_docs_contract.py`（**源码级**四类对拍，无需跑栈） | ✅ |
> | `workspace_sim/src/uv_sim/test/test_docs_contract.py`（12 例，含 6 类故意漂移的反向验证） | ✅ |
> | 去重：`hil_architecture.md` §2.2/2.3 与 `node_parameters.md` 话题节 → 链接 | ✅ |
>
> **脚本位置修正**：原计划写 `tools/`，实际落在仓库根 `scripts/`（该目录已存在，`deploy.sh` 同处）。
> **SIL 优先**：用户 2026-09-16 决定当前只做软件在环，HIL/实机内容一律标 ⏸ 保留但不验收。
>
> 历史记录（T3a 第一项）：`docs/node_parameters.md` —— 逐条对照源码核出 3 处错误
> （`save_dataset` 默认值、`real_bringup` 的 `enable_nav`、navigator 位姿源描述过时）、
> 补记遗漏的 `/perception/line/*`，重写话题参考为「节点 → 话题」（含 SIL/HIL 两套），
> 并新增「完整性状态 + 待补清单」与「包级 launch 现状（B2 具体化）」两节。
> 尚未做：README 索引与 `check_docs_contract.py`。

**目标**：在**仓库根 `docs/`**（既有权威文档所在处）建立唯一文档根，并让「文档 ↔ 代码」的漂移可被自动发现。

**T3-0 既有文档归位与去重（先做，避免两套文档并存）**
- `docs/node_parameters.md`、`docs/hil_architecture.md` **留在原位**（不迁动，零引用改动）
- 去重：`hil_architecture.md §2.2/2.3` 的话题表与 `node_parameters.md` 的话题参考**收敛为** `docs/interfaces/topic_contract.md`（单一事实源），原处改为链接
- `docs/node_parameters.md` 中已过时的部分（`sim_bridge` 节点参数、`sim_core.py`、旧 launch 参数）在 T4/T5 落地后按新结构改写
- archify 产物 `docs/auv-architecture.{html,json,png}` 留在根 `docs/`（生成物，不迁）

**目录**（仓库根 `docs/`；`workspace_auv/docs/` 下的包级文档 `basic_motion_action.md`、`coordinate-design.md`、`debug_guide.md`、`perception.md` 保持原位并互相链接）
```
docs/
├── README.md                       # 索引 + 维护规则（改接口必须同 PR 改文档）
├── roadmap.md                      # 本文档
├── node_parameters.md              # 既有（T4 已按新结构改写；T5 后需再改 sim_bridge 段）
├── hil_architecture.md             # 既有（话题表收敛进 topic_contract.md）
├── auv-architecture.{html,json,png}# 既有（archify 生成物）
├── architecture/
│   ├── system_architecture.md      # 分层、节点清单、数据流、两种入口、节点依赖图
│   ├── sim_real_boundary.md        # SIL/HIL/REAL 三模式矩阵 + MCU 边界 + /zit6/* 所有权
│   └── node_dependency.md          # 包依赖、启动顺序、就绪判据、失效影响面
├── interfaces/
│   ├── topic_contract.md           # 话题 | 类型 | 发布者 | 订阅者 | 频率 | frame_id | 单位 | QoS | 备注
│   ├── service_action_contract.md  # 服务与动作（含 basic_motion 动作完整语义表）
│   ├── launch_entries.md           # 入口清单（名/包/参数/启动内容/适用模式）+ 新旧入口映射
│   └── snapshots/                  # ros2 实时快照（校验基线，入库）
└── frames/
    ├── frame_convention.md         # T2
    └── extrinsics_reference.md     # T2
```

**必备内容要点**
- `topic_contract.md` 覆盖全部命名空间：`/auv/*`、`/sim/*`、`/zit6/*`、`/basic_motion/*`、`/perception/*`、`/nav/*`、`/task/*`，并显式列出**已知缺口**（B5 的 `/perception/annotated/*`：有订阅无发布）
- `service_action_contract.md`：`/zit6/{get_params,update_params}`、`/nav/navigate_to`、`/task/{run,stop,exec}`，动作 `basic_motion`（唯一动作入口，含 START 前置约束）
- `launch_entries.md`：含**新旧入口映射表**（real 保留；sim 迁移到仿真侧；hil 归仿真侧；`sim_core` 参数化取代）

**工具**（放仓库根 `tools/`，与 `docs/` 同级）
- `scripts/dump_interfaces.sh`（T3b）：源码抽取 + 实时快照（`ros2 topic/service/action list -t`、`node list`、`node info`）
- `scripts/check_docs_contract.py`（✅ T3a 已交付）：**源码级**四类对拍（参数/节点/话题/依赖）＋README 覆盖检查，漂移即非零退出；`--update` 生成 `docs/interfaces/snapshots/code_inventory.json`

**验收**：A3-1 目录中列出的 **README + 8 篇契约文档**齐备且含规定清单；A3-2 契约校验通过；A3-3 故意改错一个类型必须失败（自测）；A3-4 话题表无重复事实源（`hil_architecture.md`/`node_parameters.md` 改为链接）。
**依赖**：T1、T4、T5 完成后的接口才稳定（可先写仿真基线快照，再回填；T3-0 可立即做）
**预估**：2~2.5 人日（含 T3-0 约 0.5）

---

### T4 ☑ bringup 彻底分离（实机 / 仿真）— **已完成（2026-09-16）**

> **T4 已完成**。开工前用户确认了两点：新包名用 `uv_sim_bringup`（放 `workspace_sim/src/`）；
> 除 A4 四条静态验收外，额外在 console 模式做 L1（起栈 + 话题对拍）与 L2（控制闭环基线）冒烟。
>
> | 交付物 | 状态 |
> |---|---|
> | 新包 `workspace_sim/src/uv_sim_bringup`（`sim_bringup.py` + `hil_bringup.py` + `stonefish_common.py`） | ✅ |
> | `workspace_auv` 只留 `real_bringup.py`；删除 `sim_bringup/sim_core/hil_bringup` | ✅ |
> | `uv_bringup/uv_bringup/launch_common.py`（`arg()` / `include(when=…)`） | ✅ |
> | 包级 launch 修复：`hm_launch.py`（B2）、`control_launch.py`（B2）、`perception_launch.py`（+`sim_mode`）、新增 `uv_sim/launch/bridge_launch.py` | ✅ |
> | `docs/interfaces/launch_entries.md`（入口清单 + 新旧映射 + 编排规则） | ✅ |
> | 校验扩展：`launch-targets` + `launch-refs` 两类新检查（共 8 类）；修掉依赖检查的 3 个 bug | ✅ |
> | `test_launch_contract.py`（10 例，含 5 类故意漂移的反向验证） | ✅ |
> | B6 文档扫尾：14 个文件里的旧入口引用 + `node_parameters.md` launch 章节改写 + D1 结项 | ✅ |
>
> **与本节原计划的 3 处偏差**（均有理由，见 `launch_entries.md` §3）：
> 1. `enable_*` 门控**留在预设**（`include` 的 `condition`），包级 launch 不重复声明同名开关
>    —— 起不起只能有一个作者；
> 2. micro-ROS agent 归 `uv_hm/hm_launch.py`，HIL 也 include 它 ⇒ **HIL 会起 `hw_manager`**
>    （真 MCU 需要 `/zit6/cmd/agxhbt` 心跳才会解锁）。⏸ 只做静态核查；
> 3. `sim_core` 的两个**声明了却不生效**的参数（`scenario`/`gpu`）没有保留：`scenario` 统一成
>    `scenario_desc` 并真正生效，`gpu` 由语义正确的 `headless` 取代（选 `stonefish_simulator_nogpu`）。
>
> **A4-2 判据细化**：原写"`grep -r "uv_sim\|stonefish" workspace_auv/src` 无命中（除文档性说明）"，
> 但 `uv_sim_bringup` 这个名字本身含 `uv_sim`，注释里提它属于正常说明。改为**代码级**判据（可机检）：
>
> ```bash
> grep -rnE "package=['\"](uv_sim|stonefish_ros2|uv_sim_bringup)['\"]|import (uv_sim|stonefish)|get_package_share_directory\(['\"](uv_sim|stonefish_ros2)" workspace_auv/src
> # 期望：0 命中（等价断言在 test_launch_contract.py，用 AST 判 import / package= / include()）
> ```
>
> **附带发现（顺手修掉）**：`workspace_auv/docs/debug_guide.md` 仍在讲 yaw 极性 bug「已知」，
> 而代码 2026-09-15 已修（且该 bug 说明带过时行号）；`uv_nav`/`uv_task` 用了 `std_srvs` 却没声明。

**目标**：`workspace_auv` 只留实机入口且不含任何仿真依赖；仿真入口归 `workspace_sim`。原则：**节点定义归各包，系统预设只做编排**（借 robotcup 架构）。

| 工作区 | 包 | 入口 | 定位 |
|---|---|---|---|
| workspace_auv | `uv_bringup` | `real_bringup.py` | 唯一实机系统入口（sim-free） |
| workspace_sim | `uv_sim_bringup`（新建） | `sim_bringup.py` | 仿真系统入口（SIL） |
| workspace_sim | `uv_sim_bringup` | `hil_bringup.py` | HIL（Stonefish + 真 MCU）⏸ |

**改动点**
- 修 B2：`uv_hm/launch/hm_launch.py`（删失效 `sim_bridge`，改为 `hw_manager` + micro-ROS agent `ExecuteProcess`，参数 `agent_bin/serial_dev/serial_baud`，默认值取自原 `hil_bringup.py`）；`uv_control/launch/control_launch.py`（删 `minimal_control`）
- 参数化 `uv_perception/launch/perception_launch.py`（`sim_mode`）；`uv_nav`/`uv_task` 的 launch 已正确，不动
- 新建共享助手 `uv_bringup/uv_bringup/launch_common.py`（参数声明 / 包含助手）
- 删除 `workspace_auv/src/uv_bringup/launch/{sim_bringup.py,sim_core.py,hil_bringup.py}`；`sim_core` 的能力由 `enable_motion:=false enable_ai:=false` 取代
- `uv_bringup/setup.py` 与 `uv_sim_bringup/setup.py` 的 `data_files` 是**显式列表**，必须同步增删
- 同步更新 B6 清单中的记忆/agent 文档

**验收**
- A4-1 干净 shell 仅 source `/opt/ros/jazzy` + `workspace_auv/install`（**不** source workspace_sim）时 `ros2 launch uv_bringup real_bringup.py --show-args` 成功
- A4-2 `workspace_auv/src` 内**无代码级**仿真引用（见上面的 grep，0 命中；散文提及不算）
- A4-3 `test_launch_contract.py` + `check_docs_contract.py --only launch-targets`：所有入口引用的 executable 必须在对应 `setup.py`/`CMakeLists.txt` 存在（防 B2 复发）
- A4-4 HIL 参数表静态核查通过（运行时验证待实机）

**依赖**：T3 的 `launch_entries.md` 草稿
**预估**：1.5~2 人日

---

### T5 ✅ sim_bridge 按传感器拆分（MCU 视为一种「传感器」）— **已完成（2026-09-16）**

> **交付实况**（与下面的计划有三处差异，都以代码为准）：① `hil_mode` 从**节点参数**变成
> `bridge_launch.py` 的**组合开关**（SIL 7 节点 / HIL 5 节点），节点级参数变成
> `nav_sim.publish_state/publish_sim_nav` 与 `mcu_sim.hil_mode`；
> ② 没有引入 `coordinate_convention.py` 也没有复用 `thrust_mixer.py`/`pid_controller.py`
> （两者零 import 且数学已漂移：死代码 yaw ×0.5/heave ×1.0 vs 活代码 ×1.0/×0.8），
> 改为把活数学搬进 `mixer.py`/`mcu_control.py` 并用黄金夹具钉住；
> ③ `velocity_valid` 用 `altitude >= 0` 而不是 `velocity_covariance[0] >= 0`
> （后者在本引擎里恒为正，等于"永远有效"）。
> 另有一个没写在计划里的坑：`uv_sim` 缺 `setup.cfg`，所以 7 个 `console_scripts`
> 全被装到了 `install/uv_sim/bin/`（`ros2 run`/launch 看不到）——拆分前那个手写的
> `libexec` 包装脚本一直在替一个坏掉的入口干活。详见 devlog §9。

**目标**：把 765 行单体拆成 7 个单职责节点，**话题名/类型零变化**（只新增命名空间）。

| 节点 | 职责 | 输入 → 输出 |
|---|---|---|
| `odom_sim` | 位姿真值转发 | `/auv/odometry` → `/sim/sensor/odom` |
| `imu_sim` | 角速度/姿态转发 | `/auv/imu` → `/sim/sensor/imu` |
| `dvl_sim` | DVL 适配（**兼 AQUA-SLAM 适配器**）：速度透传、`velocity_valid` 映射、按 α/β 合成四波束 | `/auv/dvl` → `/sim/sensor/dvl`（`uv_msgs/Dvl`） |
| `pressure_sim` | 深度转发 | `/auv/pressure` → `/sim/sensor/pressure` |
| `nav_sim` | INS 仿真：聚合为 6 元素 `pos/vel` + 12 浮点 `sim/nav` | `/sim/sensor/*` → `/zit6/state/{pos,vel}`、`/zit6/sim/nav` |
| `mcu_sim` | **MCU 仿真**：setpoint→PID(pos/vel/force)→混控→推力；心跳/ARM/INS 状态机；参数服务 | `/zit6/cmd/*` → `/auv/thrusters_cmd`、`/zit6/state/{status,thr,zithbt}` |
| `camera_relay` | 相机转发/拼接 | `/sim/{front,down}_cam/*/image_color` → `/auv/{front,down}_cam/{left,right,stitched}` |

**`/zit6/state/*` 单一发布者规则**（写入 `sim_real_boundary.md`）
- SIL：`nav_sim` 拥有 `pos/vel`；`mcu_sim` 拥有 `status/thr/zithbt`
- HIL/REAL：物理 MCU 拥有全部 `/zit6/state/*`；`nav_sim` 以 `publish_state:=false` 运行，仅提供 `/zit6/sim/nav`

**B19 是本任务的硬需求之一**（2026-09-16 定性）：`/zit6/state/vel` 现在被 `_odom_cb`（原点 twist）与
`_dvl_cb`（DVL 处速度，含 `ω×r_dvl` 杆臂）**同时写**，两者在有角速度时相差 `0.375·|ω_z|`（最大 0.13 m/s），
这就是 T1 的 `check_dvl_frame.py` 里 `stv err` 一直 FAIL 的原因。拆分时必须：
- `dvl_sim` 发布**DVL 位置处**的速度（含杆臂）与其波束/有效性；
- `nav_sim` 发布**原点处**的 `pos/vel`（单一写者，不再被 DVL 回调覆盖）；
- 两者语义写进 `topic_contract.md`，并让 `check_dvl_frame.py` 分别对拍（`error_dvl` 对 DVL 话题、
  `error_state` 对原点话题，各自用各自参考量）。

**共享模块**：`mixer.py`（混控，活数学）、`mcu_control.py`（ROS-free 控制核）、`pid.py`/`pid_config.py`、
`odometry.py`（唯一的 `/auv/odometry` 解码器）、`mathutil.py`、`runner.py`。
`thrust_mixer.py`/`pid_controller.py` **已删除**（零 import、数学漂移，见上面的交付实况）；
`coordinate_convention.py` **不引入**（`geometry.py` 已是几何唯一权威，本场景 NED 对齐 ⇒ 恒等映射）。

**迁移映射**：`_odom_cb/_imu_cb/_dvl_cb/_pressure_cb`→四个 `*_sim`；`_publish_{pos,vel,state}`→`nav_sim`；`_setpoint_cb/_tick_60hz/_*_pid_step/_publish_thrust_from_4dof/_agxhbt_cb/_publish_zithbt/_ins_cb/_update_ins_alignment/_get_params_cb/_update_params_cb/_publish_thr`→`mcu_sim`；`_init_camera_*`/`*_img_cb`/`_publish_stitched_*`→`camera_relay`

**验收**
- ✅ A5-1 `ros2 pkg executables uv_sim` 列出 7 个节点，无 `sim_bridge`（实跑确认；另见 `test_node_split.py`）
- ✅ A5-2 拆分前后**话题名与消息类型零变化**（`test/data/topics_before_split.json` 的 AST 三元组对拍：无删除、无类型变化，新增恰好是 `/sim/sensor/*` 的 6 个端点）
- ⏳ A5-3 BasicMotion `START` + `SET(0.5,0,3.0,0)` 在峡谷场景端到端成功（console 运行记录见 devlog §9.4）
- ✅ A5-4 同 setpoint 下推力序列与拆分前一致（`test/data/thrust_golden.json`：133 个 PID 步 + 3 例力模式 + 6 例混控探针，**逐位相等**）
- ✅ A5-5 `test_camera_relay.py`：stitched 宽度 = 2× 单目宽

**依赖**：T4（新入口承载新节点）
**预估**：2~2.5 人日
**风险**：拆分引入 PID/混控漂移 → 靠 A5-4 数值回归 + A5-3 端到端动作兜底

---

### T6 ☐ AQUA-SLAM 移植（workspace_auv 原生 ROS 2 包）

**目标**：`AQUA-SLAM/`（ORB-SLAM3 + DVL/IMU 紧耦合分支，~5.8 万行 C++）移植为 `workspace_auv/src/aqua_slam`（ament_cmake），在 T1 对齐后的仿真中端到端跑通并用真值量化验收。

**要点**（详见此前的可行性评估结论）
- 算法核心零改动：新增日志兼容头覆盖 ~80% 的 ROS1 调用点（`ROS_*_STREAM` → `RCLCPP_*`）
- 真正迁移：`RosHandling.{h,cpp}`（发布/服务/tf2）、`System.{h,cc}`（`ros::param` → 构造参数）、`Tracking.cc`（`ros::Time` ×8）、`LoopClosing`/`DenseMapper`/`LKTracker`（image_transport）、新节点 `aqua_slam_node.cpp`
- 删除：Pangolin/Viewer（上游已禁用，`Viewer.cc`/`MapDrawer.cc` 未参与编译）、自带 cv_bridge（改用系统 OpenCV4 版）
- OpenCV3→4：`CV_LOAD_IMAGE_UNCHANGED`×5、遗留色转换常量（加 `<opencv2/imgproc/types_c.h>`）
- 新增消息 `uv_msgs/msg/Dvl.msg`（`header/velocity/velocity_valid/altitude/beam_velocity[4]`），仿真由 `dvl_sim` 生产、实机由 MCU 桥接生产
- 配置：`config/sim_canyon.yaml`（内参由 hfov 推出 fx≈1174、`Tlr=(0.10,0,0)`、`Tbc/T_gyro_c/T_dvl_c` 来自 T2、**α=60°、β=45+90i**、IMU 200 Hz、初始化阈值按 30 m 峡谷缩小）
- 验收：S1 构建通过；S2 10 min 无崩溃；S3 与 `/auv/odometry` SE3 对齐后 **ATE RMSE ≤ 0.15 m**、终点漂移 ≤ 2%；S4 关闭 DVL 辅助精度显著劣化；S5 一条命令完成评估（`evo`）
- ⚠️ **帧率背景（2026-09-16 实测）**：SIL 相机实测下视 ≈9.5 Hz/眼（标称 30 Hz，4 路相机时降到 ≈1.8 Hz/眼），瓶颈是引擎的渲染+回读且**没有**多渲染器可调项；实机是 30 Hz。S3/S4 的判读必须带这个背景，需要高帧率时用 SIL 专用性能变体（降分辨率/只留一对相机），主场景不动

**依赖**：T1（传感器几何）、T2（外参/α/β）、T5（`dvl_sim` 与 `/sim/sensor/*`）、B8 磁盘腾挪（≥3 GB）
**预估**：8~10 人日（含仿真验证）

---

### T7 ☐ 实机就绪（接口预留，待硬件确认）

**待确认硬件三问**
1. DVL 型号与**原始逐波束速度**是否可获取（当前数据面只有融合结果）
2. MCU 能否输出 **200 Hz 原始 IMU**（当前只有融合后 `/zit6/state/pos|vel`）
3. 双目标定计划（`vision.py` 标定当前故意留空）

**工作内容**：MCU→`uv_msgs/Dvl` 桥接节点；SLAM 位姿源以独立话题输出、由上层仲裁（不改 `basic_motion` 内部）；NED ↔ SLAM 世界系对齐复用 `uv_control/coordinate.py`；SLAM 失效时回退现有视觉定位；降级策略（无 DVL → `bDVL=false`；无原始 IMU → 降频并重标噪声）。

**依赖**：T6（仿真验收通过后再投入）
**预估**：0.5 人日（文档与接口）+ 实机联调另计

---

## 4. 依赖与推荐顺序

```
T1 传感器对齐 ──┬─→ T2 坐标系/外参 ──┐
                │                     ├─→ T6 AQUA-SLAM 移植 ─→ T7 实机就绪
                └─→ T4 bringup 分离 ─→ T5 sim_bridge 拆分 ─┘
                                                     │
                            T3 文档体系 ◀────────────┘（先出基线快照，后回填）
```

- **可并行**：T1 与 T4 互不阻塞；T3 的仿真基线快照可在 T1 前先抓
- **硬前置**：T6 需要 T1+T2+T5（磁盘已就绪：`/` 剩余 12 GB）
- **推荐节奏**：T1 → T2 → T4 → T5 → T3 回填 → T6 → T7

---

## 5. 全局不变式（任何任务都不得违反）

| # | 不变式 | 校验方式 |
|---|---|---|
| I1 | 拆分/重构**不改变**任何现有话题名与消息类型（只允许新增） | T3 快照 diff |
| I2 | 任一 `/zit6/state/*` 话题**单一发布者** | 契约表 + 运行时检查 |
| I3 | 每平台**独立外参 YAML**；sim/real 不共用 | T2 文档硬规则 |
| I4 | DVL 的 α/β 按**硬件真值**填写；不以仿真跑通作为正确性证据（D4） | T2 校验脚本 |
| I5 | 角度单位分界：ROS 消息用弧度；YAML/参数/PoseInfo 用度 | `frame_convention.md` |
| I6 | `workspace_auv` 不含仿真依赖（可独立 build/launch） | A4-1/A4-2 |
| I7 | 文档与代码同 PR 更新 | T3 契约校验 |

---

## 6. 验收总表（汇总）

| 编号 | 标准 | 归属 |
|---|---|---|
| A1-1..5 | 仿真传感器几何/频率/速度方向正确（含航向扫描回归），4 个场景可加载 | T1（V1 ✅；**V2/V3 ✅ 2026-09-16 在 GPU 会话与 console 两种模式下执行**：`dvl err` 2.5~2.7 mm/s、四段 `along ratio` 0.997~0.999 ⇒ A1-2/A1-3/A1-4 通过；`stv err` 0.033~0.042 是 B19 双写者 → 归 T5。**A1-5 部分**：2/4 场景已验证） |
| A2-1 | `derive_extrinsics.py --check` 退出 0（文档块 ↔ 场景一致） | T2（已绿） |
| A2-2 | `check_dvl_geometry.py` PASS：反演误差 < 1e-12、α=67.5° 复现 +30.7% | T2（已绿） |
| A2-3 | 48 例单测全绿（含场景↔YAML↔文档三方契约、N1 相机 offset、TF 契约） | T2（已绿） |
| A2-4 | `dump_tf_tree.py --self-test` 绿；在线核对在 console 模式可跑（无需 GPU） | T2（已绿） |
| A2-5 | 文档不再断言存在 TF；开启方式与代码守卫已写明 | T2（已绿） |
| A3-1 | README + 契约文档齐备（`frames`×2、`architecture`×3、`topic_contract`、`node_parameters`） | T3a（已绿） |
| A3-2 | `scripts/check_docs_contract.py --check` 退出 0（参数/节点/话题/依赖/README 五类） | T3a（已绿） |
| A3-3 | 故意改错必须失败——6 类漂移 + 端到端注入各验证一次 | T3a（已绿） |
| A3-4 | 话题表无重复事实源（`hil_architecture.md`/`node_parameters.md` 已改为链接） | T3a（已绿） |
| A3-5 | 服务/动作字段语义 ✅、~~launch 入口表~~（T4 已交付）、频率/QoS 列 ✅（sil-console 一份快照；sil-gpu/HIL 欠） | T3b（✅ 2026-10-10） |
| A4-1..4 | 分层干净；入口契约测试通过 | T4（✅ 已绿，见 T4 节） |
| A5-1..5 | 7 节点可执行 ✅；话题零变化 ✅；数值回归 ✅；端到端动作 ⏳（console 记录见 devlog §9.4） | T5（已交付） |
| S1..S5 | AQUA-SLAM 构建/稳定性/精度/DVL 消融/一键评估 | T6 |

---

## 7. 已决事项与开放问题

**已决（2026-09-15 确认）**

| # | 结论 |
|---|---|
| ~~O1~~ | 实机 DVL 波束倾角 = **30°（离竖直）** ⇒ `beam_angle=30`、AQUA-SLAM `α=60°`（不再是条件项） |
| ~~O2~~ | 「45°」= **DVL 斜置**（波束相对艇体系 45° 斜向）⇒ 按 D1 改仿真：`rpy=(0,0,0)` + `beam_positive_z="true"` + `xyz z=0.1` |
| ~~O3~~ | 文档树放**仓库根 `docs/`**（与既有权威文档同处），工具放仓库根 `tools/`；`workspace_auv/docs/` 只保留包级文档 |
| ~~O4~~ | `coordinate_convention.py` **不引入**（T5 定案）：`geometry.py` 已是几何唯一权威，本场景 NED 对齐 ⇒ 恒等映射，再加一层只会多一个漂移点 |
| ~~O6~~ | 磁盘已腾出：`/` 剩余 **12 GB**，满足 T6 要求 |
| ~~O7~~ | 已随 O3 消解：archify 产物本就在根 `docs/`，不迁动 |

**仍开放**

| # | 问题 | 影响 | 默认取向 |
|---|---|---|---|
| O5 | 是否需要「就绪门控」（等待 Stonefish 话题出现再启动下游） | T4 可选增强 | 做成 `startup_gate` 参数，默认关闭 |

---

## 附录 A：T1 的精确改动（`xunyun_fixed.scn` DVL 块）

**现状**（`workspace_sim/src/stonefish_ros2/Data/xunyun_fixed.scn:277-291`；为便于对照，下面省略了部分行内注释，实施时以文件实际内容为准）：
```xml
<!-- DVL 多普勒计程仪，用于发送声波测量相对水底的速度 -->
<sensor name="dvl" type="dvl" rate="20.0">
    <history samples="1000"/>
    <link name="Vehicle"/>
    <!-- rpy="3.1416 0.0 -0.785" 决定它斜向下照射的角度及偏航对齐 -->
    <origin rpy="3.1416 0 -0.785003" xyz="-0.375 0 0.2"/>
    <specs beam_angle="30.0"/> <!-- 波束俯仰角 -->
    <range velocity="9.0 9.0 9.0" altitude_min="0.5" altitude_max="81.0"/>
    <water_layer minimum_layer_size="5.0" boundary_near="10.0" boundary_far="30.0"/>
    <noise velocity="0.0015" altitude="0.001"/>
    <ros_publisher topic="/auv/dvl" altitude_topic="/auv/dvl_altitude"/>
</sensor>
```

**目标**：
```xml
<!-- DVL：四波束下视，离竖直 30°，方位 45/135/225/315（与实机一致） -->
<!-- 传感器系与机体系对齐（rpy=0），故 /auv/dvl 的速度即机体系速度；
     波束方向由 beam_positive_z 指定为传感器 +z（NED 下即向下） -->
<sensor name="dvl" type="dvl" rate="20.0">
    <history samples="1000"/>
    <link name="Vehicle"/>
    <origin rpy="0 0 0" xyz="-0.375 0 0.1"/>
    <specs beam_angle="30.0" beam_positive_z="true"/>
    <range velocity="9.0 9.0 9.0" altitude_min="0.5" altitude_max="81.0"/>
    <water_layer minimum_layer_size="5.0" boundary_near="10.0" boundary_far="30.0"/>
    <noise velocity="0.0015" altitude="0.001"/>
    <ros_publisher topic="/auv/dvl" altitude_topic="/auv/dvl_altitude"/>
</sensor>
```

**同文件其它改动**：IMU `rate="20.0"→"200.0"` 并加 `<noise .../>`；前视双目 `rate="10.0"→"20.0"`。

**参数已确认（不再是条件项）**：波束倾角 30°（离竖直）⇒ `beam_angle="30.0"`、AQUA-SLAM `α = 60°`（`1.0472 rad`）；波束方位 45/135/225/315（DVL 斜置）⇒ AQUA-SLAM `β = (45°,135°,225°,315°)`。

---

## 附录 B：影响面清单

| 受影响对象 | 影响 | 处理 |
|---|---|---|
| `xunyun_fixed.scn` | DVL/IMU/相机参数变更 | 直接改 |
| `canyon_vision_dvl.scn`、`sauvc_2026_finals.scn`、`underwater_xunyun.scn`、`wuurc_murc_2026_auv.scn` | 通过 include 继承改动 | 更新各自头部注释中的传感器说明 |
| `sim_bridge._dvl_cb` → `/zit6/state/vel` | 速度由「DVL 系（错位 45°）」变为「机体系（正确）」 | ✅ 行为修复；T5 起该话题只由 `nav_sim` 写，DVL 值走 `/sim/sensor/dvl` |
| `basic_motion._vel_cb` → `v_along` | 收敛判据输入变正确（不再被 DVL 的杆臂项间歇覆盖） | 观察 WMOVE 行为变化（可能表现为步进节奏变化，属预期修复） |
| `/auv/dvl` 其它消费者 | 实测原本仅 `sim_bridge`；T5 起新增 `dvl_sim`（转发 + 合成 4 波束 → `/sim/sensor/dvl`） | 契约在 `topic_contract.md` §1；T6 的 AQUA-SLAM 是它的消费者 |
| `workspace_sim/stonefish_notes.md` | 记录了旧的 DVL 说明 | 同步更新 |
| AQUA-SLAM `config/sim_canyon.yaml` | `T_dvl_c`、α/β 依赖新安装 | T2/T6 生成 |
