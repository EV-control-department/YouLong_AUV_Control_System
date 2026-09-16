# SIL / HIL / REAL 边界

> **当前阶段：只做软件在环（SIL）。** HIL 与实机（REAL）两列**保留在册但不纳入验收**
> （用户 2026-09-16 决定；HIL 细节见 `docs/hil_architecture.md`，其话题表已收敛到
> `docs/interfaces/topic_contract.md` §8）。
>
> 话题/类型见契约文件；参数见 `docs/node_parameters.md`。

## 1. 三模式总览

| 维度 | **SIL**（当前唯一验收路径） | HIL ⏸ | REAL ⏸ |
|---|---|---|---|
| 谁算 PID | `sim_bridge`（Python，`config/sim_pid.yaml`） | STM32 MCU 固件 | STM32 MCU 固件 |
| 谁做推力混控 | `sim_bridge` → `/auv/thrusters_cmd` | MCU → `sim_bridge` 转发 | MCU → 推进器硬件 |
| 谁发布 `/zit6/state/{pos,vel,thr,status,zithbt}` | `sim_bridge` | MCU（经 micro-ROS） | MCU |
| 传感器从哪来 | Stonefish（`/auv/*`） | Stonefish（MCU 远端订阅） | 真实 DVL/IMU/相机/压力计 |
| 图像从哪来 | Stonefish 渲染 → `sim_bridge` 拼接 | 同 SIL（相机直通） | 4 路 USB 摄像头（V4L2，MJPEG） |
| `vision` 的 `sim_mode` | `true`（订阅 `/auv/*_cam/stitched`） | `true` | `false`（V4L2 采集） |
| 位姿源 | `/basic_motion/pose_info` | 同 SIL | 同 SIL |
| 启动入口 | `uv_sim_bringup/sim_bringup.py`（节点来自 `uv_sim/bridge_launch.py` 等） | `uv_sim_bringup/hil_bringup.py` | `uv_bringup/real_bringup.py` |
| 依赖 GPU | 是（渲染）；纯物理用 `sim_bringup.py headless:=true simulation_rate:=100.0`（**无相机图像**） | 是（相机转发需要渲染） | 否 |
| 依赖真实硬件 | 否 | STM32 + 串口 | 全套 |

## 2. 所有权矩阵（谁拥有哪个话题）

**同一话题在同一模式下只能有一个发布者**（不变式 I2）。下表按模式列"发布者"：

| 话题 | SIL | HIL ⏸ | REAL ⏸ |
|---|---|---|---|
| `/zit6/cmd/setpoint` | `basic_motion` 发 → `sim_bridge` 收 | `basic_motion` 发 → MCU 收 | 同 HIL |
| `/zit6/state/pos` | `sim_bridge` | MCU | MCU |
| `/zit6/state/vel` | `sim_bridge` | MCU | MCU |
| `/zit6/state/thr` | `sim_bridge` | MCU | MCU |
| `/zit6/state/status`、`/zit6/state/zithbt` | `sim_bridge` | MCU | MCU |
| `/auv/thrusters_cmd` | `sim_bridge` | `sim_bridge`（转发 MCU 力） | — （直接驱动硬件） |
| `/auv/thrusters_state` | Stonefish（场景 `<ros_publisher thrusters=…>`） | Stonefish | — |
| `/auv/{odometry,imu,dvl,dvl_altitude,pressure}` | Stonefish | Stonefish | 真实传感器驱动 |
| `/zit6/cmd/agxhbt`（心跳） | 无发布者（`hw_manager` 不启动） | `hw_manager` | `hw_manager` |
| `/basic_motion/pose_info` | `basic_motion` | `basic_motion` | `basic_motion` |
| `/zit6/sim/nav` | 无发布者（仅 HIL 分支创建） | `sim_bridge` | — |
| `/tf`、`/tf_static` | **无任何发布者**（B18） | 同 SIL | 同 SIL |

> `sim_bridge` 的 HIL 专属发布/订阅（`/zit6/sim/nav`、`/zit6/state/thr` 订阅）在
> `hil_mode:=true` 时才创建，见契约文件 §8。

## 3. 为什么 SIL 能代表真实控制栈

SIL 与 REAL 共享**同一套上层**：`basic_motion` → `/zit6/cmd/setpoint` →（MCU 或 sim_bridge）
→ `/zit6/state/*` → `basic_motion`/`vision`/`position`/`navigator`/`task_runner`。
上层代码**看不到** PID 在哪算。因此：

- **上层的对错**（动作语义、坐标系、单位、状态机）在 SIL 里就能验证；
- **下层的对错**（PID 增益、混控矩阵、执行器极性）在 SIL 里只能验证 `sim_bridge` 自己的实现，
  不能代表 MCU 固件——固件另有一套参数化（`UserApp/Config/config.json`，与
  `chassis.pid.*` 词汇相同但数值不可互换）。

这就是为什么 SIL 阶段的结论要限定在"上层 + 仿真桥"范围内，写进文档时必须带这个限定。

## 4. 当前阶段的范围界定（SIL-only）

**纳入范围**（可验收）：

- `sim_bridge` 的 SIL 分支：PID、混控、状态发布、参数服务、相机拼接
- 全部上层节点：`basic_motion`、`vision`、`position`、`navigator`、`task_runner`
- 仿真侧：Stonefish 场景、传感器契约（`docs/frames/extrinsics_reference.md`）
- 接口契约、参数、坐标系

**不纳入范围**（保留在册，标 ⏸）：

- MCU 固件（`third_party/AUV_zit6_cmake`）与 micro-ROS 链路
- `hw_manager` 的心跳/看门狗行为（依赖真实 MCU）
- `hil_bringup.py`、`real_bringup.py` 的端到端验证（T4 后 HIL 的节点集合有变化：
  agent 与 `hw_manager` 现在来自 `uv_hm/hm_launch.py`，**只做静态核查**）
- 实机外参的**实测**复测（`docs/frames/extrinsics_reference.md` §2 的"未验证"行列）
- 串口带宽/降采样策略（`imu_sim` 相关，T5 再议）

**跨阶段约束（不要违反）**：

1. 不要把 SIL 的 PASS 当作 MCU 固件的 PASS；
2. 不要为了 SIL 方便而删掉 HIL 分支——`sim_bridge` 的 `hil_mode` 分支必须保持可导入、
   可实例化（有代码路径即算，不需要实机验证）；
3. 新增话题/参数时，HIL 侧若有对应物，在契约文件里一并登记（标 ⏸ 即可）。
