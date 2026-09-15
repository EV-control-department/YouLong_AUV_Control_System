# 坐标系与约定（权威）

本文件是**约定的唯一权威**。`docs/frames/extrinsics_reference.md` 放数值，本文件放规则。
每条约定都带代码锚点，锚点用「文件 + 符号名」而不是行号区间（行号会漂，符号名可 grep）。

引擎行为（Stonefish 怎么解析 `.scn`、怎么算浮力）见 `workspace_sim/stonefish_notes.md`，
本文件不复述，只固化**本仓库自己的约定**。

---

## 1. 四套坐标系

| 名字 | 含义 | 轴向 | 锚点 |
|---|---|---|---|
| `world_ned` | 世界系 / TF 根 | `x=北, y=东, z=下`；`z=0` 水面，正 z 更深 | `ROS2SimulationManager.cpp` 的 TF 广播（`"world_ned"`）；`stonefish_notes.md` §2 |
| `<robot>/base_link` | 机体系（艇体原点） | `x=艏, y=右舷, z=下`；`yaw=0` 时艏指北 | 同上；场景 `xunyun_fixed.scn` 的 `<base_link name="Vehicle">` |
| 传感器系 | 各 `<sensor>` 的 `<origin>` 定义的系 | 除相机外均与机体系对齐或近对齐 | `xunyun_fixed.scn` 各 `<origin>`；`extrinsics_reference.md` |
| 相机光学系 | OpenCV/ROS 光学约定 | `x=右, y=下, z=前`（光轴） | `position.py` 的 `CAM_PARAMS[*]['optical_to_body']` |

**NED 的证据**（别再翻）：`SimulationManager` 用 `setGravity(0, 0, +9.81)`，重力沿 +Z ⇒ Z 朝下；
`NED::__nRe__()` 的三行依次是 North/East/Down；`GPS.cpp` 把世界系 `x,y` 直接当 `north,east`。
`<environment><ned latitude longitude/>` **只是 NED 原点的大地坐标，不是坐标系切换**。

**注意 3D 软件导出的网格是 z-up**：在 NED 下是上下颠倒的，必须给正 z 偏移压到水下
（`canyon_vision_dvl.scn` 的 `world_transform z=+8.0`）。新写 `.scn` 切勿按 z-up 推深度。

---

## 2. 旋转与角度

### 2.1 rpy 复合顺序

Stonefish 把 `<origin rpy="r p y">` 解析为

```
R = Rz(yaw) · Ry(pitch) · Rx(roll)
```

锚点：`ScenarioParser.cpp` 的 `ParseTransform()` → `Transform(Quaternion(rpy.z, rpy.y, rpy.x), xyz)`；
`sf::Quaternion` 就是 `btQuaternion`，其 `(yaw, pitch, roll)` 构造走 `setEulerZYX`。

代码里**只允许**用 `uv_sim.geometry.euler_rpy_to_matrix()`，不要手写 `Rx@Ry@Rz`——
两者在 yaw、roll 同时非零时不等价（`test_frames_geometry.py::test_euler_order_is_rz_ry_rx` 钉住）。

### 2.2 单位对照表（最容易出错的地方）

| 位置 | 单位 | 备注 |
|---|---|---|
| `.scn` 的 `<origin rpy>` | **弧度** | 但 `<specs beam_angle>` 是**度**（见 §4） |
| `.scn` 的 `<origin xyz>` | 米 | |
| 控制栈内部（`Coordinate`、`basic_motion`） | **度** | 只在组装 ROS 消息时转弧度 |
| `PoseInfo.robot_roll/pitch/yaw`、`origin_yaw`、`target_yaw` | **度** | `basic_motion.py` 的 `_publish_pose_info()` |
| `/zit6/state/pos` | `[x, y, z, roll_rad, pitch_rad, yaw_rad]` | 位置米、姿态**弧度** |
| `/zit6/state/vel` | `[vx, vy, vz, vroll, vpitch, vyaw]` | 线速度 m/s、角速度 **rad/s** |
| `sensor_msgs/Imu` | rad, rad/s, m/s² | 标准消息 |
| `nav_msgs/Odometry` | 米、弧度 | 标准消息 |

**yaw 正方向 = 顺时针（NED 下从北转向东）**，所以 `yaw=+90°` 让艏从 +x（北）转到 +y（东）。

---

## 3. TF 树与 `frame_id` 现状

**结论先行：本仓库现在没有任何 TF。** `/tf` 话题存在但一条消息都没有，
`/tf_static` 同理。这不影响控制栈（没人订阅 TF），但意味着**外参只能靠文件/约定传递**。

```
（当前实际状态）
/tf         空
/tf_static  空
```

**为什么空**：TF 广播是**默认关闭**的，需要场景显式声明：

```
ROS2SimulationManager.cpp    if (rosRobots_[i]->publishBaseLinkTransform_)   ← 守卫
                                 PublishTF(..., "world_ned", robot->getName() + "/base_link");
ROS2ScenarioParser.cpp       if ((item = element->FirstChildElement("ros_base_link_transform")))
                                 item->QueryBoolAttribute("publish", &publishBaseLinkTransform_);
ROS2SimulationManager.h      publishBaseLinkTransform_(false)                 ← 默认 false
```

即需要在**车辆场景的 `<robot>` 元素里**写 `<ros_base_link_transform publish="true"/>`
（注意它在 `<robot>` 里，不在 `<sensor>` 里，所以 canyon 这类世界场景只能通过
`<arg>` 转发，不能自己加）。本仓库 15 个 `.scn` **一个都没写**（`grep 'publish=' Data/*.scn` 无命中）。

> ⚠️ 引用 `ROS2SimulationManager.cpp` 那行做"TF 存在"的证据是错的——它外面套着 `if`。
> 2026-09-15 的 T2 文档初稿就犯了这个错，是 `dump_tf_tree.py` 在线跑出 exit 1 才暴露的。
> 若将来开启 TF，帧名会是 **`<robot name=>/base_link`**（canyon 传的是 `XUNYUN`，
> `xunyun_fixed.scn` 的默认名才是 `Vehicle`），届时同步改 `dump_tf_tree.py` 与本文件。

| 事实 | 锚点 |
|---|---|
| **没有任何 TF（含机体位姿，也含传感器静态）** | `/tf` 实测为空；`grep 'publish=' Data/*.scn` 无命中 |
| `/auv/odometry`：`header.frame_id="world_ned"`、`child_frame_id=<odom 传感器名>` | `ROS2Interface.cpp` 的 `PublishOdometry` |
| 传感器消息的 `header.frame_id` 是**传感器自己的名字**（`dvl`、`imu_filter`、相机名），不是 `base_link` | `ROS2Interface.cpp` 各 `Publish*` 函数 |
| `/basic_motion/pose_info`（`PoseInfo`）**没有 frame 字段** | `uv_msgs/msg/PoseInfo.msg`；语义只靠本文件 |

**后果**：任何消费者都不能靠 TF 查外参；必须读
`config/extrinsics/xunyun.yaml`（仿真）或实机测量表。
`position.py` 就是自己带 `CAM_PARAMS` 外参的例子。
`line_follower.py`／`arrow_surfacer.py` 还各自带了一份下视相机 offset——N1 就是这么来的。

**里程计 twist 的系**：`pose` 在 `header.frame_id`（`world_ned`），`twist` 在
`child_frame_id`（机体系）。口诀：**pose 跟 header，twist 跟 child**。
重复旋转一次 yaw 会把 `v_body` 变成 `R(-psi)·v_body`（psi=90° 时纯前进变成 `(0,-u)`）。
唯一编码点是 `uv_sim/sim_bridge.py` 的 `body_twist_from_odometry()`
（回归：`test_odometry_frame.py`）。

---

## 4. DVL 波束几何：θ 与 α 是**两个不同的角**

| 记号 | 含义 | 值 | 出现在哪 |
|---|---|---|---|
| **θ** | 波束与**竖直**轴夹角 | **30°** | `.scn` 的 `<specs beam_angle="30.0">`（单位：**度**） |
| **α** | 波束离**水平面**的仰角 | **60° = 90° − θ** | AQUA-SLAM 的 `mE`（`DVLGroPreIntegration.cpp`） |
| **β** | 波束方位角 | **45 / 135 / 225 / 315°** | 引擎固定为 `45° + i·90°`（`sf::DVL`）；AQUA-SLAM 侧需逐波束填 4 个值 |
| **ψ** | 艇的航向（heading） | 任意 | 控制栈/测试脚本；**不要叫它 α** |

AQUA-SLAM 的波束矩阵行 = `(-cosβ·cosα, sinβ·cosα, sinα)`，速度反演
`v_body = (EᵀE)⁻¹Eᵀ·v_beam`。该反演**不要求波束正交或单位列**，只要求 `E` 列满秩。

数值（`python3 workspace_sim/tools/check_dvl_geometry.py` 可复现）：

- α=60°、β=45/135/225/315 → `EᵀE = diag(0.5, 0.5, 3.0)`，重建误差 `~1e-16`；
- 把 θ 当 α 用（α=30°）→ 水平速度 **−42.3%**、垂直 **+73.2%**；
- α=45° → 水平 **+41.4%**；
- **α=67.5°（AQUA-SLAM 出厂默认）→ 水平 +30.7%**、垂直 −6.3%。

结论：**α 填错是静默的水平尺度错误**，只有对着实机/真值比对才暴露。
`docs/frames/extrinsics_reference.md` §「T6 移植必改清单」列了必须覆盖的 SLAM 配置项。

---

## 5. odom 系（`basic_motion` 的局部原点）

- 发送 `START` 时把**当时的 map 位姿**记为 odom 原点：位置偏移 + yaw 偏移。
- 之后所有坐标相对该原点，`odom 0° = AUV 初始朝向`，单位为**度**。
- `PoseInfo` 同时携带 `origin_*`（odom 原点在 map 系的位姿）与 `robot_*`
  （当前位姿，含 roll/pitch/yaw，度）。
- 未发 `START` 前，运动类 goal 会被拒绝（`"odom origin not set, call start() first"`）。

锚点：`workspace_auv/src/uv_control/uv_control/basic_motion.py`。

---

## 6. 执行机构与归一化

| 话题 | 取值范围 | 含义 | 锚点 |
|---|---|---|---|
| `/auv/thrusters_cmd` | **[-1, 1]** | 6 个推进器归一化设定点 | `sim_bridge.py` 的推进器混控段 |
| `ZitSetpoint` | 视字段 | MCU 协议（力/力矩或轴指令） | `zit6_interfaces/msg/ZitSetpoint.msg` |
| `type_mask` | **0 = 轴生效，1 = 轴屏蔽** | 反逻辑！`0x0F` 是**全屏蔽** | `uv_control` 的 `MotionCommand.type_mask` |

`type_mask` 的反逻辑已多次造成误判，改动前先看 `CLAUDE.md` 的对照表。

---

## 7. 相机光学系

两套 `optical_to_body`（把光学系的方向映射到机体系，即 `R_c_b` = `T_b_c` 旋转块的转置）：

| 相机 | `R_c_b`（`position.py` 的 `optical_to_body`） | 含义 |
|---|---|---|
| 前视左右 | `[[0,0,1], [-1,0,0], [0,-1,0]]` | 光轴 +z → 机体 +x（前） |
| 下视左右 | `[[0,-1,0], [1,0,0], [0,0,1]]` | 光轴 +z → 机体 +z（下） |

（列向量含义：前视下 光学 x(右)→机体 +z、光学 y(下)→机体 -x、光学 z(前)→机体 +y；
两套矩阵都由 `<origin rpy>` 严格导出，`test_frames_geometry.py` 对 `R_b_c` 与 `R_c_b`
都做了断言。）

场景里对应的 `<origin rpy>`：前视 `1.5708 0 1.5708`、下视 `0 0 1.5708`（弧度）。
`R_b_c`（本文件 §3 的 `T_b_c` 旋转块）与上表互为转置，二者在
`docs/frames/extrinsics_reference.md` 中都有完整 4×4。

相机内参由 **HFOV 现算**（`position.py` 的 `CAM_PARAMS[*]['hfov']`），不是标定文件；
实机新相机标定后应改读真实 `K`（尤其是 `cx/cy`）。

---

## 8. 已知不一致与待确认（不要当成"已解决"）

| 编号 | 内容 | 状态 |
|---|---|---|
| **N1** | 相机 `offset` z 与场景 `<origin>` 曾差 0.2 m | **已决（2026-09-15）**：以代码为准（`position.py`/`line_follower.py`/`arrow_surfacer.py`/`docs/perception.md` 四处一致），场景已改齐为 `0.076`/`0.0645`；数值本身仍未实机复测 |
| **N2** | AQUA-SLAM 参考检出的 `alpha=1.1780 rad`(67.5°)、`beta=0.7854`(45°)、`Tbc/T_gyro_c/T_dvl_c` 全是 blue_gx5 的，与 XUNYUN 不符 | T6 移植必改（清单见 `extrinsics_reference.md`） |
| **N3** | 历史上 `alpha` 一词两义（航向 vs 波束仰角） | 已统一：**ψ=航向、θ=离竖直、α=离水平**；`check_dvl_frame.py` 的变量已改名 |

---

## 9. 怎么复核本文件

```bash
# 1) 波束几何（无 ROS）
python3 workspace_sim/tools/check_dvl_geometry.py

# 2) 外参表与场景对拍
python3 workspace_sim/tools/derive_extrinsics.py --check

# 3) 约定与表都被单测钉住
python3 -m pytest workspace_sim/src/uv_sim/test/ -q

# 4) 在线 TF：需要**另一个进程**正在跑模拟器（console 模式即可，不必 GPU）
#    当前仓库没开 TF，所以健康状态就是 exit 1 + "no TF received"；
#    一旦有人开了 <ros_base_link_transform>，这里应当变成 exit 0。
python3 workspace_sim/tools/dump_tf_tree.py --timeout 5
```
