# 外参参考表（权威数值）

本文件放**数值**，规则见 `docs/frames/frame_convention.md`。

- **仿真表**由场景文件**生成**，禁止手改：`python3 workspace_sim/tools/derive_extrinsics.py --out <file>`
- 下面 fenced YAML 块是机器可读版本，`derive_extrinsics.py --check` 与
  `test_frames_geometry.py::test_extrinsics_doc_matches_the_scene` 会把它与场景对拍，
  任何一处不一致都会报出具体元素。
- 变换记号：`T_<a>_<b>` 表示**把点从 b 系变到 a 系**。`b`=机体系（`Vehicle/base_link`），
  `c`=前视左目**光学系**，`gyro`=`imu_filter`，`dvl`=DVL 传感器系。
  该记号与 AQUA-SLAM 的 `Tbc`/`T_gyro_c`/`T_dvl_c` 一致（见 §3）。

---

## 1. 仿真表（由 `xunyun_fixed.scn` 生成）

传感器位姿 = `<origin>` 原值（机体系）。

| 传感器 | 类型 | xyz [m] | rpy [rad] | rate | 发布话题 |
|---|---|---|---|---|---|
| `dynamics` | odometry | `(0, 0, 0)` | `(0, 0, ~0)` | 30 Hz | `/auv/odometry`（**不**广播 TF，见 `frame_convention.md` §3） |
| `pressure` | pressure | `(-0.24, +0.175, -0.2)` | `(0, 0, ~0)` | 5 Hz | `/auv/pressure` |
| `dvl` | dvl | `(-0.375, 0, +0.1)` | `(0, 0, 0)` | 20 Hz | `/auv/dvl`, `/auv/dvl_altitude` |
| `imu_filter` | imu | `(0, 0, 0)` | `(0, 0, ~0)` | 200 Hz | `/auv/imu` |
| `front_cam_left` | camera | `(+0.23, -0.05, +0.076)` | `(1.5708, 0, 1.5708)` | 20 Hz | `/sim/front_cam/left` |
| `front_cam_right` | camera | `(+0.23, +0.05, +0.076)` | `(1.5708, 0, 1.5708)` | 20 Hz | `/sim/front_cam/right` |
| `down_cam_left` | camera | `(-0.13, -0.05, +0.0645)` | `(0, 0, 1.5708)` | 30 Hz | `/sim/down_cam/left` |
| `down_cam_right` | camera | `(-0.13, +0.05, +0.0645)` | `(0, 0, 1.5708)` | 30 Hz | `/sim/down_cam/right` |
| `line_laser` | multibeam | `(+0.05, 0, +0.19)` | `(0, -1.5708, 0)` | 30 Hz | `/auv/laser_scan` |

**本表就是「如果开了 TF，本该由 `/tf_static` 提供的那些固定变换」**。当前仿真没有 TF（见 `frame_convention.md` §3），所以外参靠本文件 + `config/extrinsics/<platform>.yaml` 传递。

DVL 波束：`beam_angle=30°`（**离竖直** θ）⇒ AQUA-SLAM 的 **α=60°**（离水平），
方位 **β=45/135/225/315°**，`beam_positive_z=true`（波束沿传感器 +z，NED 下即向下）。

线激光：`rpy=(0, -π/2, 0)` 是**纯俯仰 -90°**，于是

| 传感器轴 | 指向（机体系） | 含义 |
|---|---|---|
| x | `+z`（向下） | 扇形中心射线；`θ=0` |
| y | `+y`（右舷） | **线的方向**（横跨航向）：射线在传感器 xy 平面内，`dir=cosθ·x+sinθ·y` |
| z | `-x`（艉） | 扇形平面法线，即光平面法线 = 机体 `-x` |

即 `R_b_laser = [[0,0,-1],[0,1,0],[1,0,0]]`（忽略 `1.5708` 当 π/2 的 3.7e-6 残差）。
`<specs fov="87.19" steps="959">`：扇形 ±43.59°（与下视相机水平 FOV 相同），
引擎实际打 `steps+1 = 960` 条射线（`Multibeam.cpp:45`）。
**注意**：光平面法线是机体 `±x`，而下视相机光轴是机体 `+z` ⇒ 两者**平行**，
所以「用下视相机看这条线再三角化」是退化的（沿条纹方向没有视差）；
`/sim/sensor/laser_points` 的三维点来自引擎自己的射线求交，不经过相机。

<!-- extrinsics-table: begin -->
```yaml
platform: xunyun
source_scene: workspace_sim/src/stonefish_ros2/Data/xunyun_fixed.scn
convention: 'T_<a>_<b> maps a point from frame b into frame a; frames: b=Vehicle/base_link (body, x fwd/y stbd/z down), c=front_cam_left optical, gyro=imu_filter, dvl=dvl sensor'
truth: derived by tools/derive_extrinsics.py; do not edit by hand
sensors:
  dynamics:
    type: odometry
    link: Vehicle
    xyz: [0.0, 0.0, 0.0]
    rpy_rad: [0.0, 0.0, -2.65359e-06]
    rate_hz: 30.0
    ros_publisher:
      topic: /auv/odometry
  pressure:
    type: pressure
    link: Vehicle
    xyz: [-0.24, 0.175, -0.2]
    rpy_rad: [0.0, 0.0, -2.65359e-06]
    rate_hz: 5.0
    ros_publisher:
      topic: /auv/pressure
  dvl:
    type: dvl
    link: Vehicle
    xyz: [-0.375, 0.0, 0.1]
    rpy_rad: [0.0, 0.0, 0.0]
    rate_hz: 20.0
    ros_publisher:
      topic: /auv/dvl
      altitude_topic: /auv/dvl_altitude
    specs:
      beam_angle: '30.0'
      beam_positive_z: 'true'
    beam_angle_deg: 30.0
    alpha_deg: 60.0
    beta_deg: [45.0, 135.0, 225.0, 315.0]
  imu_filter:
    type: imu
    link: Vehicle
    xyz: [0.0, 0.0, 0.0]
    rpy_rad: [0.0, 0.0, -2.65359e-06]
    rate_hz: 200.0
    ros_publisher:
      topic: /auv/imu
  front_cam_left:
    type: camera
    link: Vehicle
    xyz: [0.23, -0.05, 0.076]
    rpy_rad: [1.5708, 0.0, 1.5708]
    rate_hz: 20.0
    ros_publisher:
      topic: /sim/front_cam/left
    specs:
      resolution_x: '1280'
      resolution_y: '960'
      horizontal_fov: '57.19'
  front_cam_right:
    type: camera
    link: Vehicle
    xyz: [0.23, 0.05, 0.076]
    rpy_rad: [1.5708, 0.0, 1.5708]
    rate_hz: 20.0
    ros_publisher:
      topic: /sim/front_cam/right
    specs:
      resolution_x: '1280'
      resolution_y: '960'
      horizontal_fov: '57.19'
  down_cam_left:
    type: camera
    link: Vehicle
    xyz: [-0.13, -0.05, 0.0645]
    rpy_rad: [0.0, 0.0, 1.5708]
    rate_hz: 30.0
    ros_publisher:
      topic: /sim/down_cam/left
    specs:
      resolution_x: '1920'
      resolution_y: '1080'
      horizontal_fov: '87.19'
  down_cam_right:
    type: camera
    link: Vehicle
    xyz: [-0.13, 0.05, 0.0645]
    rpy_rad: [0.0, 0.0, 1.5708]
    rate_hz: 30.0
    ros_publisher:
      topic: /sim/down_cam/right
    specs:
      resolution_x: '1920'
      resolution_y: '1080'
      horizontal_fov: '87.19'
  line_laser:
    type: multibeam
    link: Vehicle
    xyz: [0.05, 0.0, 0.19]
    rpy_rad: [0.0, -1.5708, 0.0]
    rate_hz: 30.0
    ros_publisher:
      topic: /auv/laser_scan
    specs:
      fov: '87.19'
      steps: '959'
transforms:
  T_b_dynamics:
    matrix:
    - [1.0, 2.654e-06, 0.0, 0.0]
    - [-2.654e-06, 1.0, 0.0, 0.0]
    - [0.0, 0.0, 1.0, 0.0]
    - [0.0, 0.0, 0.0, 1.0]
  T_b_pressure:
    matrix:
    - [1.0, 2.654e-06, 0.0, -0.24]
    - [-2.654e-06, 1.0, 0.0, 0.175]
    - [0.0, 0.0, 1.0, -0.2]
    - [0.0, 0.0, 0.0, 1.0]
  T_b_dvl:
    matrix:
    - [1.0, 0.0, 0.0, -0.375]
    - [0.0, 1.0, 0.0, 0.0]
    - [0.0, 0.0, 1.0, 0.1]
    - [0.0, 0.0, 0.0, 1.0]
  T_b_imu_filter:
    matrix:
    - [1.0, 2.654e-06, 0.0, 0.0]
    - [-2.654e-06, 1.0, 0.0, 0.0]
    - [0.0, 0.0, 1.0, 0.0]
    - [0.0, 0.0, 0.0, 1.0]
  T_b_front_cam_left:
    matrix:
    - [-3.673e-06, 3.673e-06, 1.0, 0.23]
    - [1.0, 0.0, 3.673e-06, -0.05]
    - [0.0, 1.0, -3.673e-06, 0.076]
    - [0.0, 0.0, 0.0, 1.0]
  T_b_front_cam_right:
    matrix:
    - [-3.673e-06, 3.673e-06, 1.0, 0.23]
    - [1.0, 0.0, 3.673e-06, 0.05]
    - [0.0, 1.0, -3.673e-06, 0.076]
    - [0.0, 0.0, 0.0, 1.0]
  T_b_down_cam_left:
    matrix:
    - [-3.673e-06, -1.0, 0.0, -0.13]
    - [1.0, -3.673e-06, 0.0, -0.05]
    - [0.0, 0.0, 1.0, 0.0645]
    - [0.0, 0.0, 0.0, 1.0]
  T_b_down_cam_right:
    matrix:
    - [-3.673e-06, -1.0, 0.0, -0.13]
    - [1.0, -3.673e-06, 0.0, 0.05]
    - [0.0, 0.0, 1.0, 0.0645]
    - [0.0, 0.0, 0.0, 1.0]
  T_b_line_laser:
    matrix:
    - [-3.673e-06, 0.0, -1.0, 0.05]
    - [0.0, 1.0, 0.0, 0.0]
    - [1.0, 0.0, -3.673e-06, 0.19]
    - [0.0, 0.0, 0.0, 1.0]
  T_gyro_c:
    matrix:
    - [-6.327e-06, 3.673e-06, 1.0, 0.230000133]
    - [1.0, 0.0, 6.327e-06, -0.04999939]
    - [0.0, 1.0, -3.673e-06, 0.076]
    - [0.0, 0.0, 0.0, 1.0]
  T_dvl_c:
    matrix:
    - [-3.673e-06, 3.673e-06, 1.0, 0.605]
    - [1.0, 0.0, 3.673e-06, -0.05]
    - [0.0, 1.0, -3.673e-06, -0.024]
    - [0.0, 0.0, 0.0, 1.0]
  T_b_c:
    matrix:
    - [-3.673e-06, 3.673e-06, 1.0, 0.23]
    - [1.0, 0.0, 3.673e-06, -0.05]
    - [0.0, 1.0, -3.673e-06, 0.076]
    - [0.0, 0.0, 0.0, 1.0]
  T_c_b:
    matrix:
    - [-3.673e-06, 1.0, 0.0, 0.050000845]
    - [3.673e-06, 0.0, 1.0, -0.076000845]
    - [1.0, 3.673e-06, -3.673e-06, -0.229999537]
    - [0.0, 0.0, 0.0, 1.0]
```
<!-- extrinsics-table: end -->

> 块里的 `1.0` / `0.0` 是**四舍五入**的显示值；`3.673e-06` 来自场景把
> `1.5708` 当 π/2（真值 1.5707963…）。全精度版本在
> `workspace_sim/src/uv_sim/config/extrinsics/xunyun.yaml`（同一生成器产出）。

---

## 2. 实机表（测量值 / 待确认）

数值来自 `workspace_auv/docs/perception.md` 与 `devlog/2026-07-05_status.md`。
**「状态」列必须如实标注**：未测量过的不能当已知。

| 传感器 | xyz [m]（机体系） | 状态 | 依据 |
|---|---|---|---|
| DVL | `(-0.375, 0, +0.1)` | 已测量（与仿真一致） | `docs/hil_architecture.md`；`xunyun_fixed.scn` 注释 |
| IMU | `(0, 0, 0)` | 假定（与艇体原点重合） | 场景 `<origin xyz="0 0 0">` |
| 前视相机 | `(+0.23, ∓0.05, +0.076)` | 代码权威（数值未经实机复测） | `workspace_auv/docs/perception.md`；`position.py`/`line_follower.py`/`arrow_surfacer.py` 三处一致 |
| 下视相机 | `(-0.13, ∓0.05, +0.0645)` | 代码权威（数值未经实机复测） | 同上 |
| 压力计 | `(-0.24, +0.175, -0.2)` | 假定 | 场景 `<origin>` |

**N1 已决（2026-09-15）**：仿真场景原先把相机 z 写成 `0.276`/`0.2645`，比实机表多 0.2 m；
现已把 **场景改成实机值**（`0.076`/`0.0645`）。判定依据是**一致性**而不是测量：

- `position.py` 的 `CAM_PARAMS`、`line_follower.py` 的 `_DOWN_OFFSET_*`、
  `arrow_surfacer.py` 的 `_DOWN_OFFSET_*` 三个模块与 `docs/perception.md` 都用 `0.076`/`0.0645`（4:1）；
- 场景里的 `0.276` 只此一处，且与同文件 `pressure` 的 z 值体系不自洽。

残留下限：这组数字本身仍来自早期估算（`devlog/2026-07-05_status.md` 记「实车外参是否正确**未验证**」），
若日后在艇上量出别的值，改 `position.py` + 三个模块 + 场景，再跑
`derive_extrinsics.py --out` 与 pytest 即可（`test_frames_geometry.py` 会钉住）。

---

## 3. 变换推导（含与 AQUA-SLAM 的对照）

### 3.1 从场景到矩阵

对每个传感器，`<origin xyz rpy>` **就是** `T_b_s`（机体系→传感器系），因为 Stonefish 把
`<origin>` 当作传感器在父 link 下的位姿。于是：

```
T_s_b = inv_rigid(T_b_s)                 # [R^T | -R^T t]
T_gyro_c = inv_rigid(T_b_gyro) @ T_b_c   # 陀螺系 -> 相机系
T_dvl_c  = inv_rigid(T_b_dvl)  @ T_b_c   # DVL 系 -> 相机系
```

`c` 取**前视左目光学系**：AQUA-SLAM 的 `Tbc`/`T_gyro_c`/`T_dvl_c` 全部以左目相机为参考。

### 3.2 AQUA-SLAM 的 `Tbc` 是什么

`AQUA-SLAM/data/underwater_orbslam3_blue_gx5_short.yaml`（blue_gx5 平台的标定结果）：

```yaml
Tbc: [ 0,  0,  1,  0.4,      # 旋转块 [[0,-1,0],[0,0,-1],[1,0,0]] 的转置
       -1,  0,  0,  0,       # 平移列 = 相机在机体系的位置 (0.4, 0, 0.1)
        0, -1,  0,  0.1,
        0,  0,  0,  1 ]
```

- 平移列 `(0.4, 0, 0.1)` = 相机在机体系的位置 ⇒ `Tbc` **就是** `T_b_c`，与本文档记号一致。
- 旋转块 `[[0,-1,0],[0,0,-1],[1,0,0]]` 与本文档的 `T_b_c` 旋转块**逐元素相同**
  （本文档是 `[[~0,~0,1],[1,0,~0],[0,1,~0]]`，差值来自 `1.5708` 的舍入）。
  两个独立来源给出同一个矩阵，说明记号与推导没有搞反。
- 因此：**`Tbc`（blue_gx5 的数值）与 XUNYUN 的 `T_b_c` 结构相同、数值不同**
  （XUNYUN 相机在 `(0.23, -0.05, 0.076)`）。

**硬规则 D3：外参按平台独立存放，永不跨平台复用。**
配置落在 `workspace_sim/src/uv_sim/config/extrinsics/<platform>.yaml`
（仿真侧）与实机测量表（硬件侧）；blue_gx5 的值套到 XUNYUN 上就是错的。

### 3.3 手算核对（本表被单测钉住）

| 量 | 推导 | 值 |
|---|---|---|
| `T_dvl_c` 平移 | `r_c − r_dvl = (0.23−(−0.375), −0.05−0, 0.076−0.1)` | `(0.605, −0.05, −0.024)` |
| `T_gyro_c` 平移 | `r_c − r_gyro = (0.23, −0.05, 0.076)` | `(0.23, −0.05, 0.076)` |
| `T_b_c` 旋转块 | 光学系→机体系：x=右, y=下, z=前 | `[[~0,~0,1],[1,0,~0],[0,1,~0]]` |

`test_frames_geometry.py` 对这四行逐个断言，改场景就会红。

---

## 4. DVL 波束几何与误差量级

数值验证见 `workspace_sim/tools/check_dvl_geometry.py`（无 ROS 可跑）。

| 物理量 | 值 | 来源 |
|---|---|---|
| θ（波束离**竖直**） | 30° | 实机 DVL 规格；场景 `<specs beam_angle="30.0">` |
| α（波束离**水平**） | 60° = 90°−θ | AQUA-SLAM `mE` 的 `sin(α)` 是 z 分量 |
| β（方位角） | 45/135/225/315° | 引擎固定 `45°+i·90°`；实机同布局 |
| `EᵀE` | `diag(0.5, 0.5, 3.0)` | α=60°、上述 β |
| 反演误差 | `~1e-16` | `(EᵀE)⁻¹Eᵀ(E v) = v` |

**α 敏感度**（真值 60°，只改反演用的 α）：

| 反解用的 α | 水平速度尺度 | 水平误差 | 垂直误差 |
|---|---|---|---|
| 30°（把 θ 当 α） | 0.5774 | **−42.26%** | +73.21% |
| 45° | 0.7071 | −29.29% | +22.47% |
| **60°（真值）** | **1.0000** | **0.00%** | 0.00% |
| 65° | 1.1831 | +18.31% | −4.44% |
| **67.5°（AQUA-SLAM 出厂默认）** | **1.3066** | **+30.66%** | −6.26% |
| 70° | 1.4619 | +46.19% | −7.84% |

规律：`水平尺度 ≈ cos(α_used)/cos(α_true)`；垂直几乎不串扰。
`cos(67.5°)/cos(60°) = 1.3066` 就是 2026-09 讨论里那个「+31%」的来源。

**D4**：仿真里「用 α 合成波束、再用 α 反解」永远自洽，**不能**证明 α 正确——
只有实机数据（或对真值的比对）才暴露 α 错误。别拿仿真通过当 α 的验证。

---

## 5. T6 移植必改清单（AQUA-SLAM 侧）

AQUA-SLAM 参考检出（`AQUA-SLAM/`，未纳入本仓库）里的设置文件是 **blue_gx5** 平台的，
直接拿来跑 XUNYUN 会静默错。移植时必须逐项覆盖：

| 配置项 | 检出里的值 | XUNYUN 应为 | 说明 |
|---|---|---|---|
| `alpha` | `[1.1780]×4` rad = 67.5° | `[1.0472]×4` rad = **60°** | 不改正则水平速度 **+30.7%** |
| `beta` | `[0.7854]×4`（单值 45°） | `[0.7854, 2.3562, 3.9270, 5.4978]` | 需**逐波束** 4 个方位 |
| `Tbc` | blue_gx5 的 `(0.4, 0, 0.1)` | 本表 `T_b_c`（`(0.23, -0.05, 0.076)`） | 由 `derive_extrinsics.py` 产出 |
| `T_gyro_c` | blue_gx5 标定值 | 本表 `T_gyro_c` | 同理 |
| `T_dvl_c` | blue_gx5 标定值 | 本表 `T_dvl_c` | 同理 |
| `LeftImgTopic`/`RightImgTopic` | `/camera/left|right/image_dehazed` | 本仓库话题（`/auv/front_cam/stitched` 经拆分，或 `/sim/front_cam/*`） | 见 `docs/node_parameters.md` |
| `ImuTopic` | `/imu/data` | `/auv/imu`（200 Hz） | 已与 `IMU.Frequency: 200` 对齐 |
| `DvlTopic` | `/bluerov2/DVL` | `/auv/dvl` | 消息类型也不同（见 T6 计划） |

`dvl_model/dvl.cpp` 里另有硬编码 `ALPHA = 67.5°`，同样要改成 60°。

---

## 6. T6 前置关卡（两项静态校验）

移植 AQUA-SLAM 之前必须过；两项都只依赖已有的仿真话题，不需要 GPU 之外的额外条件。

### 关卡 1：静止 30 s，SLAM 输出的相机系速度 ≈ 0

- 条件：艇静止、水面平静、DVL 底距有效（`altitude > 0`）。
- 判据：`|v_camera| < 3σ_dvl = 3 × 0.0015 = 0.0045 m/s`，且 30 s 内无单调漂移趋势。
- 意义：外参或 α 错会让静止时的速度反解出非零值。

### 关卡 2：已知方向运动，首帧位移方向与 `/auv/odometry` 一致

- 条件：沿机体系 +x 直线运动 ≥ 1.5 m，yaw 保持 0。
- 判据：SLAM 首帧位移方向与里程计真值夹角 **< 5°**；位移尺度比落在 **[0.95, 1.05]**。
- 意义：尺度比正是 α 错误的直接体现（+30.7% 会明显超出区间）。

实现方式：T6 落地时把这两项写成脚本（可复用 `check_dvl_frame.py` 的运动序列），
在 GPU 会话/实机上执行。

---

## 7. 复核命令

```bash
python3 workspace_sim/tools/derive_extrinsics.py            # 打印推导
python3 workspace_sim/tools/derive_extrinsics.py --check    # 与本文档块对拍
python3 workspace_sim/tools/check_dvl_geometry.py           # 波束几何 + 敏感度
python3 workspace_sim/tools/dump_tf_tree.py --self-test     # TF 报告逻辑（无 ROS）
python3 -m pytest workspace_sim/src/uv_sim/test/ -q         # 含本文件的契约测试
```
