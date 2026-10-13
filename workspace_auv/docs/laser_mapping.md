# 线激光建图（`uv_mapping`）

> 本文是线激光建图的**设计说明与实测误差预算**。代码在
> `workspace_auv/src/uv_mapping/`；参数与启动参数见 `docs/node_parameters.md`；
> 话题契约见 `docs/interfaces/topic_contract.md` §1.1/§1.2/§7.5。
>
> 建立于 2026-10-13。**所有数值都来自实测**（三份 rosbag，见本文 §6），
> 推导过程与反例也写在下面 —— 因为建图最容易出的不是崩溃，而是**看起来对的错图**。

## 1. 传感器与数据链路

```
Stonefish multibeam `line_laser`      一个平面上的射线扇形 = 海床上的一根线
  → /auv/laser_scan                    sensor_msgs/LaserScan, 30 Hz, 960 条射线
  → uv_sim/laser_sim                   丢掉未命中的射线（+ 可选水下衰减）
  → /sim/sensor/laser_points           sensor_msgs/PointCloud2, 激光平面系, z≡0
  → uv_mapping/laser_map               monitor: 记录期 QA；live: 滚动窗口地图
  → ros2 bag record                    （原始数据，充分统计量）
  → uv_mapping/map_replay              离线：2.5D 网格 + metrics.json + 图
```

外参 `T_b_line_laser` 的唯一数值权威是 `docs/frames/extrinsics_reference.md`
（由 `workspace_sim/tools/derive_extrinsics.py` 从场景生成）。
`workspace_auv` 不能依赖 `workspace_sim`，所以这里有一份运行时副本
`uv_mapping/config/laser_extrinsics.yaml`，由 `test/test_extrinsics.py` 双向对拍钉住：
既比派生矩阵，也比文档里的 `xyz`/`rpy_rad` 重新推一遍的结果。

**安装的平移项不能丢**：`(0.05, 0, 0.19)`。GUI 的 `_LASER_R_BS` 只做旋转
（`visualization/auv_visualizer.py` 的注释也说了那是为了剖面图），做地图时漏掉它
就是 19 cm 固定深度偏置 —— 6 m 量程上 3% 尺度误差。`test_bathymetry.py` 里有一条
测试专门用"只转不平移"跑一遍，断言整张图浅 0.19 m。

## 2. 为什么是"先记录、后处理"

`ros2 bag record` 存下的是**充分统计量**：激光系条纹 + 位姿流。世界点由
`T_map_body(t) · T_b_laser · p_laser` 唯一决定，所以

* 离线可以从同一份记录重放**任意**算法、任意位姿源、任意插值策略；
* 实时建图的结果**推不回**原始数据 —— 它把"用哪个位姿"这个本该事后决定的
  选择焊死在当时。

再加三条实际理由：仿真的启动成本远高于离线读 npz；**误差只能量化才能验收**
（仿真里 `/auv/odometry` 是真值，可以算 RMSE，而屏幕上的图只能"看着像"）；
实机跑一次就没了，记录是黑匣子。

同一条路在水下 SLAM 那边已经被走过：参考检出 `AQUA-SLAM/src/DenseMapper.cpp`
有 `ReconstructFromBagPose(bag, pose_file)`，`src/test/dense_viewer.cpp` 是独立的
离线回放工具，轨迹文件是 `<t tx ty tz qx qy qz qw>` 八列 —— 与本文的
`uv_mapping/replay.py` 是同一个分工。

**现成节点不能直接借**：`slam_toolbox` / `nav2_costmap` / `octomap_server` 都假设
**水平旋转式激光 + 完整 TF**，而本项目是单线朝下、`/tf` 实测 0 条消息（B18）。
硬套就得伪造 TF、谎报扫描几何 —— 这正是本项目已经踩过两次的坑型
（B16 用错 α 让水平速度 +31%、B21 忽略 frame 位）。借的是**算法与流水线**，
不是节点。

## 3. 算法：死推地形，不做 SLAM

```python
p_map = T_map_body(t_strip) · T_b_laser · p_laser
```

就这一行。单趟不需要配准：实测真值位姿下重叠条纹的一致度是 **0.34 cm**，
而 5 cm 网格的噪声地板（§4）本身就是 0.34 cm —— 也就是说**底下没有别的误差了**。
扫描匹配（ICP/GICP）买的是长航程的闭环，不是一阶精度；在把位姿链搞清楚之前上它，
只会把主导误差项藏起来。

网格是 2.5D（每个 `(x, y)` 格一个深度中位数）。

## 4. 误差预算（实测）

指标叫**条纹间自洽度**：同一 5 cm 格里，各条纹中位深度之间的极差。
位姿精确、海面静止时它应当为 0，所以它是**不需要真值的误差探针**。
地形坡度会污染它，所以按粗网格（0.25 m）坡度把它分成**平坦**与**陡峭**两组：
在平地上，纯水平位姿误差**不产生高度误差**（§5），所以平坦组主要量的是姿态与量测。

| 配置 | 平坦区条纹间极差 p50 / p95 |
|---|---|
| 真值位姿（`/auv/odometry`，30.0 Hz，带 header） | **0.34 / 0.63 cm** |
| `/zit6/state/pos`（29.9 Hz，**无 header**，接收时刻打戳） | 1.11 / 2.68 cm |
| 同上 + 插值到位姿时间戳 | 0.94 / 2.33 cm |
| 平滑定深巡航 + `/zit6/state/pos` + 插值 | **0.45 / 1.87 cm** |
| `/basic_motion/pose_info`（21~25 Hz，最大空洞 0.65 s，B20） | 1.53 / 7.36 cm |

**噪声地板可以被预测，也验证了**：场景激光噪声 σ = 2 mm，5 cm 格里约 5 条条纹
各贡献 1 个点，极差期望 `d2(5)·σ = 2.326 × 2 mm = 4.7 mm`。实测 p50 = 0.34 cm。
这一条很重要：它说明 0.34 cm 里没有剩给外参、插值或网格的系统误差。

### 主导项是位姿的**时间戳**，不是位姿的精度

把"贴最近位姿"与"插值到位姿"的差别按分量分解（真值 = 引擎 odometry 插值）：

| 误差分量 | 对深度的 rms 贡献 |
|---|---|
| **平移** | **1.572 mm**（p95 4.05 mm） |
| 姿态（合计） | 0.301 mm |
| └ 横滚 | 0.299 mm |
| └ 俯仰 | 0.026 mm |
| └ 偏航 | 0.301 mm |

`/zit6/state/pos` **没有 header**，只能按接收时刻打戳，误差约 ±半个周期（≈17 ms）。
于是垂直位置误差 ≈ `dz/dt × δt`：平缓巡航时 `dz/dt` 的 rms 是 **26.5 mm/s**，
停走航迹是 **199.7 mm/s**（7.5×）。17 ms × 200 mm/s = 3.4 mm —— 与上表的
平移项同量级，**机制对上**。

这解释了为什么"运动更平稳 → 建图更准"，也把优化方向指清楚了：

1. **让位姿带上时间戳**（或融合 60 Hz 的 `/zit6/state/vel` 做一阶外推）——
   这是攻根因；
2. **定深、匀速巡航** —— 软件一行不改就把残差压掉。

> 作者一开始猜的是"偏航误差 × 射程"，还据此写了一条测试。分解结果证明**不是**：
> 绕竖直轴转不改变任何点的 z，平地深度的偏航灵敏度严格为 0（那条测试现在断言的是
> `spread < 1e-9`）。留在这里是因为这个坑很自然 —— 单线朝下测的是地形，
> 不是水平切片。

## 5. 地形误差的机制（`test_bathymetry.py` 逐条钉住）

| 扰动 | 对平地深度的影响 | 测试 |
|---|---|---|
| 垂直位置误差 | **1:1** 进深度 | `test_vertical_position_error_enters_the_depth_one_to_one` |
| 水平位置误差 | **完全不进深度**（地图整体平移，深度是对的） | `test_horizontal_position_error_does_not_corrupt_depths_on_flat_ground` |
| 横滚抖动 δ | `横向偏移 × sin δ`：近 nadir 处为零、边缘最大 | `test_roll_jitter_depth_error_is_proportional_to_the_across_offset` |
| 俯仰抖动 δ | 力臂是 0.05 m 的安装前移 ⇒ 比横滚小 ~30 倍 | `test_pitch_jitter_is_negligible_next_to_roll_jitter` |
| 偏航抖动 δ | **严格为 0**（绕竖直轴不改变 z） | `test_yaw_jitter_has_no_effect_on_flat_ground_at_all` |

"平地上水平误差不进深度"这一条有个实践含义：**自洽度指标在平地上量的是垂直精度，
不是定位精度**。所以一个没有时间戳的位姿在平地上代价很小，只有车在上下动的时候
它才变成深度误差 —— 这与 §4 的实测完全一致。

## 6. 实测数据从哪来

三份 bag（`/tmp/laser_survey/`，不入库：约 150 MB）：

| bag | 航迹 | 用途 |
|---|---|---|
| `bag1` | BTRAVEL/BMOVE 走走停停，12.9 s 原地打转 | 停走对照 |
| `bag2` | 首次平滑尝试，**作废**：下潜目标 9.06 m 在海床（~6.2 m）以下，船被压在底上爬了 50 s | 反例，留作"命令深度低于海床不会报错，而是静默毁掉测绘"的证据 |
| `bag3` | 单条 BVELOCITY 75 s 直线，0.19~0.28 m/s | 平滑对照 |

复现命令：

```bash
ros2 launch uv_sim_bringup sim_bringup.py \
    scenario_desc:=canyon_vision_dvl_headless.scn headless:=true \
    enable_motion:=true enable_ai:=false enable_nav:=false enable_task:=false
ros2 bag record -o /tmp/survey/bag \
    /sim/sensor/laser_points /zit6/state/pos /auv/odometry /basic_motion/pose_info
ros2 run uv_mapping map_replay --bag /tmp/survey/bag --out /tmp/survey/map \
    --pose all --figure
```

**约 21.7 MB/min**（30 Hz × 960 点 × 12 字节 ≈ 345 KB/s，加位姿流）。
30 Hz 全帧、0 空帧、引擎戳到收到 p95 = 1.4 ms。

## 7. 两条已定量化的工程结论

**① 运动要平滑，而且不平滑的代价不在"图不好看"。**
停走航迹 12.9 s 在原地打转、8.28% 的条纹相对前一帧转了角度、速度掉到中位数 50% 以下
的时间占 32.5%；平滑航迹这三项分别是 0.00 s / 0.12% / 0.0%。条纹间距 CV 0.72 → 0.47。
真实收益是**位姿误差的放大倍数**：停走 3.3×、平滑 1.3×（相对真值）。

**② 监视节点必须比被监视的链路便宜。**
`live` 模式第一版每 2 s 对**全量**累积点重算网格，结果条纹速率从 30.0 Hz 掉到
**13.9 Hz**、位姿间隔涨到 **3.74 s** —— 一个把被监视对象拖垮的监视器比没有更糟。
现在 `live` 用滚动窗口（`map_window_strips`）+ 每格每条纹只留一个点，实测回到
**29.9 Hz / 51 ms**。`monitor` 模式不算网格，全程 30.0 Hz / 44 ms。
离线路径刻意**不封顶**：它没有截止时间。

## 8. 已知缺口

| 缺口 | 说明 |
|---|---|
| **实机线激光话题未定义** | 目前只有仿真传感器（`line_laser`）。`laser_map` 直接订阅 `/sim/sensor/laser_points`，与 T6 的 AQUA-SLAM 消费 `/sim/sensor/dvl` 同一形状；实机入口出现时需补 `/auv/*` 契约行 |
| **`live` 模式未验收（L3）** | 精度结论全部来自离线 `map_replay`；`live` 只保证"不比 `monitor` 更贵"，不保证同样准 |
| **位姿无时间戳** | 根因见 §4。优先级最高的改进是让 MCU 给 `/zit6/state/pos` 打戳，或融合 `/zit6/state/vel` |
| **长航程没有闭环** | 单趟死推；多趟或长距离需要扫描匹配（`ros-jazzy-pcl-ros` 可从 tuna 镜像装，`libpcl-dev` 1.14 已在机器上）|
| **陡坡处的水平误差未量化** | 平坦区指标对水平误差盲（§5）；崖壁附近需要另一套度量 |
