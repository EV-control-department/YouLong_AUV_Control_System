# 抓海参：近底DVL偏移与大收集框投放修正

本报告对应正式任务 `src/uv_task/uv_task/grab_sea_cucumber.py` 和
`src/uv_task/config/tasks/grab_sea_cucumber.yaml`。2026-10-10 修正；未做水下实机验证。

2026-10-12导入调试参数后：速度环上浮上限0.10m/s、近目标减速增益0.20/s、
最低指令速度0.003m/s；近底纯推力离底仍由原lift_thrust控制，速度指令不等于实测速度。
海参总预算900秒、首次/返回搜索区180秒、运输/投放兜底190秒、抓后上浮与水平回观察位360秒。
最后一项不控制投放后返回搜索区。每次travel记录局部上限、剩余总预算和有效预算，
以区分局部动作超时和总预算截断；全流程引用该任务配置时同样生效。

针对1779122057的视觉小步超时：旧到位阈值为min(3cm,半步)，1.8cm指令仅允许
约0.9cm残差；日志出现约1cm反馈残差，稳定等待未通过。
新增`servo.small_step_tolerance_m: 0.015`，实际容差为
`min(position_tolerance_m, 0.8*步长, max(small_step_tolerance_m, 0.5*步长))`。
该1.8cm指令允许约1.44cm残差，同时仍拒绝未移动的反馈，要求连续稳定0.3秒，
再取得新采集帧确认视觉居中；位置容差不能单独授权下压。
单轮`servo.timeout`从30秒增至60秒。总预算耗尽或节点停止会明确说明无法重试；
仅凭这段日志不能确定当次是否总预算耗尽，局部伺服失败通常仍进入视觉重试。

## 2026-10-12 收集框简化流程

现实现为 `运输至delivery.pose → 下视双目bbox中心居中 → collection_release_offset_body_xy
实际偏置 → 实测位置稳定 → gripper.settle_seconds固定等待 → 释放 → delivery.release_wait_seconds`。
默认偏置为body `[+0.30,0.0]` 米，仅执行一次；不叠加海参抓取的gripper.offset_x/y。
偏置保持z/yaw；偏置或实测到位失败仍尝试释放，外部停止除外。总预算耗尽时只发一次释放清理，不继续导航。
关闭视觉或视觉失败时保留配置点兜底；兜底移动失败也释放，当前区域未确认时不能当作有效投放证明。

每目从收集框类别、置信度及有限且有效的bbox中选取目标，两目各需恰好一个目标。
优先用非零stereo_pair_id匹配同一双目对；旧消息没有pair id时允许采集时间差最多50ms。
缺目、不同步或多候选框时使用新鲜单目框；多框选最靠近该目光心的可信框，不再因双目不完整直接拒绝。
两目都必须新鲜且采集于动作稳定后；重复采集帧不再次下发修正。
使用实际左右K分别计算du=(u-cx)/fx、dv=(v-cy)/fy，再平均两目的du/dv。
这个误差驱动已有水平步进，保持z/yaw，不进行三角测距或落点几何判断。
`collection_projection_depth_m`仅作为步长比例的参考距离，`collection_pixel_tolerance_fraction`
与`collection_hold_seconds`控制双目平均误差的居中判断。

已移除多边形内部点搜索、爪子落点投影、边缘余量、完整框心采样、收集框odom校正，
以及偏置后再次视觉对准。收集框只需检测框，不要求分割mask；海参抓取与计数仍需mask。
裁切框允许使用可见bbox，但中心会随裁切改变，这个版本不保证真实框心或携带物落点在框内。
左右中心平均不是高精度三维定位，需要现场录像验证独立投放偏置。
参数清理详见[参数复核](grab_sea_cucumber_parameter_review_20261012.md)。

## 2026-10-12 投放与上浮顺序修正

原travel在WTRAVEL返回后把目标z替换为实测z，只做xyrz归向与XY/yaw确认。
由于BasicMotion本身允许较宽的位置容差，目标更深时可能在下沉尚未完成时释放。
现目标更深时保留配置z，用xyzrz保持并继续确认实测深度，误差不大于
min(ascent.tolerance_m,0.05m)；未到位禁止视觉投放、偏置与释放。
速度上浮完成后使用实测z运输和保持，上浮过冲不再下沉回补。

释放与release_wait_seconds完成后，统一速度上浮到search.pose.z，包括最后一轮。
然后才按return_to_search决定是否水平返回搜索区。当前delivery.pose.z=0.20m，
search.pose.z=0.10m，均为START后深度坐标，向下为正；没有新增投放下沉深度参数。
近底上浮仍先lift_thrust=-0.3，最低参考点离底超过0.20+0.03m后切速度环。
速度环20Hz，vz=-min(0.10,max(0.003,0.20*(z-target_z)))m/s，
剩余深度≤0.05m时发零速度结束，已经更浅时不向下纠偏。
原return_timeout仅控制抓后上浮与回观察位，投放后上浮受剩余整项任务预算限制。

## DVL偏移处理的完整链路

### 1. 先区分两个XY位置

设A为海参视觉居中后的艇体观察点，当前航向为ψ；爪子水平补偿为o。
艇体主动移动后的抓取位置为B：

```text
B_xy ≈ A_xy + R(ψ) o
```

任务顺序是 `视觉居中 → 抓前计数 → 爪子补偿 → 稳定等待 → 记录下压前XY`。
`_pre_press_pose` 记录的是补偿完成后的实测B，不是A，也不是补偿命令的理论目标。
这一顺序原来就是正确的，本次未将B改成A。
若错误地用A作为恢复锚点，会把爪子主动移动的约0.204m误认为DVL漂移。
当前舵机1补偿 `(+0.20,+0.04)m`，ψ=−90°时，世界补偿为 `(+0.04,−0.20)m`。

### 2. 20cm开环切换必须计入爪子下伸

旧舵机1只用了 `bottom_reference_offset_z_m: 0.10`。
圆盘爪名义位置为 `[-0.430,0,0.290]m`，旧算法可能直到爪子只剩约1cm离底时
才触发“20cm”开环。因此遗漏主要在垂直参考点，而不是XY锚点的记录顺序。

新增 `gripper.contact_body_xyz`，舵机1默认采用上述名义参考点；舵机2采用
`front_camera_body_xyz + gripper_from_front_xyz` 的实际配置。
对爪子参考点g=(gx,gy,gz)，按实测roll φ、pitch θ计算向下投影：

```text
g_down = -sin(θ) gx + cos(θ) sin(φ) gy + cos(θ) cos(φ) gz
reference_down = max(bottom_reference_offset_z_m, g_down)
clearance = floor_z_m - robot_z - reference_down
```

水平时，本配置切换深度为 `1.20−0.29−0.20=0.71m`；旧值为 `0.90m`。
实际接触点须实测，0.290m是仓库机械参考位置，不是本次现场标定结果。
`floor_z_m`、robot_z和安装偏置必须采用一致的深度参考基准。
`bottom_reference_offset_z_m` 还用于DVL/艇底更低的情况，不再被舵机2参数直接覆盖。

### 3. 近底纯推力只隔离控制，不冻结导航

高度大于20cm时使用配置的正z速度；达到门限后锁定ACTUATOR模式：
`control_key=0x12`，六轴全写入，只有z使用 `press_thrust`，其余为0。
纯推力持续 `press_thrust_seconds`，受下压时间和任务总预算约束；结束或异常发零推力。
进入后不因位置跳变回切速度模式。

离底先用负 `lift_thrust`，参考点高于23cm后清零推力，恢复慢速负z速度上浮到A的观察深度。
两段纯推力均不保持XY/姿态，所以真实水平移动和航向变化仍可能发生。
当前现场配置为下压推力+0.2、离底推力−0.3；归一化推力不是m/s，数值大小不能证明
实际离底速度缓慢。本次保留了这些现场推力值。

仓库固件的 `INS_Driver::decodePacket` 仍接收NAV-300的north/east偏移作为XY；
`ControlTask::updateNavigation` 的NORMAL路径用独立深度驱动覆写z。
任务没有冻结NAV/DVL积分，也没有使用DVL高度值作为20cm开关。
正在运行的固件是否与源码一致，需要部署时核对。

### 4. 离底后按锚点修正任务odom

上浮到观察深度后读取实测C。若 `restore_pre_press_odom_xy: true`：

```text
任务坐标平移 δ = B_xy - C_xy
```

通过 `/basic_motion/correct_odom_xy` 修改BasicMotion任务坐标变换，使当前反馈
恢复到B。服务同时调整内部目标的坐标表达，保持已发MCU目标不跳变；任务侧
`_cmd_x/_cmd_y` 同步平移。之后再执行水平SET，从B真实返回A，才能在原观察点复检。
搜索点和投放点保留原START坐标定义，服务不改z/yaw，也不调用MCU setorigin。
再次START会重置这项平移修正。

服务要求没有正在执行的BasicMotion动作、位姿新鲜、调用期间位置变化不超过5cm。
任务的单次修正上限当前为 `odom_correction_max_m: 1.0`。

这一方法将B→C的全部XY变化视为导航漂移，覆盖下压和上浮整个时段。
如果水流让艇体真的移动了d，那么恢复后会残留约d的定位误差。
爪子偏置可以算出主动补偿，但不能用它判断纯推力阶段的真实水平位移。
开关关闭时不改odom，直接按当前反馈返回A；若反馈已漂移，真实返回点也会偏。

### 5. 收集框地标校正已移除

2026-10-12简化版不再根据收集框更新任务odom。近底下压前B锚点校正仍保留，
仅在restore_pre_press_odom_xy开启时使用，仍假定真实水平漂移为零。
如需辨别真实水流漂移与DVL漂移，需要独立观测；当前双目框中心伺服不承担该功能。

## 部署与验证

新增参数不改变ROS消息/服务定义，部署至少重编 `uv_task` 并重启task_runner；
既有BasicMotion必须支持CorrectOdomXY接口。只修改源码YAML不会让已运行任务自动重载。

回归覆盖无mask双目框中心、左右K归一化、缺目/不同pair/多框单目降级、旧帧拒绝、
重复帧不追加动作、裁切bbox、独立投放偏置仅一次、实际到位/稳定等待失败仍释放，
以及两种爪子参考点、姿态投影与不同航向下B锚点校正后返回A。
这些是软件回归，不代表水下抓取、DVL恢复和实际落点已经通过实机验证。
