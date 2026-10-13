# BLINE 定速有限直线段

`BasicMotion.Goal.BLINE=8` 使用动作开始时的实测位置、航向固定起终点。
`target=[dx,dy,dz,dyaw_deg]`，`axes` 为空或 `xyz`。XY 位移只按开始航向旋转，
Z 为深度变化；速度转换使用实时完整 RPY。巡航速度 `cruise_speed=0`
使用 `line.default_speed`，默认 0.30 m/s；正的有限速度请求超过上限时自动限幅，不拒绝请求。
沿线巡航速度严格小于 0.80 m/s，纠偏后的 XYZ 合成指令速度严格小于
0.80 m/s（预留 0.000001 m/s 浮点传输余量）。`line.max_speed`
可进一步收紧合速度限制；负速度、NaN/Inf 和无效位移仍拒绝。
满速时先计算所需纠偏速度，再降低沿线目标速度预留合速度预算；
沿线加减速和整个速度向量的变化率限制仍生效。

阶段：保持起点转向（ALIGN，最多 30 秒）→ 捕获/沿线跟踪
（CAPTURE/CRUISE）→ 制动（BRAKE）→ 位置保持停车（STOPPING）
→ 保持终点并转向（FINAL_TURN）→ 稳定保持（HOLD）。距固定终点的三维距离
不超过 0.35 m，或已进入到点容差范围时，即结束速度租约、发送零速度，
切换到固定终点的位置保持模式制动；此时保持行进航向。停车阶段不再切回速度回拉。
默认 XY 每轴误差不超过 0.30 m、Z 误差不超过 0.15 m、
实测三维合速度不超过 0.03 m/s、航向误差不超过 3°，连续满足 1.2 秒后
才发送最终航向。转向后在 HOLD 再连续满足上述条件 1.2 秒才返回成功。
速度始终不达标或位置持续超差时不会报成功，仍受全程超时限制。
控制频率 20 Hz；阶段切换及每秒输出进度、偏差、固定终点、
目标/实测沿线速度、实测三维合速度和终点 XYZ 误差。

`timeout>0` 覆盖转向、跟踪和保持全过程；否则自动取
`max(15, length/cruise_speed+10+终点转向预算)` 秒。
`position_tolerance` 可为单个 BLINE/WLINE 目标覆盖到点位置容差（米）；0 使用
`line.position_tolerance`。它影响停车入口和终点到达判断，不改变起点对准容差。
速度反馈和定位、导航样本、MCU 状态都必须新鲜，并通过已有 START、
原点代次、解锁检查。现有 real 模式临时 `force_nav_valid` 设置保持有效。

跟踪期间拒绝其他非零 BODY_VELOCITY；零速度请求中断 BLINE。
取消、超时、状态异常、租约过期都结束速度租约并发零速度。
定位及运动状态仍有效时切换当前位置保持；无效时只发送零速度。
成功后保留终点位置模式。Result 的 `final_target` 返回固定世界终点和航向，
`bline` 任务使用它同步指令位姿。BTRAVEL 接口保持原有行为。

## 调用

完成手动 START 后：

```bash
ros2 action send_goal /auv/basic_motion uv_msgs/action/BasicMotion \
  '{cmd_type: 8, axes: xyz, target: [1.0, 0.0, 0.0, 0.0], cruise_speed: 0.15, timeout: 0.0}' --feedback
```

任务默认文件：`uv_task/config/tasks/bline.yaml`，参数 `dx/dy/dz/speed_mps/timeout`。
导引参数均放在 `uv_control/config/default.yaml` 的 `line.*`；当前为初始值，
需先仿真验收，再进行实机标定。

## 稳定到点参数

内置 `LineConfig` 与启动 YAML 使用相同的默认值。下表参数均带 `line.` 前缀；
任务 YAML 显式指定的巡航速度仍优先，bline/wline 独立任务默认仍为 0.15 m/s。

| 参数 | 默认值 | 单位 | 作用 |
|---|---:|---|---|
| default_speed | 0.30 | m/s | cruise_speed=0 时使用 |
| max_speed | 0.80 | m/s | XYZ 合速度上限 |
| acceleration | 0.30 | m/s² | 沿线指令加速度 |
| deceleration | 0.30 | m/s² | 沿线指令减速度 |
| vector_acceleration | 0.40 | m/s² | 整个速度向量变化率 |
| brake_acceleration | 0.20 | m/s² | 制动能力估计，不是实际减速度指令 |
| brake_delay | 0.30 | s | 制动响应延迟估计 |
| cross_gain | 0.60 | s⁻¹ | 横向和终点比例修正增益 |
| cross_speed | 0.18 | m/s | 横向纠偏速度上限 |
| capture_enter | 0.30 | m | 偏差超过此值进入捕获 |
| capture_exit | 0.10 | m | 偏差恢复到此值退出捕获 |
| capture_speed | 0.15 | m/s | 捕获时沿线目标速度上限 |
| reverse_speed | 0.08 | m/s | 终点超调回退速度上限 |
| terminal_speed | 0.10 | m/s | 速度跟踪终点阶段的水平速度目标上限 |
| terminal_radius | 0.35 | m | 切换位置保持停车的三维距离 |
| position_tolerance | 0.30 | m | 默认 XY 每轴到点容差 |
| z_tolerance | 0.15 | m | Z 到点及起点对准容差 |
| stop_speed | 0.03 | m/s | 到点允许的最大实测三维合速度 |
| yaw_tolerance | 3.0 | ° | 对准和到点航向容差 |
| stable_seconds | 1.20 | s | 停车和最终转向后分别连续满足条件的时间 |
| align_timeout | 30.0 | s | 起点对准阶段时限 |
| yaw_gain | 0.80 | s⁻¹ | 巡航航向比例修正增益 |
| yaw_rate_limit | 15.0 | °/s | 巡航转向指令上限 |

进入停车范围前，速度跟踪按上述减速度及向量变化率制动。进入 STOPPING
后由底层位置控制器保持固定终点并制动；长度不超过 0.35 m 的短段在起点
对准后直接交给终点位置保持，仍需满足位置、实测低速和连续稳定时间。
单次目标显式指定的 `position_tolerance` 仍优先于默认 0.30 m，
但不会推迟 0.35 m 范围内的停车模式切换。
起终点位置模式的转向由底层控制器执行，`yaw_rate_limit` 限制巡航速度模式转向，
也用于估算终点转向预算。

按 `v²/(2*brake_acceleration)+v*brake_delay`，0.8 m/s 的规划制动距离约为
1.84 m，0.3 m/s 时约为 0.315 m。制动估计和响应延迟需用水中实测数据校准；
这些数值不能作为实际停车距离保证。uv_task 近距 BLINE 的 fore_aft.speed_mps
保持严格小于 0.80 m/s，近距伺服、定深、升降专用限速不随此配置提高。

## 构建与验收

Action 字段结构不变。已有包含 cruise_speed 和
position_tolerance 的客户端接口仍兼容；旧版缺字段的类型需要先统一构建。
重新构建 uv_control 并重启节点后才会加载新的代码和默认配置。

```bash
colcon build --packages-select uv_msgs uv_control uv_task uv_bringup
source install/setup.bash
```

`test_line_guidance.py` 覆盖航向/横移/斜移/垂直、速度及参数拒绝、
捕获迟滞、制动、超调回退、速度上限与变化率。
`test_line_runtime.py` 运行生产回调和虚拟时钟/运动模型，覆盖固定终点、
HOLD 稳定等待、取消、零速度、过期反馈、原点变化、租约及并发命令。
这些是确定性运动模型检查，尚不能替代 Stonefish 动力学验收和实机标定。


BLINE 的最后一项为相对起始航向的转角（度），0 表示到点后恢复起始航向。
WLINE=9 使用同一个 action，target=[odom_x,odom_y,odom_z,yaw_deg]，
位置和最终航向均为世界系绝对值。两者移动时朝向行进方向，停止到位后
才发送最终航向，并保持终点位置；最终稳定才返回成功。
自动超时额外加入 abs(wrap(最终航向-行进航向))/line.yaw_rate_limit 秒；
显式正 timeout 仍为全程总上限。零位移且有转角的 BLINE、目标已在当前位置
的 WLINE 也支持，按 ALIGN → FINAL_TURN → HOLD 完成。
任务 YAML：bline 使用 dx/dy/dz/drz，wline 使用 x/y/z/rz。
