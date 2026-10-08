# BLINE 定速有限直线段

`BasicMotion.Goal.BLINE=8` 使用动作开始时的实测位置、航向固定起终点。
`target=[dx,dy,dz,0]`，`axes` 为空或 `xyz`。XY 位移只按开始航向旋转，
Z 为深度变化；速度转换使用实时完整 RPY。巡航速度 `cruise_speed=0`
使用默认 0.15 m/s，否则必须严格满足 `0<speed<0.18`。
纠偏后的合成速度默认限制为 0.25 m/s。

阶段：保持起点转向（ALIGN，最多 30 秒）→ 捕获/沿线跟踪
（CAPTURE/CRUISE）→ 制动（BRAKE）→ 终点收敛/超调回退（TERMINAL）
→ 零速度后切换终点位置保持（HOLD）。XYZ 各轴误差不超过 0.10 m、
实测平移速度不超过 0.03 m/s、航向误差不超过 5°，在 HOLD 连续满足
1 秒后成功。控制频率 20 Hz；阶段切换及每秒输出进度、偏差、
固定终点、目标/实测沿线速度。

`timeout>0` 覆盖转向、跟踪和保持全过程；否则自动取
`max(60, length/cruise_speed+50)` 秒。
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

## 构建与验收

新增 action 字段改变 ROS 类型定义，所有 BasicMotion 客户端和服务端必须
使用相同的新 `uv_msgs` 并重启，包括独立启动的 task_runner 和导航节点。

```bash
colcon build --packages-select uv_msgs uv_control uv_task uv_bringup
source install/setup.bash
```

`test_line_guidance.py` 覆盖航向/横移/斜移/垂直、速度及参数拒绝、
捕获迟滞、制动、超调回退、速度上限与变化率。
`test_line_runtime.py` 运行生产回调和虚拟时钟/运动模型，覆盖固定终点、
HOLD 稳定等待、取消、零速度、过期反馈、原点变化、租约及并发命令。
这些是确定性运动模型检查，尚不能替代 Stonefish 动力学验收和实机标定。
