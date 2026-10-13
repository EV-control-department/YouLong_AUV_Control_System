# 简单转盘：YOLO box → 前向MOVE顶碰一次

任务链 `config/missions/turn_table.yaml` 已改用 `tasks/turntable_simple.yaml`，
任务名仍为 `turntable`，`mode=yolo_bump`。
前视左目框中心决定yaw/升沉微调；居中后前进，框宽达到画面宽45%时，
结束视觉速度控制，沿body+x执行一次BMOVE（当前0.08m），保持深度和航向，最后零速度结束。
没有向上撞击，也没有顶碰后的yaw动作；视觉居中阶段仍需yaw/升沉微调。
接近仅使用YOLO box，不需要掩膜、双目深度、孔位或条幅角度。
可选预输入盘心仅用于先到观察位，不代替后续视觉。
框失效时立即停止接近，持续丢失配置的target_lost_timeout_s后使用已知盘心兜底；
未填盘心则失败，不猜测目标位置。任务总时限180秒，包含可选预定位。
MOVE后用新鲜实测XY等待到位，避免BasicMotion全局10cm容差使8cm指令空完成。
实测到位仍不等于碰撞确认；无接触传感器，不验证实际触碰/盘体角度，不执行插棍或退杆。

camera和basic_motion已运行时，从仓库根目录启动：

```bash
source /opt/ros/humble/setup.bash
source workspace_auv/install/setup.bash
ros2 launch uv_task task_launch.py target_id:=turntable \
  mission_file:="$PWD/workspace_auv/src/uv_task/config/missions/turn_table.yaml"
```

修改 `turntable_simple.yaml`：

- `disk_pose_odom: []`：留空跳过预定位；填写 `[x,y,z,yaw度]`，表示START后odom系中的盘心与正视航向。
- `front_standoff_m`、`front_camera_center_body`：由名义盘心换算盘前艇体观察位；先WTRAVEL再SET归向，随后只使用移动结束后采集的YOLO框。
- `motion_timeout_s`：每次预定位动作上限，同时受整任务 `timeout` 限制；预定位失败不执行顶碰。
- `disk_class_id`：实际转盘YOLO类别。当前沿用camera配置的3，但共享模型表3标为gate_front，必须确认实际权重标签。
- `front_camera_upside_down`：当前true；box恢复到原标定像素后，反转横纵误差。正装时改false。
- `bump_box_width_fraction`：前向MOVE的接近阈值；仅代理距离，需按录像调整。
- `approach_speed_mps`：视觉接近速度；MOVE阶段速度由下位机闭环决定，不受本项限制。
- `bump_forward_m`：一次body+x MOVE行程，当前0.08m沿用旧速度×时间的名义行程，未作为实机触碰距离标定。
- `bump_move_timeout_s`：BMOVE及实测到位等待共用预算，当前10秒。
- `bump_position_tolerance_m`：独立XY到位容差，当前2cm，不修改全局WTRAVEL/MOVE。
- 旧`bump_seconds`、`stroke_yaw_deg`、`yaw_rate_deg_s`及棍子/孔位参数不参与此简单模式；旧precise模式保留。

旧孔位版本保留在 `tasks/turntable.yaml`，无mode时走precise分支；其几何标定和放行参数只用于旧版本。
