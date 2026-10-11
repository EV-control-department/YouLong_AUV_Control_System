# 简单转盘：YOLO box → 顶碰 → yaw

任务链 `config/missions/turn_table.yaml` 已改用 `tasks/turntable_simple.yaml`，
任务名仍为 `turntable`，`mode=yolo_bump`。
前视左目框中心决定yaw/升沉微调；居中后前进，框宽达到画面宽45%时，
以0.08m/s顶碰1秒，再以6°/s右转1秒，最后零速度结束。
接近仅使用YOLO box，不需要掩膜、双目深度、孔位或条幅角度。
可选预输入盘心仅用于先到观察位，不代替后续视觉。
框失效时立即停止接近，持续丢失1秒退出；任务总时限180秒，包含可选预定位。
顶碰/yaw均为定时速度指令，不验证实际触碰或转盘角度，不执行退杆。

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
- `bump_box_width_fraction`：定时顶碰的接近阈值；仅代理距离，需按录像调整。
- `approach_speed_mps`、`bump_seconds`：前进速度与单次顶碰时间。
- `stroke_yaw_deg`：正右转、负左转；实际持续时间为绝对值除以 `yaw_rate_deg_s`。

旧孔位版本保留在 `tasks/turntable.yaml`，无mode时走precise分支；其几何标定和放行参数只用于旧版本。
