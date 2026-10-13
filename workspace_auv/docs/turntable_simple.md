# 简单转盘：视觉对准 → 直接前向撞击一次

任务链 `config/missions/turn_table.yaml` 已改用 `tasks/turntable_simple.yaml`，
任务名仍为 `turntable`，`mode=yolo_bump`。
参考仓库 `origin/robotcup` 的撞球任务及 `_charge_forward` 速度刷新方式：
前视左目YOLO框中心决定yaw/升沉微调，对准期间不前进；横纵误差都进入容差后，
立即结束视觉修正，再以配置的body+x速度前冲一段时间，最后发送零速度。
不再要求框宽达到45%，不再调用短行程BMOVE，也不再等实测XY进入2cm容差。
没有向上撞击，也没有撞击后的yaw或自动返回动作；对准阶段仍需yaw/升沉微调。
只使用YOLO box，不需要掩膜、双目深度、孔位或条幅角度。
可选预输入盘心仅用于先到观察位，不代替后续视觉。
对准阶段框失效时立即停止修正，持续丢失配置的target_lost_timeout_s后使用已知盘心兜底；
视觉对准超过approach_timeout_s（当前25秒）也使用同一盘心兜底，不再直接退出。
未填盘心则失败，不猜测目标位置。任务总时限180秒，包含可选预定位。
进入前冲后不再因近距离YOLO目标出画而中断或重启对准，但停止指令/总超时仍立即生效。
前冲命令是速度闭环，不是开环推力；z/yaw发零速度，不是绝对深度/航向位置闭环。
成功仅表示本次定时冲撞指令执行完成；无接触传感器，不验证真实触碰/盘体角度。
现场应从已确认的盘前观察位调试速度和持续时间，名义行程不等于实测行程。

camera、basic_motion等基础节点已运行时，真机从工作区启动（本机Humble用对应ROS环境）：

```bash
cd /home/nvidia/YouLong_AUV_Control_System/workspace_auv
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 run uv_task task_runner --ros-args \
  -p mission_file:="$PWD/src/uv_task/config/missions/turn_table.yaml"
```

修改 `turntable_simple.yaml`：

- `disk_pose_odom: []`：留空跳过预定位；填写 `[x,y,z,yaw度]`，表示START后odom系中的盘心与正视航向。
- `front_standoff_m`、`front_camera_center_body`：由名义盘心换算盘前艇体观察位；先WTRAVEL再SET归向，随后只使用移动结束后采集的YOLO框。
- `motion_timeout_s`：每次预定位动作上限，同时受整任务 `timeout` 限制；预定位失败不执行顶碰。
- `disk_class_id`：实际转盘YOLO类别，当前具身智能模型wheel=3；换模型时必须核对。
- `front_camera_upside_down`：当前true；box恢复到原标定像素后，反转横纵误差。正装时改false。
- `center_tolerance_fraction`：横纵框心误差容差，当前0.1，按单目640×480归一化。
- `approach_timeout_s`：保留旧参数名，但现在只限制原地视觉对准，不含前进接近。
- `charge_speed_mps`：前冲body+x速度，当前0.15m/s。
- `charge_duration`：持续时间，当前5秒；两项乘积名义0.75m，需要现场调整。
- `charge_publish_period`：刷新周期，当前0.05秒（20Hz）。
- 以上速度/时间参考robotcup代码默认值；没有照搬撞球YAML中的5m/s，也未进行真机试撞。
- 旧`bump_*`、`approach_speed_mps`、`stroke_yaw_deg`、`yaw_rate_deg_s`及棍子/孔位参数不参与此简单模式；旧precise模式保留。

旧孔位版本保留在 `tasks/turntable.yaml`，无mode时走precise分支；其几何标定和放行参数只用于旧版本。
