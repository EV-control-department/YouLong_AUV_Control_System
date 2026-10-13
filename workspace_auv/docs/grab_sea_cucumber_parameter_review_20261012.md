# 抓海参参数复核：2026-10-12

对应正式任务config/tasks/grab_sea_cucumber.yaml及完整mapping→转盘→海参→返回任务树。
收集框新流程：运输到观察位→同对双目bbox中心伺服→独立投放偏置→实测到位→固定稳定等待→释放。
抓取与计数继续使用左目mask。收集框不做三角测距、点集包含、边缘余量、爪子投影或框心地标校正。

| 参数 | 结论与处理 |
| --- | --- |
| collection_release_margin_m | 已删除；新流程没有内部区域/边缘距离判断。 |
| collection_correct_odom_xy | 已删除；不再根据收集框修改odom。 |
| collection_center_odom_xy | 已删除；无需预输入框心地标坐标，delivery.pose仍可预输入观察/兜底位姿。 |
| collection_camera_body_xy | 已删除；原来仅用于框心反投影。 |
| descent.max_press_distance_m | 已删除且不再必填；旧实现没有限幅，不能当作触底保护。 |
| ascent_step_m / ascent_step_timeout_seconds / ascent_pause_seconds | 旧步进上浮遗留，不在当前YAML；旧配置读取时告警并忽略。 |
| collection_projection_depth_m / servo.projection_depth_m | 保留；分别是收集框/海参像素误差的步长比例，不是实际测距，投放面与池底可不同。 |
| collection_pixel_tolerance_fraction / vision.pixel_tolerance_fraction | 保留；分别控制双目框中心/左目海参居中误差。实际含义是归一化像素误差，非图像宽高比例。 |
| collection_hold_seconds / servo.hold_seconds | 保留；分别控制收集框/海参持续视觉居中，默认配置均0.5秒。 |
| collection_release_offset_body_xy / gripper.offset_x_m、offset_y_m | 保留独立定义；前者投放body[+0.30,0]，后者昨日调试抓取[+0.20,+0.04]。投放只执行前者一次。 |
| gripper.settle_seconds | 共用4秒固定等待：海参抓取补偿后、收集框投放偏置后。之前必须通过独立实测到位判定。 |
| servo.motion_settle_seconds | 保留0.3秒实测XYZ/yaw连续到位时间，区别于固定等待及视觉持续居中。 |
| gripper.servo2.*、gripper.close_wait_seconds | 当前servo_id=1时不启用，已标记；保留用于切换舵机2，不应按冗余删除。 |
| gripper.pickup_angle_deg / release_angle_deg | 2026-10-13更新：舵机1抓180°、放80°；START发送一次180°。与旧0/90度配置冲突时初始化拒绝。 |
| ascent.min_speed_mps | 当前0.003在tolerance=0.05、slowdown_gain=0.20下不会触发：退出前剩余深度乘gain>0.01。保留供以后收紧深度容差。 |
| bottom_reference_offset_z_m / gripper.contact_body_xyz | 非重复；取艇底参考点与姿态旋转后爪子最低点的较大下伸量。水平时0.29大于0.10。 |
| near_floor_open_loop及floor_z/clearance/press/lift参数 | 当前启用，均实际使用；descent.duration_seconds限制总下压时间，press_thrust_seconds限制近底推力时间。 |
| restore_pre_press_odom_xy / odom_correction_max_m | 保留近底B锚点校正；仍假定压抓中无真实水平漂移。去掉收集框地标校正不等于去掉近底校正。 |
| 各层timeout | 保留；整项900秒，单轮视觉60秒，单次动作30秒，运输/搜索180秒，计数20秒，上浮回观察位360秒，不是重复参数；有效预算受总预算截断。 |

旧参数仍在config_loader兼容schema内，因此旧任务文件不会仅因存在旧键而无法读取，
GrabSeaCucumberTask初始化会明确告警并忽略；新YAML不再提供这些键。
继承抓球基类的verification_*、max_grab_retries只被基类初始化，海参自有计数与重试流程不使用；
当前海参schema/YAML没有提供这些键。无需在海参参数表补充它们。

## 现场需要确认

- collection_class_id必须与实际权重一致；两目各只出现一个可信框，无配对目标时不进行伺服。
- 下视安装yaw=180度沿用原标定坐标；不把监控显示旋转再次应用到输入像素。
- 左右实际K分别归一化，平均中心误差代表双目中间方向，区别于原单左目对准。
  投放偏置保留现场指定的body+x 30cm，不能以软件平均中心证明爪子几何已标定。
- bbox允许裁切，但可见中心会漂移。本版按用户要求简化，不保证落点在框内。
- 视觉失败回delivery.pose兜底后，同样偏置、稳定、释放；若任务/偏置失败或稳定等待未完成则不释放。

修改部署需构建uv_task并重启task_runner；软件测试不等于水下投放验证。
