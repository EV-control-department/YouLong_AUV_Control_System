# 目标位置可视化 GUI

## 目的

`target_position_gui` 用于同时查看定位器的融合结果和每一条参与融合的几何观测，便于验证：

- 每个物理类别/实例（`physical_class_name/instance_id`）最终被估计在什么位置；
- 该位置由前视双目、前视多位置或下视直接观测中的哪些证据支撑；
- 单条直接三维测量与融合结果之间是否存在明显偏差；
- 多个目标是否被错误关联到同一实例。

GUI 是诊断工具，不参与定位或控制；不要在无桌面显示的机载运行环境中默认启动它。

## 启动

先启动仿真和感知定位节点：

~~~bash
source /opt/ros/jazzy/setup.bash
source /home/doc049/dev/UUV/YouLong_AUV_Control_System/workspace_auv/install/setup.bash
ros2 launch uv_bringup sim_bringup.py
~~~

在有桌面环境的另一个终端启动 GUI：

~~~bash
source /opt/ros/jazzy/setup.bash
source /home/doc049/dev/UUV/YouLong_AUV_Control_System/workspace_auv/install/setup.bash
ros2 run uv_camera target_position_gui
~~~

可选参数：

~~~bash
ros2 run uv_camera target_position_gui --ros-args \
  -p ray_display_length_m:=6.0 \
  -p observation_table_limit:=800 \
  -p max_plot_observations:=300
~~~

为避免 Tk 在高频重建数百条射线时卡顿，GUI 默认每 250 ms 刷新一次，表格显示最近 200 条观测，画布显示最近 160 条观测。定位器发布的全部历史仍保留在 GUI 内存中，且可通过上面的两个参数分别调大；不建议在普通桌面环境把画布上限设为 300 以上。

## 读取的话题

| 话题 | 用途 |
|---|---|
| `/perception/target_positions` | 每个静态目标的融合位置、协方差、融合观测来源、状态和年龄 |
| `/perception/target_observations` | 定位器保留的最近有效几何观测，默认 500 条 |

第二个话题由 `object_localizer` 发布。默认保留 500 条；如需更长回放历史，可在启动文件的 `object_localizer` 节点参数中设置 `observation_history_size`。

## 界面含义

左侧默认是 `odom` 坐标系中的 N-E 俯视平面（项目约定仍为 X=North、Y=East；单位米），但显示方向固定为**上方 North、右方 East**，以与赛场俯视图一致；可在顶部切换为 X-Z 或 Y-Z 侧视图，以检查 Z=Down。当前 `object_localizer` 只使用下视，因此正常运行时主要显示绿色下视观测点；蓝色/橙色元素仅用于兼容旧的历史消息：

N-E 俯视图支持交互浏览：将光标放在目标附近滚动鼠标滚轮可围绕该位置缩放；按住鼠标左键拖拽可平移视图。拖拽或缩放后，新到达的定位消息不会重置当前视野；点击“重置 N-E 视图”才恢复为根据全部数据自动取景。X-Z、Y-Z 侧视图仍保持自动取景。

- 黑边十字：某个 `class_id/instance_id` 的融合位置；虚线椭圆是 X-Y 平面约 2σ 不确定度范围；
- 蓝色点：旧版本 `FRONT_STEREO` 前视双目产生的单次三维测量点；当前节点不会产生；
- 绿色点：`DOWN_DIRECT` 下视直接产生的单次三维测量点，也是当前节点的有效观测；
- 橙色虚线箭头：旧版本 `FRONT_MULTI_VIEW` 的单次前视观测射线；当前节点不会产生。

旧版本的前视多位置单帧没有唯一三维位置，只有“从当时相机光心向哪个方向看”的约束，因此 GUI 刻意画射线；当前版本不再接收这类前视射线。

右上表展示所有融合目标；第一列是物理类别/实例，检测类别列显示代表原始标签及本轨迹已经接受的 YOLO ID。`已融合观测` 是位掩码组合，前视双目=1、前视多视角=2、下视直接=4。右下表逐条列出观测序号、物理类别/实例、该帧的原始检测类别、观测形式、测量点或射线原点、置信度与时间。点击任意目标或观测行会高亮对应实例；勾选“仅显示选中目标”可以排查错误关联。

定位器的聚类数量先验为：引导线 K=6、门 K=4、其余物品每类 K=1。当前所有检测只来自下视；`*_down` 后缀会归一到物理类别，观测历史中的 `class_id` 则始终是该帧检测模型实际输出的 ID。观测先进入类别池，再按二维 N/E 聚类；每个实例最多保留 50 条下视卡尔曼队列，类别观测池默认保留 2000 条。Z/Down 不参与实例空间聚类，因此深度异常不会单独制造第二个实例。

“清空本地显示历史”只清空 GUI 当前显示；定位器端的历史与融合状态不会被修改。下一次定位器发布后，历史仍会重新显示。
