# 转盘任务：实现与真机调试

此任务只注册为独立调试任务，**未加入仿真 mission**。目前没有可转动、可与细棍接触的 Stonefish 模型；以下是代码级实现与无接触验证方案，不代表已在真机上完成推盘。

## 一、修改前如何运行，以及为什么不适用

旧链路是 `uv_sensor` 采集前视左右图 → 进程内 YOLO-Seg 输出盘体和“黄色标签”两个掩膜 → `turntable_vision.py` 拟合盘体椭圆，用黄标质心计算图像角度 → DDS `/perception/turntable/observation` 小型 JSON → `TurntableTask` 订阅观测和 `/basic_motion/pose_info` → **用任务文件里手填的盘心、盘轴、棍端**反算 `WTRAVEL` 接近位置 → 每程 2 cm 以下 `BMOVE x` 插杆、3° 以下 `BMOVE rz` 推盘、完全退出、盘外反向复位，共三程。每个动作后会等待新的观测和实测里程计。默认 `allow_contact_motion=false`、`force_limited_control_confirmed=false`。

旧视觉既不测盘心距离，也不修正盘轴；`last.pt` 真机模型只有 `wheel=3`，并没有黄标 YOLO 类别。因此旧实现的视觉节点无法按真实模型启用。相机画面的圆盘外形最多确定盘面是否正视，**不能**从四重对称的轮廓求出唯一绝对转角；长宽比也不等于黄标相位。

## 二、现在的信息流和几何

`uv_camera` 的 YOLO 路径接收前视拼接原图，在进程内切成左右目。前视检测得到转盘整盘掩膜（真机 class ID `3`）；独立的 latest-frame worker 执行下面的测量，不在 DDS 上发布原始图像：

1. 用整盘多边形拟合椭圆，给出像素中心、长短轴比和检测置信度。长短轴比用于正视程度门控，不能单独判断法向符号。
2. 在整盘掩膜内做 HSV 黄色检测，作为相位来源；也兼容未来独立黄标 YOLO 类别。黄标缺失时 `phase_valid=false`，仍可发布几何，但**不得插杆**。掩膜外黄色不会参与；盘体内其他黄色区域仍可能误检，需通过录像确认。
3. 使用前视双目真实标定对左右图去畸、极线校正、SGBM。深度候选必须同时落在整盘掩膜、阈值化黑色区域和避开中心透孔及外缘的环带内；有效视差、测距范围、点数和平面残差均要通过门控。SGBM 点云拟合盘面，盘心像素射线与该平面相交得到左目 optical 坐标系里的盘心。根据盘面法向求实际盘轴方向，同时用 23 cm 外径验证检测目标。
4. `uv_camera` 按采集时间匹配 `/basic_motion/pose_info`，应用前视左目 Body 外参，得到 `disk_center_world`、`disk_axis_yaw_deg`、深度点数、拟合残差、估计直径等小型观测，发至 `/perception/turntable/observation`。`uv_task` **不订阅图像、不运行 YOLO 或 SGBM**。

坐标约定：Body `x` 前、`y` 右、`z` 下；前视 optical `z` 前、`x` 右、`y` 下。真机双目中点相对 Body 的名义位置为 `(0.230,0,0.076)m`；左目为 `(0.230,-0.050,0.076)m`。本任务把用户提供的棍根 `(0,-0.09,0)m` 解释为**相对双目中点、沿 Body 轴**的偏移，棍尖沿 Body `+x` 再伸出 `0.16m`，因此名义棍尖 Body 坐标为 `(0.390,-0.090,0.076)m`。如果机械图纸里的“前视摄像头”指左目镜头，或者坐标在 optical 轴下，则这个换算不成立；必须重新测量并修改 `front_camera_center_*`，不能直接用这组值接触。盘轴世界航向和盘心均来自视觉，任务文件里的名义位置只可用于手动搜索。

`TurntableTask` 先验证观测采集戳和里程计新鲜度，再用视觉盘心/法向计算距盘约 0.55m 的**非接触正视对准位**；到位后重测，长短轴比须不小于 0.88、盘面拟合残差不大于 2cm、盘轴航向与机体航向差不大于 15°。黄标相位加现场标定的 `label_to_hole_deg` 才能选择四个孔位中的顶部或底部孔；计算棍尖的预插入目标，`WTRAVEL` 接近并再次视觉对孔，实际棍尖与目标孔误差大于 2.5cm 就停止。推盘期间比较**世界系**盘心，若移动大于 `max_disk_world_shift_m` 则停止，避免把机器人运动造成的像素位移误当作盘体移动。每程退出后须观测到同向黄标变化，下一程重新测盘心、盘轴和相位，再盘外对孔。

转盘几何按当前提供的尺寸：最外圆直径 230 mm（半径 115 mm），外环内沿直径 200 mm（半径 100 mm），中心内圈外沿直径 35 mm（半径 17.5 mm），四根条幅各宽 20 mm。可插入孔的径向范围仅为 17.5–100 mm；`contact_radius_m=0.05875` 是两边界中点的**候选值**，不是已验证的接触位置。代码检查静态径向净空，以及假定孔位在相邻 90° 条幅正中时的切向净空；尚不模拟整个 yaw 扫掠、条幅真实厚度或盘体变形。棍半径、黄标到孔位的相位仍未知。盘面点云法向受低纹理和折射影响，离线几何检查通过不代表可自动接触；当前 `BasicMotion` 接口没有接触力控制，现场应具备限推力、急停及人工退杆方案。

## 三、参数与放行

任务模板：[turntable.yaml](../src/uv_task/config/tasks/turntable.yaml)；逐项验收记录：[turntable_calibration_checklist.md](turntable_calibration_checklist.md)。已写入 `disk_diameter_m=0.230`、`inner_radius_m=0.0175`、`outer_radius_m=0.100`、`spoke_width_m=0.020`，以及候选 `contact_radius_m=0.05875`；棍根相机偏移和杆长仍沿用已有值。相机中点 Body 位置是 PDF 名义值，仍需装配复核。`rod_radius_m`、`label_to_hole_deg`、`image_angle_to_disk_sign`、`approach_standoff_m`、`insert_depth_m`、`drive_yaw_sign` 均须现场测量或标定。接触半径必须满足 `inner < contact < outer < 0.115m`，径向和静态条幅净空还须各留下棍半径与 1cm 余量。目标累计角度/绝对角度目前未确定，代码仅验证三次同向短行程，不能声称满足最终比赛角度。

相机关键参数：`enable_turntable_vision=true`、`turntable_disk_class_id=3`、`turntable_label_class_id=-1`（不需要黄标 YOLO 类）、`turntable_calibration_file`、`turntable_image_size=[640,480]`（**单目**；拼接图为 1280×480）、`turntable_calibration_native_size=[1280,960]`、`turntable_black_threshold=95`、`turntable_min_depth_points=30`、左右目 Body 平移及 optical→Body 旋转。真机默认 `config/front.npz`，运行时把原始每目 1280×960 标定等比换算为每目 640×480；若驱动采用裁剪或改变视场，需重新标定并填写实际原始标定尺寸。代码会拒绝工作尺寸不符的图像。仿真模式采用 `/sim/front_cam/{left,right}/camera_info`，但转盘接触任务不在仿真中执行。

## 四、分阶段调试命令

以下命令均从仓库根目录执行。先停止其他占用 `/dev/video*` 的相机节点。第一阶段只启动硬件、控制和本任务所需的前视相机，**不自动启动任务**：

```bash
cd /home/laurie/AUV_2026_Robocup/YouLong_AUV_Control_System
source /opt/ros/humble/setup.bash
cd workspace_auv
colcon build --symlink-install --packages-select uv_camera uv_task
source install/setup.bash
ros2 launch uv_bringup real.launch.py profile:=real_default enable_ai:=false enable_nav:=false enable_task:=false
```

新终端同样 source 后，启用独立的前视检测/转盘测量。设置真实模型的映射表，避免把旧比赛类表解释为当前 `last.pt`：

```bash
cd /home/laurie/AUV_2026_Robocup/YouLong_AUV_Control_System/workspace_auv
source /opt/ros/humble/setup.bash
source install/setup.bash
export UV_MODEL_MAPPING_FILE="$PWD/src/uv_camera/weights/real_last.yaml"
ros2 run uv_camera uv_camera --ros-args \
  --params-file "$PWD/src/uv_camera/config/profiles/real_default.yaml" \
  -p sim_mode:=false -p enable_ai:=true -p device:=cuda:0 \
  -p enable_down_camera:=false -p enable_mapping_vision:=false \
  -p enable_turntable_vision:=true -p turntable_disk_class_id:=3 \
  -p turntable_label_class_id:=-1 \
  -p turntable_calibration_file:="$PWD/src/uv_camera/config/front.npz"
```

检查 `ros2 topic echo /perception/turntable/observation`。期望 `valid=true`、采集戳持续更新、`depth_points≥30`、估计直径接近 0.230m、静止时 `disk_center_world` 波动可接受；有黄色标记时还应有 `phase_valid=true`、`phase_source=hsv`。分别从左/右偏角拍摄，确认 `axis_ratio` 随偏角变小、法向航向变化方向正确。在**未经过实物复核前**，不要仅凭一次 `valid=true` 开接触。

第三个终端启动调试任务节点。`/task/exec` 只在 debug 模式开放，不会自动加载 YAML；需把已经测量的参数作为 JSON 传给服务。首次保持 `allow_contact_motion=false`，且建议先不要发执行服务，只观察测量话题和手动操控下的世界盘心稳定性。

```bash
ros2 run uv_task task_runner --ros-args -p debug_mode:=true
ros2 topic echo /basic_motion/pose_info
ros2 topic echo /perception/turntable/observation
ros2 service call /task/exec uv_msgs/srv/ExecTask \
  '{task_name: turntable, params_json: "{}", timeout: 0.0}'
```

上述 `{}` **应安全失败**，用来验证任务注册与禁动路径；要验证完整的只读几何检查，请把清单里的必填值填进 `params_json`，但继续令两个放行开关为 `false`。执行 START 建立里程计原点后再做世界坐标验证。人工测量四辐条净空和黄标相位、检验空载正视对准位、急停和限推力，最后才考虑分阶段将两个开关置 `true`。现场接触调试需有人监护，先测试一次最小 yaw 行程，再增加到三次。

常见拒绝原因：`missing_disk_mask`＝模型未检出整盘；`black ... 视差仅 N 点`＝黑色盘面缺纹理或 SGBM 阈值不适；`视觉直径 ... 不符合 23cm`＝标定/检测或错误目标；`采集时间附近无新鲜位姿`＝时钟/同步延迟；`yellow_marker_not_found`＝HSV 未见黄标，几何仍可用但不得插杆；`盘面尚未正视`＝先手动或非接触调整；`盘心世界坐标位移 ... 超限`＝可能拉动转盘或双目抖动，应停止并复核。调 `turntable_black_threshold` 时必须检查黑色阈值化是否落在真实盘面，不得为了通过门控任意降低最少点数。
