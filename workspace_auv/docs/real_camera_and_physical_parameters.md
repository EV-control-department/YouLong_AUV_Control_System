# 真机视觉与物理参数核对

## 当前真机配置

- `real.launch.py` 选用 `uv_camera/resource/last.pt`，类别依次为 `square_cone=0`、`round_cone=1`、`trepang=2`、`wheel=3`、`platform=4`。启动时使用 `weights/real_last.yaml`，并核对权重中的类别名；不匹配则停用检测。仿真仍使用原模型配置。
- 前视、下视 V4L2 **整幅拼接画面均固定为 1280×480**，左/右目各 640×480。任一相机没有输出该尺寸，启动预检会报错；采集中尺寸变化也会停止使用该流。应确认驱动输出双目左右拼接，而不是单目缩放图。
- `docs/stereo_parameters.json` 是下视原始标定的权威来源，原始每目为 1280×960。运行时两个方向等比缩小 0.5：K 的焦距、主点、斜切项减半，畸变系数不变；`Stereo.T` 从毫米转换为米。下视建图和目标定位共用这一逻辑，随包提供 `uv_camera/config/down_real.json` 副本用于脱离源码部署。
- 缩放后左目 `fx=579.550, fy=578.955, cx=339.560, cy=255.336`；右目 `fx=575.799, fy=574.994, cx=347.496, cy=274.735`（像素，四舍五入）。有效双目基线约 `0.06117 m`。左右目不得共用一个 K/D。
- 前视 `config/front.npz` 保留原始每目 1280×960 标定；前视定位与转盘识别在运行时将左右目 K/P 的像素行各缩小 0.5，Q 按像素/视差变换，畸变系数和物理基线不变。单目工作尺寸为 640×480，左目约 `fx=570.319, fy=570.323, cx=295.431, cy=238.544`。原始 `front.npz` 的 T 与 P2 基线不一致，现有定位使用 P1/P2 的基线；必须用实物测距与双目顺序验证，不能仅因缩放正确就视为完成前视标定。

## 第五版 PDF 中可直接引用的机体系名义值

坐标采用 PDF 的 Body-FRD（前/右/下为正），位置单位米。以下是机械参考点，不等同于运动目标的世界坐标。

| 参考点 | 机体系位置 `[x,y,z]` | 用途与限制 |
| --- | --- | --- |
| 下视相机参考点 Cd | `[-0.130, 0, 0.0645]` | 建图外参中心；左右目安装位置按约 61 mm 基线对称暂估，需现场核验左右顺序与实际安装。 |
| 前视相机参考点 Cf | `[0.230, 0, 0.076]` | 前视定位外参；朝向采用 PDF 名义矩阵，仍需实机外参标定。 |
| INS 测量原点 I | `[0.030, 0, 0]` | 仅杆臂位置已知；INS 安装姿态尚未标定。 |
| 圆盘爪 G1 | `[-0.430, 0, 0.290]` | 海参抓取参考点。由 Cd 到 G1 的机体系平移是 `[-0.300, 0, 0.2255]`，但相机左目与爪子有效接触点仍需测量，不能直接作为抓取步进命令。 |
| 发夹爪 G2 | `[0.080, 0, 0.130]` | 仅参考点；有效接触点和舵机零位未给出。 |

真机 `real_default/real_safe` 相机配置使用上述名义中心、PDF 的名义朝向以及按基线对称展开的左右目位置。请先在干燥台架上核对左目实际对应 `Camera1`、右目对应 `Camera2`；若接线顺序相反，不要仅靠调换类别修补，应同步修改左右标定和外参。

## 仍需现场填写/核验

- 建图任务的九宫格中心、朝向、池底深度、AprilTag 坐标与 ID、巡检深度以及默认地图格点，都不是该 PDF 的内容；`uv_task/config/tasks/mapping_grid.json` 中的仿真示例值不得直接用于真机自主航行。
- 海参抓取须填圆盘爪**实际接触点**相对左下视相机的偏移、下压速度/行程、舵机角度、投放区位姿。`grab_sea_cucumber.py` 使用此 JSON 缩放后的左目 K，图像宽高须填 `640`、`480`；模型类别为 `2`。爪子机械参考点不能代替实际抓取偏移。
- 转盘尺寸：最外圆直径 230mm、外环内沿直径 200mm、中心内圈外沿直径 35mm、四根条幅各宽 20mm；对应半径 0.115/0.100/0.0175m。细棍根相对前视双目中点为 `(0,-0.09,0)m`，沿机体 `+x` 延伸 0.16m。双目中点的 Body 名义位置需装配复核；任务只要求现场填写棍半径、盘外距离和插入深度。当前程序固定推盘三次，不测量实际转角或目标累计角。`last.pt` 只有整盘 `wheel` 类、没有黄色标签类；转盘视觉用 HSV 提取黄标并用前视 SGBM 测盘心/法向，实物测试前不得认为接触几何已验证。详见 [turntable_task.md](turntable_task.md)。
- 前视内参不在 `stereo_parameters.json` 中，使用独立的 `front.npz`；等比缩放只适用于采集图像确实由原标定视场缩小而来。若相机驱动裁剪、改变视场、去畸或重新对齐，必须重新标定。第五版 PDF 的相机旋转为名义安装值，不能替代最终实测外参。

## 启动与校验

```bash
cd /home/laurie/AUV_2026_Robocup/YouLong_AUV_Control_System
source /opt/ros/humble/setup.bash
cd workspace_auv
colcon build --symlink-install --packages-select uv_camera uv_task uv_bringup
source install/setup.bash
# 真机 launch 会自动设置类别映射；先不启用任务，确认采集尺寸及检测类别。
ros2 launch uv_bringup real.launch.py profile:=real_default enable_task:=false
```

单独运行节点时，需要在启动 `uv_camera`、`object_localizer`、`task_runner` **之前**给三者设置同一个映射：

```bash
export UV_MODEL_MAPPING_FILE="$PWD/src/uv_camera/weights/real_last.yaml"
```

`last.pt` 当前被仓库 `*.pt` 忽略规则排除；部署到其他机器时，必须另行把权重放到 `workspace_auv/src/uv_camera/resource/last.pt` 再构建，不能以源码分支已有该文件为前提。
# 下视倒装的方向修正

海参与收集框水平伺服现在在每次进入对准时锁定实测深度z和航向yaw，
先发送一次`SET axes=xyzrz`进入位置保持，随后所有XY修正和夹爪偏置沿用同一z/yaw。
旧`axes=xy`会在每次调用时把当时的z/yaw漂移当成新目标；现在不再重新锁存。
这里保持的是进入对准时的实测深度，不是将艇体强拉回配置中的扫描/投放深度。
近底下压/离底仍按开环推力或速度模式执行，并不因水平伺服修改而增加近底位置闭环。

方向核对必须使用camera还原后的标定左目像素，不要按已旋转的监控画面再次反号。
下视安装yaw=180°、机器人yaw=0°时：原始像素向右偏移对应机体-y修正，
原始像素向下偏移对应机体+x修正；机器人yaw改变后再旋转到odom系。
图像竖向偏移不是机体深度z偏移，`collection_projection_depth_m`也只是投影距离。
日志同时给出像素误差、机体/世界步长，以及固定的z/yaw目标，便于区别方向问题与保持问题。

海参/收集框伺服每轮只在首次选择时按置信度选目标；随后按照最近匹配锁定同一目标，
并用实际XY位移及yaw预测新像素位置。丢失时保持当前位置目标，短暂等待原目标，
超过等待时间结束本轮对准；下一轮才重新选择，不在本轮立即跳抓别的海参。
这属于几何邻近匹配，不是永久身份跟踪；目标互相重叠或DVL漂移时仍可能关联错误。

每次微调顺序：`SET xyzrz` → 独立检查实测位姿 → 连续稳定 → 等待稳定后采集的新帧。
BasicMotion原10cm容差及WTRAVEL均不修改。任务侧默认XY容差1cm，
小步同时要求至少移动半个步长；z容差5cm、yaw容差5°，连续稳定0.3秒。
同一采集时间戳只处理一次，不以动作返回成功代替实际位移确认。
调试参数在`config/tasks/grab_sea_cucumber.yaml`的`servo`段：
`target_match_radius_px`（70px）、`target_lost_wait_seconds`（2秒）、
`position_tolerance_m`（1cm）、`motion_settle_seconds`（0.3秒）。
到位等待受单次`position_command_timeout`和本轮`timeout`共同约束；
参数不是硬件精度保证，实测无法稳定到1cm时应据反馈调整，不应仅增大步长。

收集框采用独立的`collection_pixel_tolerance_fraction: 0.08`和
`collection_hold_seconds: 0.0`：稳定动作后的新采集帧只要满足
`|du|、|dv|≤0.08`即可通过视觉居中，不额外等待0.5秒；海参抓取门限不变。
例如参考投影距离0.8m时，这相当于各投影方向约6.4cm的居中范围，
并非真实误差或投放成功的保证。通过后仍执行启用的地标校正、夹爪偏置和释放。
重复帧/等待新帧不会清除居中保持，也不会被反复算作目标丢失；
只有新的不匹配观测才能触发丢失判定。失败日志会区分视觉阶段与地标校正阶段。

收集框对准失败后采用配置点投放兜底：重新前往`delivery.pose`、恢复配置航向，
到位后直接释放，再按原流程返回搜索点。不在视觉修正后的未知位置直接释放，
兜底不再追加夹爪偏置，也不要求视觉确认。`delivery.pose`应填写可直接释放的艇体位姿。
视觉阶段预留剩余可用时间的一半（不超过投放移动超时）及释放等待时间用于兜底。
兜底移动失败、外部停止或总任务超时不释放；抓前抓后数量减少仍不等于投放成功。

真机 `real_default.yaml` 启用 `down_rotate_180: true`；也可在启动命令中显式传入
`-p down_rotate_180:=true`，关闭则设为 `false`。仿真忽略该开关。

左右目分别旋转 180°，不整体旋转拼接图，以免交换左右目。YOLO 使用旋转后的
正向图；输出检测框和掩膜恢复到原始标定像素坐标后再发布，交给 SGBM 和建图。
raw/annotated 推流同步旋转，位姿文字在旋转后叠加；前视不变。
相机内部原图、原图数据集和建图诊断图仍保留标定方向，时间戳与配对 ID 不变。
因此现有 K/D/R/T 不需修改，下游不能再次旋转检测坐标。

已确认下视双目整体物理倒装，`real_default` 和 `real_safe` 另同步修正外参：
`R_new = R_old @ diag(-1,-1,1)`，相机中心暂保持 Cd 不变，Camera1/Camera2
名义平移分别改为 `[-0.130,+0.0305,0.0645]` 和 `[-0.130,-0.0305,0.0645]`。
这是绕光轴180°且中心不变的安装模型，不是实测外参；仍需用已知位置目标核验。
原始左右图不交换，因此 stereo_parameters.json 中 K/D/R/T 保持不变。
`down_rotate_180` 只控制YOLO/推流方向，不能撤销配置中独立的物理倒装外参。
当前未发现前视软件翻转代码，
若前视已正向应保持不变，不重复翻转。

## 正式海参任务与舵机接口（2026-10-07）

已将工作区外 `trepang` 的抓取区定位和新鲜帧筛选并入正式
`uv_task/grab_sea_cucumber.py`，任务名为 `grab_sea_cucumber`。
新增独立配置 `config/tasks/grab_sea_cucumber.yaml` 及任务链
`config/missions/grab_sea_cucumber.yaml`，不替换默认任务链，也不接入仿真。

流程：START 初始化 → 前往 `search.pose` → 新鲜海参分割帧对齐 → 抓前保守计数
→ 夹爪偏移补偿 → 连续速度下压 → 低速连续上浮回观察位 → 抓后保守计数。
数量不变则重复抓取（受 `max_failed_attempts`、总超时限制）；减少则运往
`delivery.pose` 放下，未累计减少5只时返回抓取区继续。视觉数量减少不是实际投放成功的证明。
上浮直接发送body-z速度，默认上限1cm/s，接近目标减速，到位发零速度，不发向下纠偏。
该方式绕过位置环而不是速度环；MCU仍可能制动或反向出力，严格限推力需固件支持。

舵机唯一出口为 `/zit6/cmd/servo`，类型 **`zit6_interfaces/msg/ZitServo`**：
`servo_id=1`，`angle=0.0` 是抓取，`angle=90.0` 是90°释放（当前真机rqt实测角度制）。
START 的跳过准备、回车和正常准备路径均发送舵机1置零。
任务YAML采用 `pickup_angle_deg: 0`、`release_angle_deg: 90`；最终消息同样使用度。
旧任务内部接口仍使用弧度，统一出口转换为度；关闭旧的Float32发布器。发布成功不等于机械到位，
`ZitServoState` 也只是UART接受的目标角度，并非实测舵机位置。

现场先核对海参类别2、每目640×480，填写 `search.pose`、`delivery.pose`、
夹爪相机偏移和下压参数。YAML 中占位数值能够通过校验，并不表示路径安全。
两个位姿均为START后的odom绝对位置 `[x,y,z,yaw_deg]`，不是艇体增量；WTRAVEL
到达航向遵循现有运动实现，不能假定严格等于配置yaw。总超时默认600秒。

复制代码、确认参数及现场安全后，真机构建和启动方式：

```bash
cd /home/nvidia/YouLong_AUV_Control_System/workspace_auv
source /opt/ros/foxy/setup.bash
colcon build --packages-select zit6_interfaces uv_task --symlink-install
source install/setup.bash
# 需已启动相机AI、新版BasicMotion；不要同时启动多个解锁心跳发布器。
# 新版START会调用MCU setorigin，不必提前手动设置；导航必须有效。
# 以下命令会自动执行START、移动和抓放，不是只读测试。
ros2 run uv_task task_runner --ros-args \
  -p mission_file:="$PWD/src/uv_task/config/missions/grab_sea_cucumber.yaml"
```

### 近底开环抓取与收集框视觉投放（2026-10-10）

参数仍在 `src/uv_task/config/tasks/grab_sea_cucumber.yaml`，未改任务链。
流程：扫描海参 → 居中/计数/夹爪补偿 → 记录下压前XY → 下压 → 离底上浮
→ 水平回到原观察点 → 复检 → 前往投放区 → 类别4收集框视觉居中
→ 可选坐标校正 → 夹爪偏置补偿 → 90°释放 → 返回扫描点。

- `near_floor_open_loop` 默认 **false**，待标定后开启。关闭时仍为旧速度下压。
  必填 `floor_z_m`（START后池底z）、`press_thrust`（正向下压）、`lift_thrust`（小负值离底）。
  推力单位按本仓库固件为归一化 `[-1,1]`，不是 m/s；开环不保证恒速，需现场观察。
- 离地高度 = `floor_z_m - 实测机器人z - bottom_reference_offset_z_m`。
  最后一项是机体原点到DVL探头/夹爪参考点的向下偏置；0表示以机体原点计算。
  依赖位姿z正确且新鲜，不是独立测高传感器；若z也跳变，先修深度来源，不能靠XY校正解决。
- 高度≤`open_loop_clearance_m: 0.20`后锁定ACTUATOR绝对机体系指令 `control_key=0x12`，
  六轴全部写入，只有z非零，绕开位置/速度PID，不因DVL跳回而恢复闭环。
  持续 `press_thrust_seconds`，受下压总时间和任务总时间约束；退出发ACTUATOR零推力。
  近底上浮也使用 `lift_thrust`，高于20cm+3cm后清零推力再恢复原慢速速度上浮。
  纯开环期间不保持XY、yaw、roll、pitch；保持原心跳/固件解锁保护，不绕过固件失联保护。
- 默认离底后执行原水平回位，并打印前后XY差。无法仅据差值区分水流漂移和DVL跳变。
  `restore_pre_press_odom_xy: true` 才把离底后XY校正为下压前XY：
  **它假设压抓期间没有真实水平位移，不是测量结果**，有水流不要打开。
- `collection_visual_align: true` 默认启用；`collection_class_id: 4`。
  先到 `delivery.pose`（现在是粗定位点），再复用下视伺服及安装yaw180°补偿。
  `collection_projection_depth_m` 填相机到框平面的距离；对准失败不盲放。
  框须进入视野；当前不新增大范围搜索。沿用经验证的 `gripper.offset_*` 补偿后才释放。
- `collection_correct_odom_xy` 默认 **false**。开启需填固定框心的
  `collection_center_odom_xy: [x,y]` 和实际左相机光心的 `collection_camera_body_xy: [x,y]`。
  坐标校正在视觉居中后、夹爪偏置前执行，取3张独立新帧，要求单一类别4目标、艇体近水平且稳定。
  用相机平移、安装yaw、机器人yaw及残余像素反投影计算机器人XY。
  这是已知固定地标的平面近似；框会移动、相机距离/安装位置不准时不要启用。
  误差超过 `odom_correction_max_m` 拒绝校正，并停止本次投放，不隐藏失败。

新接口 `/basic_motion/correct_odom_xy`（`uv_msgs/srv/CorrectOdomXY`）只校正上位机任务odom
XY平移，反馈和目标反向变换一起更新，已发MCU目标不跳变，不调用MCU setorigin，也不改原始DVL。
任务的搜索点/投放点仍保留原START坐标定义；再次START会重置这项修正。
不要在运动动作进行中调用此接口；校正响应超时代表结果未知，任务停止后续移动。

因新增接口，不能只重编uv_task；停止任务后在真机工作区重新构建并重启相关节点：

```bash
source /opt/ros/foxy/setup.bash
colcon build --packages-up-to uv_control uv_task --symlink-install
source install/setup.bash
```

首轮建议只启用收集框视觉对准，坐标修正保持关闭；测好推力/池底后再开启近底开环。
以上未作真机推力标定，原现场速度和位置配置未替换。

### 海参夹爪切换：舵机1与舵机2

在 `config/tasks/grab_sea_cucumber.yaml` 的 `gripper.servo_id` 选择 `1` 或 `2`。
默认仍是1。任务链加载同一份配置，START紧接海参任务时也跟随这个选择。

| 选择 | 抓前准备 | 下压后 | 投放 | 对准补偿 |
|---|---|---|---|---|
| 1：原圆盘爪 | 舵机1、0° | 保持0° | 舵机1、90° | 原`gripper.offset_x_m/y_m` |
| 2：前下方夹爪 | 舵机2、270°张开 | 舵机2、150°闭合，等待后上浮 | 舵机2、270°张开 | 根据相机与新夹爪安装坐标计算 |

选择2后，START初始闭合150°；到扫描阶段再张开270°。
实际发送仍是 `/zit6/cmd/servo` 的 `zit6_interfaces/msg/ZitServo`，角度单位度。
不增加新的图像接口；下视海参segment、抓前/抓后计数、重试、下压、慢速上浮、
收集框类别4对准和返回扫描点沿用当前流程，新增闭合发生在下压结束与上浮之间。

`gripper.servo2` 中的三个位置均使用物理安装后的艇体系，x前、y右、z下，单位米：

- `front_camera_body_xyz`：前视双目中点的位置，名义 `[0.230,0,0.076]`。
- `down_camera_body_xyz`：当前用于下视伺服的左光心位置 `[-0.130,0.030,0.0645]`。
  按本次提供的Cd中心 `[-0.130,0,0.0645]` 与6cm双目距离，沿机体y轴对称展开；
  下视物理yaw倒装180°，原始左目位于机体+y、右目位于-y。
  收集框校正的 `collection_camera_body_xy` 同时填为 `[-0.130,0.030]`。
- `gripper_from_front_xyz`：实际抓取点相对前视中点的位置，按本次说明填 `[0,0,0.10]`。
  如果10cm描述的是机构根部，应把这里改成真正的接触/夹持点位置。

视觉居中后，海参在下视左光心下方。为了让海参处于新夹爪前下方抓取位置，
艇体的水平移动为 `下视光心XY - 新夹爪XY`；名义值为后退0.36m、向右0.030m。
这一正负号不能直接照搬旧爪子的前移补偿。抓取朝向沿用 `search.pose[3]`，
不会额外转向或识别海参长轴；实际有特定夹口方向时在搜索航向与安装坐标中调整。
投放时也使用新夹爪位置补偿，让夹爪位于视觉对准的框心上方。
近底开环启用时，新夹爪方案以实际抓取点的z为离底参考，而不是原圆盘爪的参考偏置。

调试选择：把 `gripper.servo_id: 1` 改为 `2`，按装配校准上述三个位置。
闭合等待可改 `gripper.close_wait_seconds`；角度可改 `servo2.open_angle_deg/close_angle_deg`。
源码绝对路径加载配置时，修改参数后重启task_runner即可；本次代码更新需先重编uv_task。

这些安装数据不能确定池底的任务odom深度、归一化下压/离底推力、收集框的已知世界位置
或相机到收集框的距离；相关待测参数继续保留，不用相机安装z代替池底深度。
本次6cm用于海参任务的物理安装偏置，未修改SGBM标定JSON中实测的R/T和基线。
