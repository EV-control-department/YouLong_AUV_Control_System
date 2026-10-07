# 转盘任务：实现和调试

此任务只用于真机独立调试，未加入 Stonefish mission。它执行**三次预设动作**，不根据黄标变化、盘心位移或累计角度判定转盘是否转成。日志中的“执行完毕”只表示运动指令完成，不代表实际盘体转角达标。

默认列表为 [mapping_grid.json](../src/uv_task/config/missions/mapping_grid.json)，现已按要求覆盖为 `start → turntable`；原建图任务实现和任务参数文件仍保留，但这个默认列表不再执行建图。启动前须把 AUV 人工放在前视能看到转盘、且不会碰撞盘架的位置；START 会建立 odom 原点，保留真机拔缆倒计时。若需要任务链自行粗定位，先实测盘心 odom 位姿，再在两项之间插入现有 `wtravelxyz` 和 `setrz`；这些基础动作的参数直接写在 mission 的 `params` 中，不需另建 YAML。当前没有填入猜测坐标。

粗定位填写的是**机器人目标位姿**，不是盘心。若量得盘心 odom 坐标 `(Dx,Dy,Dz)`，并确定机器人机头指向盘心的 odom 航向 `φ`（度，0° 沿 +x），则可先取相机离盘面 0.55 m 的观察位：

```text
robot_x = Dx - (0.55 + 0.23) cos(φ)
robot_y = Dy - (0.55 + 0.23) sin(φ)
robot_z = Dz - 0.076
robot_yaw = φ
```

其中 `0.23` 和 `0.076` 是当前相机 Body 名义外参，`z` 向下；装配不符须改用实测外参。把 `robot_x/y/z` 交给 `wtravelxyz`，再用 `setrz` 设 `robot_yaw`。`WTRAVEL` 会内部先转向行进方向、再直线前进，最后 `setrz` 正对盘面；它不做避障，因此必须先核对路径无障碍、观察位不会撞盘架。随后转盘任务仍会从前视观测重新计算非接触对准位和插孔位。若只有盘面朝外法向航向，而非“机头指向盘心”的航向，应先换算 180°，不能直接填入。

## 信息流

前视双目图像在 `uv_camera` 进程内由 YOLO-Seg 提取整盘掩膜（真机 `last.pt` 中 `wheel=3`），再由 HSV 找黄色条幅。SGBM/盘面拟合估计盘心、盘轴和质量；视觉节点只向 DDS 发布带采集时间的小型 `/perception/turntable/observation` JSON。任务节点只订阅该观测与 `/basic_motion/pose_info`，不订阅图像话题，也不运行 YOLO/SGBM。黄标是其中一根辐条，而非必须另有 YOLO 类别。

任务流程：

1. 验证观测采集时间和实测里程计新鲜度；要求配置中的棍半径、盘外距离和插入深度有效，且两个接触放行开关都为 `true`。
2. 用视觉盘心/盘轴走到约 0.55 m 外的非接触正视位，取一帧新观测。程序从黄色辐条起算 45°，在四个相邻辐条中间选最接近盘顶/盘底的孔。这里假设四辐条等间隔，需现场核对镜像与黄标位置。
3. 每程都用**新观测**计算预插入位，以 `WTRAVEL` 对孔，到位后再视觉复核孔位误差。先用 `BMOVE x` 分段插入（每步 ≤2 cm），用 `BMOVE rz` 分段向固定方向推盘（每步 ≤2°），再分段退出。前两程在盘外反向复位 yaw，重新找孔；第三程退出即结束。
4. `BasicMotion` 返回后仍核对实测里程计。图像、位姿、动作或几何检查失败即停止后续运动；**不测量推盘后的实际盘体转角，不据此增加或减少次数**。

固定几何：盘外径 230 mm，外环内沿直径 200 mm，内圈直径 35 mm，四辐条宽 20 mm；候选接触半径 58.75 mm。名义前视双目中点 Body `(0.230,0,0.076)m`，杆根相对中点 `(0,-0.09,0)m`、沿 Body `+x` 伸出 0.16 m，故名义棍尖 Body `(0.390,-0.090,0.076)m`。Body `x` 前、`y` 右、`z` 下。这些值固化于代码，**装配改变时必须同步改代码并空载复核**。盘心和盘轴不是手填值，运行时由视觉获得。

只需现场填写 3 个尺寸，另可调整单程 yaw 行程；见 [最小标定清单](turntable_calibration_checklist.md) 和 [任务模板](../src/uv_task/config/tasks/turntable.yaml)。默认两个放行开关为 `false`。位置小步进不等于力控制；接触前必须确认控制器限速/限推力、人工急停和退杆办法。因为不再监测盘体位移或转角，三次动作可能未转动、反向滑脱或把盘拉动；须通过录像/人工验收，不可把本任务的 `True` 当作物理完成证明。

## 无接触调试

在仓库根目录构建并 source：

```bash
source /opt/ros/humble/setup.bash
cd workspace_auv
colcon build --symlink-install --packages-select uv_camera uv_task
source install/setup.bash
```

前视单目工作尺寸为 640×480，左右拼接 1280×480，标定文件为 `front.npz`。若真机底层节点尚未运行，先启动硬件、`basic_motion` 与转盘视觉，但关闭 launch 内的任务节点，以免出现两个 `task_runner`。在仓库根目录执行：

```bash
cd /home/laurie/AUV_2026_Robocup/YouLong_AUV_Control_System
source /opt/ros/humble/setup.bash
source workspace_auv/install/setup.bash
ros2 launch uv_bringup real.launch.py \
  profile:=real_default enable_ai:=true enable_nav:=false enable_task:=false \
  turntable_mode:=true
```

另一终端运行：

```bash
source /opt/ros/humble/setup.bash
source /home/laurie/AUV_2026_Robocup/YouLong_AUV_Control_System/workspace_auv/install/setup.bash
ros2 run uv_task task_runner
```

不传参数时自动加载安装后的 `mapping_grid.json`，立即开始 `start → turntable`，无需任务 launch。先检查 `ros2 topic echo /perception/turntable/observation` 与 `ros2 topic echo /basic_motion/pose_info`；观测应持续更新、`valid=true`、`phase_valid=true`，盘心/法向在静止时稳定。`turntable_mode:=true` 开启前视转盘视觉；真机 launch 已自动选用 `real_last.yaml` 类别映射与 `last.pt`。只运行 `task_runner` 不会自行启动硬件、运动控制或相机节点。默认接触开关关闭，且三个几何尺寸待实测，因此默认链**只能验证 START/任务加载，不能推盘**。完成 [放行清单](turntable_calibration_checklist.md) 并填写三个尺寸后，才考虑在人工监护下将两个接触开关置 `true`。

常见拒绝：`missing_disk_mask` 为整盘未检出；`phase_valid=false` 为黄色条幅未检出；盘面残差/长短轴比不达标说明未正视或深度不可靠；图像采集时间超限和里程计过旧表示同步/负载问题。不得通过放宽插孔误差绕过几何安全检查。

## 仿真冒烟结果（2026-10-04）

在独立的临时任务列表中仅运行 `start → turntable`，为测试填写三个几何量，同时保持两个接触开关为 `false`。Stonefish 采用 `water_embodied_intelligence_random.scn`、软件 OpenGL；前视视觉启用 `turntable_mode:=true` 并显式加载 `last.pt`。START 成功，YOLO 正常加载，但相机持续报 `missing_disk_mask`；任务 8 秒后报“初始等待超时：没有新的有效转盘视觉观测；最近无效原因=missing_disk_mask”，列表记录 1/2 失败。没有发出 WTRAVEL、插杆或推盘指令。

当前场景里仅有静态水平圆盘示意物，不是 230 mm、竖直、可转动、可插孔的实物转盘；从初始位置也无法指望直接看到可供真机模型识别的目标。因此这次测试只证明任务链、模型加载、失败日志和禁动行为，**没有验证盘面定位精度或三次接触运动**。测试结束时由定时器送出的 SIGINT/退出码 -2 是关闭仿真的结果，不是转盘任务的失败原因。
