# 真机转盘任务（独立调试，不进入仿真链路）

当前实现位于 `uv_camera/turntable_vision.py` 和 `uv_task/turntable_task.py`。`uv_camera` 在进程内读取前视 YOLO-Seg 结果，拟合盘面椭圆，以黄色标签掩膜的面积质心计算角度；向 DDS 只发布 `/perception/turntable/observation` 的小型 JSON，包含采集时间戳、角度、盘心、视半径、置信度和有效性。`uv_task` 不订阅图像 topic，也不处理像素或运行 YOLO。

当前仓库的 `robotcup20260901.yaml` 类别表没有转盘整体和黄色标签；已有 `best.pt` 也不能被当作已训练的转盘模型。先训练/提供**同时含盘体与黄标分割类**的模型，查明两个实际 class ID，再设置相机参数 `model_path`、`enable_turntable_vision:=true`、`turntable_disk_class_id`、`turntable_label_class_id`。两类 ID 必须不同。若黄标横跨圆心导致掩膜质心接近圆心，单帧角度无法辨识，相机会发布无效观测；不得把它当作 0°。

## 标定与坐标

模板见 `config/tasks/turntable.yaml`。必须现场测量盘轴心在 START 后里程计系中的 x/y/z、盘面法向航向、棍端相对机体的 x/y/z、棍半径、内外圈净空、选择的接触半径、相机角度到盘面角度的符号、黄标到孔位的机械相位、接近距离、插入深度，以及 yaw 推动方向。机体系约定 x 前、y 右、z 下；图像角度右为 0°、上为 90°。需要确认真机的低速/限推力控制能力，`BasicMotion` 动作接口本身**没有速度或接触力上限参数**。没有力传感器时，盘体被整体拉动的风险不能由图像启发式检查完全消除。

`allow_contact_motion: false` 和 `force_limited_control_confirmed: false` 为默认禁动。配置项没有测量值时故意省略，运行时明确报“尚未标定”，不会使用猜测几何量。`drive_yaw_sign: 0` 同样表示未标定，必须确定为 +1 或 -1。不要直接把模板改成 `true` 发车。

待测量值、待水中调参项、尚未确定的比赛判据与逐阶段放行记录，统一维护在 [转盘标定与调试清单](turntable_calibration_checklist.md)。请先填写清单，再修改参数模板；不要根据示例值直接开启接触动作。

## 动作与判定

1. 收到时间戳有效、前视新鲜的盘体及黄标观测，同时要求新鲜的**实测** `PoseInfo`；不会用任务指令位姿代替。
2. 黄标角 + 标定机械相位推算四个孔位，选择接近盘面顶部/底部的孔：这里 yaw 引起的水平棍端位移才有较大的切向分量。通过棍端外参反算接近位姿，并验证孔位半径留有棍半径加 1 cm 余量。
3. 显式解除禁动、确认低速控制后，`WTRAVEL` 到盘面前的标定接近位；逐段 `BMOVE x` 插入，每步不超过 2 cm；逐段 `BMOVE rz` 推动，每步不超过 3°、每程不超过 12°。
4. 每程先沿 x **完全退出**盘面，再看黄标角变化是否达到最小进展。两次推盘方向若不一致则停止。盘外反向复位 yaw，并依据新黄标角重新计算下一孔位及接近位置，重复共三次。这样不会带棍原路倒转转盘。
5. 动作后若无新图像、新实测位姿、盘心图像突变、视半径突变、位姿没有实际到达、黄标没有角度进展，立刻停止后续接触。图像盘心突变只能充当粗略安全提示，**不能证明转盘轴心未被整体推走**。现场应加机械止挡/力反馈并安排人工急停。

`BMOVE` 小行程即使回报成功，也可能落入基础控制器约 0.1 m 的到位容差而几乎没动。任务在每个 x/yaw 小步后再次检查实测里程计；检查失败就停止。当前目标角度、内外圈半径和机械零位均未知，所以只定义“三次可确认的同向短行程”，不声称达到比赛规定的绝对角度。目标角度明确后需增加闭环终止判据；仅靠三次计数不够。

## 单独试运行

先构建并 source `workspace_auv`，启动基础控制器和 `uv_camera`，为相机配置新的转盘模型/类别参数，再启动调试任务节点。不要把 `turntable` 写进 `sim.launch.py`、`mapping_grid.json` 或自动 mission。

```bash
cd YouLong_AUV_Control_System/workspace_auv
colcon build --symlink-install --packages-select uv_camera uv_task
source install/setup.bash
ros2 run uv_task task_runner --ros-args -p debug_mode:=true
# 另一个终端：先确认小型角度观测是否更新
ros2 topic echo /perception/turntable/observation
# /task/exec 仅用于调试；参数 JSON 必须由现场标定值填入。
ros2 service call /task/exec uv_msgs/srv/ExecTask '{task_name: turntable, params_json: "{}", timeout: 0.0}'
```

上面的 `{}` 会因几何参数缺失而安全失败；先填写模板或向服务传入等价的完整参数 JSON。`debug_mode` 的服务不自动读取任务模板。若想从配置运行，可用任务文件单独启动任务节点；**文件一旦设置为允许接触就会自动执行**，仅建议在具备急停与低速限推力验证后使用。

本任务尚未在实物转盘上验证。当前仿真转盘是固定实心体，和可转动的双圆环/四辐条结构不一致，因此本任务没有接入仿真链路，也无法用现有 Stonefish 场景作接触验证。
