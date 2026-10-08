# YouLong AUV real 端实机下水测试指导

依据：2026-10-06 当前仓库源码及仓库内 ZIT6 固件。下面是待现场执行的流程，软件离线检查通过不代表硬件已验收。指令默认在 Foxy 容器内执行，仓库挂载为 `/workspace`。

## 1. 本次测试的顺序和放行条件

按 **断开推进器动力检查 → 静态浸水 → 单项执行器 → 小幅闭环运动 → 感知与录制联测 → 收尾** 执行。上一阶段未通过就结束这一轮、修复并复测。

| 阶段 | 测什么 | 进入下一阶段的条件 |
|---|---|---|
| 陆上、推进器动力隔离 | 接线、防水、电压、计算机、MCU 通信、相机、日志、实体停机 | 通信稳定，部件编号清楚，实体停机有效，MCU 未解锁 |
| 水中静态、仍禁止推进器输出 | 漏水、浮态、深度变化、INS/DVL、USBL、定位反馈 | 无进水，姿态稳定，导航有效、反馈无跳变 |
| 水中单项执行器 | M0–M5、两路舵机、灯、推杆（按实际安装） | 方向、接线、运动范围、截止行为均确认 |
| 水中小范围闭环 | hold → set → bmove → wmove → travel → velocity | 每阶段 PASS，实际轨迹与估计轨迹一致，软件停止和实体停机均已验证 |
| 联测 | 感知、目标定位、推流、录制、可选规划 | 硬件运动正常时数据仍持续，无重启/超时，记录可回看 |

至少两人配合：一人操作和观察数据，一人负责回收绳与实体急停。第一次在静水、远离池壁/人员/障碍的区域进行，推进器完全浸没后才接通动力。回收绳留足运动余量并固定在不会接近桨叶的位置。不要依靠电脑服务调用替代实体急停。

本次水深 **1.1m**、电池 **6S**、实体急停为**应急开关**，也可关闭所有心跳源让 MCU 超时上锁。默认水平试移 0.25m、偏航 10°，**默认关闭垂向动作**；确认上下净空后，每次显式开启垂向测试，步长为 0.15m。这些是目标位移，**不是位置模式的速度/推力限幅**；实际速度由固件控制器决定。测试任务会在估计线速度超过 0.20m/s 时请求取消，但这也不能保证物理制动距离。现场需要的池壁/池底/水面余量应依据实物尺寸和制动距离确定。

现场记录：水池深度 **1.1m**；可用平面区域______；测试进入深度______；实体停机 **应急开关**；电池串联数 **6S**，化学体系/容量______；本次测试电压下限 **21.0V**；MCU 固件版本/预设______；回收负责人______。

21.0V 是本次首次测试采用的总压下限（6 节平均 3.5V），应与电池标签及厂家要求核对，单节电压也要检查；它不替代 BMS/ESC 保护。未提供电池化学体系，本文不设定充电上限或改动固件低压保护。

浅水垂向放行：先静态测量车体最高点/最低点距水面/池底的距离。一次下降 0.15m 后仍要留足超调与回收余量；首次可暂以至少 0.20m 净空作为更保守的现场检查值，实际还要按制动距离、车体倾斜和波动增加。若整机尺寸或入水姿态使这些余量无法满足，本池只验收深度计和深度保持，垂向移动换到更深水域。

## 2. 当前软件接口中要先弄清的事实

1. `hw_manager` 默认 `arm_mode=1`，启动后直接向 `/zit6/cmd/agxhbt` 发送 5Hz `UInt32` 心跳，`data` 为 `arm_mode`；仓库固件要求至少 10 次心跳、持续至少 1s 且导航有效，5Hz 下通常约 2s 后会自动解锁。首次陆上检查使用下方的 `test_disarmed.yaml`，同时物理隔离推进器动力。
2. **已解锁后把 `arm_mode` 改为 0 不会立即上锁。** 固件在已解锁分支只检查心跳超时。停止所有心跳发布者后，仓库固件约 1s 后上锁；`hw_manager.watchdog_timeout=7s` 是上位机监控阈值，两者不是同一个时间。
3. real bringup 的所有 profile 都不启动或监控 task_runner，不发送 BasicMotion START 或 safe-stop；状态页只显示硬件、定位、相机、感知和导航健康。测试采用 `profile:=debug`，然后单独启动处于 `debug_mode=true` 的任务节点。
4. 真机固件发布 `/zit6/state/*`，`hw_manager` 将状态适配到 `/auv/hardware/zit6/state/*`。`basic_motion` 同时发布新旧 setpoint；解锁心跳由 `hw_manager` 直接发送到 `/zit6/cmd/agxhbt`。原 `/auv/hardware/zit6/cmd/heartbeat` 和 `/zit6/cmd/heartbeat` 已弃用，不再发布或转发。
5. canonical 灯光和舵机命令由 `hw_manager` 转发到固件 `/zit6/cmd/light`、`/zit6/cmd/servo`；舵机命令类型为 `zit6_interfaces/msg/ZitServo`（`servo_id` 为 1 或 2，`angle` 单位为弧度）。固件 `/zit6/state/servo` 的已接受目标角会适配到 `/auv/hardware/zit6/state/servo`；它不是物理角度测量。INS 命令仍需使用固件实际接口。
6. 相机像素经 `youlong/camera/front`、`youlong/camera/down` 共享内存服务传输。没有 ROS `Image` 持续发布不代表相机坏了。相机状态和 CameraInfo 当前也不是周期健康心跳，不能用它们的频率来验收持续采集。
7. real 定位主要读取 MCU 的位置/速度反馈。当前 real bringup 没有额外启动独立 IMU、DVL、压力 ROS 驱动；不能要求这些 canonical 原始传感器 topic 一定有发布者。需要通过 MCU 诊断/日志及位置速度变化验证，额外安装了 ROS 驱动时再检查其原始 topic。
8. `/auv/hardware/zit6/state/thruster` 的六个数是 `[Fx,Fy,Fz,Mroll,Mpitch,Myaw]` 控制力/矩，不是 M0–M5 的独立 RPM/电流反馈。
9. **仓库 MCU 固件将 `battery_voltage` 固定发布为 0.0，当前没有电压遥测。** `hw_manager` 的低压告警会跳过这个值，调高告警阈值也不能产生真实电压监控。本任务默认拒绝在没有电压依据时运动：每轮需通过电压表/单节检测仪测量，填写 `external_battery_voltage`，并由现场独立仪表监控带载电压。任务不能靠这一份静态读数判断运行中的压降；若部署固件已提供真实非零遥测，则按遥测检查，外部数值不能覆盖实际低压。

BasicMotion 的 real 上游链路为 **MCU `zit6_node` → `hw_manager` 状态适配 → `uv_localization` → `basic_motion`**。核心输入是 `/auv/state/odom`（PoseInfo，位置、角度及 origin_* 原点字段）和 `/auv/state/twist`（TwistWithCovarianceStamped，速度）；它还订阅 MCU status，但目前主要缓存，未自动阻止所有不健康运动。当前 localizer 临时忽略 MCU `ZitOdom.nav_valid`：只要 odom 有限且新鲜，就会报告定位可用并发布速度，原点已初始化时也会发布 TF。因此健康 `available=true` 不代表 MCU 的 `navigation_ready=true`；实机操作仍须单独检查 MCU status。`task_runner` 是运动指令来源，感知和 navigator 不属于基本运动闭环必须的上游。

real bringup 只启动 BasicMotion 节点，不发送 START。操作员在运行任务中手动发送 START 会让 BasicMotion 重置原点；bringup 会继续运行并只打印健康状态，但正在执行的任务可能因坐标原点变化而失败。手动 START 前应先让任务进入空闲/停止状态，START 完成后再按任务流程启动。

## 3. 准备环境与三个独立终端

宿主机先确认相机和串口的实际枚举及权限：

```bash
cd /home/doc049/dev/UUV/YouLong_AUV_Control_System
ls -l /dev/ttyUSB* /dev/serial/by-id/ /dev/video*
YOULONG_RUNTIME=real ./scripts/compose_up.sh up -d
docker exec -it youlong_auv bash
```

默认 front `/dev/video2`、down `/dev/video0`、MCU `/dev/ttyUSB0`。若实际不同，先调整 Compose 的 `CAMERA_FRONT_DEVICE`、`CAMERA_DOWN_DEVICE`、`HARDWARE_SERIAL_DEVICE` 映射，并让相机 real 配置中的 device 与容器内路径一致。相机标定文件为 `workspace_auv/src/uv_camera/stereos/front.yaml`、`down.yaml`，real 分支采集 2560×960、每目 1280×960。

首次应用新增测试任务后，在推进器动力隔离时构建：

```bash
cd /workspace
source scripts/source_workspace.sh
cd /workspace/workspace_auv
colcon build --symlink-install --packages-up-to uv_control uv_task uv_hm uv_bringup
cd /workspace
source scripts/source_workspace.sh
```

每个新终端都进入同一容器并执行 `cd /workspace; source scripts/source_workspace.sh`。现场只保留一个 MCU Agent、一个硬件心跳发布者、一个 BasicMotion server、一个 task runner。不要同时运行 `upper_examples heartbeat`、摇杆解锁程序或另一份 bringup。

**终端 A：MCU Agent。** real.launch 不自动启动 Agent。若部署服务已提供 Agent，使用已有进程，不重复启动。否则按仓库固件 USART2 波特率启动；可执行文件不在 PATH 时使用实际安装位置。

```bash
micro_ros_agent serial -D /dev/ttyUSB0 -b 921600 -v 4
```

**终端 B：保持未解锁的硬件管理器。** 首次启动前重启 MCU、确认推进器动力隔离；此配置不能把一个已经解锁的 MCU 立即上锁。

```bash
ros2 launch uv_hm hardware_launch.py \
  params_file:=/workspace/workspace_auv/src/uv_hm/config/test_disarmed.yaml
```

**终端 C：其他 real 组件与录制。** 硬件节点在 B 中单独运行，因此这里关闭重复启动。首轮先关闭 AI；开始感知联测时，停下 C 后把 `enable_ai` 改成 true 重启。重启 C 后任务节点也要按第 7 节重新准备。

```bash
ros2 launch uv_bringup real.launch.py profile:=debug \
  enable_hardware:=false enable_motion:=true \
  enable_nav:=false enable_ai:=false enable_stream:=true \
  record_session:=true record_mode:=raw \
  record_use_sim_time:=false
```

按 Ctrl+C 停 C 不会停止独立终端 B 的心跳，收尾必须同时处理 B。数据流中发现 MCU 不在线，不要启用强制解锁 `arm_mode=3` 绕过导航检查。

## 4. 节点和基础通信逐项验收

另开检查终端。节点没有 `/auv` 前缀通常正常；namespace 主要体现在 topic。录制、go2rtc、streamer 部分为普通子进程，不一定出现在 `ros2 node list`。

```bash
ros2 node list
ros2 topic list -t
ros2 service list -t
ros2 action list -t
ros2 node info /hw_manager
ros2 node info /uv_localization
ros2 node info /basic_motion
ros2 param get /hw_manager arm_mode
ros2 param get /uv_localization sim_mode
ros2 param get /basic_motion sim_mode
```

要求两个 `sim_mode` 均为 false；首次检查 `arm_mode=0`。不存在 `stonefish`、`sim_bridge` 等仿真运行进程。可用下面的 `topic info -v` 查看发布者数量、订阅者、类型及 QoS：

```bash
ros2 topic info /auv/state/odom -v
ros2 topic info /auv/state/twist -v
ros2 topic info /auv/hardware/zit6/state/status -v
ros2 topic info /zit6/cmd/setpoint -v
ros2 topic info /zit6/cmd/agxhbt -v
```

`/auv/state/odom` 与 `/auv/state/twist` 应由 `uv_localization` 独立发布；`/zit6/cmd/setpoint` 和心跳应能看到固件 `zit6_node` 的订阅。同一 topic 出现不同类型，或多个估计位姿发布者，先解决再运动。

| 节点/进程 | 检查依据 | 通过标准 |
|---|---|---|
| MCU `zit6_node` + Agent | `/zit6/state/zithbt`、`status`、`pos`、`vel`、`/zit6/log` | 连接稳定，约 1Hz 心跳、10Hz 状态、30Hz 位姿速度；无持续重连 |
| `hw_manager` | canonical 状态与 legacy 对照、5Hz `/zit6/cmd/agxhbt` | 数值一致、时间连续；未解锁测试时 `is_armed=false` |
| `uv_localization` | odom、twist、`/auv/state/health` | 约 30Hz；健康 `available=true`；原始反馈停更不能仅凭 odom 仍有发布判定正常 |
| `basic_motion` | `/auv/basic_motion` Action、setpoint 连接 | server 唯一；功能按第 7 节实际动作验证 |
| `robot_state_publisher` | `/auv/tf_static`、`/auv/tf` | `odom→base_link→相机光学帧` 可查询；没有伪造 DVL 安装外参 |
| `uv_camera` | 共享内存画面、四路 CameraInfo | front/down 左右目均有新帧、身份和方向正确 |
| `model_class_publisher` | `/auv/perception/model_classes` | latched 映射可读取；task runner 能启动 |
| `object_detector` | `/auv/perception/detections` | 真实目标进入不同视角后检测对应变化；没有模型加载失败/重启循环 |
| `object_localizer` | `/auv/perception/measurements` | 新观测到达；frame 为 odom；射线/双目位置方向和距离合理 |
| `object_estimator` | `/auv/perception/tracks` | 跟踪稳定、有目标时可对应；移动相机后仍对应同一静态目标 |
| go2rtc + `camera_streamer` | 1984 HTTP 页面、front/down/annotated 流 | 视频持续更新；存在观看/录制消费者时，streamer 才按需启动 |
| `uv_record` + rosbag/PNG 写入器 | session 文件、manifest、日志 | 文件持续增长、无重复启动；退出后可读取记录 |
| `task_runner` | mission 三个服务及 status | 先保持空闲，启动测试后发布 RUNNING、结束有 DONE/ERROR |
| 可选 `navigator` | `/auv/planning/navigate_to`、`/auv/control/trajectory` | 规划输出合法；当前没有路径执行闭环，不能以服务成功验收自主导航 |

## 5. 水中静态：硬件与传感器逐项检查

先保持 `arm_mode=0`、MCU 未解锁，缓慢把整机放入水中，观察密封、接插件、浮态和回收绳。出现进水、异常电流、姿态翻倒就收回处理。

### 5.1 电源、通信、INS、深度和 DVL

```bash
ros2 topic echo /auv/hardware/zit6/state/status
ros2 topic echo /auv/hardware/zit6/state/position
ros2 topic echo /auv/hardware/zit6/state/velocity
ros2 topic echo /auv/state/health
ros2 topic echo /zit6/log
```

另开终端可分别检查频率：

```bash
ros2 topic hz /zit6/state/zithbt
ros2 topic hz /auv/hardware/zit6/state/status
ros2 topic hz /auv/state/odom
ros2 topic hz /auv/state/twist
```

逐项记录：

- **电池/电源**：用独立仪表记录静态和短时带载电压、各单节电压、电流、计算机是否重启、`thrust_tx_fail_count` 是否增长。状态电压固定 0 的固件不能验收为“电压遥测通过”。仓库原 hardware 默认告警阈值为 14V，本次 test_disarmed 配置与测试任务设为 21V；告警阈值不是自动断电保护，没有遥测时不会告警。后续切换解锁只改 arm_mode，不回到原来的 14V 配置。
- **INS/IMU**：静止时位姿不持续旋转；人为小幅俯仰/横滚/顺时针偏航，位置数组 `[x,y,z,roll_rad,pitch_rad,yaw_rad]` 的对应项变化正确。`ins_state=3/4` 才满足仓库固件正常导航模式；粗/精对准或 MRU 状态不放行运动。
- **深度/压力**：人工在上下净空允许时缓慢下降约 0.10m，Z 正向增加；升回原位置后读数回到附近。按固件实际 `z_data_sourse` 确认取的是 INS Z 还是深度计，单测一个源不能验收另一个源。M14/MS5837 有效帧超时应在 `/zit6/log` 报警；收到串口字节不等于解析有效。
- **DVL**：先确认上电、池底距离满足锁底条件；人工沿机头、机体右侧小幅移动车体，机体速度 X/Y 的符号应对应向前/向右为正，静止后回到近零。不要用转圈运动当作第一次 DVL 轴向检查。若有独立 DVL 驱动，再核对其 `valid`/锁底/速度原始消息。
- **失联诊断**：推进器动力隔离状态下，依次断开 Agent 或传感器数据链路，确认日志/健康标志响应；重新接入后确认恢复。水中推进器通电时的失联测试另见第 8 节。

只有需要开启 DVL 且串口/接线已确认时，发送固件支持的 INS 命令：

```bash
ros2 topic pub --once /zit6/cmd/ins std_msgs/msg/UInt8 "{data: 1}"
```

固件定义：1=DVL 开电、2=DVL 关电、3=INS 重启、4=INS 位置复位、5=设置初始经纬度。3/4/5 会改变导航状态/参考，不在运动测试中发送。

### 5.2 USBL

```bash
ros2 topic echo /zit6/state/USBL
ros2 topic echo /auv/sensors/usbl/measurement
```

记录有无信标、信标位置、斜距与信号质量。放置已知位置的信标，改变方向/距离，核对有效帧和位置是否相应变化；无信标时“没有有效帧”需记录为未验收，不能把稳定零值算通过。canonical `valid` 也要查看。仓库中没有为 real 发布未经测量的 `dvl_link`；USBL/INS 安装关系须按实物标定复核。

### 5.3 相机、左右目、共享内存与 TF

在操控电脑打开 `http://<AUV计算机IP>:1984/` 的 front/down 原图。依次遮住 front-left、front-right、down-left、down-right，确认相机身份；转动物体，确认持续新帧和方向。检查清晰度、曝光、壳体反光、防水窗口、左右配对和实际分辨率。

```bash
ros2 topic echo /auv/tf_static --qos-durability transient_local
ros2 run tf2_ros tf2_echo base_link front_left_camera_optical_frame \
  --ros-args -r /tf:=/auv/tf -r /tf_static:=/auv/tf_static
ros2 run tf2_ros tf2_echo odom base_link \
  --ros-args -r /tf:=/auv/tf -r /tf_static:=/auv/tf_static
```

CameraInfo 在 real 采集首帧时发布一次；若启动后才订阅而看不到，先让订阅者运行，再重启相机对应 launch，或检查启动日志和 recorder 数据。不要把 CameraInfo 无持续频率误判为采集停顿。原图连续性用视频及 `camera/raw/*/frames.jsonl` 的时间戳、capture_id、左右配对 ID 验证。

## 6. 执行器单项检查

### 6.1 六个推进器 M0–M5

按实物标号记录 **物理位置 → 电机板通道/ESC ID → 正/反方向 → 电流/声音 → 是否按时停止**。M0/M1/M4/M5 为水平推进器，M2/M3 为垂向推进器，具体方向以车辆 description 和实际安装为准。

当前上层 ROS 接口没有暴露每个物理推进器的独立转速/电流命令。因此，完整的一对一通道验收应使用已确认的电机板/MCU 维护程序，在浸水夹具中逐个短时驱动。没有可靠的维护入口时，这一项记录为“待验收”；不要通过修改混控或发送无来源的 PWM 数值来补齐检查。

第 7 节的 X/Y/Z/yaw 小幅运动能验证混控组合的实际效果，但不能替代 M0–M5 一对一接线与正反推验收，也不能证明某个未观测的电机没有故障。

### 6.2 灯光

确认 canonical 命令由 `hw_manager` 订阅、固件端点有订阅者后，逐个检查 1/2/3 状态与最终关闭，记录颜色/状态的实物对应：

```bash
ros2 topic info /auv/hardware/zit6/cmd/light -v
ros2 topic info /zit6/cmd/light -v
ros2 topic pub --once /auv/hardware/zit6/cmd/light std_msgs/msg/UInt8 "{data: 1}"
ros2 topic pub --once /auv/hardware/zit6/cmd/light std_msgs/msg/UInt8 "{data: 2}"
ros2 topic pub --once /auv/hardware/zit6/cmd/light std_msgs/msg/UInt8 "{data: 3}"
ros2 topic pub --once /auv/hardware/zit6/cmd/light std_msgs/msg/UInt8 "{data: 0}"
```

### 6.3 舵机 1/2、释放机构

```bash
ros2 topic info /auv/hardware/zit6/cmd/servo -v
ros2 topic info /zit6/cmd/servo -v
ros2 topic echo /auv/hardware/zit6/state/servo
```

固件命令为 `{servo_id: 1或2, angle: 弧度}`。先根据实物当前角和机械限位选择测试角，只做小幅变化，例如在已确认安全角范围内改变 0.05rad；分别确认两个物理通道，再恢复。发送形式如下，先填写实际安全角：

```bash
# 示例结构：把 TEST_ANGLE_RAD 改成根据实物选定的弧度数值。
ros2 topic pub --once /auv/hardware/zit6/cmd/servo zit6_interfaces/msg/ZitServo \
  "{servo_id: 1, angle: TEST_ANGLE_RAD}"
```

`ZitServoState` 表示控制板接受下发的角度，不是物理角度闭环测量；必须现场观察连杆/释放机构，不能仅凭 echo 的角度判定成功。测试时移除会意外掉落/夹伤的负载。

### 6.4 推杆（若安装）

先核对当前固件预设与 GPIO/UART 驱动类型。接口为 `/zit6/cmd/pushrod`，类型 `zit6_interfaces/msg/ZitPushrod`，字段 `speed ∈ [-1,1]`、`duration_ms>0`；固件要求 MCU 已解锁。

GPIO 推杆正负 speed 主要决定方向，较小的数值不一定降低机械速度或供电强度。先使用专门夹具和已确认的短脉冲持续时间、足够的行程余量，验证正向、反向、定时截止和限位。ROS 有日志不等于推杆实际运动成功。本次没有安装的执行器明确填 N/A。

## 7. `basic_motion_test` 分阶段运动任务

新增 task 名为 `basic_motion_test`，配置在 `workspace_auv/src/uv_task/config/tasks/basic_motion_test.yaml`。这是一个包含内部检查的单项 task；失败后不会继续后续测试步骤，放进 mission 时失败也会停止该 mission。

测试开始和执行过程中检查：定位/速度/健康/MCU 状态新鲜度、定位 available、MCU 已解锁和 navigation_ready、error_flags=0、位姿有限值、倾斜角、估计速度和相对进入位姿的运动范围。有真实电压遥测时持续检查电压；电压为占位 0 时只检查本轮填写的外部实测值，持续带载监控由现场完成。任何检查失败都会请求取消当前动作并尝试发送零速度。它不能发现所有物理故障，也没有独立电气急停能力。

首轮 hold 会以当前水下位姿 START；后续各阶段沿用同一原点、以每轮进入时的实测位姿作为返回点。每次成功的偏移动作后用 SET 返回。任务终止时确认一次零速度 Action，**不会自动上锁，也不会保持位置/深度**，必须有人负责回收和停机。

### 7.1 手动准备与解锁

完成第 1～6 节需要的检查，确认 AUV 完全浸水、人员离开桨叶、回收和实体停机就绪。让终端 B 在原来的 test_disarmed 配置下继续运行，再手动切换：

```bash
ros2 param set /hw_manager arm_mode 1
ros2 topic echo /auv/hardware/zit6/state/status
```

确认 `is_armed=true`、`navigation_ready=true`、`ins_state=3/4`、`error_flags=0`，并观察至少几秒定位稳定后再运行任务。不要用模式 3 让导航未就绪的整机下水运动。

终端 D 启动手动任务执行器。`real_test_runner.yaml` 设置 `debug_mode=true`，启动本身不发运动指令。class mapping 即使关闭 AI 也由 real bringup 的 `model_class_publisher` 提供。

```bash
ros2 launch uv_task task_launch.py camera_mode:=real \
  params_file:=/workspace/workspace_auv/src/uv_task/config/real_test_runner.yaml \
  mission_file:=/workspace/workspace_auv/src/uv_task/config/tasks/basic_motion_test.yaml
```

监控终端持续看状态、实测位姿/速度与 setpoint：

```bash
ros2 topic echo /auv/mission/status
ros2 topic echo /auv/state/odom
ros2 topic echo /auv/state/twist
ros2 topic echo /auv/hardware/zit6/cmd/setpoint
```

### 7.2 第一次：hold + START

确认测试 YAML 中的电压下限符合实际电池。当前固件状态电压为 0 时，先测量并把 YAML 的 `external_battery_voltage: 0.0` 改成本轮实际总压；默认 0 会拒绝运动。不要填写估计值。然后手动启动文件：

```bash
ros2 service call /auv/mission/run uv_msgs/srv/RunTask \
  "{task_name: '/workspace/workspace_auv/src/uv_task/config/tasks/basic_motion_test.yaml', start: true}"
```

默认 `stage=hold`：先发送零速度，START 重置原点，等待 odom 重置反馈，然后 SET 保持当前位姿、检查稳定后的实测误差，最后零速度。

要求没有向甲板启动深度/旧位置跳转；观察横滚/俯仰、实际推进器输出与物理位置是否稳定。此阶段若异常，立即停止，不启动 set。每次 service 返回 `success=true` 仅说明任务已接收，最终结果看 `/auv/mission/status` 与任务日志。

### 7.3 后续：逐阶段，上一阶段通过后再发下一条

后续 execute 会继承上一轮测试的距离、电压下限和范围设置；每次默认不重复 START，并默认关闭垂向。**外部电压读数不继承**：当前固件无电压遥测，每轮测量后显式填写一次。每轮之间留出观察、记录、回收绳检查时间；不要连续粘贴整组命令。

| stage | 验证内容 | 默认动作 |
|---|---|---|
| `set` | SET 绝对位置/航向、轴方向与返回 | yaw +10°、X +0.25m、Y +0.25m，每项后返回；显式开启垂向时增加 Z +0.15m |
| `bmove` | BMOVE 机体增量 | 右偏航/前/右小步，每项后返回；垂向需显式开启 |
| `wmove` | WMOVE 世界绝对目标的步进执行 | 基于进入位姿构造 yaw/X/Y 小步目标，每项后返回；垂向需显式开启 |
| `travel` | WTRAVEL + BTRAVEL 转向/直线移动 | 沿进入航向前进 0.25m，各项后 SET 返回，避免用向后 TRAVEL 导致 180° 转身 |
| `velocity` | BODY_VELOCITY 及租约 | X/Y 0.05m/s、yaw 3°/s，各持续 1s、归零并返回；Z 需显式开启；最后单次 0.25s 租约，等待 0.8s 验证看门狗零速度输出 |
| `all` | 单项通过后的连续联测 | 顺序执行上述五组，仅在场地与前面阶段均通过后使用 |

当前源码的 WMOVE/WTRAVEL 参数为世界坐标绝对目标；BMOVE/BTRAVEL 为机体位移。任务已按这套实际语义生成目标。运行下面示例前，将 `MEASURED_V` 替换为本轮现场测得的数值（无引号，单位 V）；若固件已提供可靠的非零电压遥测，可删去 `external_battery_voltage`。

这里“世界坐标”指 START 后的 odom 坐标，不是 MCU 的原始 map 坐标。倾斜角检查也使用估计器 odom 中的相对横滚/俯仰，现场仍需观察真实车体浮态。

```bash
ros2 service call /auv/mission/execute uv_msgs/srv/ExecTask \
  "{task_name: basic_motion_test, params_json: '{\"stage\":\"set\",\"external_battery_voltage\":MEASURED_V}', timeout: 0.0}"
```

上下净空已确认且本池允许垂向移动时，单独运行下面这一轮；不满足条件就不执行。每次需要 Z 动作都显式写 `test_depth:true`，后续普通阶段仍默认关闭 Z。

```bash
ros2 service call /auv/mission/execute uv_msgs/srv/ExecTask \
  "{task_name: basic_motion_test, params_json: '{\"stage\":\"set\",\"test_depth\":true,\"external_battery_voltage\":MEASURED_V}', timeout: 0.0}"
```

完成并确认 PASS 后，用同样的格式把 `set` 依次改为 `bmove`、`wmove`、`travel`、`velocity`。`timeout:0` 表示使用测试自身每个位置动作的 20s 超时；不是整轮任务没有超时。`all` 会包含几十个动作，应留足观察和录制时间。

测试日志会给出步骤号、实测位姿和稳定误差。软件验收阈值是三维位置误差 ≤0.15m、航向误差 ≤6°；可作为首次小幅通路测试的参考，不能替代实际控制精度要求。用池边标尺/固定视频/外部测量确认真实位移；仅用同一个 odom 判断自身精度可能掩盖估计错误。

本任务结束后 status=3 为 DONE，status=4 为 ERROR；ERROR 的 `error_message` 包含失败阶段。其他历史任务不一定使用这些终态，不能把普通任务恢复 IDLE 当作成功。即使软件 PASS，也要记录实际轨迹是否正确、是否有异常电流/漏水/桨叶碰撞。

## 8. 停止、失联和看门狗验收

先在已经完成 hold、且有实体停机和回收条件的小范围水中验证软件停止。出现实际失控、进水、桨叶碰撞或人员进入危险区域时，优先操作你提供的应急开关。先在动力隔离状态验证开关确实切断推进器输出；若它仅关闭计算机，必须另外验证 MCU/电机板能停止，不把“电脑断电”当成动力已断。

```bash
ros2 service call /auv/mission/stop std_srvs/srv/Trigger "{}"
ros2 action send_goal /auv/basic_motion uv_msgs/action/BasicMotion \
  "{cmd_type: 7, axes: xyzrz, target: [0.0, 0.0, 0.0, 0.0], velocity_lease: 0.25}"
```

检查任务停止、零速度 setpoint、实际速度下降。零速度是速度环目标，仍可有控制输出，不等于关闭动力。

正式上锁：停止终端 B 的 hardware launch，并确认没有其他 heartbeat 发布者。仓库 MCU 应在最后心跳后约 1s 自动上锁；通过仍运行的 Agent 读取 `/zit6/state/status`，确认 `is_armed=false`、控制层级归零和实物停止。若不符合，不再测试运动，执行实体停机并查清固件与心跳源。

为避免下一次误解锁，重新启动 B 时仍使用 `test_disarmed.yaml`；不要以为把参数改为 0 能立即上锁当前已经解锁的系统。

速度租约由 `velocity` 阶段验证：检查 `/auv/hardware/zit6/cmd/setpoint` 的 `control_key=17 (0x11)` 机体速度模式，单次非零指令后约 `0.25s + 0.05s` 看门狗周期出现四轴零速度。任务在 0.8s 后验证最后一次输出；现场还要查看 legacy `/zit6/cmd/setpoint` 是否相同、MCU 和车体是否实际减速。

Agent/网络/计算机失联保护先在动力隔离夹具中验证，再决定是否进行水中验收。不要在自由游动时随意拔传感器或拔掉控制链路。失联后记录从最后心跳到上锁的时间；恢复通信时仍先禁止解锁。

## 9. 感知、推流、录制和规划联测

让整机保持未解锁或完成实体固定，重启终端 C 并设 `enable_ai:=true`。确认模型权重 `robotcup20260901.pt` 已提供，或在启动前通过 `UV_YOLO_MODEL` 指向实际权重。出现模型加载失败时，节点/空消息存在不能算检测通过。

```bash
ros2 topic echo /auv/perception/model_classes --qos-durability transient_local
ros2 topic echo /auv/perception/detections
ros2 topic echo /auv/perception/measurements
ros2 topic echo /auv/perception/tracks
```

把真实比赛目标依次放在前视/下视和左右目可见区域，记录类别映射、camera_name、检测置信度、遮挡/移出视野响应。用已知位置/距离核对目标射线/双目估计与 odom 坐标；静态目标移动观察点后不应无原因跳到另一个方向。首次无目标时不要求空数组中必须出现目标。

raw 模式检查当前 session 的 `camera/raw/front/`、`camera/raw/down/`、`frames.jsonl`、`bag/part_*/`、`logs/`、`manifest.json` 是否持续写入；默认目录 `/workspace/records/sessions/<时间戳>/`。检查磁盘空间和CPU/GPU负载，保留一段静止、一段小运动、一段停止的记录。

需要检查 go2rtc 录制时，单独短测 `record_mode:=go2rtc go2rtc_stream_mode:=both`。查看 front/down 原图与 annotated 视频、逐帧映射、manifest 中的时间对齐状态；没有新帧或映射 degraded 时排查，不能只看网页能打开。

Foxy 可能只生成 SQLite rosbag、视频/PNG 和日志，未生成 `session.mcap` 不一定是录制失败。正常 Ctrl+C 收尾后对选定记录执行 `ros2 bag info <bag分段路径>`；回放前先实体隔离推进器动力并停止真实控制和心跳进程，避免回放到真实控制链路。

若希望把规划节点也过一遍，在未解锁时单独启动：

```bash
ros2 launch uv_planning planning_launch.py enable_nav:=true
ros2 service call /auv/planning/navigate_to uv_msgs/srv/RunTask \
  "{task_name: '0.5,0.0,0.0', start: true}"
ros2 topic echo /auv/control/trajectory
```

核对 path frame=odom、起点为当前位姿、目标和航点有限且在池内。当前 navigator 只做规划/发布航点，没有驱动 BasicMotion 的路径跟随闭环，规划服务成功不能等同于“导航运动通过”。首轮水中闭环保持 `enable_nav=false`。

## 10. 测试记录和收尾

每项填写 PASS / FAIL / 未验收 / N/A，并保留失败前后的日志和视频。FAIL 项写清物理表现、软件表现、停止方式与下一次复测条件。

| 项目 | 结果 | 证据/数值/备注 |
|---|---|---|
| 密封、接线、电池、实体急停、回收绳 | | |
| MCU Agent、心跳、INS 导航就绪、错误标志 | | |
| 深度计、INS/IMU 轴向、DVL 锁底及速度轴向、USBL | | |
| M0 / M1 / M2 / M3 / M4 / M5 一对一方向和截止 | | |
| 舵机 1 / 2、灯 1 / 2 / 3 / off、推杆 | | |
| front 左/右、down 左/右、共享内存、CameraInfo、TF | | |
| hold / set / bmove / wmove / travel / velocity | | |
| 软件取消、速度租约、心跳失联上锁 | | |
| 模型映射、检测、观测、目标跟踪 | | |
| 推流、raw录制、go2rtc录制、日志/rosbag可读 | | |
| 可选规划节点（只验收规划） | | |

收尾顺序：停止任务并发零速度 → 停硬件心跳、确认 MCU 上锁 → 断开推进器动力 → 收回整机 → 正常停止任务/bringup/录制/Agent → 检查进水、接插件、温度和电量 → 保存记录。保留终端 C 录制到确认上锁之后，便于追踪停止过程。

## 11. 本次软件修改与验证范围

- 新增 `basic_motion_test` 单项 task 和配置，支持分阶段以及 all 联测；反馈/硬件条件不满足时拒绝，执行过程中持续检查，失败终止后续步骤，终态输出 DONE/ERROR。
- 修正真机 START 使用当前实测 map 位姿，避免已经从早期估计原点移到水中后，零 odom 目标对应旧位置/深度；仿真原点重置方式保留。
- 将任务停止调用改为公开的 `ClientGoalHandle.cancel_goal_async()`，并拒绝重叠任务请求。API 根据 [ROS 2 Foxy rclpy 原始接口](https://github.com/ros2/rclpy/blob/foxy/rclpy/rclpy/action/client.py) 核对。
- 修正短时 BODY_VELOCITY/零速度 Action 清掉在途位置目标句柄的问题，保留旧位置动作的取消状态，避免停止过程中把取消状态遮掉。
- 本次执行的是离线自动检查，包括反馈不健康拒绝、动作失败不继续、速度租约证据、电压/导航/范围检查、停止 API 和 START 深度/旋转原点回归；没有连接或驱动真实 AUV。

最终放行仍需现场实物验收。当前电压遥测缺失、应用舵机/灯/INS 指令适配、物理单推进器维护入口、实测 DVL 外参及实际固件版本都应在测试记录中明确状态。
