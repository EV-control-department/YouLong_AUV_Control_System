# 开发日志 — 管道巡线任务

## 2026-07-21

### LineFollower 子对象抽取
- 从 `task_runner.py` 中提取 `LineFollower` 类到独立文件 `line_follower.py`
- `task_runner.py` 精简为只负责调度，`_task_follow_line()` 创建/销毁 LineFollower
- LineFollower 仅在 `follow_line` 任务执行期间存活，不影响其他任务

### 双通道 PID 巡线控制
- 横向 PID：`center_error` → `dy`（机体系横移，推回管道中心）
- 偏航 PID：`heading_error_deg` → `dyaw`（旋转对齐管道方向）
- 前进步长自适应：修正量越大前进越慢
- 修复控制映射 bug：之前 center_error 错误映射到 dyaw

### YOLO 丢帧恢复
- 连续丢帧 ≤ 50 帧 → coast 盲跟模式（最后有效 heading，不更新 I/D）
- 超过阈值 → 进入搜索模式

### 搜索策略
- 8 方向扩展方框螺旋搜索，纯平移 BMOVE xy，不旋转
- 初始框 0.02m，每圈扩展 0.30m，最大 2.0m

## 2026-07-22

### 三角形标记处理 (class_id=5)
闭环接近 100 次 → 下沉 0.5m → 上升 0.5m → flag=1 抑制
抑制恢复条件：三角形 bbox 中心进入图像上 1/3

### 正方形标记处理 (class_id=6)
闭环接近 100 次 → 自转 360° (3×120° BMOVE rz) → flag=1 抑制
抑制恢复条件：正方形 bbox 中心进入图像上 1/3

### 双目三角测量
不再依赖 position 节点的 `ObjectPositionArray`，LineFollower 内部实现下视双目三角测量：
**实现细节：**
- 相机参数与 `position.py` `CAM_PARAMS` 保持一致
- 左右目像素 → camera ray → body NED → world ray → 公垂线中点交会
- 订阅 PoseInfo 获取完整 6-DOF 姿态 (robot_x/y/z + roll/pitch/yaw)
- ZYX 欧拉角 → 旋转矩阵将射线从机体转到世界系

**新增模块：**
- `_DOWN_HFOV/_FX/_CX` 等相机内参常量
- `_euler_to_rotation_matrix()` — 欧拉角→旋转矩阵
- `_ray_intersection_midpoint()` — 两射线公垂线中点
- `_stereo_pair()` — 左右目对 class_id 同步匹配
- `_pixel_to_world_ray()` — 像素→世界射线
- `_triangulate_object()` — 双目三角测量主方法

### 标记处理对比

| 步骤 | 三角形 (class=5) | 正方形 (class=6) |
|---|---|---|
| 1 | 100 次双目接近 | 100 次双目接近 |
| 2 | 下沉 0.5m → 上升 0.5m | 自转 360° (3×120°) |
| 3 | flag=1 抑制 | flag=1 抑制 |

两者共享完全相同的触发条件（下 4/5）、抑制恢复条件（上 1/3）、抑制逻辑。

### 任务完成计数 + 提前结束
- 新增 `_triangle_count` / `_square_count` 两个计数器
- 每完成一次三角形下潜或正方形旋转，对应计数器 +1
- 丢线时检查：若已完成 ≥4 个任务 → 返回 `True`（巡线任务成功结束）
- 不足 4 个任务时仍走原有搜索流程

### 文档更新
- CLAUDE.md：新增标记处理章节 + 双目三角测量架构说明
- debug_guide.md：新增标记处理流程表、常见问题、日志关键字
- perception.md：新增 YOLO-Seg 管道分割章节 + LineState 数据流

## 2026-07-23

### basic_motion WMOVE/TRAVEL 语义重构
**问题：** WMOVE 和 TRAVEL 的 target 参数同时存在偏移量和绝对坐标两种语义，在 `p`(pose) 和 `t`(target) 之间混用，导致连续 WMOVE 累积误差、WTRAVEL 偏移→绝对→偏移冗余转换。

**重构内容：**
- **WMOVE target 改为绝对坐标**：`wmove*` 方法不再做 `t + dx` 叠加，直接将绝对坐标传入 `_step_move_world`
- **WTRAVEL target 改为绝对坐标**：`_travel_world` 参数从 `(dx_w, dy_w, dz)` 改为 `(x_w, y_w, z, rz)`，接受绝对世界坐标
- **新增 rz 参数**：TRAVEL 的 `rz` 定义任务完成后的最终朝向。第一阶段转向用 `atan2(dy,dx)` 算方向角，最终定位用 `rz`
- **删除 TRAVEL 便捷方法**：移除 `wtravel*/btravel*` 系列 wrapper（无外部调用方，action 回调直接调 `_travel_world`）
- **运动基准点统一**：WMOVE/WTRAVEL 未指定轴从 `t` 补齐，BMOVE/BTRAVEL 从 `p` 补齐
- **STEP_Y 0.2→0.4**：放宽椭圆横向步长
- **TRAVEL 距离阈值 0.03→0.01m**：更精确的短距判断

### task_runner 同步适配
- WMOVE/WTRAVEL 任务不再计算 `dx = p['x'] - _cmd_x`，直接发绝对坐标

### LineFollower 任务计数 + 丢线容限
- 丢线容忍帧数默认值 50→100
- 巡线任务完成计数 + ≥4 丢线即结束（详见上节）
