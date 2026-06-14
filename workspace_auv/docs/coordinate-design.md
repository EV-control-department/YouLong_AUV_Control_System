# YouLong AUV 坐标系设计文档

> 基于 AUV2026 项目 CoordinateSystem.py 的分析，用于指导本工作区的坐标系重构。

---

## 1. 坐标系定义 (NED)

| 符号 | 含义 | 方向 |
|---|---|---|
| X / North | 北 | 正 = 朝北 (世界), 正 = 前进 (机体) |
| Y / East | 东 | 正 = 朝东 (世界), 正 = 右移 (机体) |
| Z / Down | 深 | 正 = 下沉 (共用) |
| Roll (rx) | 横滚 | 右舷下沉为正 |
| Pitch (ry) | 俯仰 | 抬头为正 (NED) |
| Yaw (rz) | 偏航 | 0°=北, CW=正 (0→90°=北→东) |

**单位约定：** 内部存储和计算使用度 (degrees)，仅在与 ROS 消息 (rad) 或三角函数交互时转换。

## 2. 核心抽象：MotionState

AUV2026 用一个**统一的类**同时表示"位姿"和"变换"：

```
MotionState
├── vector (RobotAxis)     ← x, y, z, rx, ry, rz (位置+欧拉角)
└── h_matrix (float[16])   ← 4×4 齐次变换矩阵 (从 extract() 计算)
```

**生命周期：** 调用 `extract()` 把 vector 编码为 h_matrix，然后才能用于乘法/求逆。

> 当前工作区只有两个独立函数 `world_to_body()` / `body_to_world()`，只能处理 2D XY 旋转 (无 roll/pitch)。
> AUV2026 用完整 3D 齐次矩阵：`extract()` 计算 Rz·Ry·Rx → 4×4，`r_extract()` 从矩阵反解欧拉角。

## 3. 核心抽象：CoordinateSystems

```
CoordinateSystems
├── base (MotionState)           ← 参考坐标系的原点位姿
├── target_inbase (MotionState)  ← 目标在参考系中的坐标
└── target_inworld (MotionState) ← 目标在世界系中的坐标 (base*target_inbase)
```

**使用模式（来自 uv_automaton.py）：**

```python
# 1. 把世界系目标转换到机器人视角 (world → base)
self.robot.target_inworld.vector.x = backpoint["x"]   # 世界系目标
self.robot.target_inworld.vector.y = backpoint["y"]
self.robot.target_inworld.vector.z = backpoint["z"]
self.robot.target_inworld.vector.rz = backpoint["rz"]
self.robot.world2base()                                # 计算 target_inbase
x = self.robot.target_inbase.vector.x                  # 取结果
y = self.robot.target_inbase.vector.y
```

辅助函数：
- `Cs_Move(raw, move)` — raw 坐标系内叠加 move 偏移，返回新的 MotionState
- `Cs_Back(raw, back)` — raw 坐标系相对于 back 的逆变换

## 4. 设计要求 (新 Coordinate 类)

### 目标

- 一个类代表一个坐标系位姿
- 支持 world↔body 变换、坐标平移、旋转
- 节点间可直接构造 `Coordinate` 对象，无需关心内部矩阵

### 接口草案

```python
class Coordinate:
    """NED 坐标系位姿 (x, y, z, rx, ry, rz)，单位全部使用度。"""
    
    def __init__(self, x=0, y=0, z=0, rx=0, ry=0, rz=0)
    
    # 序列化
    def to_dict(self) -> dict          # {'x','y','z','rx','ry','rz'}
    def from_zit6_pos(data: list)      # 从 /zit6/state/pos Float32MultiArray
    
    # 变换
    def transform_to(self, target: Coordinate) -> Coordinate   # target 在 self 中的坐标
    def transform_from(self, local: Coordinate) -> Coordinate  # local (self系) → 世界
    
    # 偏移
    def offset(self, dx=0, dy=0, dz=0, drx=0, dry=0, drz=0) -> Coordinate
    
    # 2D 快捷 (roll=pitch=0)
    def world_to_body(dx_w, dy_w, yaw_deg) -> Coordinate
    def body_to_world(dx_b, dy_b, yaw_deg) -> Coordinate
```

### 使用示例

```python
# sim_bridge: 记录 AUV 当前位置
auv_pose = Coordinate(x=pos['x'], y=pos['y'], z=pos['z'], rz=pos['rz'])

# basic_motion: 计算目标点相对于 AUV 的位置
target_world = Coordinate(x=10, y=5, z=2, rz=45)
target_in_auv = auv_pose.transform_to(target_world)   # AUV 视角看目标

# minimal_control: 机体系偏移转世界系
step = Coordinate(x=1.0, y=0.0, z=0.0)                # 前进 1m
new_target = auv_pose.transform_from(step)             # 世界系新目标
```

## 5. 与现有代码的替换关系

| 旧 (coordinate_utils.py) | 新 (Coordinate 类) |
|---|---|
| `world_to_body(dx, dy, yaw)` | `pose.transform_to(target)` |
| `body_to_world(dx, dy, yaw)` | `pose.transform_from(local)` |
| `wrap_angle_deg/rad()` | `Coordinate.wrap()` 静态方法 |
