# Stonefish 理解笔记（YouLong AUV / workspace_sim）

> 本文记录对 Stonefish + stonefish_ros2 在**本仓库**中的实际工作方式的理解，
> 重点是「场景文件怎么写」「物理量从哪来」「为什么可视形状和物理形状会不一致」。
>
> 结论均来自本机源码与实测，来源标注在每节末尾：
> - Stonefish 库源码：`/home/origin/stonefish`（上游 patrykcieslak/stonefish，`Library/src`、`Library/include`）
> - 已安装库：`/usr/local/lib/libStonefish.so`
> - ROS 封装：`workspace_sim/src/stonefish_ros2`
> - 场景数据：`workspace_sim/src/stonefish_ros2/Data`
>
> 与官方文档重合的部分不重复展开，官方 rst 在
> `workspace_sim/src/stonefish_ros2/docs/`（parser / robots / sensors / actuators / running）。

---

## 0. 一句话总结

Stonefish 是「Bullet 刚体 + 海洋浮力/水动力 + OpenGL 渲染」的 C++ 库；
`stonefish_ros2` 只是把它包成一个 ROS 2 节点，用 **XML 场景文件（.scn）** 描述世界和机器人。
**渲染用的网格和物理用的网格是两个独立概念**——
不写 `<visual>` 时二者恰好是同一个网格，但这只是默认，不是保证。
机器人的浮力也不是「整个外形排水」，而是**每个部件独立按 `<physical>` 体积算**，
所以「看起来的样子」和「物理上排水/受力的样子」完全可以在空间上错开。

---

## 1. 组件与运行方式

| 组件 | 说明 |
|---|---|
| `libStonefish.so` | 引擎本体（物理 + 渲染 + 传感器模拟） |
| `stonefish_simulator` | GPU 版节点，**需要 X Display / OpenGL 窗口**才能渲染（相机传感器也依赖它） |
| `stonefish_simulator_nogpu` | 无头版，只跑物理，不渲染 |
| `stonefish_ros2` | 本仓库的 ROS 2 封装（workspace_sim） |

### 命令行参数（`main()` 直接解析 argv）

```
stonefish_simulator      <simulation_data> <scenario.scn> <rate> <windowW> <windowH> <quality>
                         argc < 7 直接报错退出
stonefish_simulator_nogpu <simulation_data> <scenario.scn> <rate>
                         argc < 4 直接报错退出
```

- `<simulation_data>`：`Data/` 目录的路径（代码里会补一个 `/`）。
- `<quality>`：`low` / `medium` / `high`，控制阴影、AO、大气、海洋、AA、SSR。
- 场景文件路径是**必填的完整路径**；`Data/` 目录只用于解析场景里引用的 mesh/texture。

证据：`workspace_sim/src/stonefish_ros2/src/stonefish_simulator.cpp:55-93`、
`stonefish_simulator_nogpu.cpp:55-69`。

### 本仓库的启动入口

仿真入口都在 `workspace_sim` 的 **`uv_sim_bringup`** 包（`workspace_auv` 自 T4 起不含任何
仿真依赖，只留 `uv_bringup real_bringup.py`）：

- `ros2 launch uv_sim_bringup sim_bringup.py`：仿真全栈（引擎 + sim_bridge + control + perception + 可选 nav/task）。
- `ros2 launch uv_sim_bringup sim_bringup.py enable_motion:=false enable_ai:=false`：只有引擎 + sim_bridge（等价于已删除的 `sim_core.py`）。
- `ros2 launch uv_sim_bringup hil_bringup.py`：HIL ⏸，sim_bridge 只透传相机、PID 交给实机 MCU。
- `ros2 launch uv_sim_bringup sim_bringup.py headless:=true simulation_rate:=100.0`：**无头物理仿真**
  （console 版引擎，无相机图像）；相机传感器需先用 `tools/strip_vision_sensors.py` 去掉。

`Data/` 目录的解析由 `uv_sim_bringup/uv_sim_bringup/stonefish_common.py:resolve_simulation_data()`
统一负责（T4 收敛了三份重复代码）：依次尝试 `$STONEFISH_DATA_DIR`、
`install/stonefish_ros2/share/stonefish_ros2/Data`、以及
`install/stonefish_ros2/share/stonefish_ros2/../../../..` 再拼 `src/stonefish_ros2/Data`
——即**直接指向源码目录**（引擎的 CMakeLists 不安装 `Data/`）。都找不到时打印警告，
场景名写错时会列出 `Data/` 里可选的 `.scn`。

---

## 2. 坐标系与变换约定

- **世界系就是 NED**：`x=北`、`y=东`、`z=下`，`z=0` 是水面，**z 越大越深**，负 z 在水面之上。
  引擎创建世界时 `setGravity(0, 0, +9.81)`，重力沿 +Z，与之一致。
- **机体系**：`x=前（艏）`、`y=右舷`、`z=下`；`yaw=0` 时艏指向 +x（北）。
- 角度：`.scn` 里用**弧度**；控制栈内部用度，只在 ROS 消息里转弧度。
- **rpy 顺序**：`ParseTransform()` 解析 `xyz` / `rpy` 后调用
  `Transform(Quaternion(rpy.z, rpy.y, rpy.x), xyz)`，
  而 `sf::Quaternion` 就是 `btQuaternion`，其 `(yaw, pitch, roll)` 构造函数走 `setEulerZYX`
  ⇒ **旋转矩阵 `R = Rz(yaw) · Ry(pitch) · Rx(roll)`**。
- 变换复合：部件在 base_link 下的位姿 = `compound_transform ∘ <physical><origin>`。

证据：`ScenarioParser.cpp` 的 `ParseTransform`；`StonefishCommon.h:47` `typedef btQuaternion Quaternion`；
`3rdparty/LinearMath/btQuaternion.h` `setEulerZYX`。

### 3D 软件导出的 mesh 通常是 z-up

NED 是 z-down，所以从 SolidWorks/Blender 导出的 z-up 网格在 Stonefish 里是**上下颠倒**的，
必须靠 `<origin>` 或 `<world_transform>` 翻转并平移。
本仓库的两种写法：

- 地形整体平移：`canyon_vision_dvl.scn` 的 `world_transform z=+8.0`（把 z-up 峡谷压到水下）。
- 部件逐个翻转：`xunyun_fixed.scn` 里 `xy_*.obj` 用
  `<origin rpy="3.1415926 0.0 3.14159" xyz="0 0 0.18"/>`（≈ 绕 Y 转 180° 再沿 z 移 0.18 m）。

---

## 3. `.scn` 场景结构

```xml
<scenario>
  <environment>
    <ned latitude longitude/>                 <!-- 仅 NED 原点的大地坐标，不是坐标切换 -->
    <ocean>
      <water density="1000.0" jerlov="0.20"/> <!-- jerlov=水体浑浊度 -->
      <waves height="0.0"/>
      <current type="uniform"><velocity xyz="0 0 0"/></current>
    </ocean>
    <atmosphere><sun azimuth elevation/></atmosphere>
  </environment>

  <materials>
    <material name="Neutral" density="1000.0" restitution="0.5"/>
    <friction_table> ...成对列出静/动摩擦... </friction_table>
  </materials>

  <looks>
    <look name="yellow" rgb="1.0 0.9 0.0" roughness="0.3"/>
    <look name="canyon" rgb="..." roughness="0.95" texture="canyon.png"/>
  </looks>

  <static name="..." type="model|plane|box|cylinder|sphere|torus">
  <dynamic name="..." type="..." physics="submerged" buoyant="true">
  <robot name="..."> 或 <include file="..."/>
  <cable .../>
</scenario>
```

- **材质（material）** 决定密度、恢复系数、摩擦；**外观（look）** 决定颜色/粗糙度/贴图。
  两个都必须存在，否则解析报错。
- `<static>` / `<dynamic>` 是自由刚体；机器人用 `<robot>` / `<include>`。
- `<include file="x.scn">` 配合 `<arg name= value=>` 做参数化子场景（机器人模型标配）。
- 机器人模型 `.scn` 顶层是 `<arg>` + `<robot>`，用
  `<world_transform rpy="$(arg robot_orientation)" xyz="$(arg robot_position)"/>` 出生。

证据：`ScenarioParser.cpp` 各 `Parse*` 函数；本仓库 `Data/*.scn` 实例。

---

## 4. 机器人 / 刚体模型结构

```xml
<robot name="XUNYUN" fixed="false" self_collisions="false">
  <base_link name="Vehicle" type="compound" physics="submerged">
    <external_part name="..." type="model" physics="submerged" buoyant="false">
      <physical>
        <mesh filename="x.obj" scale="0.001"/>   <!-- 物理网格（同时决定默认的渲染网格） -->
        <thickness value="0.0015"/>              <!-- 有值 ⇒ 按"薄壳"算体积 -->
        <origin rpy="..." xyz="..."/>            <!-- 物理网格在本部件坐标系下的位姿 -->
      </physical>
      <material name="Fiberglass"/>
      <mass value="1.5"/>                        <!-- 可选：覆盖算出来的质量 -->
      <look name="yellow"/>
      <compound_transform rpy="..." xyz="..."/>  <!-- 本部件在 base_link 下的装配位姿 -->
    </external_part>

    <internal_part name="BatteryCylinder" type="cylinder" physics="submerged" buoyant="true">
      <dimensions radius="0.065" height="0.3"/>
      <origin rpy="0 0 3.14159" xyz="0 0 0"/>
      <material name="Neutral"/>
      <mass value="7.2552"/>
      <look name="manipulator"/>
      <compound_transform rpy="0 1.571 3.14159" xyz="0.0495 0 0.1805"/>
    </internal_part>
  </base_link>

  <sensor name="..." type="camera|imu|dvl|pressure|odometry" rate="..."> ... </sensor>
  <actuator name="Thruster0" type="thruster"> <propeller .../> ... </actuator>
  <comm name="USBL" type="usbl" .../>
  <ros_subscriber thrusters="/auv/thrusters_cmd"/>
  <ros_publisher thrusters="/auv/thrusters_state"/>
  <world_transform rpy="..." xyz="..."/>
</robot>
```

### `external_part` vs `internal_part`

| | external_part | internal_part |
|---|---|---|
| 参与碰撞体 | ✅ 会加入 compound 的凸包集合 | ❌ 只参与质量/浮力 |
| 默认渲染 | ✅ | ❌（可加 `always_visible="true"` 强制显示） |
| 典型用途 | 外壳、支架、法兰 | 密封耐压舱（真正提供浮力的部分）、电池 |

**`internal_part` 是关键陷阱**：它是**看不见但真实存在**的物理体积，
很多 AUV 模型的浮力完全来自 internal_part，于是「看到的壳」和「排水的壳」不是同一个东西。
`xunyun_fixed.scn` 就是这种设计（见第 9 节）。

### 部件类型

`box` / `cylinder` / `sphere` / `torus` / `wing` / `model`。
其中 `box`、`cylinder` 等**必须**有 `<origin>`；`model` 的 `<origin>` 放在 `<physical>` 里面。
`model` 若再写一个 `<visual>`，则渲染和物理用两套网格——这是官方支持的「高模渲染 + 低模物理」写法。

证据：`ScenarioParser.cpp:1785 ParseSolid`，`typeStr == "model"` 分支在 `:2049`。

---

## 5. 质量、浮力与物理特性（最容易出错的地方）

### 5.1 体积：`thickness` 决定「实心」还是「薄壳」

`ComputePhysicalProperties()`：

```
thickness > 0 :  volume = Σ(三角形面积) × thickness      // 薄壳，只算表面
thickness ≤ 0 :  volume = Σ(有符号四面体体积) / 6        // 实心，按封闭网格
```

所以同一个 mesh，写不写 `<thickness>`，体积可能差一个数量级，
**而浮力正比于体积**。`xy_buoyancy_block.obj` 的例子：
实心体积 0.00433 m³，但带 `thickness=1.5mm` 时只有 0.00041 m³。

证据：`Library/src/utils/GeometryFileUtil.cpp:299 ComputePhysicalProperties`。

### 5.2 质量：显式 `<mass>` 只改质量，不改体积

`<mass value="...">` 会走 `ScalePhysicalPropertiesToArbitraryMass()`，
它只缩放 `mass` 和主惯量 `Ipri`，**`volume` 原封不动**。
⇒ 显式质量 ~ 密度被人为改掉了，但排水量还是几何体积。

证据：`SolidEntity.cpp:124`。

### 5.3 `buoyant` 的默认值是 `true`

`PhysicsSettings() : mode(SUBMERGED), collisions(true), buoyancy(true)`。
所以 **不写 `buoyant` 就是有浮力的**；要「进水/不排水」必须显式写 `buoyant="false"`。

证据：`Library/include/entities/Entity.h:60-69`。

### 5.4 Compound（`type="compound"`）的汇总规则

`Compound::RecalculatePhysicalProperties()`：

```
compoundMass   = Σ 所有部件的 mass
compoundCG     = Σ (部件 CG × mass) / M
compoundVolume = Σ 仅"可浮"部件的 volume        // ← 只累加 isBuoyant() 的部件
compoundCB     = Σ (可浮部件 CB × volume) / compoundVolume
compoundSurface= Σ 仅 external 部件的表面积
```

节点：base_link 的 `physics="submerged"` 且未写 `buoyant` ⇒ `isBuoyant()==true`，
但它自身没有网格，浮力完全等于「可浮子部件」之和。

证据：`Library/src/entities/solids/Compound.cpp:160 RecalculatePhysicalProperties()`。

### 5.5 浮力怎么算（三种状态）

`CheckBodyFluidPosition()` 用 **AABB 的 8 个角点**判断：

- 8 个角全在水下 → `INSIDE`
- 全在水上 → `OUTSIDE`
- 部分 → `CROSSING_SURFACE`

然后 `ComputeHydrodynamicForces()`：

| 状态 | 浮力 |
|---|---|
| `OUTSIDE` | 全部为 0 |
| `INSIDE` | `Fb = -volume · ρ · g`，作用点 `P_CB`（预先算好的浮心） |
| `CROSSING_SURFACE` | **逐三角面**积分实际水下体积；`reallisticBuoyancy` 仅对 `isBuoyant()` 的部件开启 |

`Ocean` 默认 `settings.dampingForces = reallisticBuoyancy = true`。
⇒ 跨水面时是**真实分段浮力**：物体一旦露出水面，排水体积就减少、浮力下降。

**由此可得一个直接结论：净浮力≈0 的物体永远不可能自己浮出水面。**
只有净浮力为正，平衡水线才会落在物体内部，露出水面的部分才是被浮力支撑的。

证据：`SolidEntity.cpp:1239 CheckBodyFluidPosition`、`:1799-1846 ComputeHydrodynamicForces`、
`Compound.cpp:291-437`、`Ocean.cpp:249-254`。

### 5.6 碰撞形状 = 凸包；水动力 = 等效椭球

- `Polyhedron::BuildCollisionShape()` 用**所有顶点建了 `btConvexHullShape`**，
  所以凹形/带孔的真实外形，物理上是一个「凸的实心块」。
- 水动力近似用 `ComputeFluidDynamicsApprox()` 生成的**等效椭球**（added mass 等）。

这两点本身就是「可见形状 ≠ 物理形状」的常见来源，但属于引擎的常规简化，
和「浮力体积跑到壳外」是两回事。

证据：`Polyhedron.cpp:92 BuildCollisionShape()`。

---

## 6. 渲染：可见形状和物理形状的关系

这是本节最需要记住的一点。

### 6.1 一个 `model` 部件的两套网格

`ParseSolid()` 的 `model` 分支：

```cpp
if (有 <visual>) {
    solid = new Polyhedron(name, phy,
                 visualMesh,  visualScale, visualOrigin,   // ← 渲染用
                 physMesh,    physScale,   physOrigin,     // ← 物理用
                 ...);
} else {
    // 单网格构造：graMesh = phyMesh，T_O2G = T_O2C = physOrigin
    solid = new Polyhedron(name, phy, physMesh, physScale, physOrigin, ...);
}
```

**默认（不写 `<visual>`）渲染网格和物理网格是同一个对象、同一个 origin**，
所以此时可视和物理是重合的——`xy_*.obj` 全是这种写法。

### 6.2 渲染用的是哪个变换

`SolidEntity::Render()`：

| DisplayMode | 变换 | 网格 |
|---|---|---|
| `GRAPHICAL`（默认） | `getGTransform()` ← `T_O2G` | `graMesh` |
| `PHYSICAL`（GUI 里切换调试用） | `getCTransform()` ← `T_O2C` | `phyMesh` |

单网格时两者相同；只有写了 `<visual>` 且 origin 不同才会分叉。
GUI 里按显示模式切换会看到凸包/物理网格，容易误判成「模型坏了」。

证据：`SolidEntity.cpp:228-265`、`Polyhedron.cpp:37-77`。

### 6.3 那为什么还会「可见 ≠ 物理」？

三种在真实模型里都会遇到的情况，按发生频率排序：

1. **浮力来自 `internal_part`**（默认不可见）。
   外壳 `buoyant="false"`，真正排水的是内部小圆筒。
   → 物理上的排水体积/浮心在空间上可以完全不在外壳里。
2. **`<visual>` 与 `<physical>` 用了不同网格/origin**（高模渲染 + 低模物理），这是特性，不是 bug。
3. **凸包/等效椭球简化**：碰撞和 added mass 与精细网格天然不同，一般可接受。

只有第 1 种会导致「AUV 永远浮不出水面」这种现象，因为它同时满足
「浮力小/净浮力≈0」和「浮力体积位于壳外」。见第 9 节实例。

---

## 7. 传感器与 ROS 接口

- 每个传感器用 `<ros_publisher topic="/..."/>` 直接把数据发到指定话题；
  机器人整体用 `<ros_subscriber thrusters="/auv/thrusters_cmd"/>`、
  `<ros_publisher thrusters="/auv/thrusters_state"/>` 收/发推进器矩阵。
- **节点名/机器人名（`robot_name`）不影响话题名**，话题是显式写死的。
  所以 `sauvc_2026_finals.scn` 用 `robot_name="GIRONA500"` 去 include `xunyun_fixed.scn`
  也能正常工作——代码里没人依赖这个名字。
- 本仓库当前机器人模型的固定话题（`girona500auv_full.scn` 与 `xunyun_fixed.scn` 完全一致，
  所以两者可互换）：

```
/auv/odometry        30 Hz   nav_msgs/Odometry（真值；pose=NED 世界系，twist=机体系速度/角速度）
/auv/imu            200 Hz   + 噪声（对齐 AQUA-SLAM 的噪声密度，见 2026-09-15 T1）
/auv/dvl             20 Hz   + /auv/dvl_altitude（机体系速度，见下方 DVL 安装说明）
/auv/pressure         5 Hz
/sim/front_cam/left|right/image_color   20 Hz 1280x960
/sim/down_cam/left|right/image_color    30 Hz 1920x1080（xunyun_fixed）
/auv/thrusters_cmd  (订阅)   Float64MultiArray，6 个推进器
/auv/thrusters_state(发布)
```

- **DVL 安装（2026-09-15 与实机对齐）**：`<origin rpy="0 0 0" xyz="-0.375 0 0.1"/>` +
  `<specs beam_angle="30.0" beam_positive_z="true"/>`。实机 DVL 是「yaw=0 安装 + 波束斜置 45°、
  离竖直 30°」，所以仿真里传感器系与机体系对齐，波束在艇体系内为 45/135/225/315 方位。
  旧配置 `rpy=(3.1416 0 -0.785)` + 默认 `beam_positive_z=false` 会让 Stonefish 把速度按
  传感器系发出来——相对机体系转了 45° 且 y/z 取反，`basic_motion` 的沿航向速度
  `v_along = V·cos(2α+45°)`（α=22.5° 时恒为 0）。回归测试见
  `src/uv_sim/test/test_scene_sensors.py`，端到端检查见 `tools/check_dvl_frame.py`。

- 数据流：`Stonefish → /auv/* → sim_bridge → /zit6/state/* → basic_motion → PoseInfo`。
  GPU 版是相机出图的前提；无图形环境要 `xvfb-run`。

- **无头（nogpu / console）模式的两个坑**：
  1. 控制台版**不支持相机传感器**：任何带 `<sensor type="camera">` 的场景都会
     `Cameras not supported in console mode!` 然后解析失败。用
     `tools/strip_vision_sensors.py` 生成去相机的临时场景副本即可跑物理
     （DVL/IMU/odometry/pressure 都不需要渲染）。
     该工具 2026-09-16 起还有 `--keep-camera SUBSTRING`（例：`--keep-camera down_cam`
     = 只留下视一对、丢掉前视一对），**用来判别 GPU 出图慢的瓶颈**：若保留的那对速率近似翻倍，
     说明成本 ∝ 相机数 × 分辨率（每帧一次 FBO 渲染 + `glMapBuffer` 阻塞回读），
     而**与 `rendering_quality` 关系不大**（用户实测降画质帮助有限）。
     ⚠️ 量图像频率时注意：Stonefish 的相机话题是 `create_publisher<Image>(topic, 10)`（**RELIABLE, depth 10**），单帧 6 MB 量级；用 `ros2 topic hz` 或 best-effort/depth1 的 Python 订阅者会**严重低估**（实测 1.93 Hz，而由同一批回调驱动的 `/auv/down_cam/stitched` 却是 19 Hz ⇒ 单眼实际约 9.5 Hz）。交叉验证口径：`/auv/{front,down}_cam/{left,right}`（sim_bridge 从回调里转发）、`ros2 topic bw` ÷ 帧大小、写者侧计数。
  2. **`simulation_rate` 必须给正数**（默认 `100.0` 即可）。⚠️ **2026-09-16 更正**：本节此前写
     「传 `0` 走实时模式才会发布 ROS 话题」，**实测相反**——`rate=0` 时仿真线程不再推进物理，
     **一条 ROS 消息都不发**。根因：`SimulationManager` 用
     `ssus = (uint64_t)(1000000.0/stepsPerSecond)`（`Library/src/core/SimulationManager.cpp:582`），
     `rate=0` 是浮点除零→转 `uint64` 属未定义行为，于是 `AdvanceSimulation()` 里的
     `if(deltaTime < ssus) SimulationClockSleep(ssus - deltaTime)` 几乎永不返回。
     旧结论把两件事混为一谈：`setCallSimulationStepCompleted(timeStep_==0)` 里的 `timeStep_`
     是 **App 的固定步长**，本构建里从未被赋值（`setSimulationRate()` 无人调用）恒为 0，
     与命令行 `rate` 无关；命令行 `rate` 只进 `stepsPerSecond`→`ssus`。

     实测（`canyon_headless.scn`，`headless:=true`，默认 `simulation_rate:=100.0`）：
     `/auv/odometry` **30.0 Hz**、`/auv/imu` **100.2 Hz**、`/zit6/state/pos` **30.1 Hz**、
     `/zit6/state/vel` **60.1 Hz**；同一条命令改成 `simulation_rate:=0.0` 则全部为 0（话题存在但无数据）。
     注意 IMU 场景里写的是 200 Hz，实测只有 ~100 Hz——**发布速率受物理步长（100 Hz）上限约束**；
     要让 IMU 真跑 200 Hz 需要 `simulation_rate:=200.0`（T6 移植 AQUA-SLAM 时要注意这一点）。

---

## 8. 本仓库的约定与已知坑

- **NED 深度别按 z-up 推**：新写 `.scn` 时正 z 是「更深」，负 z 在水面之上。
  `canyon_vision_dvl.scn` 初版就踩过这个坑（见 `CLAUDE.md`）。
- **launch / Python 改动**：源码 `launch/` 与 `build/`、`install/` 之间是软链接
  （symlink install），改 launch 一般**不用重新 `colcon build`**；
  但 msg/srv/action 和 C++ 必须重编。
- **窗口分辨率**：由 bringup 传 `arguments=[..., '2560', '1440', ...]`
  （对应 `windowW/windowH`）控制；`stonefish_simulator.launch.py` 的默认值是 1600×1200。
  窗口按像素创建，超过当前显示模式会被裁切。
- **石斑鱼/文件名**：`xunyun.scn` 是无相机、含 TopPlate 的旧版；
  `xunyun_fixed.scn` 是去掉了 TopPlate、加了 4 个相机、并把入水朝向参数化的现役版。
  新场景应 include `xunyun_fixed.scn`。
- **不要迷信文档**：`docs/node_parameters.md` 说 `gpu` 参数未使用；
  `docs/debug_guide.md` 的 `task_map` 列表滞后于 `task_runner.py`。
  权威顺序：源码 > `CLAUDE.md` > 其它文档。
- **`nav_msgs/Odometry` 的 twist 是机体系**：`pose` 属于 `header.frame_id`（这里是
  `world_ned`），`twist`/角速度属于 `child_frame_id`；Stonefish 在填消息前已经乘过
  `R_sensor←worldᵀ`（`Odometry.cpp:59,65`），而 odometry 传感器就装在艇体原点
  （`rpy≈0`），所以 twist 就是机体速度，**不能再按 yaw 转一次**。
  2026-09-15 修掉的 B12 就是踩了这个坑：`sim_bridge` 的两条速度通道
  （`/zit6/state/vel` 的 `_odom_cb`、`/zit6/sim/nav` 的 `_sim_nav_odom_cb`）各多乘了
  一次 `R(−ψ)`，ψ=90° 时纯 surge 0.3 m/s 会被报成 `(0, −0.3)`；而固件对
  `/zit6/sim/nav` 的契约明确要求 `vel_body`。
  现在约定集中在 `sim_bridge.body_twist_from_odometry()`，回归测试
  `src/uv_sim/test/test_odometry_frame.py`（ψ=90° 合成消息）钉住它。
  **判别口诀：pose 跟 header、twist 跟 child。**

---

## 9. 案例：`xunyun_fixed.scn` 的「可见形状 ≠ 物理形状」

这是本文最具体的实测案例，也解释了「AUV 任何一部分都无法露出水面」的成因。

### 9.1 实测：把每个部件变换到 base_link 系

用 `R = Rz(yaw)Ry(pitch)Rx(roll)`、`X = T_compound ∘ T_physical` 计算 8 个角点得到包围盒：

```
=== 可见部件（external_part） ===
BuoyancyBlock    z[-0.040, 0.010]   ← 顶部
RightSidePlate   z[-0.043, 0.210]
LeftSidePlate    z[-0.043, 0.210]
ForwardBracket   z[ 0.010, 0.180]
AftBracket       z[ 0.010, 0.180]
CoreEnclosure    z[ 0.009, 0.139]
InsBasePlate     z[ 0.048, 0.180]   ← 底部
BatteryPod       z[-0.033, 0.053]
可见总包围盒     x[-0.240, 0.240]  y[-0.185, 0.185]  z[-0.043, 0.210]

=== 不可见的浮力部件（internal_part，cylinder，buoyant=true）===
BatteryCylinder    z[ 0.115, 0.246]
PortCylinder       z[-0.235,-0.104]
StarboardCylinder  z[-0.235,-0.104]
```

（单位 m，**z 越小越靠上**，z=0 是 base_link 原点。）

### 9.2 结论

- 可见外形占据 `z ∈ [-0.043, 0.210]`。
- 两个侧向浮力圆筒占据 `z ∈ [-0.235, -0.104]`，**比可见外形顶端还高 0.191 m**；
  x 方向也超出（±0.275 vs ±0.240），y 方向超出（0.240 vs 0.185）。
- 电池圆筒则**比可见外形底部低 0.036 m**。

也就是说，**真正排水的体积整体"套"在可见外壳外面**，像给 AUV 套了一个更大的隐形壳。

### 9.3 为什么表现为「无法露出水面」

1. 浮力只来自这 3 个圆筒（外壳 `buoyant="false"`，不排水）。
   圆筒总体积 ≈ `0.01725 m³`，对应排水质量 ≈ `17.25 kg`。
2. 全机质量 ≈ 外壳 5.0 kg + 圆筒 12.255 kg + 6 个导流罩（材质密度 1000，自质量≈自排水）≈ `17.7 kg`。
   ⇒ **净浮力 ≈ 0，全机正好中性浮力**。
   （尚未计入推进器/螺旋桨等附加质量；计入后只会更偏"下沉"，不改变结论。）
3. 中性浮力下唯一平衡点是**完全浸没**；一旦有部件出水，浮力下降、净力向下。
4. 更糟的是，最先出水的是**看不见的圆筒**（比外壳高 19 cm）。
   于是从外部看：水面处什么也没有，可见的 AUV 整体潜在水下——
   正是「visible shape 与 physical shape 不一致，任何部位都上不了水面」。

### 9.4 实际修复（2026-09-15 已完成）

按「可见外形 == 物理外形、微微正浮力、顶部整块是浮力材」的目标重构了 `xunyun_fixed.scn`：

1. **删除 3 个 `internal_part` 浮力圆筒** —— 物理形状不再伸出可见外壳之外。
2. **8 个 `external_part` 全部 `buoyant="true"`** —— 浮力来自可见外壳网格本身。
3. **顶部 `BuoyancyBlock` 作为浮力材**：4.33 L 排水，质量仅 0.25 kg（≈58 kg/m³ 泡沫）。
4. **质量按"浮心在重心之上、CG_x 与 CB_x 对齐"求解**，整机微微正浮力。

最终参数（引擎实测确认）：

| 量 | 值 |
|---|---|
| 排水体积 | 12 747.1 cm³（12.747 kg） |
| 总质量 | 12.418 kg |
| 剩余浮力 | **+0.330 kg**（微微正浮力） |
| 惯量 | 0.563 / 0.711 / 0.957 kg·m²（正常） |
| CB − CG | 浮心高 1.65 cm（稳定） |

**上浮实测**（无头仿真，从 1 m 深处自由释放）：

```
t≈ 8s  z=0.540      t≈40s  z=0.147
t≈16s  z=0.440      t≈48s  z=0.050
t≈24s  z=0.341      t≈56s  z=0.042
t≈32s  z=0.243      t≈80s  z=0.039  ← 稳定，±0.5 mm
```

约 60 s 后稳定在 `z ≈ 0.039 m`（理论预测 0.036 m），即水面刚好切在浮力材顶端，
约 4 mm 泡沫露出水面——正是"+0.33 kg 剩余浮力"对应的露出量。

**质量分配（kg）**：BuoyancyBlock 0.250 / RightSidePlate 1.536 / LeftSidePlate 1.536 /
ForwardBracket 1.916 / AftBracket 0.211 / CoreEnclosure 3.076 /
InsBasePlate 0.755 / BatteryPod 2.674 / 6×duct 0.464（按体积自动）。

### 9.5 修复途中踩到的引擎坑：实心体积路径会产生 NaN 惯量 ⚠

这是本次最重要的新发现，单独记录。

**现象**：把外壳从「薄壳（`<thickness>`）」改成「实心（去掉 `<thickness>`）」后，
引擎日志出现

```
Built multibody link XUNYUN/Vehicle (mass: 12.417; inertia: -nan, -nan, -nan; volume: 12747.1)
[WARN] Overflow in AABB, object removed from simulation
```

质量、体积都对，唯独主惯量是 NaN，物体会被 Bullet 移出仿真。

**定位**（逐一变量隔离，其余改动都不触发）：

| 变体 | 惯量 |
|---|---|
| 原模型（薄壳） | 1.064, 1.146, 1.242 ✓ |
| 只删 `internal_part` | 0.428, 0.654, 0.766 ✓ |
| 只把 external 改 `buoyant="true"` | 1.064, 1.146, 1.242 ✓ |
| **只去掉 `<thickness>`** | **-nan** ✗ |
| 只改 `<mass>` | 1.179, 1.271, 1.339 ✓ |

且是**网格相关**的：逐部件测试中 `AftBracket` 单独去掉 `thickness` 就 NaN，
`BuoyancyBlock` 去掉却正常。

**根因**：`GeometryFileUtil.cpp` 的实心路径里，主轴求解用
`FindInertialAxis()` 对 `(I - λI₃)` 的行向量做叉积再归一化——
当叉积长度为 0（退化惯量矩阵）时 `normalized()` 返回 NaN；
NaN 的 `Irot` 会污染 `T_CG2C`，进而让 `ComputeEllipsoidalApprox()`（MVAE 包围椭球）
算出 NaN 的附加质量 `aMass`，而 `Compound` 汇总时用的是
`getAugmentedMass()` = `mass + (aMass.x+y+z)/3` ⇒ 整个刚体惯量变 NaN。

**规避办法（已采用）**：保留 `<thickness>` 走薄壳路径（对这批网格长期稳定），
但对每个部件取

```
thickness = 该网格封闭体积 / 该网格表面积
```

使 `volume = S × t` **正好等于**封闭体积。于是：
- 排水体积仍然精确等于可见网格体积（可视 == 物理的目标不变）；
- 完全绕开实心路径的 NaN；
- 质量仍由 `<mass>` 显式给定，不受影响。

各部件厚度（mm）：BuoyancyBlock 15.899 / RightSidePlate 4.286 / LeftSidePlate 4.286 /
ForwardBracket 3.299 / AftBracket 3.324 / CoreEnclosure 25.292 /
InsBasePlate 6.269 / BatteryPod 18.110。6 个 duct 本来就没写 `thickness`（实心），
在原模型里就工作正常，保持不变。

> 注意：薄壳路径下部件的 CG/浮心是**面积加权质心**（不是体积质心），
> 配重计算必须用面积质心，否则 CB 会差好几厘米。
> 另外写 `.scn` 的 XML 注释里**不能出现 `--`**（例如长横线分隔行），
> 严格的 XML 解析器会直接报 not well-formed。

> 关键判据（更新）：**先看 `<internal_part>` 在不在可见包络内，再算净浮力是否为正，
> 最后确认惯量不是 NaN。** 三条都满足，模型才算"物理上正确"。

（本节的数值由 `Data/*.obj` 的顶点 + `.scn` 里的 rpy/xyz 计算得到，可复算；
最终以引擎日志 `Built multibody link ...` 一行为准。）

---

## 10. 快速排查清单

写/改场景时按顺序过一遍：

1. **坐标系**：这组 xyz 是 NED（z 向下）吗？mesh 是 z-up 还是 z-down？要不要翻转？
2. **两套网格**：写了 `<visual>` 吗？没写时渲染==物理；写了就要同时检查两个 `<origin>`。
3. **体积口径**：部件该是实心还是薄壳？`<thickness>` 写了会不会把排水量算没了？
   ⚠ 实心路径对某些网格会让主惯量变 NaN（见 9.5），保险做法是
   `thickness = 封闭体积/表面积` 走薄壳路径。
4. **浮力来源**：谁是 `buoyant="true"`？它是否在可见外形内部？尺寸够不够？
5. **净浮力**：`Σ(可浮体积×1000)` 与 `Σ质量` 谁大？差多少？要中性、正、还是负？
6. **稳定性**：浮心 CB 是否高于重心 CG？（Compound 会按质量/体积自动加权）
7. **质量覆盖**：显式 `<mass>` 只改质量、不改体积，配重时别把两者搞混。
8. **碰撞**：凹形外壳会被当成凸包，注意别在凹处卡住。
9. **话题**：模型给的话题和下游节点（sim_bridge / vision）约定的一致吗？
10. **渲染环境**：有没有 X Display？没有就用 nogpu 或 `xvfb-run`。

### 常用命令

```bash
# 语法/物理快速验证（无头，只跑物理）
cd workspace_sim && source install/setup.bash
ros2 launch stonefish_ros2 stonefish_simulator_nogpu.launch.py \
    simulation_data:=$PWD/src/stonefish_ros2/Data \
    scenario_desc:=$PWD/src/stonefish_ros2/Data/canyon_vision_dvl.scn

# 完整仿真
cd workspace_auv && source install/setup.bash
ros2 launch uv_sim_bringup sim_bringup.py scenario_desc:=canyon_vision_dvl.scn

# 看真值位姿/深度
ros2 topic echo /auv/odometry
ros2 topic echo /auv/pressure
```

---

## 附：关键源码索引

| 主题 | 文件 | 位置 |
|---|---|---|
| 参数解析 | `src/stonefish_simulator.cpp` | `main():55` |
| 场景解析入口 | `Library/src/core/ScenarioParser.cpp` | `ParseSolid():1785`、model 分支 `:2049` |
| 变换/rpy | 同上 | `ParseTransform()` |
| 几何/体积 | `Library/src/utils/GeometryFileUtil.cpp` | `ComputePhysicalProperties():299` |
| 单/双网格 | `Library/src/entities/solids/Polyhedron.cpp` | 构造函数 `:36` / `:74` |
| 碰撞形状（凸包） | 同上 | `BuildCollisionShape():92` |
| 质量覆盖 | `Library/src/entities/SolidEntity.cpp` | `ScalePhysicalPropertiesToArbitraryMass():124` |
| 流体位置判定 | 同上 | `CheckBodyFluidPosition():1239` |
| 浮力/水动力 | 同上 | `ComputeHydrodynamicForces():1799` |
| Compound 汇总 | `Library/src/entities/solids/Compound.cpp` | `RecalculatePhysicalProperties():160` |
| 跨水面浮力 | 同上 | `ComputeHydrodynamicForces():291` |
| 默认物理参数 | `Library/include/entities/Entity.h` | `PhysicsSettings:60` |
| 默认水动力开关 | `Library/src/entities/forcefields/Ocean.cpp` | `:249-254` |
| 渲染/显示模式 | `Library/src/entities/SolidEntity.cpp` | `Render():228` |
