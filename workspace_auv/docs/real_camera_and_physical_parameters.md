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
- 转盘尺寸：最外圆直径 230mm、外环内沿直径 200mm、中心内圈外沿直径 35mm、四根条幅各宽 20mm；对应半径 0.115/0.100/0.0175m。细棍根相对前视双目中点为 `(0,-0.09,0)m`，沿机体 `+x` 延伸 0.16m。双目中点的 Body 名义位置需装配复核；棍半径、允许插入深度、黄标相位和目标累计角度仍待测。`last.pt` 只有整盘 `wheel` 类、没有黄色标签类；转盘视觉现用 HSV 提取黄标并用前视 SGBM 测盘心/法向，实物测试前不得认为接触几何已验证。详见 [turntable_task.md](turntable_task.md)。
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
