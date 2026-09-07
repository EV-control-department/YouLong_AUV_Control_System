# 视觉融合系统改造方案：BBox-only 多相机多视角几何融合 V2

> 本文是 `vision_fusion_system_bbox_multimodel_v1.md` 的 V2 版本。
>
> V2 的核心变化是：**前视定位彻底取消“左右检测先配对 → 双目三角化 / 单目多视角”两条主路径的区分。**
> 左相机、右相机以及未来可能增加的任何相机，都只产生统一的 `ObjectObservation`；所有时刻、所有相机的 bbox 通过同一套已知尺寸物体投影模型、多模型拟合、clutter 与独占分配来完成实例发现和三维定位。
>
> 本版明确 **不使用 segmentation、不提取角点、不在 bbox 内做传统 CV**；同时暂时 **不考虑机器人位姿、相机外参的不确定度**，采集时相机位姿视为准确值。

---

## 1. V2 设计目标

V2 要解决的不是“怎样把一个已经分好 ID 的目标估得更准”，因为当前单 ID 的 bearing / LM 估计已经很稳定；真正要解决的是：

1. 多个同类别目标同时存在时，不让一条错误 observation 被多个目标重复使用；
2. 不让一条错误射线通过大量两两组合制造出一堆伪 XYZ，从而形成假的高密度簇；
3. 不让后来的错误聚类重新解释并挤占已经正确的历史观测；
4. 利用 bbox 的中心、线度和宽高比，增强“两个 measurement 是否可能来自同一个物体”的判别；
5. 不要求预先知道同类别目标数量 `K`；
6. 新目标必须有足够独立几何证据才能出生，单个假框不能直接形成新实例；
7. 已确认目标的位置只在有新几何信息或关联结构变化时重新优化；
8. 前视左右相机在算法上完全等价于任意两个不同视角，不再维护“stereo estimator”和“multi-view estimator”两套逻辑；
9. 保留当前 `odom/NED`、时间对齐、相机标定、下视定位与 ROS 输出体系，V2 主要重构前视实例发现和定位。

最终希望前视形成：

```text
任意 camera / 任意 timestamp 的 YOLO bbox
                    │
                    ▼
             ObjectObservation
                    │
                    ▼
        semantic-class observation pool
                    │
          ┌─────────┴─────────┐
          ▼                   ▼
   已知尺寸几何模型       未知尺寸 fallback
          │                   │
          ▼                   ▼
 single-bbox proposal     bearing proposal
          └─────────┬─────────┘
                    ▼
          object model hypotheses
                    │
                    ▼
   所有 raw observations 对所有模型评分
                    │
                    ▼
     global exclusive assignment + clutter
                    │
                    ▼
           robust Huber / LM refine
                    │
                    ▼
      model selection / birth / merge / split
                    │
                    ▼
        persistent instance_id + X,P
```

---

## 2. V2 最重要的架构原则

### 2.1 前视算法中取消“双目配对”

前视定位主链不再执行：

```text
left detections
      │
      ├── _match_detections() ── right detections
      │
      ▼
 stereo triangulation XYZ
      │
      ▼
 front stereo estimator
```

也不再先判断：

```text
L1 ↔ R3
L2 ↔ R1
```

然后再进行三维处理。

V2 改为：

```text
left detection  at t0 ──→ observation #100
right detection at t0 ──→ observation #101
left detection  at t1 ──→ observation #102
right detection at t1 ──→ observation #103
...
```

它们统一满足：

\[
z_i = h_{\mathcal M}(X_k, T_{cw,i}) + \epsilon_i.
\]

这里：

- `camera_id` 决定相机内参和相机到机体外参；
- `capture_timestamp` 决定采样时机器人位姿；
- `T_cw,i` 对每条 observation 独立计算；
- 左右相机只是两个不同光心；
- 同时刻左右 observation 不需要知道彼此是否“一对”。

### 2.2 取消配对不会丢掉双目 baseline

左右相机光心：

\[
o_L \neq o_R.
\]

因此即使完全不计算 disparity、不显式三角化，两个 bbox 对同一个目标状态的 Jacobian 仍然不同：

\[
H = J_L^T R_L^{-1}J_L + J_R^T R_R^{-1}J_R.
\]

空间 baseline 已经自然进入信息矩阵。

所以 V2 不是“不用双目硬件”，而是：

\[
\boxed{\text{双目硬件} \rightarrow \text{两个普通相机视角}}
\]

前视定位器本身变成：

\[
\boxed{\text{multi-camera multi-view object estimator}}
\]

以后增加第三个前视相机也不需要改算法，只需新增 `camera_id + calibration + extrinsic`。

### 2.3 所有 raw observation 只保存一次

一个 YOLO detection 必须拥有唯一：

```text
observation_id
```

所有：

- candidate；
- hypothesis；
- model；
- track；

只保存 `observation_id` 引用，不能复制它形成“新的独立证据”。

核心原则：

\[
\boxed{\text{one raw detection = one independent vote at most}}
\]

---

## 3. 坐标系与时间模型

V2 沿用当前 NED 约定：

- `odom/NED`: `x=North, y=East, z=Down`；
- `body/FRD`: `x=Forward, y=Right, z=Down`；
- `camera_optical`: `x=Right, y=Down, z=Forward`。

对于 observation `i`：

1. 使用 detection 自己的 `capture_timestamp`；
2. 在位姿 buffer 中插值得到 `T_wb(t_i)`；
3. 根据 `camera_id` 取得 `T_bc(camera_i)`；
4. 得到：

\[
T_{wc,i}=T_{wb}(t_i)T_{bc,i}.
\]

以及：

\[
o_i = t_{wc,i},
\qquad
R_{cw,i}=R_{wc,i}^T.
\]

V2 不要求：

\[
t_L=t_R.
\]

只要求每条 observation 有正确的自己的 pose。

---

## 4. 统一原始 Observation

建议前视主路径只保留一个核心结构：

```python
@dataclass
class ObjectObservation:
    observation_id: int

    physical_class: str
    detector_class_id: int
    camera_id: int
    capture_timestamp: float

    # raw bbox
    left: float
    top: float
    right: float
    bottom: float
    confidence: float

    # capture camera geometry
    K: np.ndarray
    camera_origin_world: np.ndarray
    R_cw: np.ndarray

    image_width: int
    image_height: int

    truncated_left: bool
    truncated_right: bool
    truncated_top: bool
    truncated_bottom: bool
```

派生：

\[
u=\frac{l+r}{2},\qquad
v=\frac{t+b}{2},
\]

\[
w=r-l,\qquad h=b-t.
\]

必要时还保存：

\[
a=\frac{w}{h},
\qquad
s=\sqrt{wh}.
\]

### 4.1 原始池按 physical class 分开

```python
_front_object_pool: dict[str, deque[ObjectObservation]]
```

例如：

```text
gate
impact_ball_red
impact_ball_blue
pink_golf
yellow_golf
target_rack
collection_frame
red_ring
```

不同类别默认不互相竞争同一个模型。

---

## 5. BBox measurement 的统一表示

### 5.1 优化表示

完整 bbox 推荐写成：

\[
\boxed{
z_i=
\begin{bmatrix}
u_i\\
v_i\\
\log w_i\\
\log h_i
\end{bmatrix}}
\]

而不是直接使用 `w,h`，原因是尺度误差通常更接近比例误差。

### 5.2 分析表示

定义：

\[
\log s_i = \frac12(\log w_i+\log h_i),
\]

\[
\log a_i = \log w_i-\log h_i.
\]

物理意义：

```text
u,v       → image direction / bearing
log(s)    → overall apparent scale / range cue
log(a)    → shape / aspect consistency
```

### 5.3 V2 对 Gate 的进一步简化

门通常近似正视，且真实 `W/H` 已知。

因此 Gate V2 的**位置优化残差**建议只使用：

\[
\boxed{
z_{geo}=[u,v,\log h]^T}
\]

而宽高比：

\[
\boxed{a=w/h}
\]

主要作为 association / clutter 的 shape evidence：

\[
D_{shape}^2
=
\frac{\log^2(a/a_0)}{\sigma_{\log a}^2}.
\]

这样可以避免把一个弱 yaw 变量硬塞进位置优化。

---

## 6. 类别几何模型

### 6.1 Gate：已知宽高、近似正视

V2 默认状态：

\[
\boxed{X_g=[N,E,D]^T}
\]

已知：

\[
W_g,H_g.
\]

本版不估 `yaw`。

门的真实宽高比：

\[
a_{world}=W_g/H_g.
\]

在近似正视的小畸变条件下，像素期望宽高比：

\[
\boxed{
a_0 \approx \frac{f_x}{f_y}\frac{W_g}{H_g}}
\]

它是一个有噪声的 shape prior，不作为绝对硬真值。

#### 6.1.1 预测中心

门中心世界坐标为 `X`：

\[
p_c = R_{cw}(X-o_c).
\]

投影：

\[
\hat u=f_x\frac{p_x}{p_z}+c_x,
\]

\[
\hat v=f_y\frac{p_y}{p_z}+c_y.
\]

#### 6.1.2 预测高度

NED 中向下：

\[
e_D=[0,0,1]^T.
\]

门顶、门底：

\[
P_t=X-\frac{H_g}{2}e_D,
\]

\[
P_b=X+\frac{H_g}{2}e_D.
\]

分别经过完整相机模型投影：

\[
(\hat u_t,\hat v_t)=\pi(T_{cw},P_t),
\]

\[
(\hat u_b,\hat v_b)=\pi(T_{cw},P_b).
\]

预测 bbox 高度：

\[
\boxed{\hat h=|\hat v_b-\hat v_t|}
\]

因此相机 roll/pitch 和安装姿态自动进入模型。

#### 6.1.3 Gate residual

位置优化残差：

\[
\boxed{
r_{geo}=
\begin{bmatrix}
u-\hat u\\
v-\hat v\\
\log(h/\hat h)
\end{bmatrix}}
\]

shape residual：

\[
\boxed{r_a=\log(a/a_0)}.
\]

Association 总代价：

\[
\boxed{
D^2=D_{geo}^2+D_{shape}^2}
\]

其中：

\[
D_{geo}^2=r_{geo}^TR_{geo}^{-1}r_{geo}.
\]

### 6.2 Sphere：小球 / 撞击球

状态：

\[
\boxed{X_s=[N,E,D]^T}
\]

半径 `R` 已知。

V2 初始实现使用近似：

\[
\hat d_{px}\approx\frac{2f_{eff}R}{Z},
\qquad
f_{eff}=\sqrt{f_xf_y}.
\]

观测可用：

\[
\boxed{z=[u,v,\log s]^T}
\]

其中：

\[
s=\sqrt{wh}.
\]

球的 `w/h≈1` 可作为 shape quality，不必作为强位置项。

### 6.3 target_rack / collection_frame / red_ring

如果比赛布置中它们朝向基本固定，V2 默认也优先采用：

\[
\boxed{X=[N,E,D]^T}
\]

并使用已知外轮廓尺寸预测：

- center；
- apparent height / scale；
- expected aspect with larger covariance。

如果后续实测发现 yaw 变化明显，再单独开启：

\[
[N,E,D,\psi]
\]

作为 V2.1，而不是 V2 默认状态。

### 6.4 未知尺寸类别

没有可靠尺寸模型时退化成：

\[
\boxed{\text{bearing-only multi-view model}}
\]

状态：

\[
X=[N,E,D].
\]

这类 observation 的 bbox `w,h` 只做检测质量辅助，不用于绝对尺度估计。

---

## 7. Observation covariance

### 7.1 基础 bbox noise

定义：

\[
\Sigma_z=
\operatorname{diag}
(\sigma_u^2,\sigma_v^2,
\sigma_{\log w}^2,
\sigma_{\log h}^2).
\]

不同类别使用不同底座。

推荐起始值只用于调试，后续必须从实测 residual 标定：

| 类别 | `sigma_uv` | `sigma_log_scale` | `sigma_log_aspect` |
|---|---:|---:|---:|
| golf / impact ball | 2~4 px | 0.08~0.12 | 0.10~0.15 |
| gate | 3~6 px | 0.10~0.15 | 0.12~0.20 |
| rack / collection frame | 5~10 px | 0.18~0.30 | 0.20~0.35 |

### 7.2 confidence

不要直接写：

\[
w=confidence.
\]

建议 confidence 调整 covariance：

\[
\sigma_i
=\sigma_{class}
\left[1+\alpha(1-c_i)\right].
\]

### 7.3 图像边缘截断

如果 bbox 接触图像上下边缘：

- `height` 不再是完整物体高度；
- 不允许它作为强 scale measurement。

因此：

```text
truncated_top/bottom → mask log(h)
truncated_left/right → mask log(w)/aspect
```

Gate 若上下截断，仍可保留中心 bearing，但不能用高度直接反推距离。

---

## 8. 已知尺寸目标的单 Observation Proposal

V2 不再需要：

```text
left-right pair → stereo XYZ
```

来生成已知尺寸物体候选。

### 8.1 Gate proposal

由 bbox 中心得到 camera bearing：

\[
d_i^c=\operatorname{normalize}(K^{-1}[u,v,1]^T).
\]

变换到世界：

\[
d_i^w=R_{wc}d_i^c.
\]

候选位置沿射线：

\[
X(\lambda)=o_i+\lambda d_i^w.
\]

利用 bbox 高度寻找：

\[
\boxed{
\lambda^*
=\arg\min_{\lambda>0}
\left[
\log\frac{h_i}{\hat h(X(\lambda))}
\right]^2}
\]

然后：

\[
X_0=o_i+\lambda^*d_i^w.
\]

这只是 `proposal`，不能直接成为新实例。

### 8.2 Sphere proposal

近似：

\[
Z_0\approx\frac{2f_{eff}R}{\sqrt{wh}}.
\]

结合中心 bearing 得到 `X0`。

### 8.3 Proposal 的本质

\[
\boxed{\text{proposal ≠ measurement ≠ confirmed object}}
\]

一条 observation 可以产生 proposal，但它依然只是一条 observation。

---

## 9. 科学的“聚类”：从 XYZ clustering 改为 Multi-Model Fitting

### 9.1 禁止把 pairwise XYZ 当成聚类数据

旧问题：

```text
false observation f
 × 30 historical rays
 → 30 candidate XYZ
```

如果对这些 XYZ 做 DBSCAN/HDBSCAN/K-means，会把“一个假 observation 的 30 次组合”错误地视为 30 份独立证据。

V2 明确规定：

\[
\boxed{\text{candidate parameters are hypotheses, not data points}}
\]

真正的数据永远是 raw observations。

### 9.2 模型评分

对某个 hypothesis `H_j`，遍历全部同类别 observation：

\[
D_{ij}^2
=
r_{ij}^TR_i^{-1}r_{ij}+D_{shape,ij}^2.
\]

定义软 preference：

\[
\boxed{
p_{ij}=\exp(-D_{ij}^2/2)}.
\]

超过宽松统计门限后可截断为 0。

### 9.3 Observation preference vector

每条 observation：

\[
p_i=[p_{i1},p_{i2},...,p_{iM}].
\]

两个 observation 如果来自同一真实物体，应偏好相似的一组 object hypotheses。

可定义 Tanimoto similarity：

\[
\boxed{
S(i,j)=
\frac{p_i^Tp_j}
{\|p_i\|^2+\|p_j\|^2-p_i^Tp_j}}
\]

V2 不一定需要完整实现 T-Linkage 的层次聚类，但 preference-space 是模型去重、support 判断和 split 的科学依据。

### 9.4 V2 推荐工程算法：Progressive multi-model fitting

实际代码优先使用以下轻量过程：

```text
A. 先用已有 confirmed models 解释 observation pool
B. 未解释 observation → clutter/unexplained set
C. 从 unexplained observation 生成 candidate model proposals
D. 每个 proposal 对所有 raw observations 做 likelihood scoring
E. 对高支持 proposal 做 Huber/LM refine
F. 临时加入全局模型集合
G. 全局重新 assignment
H. 如果总能量显著下降，则接受新模型
I. 重复 C~H，直到再也没有值得增加的新模型
```

这样不需要预先指定 `K`。

---

## 10. 全局独占 Assignment

### 10.1 observation-level exclusivity

每条 raw detection 最多属于一个实例：

\[
\boxed{
\sum_k \mathbf 1(y_i=k)\le1}
\]

标签：

\[
y_i\in\{0,1,...,K\},
\]

其中：

\[
y_i=0
\]

表示 clutter。

### 10.2 同一 camera / frame 的物理约束

对固定 `(camera_id, frame_id, model_k)`：

\[
\boxed{
\sum_{i\in(camera,frame)}\mathbf1(y_i=k)\le1}
\]

防止同一帧两个重复 YOLO 框同时给同一个物体贡献两份证据。

### 10.3 不存在 left-right exclusivity

左、右是不同 camera：

```text
left obs  → Gate A
right obs → Gate A
```

完全允许。

但它们不是通过“stereo pair”绑定，而是各自独立发现自己都被 Gate A 的 3D 模型解释。

### 10.4 Cost matrix

对 observation `i` 和 model `k`：

\[
C_{ik}=D_{ik}^2.
\]

clutter：

\[
C_{i0}=\tau_{clutter}.
\]

同一 camera/frame 内可以通过 Hungarian/min-cost assignment 满足一对一约束；跨时间的最终 label 由全局模型一致性决定。

---

## 11. Robust Object Optimization

固定某个模型的 assigned observations：

\[
\mathcal I_k=\{i:y_i=k\}.
\]

### 11.1 Gate

\[
\boxed{
X_k^*
=
\arg\min_X
\sum_{i\in\mathcal I_k}
\rho
\left(
r_{geo,i}^TR_{geo,i}^{-1}r_{geo,i}
\right)}
\]

使用 Huber loss。

Shape 项 `aspect` 默认主要用于 assignment / outlier score，不需要强行参与位置梯度。

### 11.2 Sphere / rack / frame

形式相同，只替换几何 `project_measurement()`。

### 11.3 Jacobian

V2 状态维度通常只有 3，因此第一版可以使用数值 Jacobian：

\[
\frac{\partial r}{\partial x_j}
\approx
\frac{r(x+\epsilon e_j)-r(x-\epsilon e_j)}{2\epsilon}.
\]

建议位置：

```text
eps_xyz = 1e-4 ~ 1e-3 m
```

后续性能不足再使用 analytic/autodiff。

### 11.4 covariance

收敛后：

\[
H_k
=
\sum_i
w_iJ_i^TR_i^{-1}J_i.
\]

\[
\boxed{P_k\approx H_k^{-1}}
\]

同时输出：

- `trace(P)`；
- information eigenvalues；
- `lambda_min(H)`；
- condition number。

---

## 12. 何时更新已有目标的位置

V2 不建议每收到一个 bbox 就立刻做完整 batch LM。

需要区分：

### 12.1 observation/existence 状态立即更新

只要新 detection 被当前模型高概率解释：

立即更新：

```text
last_seen
latest_confidence
support_count
existence_score
```

### 12.2 geometry state 的重优化触发

满足以下任一条件时执行 Huber/LM：

#### 条件 A：新 observation 带来明显信息增益

当前信息矩阵：

\[
\Lambda=P^{-1}.
\]

新 observation 的局部信息：

\[
\Delta\Lambda_i=J_i^TR_i^{-1}J_i.
\]

定义 D-optimal information gain：

\[
\boxed{
\Delta I_i
=
\log\det(\Lambda+\Delta\Lambda_i)
-
\log\det\Lambda}
\]

如果：

\[
\Delta I_i>\eta_I,
\]

立即重优化。

#### 条件 B：association 结构改变

例如：

- 某 observation 从 clutter 变为 model support；
- 一个旧 observation 被另一个更好的 model 解释；
- 新 model 出生导致全局 assignment 重排；
- merge/split 发生。

这时必须重优化相关 models。

#### 条件 C：累计若干低信息 observation

低信息 observation 虽然单条不值得立即 LM，但多个 detection 的随机误差平均仍然有价值。

可以设置：

```text
front_model_pending_obs_before_refit = 3~5
```

累计到阈值后 batch refresh。

#### 条件 D：低频强制刷新

例如：

```text
front_model_max_refit_period_sec = 0.5 ~ 1.0 s
```

防止长期没有触发 A/B/C 时状态不刷新。

这不是“匹配时间窗口”，而只是计算调度。

---

## 13. 如何避免同一视角重复检测造成虚假自信

在当前 V2 假设下暂不建模 pose correlation，但 detector/model systematic error 仍可能使同视角重复 observation 过度降低 covariance。

推荐两种工程方式二选一：

### 13.1 View-cell representative（推荐）

按采集相机中心和观察方向把 observation 分到 view cells：

```text
camera position cell + view angle cell
```

同一 cell 内：

- 可全部用于 robust cost；
- 但计算 covariance / information 时只保留最高质量的 1~N 条代表 observation。

### 13.2 Redundancy weight

同一个 view cell 有 `n` 条：

\[
w_{view}=\frac{1}{\sqrt n}
\]

或设上限。

这不是 association window，也不会锁死历史归属；只是避免 100 个几乎同视角 bbox 被当成 100 个独立几何基线。

V2 第一版如果希望最简单，可以先关闭此机制，在实测发现 covariance 下降过快后再启用。

---

## 14. 新目标出生：Gate 何时被判定为“新门”

这是 V2 最核心的状态机。

### 14.1 四个阶段

```text
UNEXPLAINED OBSERVATION
        ↓
     PROPOSAL
        ↓
  TENTATIVE MODEL
        ↓
  CONFIRMED OBJECT
```

### 14.2 Unexplained observation

某 observation 对全部 confirmed models：

\[
D_{ik}^2>\tau_{assoc}
\]

或者 clutter cost 更优，则进入 unexplained set。

它**不是新门**。

### 14.3 Proposal

完整 bbox 且 scale 有效时，单 observation 可利用已知尺寸生成 `X0`。

这只是 proposal：

```text
state = X0
support = unknown
publish = false
instance_id = none
```

### 14.4 Tentative model

对 proposal `X0`：

1. 对所有同类别 raw observations 计算 `D_i²`；
2. 得到 soft support：

\[
p_i=\exp(-D_i^2/2);
\]

3. 收集高 preference observation；
4. 用它们执行一次 robust LM；
5. 重新评分。

若满足基础几何条件，则成为 Tentative。

### 14.5 Confirmed Gate 的建议条件

Gate V2 推荐同时满足：

#### A. unique support

\[
\boxed{N_{unique}\ge3}
\]

每个 `observation_id` 只能算一次。

#### B. independent views

至少：

\[
\boxed{N_{independent\_views}\ge2}
\]

不同 `camera_id` 的左右相机可以天然构成两个不同 camera centers；也可以由机器人运动形成。

不需要它们先 stereo pairing。

#### C. effective support

\[
\boxed{
N_{eff}=\sum_i p_i \ge N_{eff,min}}
\]

例如起始可设：

```text
N_eff_min = 2.2 ~ 2.5
```

防止三条都刚刚擦着门限的 observation 被视为强支持。

#### D. geometry observability

\[
\lambda_{min}(H)>\lambda_{min,birth}
\]

且：

\[
\kappa(H)<\kappa_{max}.
\]

#### E. finite / physical

- 所有关键投影在相机前方；
- 位置有限；
- range 在允许物理范围内；
- `P` 有限正定。

#### F. bbox shape consistency

Gate：

\[
|\log(a/a_0)|
\]

的 robust aggregate 必须合理。

注意 aspect 是软证据，不建议单条硬拒绝。

#### G. model birth energy gain

最重要的一条：必须证明“世界中多一个 Gate”比“把这些 observation 当 clutter / 旧模型”更合理。

---

## 15. 全局模型数量选择

### 15.1 Energy

定义：

\[
\boxed{
E(Y,\Theta)
=
\sum_i C_i(y_i)
+
\lambda_mK}
\]

其中：

\[
C_i(k)=\rho(D_{ik}^2),
\]

clutter：

\[
C_i(0)=\tau_{clutter}.
\]

`K` 是当前同类别 object models 数量。

### 15.2 Birth test

加入新 proposal 前：

\[
E_{before}.
\]

临时加入新模型、重新 assignment/refit 后：

\[
E_{after}.
\]

只有：

\[
\boxed{
E_{before}-E_{after}>\Delta E_{birth}}
\]

才正式确认新实例。

这样：

- 一个孤立假框通常不值得支付 `lambda_m`；
- 多个视角一致支持同一新 Gate 时 residual reduction 会超过 model cost；
- `K` 不需要人工指定。

### 15.3 `max_instances` 只作为安全上限

例如：

```text
gate_max_instances = 4
```

它只表示“最多允许发布 4 个”，不是聚类的 `K=4`。

---

## 16. Merge：两个模型是不是同一个目标

禁止简单使用：

```text
||X_a-X_b|| < 0.5 m → merge
```

距离只能用于限制 merge 搜索范围。

对模型 A/B：

### 分开

\[
E_{sep}=E(A)+E(B)+2\lambda_m.
\]

### 合并

将：

\[
\mathcal I_{ab}=\mathcal I_a\cup\mathcal I_b
\]

用一个模型重新 LM：

\[
E_{merge}=E(AB)+\lambda_m.
\]

只有：

\[
\boxed{E_{merge}<E_{sep}-\Delta E_{merge}}
\]

才合并。

因此两个物理位置很近但 bbox scale evolution 无法由一个共同 3D Gate 解释时，不会被距离阈值错误合并。

---

## 17. Split：一个模型是否其实包含两个目标

如果一个模型内部：

- residual 明显双峰；
- preference vector 分成两个稳定群；
- 一部分 observation 在不同视角下持续互相冲突；

则尝试生成两个子模型。

比较：

\[
E_{one}
\]

和：

\[
E_{two}+\lambda_m.
\]

如果后者显著更低：

\[
\boxed{\text{split}}
\]

V2 第一版可先不实现主动 split，只需要通过 progressive birth 让新模型从原模型解释不了的 observations 中出生；显式 split 可作为后续优化。

---

## 18. Persistent instance ID

几何模型 label 与 `instance_id` 必须分开。

每次 batch reconstruction 后得到：

```text
model_0
model_1
model_2
```

这些编号没有时间连续意义。

最终对新的 confirmed models 与 persistent tracks 做：

\[
D_{track,model}^2
=
(X_t-X_m)^T(P_t+P_m)^{-1}(X_t-X_m).
\]

然后 Hungarian 保持 ID 连续。

注意：

\[
\boxed{\text{track matching 只用于 ID continuity，不用于 raw observation acceptance}}
\]

旧 track 一轮没被新 model 命中，不立刻删除；按 timeout 转为 `STALE`。

---

## 19. V2 前视主循环

推荐伪代码：

```python
def process_front_observation(obs):
    pool = object_pool[obs.physical_class]
    pool.append(obs)

    # 1. 先由 confirmed models 解释当前 pool
    models = persistent_model_states(obs.physical_class)

    # 2. 计算 raw observation × model likelihood
    cost = compute_model_costs(pool, models)

    # 3. 独占 assignment + clutter
    labels = global_assign(pool, models, cost)

    # 4. association 变化或信息增益够大 -> refine existing models
    for model in models:
        assigned = get_assigned(pool, labels, model)
        if should_refit(model, assigned):
            robust_lm_refit(model, assigned)

    # 5. 未解释 observation
    unexplained = get_clutter_or_unexplained(pool, labels)

    # 6. progressive birth loop
    while True:
        proposals = generate_known_size_proposals(unexplained)
        proposals = score_all_raw_observations(proposals, pool)
        candidate = select_best_candidate(proposals)

        if candidate is None:
            break

        candidate = robust_lm_refit(candidate, candidate.support)

        if not basic_birth_checks(candidate):
            suppress_proposal(candidate)
            continue

        E_before = global_energy(pool, models, labels)

        trial_models = models + [candidate]
        trial_cost = compute_model_costs(pool, trial_models)
        trial_labels = global_assign(pool, trial_models, trial_cost)
        refit_affected_models(trial_models, trial_labels)
        E_after = global_energy(pool, trial_models, trial_labels)

        if E_before - E_after > birth_energy_gain_threshold:
            models = trial_models
            labels = trial_labels
            confirm_new_instance(candidate)
            unexplained = get_clutter_or_unexplained(pool, labels)
        else:
            suppress_proposal(candidate)

        if no_more_supported_proposals(unexplained):
            break

    # 7. merge-energy test
    models = energy_based_merge(models, pool, labels)

    # 8. persistent instance id matching
    match_models_to_tracks(models)

    # 9. publish
    publish_tracks()
```

---

## 20. Proposal 抑制与去重

多个 observation 很可能对同一真实物体生成很多相近 proposal。

但不要重新退化成 `XYZ DBSCAN`。

优先根据：

1. 两个 proposal 对 raw observations 的 preference vector 是否相似；
2. 对同一支持集合优化后是否收敛到相同后验；
3. merge energy test 是否支持一个模型解释两者；

进行去重。

位置距离可以作为 cheap pre-filter：

```text
if ||Xa-Xb|| > merge_search_radius:
    不做昂贵 merge test
```

但绝不能作为最终 merge 判据。

---

## 21. Gate 的科学 association 例子

两个 Gate 沿接近方向排列：

```text
Gate A: near
Gate B: far
```

当前 observation：

\[
z=[u,v,h,a].
\]

对 Gate A：

\[
\hat h_A=185px,
\]

对 Gate B：

\[
\hat h_B=90px.
\]

检测：

\[
h=178px.
\]

即使：

\[
D_{bearing,A}\approx D_{bearing,B},
\]

scale 项：

\[
D_{scale,A}^2
=
\frac{\log^2(178/185)}{\sigma_h^2}
\]

会远小于：

\[
D_{scale,B}^2
=
\frac{\log^2(178/90)}{\sigma_h^2}.
\]

因此无需先决定它和另一相机中的哪一个框“配对”。

每一条 bbox 只需要问：

\[
\boxed{\text{哪个 3D Gate model 最能解释我？}}
\]

---

## 22. 为什么不需要 stereo correspondence

传统 stereo：

```text
L bbox + R bbox
      ↓ correspondence
 disparity / triangulation
      ↓
 XYZ
```

V2：

```text
L bbox → likelihood against all object models
R bbox → likelihood against all object models
```

如果它们真属于同一个物体：

- 同一个 3D model 会同时给两者低 residual；
- 两个不同 camera centers 会共同增强该 model 的 Hessian；
- 不需要人为声明 `L1 ↔ R3`。

如果左右 observation 实际来自两个不同物体：

- 一个共同 model 通常无法同时解释 center + scale + shape；
- 独占 assignment 会把它们给不同 models；
- 如果暂时仍 ambiguous，就保留 clutter / alternative model competition，不做早期 hard pair。

---

## 23. V2 中哪些旧前视逻辑应废弃

前视主路径建议废弃或仅保留 debug/legacy：

```text
_match_detections() 用于前视目标实例级左右配对
front stereo target correspondence
stereo XYZ 作为前视目标主观测
FRONT_STEREO 独立最终 estimator
FRONT_MULTI_VIEW 独立最终 estimator
ray_association_angle_deg
front_gate_ray_association_angle_deg
pairwise XYZ → spatial clustering 作为已知尺寸目标主路径
```

### 23.1 可保留的 stereo metadata

如果上游图像拼接 / 同步仍需要：

```text
stereo_pair_id
stereo_info
```

可以保留在采集链路中。

但前视定位器不再使用它们决定实例对应关系。

### 23.2 兼容消息

如果 `TargetObservation` 历史上只有：

```text
FORM_FRONT_STEREO
FORM_FRONT_MULTI_VIEW
```

建议新增：

```text
FORM_FRONT_BBOX_MODEL
```

或者：

```text
FORM_FRONT_OBJECT_OBSERVATION
```

旧常量可以保留兼容，但 V2 主输出应该明确表示：

> 前视最终状态由多相机、多视角 raw object observations 联合估计。

---

## 24. 下视路径

V2 暂不要求同步重构下视。

下视可继续：

```text
known-height plane intersection
或可选 stereo
→ down observation pool
→ existing clustering / filtering
```

前视与下视未来可以在 object-level 再统一，但这不属于 V2 必须项。

---

## 25. 参数建议

建议新增/重命名：

| 参数 | 初始建议 | 说明 |
|---|---:|---|
| `front_object_pool_size` | 300 / class | raw bbox pool |
| `front_model_assoc_chi2` | 按有效 residual 维数取 99% 左右 | observation → model compatibility |
| `front_model_clutter_cost` | 与 assoc gate 同量级 | clutter cost |
| `front_model_birth_cost` | 实测调参 | 新实例复杂度代价 |
| `front_model_birth_min_unique_obs` | 3 | 新实例唯一 raw observation 数 |
| `front_model_birth_min_views` | 2 | 独立 camera/view 数 |
| `front_model_birth_min_effective_support` | 2.2~2.5 | soft support |
| `front_model_birth_max_condition` | `1e5~1e7` 量级起调 | 几何退化过滤 |
| `front_model_info_gain_refit` | 实测标定 | 新 observation 触发 LM 的信息增益 |
| `front_model_pending_obs_before_refit` | 3~5 | 低信息 observation 批量刷新 |
| `front_model_max_refit_period_sec` | 0.5~1.0 s | 最长不重优化时间 |
| `front_model_lm_iterations` | 8~15 | Huber/LM |
| `front_gate_height_m` | 实际测量 | Gate known size |
| `front_gate_width_m` | 实际测量 | shape prior |
| `front_gate_sigma_uv_px` | 3~6 px 起调 | bbox center noise |
| `front_gate_sigma_log_h` | 0.10~0.15 起调 | scale noise |
| `front_gate_sigma_log_aspect` | 0.12~0.20 起调 | shape noise |
| `front_model_merge_search_radius_m` | 仅 cheap pre-filter | 不是 merge 判据 |
| `front_model_merge_energy_gain` | 实测调参 | energy merge threshold |

旧参数：

```text
front_epipolar_error_px
front_bbox_aspect_ratio_max (left-vs-right)
min_disparity_px (front main path)
stereo_pending_timeout_sec (front localization)
front_multi_view_min_angle_deg (known-size main path)
front_bearing_cluster_radius_m (known-size main path)
```

在 V2 已知尺寸主路径中不再作为核心参数。

---

## 26. Status 状态机

建议 object model 状态：

```text
PROPOSAL
TENTATIVE
ESTIMATING
STABLE
STALE
```

### PROPOSAL

- 由单 bbox / bearing seed 产生；
- 未通过 consensus；
- 不发布为真实实例。

### TENTATIVE

- 已有多个 raw observation 支持；
- 但未通过全部 birth / observability / energy 检查；
- 可以在 debug topic 中可视化。

### ESTIMATING

- 已正式确认并获得 `instance_id`；
- 位置 covariance 尚大。

### STABLE

建议不要只看 observation 数。

至少：

\[
\operatorname{trace}(P)<\tau_P
\]

且：

\[
\lambda_{min}(H)>\tau_H.
\]

### STALE

超过 `track_timeout_sec` 没有新的高概率 observation。

---

## 27. 调试指标

V2 建议重点记录：

```text
front_raw_object_observations
front_model_proposals_generated
front_model_proposals_scored
front_model_proposals_suppressed
front_model_birth_attempts
front_model_birth_accepted
front_model_birth_energy_rejected
front_model_assignments
front_model_clutter
front_model_refits_information_triggered
front_model_refits_assignment_triggered
front_model_refits_batch_triggered
front_model_merge_attempts
front_model_merge_accepted
front_model_rank_deficient
front_model_observation_reuse_prevented
front_model_duplicate_same_frame_prevented
```

每个 model 建议 debug 输出：

```text
instance_id
physical_class
state XYZ
covariance
support_unique_count
effective_support
independent_view_count
mean_whitened_residual
median_whitened_residual
shape_residual
information_eigenvalues
condition_number
last_seen
status
```

---

## 28. 必做测试

### Test A：单 Gate

所有 observation 人工来自同一个门：

- 验证位置稳定；
- covariance 随有效独立视角改善；
- 左右相机无需 pairing 也能同时支持该 Gate。

### Test B：两个前后 Gate

两个门 bearing 接近但距离不同：

- 验证 bbox height 能把它们拆开；
- 不允许同 observation 同时进入两个 Gate。

### Test C：同一张图多个门

同 camera/frame 有多个 detections：

- 每个 Gate 最多拿一个 detection；
- 同一 detection 只能给一个 Gate。

### Test D：单假框

加入一条不存在目标的 false bbox：

- 可以产生 proposal；
- 不得直接成为 confirmed model；
- birth energy 应倾向 clutter。

### Test E：错误 observation 组合爆炸

构造：

```text
1 false observation + 30 good observations
```

验证无论 false observation 能生成多少 proposal：

\[
\boxed{\text{它只贡献 1 个 observation support}}
\]

### Test F：左右相机错位时间

令：

\[
t_L\neq t_R.
\]

只要 pose 插值正确，模型仍应稳定；不依赖 stereo sync。

### Test G：AUV 静止

机器人基本不动，但左右相机同时观测：

- 两个 camera centers 应仍增强信息；
- 不需要显式 disparity。

### Test H：只单目可见

某时刻右相机没检测到：

- 左相机 observation 仍正常入池；
- 不等待 pair timeout；
- 已知尺寸目标仍可产生 proposal / 更新模型。

### Test I：目标出图边缘

bbox 上下截断：

- 高度项 mask；
- 不能用错误截断高度推深度；
- center/bearing 仍可作为弱证据。

### Test J：两个模型 merge

人为放两个近距离 Gate：

- 距离小不应自动 merge；
- 只有共同模型的 global energy 更低才允许 merge。

---

## 29. 建议的软件拆分

```text
object_localizer.py
    │
    ├── ObservationBuilder
    │       └── raw bbox + camera pose
    │
    ├── ObjectObservationPool
    │
    ├── geometry/
    │       ├── base.py
    │       ├── gate.py
    │       ├── sphere.py
    │       ├── fixed_box.py
    │       └── bearing_fallback.py
    │
    ├── proposal_generator.py
    │
    ├── model_scorer.py
    │
    ├── global_assignment.py
    │
    ├── object_optimizer.py
    │
    ├── model_selection.py
    │       ├── birth
    │       ├── merge
    │       └── optional split
    │
    └── track_manager.py
            └── persistent instance_id only
```

核心接口：

```python
class GeometryModel:
    def proposal_from_observation(self, obs): ...
    def predict_measurement(self, state, obs): ...
    def residual(self, state, obs): ...
    def whitened_residual(self, state, obs): ...
```

上层 multi-model fitting 不关心具体类别。

---

## 30. V2 的核心数学模型

对类别 `c`，几何模型为：

\[
\mathcal M_c.
\]

第 `k` 个实例状态：

\[
X_k.
\]

第 `i` 条 observation：

\[
z_i.
\]

预测：

\[
\boxed{
\hat z_{ik}
=h_{\mathcal M_c}(X_k,T_{cw,i})}
\]

观测模型：

\[
\boxed{
z_i
=h_{\mathcal M_c}(X_{y_i},T_{cw,i})+\epsilon_i}
\]

其中：

\[
\epsilon_i\sim\mathcal N(0,R_i).
\]

标签：

\[
\boxed{
y_i\in\{0,1,...,K_c\}}
\]

`0` 为 clutter。

最终联合目标：

\[
\boxed{
\min_{\Theta,Y,K}
\left[
\sum_i
\rho\left(
\|z_i-h(X_{y_i})\|_{R_i}^2
\right)
+
\lambda_cN_{clutter}
+
\lambda_mK
\right]}
\]

并满足：

\[
\boxed{\text{one raw observation belongs to at most one object}}
\]

以及：

\[
\boxed{\text{one object uses at most one detection from the same camera/frame}}
\]

这就是 V2 的统一核心。

---

## 31. 与 V1 的关键差异总结

| 项目 | V1 | V2 |
|---|---|---|
| 前视左右检测 | 仍可能先 stereo pair | **完全独立 observation** |
| stereo XYZ | 可作 candidate init | **已知尺寸主路径不再需要** |
| FRONT_STEREO / MULTI_VIEW | 仍有概念残留 | **统一为 multi-camera multi-view** |
| Gate state | `[N,E,D,yaw]` 倾向 | **默认 `[N,E,D]`** |
| Gate bbox | `[u,v,logw,logh]` | **位置用 `[u,v,logh]`，aspect 做关联** |
| 候选聚类 | model scoring + 部分几何聚类 | **raw observation preference / progressive multi-model fitting** |
| K | 实例上限 + 候选簇 | **energy-driven，自适应 K；上限仅 safety** |
| 新目标 | support + model cost | **support + independent views + observability + global energy gain** |
| 位置更新 | batch rebuild 为主 | **information gain / assignment change / batch trigger** |
| merge | 有搜索距离 | **距离只预筛，最终由 merge energy 决定** |
| left-right timing | pairing 相关 | **各用自己的 timestamp / pose** |

---

## 32. 最终推荐落地顺序

### P0：先实现统一 observation

1. 前视左右 detection 分别直接变成 `ObjectObservation`；
2. 不等待另一相机；
3. 不执行 target-level stereo pairing；
4. 每条 observation 用自己的 timestamp / pose / camera model。

### P0：Gate 三维模型

1. 状态 `[N,E,D]`；
2. bbox center → `u,v`；
3. bbox height → scale constraint；
4. `w/h` → shape likelihood；
5. Huber LM。

### P0：独占 association + clutter

严格保证：

```text
one observation → max one object
same camera/frame/object → max one bbox
```

### P0：Progressive birth

unexplained bbox → single-size proposal → consensus → LM → energy test → confirmed Gate。

### P1：information-triggered refit

先用简单：

```text
association changed
OR new independent view
OR pending >= 3
OR 0.5s elapsed
```

后面再换成严格 `Δlogdet(H)`。

### P1：energy merge

彻底移除“距离小于多少直接合并”。

### P2：view redundancy / covariance calibration

收集实测数据标定：

- `sigma_uv`；
- `sigma_log_h`；
- `sigma_log_aspect`；
- birth/model cost；
- information thresholds。

---

# 结论

V2 前视定位的核心不再是：

```text
stereo correspondence
→ disparity / XYZ
→ multi-view association
```

而是：

```text
任何相机的任何 bbox
→ 独立 raw observation
→ 已知尺寸几何 proposal
→ 对所有 3D object models 做重投影 likelihood
→ exclusive assignment + clutter
→ robust multi-view object optimization
→ energy-based model birth/merge
```

因此左/右相机仍然提供非常有价值的不同空间光心，但算法不需要知道哪两个 bbox 是“stereo pair”。

对 Gate，已知高度提供强距离尺度，已知宽高比提供强 shape consistency；结合不同 camera pose 的中心重投影约束，足以成为 V2 的主实例判别方式。

最终真正统一的不是“所有东西都变成射线”，而是：

\[
\boxed{
\text{3D object state}
\xrightarrow{\text{camera projection}}
\text{2D bbox observation}}
\]

并由所有相机、所有时刻的原始 bbox 共同决定世界中有几个目标以及它们在哪里。
