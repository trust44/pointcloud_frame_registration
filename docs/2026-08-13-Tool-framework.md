# 单帧点云与全局点云 6DOF 剖面配准工具实施方案

## 1. 目标与范围

实现一个桌面端交互工具，用于将**单帧 LiDAR 点云**与**全局地图点云**进行局部 6DOF 配准。用户通过 X-Z、Y-Z 及对角剖面观察两类点云是否重合，并手动微调或使用 ICP/GICP 精修，最终导出校正后的 LiDAR 位姿和变换矩阵。

本工具的核心约束如下：

- 全局点云和单帧点云均可提供为 `map` 坐标系下的点；
- 用户提供当前帧初始位姿 `T_map_lidar`；
- 所有旋转的物理基准为**当前帧 LiDAR 原点**，即 LiDAR 原点在 `map` 下的位置；
- 不允许使用单帧点云包围盒中心、全局地图原点或可变裁剪中心作为旋转 Pivot；
- 软件输出必须与标准 ICP 的 Source-to-Target 4×4 矩阵乘法逻辑兼容。

第一版仅覆盖单帧人工配准闭环：加载、局部裁剪、6DOF 调整、多剖面验证、导出。暂不实现批量多帧标定、地图编辑、语义分割和自动全局定位。

---

## 2. 坐标系、矩阵及术语约定

### 2.1 坐标系

| 名称 | 记号 | 说明 |
|---|---|---|
| 全局地图坐标系 | `map` / M | 全局点云的坐标系 |
| 雷达坐标系 | `lidar` / L | 当前帧雷达坐标系；原点为物理 LiDAR 原点 |
| 初始 LiDAR 位姿 | `T0 = T_map_lidar` | 当前帧 LiDAR 从 `lidar` 变换到 `map` 的 4×4 矩阵 |
| 初始 LiDAR 原点 | `C0` | `T0[:3, 3]`，即 LiDAR 原点在 `map` 下的位置 |

统一采用右手坐标系、列向量、齐次矩阵：

\[
P_M = T^0_{M\leftarrow L} P_L
=
R_0 P_L + t_0
\]

其中：

- `t0 = [tx, ty, tz]^T` 是 LiDAR 原点在 `map` 下的位置；
- `R0` 表示 LiDAR 的 X、Y、Z 三轴在 `map` 下的朝向；
- `R0` 的三列依次是 LiDAR X/Y/Z 轴在 `map` 下的单位方向向量。

因此，`t0` 只决定 LiDAR 的**位置**，LiDAR 的轴方向必须通过 `R0` 确定。

### 2.2 ICP 输出的标准语义

当 Source 和 Target 都在 `map` 坐标系下时，ICP/GICP 返回修正矩阵：

\[
T_{icp}^{map} =
\begin{bmatrix}
R_{icp} & t_{icp}\\
0 & 1
\end{bmatrix}
\]

并满足：

\[
P_M^{after}=R_{icp}P_M^{before}+t_{icp}
\]

此矩阵的旋转和平移均在 `map` 坐标系表达；其矩阵形式相当于绕 `map` 原点旋转后再平移。因此 `t_icp` **不是 LiDAR 原点的实际位移**。

修正后的 LiDAR 位姿与 LiDAR 原点位置分别为：

\[
T^1_{M\leftarrow L}=T_{icp}^{map}T^0_{M\leftarrow L}
\]

\[
C_1=R_{icp}C_0+t_{icp}
\]

LiDAR 原点在 `map` 下的实际位置修正为：

\[
\Delta C_M=C_1-C_0=R_{icp}C_0+t_{icp}-C_0
\]

### 2.3 本工具的手动调整语义

为保证用户交互符合“绕 LiDAR 原点调整”，软件将 UI 参数定义为：

```text
平移坐标系：map
旋转轴坐标系：map
旋转中心：初始 LiDAR 原点 C0
```

UI 参数：

```text
Δx_map, Δy_map, Δz_map：LiDAR 原点在 map 下的实际位置变化
Δroll_map, Δpitch_map, Δyaw_map：绕 map 的 X/Y/Z 轴旋转
```

固定轴外旋顺序：

\[
R_\Delta=R_z(\Delta yaw)R_y(\Delta pitch)R_x(\Delta roll)
\]

以 LiDAR 原点为 Pivot 的点变换为：

\[
P_M^{after}=R_\Delta(P_M^{before}-C_0)+C_0+\Delta C_M
\]

对应的、可直接作用于 `map` 点云的标准矩阵为：

\[
T_{manual}^{map}=
\begin{bmatrix}
R_\Delta & C_0-R_\Delta C_0+\Delta C_M\\
0&1
\end{bmatrix}
\]

最终 LiDAR 位姿：

\[
T^1_{M\leftarrow L}=T_{manual}^{map}T^0_{M\leftarrow L}
\]

> 说明：`T_manual_map[:3, 3]` 一般不等于 UI 中的 `Δx_map, Δy_map, Δz_map`。前者包含“绕 LiDAR 原点而非 map 原点旋转”的平移补偿；后者才是 LiDAR 原点真实移动量。

---

## 3. 输入、输出与配置

### 3.1 必选输入

| 输入 | 坐标系 | 格式 | 说明 |
|---|---|---|---|
| `global_map` | `map` | LAS/LAZ/PCD/PLY | 全局点云 Target |
| `frame_cloud_map` | `map` | LAS/LAZ/PCD/PLY | 当前帧点云 Source，已转换到 map |
| `T_map_lidar` | `map ← lidar` | JSON/YAML/CSV/ROS TF | 当前帧初始 LiDAR 位姿 |

要求：`frame_cloud_map` 与 `global_map` 必须使用同一 `map` 坐标定义和同一长度单位（m）。

### 3.2 配置文件示例

```yaml
global_map_path: /data/global_map.laz
frame_cloud_map_path: /data/frame_001.pcd

initial_pose:
  parent_frame: map
  child_frame: lidar
  matrix:
    - [1.0, 0.0, 0.0, 100.0]
    - [0.0, 1.0, 0.0, 200.0]
    - [0.0, 0.0, 1.0, 108.0]
    - [0.0, 0.0, 0.0, 1.0]

display:
  map_roi_radius_m: 35.0
  display_voxel_m: 0.05
  slice_half_length_m: 20.0
  slice_thickness_m: 0.20
  profile_angles_deg: [0, 90, 45, -45]

interaction:
  translation_step_m: 0.01
  translation_large_step_m: 0.10
  rotation_step_deg: 0.05
  rotation_large_step_deg: 0.50
```

### 3.3 输出

必须输出 `alignment_result.yaml`：

```yaml
source_frame: lidar
target_frame: map

initial_T_map_lidar: [[...], [...], [...], [...]]

manual_delta_about_lidar_origin:
  translation_frame: map
  rotation_axes_frame: map
  pivot_map: [Cx, Cy, Cz]
  dx_m: 0.00
  dy_m: 0.00
  dz_m: 0.00
  roll_deg: 0.00
  pitch_deg: 0.00
  yaw_deg: 0.00

T_manual_map: [[...], [...], [...], [...]]
corrected_T_map_lidar: [[...], [...], [...], [...]]

quality:
  slice_half_length_m: 20.0
  slice_thickness_m: 0.20
  nn_residual_median_m: null
  nn_residual_p95_m: null
  icp_rmse_m: null
```

可选输出：

```text
aligned_frame_map.pcd / .las / .laz
profile_xz.png
profile_yz.png
profile_plus45.png
profile_minus45.png
```

---

## 4. 推荐实现技术栈

### 4.1 MVP（第一版）

```text
Python 3.10 或 3.11
Open3D 0.19+         3D 渲染、PCD/PLY 读取、点云基础处理、ICP
PySide6              主界面、面板、参数输入、快捷键、文件对话框
pyqtgraph            实时 2D 剖面绘制
NumPy                批量矩阵与剖面计算
SciPy                Rotation、KD-tree，可选
laspy + lazrs        LAS/LAZ 文件读取与写出
PDAL（可选）         大地图 LAS/LAZ/COPC 的 ROI 裁剪和高性能读取
PyYAML               YAML 配置与结果导出
```

推荐以 **PySide6 + Open3D** 做独立桌面程序。初版可保留 Open3D 的 3D SceneWidget；2D 剖面改用 `pyqtgraph`，不要继续用 Matplotlib 在每次按键后整图栅格化，后者在高频交互时会明显卡顿。

### 4.2 大数据性能边界

| 模块 | 策略 |
|---|---|
| 全局点云 | 只加载 LiDAR 原点周围 35 m ROI；必要时先用 PDAL/COPC 裁剪 |
| 主 3D 视图 | 对地图和单帧分别做 5 cm 左右体素降采样 |
| 剖面 | 只使用薄片范围内的点；每次参数变化仅重算 Source 剖面 |
| 原始点云 | 仅用于最终导出，不随每次交互重复拷贝和变换 |
| ICP | 使用局部 ROI + 5~10 cm 下采样点云 |

目标：局部地图 10 万～100 万显示点、单帧 10 万～30 万点时，普通独显电脑中常规参数拖动达到可交互流畅度；剖面刷新不应依赖全局地图全量扫描。

---

## 5. 程序模块与职责

```text
align_tool/
├── main.py                    # 程序入口、参数解析
├── config.py                  # YAML 读取与参数校验
├── io/
│   ├── point_cloud_io.py      # PCD/PLY/LAS/LAZ 读取、颜色与字段处理
│   └── pose_io.py             # 读取/校验 T_map_lidar
├── core/
│   ├── pose_model.py          # 6DOF、Pivot 变换、Undo/Redo
│   ├── point_cloud_model.py   # 原始点、显示点、ROI 点管理
│   ├── slice_engine.py        # 剖面提取和投影
│   ├── registration.py        # ICP/GICP 与质量指标
│   └── validation.py          # 输入坐标系、单位、矩阵合法性检查
├── ui/
│   ├── main_window.py         # 窗口布局
│   ├── scene_3d.py            # 3D 视图与切片范围显示
│   ├── profile_view.py        # 2D 剖面组件
│   └── pose_panel.py          # 6DOF 输入、快捷键、导出按钮
└── export/
    └── result_exporter.py     # YAML/JSON/点云/截图导出
```

### 5.1 `PoseModel` 必须提供的接口

```python
class PoseModel:
    def set_initial_pose(self, T0_map_lidar: np.ndarray) -> None: ...
    def set_delta_map(self, dx, dy, dz, roll_deg, pitch_deg, yaw_deg) -> None: ...
    def reset_delta(self) -> None: ...
    def get_lidar_origin_map(self) -> np.ndarray: ...       # C0
    def get_manual_transform_map(self) -> np.ndarray: ...   # T_manual_map
    def get_corrected_pose(self) -> np.ndarray: ...         # T1_map_lidar
    def transform_source_map(self, points_map: np.ndarray) -> np.ndarray: ...
```

不得在 UI 层自行拼接矩阵；所有矩阵计算必须集中在 `PoseModel` 中。

---

## 6. 核心算法实现要求

### 6.1 以 LiDAR 原点为 Pivot 构造手动变换

```python
import numpy as np
from scipy.spatial.transform import Rotation


def make_manual_transform_map(
    T0_map_lidar: np.ndarray,
    dx_map: float,
    dy_map: float,
    dz_map: float,
    roll_deg: float,
    pitch_deg: float,
    yaw_deg: float,
) -> np.ndarray:
    """构建以初始 LiDAR 原点为 Pivot 的 map 坐标修正矩阵。

    欧拉角采用固定 map 轴的 Z-Y-X 顺序：yaw -> pitch -> roll。
    """
    C0 = T0_map_lidar[:3, 3]
    R_delta = Rotation.from_euler(
        "ZYX", [yaw_deg, pitch_deg, roll_deg], degrees=True
    ).as_matrix()
    delta_C_map = np.array([dx_map, dy_map, dz_map], dtype=np.float64)

    T_manual = np.eye(4, dtype=np.float64)
    T_manual[:3, :3] = R_delta
    T_manual[:3, 3] = C0 - R_delta @ C0 + delta_C_map
    return T_manual


def corrected_pose(T0_map_lidar: np.ndarray, T_manual_map: np.ndarray) -> np.ndarray:
    return T_manual_map @ T0_map_lidar
```

必须具备以下单元测试：

```text
1. 全部 Δ6DOF=0：T_manual=I，T_corrected=T0。
2. 仅旋转：LiDAR 原点 C0 变换后仍等于 C0。
3. 仅平移 ΔC：LiDAR 原点变换后等于 C0+ΔC。
4. 任意旋转和平移：T_corrected 的平移列等于 C0+ΔC。
5. T_manual 作用于已在 map 的 Source 点，与公式 R(P-C0)+C0+ΔC 一致。
```

### 6.2 点云显示与更新原则

```text
原始 Source map 点云：永不修改
显示用 Source 点云：由原始显示降采样点云 × 当前 T_manual_map 实时计算
导出用 Source 点云：由原始全分辨率点云 × 当前 T_manual_map 一次性计算
```

不得连续对“上一次已变换点云”再次应用变换，否则会产生累计误差并导致 Reset 无法恢复。

### 6.3 剖面提取

剖面中心始终跟随修正后的 LiDAR 原点：

\[
C_{current}=C_0+[\Delta x_{map},\Delta y_{map},\Delta z_{map}]^T
\]

对于剖面角度 \(\theta\)，定义：

\[
u_\theta=[\cos\theta,\sin\theta,0]^T
\]

\[
v_\theta=[-\sin\theta,\cos\theta,0]^T
\]

对 `map` 点 \(P\)：

\[
q=P-C_{current}
\]

\[
s=u_\theta^Tq,\quad d=v_\theta^Tq,\quad h=q_z
\]

保留条件：

\[
|s| \le L,\quad |d| \le w/2
\]

其中 `L` 是剖面半长，`w` 是切片厚度。剖面 2D 坐标为 `(s, h)`。

默认剖面配置：

| 名称 | 角度 θ | 作用 |
|---|---:|---|
| 前后 X-Z | 0° | 观察 Z、Pitch、前后结构 |
| 左右 Y-Z | 90° | 观察 Z、Roll、左右结构 |
| 对角剖面 A | +45° | 辅助消除方向退化 |
| 对角剖面 B | -45° | 辅助消除方向退化 |

### 6.4 剖面刷新优化

```text
全局地图：ROI 建立完成后，预先保存其显示用点；仅在剖面中心位置移动超过阈值时重算切片。
单帧点云：每次 Δ6DOF 修改后重算已降采样 Source 的变换及切片。
全局点云与单帧点云：分别显示不同颜色，支持透明度、点大小、显隐开关。
```

---

## 7. UI 与交互设计

### 7.1 界面布局

```text
┌───────────────────────────── 3D 主视图 ──────────────────────────────┐
│ 全局地图 ROI（灰/青）  单帧 Source（黄/红）  LiDAR 坐标轴  剖面薄片 │
└──────────────────────────────────────────────────────────────────────┘

┌──────────── X-Z 剖面 ────────────┐ ┌──────────── Y-Z 剖面 ────────────┐
│ θ=0°                              │ │ θ=90°                             │
└───────────────────────────────────┘ └───────────────────────────────────┘

┌────────── +45° 剖面 ─────────────┐ ┌────────── -45° 剖面 ─────────────┐
│ 可独立显示/隐藏                    │ │ 可独立显示/隐藏                    │
└───────────────────────────────────┘ └───────────────────────────────────┘

┌──────────────────────── 6DOF 与导出面板 ─────────────────────────────┐
│ Δx Δy Δz | Roll Pitch Yaw | Reset | Undo | Redo | ICP | Export        │
│ Pivot C0 | T_manual_map | T_corrected_map_lidar | residual statistics │
└──────────────────────────────────────────────────────────────────────┘
```

### 7.2 交互规则

| 参数 | 小步长 | 大步长（Shift） | 默认快捷键 |
|---|---:|---:|---|
| Δx map | 0.01 m | 0.10 m | A / D |
| Δy map | 0.01 m | 0.10 m | W / S |
| Δz map | 0.01 m | 0.10 m | Q / E |
| Yaw map | 0.05° | 0.50° | 1 / 3 |
| Pitch map | 0.05° | 0.50° | 4 / 6 |
| Roll map | 0.05° | 0.50° | 7 / 9 |

每次调整后必须同步执行：

```text
1. 从 T0 和当前 Δ6DOF 重新构建 T_manual_map；
2. 更新 Source 显示点云；
3. 更新 T_corrected_map_lidar；
4. 更新 3D 视图中 LiDAR 原点与切片范围；
5. 更新四个剖面；
6. 更新参数、矩阵和质量统计显示；
7. 将本次参数写入 Undo 栈。
```

### 7.3 推荐操作顺序

```text
1. 加载全局地图、单帧 map 点云和 T_map_lidar。
2. 自动裁剪 C0 周围的全局地图 ROI。
3. 优先调 Δz，消除整体高度差。
4. 调 Roll 与 Pitch，使两类点云的地面线、墙面或路沿不再呈系统性斜角。
5. 调 Δx、Δy 和 Yaw，使道路边缘、固定杆件、墙体等横向结构重合。
6. 必要时执行一次受限 ICP/GICP。
7. 在 X-Z、Y-Z、±45° 剖面中确认没有稳定的双层、斜角或偏移。
8. 导出结果与截图。
```

---

## 8. ICP/GICP 功能设计

### 8.1 定位

ICP/GICP 是人工剖面调节后的辅助精修，不是唯一验收依据。道路地面会使 XY/Yaw 出现退化或局部最优，必须保留人工剖面确认。

### 8.2 执行流程

```text
1. 取当前 C_current 周围 20~30 m 的全局地图 ROI。
2. 取当前手动变换后的 Source ROI。
3. 对两者进行体素降采样（建议 0.05~0.10 m）。
4. 可选过滤明显动态点、草地噪声和远距离稀疏点。
5. 以当前 T_manual_map 为初始值，运行 point-to-plane ICP 或 GICP。
6. 限制最大对应距离、最大迭代次数、单次允许修正范围。
7. 得到 T_icp_increment_map，并左乘累积：
   T_manual_map_new = T_icp_increment_map × T_manual_map_current。
8. 重新由 T_manual_map_new × T0 得到 T_corrected_map_lidar。
9. 将最终矩阵重新分解为 UI 所需的“绕 C0 的 Δ6DOF”并更新界面。
```

建议初值：

```text
最大对应距离：0.20 ~ 0.50 m
ICP/GICP 体素尺寸：0.05 ~ 0.10 m
单次最大平移：0.50 m
单次最大旋转：3°
```

### 8.3 ICP 后的参数回写

ICP 结果是标准 `map` 变换矩阵。为保持 UI 语义，需从最终矩阵提取：

```text
R_delta = T_manual_map[:3, :3]
ΔC_map = R_delta @ C0 + T_manual_map[:3, 3] - C0
```

然后按固定 Z-Y-X 顺序将 `R_delta` 转为 yaw/pitch/roll。应处理欧拉角奇异性：当 pitch 接近 ±90° 时，界面提示并保留矩阵/四元数作为真实结果。

---

## 9. 对现有初版代码的改造要求

现有 `粘贴的代码 (1)。py` 可作为 Open3D 渲染和键盘交互的参考，但不得直接作为最终实现。需要至少完成以下改造：

| 当前实现 | 问题 | 必须改造 |
|---|---|---|
| `src_center = bbox centre` | Pivot 会随视野、裁剪、点云形状变化 | 改为 `C0 = T_map_lidar[:3, 3]` |
| 未读取 `T_map_lidar` | 无法验证 LiDAR 原点与姿态语义 | 增加位姿配置输入和矩阵校验 |
| `make_tf(..., src_center)` | 输出仅是局部显示矩阵，非标准位姿链 | 替换为 `T_manual_map` 与 `T_corrected_map_lidar` |
| 默认只 X/Y 两剖面 | 不足以辅助排除姿态退化 | 增加 +45°、-45°，角度可配置 |
| Matplotlib 每次重绘为图片 | 高频调参可能卡顿 | 换为 pyqtgraph 或降低刷新频率/点量 |
| 原始点云读取后全量参与 | 大地图会卡顿或内存过高 | 按 C0 建立 map ROI 和显示降采样 |
| 参数步长命令行参数未实际生效 | 行为与配置不一致 | 使用 CLI/YAML 参数初始化步长 |
| 输出 `transform_matrix` | 未说明矩阵的 source/target 与 Pivot 语义 | 同时输出 `T_manual_map`、`T_corrected_map_lidar`、Pivot、Δ6DOF |

---

## 10. 验收标准

### 10.1 功能验收

```text
1. 输入 T_map_lidar 后，3D 视图能正确显示 LiDAR 原点 C0 和坐标轴方向。
2. 仅调整 Roll/Pitch/Yaw 时，LiDAR 原点位置不变。
3. 仅调整 Δx/Δy/Δz 时，LiDAR 原点在 map 下按对应数值移动。
4. Reset 后，显示点云、剖面、矩阵和参数完整恢复初始状态。
5. 导出的 corrected_T_map_lidar 重新加载后，显示效果与导出前一致。
6. 四个剖面均以修正后 LiDAR 原点为中心显示。
7. ICP 输出后能正确回写矩阵及 UI 参数，不破坏 Pivot 语义。
```

### 10.2 配准质量验收

人工验收：

```text
X-Z、Y-Z、+45°、-45° 剖面中，地面线、路沿、墙面或固定结构无明显稳定双层、倾斜或高度差。
```

建议辅助指标：

```text
局部最近邻距离中位数 ≤ 3 cm
局部最近邻距离 P95 ≤ 8 cm
```

上述阈值需根据全局地图密度、单帧 LiDAR 精度、植被覆盖和地图建图精度最终确认；ICP 的 Fitness/RMSE 仅作为辅助指标，不可替代剖面人工确认。

---

## 11. 实施阶段

### Phase 1：最小可用版本

```text
- 支持 PCD/PLY 输入；
- 输入全局 map、单帧 map 点云、T_map_lidar；
- C0 周围地图 ROI；
- 3D 主视图；
- X-Z、Y-Z、±45° 剖面；
- 以 LiDAR 原点为 Pivot 的手动 6DOF；
- Reset、Undo/Redo、YAML 导出；
- 变换与 Pivot 单元测试。
```

### Phase 2：数据格式与质量能力

```text
- 支持 LAS/LAZ/COPC；
- PDAL ROI 裁剪；
- 原始 RGB / intensity 颜色保留；
- 最近邻残差统计与差异颜色；
- 剖面 PNG 导出。
```

### Phase 3：自动精修

```text
- point-to-plane ICP / GICP；
- 可配置的静态结构过滤；
- 限幅与失败提示；
- ICP 前后质量指标对比。
```

---

## 12. AI 实现时的禁止事项

```text
1. 不得使用 Source 点云包围盒中心作为旋转中心。
2. 不得在每次输入后累计变换已经变换过的 Source 点。
3. 不得将 ICP 输出 t_icp 直接显示为 LiDAR 原点的实际平移量。
4. 不得只输出一个未标明坐标系、source/target 方向和 Pivot 语义的 transform_matrix。
5. 不得让切片中心固定在 map 原点或初始包围盒中心；必须跟随当前 LiDAR 原点。
6. 不得以 ICP 收敛作为唯一“配准完成”判据。
7. 不得混用行向量/列向量、内旋/外旋或不同欧拉角顺序；统一使用本文定义的列向量和 Z-Y-X 固定轴外旋。
```
