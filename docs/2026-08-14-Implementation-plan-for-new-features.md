# 单帧点云与全局点云 6DOF 剖面配准工具：增量功能实施方案

## 1. 目的与边界

本说明仅描述在已有单帧 6DOF 配准工具上新增的功能，不重写或改变既有的坐标系、以 LiDAR 原点为 Pivot 的手动调整、ICP/GICP、四方向剖面、Undo/Redo 与残差统计逻辑。

新增目标：

1. 在同一主窗口内直接显示 3D 点云、四个剖面位置及剖面名称；
2. 在每个 2D 剖面中，以 legend 区分 `Adjusted Frame` 与 `Reference Map`；
3. 通过图形界面选择全局地图、单帧点云目录、初始位姿目录、输出目录；
4. 通过 `frame_id` 加载同名单帧点云与初始位姿；
5. 按当前帧名导出 YAML，默认必出；按需导出调整后的 PCD。

本次不增加批量处理、跨帧浏览、自动保存、自动 ICP 或新的点云格式。现有功能的入口、快捷键、矩阵定义和调整结果必须保持兼容。

---

## 2. 固定数据约定

### 2.1 输入路径与帧匹配

| UI 字段 | 选择类型 | 内容与规则 |
|---|---|---|
| `global_map_path` | 单个文件 | 全局地图点云文件，例如 `../global_map_path/colored_map_global_voxel_blue_filled.pcd`；作为固定 Target / Reference。 |
| `frame_cloud_map_path` | 文件夹 | 单帧点云目录，例如 `../frame_cloud_map_path/`；当前只读取 `<frame_id>.pcd`。 |
| `initial_pose_path` | 文件夹 | 初始位姿目录，例如 `../initial_pose_path/`；当前只读取 `<frame_id>.txt`。 |
| `frame_id` | 文本输入 | 不含扩展名的帧名，例如 `1781158324500077000`。 |
| `output_path_yaml` | 文件夹 | YAML 结果输出目录，导出时必填。 |
| `output_path_pcd` | 文件夹 | PCD 结果输出目录；仅在勾选“导出调整后 PCD”时必填。 |

加载时唯一的文件解析规则为：

```text
frame_cloud_file = frame_cloud_map_path / (frame_id + ".pcd")
initial_pose_file = initial_pose_path / (frame_id + ".txt")
```

例如 `frame_id = 1781158324500077000` 时，必须成对加载：

```text
../frame_cloud_map_path/1781158324500077000.pcd
../initial_pose_path/1781158324500077000.txt
```

`frame_id` 仅允许文件基名字符：字母、数字、`_`、`-`、`.`；UI 输入若带 `.pcd` 或 `.txt`，应自动去掉扩展名后再匹配。不得接受路径分隔符，避免路径穿越或错误拼接。

### 2.2 初始位姿文本解析

初始位姿 `.txt` 可能包含 `P0`～`P3`、`R0_rect`、`Tr_velo_to_cam`、`Tr_imu_to_velo` 等无关字段。只读取键名为 **`Tr_velo_to_map`** 的这一行。

```text
Tr_velo_to_map: r00 r01 r02 tx r10 r11 r12 ty r20 r21 r22 tz
```

解析规则：

1. 从首个匹配 `Tr_velo_to_map:` 的行读取后续 12 个浮点数；
2. 按行优先顺序 reshape 为 3×4 矩阵；
3. 拼接最后一行 `[0, 0, 0, 1]`，得到 `T0 = T_map_lidar`；
4. `C0 = T0[:3, 3]` 是 LiDAR 原点在 map 下的位置；矩阵左上角 `R0 = T0[:3, :3]` 决定 LiDAR 三轴在 map 下的方向；
5. P0/P1/P2/P3 等字段不得参与位姿计算，也不得覆盖 `Tr_velo_to_map`。

对示例数据，得到：

```text
T0 = [[ 0.807819, -0.589212,  0.016066, 8628.860000],
      [ 0.588546,  0.807800,  0.032764, 9650.010000],
      [-0.032282, -0.017011,  0.999334,  106.995000],
      [ 0.000000,  0.000000,  0.000000,    1.000000]]
```

加载前必须校验：`Tr_velo_to_map` 唯一、数值数量恰为 12、全部为有限数值、`det(R0)` 接近 +1、`R0.T @ R0` 接近单位阵。校验失败时不加载、不改变当前已加载帧，并在界面显示文件路径和具体原因。

---

## 3. 主窗口 UI 增量

### 3.1 路径与帧控制区

在现有主窗口的顶部或左侧增加“数据与输出”折叠面板；路径必须使用可编辑输入框，旁边提供浏览按钮，不能只依赖命令行参数。

```text
数据与输出
├─ 全局地图文件       [ QLineEdit                                  ] [选择文件]
├─ 单帧点云目录       [ QLineEdit                                  ] [选择文件夹]
├─ 初始位姿目录       [ QLineEdit                                  ] [选择文件夹]
├─ Frame ID           [ 1781158324500077000                       ] [加载当前帧]
├─ YAML 输出目录      [ QLineEdit                                  ] [选择文件夹]
├─ [ ] 导出调整后 PCD（默认未勾选）
└─ PCD 输出目录       [ QLineEdit                                  ] [选择文件夹]  （未勾选时禁用）
```

交互要求：

1. 用户先填路径和 `frame_id`，点击“加载当前帧”后才执行文件存在性检查、位姿解析、地图 ROI 裁剪与渲染；路径编辑时不得自动反复加载大点云。
2. 全局地图切换后必须重新加载 Reference；单帧目录、位姿目录或 `frame_id` 切换后只重载 Source 与 `T0`。
3. 成功加载后在状态栏显示：`frame_id`、两份实际文件的绝对路径、点数、`C0` 和加载耗时。
4. 加载新帧时，清空上一帧的手动增量、ICP 增量、Undo/Redo 栈和质量统计；重新以新 `T0` 作为初始状态。
5. 路径为空、文件不存在、PCD 无点、`Tr_velo_to_map` 缺失或矩阵非法时，“导出”按钮应禁用；不得用上一帧的位姿配准新一帧点云。
6. 允许 global map 是 PCD；本增量的帧输入及可选点云输出固定为 PCD。其他既有格式支持不应被删除。

### 3.2 主 3D 视图

3D 场景必须位于该主窗口中，使用现有 `SceneWidget` / 等效嵌入式渲染组件；禁止用 `draw_geometries()` 打开独立临时窗口。

主视图应同时显示：

| 对象 | 数据与样式 |
|---|---|
| `Reference Map` | 当前 LiDAR 原点邻域内的全局地图 ROI；青灰色 `#90A4AE`。 |
| `Adjusted Frame` | 当前帧 map 点云应用 `T_manual_map` 后的结果；黄橙色 `#F6C445`。 |
| LiDAR Origin | 当前 `C_current` 的坐标轴/十字标记与 `LiDAR Origin` 标签。 |
| X-Z / 0° | 以 `C_current` 为中心、方向固定在 map X 轴的红色切片矩形，标签 `X-Z / 0°`。 |
| Y-Z / 90° | 以 `C_current` 为中心、方向固定在 map Y 轴的绿色切片矩形，标签 `Y-Z / 90°`。 |
| Diag +45° | 蓝色切片矩形，标签 `Diag +45°`。 |
| Diag -45° | 紫色切片矩形，标签 `Diag -45°`。 |

切片矩形四个顶点按既有切片定义计算：

\[
V_{1,2,3,4}=C_{current}\pm L u_\theta \pm \frac{w}{2}v_\theta
\]

其中 `L` 是剖面半长，`w` 是剖面厚度。剖面标签放在 `C_current + L*u_theta` 附近。旋转或平移后，Reference、Adjusted、LiDAR Origin、4 个矩形与标签必须在同一次刷新中同步更新。

性能要求：全局地图 ROI 仅在全局文件或 ROI 配置改变时重建；每次 6DOF 微调只更新 Adjusted 点、LiDAR Origin、4 个矩形与标签，不重复读取全局地图或创建额外窗口。

### 3.3 2D 剖面 Legend

每个 X-Z、Y-Z、+45°、-45° 剖面右上角均显示下列固定图例：

| 图例文字 | 点数据 | 颜色 | 绘制顺序 |
|---|---|---|---|
| `Reference Map（全局点云）` | 该剖面内的全局地图 ROI 点 | `#90A4AE` | 先绘制 |
| `Adjusted Frame（单帧点云）` | 应用当前 `T_manual_map` 后落入该剖面的单帧点 | `#F6C445` | 后绘制 |

要求：

1. 文本必须为 `Adjusted`，不要使用 `Adjucted`；
2. 图例代表点云来源，不代表剖面方向；3D 剖面红/绿/蓝/紫色不能用于替换 2D 点云来源色；
3. Reference 和 Adjusted 必须是独立图元。每次刷新只更新其点坐标，legend 不应消失、重复累积或与实际颜色不一致；
4. 图例背景半透明，位置为右上角；如果用户关闭某一层，应同步隐藏该层图元及图例项，或以灰色禁用态显示；
5. 对重叠点先绘制 Reference，再绘制 Adjusted，使单帧微调效果可见。

---

## 4. 加载、调整和导出链路

### 4.1 加载链路

```text
用户选择路径并输入 frame_id
  → 根据 frame_id 组装 .pcd 与 .txt 路径并校验存在
  → 读取 global_map_path 作为 Reference
  → 读取 <frame_id>.pcd 作为 Source（已在 map 坐标系）
  → 解析 <frame_id>.txt 中 Tr_velo_to_map，得到 T0 / C0
  → 清零本帧手动增量、ICP 增量及历史栈
  → 以 C0 构建全局地图 ROI
  → 渲染主 3D 视图和四个剖面
```

### 4.2 调整链路（保持不变）

Source 已在 map 坐标系。调整仍按既有约定，以初始 LiDAR 原点 `C0` 为 Pivot：

\[
P_{map}^{adjusted}=R_\Delta(P_{map}^{source}-C_0)+C_0+\Delta C_{map}
\]

\[
T_{manual}^{map}=\begin{bmatrix}
R_\Delta & C_0-R_\Delta C_0+\Delta C_{map}\\
0 & 1
\end{bmatrix}
\]

\[
T_{corrected\_map\_lidar}=T_{manual}^{map}T_0
\]

本增量不得把 PCD 内已经在 map 坐标系的点再次左乘 `T0`。PCD 导出和 3D/2D 显示均使用 `T_manual_map` 作用于原始 `frame_cloud_map` 点。

### 4.3 导出链路与命名

导出基名取**实际已加载的单帧 PCD 文件的 stem**，不使用用户未校验的原始输入字符串。假设加载文件为：

```text
D:/1_data/map_seg/global_map/jinhua/velodyne_map/1781158324500077000.pcd
```

则：

```text
frame_stem = 1781158324500077000
YAML 输出 = output_path_yaml/1781158324500077000.yaml       （必出）
PCD 输出  = output_path_pcd/1781158324500077000.pcd         （仅勾选时输出）
```

导出按钮行为：

1. 未选择 `output_path_yaml` 时，拒绝导出并提示；
2. 始终写出 `<frame_stem>.yaml`；
3. “导出调整后 PCD”复选框默认关闭；勾选后要求 `output_path_pcd` 有效，并写出 `<frame_stem>.pcd`；
4. 同名文件存在时先弹出覆盖确认；写入使用临时文件再原子替换，避免中断后留下半个结果文件；
5. 导出成功后状态栏显示最终绝对路径；YAML 与 PCD 成败分别提示。

YAML 至少包含以下内容，以便结果可复现和追溯：

```yaml
frame_id: "1781158324500077000"
input:
  global_map_path: "..."
  frame_cloud_map_path: ".../1781158324500077000.pcd"
  initial_pose_path: ".../1781158324500077000.txt"

initial_T_map_lidar: [[...], [...], [...], [...]]
manual_delta_about_lidar_origin:
  translation_frame: map
  rotation_axes_frame: map
  pivot_initial_lidar_origin_map: [Cx, Cy, Cz]
  dx_m: 0.0
  dy_m: 0.0
  dz_m: 0.0
  roll_deg: 0.0
  pitch_deg: 0.0
  yaw_deg: 0.0
T_manual_map: [[...], [...], [...], [...]]
corrected_T_map_lidar: [[...], [...], [...], [...]]
output:
  adjusted_pcd_written: false
  adjusted_pcd_path: null
```

若 PCD 已导出，将 `adjusted_pcd_written` 置为 `true`，并写入其实际绝对路径。YAML 中的 `frame_cloud_map_path` 为本次实际加载的单帧文件；名称虽然保留既有字段名，但值不是目录。

---

## 5. 建议模块改动

| 模块 | 增加职责 | 关键接口 |
|---|---|---|
| `ui/data_io_panel.py` | 路径输入、文件/文件夹对话框、frame_id、导出复选框与按钮状态 | `get_load_request()`、`get_export_request()` |
| `io/frame_loader.py` | 拼接帧文件路径、读取 PCD、解析 `Tr_velo_to_map` | `load_frame(request) -> FrameData` |
| `io/pose_parser.py` | 校验并将 12 数值转换为 4×4 `T_map_lidar` | `parse_tr_velo_to_map(path) -> np.ndarray` |
| `ui/scene_3d.py` | 主窗口内嵌 3D 点云、LiDAR Origin、四个剖面矩形和 3D 标签 | `set_reference()`、`set_adjusted()`、`update_slice_overlays()` |
| `ui/profile_view.py` | 两组独立剖面点图元和稳定 legend | `set_reference_points()`、`set_adjusted_points()` |
| `io/exporter.py` | 基于实际 frame stem 写 YAML、可选 PCD、覆盖确认后的原子写入 | `export_result(request, state)` |
| `app/controller.py` | 串联加载、清状态、渲染、调整与导出 | `load_current_frame()`、`refresh_views()`、`export_current_frame()` |

禁止在 UI 层解析矩阵、在导出层重新计算 6DOF，或在视图层直接拼接文件路径；这些职责必须分别由 `pose_parser`、现有变换核心和 `frame_loader` 承担。

---

## 6. 实施顺序

1. 新增 `LoadRequest`、`ExportRequest`、`FrameData` 数据结构及路径校验单元测试；
2. 实现 `Tr_velo_to_map` 解析与 4×4 矩阵校验；
3. 增加路径/帧/输出 UI 面板与“加载当前帧”状态机；
4. 将现有 3D SceneWidget 置于主窗口布局，补充 Reference、Adjusted、Origin、四个矩形和标签的增量更新；
5. 将四个 2D 剖面改为 Reference/Adjusted 独立图元，并增加固定 legend；
6. 实现以实际 PCD stem 命名的 YAML 必出与 PCD 可选导出；
7. 进行端到端验证、性能检查和异常路径验证。

---

## 7. 验收标准

### 7.1 文件加载与位姿

1. 选择上述三个输入路径，输入 `1781158324500077000` 后，工具只加载同名 `.pcd` 与 `.txt`。
2. 不存在同名文件、空 PCD、缺少 `Tr_velo_to_map`、非 12 个数或非刚体旋转时，加载失败且当前有效帧不被覆盖。
3. `Tr_velo_to_map` 的平移正确成为 `C0`，旋转正确决定 LiDAR 原点三轴方向；P0/P1/P2/P3 不影响结果。
4. 切换帧后，手动调整、ICP 调整和历史栈归零，Reference 地图仍可复用。

### 7.2 可视化

1. 主窗口内同时可见 Reference、Adjusted、LiDAR Origin、X-Z/Y-Z/+45°/-45° 四个切片矩形和各自名称；不产生独立 3D 弹窗。
2. 任意 Δ6DOF、剖面长度或厚度变化后，主 3D 叠加层和四个 2D 剖面在同一次刷新中同步变化。
3. 四个 2D 剖面均有来源一致的 legend：Reference 青灰、Adjusted 黄橙；图例不重复、不丢失，且 Adjusted 拼写正确。

### 7.3 导出

1. 对加载的 `1781158324500077000.pcd`，默认导出文件名必须是 `1781158324500077000.yaml`，不再使用固定的 `alignment_result.yaml`。
2. 未勾选 PCD 时，只生成 YAML；勾选后生成同名 `.pcd` 与 `.yaml`，分别位于各自配置的输出文件夹。
3. YAML 内的 `initial_T_map_lidar`、`T_manual_map`、`corrected_T_map_lidar` 与当前界面一致；`pivot_initial_lidar_origin_map` 等于初始 `C0`。
4. 导出的 PCD 点坐标与在界面中显示的 Adjusted 点一致；重新读取该 PCD 并应用 YAML 中的 `T_manual_map` 不得发生第二次变换。
5. PCD 输出失败不能使 YAML 内容伪造为“已输出”；错误状态应清晰提示并保留可重试能力。

---

## 8. 不允许的实现方式

```text
1. 不得通过扫描目录后按排序序号选择帧；必须严格按 frame_id 同名匹配。
2. 不得解析 P0/P1/P2/P3 或 Tr_velo_to_cam 作为 map 位姿。
3. 不得把 T0 再作用到已经处于 map 坐标系的单帧 PCD。
4. 不得使用点云包围盒中心作为 Pivot；始终使用 C0 = T0[:3, 3]。
5. 不得将 3D 视图单独弹窗，或只显示剖面而不显示实际点云。
6. 不得把 Reference 与 Adjusted 合成一个点图元，导致 legend 与来源失去对应关系。
7. 不得固定输出为 alignment_result.yaml，或用 frame_id 的未验证原始字符串拼接输出路径。
8. 不得在未选择 YAML 输出目录时静默写到工作目录、输入目录或临时目录。
```
