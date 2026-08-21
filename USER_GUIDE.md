# 用户手册：单帧点云与全局地图 6DOF 剖面配准工具

## 1. 项目名称与作用

这是 Windows 桌面 GUI，用于逐帧检查已经在 `map` 坐标系的点云与全局地图的重合关系。推荐入口为 `src/frame_align_6dof.py`；`frame_alignment/` 是核心实现。历史 `src/frame_register_manual.py` 不属于推荐工作流。

## 2. 解决的问题、适用场景和边界

它解决“已粗定位的单帧 map 点云仍需人工检查/微调”的问题，适用于：有全局地图、逐帧 map 点云、同名初始位姿 TXT 的人工配准任务；或已有外部配准点云/JSONL 的结果审核任务。

不支持：全局建图、原始 LiDAR 点云到 map 的自动转换、全局粗配准、回环检测、批量无人值守配准、Web/数据库协作、Docker 镜像及经过正式验证的 WSL/Linux 运行流程。

## 3. 当前功能

- `register` 配准模式：6DOF 编辑、撤销/重做/重置、匹配误差、受限 ICP、YAML 和可选 PCD 导出。
- `review` 审核模式：已配准点云浏览、自动邻帧加载、当前位置、剖面、JSONL 统计与 GT 对比。
- 3D 视图：地图 ROI、调整后帧、初始/当前 LiDAR 原点和 LiDAR XYZ 轴。
- 2×3 剖面、云颜色控制、全局地图缓存、失败加载保留上一成功帧、导出原子提交/回滚。

## 4. 系统、Python 和依赖

已确认工作方向为 Windows + PowerShell + 可用桌面 OpenGL。Python 正式支持范围未声明，以依赖在目标环境实际安装结果为准。运行依赖以 `requirements.txt` 为准：NumPy、SciPy、Open3D、PySide6、PyQtGraph、PyOpenGL、PyYAML、laspy。`pytest` 是测试依赖，不是运行依赖。

```powershell
cd D:\0_code\feature_new
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install --upgrade pip
.\.venv\Scripts\python.exe -m pip install -r .\requirements.txt
.\.venv\Scripts\python.exe -c "import numpy, scipy, open3d, PySide6, pyqtgraph, OpenGL, yaml, laspy; print('runtime imports OK')"
```

完整测试另行安装 `pytest`：

```powershell
.\.venv\Scripts\python.exe -m pip install pytest
```

## 5. CLI、自检与启动方式

```powershell
.\.venv\Scripts\python.exe .\src\frame_align_6dof.py --help
.\.venv\Scripts\python.exe .\src\frame_align_6dof.py --self-test
```

预期自检输出：`Self-test passed`。

| 参数 | 作用 |
|---|---|
| `--config FILE` | YAML 配置；内部相对路径相对 YAML 目录 |
| `--map FILE` | 兼容单文件模式的全局地图 |
| `--source FILE` | 已在 map 坐标系的单帧点云 |
| `--pose FILE` | TXT `Tr_velo_to_map` 或 4×4 位姿 YAML |
| `--output-dir DIR` | 预填 YAML 输出目录；兼容模式下也用于 PCD 输出 |
| `--self-test` | 不读取真实数据的矩阵自检 |

`--map`、`--source`、`--pose` 必须同时给出。无参数打开空 GUI。

## 6. 输入数据、目录和 Frame ID

推荐的配准模式目录：

```text
data/
├─ global_map.pcd
├─ frames/
│  ├─ 1001.pcd
│  └─ 1002.pcd
├─ poses/
│  ├─ 1001.txt
│  └─ 1002.txt
└─ output/
   ├─ yaml/
   └─ pcd/
```

全局地图支持 `.pcd`、`.ply`、`.las`、`.laz`，且必须是非空 XYZ。帧目录只扫描顶层 `.pcd`。Frame ID 是 stem，只允许字母、数字、`_`、`-`、`.`；输入末尾 `.pcd`/`.txt` 会自动去掉，禁止路径分隔符、`.`、`..`。

配准模式要求 `<frame_id>.pcd` 和 `<frame_id>.txt` 同名存在。TXT 必须只有一条以下格式，且后接 12 个有限数值：

```text
Tr_velo_to_map: 1 0 0 10 0 1 0 20 0 0 1 30
```

兼容 `--pose` 还支持含 `matrix`、`T_map_lidar` 或 `initial_T_map_lidar` 的 4×4 YAML。

## 7. 坐标系、map_anchor 和矩阵约定

**单帧点云必须已在 map 坐标系，且不会再次乘初始位姿。** 初始位姿仅用于 LiDAR 原点 C0、地图 ROI、旋转中心、轴显示和导出姿态。

若点云/地图已经整体偏移、TXT 位姿仍在旧 world 范围，在地图同目录放置：

```yaml
map_translation_offset_xyz: [50000.0, 10000.0, 0.0]
```

程序只执行 `pose_map_translation = pose_world_translation - offset`，不会变换 PCD 或旋转。

人工编辑量为 map 坐标系：ΔX/ΔY/ΔZ 分别沿 map X/Y/Z；yaw/pitch/roll 分别绕 map Z/Y/X，按 ZYX 顺序；旋转中心为 `C0 = initial_T_map_lidar[:3, 3]`。

```text
T_manual_map = [R, C0 - R @ C0 + Δt]
corrected_T_map_lidar = T_manual_map @ initial_T_map_lidar
P_map_corrected = T_manual_map @ P_map
```

新导出 YAML 的 `manual_delta_lidar` 是局部点云校正量：

```text
C_local = inverse(initial_T_map_lidar) @ corrected_T_map_lidar
corrected_lidar = C_local @ original_lidar
```

它采用 LiDAR 坐标系、`original_to_corrected` 方向、ZYX 欧拉角；与 map 坐标系的 `manual_delta_about_lidar_origin` 不能直接混比。

## 8. 配置文件字段

路径均相对 `config.yaml` 所在目录：

| 字段 | 模式 | 说明 |
|---|---|---|
| `mode` | 两者 | `register`（默认）或 `review` |
| `global_map_path` | 两者 | 全局地图文件 |
| `frame_cloud_map_path` | register | 输入 map 点云目录 |
| `initial_pose_path` | register | TXT 位姿目录 |
| `registered_cloud_path` | review | 已配准 map 点云目录 |
| `registered_pose_path` | review | 可选 YAML 文件或目录 |
| `registration_summary_path` | review | 可选 JSONL 统计 |
| `frame_id` | 两者 | 初始帧 |
| `output_path_yaml` / `output_path_pcd` | register | 已存在的输出目录 |
| `display.map_roi_radius_m` | 两者 | ROI 半径，默认 35 m |
| `display.display_voxel_m` | 两者 | 显示体素，默认 0.05 m |
| `display.slice_half_length_m` | 两者 | 剖面半长，10–35 m，默认 20 m |
| `display.slice_thickness_m` | 两者 | 剖面总厚度，默认 0.20 m |
| `display.clouds` | 两者 | `global`/`source` 的颜色、色图设置 |
| `interaction.allow_manual_adjustment` | register | 启用 6DOF、重置、撤销/重做、ICP |
| `interaction.*step*` | register | 普通和 Shift 大步长 |
| `error.method` | register | `nearest_neighbor`、`symmetric_chamfer`、`point_to_plane` |
| `error.evaluation_radius_m` / `match_threshold_m` | register | 误差 ROI 和匹配阈值 |

配准配置示例：

```yaml
mode: register
global_map_path: ./data/global_map.pcd
frame_cloud_map_path: ./data/frames
initial_pose_path: ./data/poses
frame_id: "1001"
output_path_yaml: ./alignment_output/yaml
output_path_pcd: ./alignment_output/pcd
interaction:
  allow_manual_adjustment: true
```

## 9. 配准模式完整操作

1. 设为 `mode: register`，填写地图、帧目录、位姿目录、YAML 输出目录。
2. 点击“刷新帧列表”，选择/输入 Frame ID。
3. 点击“加载当前帧”或按 `Z`。状态栏显示文件、点数、C0 与耗时。
4. 检查 3D 中 Reference Map、Adjusted Frame、LiDAR Origin/XYZ 轴及各剖面。
5. 通过按钮、数值框、快捷键调整；必要时先按 `C` 计算误差，再按 `V` ICP。
6. 点击“导出当前帧”或按 `G`，处理覆盖确认。

配准模式切换上一/下一帧只改变选择，不自动加载。成功加载新帧会重置调整和质量；加载失败保留上一成功帧。

“已标注/未标注”和标注数根据 YAML 输出目录同名文件计算。“导出调整后 PCD”开启后，PCD 输出目录也必须存在。“计算误差”不改变姿态；人工改变姿态后当前误差指标失效，应重新计算或 ICP。

“点云渲染”面板分别控制 Global 全局点云和 Source 单帧点云的显示方式：`uniform` 单色、`native` 原始颜色（没有原始颜色时回退为单色）、`cmap` 色图；可选择色图和单色。该面板只改变显示，不修改输入或导出数据。

## 10. 6DOF、快捷键、撤销和导出

| 操作 | 快捷键 | 适用 |
|---|---|---|
| 上/下帧 | `Left` / `Right` | 两种模式 |
| 上/下 10 帧 | `Shift+Left` / `Shift+Right` | 两种模式 |
| 加载当前帧 | `Z` | 两种模式 |
| ΔX −/+ | `A` / `D` | 配准、编辑启用 |
| ΔY −/+ | `S` / `W` | 同上 |
| ΔZ −/+ | `E` / `Q` | 同上 |
| roll −/+ | `9` / `7` | 同上 |
| pitch −/+ | `6` / `4` | 同上 |
| yaw −/+ | `3` / `1` | 同上 |
| 重置/导出/误差/ICP | `R` / `G` / `C` / `V` | `R/G/V` 为配准模式；`C` 不改变位姿 |

按住 Shift 调整 6DOF 时使用配置的大步长。撤销、重做和重置只影响人工增量；ICP 成功后继续手动调整时，最终输出包含先前手动量、ICP 累积量和后续手动量。

## 11. 剖面视图

默认 2×3：固定 `(0,0)` XZ/0°、`(1,0)` YZ/90°；可编辑不可删除 `(0,1)` 平行 XZ/+10 m、`(1,1)` 平行 YZ/+10 m；可编辑可删除 `(0,2)` +30°、`(1,2)` -60°。

剖面水平基准跟随**校正后 LiDAR yaw**：XZ 沿 LiDAR 水平 X，YZ 沿 LiDAR 水平 Y；高度轴始终为 `map Z`，pitch/roll 不会使高度轴倾斜。角度模式相对 XZ 基准；平行模式选择 XZ/YZ 并给出有符号横向偏移。正 XZ 偏移沿 yaw 相关 +Y，正 YZ 偏移沿 yaw 相关 −X。

半长控制显示距离（10–35 m）；厚度是选取带总厚度。新增、删除或编辑剖面仅在当前 GUI 会话有效。

## 12. ICP 和误差解释

ICP 仅在配准模式且人工调整启用时可运行。它对**当前已人工调整的点云**做 point-to-plane ICP，成功结果累积进当前位姿，不会清除手动调整。

固定 ICP 参数：ROI 25 m、体素 0.08 m、最大对应距离 0.35 m、最大 50 次迭代；源/目标 ROI 各少于 10 点会拒绝。若本次 ICP 让当前中心移动超过 0.5 m 或旋转超过 3°，也拒绝应用。

ICP 记录最近邻残差中位数/P95、`icp_rmse_m`、`icp_fitness`。`C` 使用配置的误差方法输出 median、mean、RMSE、P95、`match_ratio`；点到平面方法会估计目标法向。

## 13. 审核模式配置与使用

```yaml
mode: review
global_map_path: D:/data/global_map.pcd
registered_cloud_path: D:/data/corrected_velodyne_map
registered_pose_path: D:/data/gt_manual/yaml       # 可选
registration_summary_path: D:/data/summary.jsonl   # 可选
frame_id: "1001"
```

审核点云必须为 `<frame_id>.pcd`。若同帧 YAML 不存在、目录未提供或无效，使用该点云 XYZ 坐标逐维中位数为中心、单位旋转为朝向，仍可加载。若 YAML 有效，须包含合法 `corrected_T_map_lidar`。

审核模式只读：6DOF、ICP、导出禁用。`Left/Right`、`Shift+Left/Right` 切换后立即加载；`Z` 可重载当前帧。状态栏显示例如 `当前位置：190/879`。

## 14. 配准统计与 GT 对比

矩阵面板从 `registration_summary_path` 或“选择”按钮加载 JSONL；每个非空行必须是带 `frame_id` 的 JSON，对同一帧后出现的条目覆盖前者。

自由度表：`dof | corr | GT | Δ`。

- `corr`：JSONL 的外部配准局部 correction，例如 `correction_yaw_pitch_roll_deg`、`z_offset_m`/`correction_matrix_local`。当前受限外部配准中 tx、ty 按 0 显示。
- `GT`：同帧 YAML 的 `manual_delta_lidar`；缺失时由 `initial_T_map_lidar` 和 `corrected_T_map_lidar` 推算。
- `Δ`：`corr - GT`。

二者均为 LiDAR 局部坐标系、`original_to_corrected` 方向、ZYX 欧拉角时才可直接比较。不要将 map 坐标系的 `manual_delta_about_lidar_origin` 直接作为 corr 的 GT。

统计表读取 JSONL 的 `corrected`、`identity`：`count`、`max_m`、`mean_m`、`p50_m`、`p95_m`、`rmse_m`；显示 `corrected | identity | Δ`。count 为整数，零显示 `0`，其余保留四位小数。

## 15. YAML / PCD 输出与异常处理

输出名以**实际加载 PCD 的 stem**确定：`<stem>.yaml` 和可选 `<stem>.pcd`。显示降采样仅影响视图/局部计算；导出 PCD 使用完整输入帧点集并应用一次 `T_manual_map`，不会再乘初始位姿。

YAML 写入：输入绝对路径、`initial_T_map_lidar`、map 坐标系 `manual_delta_about_lidar_origin`、`T_manual_map`、`corrected_T_map_lidar`、局部 `manual_delta_lidar`、质量字段及 PCD 实际写入状态。

写入前使用临时文件。覆盖现有目标时 GUI 需确认；YAML 和可选 PCD 在替换阶段使用备份/回滚。若可选 PCD 临时写失败，YAML 仍会输出，界面同时报告 PCD 错误。

## 16. 测试和验收

```powershell
$env:QT_QPA_PLATFORM='offscreen'
.\.venv\Scripts\python.exe -m pytest -q
```

当前测试覆盖 CLI、配置路径、帧加载、map anchor、位姿、剖面几何/布局、GUI、当前姿态 ICP、误差、导出颜色和回滚、审核加载/无位姿回退/自动导航以及 JSONL 统计。部分真实样本测试在本机数据缺失时跳过。

验收应确认：GUI 能打开；帧/地图非空；ROI 与剖面可见；方向和快捷键正确；导出 stem 正确；PCD 状态正确。

## 17. 常见问题和已知限制

- `ModuleNotFoundError`：确认安装和启动使用同一个 `.venv\Scripts\python.exe`。
- 配置/相对路径错误：`--config` 指向真实文件；配置相对路径相对 YAML 本身。
- GUI/3D 失败：检查 Windows 桌面会话、显卡 OpenGL 驱动、PyQtGraph/PyOpenGL；`QT_QPA_PLATFORM=offscreen` 只用于测试。
- ROI 为空：检查地图、点云、位姿是否在同一坐标范围；检查 `map_anchor.yaml` 偏移符号与 ROI 半径。
- LAZ 失败：依赖本机 laspy 的可用解压 backend，项目未捆绑或固定 backend。
- ICP 拒绝：检查 ROI 点数、粗调、对应距离和 0.5 m/3° 安全限制。
- 导出按钮不可用：先加载成功；YAML 输出目录必须存在；勾选 PCD 时 PCD 目录也必须存在。
- 审核统计为空/GT 为 `-`：检查 JSONL 当前 `frame_id`、统计路径、同帧 YAML 与必要字段。

已知限制：大地图读取、ROI、降采样和 ICP 在 GUI 线程执行，可能阻塞窗口；剖面设置不持久化；审核模式不能编辑或导出；Python、WSL/Linux、Docker 和 LAZ backend 的正式兼容性尚未声明。
