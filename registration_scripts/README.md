# `register_lidar_frame_adjusted.py`

批量将 LiDAR 原始帧与全局地图进行 ICP 配准，并输出校正后的点云及逐帧统计。输入为 LiDAR 坐标系的 binary PCD、同名 `Tr_velo_to_map` TXT 和全局地图。

脚本先根据初始位姿将点云转换到 map 坐标系，在局部地图上运行 ICP；随后把 ICP 的旋转转换回 LiDAR 局部坐标系，应用到完整输入点云，再分别输出 LiDAR/map 坐标系结果。

`audit_lidar_global_map_alignment.py` 是本脚本的依赖脚本，提供局部地图裁剪、体素降采样、ICP、最近邻残差、ZYX 欧拉角分解及 ASCII PCD 读取函数。它不是本 README 的使用对象。

## 目录与兼容性

当前目录结构应为：

```text
registration_scripts/
├─ README.md
└─ src/
   ├─ register_lidar_frame_adjusted.py
   └─ audit_lidar_global_map_alignment.py
```

`register_lidar_frame_adjusted.py` 会优先从同目录导入 `audit_lidar_global_map_alignment.py`，因此可直接执行：

```powershell
python .\src\register_lidar_frame_adjusted.py --help
```

同时保留旧项目中 `tools.golf.audit_lidar_global_map_alignment` 的导入兼容性。

## Python 与依赖

脚本使用 Python 3.10+ 的类型注解语法，要求 Python **3.10 或更高版本**。

建议 `requirements.txt`：

```text
numpy>=1.24
scipy>=1.10
open3d>=0.19
PyYAML>=6.0
```

安装：

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r .\requirements.txt
```

仅在输出 LAS 时需要额外安装：

```powershell
.\.venv\Scripts\python.exe -m pip install laspy
```

## 输入要求

| 输入 | 格式和规则 |
|---|---|
| LiDAR 帧目录 | 顶层 `*.pcd`，按文件名排序处理，不递归扫描 |
| LiDAR 帧 PCD | **binary PCD**；每点严格为 5 个 `float32`：`x,y,z,intensity,time` |
| 标定目录 | 每个 PCD 必须有同名 `<frame_id>.txt` |
| TXT | 第一条 `Tr_velo_to_map:` 行后必须恰好有 12 个数，构成 LiDAR→map 的 3×4 矩阵 |
| 全局地图 | `.pcd`（ASCII 或 binary）或 `.npy`；NPY 形状必须为 `(N, >=3)` |
| `map_anchor.yaml` | 可选；读取 `map_translation_offset_xyz`，从 LiDAR→map 结果中减去该偏移 |

全局地图必须与应用 `map_anchor` 偏移后的点云处于同一坐标范围。

## 示例数据

可从/home/share_data/lyh/data/pcd_converted/3DBox_Annotation_Final_20260601151549_XiangXue_20260323/ 获取

| 输入 | 格式和规则 |
|---|---|
| LiDAR 帧目录 | training/velodyne/ |
| 标定目录 | /home/share_data/mmdet3d_share_data/data/3DBox_Annotation_Final_20260601151549_XiangXue_20260323/training/calib |
| 全局地图 | colored_map_world_voxel_blue_filled.pcd |
| `map_anchor.yaml` | map_anchor.yaml |


## 用法

```text
python register_lidar_frame_adjusted.py \
  --lidar-frame-in DIR \
  --calib-frame-in DIR \
  --global-map FILE \
  --out-dir DIR [options]
```

| 参数 | 必填 | 默认值 | 说明 |
|---|---:|---|---|
| `--lidar-frame-in DIR` | 是 | - | 输入 binary PCD 目录 |
| `--calib-frame-in DIR` | 是 | - | 同名 TXT 标定目录 |
| `--global-map FILE` | 是 | - | ASCII/binary PCD 或 NPY 全局地图 |
| `--out-dir DIR` | 是 | - | 输出根目录；自动创建 |
| `--map-anchor FILE` | 否 | 无 | 含 `map_translation_offset_xyz` 的 YAML |
| `--lidar_corrected_out {0,1}` | 否 | `1` | `1` 输出 LiDAR 坐标系校正点云；`0` 不输出 |
| `--voxel-size FLOAT` | 否 | `0.20` | ICP 使用的体素尺寸 |
| `--crop-margin FLOAT` | 否 | `3.0` | 局部地图裁剪的包围盒外扩距离，单位米 |
| `--icp-threshold FLOAT` | 否 | `0.50` | ICP 最大对应距离，单位米 |
| `--max-frame INT` | 否 | 全部 | 仅处理排序后的前 N 帧；必须大于 0 |
| `--output-format {pcd,las}` | 否 | `pcd` | 校正点云格式；`las` 需要 `laspy` |
| `--z-offset FLOAT` | 否 | `1` | 默认启用全局 Z 校正；数值本身不参与计算，仅作为启用开关 |

### `--z-offset` 行为

默认值为 `1`，因此默认启用 Z 校正：保留 ICP 的旋转和**全局 Z** 平移，丢弃全局 X/Y 平移。

源码通过“参数是否为 `None`”判断是否启用，所以传入任意数值效果相同：

```powershell
--z-offset 1
```

当前 CLI 没有“关闭 Z 校正”的参数；如需仅输出旋转校正，需要修改源码中的默认值或启用判断。

## 最小示例

在 `registration_scripts` 根目录运行：

```powershell
.\.venv\Scripts\python.exe .\src\register_lidar_frame_adjusted.py `
  --lidar-frame-in D:\data\velodyne `
  --calib-frame-in D:\data\calib `
  --global-map D:\data\global_map.pcd `
  --out-dir D:\data\registration_out `
  --max-frame 1
```

使用 NPY 地图和 map anchor：

```powershell
.\.venv\Scripts\python.exe .\src\register_lidar_frame_adjusted.py `
  --lidar-frame-in D:\data\velodyne `
  --calib-frame-in D:\data\calib `
  --global-map D:\data\global_map.npy `
  --map-anchor D:\data\map_anchor.yaml `
  --out-dir D:\data\registration_out
```

## 输出

```text
<out-dir>/
├─ corrected_velodyne/              # 默认生成：LiDAR 坐标系校正点云
├─ corrected_velodyne_map/          # 始终生成：map 坐标系校正点云
├─ registration_summary.jsonl       # 每帧一行 JSON
└─ registration_summary.csv         # 扁平化摘要
```

输出点云名为 `<frame_id>.pcd` 或 `<frame_id>.las`。`corrected_velodyne/` 在 `--lidar_corrected_out 0` 时为空；目录仍会创建。

`registration_summary.jsonl` 成功条目包含：配准前后最近邻统计、ICP `fitness`/`inlier_rmse`、`correction_yaw_pitch_roll_deg`、`z_offset_m`、`correction_matrix_local`、初始 `tr_lidar2map_34_init`、输出路径和状态。单帧失败不会中断整批处理，会写入：

```json
{"frame_id": "...", "status": "failed", "error": "..."}
```
 
## 常见错误

| 现象 | 排查 |
|---|---|
| `No module named 'audit_lidar_global_map_alignment'` | 确认两个脚本位于同一个 `src` 目录，或使用旧 `tools/golf` 包结构。 |
| `No pcd found under ...` | 仅扫描输入目录顶层 `.pcd`，检查路径和文件扩展名。 |
| `DATA binary` / `binary size mismatch` | 输入帧必须是每点 5 个 float32 的 binary PCD；不支持 ASCII 帧 PCD。 |
| `missing Tr_velo_to_map` / `needs 12 numbers` | 检查同名 TXT 是否存在、矩阵行是否正确。 |
| `target crop is empty` | 检查地图和点云坐标范围、初始位姿、`map_anchor` 偏移；必要时增大 `--crop-margin`。 |
| `Open3D is required` | 安装 `open3d`；正常 ICP 和体素降采样需要它。 |
| `LAS output requires laspy` | 使用 `--output-format las` 时安装 `laspy`。 |
| 输出状态为 `failed` | 打开 `registration_summary.jsonl` 对应帧的 `error` 字段定位具体原因。 |

## 参数核对

参数以脚本 `parse_args()` 为准。`--z-offset` 默认值是 `1`，默认已启用全局 Z 校正。
