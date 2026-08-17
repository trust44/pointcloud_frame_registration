# 单帧点云与全局点云 6DOF 剖面配准工具

这是一个面向点云标注与配准复核的 Windows 桌面 GUI。它将一帧**已经位于 `map` 坐标系**的 PCD 与全局地图叠加显示，支持人工 6DOF 微调、LiDAR 航向相关剖面对比、受限 ICP 辅助优化，并把校正矩阵和质量指标导出为 YAML；也可同时导出校正后的 PCD。

推荐入口是 `src/frame_align_6dof.py`，功能代码位于 `frame_alignment/`。历史脚本 `src/frame_register_manual.py` 保持不变，不是本文推荐入口。

详细的数据格式、界面操作和故障排查见 [USER_GUIDE.md](USER_GUIDE.md)。

## 主要功能

- 同时显示全局地图 Reference、校正后单帧 Adjusted、LiDAR 原点和当前 LiDAR XYZ 轴。
- 从单帧目录扫描顶层 `.pcd`，按 Frame ID 浏览、手动输入、上一帧/下一帧切换和显式加载。
- 根据 YAML 输出目录中的 `<frame_id>.yaml` 显示“已标注/未标注”和 `已标注数/总帧数`。
- 在 `map` 坐标系调整 ΔX、ΔY、ΔZ、roll、pitch、yaw；支持键盘步进、Shift 大步长、重置、撤销和重做。
- 默认显示 XZ、YZ、`+45°`、`-45°` 四个剖面；最多新增两个会话内自定义剖面，布局由 2×2 自动变为 2×3。
- XZ/YZ 及自定义剖面的水平方向跟随校正后 LiDAR yaw，高度方向始终为 `map Z`；支持角度和相对 XZ/YZ 的有符号平行偏移。
- 可调整剖面半长 `10～35 m` 和剖面厚度。
- 提供带平移/旋转幅度限制的 point-to-plane ICP，并显示最近邻残差、ICP RMSE 和 fitness。
- 必选导出 YAML，可选导出完整校正后 PCD；同名文件覆盖前确认，写入采用临时文件和回滚机制。
- 全局地图路径不变时复用地图缓存；加载失败不会清除上一帧已成功加载的数据。

## 核心输入

| 输入 | 格式与要求 |
|---|---|
| 全局地图 | `.pcd`、`.ply`、`.las` 或 `.laz`；必须包含非空 XYZ 点集 |
| 单帧点云 | 目录顶层的 `<frame_id>.pcd`；必须已处于与全局地图一致的 `map` 坐标范围 |
| 初始位姿 | `<frame_id>.txt`，包含且只包含一条 `Tr_velo_to_map:`，后接 12 个有限数值组成 3×4 刚体矩阵 |
| Frame ID | 字母、数字、下划线、连字符和点；输入末尾的 `.pcd`/`.txt` 会被去除 |
| YAML 输出目录 | 必须预先存在；输出名为实际加载 PCD 的 stem 加 `.yaml` |
| PCD 输出目录 | 仅勾选“导出调整后 PCD”时需要，且必须预先存在 |
| 地图锚点（可选） | 全局地图同目录的 `map_anchor.yaml`，用于把 world 坐标位姿平移到已偏移的地图坐标范围 |

目录模式要求每帧点云和位姿同名：

```text
data/
├─ global_map.pcd
├─ frames/
│  ├─ 1001.pcd
│  └─ 1002.pcd
└─ poses/
   ├─ 1001.txt
   └─ 1002.txt
```

位姿 TXT 示例：

```text
Tr_velo_to_map: 1 0 0 10 0 1 0 20 0 0 1 30
```

## 坐标与矩阵约定

- 全局地图和单帧点云必须已经位于相同的 `map` 坐标范围。
- 单帧点不会再次乘以初始 `T_map_lidar`。
- 人工平移和旋转轴均使用 `map` 坐标系。
- 旋转围绕初始 LiDAR 原点 `C0 = initial_T_map_lidar[:3, 3]`。
- 对单帧齐次点 `P_map`：

  ```text
  P_map_corrected = T_manual_map @ P_map
  corrected_T_map_lidar = T_manual_map @ initial_T_map_lidar
  ```

若全局地图和单帧点云已经整体平移，但 TXT 位姿仍在原 world 坐标范围，可在全局地图同目录放置：

```yaml
map_translation_offset_xyz: [50000.0, 10000.0, 0.0]
```

程序自动执行：

```text
initial_pose_map_translation = initial_pose_world_translation - map_translation_offset_xyz
```

`map_anchor.yaml` 只校正位姿平移，不变换 PCD 点。文件不存在时保持原位姿不变。

## 环境安装

仓库正式声明的运行依赖见 `requirements.txt`。仓库未声明正式 Python 版本范围；当前依赖能否安装应以目标 Python 环境实际结果为准。

以下命令在 Windows PowerShell、项目根目录执行：

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r .\requirements.txt
.\.venv\Scripts\python.exe .\src\frame_align_6dof.py --self-test
```

GUI 需要可用的桌面 OpenGL 环境。仓库没有提供 Docker、WSL 或 Linux 启动方案，其正式支持状态需确认。

## 启动方式

### 推荐：配置文件启动

编辑仓库根目录已有的 `config.yaml`，并确保输入文件和输出目录存在：

```yaml
global_map_path: ./data/global_map.pcd
frame_cloud_map_path: ./data/frames
initial_pose_path: ./data/poses
frame_id: "1781158324500077000"

output_path_yaml: ./alignment_output/yaml
output_path_pcd: ./alignment_output/pcd

display:
  map_roi_radius_m: 35.0
  display_voxel_m: 0.05
  slice_half_length_m: 20.0
  slice_thickness_m: 0.20

interaction:
  translation_step_m: 0.01
  translation_large_step_m: 0.10
  rotation_step_deg: 0.05
  rotation_large_step_deg: 0.50
```

相对路径以 `config.yaml` 所在目录为基准。创建示例输出目录并启动：

```powershell
New-Item -ItemType Directory -Force .\alignment_output\yaml, .\alignment_output\pcd
.\.venv\Scripts\python.exe .\src\frame_align_6dof.py --config .\config.yaml
```

### 可选：空界面启动

```powershell
.\.venv\Scripts\python.exe .\src\frame_align_6dof.py
```

启动后在右侧“数据与输出”区域选择路径、扫描 Frame ID，再点击“加载当前帧”。

### 可选：兼容单文件入口

`--map`、`--source` 和 `--pose` 必须同时提供：

```powershell
.\.venv\Scripts\python.exe .\src\frame_align_6dof.py `
  --map .\data\global_map.pcd `
  --source .\data\frames\1001.pcd `
  --pose .\data\poses\1001.txt `
  --output-dir .\alignment_output\yaml
```

`--pose` 支持上述 TXT，也兼容含 `matrix`、`T_map_lidar` 或 `initial_T_map_lidar` 4×4 矩阵的 YAML。兼容入口会创建 `--output-dir`，并同时预填 YAML/PCD 输出路径。

## 基本操作

1. 选择全局地图、单帧点云目录、初始位姿目录和 YAML 输出目录。
2. 点击“刷新帧列表”，选择或输入 Frame ID。
3. 点击“加载当前帧”；仅切换 Frame ID 不会自动加载。
4. 在 3D 与剖面视图中检查 Reference 和 Adjusted 的重合情况。
5. 使用右侧 6DOF 控件、快捷键或受限 ICP 调整。
6. 根据需要设置剖面角度、平行偏移、半长和厚度，或新增最多两个剖面。
7. 点击“导出当前帧”或按 `G`；成功后标注状态和计数立即刷新。

快捷键：

| 功能 | 负方向 | 正方向 |
|---|---:|---:|
| ΔX | `A` | `D` |
| ΔY | `S` | `W` |
| ΔZ | `E` | `Q` |
| yaw | `3` | `1` |
| pitch | `6` | `4` |
| roll | `9` | `7` |

按住 `Shift` 使用大步长；`R` 重置，`G` 导出。

## 输出

YAML 至少记录：输入绝对路径、`initial_T_map_lidar`、六自由度增量、`T_manual_map`、`corrected_T_map_lidar`、质量指标及可选 PCD 的实际写入状态。可选 PCD 使用完整原始单帧点集合应用一次校正矩阵生成；显示降采样不会影响导出点数。

## 最小验证

以下命令在 Windows PowerShell、项目根目录执行：

```powershell
.\.venv\Scripts\python.exe .\src\frame_align_6dof.py --help
.\.venv\Scripts\python.exe .\src\frame_align_6dof.py --self-test
```

完整测试需要环境中另行安装 `pytest`；`requirements.txt` 当前未声明 pytest：

```powershell
$env:QT_QPA_PLATFORM='offscreen'
.\.venv\Scripts\python.exe -m pytest -q
```

部分真实样本测试会在本机样本不存在时自动跳过。详细验收和排查步骤见 [USER_GUIDE.md](USER_GUIDE.md)。
