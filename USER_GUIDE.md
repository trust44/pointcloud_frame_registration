# 项目名称与作用

## 项目名称

单帧点云与全局点云 6DOF 剖面配准工具。

当前工具的主入口是 `src/frame_align_6dof.py`，实际功能代码位于 `frame_alignment/`。仓库中的 `src/frame_register_manual.py` 是保留未改动的旧版脚本，不是本文推荐入口；`align_tool/` 也未被当前主入口导入。

## 项目解决什么问题

本项目提供一个桌面 GUI，用于将一帧已经位于 `map` 坐标系中的点云，与全局地图点云进行人工 6DOF 微调和剖面对比。使用者可以在 3D 场景及四个 2D 剖面中观察配准效果，调整平移和旋转，必要时执行受限 ICP，然后导出校正结果。

## 适用场景

- 已有一份全局地图点云和若干单帧 PCD，希望逐帧检查或修正配准结果。
- 每帧都有同名初始位姿 TXT 文件，可从 `Tr_velo_to_map` 读取初始 LiDAR 位姿。
- 需要通过 X-Z、Y-Z、`+45°` 和 `-45°` 四个剖面辅助判断点云是否对齐。
- 需要把人工调整量、校正矩阵和质量指标保存为 YAML，并可选导出调整后的 PCD。

## 核心输入、处理流程和输出

| 阶段 | 内容 |
|---|---|
| 核心输入 | 全局地图文件、单帧点云目录、初始位姿目录、Frame ID，以及 YAML/PCD 输出目录 |
| 加载 | 根据 Frame ID 精确读取 `<frame_id>.pcd` 和 `<frame_id>.txt`；全局地图在路径不变时复用缓存 |
| 显示 | 围绕初始 LiDAR 原点 `C0` 裁剪地图 ROI、体素降采样显示，并渲染 3D 场景和四方向剖面 |
| 调整 | 在 `map` 坐标系中调整 ΔX、ΔY、ΔZ、roll、pitch、yaw；旋转围绕初始原点 `C0` |
| 可选优化 | 执行有修正量限制的 point-to-plane ICP |
| 核心输出 | `<实际加载的 PCD stem>.yaml`，以及可选的同名调整后 PCD |

单帧 PCD 的点已经位于 `map` 坐标系。程序不会再次把初始位姿矩阵乘到单帧点上；初始位姿用于确定 LiDAR 原点、旋转中心和校正后的位姿。

## 不在本项目范围内的能力

- 不负责生成全局地图或单帧点云。
- 不负责计算缺失的初始位姿。
- 不提供多帧无人值守批量自动标注；帧浏览只切换 Frame ID，仍需明确点击“加载当前帧”。
- 不提供独立服务端、Web UI 或数据库。
- 仓库未提供 Docker 镜像或容器启动方式。
- `src/qgis_register.py`、`src/las_stats_z.py` 和旧版 `src/frame_register_manual.py` 是其他/历史脚本，不属于本文描述的推荐工作流。

## 本指南核对的信息来源

本指南依据以下仓库文件编写：

- `README.md`
- `requirements.txt`
- `config.example.yaml`
- `src/frame_align_6dof.py`
- `frame_alignment/contracts.py`
- `frame_alignment/core/`
- `frame_alignment/io/`
- `frame_alignment/ui/`
- `tests/`

仓库中未找到 `pyproject.toml`、`package.json`、`environment.yml`、`environment.yaml`、`Dockerfile*`，也未找到独立的 `.bat`、`.cmd`、`.ps1` 或 `.sh` 启动脚本。

# 系统与环境要求

## 操作系统

| 环境 | 状态 | 说明 |
|---|---|---|
| Windows 原生环境 | 已确认 | README 使用 PowerShell 命令；本项目当前在 Windows 环境完成了入口和自检验证 |
| WSL | 需确认 | 当前主入口和 README 没有给出 WSL 支持声明；部署前应检查 `README.md` 与 `src/frame_align_6dof.py`，并验证 Qt/OpenGL 图形显示 |
| Linux | 需确认 | 仓库没有 Linux 安装、桌面依赖或启动说明；应检查 `requirements.txt` 并在目标发行版验证 PySide6、Open3D 和 PyOpenGL |
| Docker | 不提供 | 仓库没有 `Dockerfile*` 或容器入口 |

本文所有可复制命令均以 Windows PowerShell 为准。没有给出未经仓库验证的 WSL、Linux 或 Docker 命令。

## Python 版本要求

仓库没有在 `pyproject.toml`、`setup.py` 或其他元数据中声明正式 Python 版本范围，因此正式支持范围为“需确认”。确认版本时应同时检查：

- `requirements.txt` 中各依赖的 Python 支持范围；
- 目标平台能否安装指定版本的 PySide6、Open3D 和 PyOpenGL；
- `src/frame_align_6dof.py --self-test` 是否通过。

当前项目本机 `.venv` 已验证使用 Python `3.8.10`，但这只是当前环境事实，不等同于仓库的正式版本承诺。

## 外部工具与系统依赖

- Python 及其 `venv`、`pip` 模块。
- 运行 GUI 时需要可用的桌面图形会话；3D 视图由 PyQtGraph OpenGL 和 PyOpenGL 提供。
- PCD/PLY 读取和 PCD 写出使用 Open3D。
- LAS/LAZ 读取使用 laspy。仓库只声明了 `laspy>=2.5`，没有声明 LAZ 解压后端；如 LAZ 读取报缺少 backend，具体后端及版本为“需确认”，应以 `requirements.txt` 和 laspy 的实际错误信息为准。
- 仓库未声明模型文件、外部服务、数据库或运行时代理配置。

# 依赖安装与环境配置

## 推荐方式：Windows 项目内虚拟环境

以下命令均在 Windows PowerShell 中执行。

### 1. 进入项目根目录

```powershell
cd D:\0_code\frame_register_manual
```

### 2. 确认 Python 可用

```powershell
python --version
```

仓库没有正式声明 Python 版本。当前本机已验证版本为 `3.8.10`；其他版本需要依据实际依赖安装结果确认。

### 3. 创建虚拟环境

如果 `.venv` 不存在，在 Windows PowerShell 中执行：

```powershell
python -m venv .venv
```

`.venv/` 已写入 `.gitignore`，不应提交到 Git。

### 4. 安装运行依赖

推荐直接调用项目虚拟环境解释器，不依赖 PowerShell 激活状态：

```powershell
.\.venv\Scripts\python.exe -m pip install -r .\requirements.txt
```

当前工作树实际存在的依赖文件是 `requirements.txt`。`README.md` 仍引用已经不存在的 `requirements-align-tool.txt`；这是当前仓库信息不一致，需由维护者确认是否更新 README。本文安装命令以实际存在的 `requirements.txt` 为准。

仓库声明的依赖如下：

| 包 | 仓库约束 | 用途 |
|---|---:|---|
| NumPy | `numpy>=1.24` | 点云数组和矩阵运算 |
| SciPy | `scipy>=1.10` | 旋转、KD-tree 和距离统计 |
| Open3D | `open3d>=0.19` | PCD/PLY 读写及 ICP |
| PySide6 | `PySide6==6.6.3.1` | GUI 框架 |
| PyQtGraph | `pyqtgraph==0.13.3` | 2D 剖面和 OpenGL 3D 视图 |
| PyOpenGL | `PyOpenGL==3.1.7` | OpenGL 接口 |
| PyYAML | `PyYAML==6.0.1` | 配置和结果 YAML |
| laspy | `laspy>=2.5` | LAS/LAZ 读取 |

### 5. 可选：激活虚拟环境

本文后续命令不要求激活；如希望在当前 PowerShell 会话中激活，可执行：

```powershell
.\.venv\Scripts\Activate.ps1
```

若 PowerShell 执行策略阻止激活，继续使用 `\.venv\Scripts\python.exe` 的直接调用方式即可，不需要修改系统执行策略。

## 测试依赖说明

`README.md` 提供了 `python -m pytest -q`，但 `requirements.txt` 没有声明 `pytest` 及其版本。首次使用者应把 pytest 视为可选测试依赖：

- 是否要纳入正式依赖、采用哪个版本：**需确认**；应查看 `README.md`、`requirements.txt` 和项目维护约定。
- 不安装 pytest 也可以运行内置 `--self-test` 和基于标准库 `unittest` 的最小检查。

如维护者确认可以安装未锁版本的 pytest，可在 Windows 项目虚拟环境中执行：

```powershell
.\.venv\Scripts\python.exe -m pip install pytest
```

## 环境变量、模型文件和数据目录

### 运行时环境变量

仓库没有发现 GUI 正常运行必须设置的环境变量。

测试在无显示器模式下会使用 `QT_QPA_PLATFORM=offscreen`。仅在运行 GUI 自动测试时，在 Windows PowerShell 中设置：

```powershell
$env:QT_QPA_PLATFORM = 'offscreen'
```

不要在需要人工操作真实 GUI 的会话中依赖 offscreen 模式。

### 模型文件

本项目不使用机器学习模型；仓库没有模型路径配置。

### 数据目录

仓库没有附带可直接运行的 PCD/PLY/LAS/LAZ 示例数据。`config.example.yaml` 使用 `data/` 和 `alignment_output/` 作为示例相对路径，使用者需要自行准备对应文件和目录。

## 验证环境配置成功

在 Windows PowerShell、项目根目录执行：

```powershell
.\.venv\Scripts\python.exe .\src\frame_align_6dof.py --help
.\.venv\Scripts\python.exe .\src\frame_align_6dof.py --self-test
```

预期：

- `--help` 显示 `--config`、`--map`、`--source`、`--pose`、`--output-dir` 和 `--self-test`。
- `--self-test` 输出 `Self-test passed` 并以退出码 `0` 结束。

如需额外确认 GUI 依赖可以导入，可在 Windows 项目虚拟环境中执行：

```powershell
.\.venv\Scripts\python.exe -c "import numpy, scipy, open3d, PySide6, pyqtgraph, OpenGL, yaml, laspy; print('runtime imports OK')"
```

## 常见安装失败及排查

| 现象 | 核对与处理 |
|---|---|
| `python` 不是命令 | Python 没有加入 PATH。使用已安装 Python 的完整路径创建 `.venv`；正式支持的 Python 版本需查看 `requirements.txt` 并确认 |
| `ModuleNotFoundError` | 确认使用的是项目解释器 `\.venv\Scripts\python.exe`，然后重新执行依赖安装命令 |
| PySide6/Open3D 无可安装版本 | 仓库未声明正式 Python 范围；核对当前 Python 架构和版本，以及 `requirements.txt` 中的固定/最低版本 |
| `Activate.ps1` 被禁止 | 不必修改执行策略；直接使用 `\.venv\Scripts\python.exe` |
| LAZ 读取提示缺少 backend | 仓库未声明 LAZ 解压后端，属于“需确认”；保留完整错误信息并与维护者确认应增加的 backend 和版本 |
| OpenGL/GUI 初始化失败 | 先运行 `--self-test` 区分核心逻辑和图形环境问题，再核对桌面会话、PySide6、PyQtGraph 与 PyOpenGL 安装 |

# 数据与配置说明

## 推荐目录结构

以下结构与 `config.example.yaml` 和目录模式加载规则一致。仓库本身不附带这些数据文件：

```text
frame_register_manual/
├─ config.yaml
├─ data/
│  ├─ global_map.pcd
│  ├─ frames/
│  │  ├─ <frame_id>.pcd
│  │  └─ ...
│  └─ poses/
│     ├─ <frame_id>.txt
│     └─ ...
└─ alignment_output/
   ├─ yaml/
   └─ pcd/
```

帧目录扫描是非递归的，只读取目录最外层、扩展名不区分大小写的 `.pcd` 文件。

## 输入数据格式

### 全局地图

主 GUI 的全局地图选择器和读取代码支持：

- `.pcd`
- `.ply`
- `.las`
- `.laz`

PCD/PLY 由 Open3D 读取；LAS/LAZ 由 laspy 读取。空点云会被拒绝。

### 单帧点云

推荐目录模式只按以下规则加载：

```text
<frame_cloud_map_path>/<frame_id>.pcd
```

单帧 PCD 必须已经位于 `map` 坐标系。程序不会再应用初始 `T_map_lidar` 来变换这些点。

### Frame ID

- 可包含字母、数字、下划线 `_`、连字符 `-` 和点 `.`。
- 手动输入末尾的 `.pcd` 或 `.txt` 会被去除。
- 空值、空格、目录分隔符、`.`、`..` 和路径穿越形式会被拒绝。
- 输出文件名最终以实际加载的 PCD stem 为准，而不是未经验证的界面文本。

### 初始位姿 TXT

目录模式按以下规则加载：

```text
<initial_pose_path>/<frame_id>.txt
```

TXT 中必须且只能出现一条 `Tr_velo_to_map`，其后恰好有 12 个有限数值，按 3×4 行优先排列。程序补全最后一行为 `[0, 0, 0, 1]`，并验证旋转矩阵正交且行列式为 `+1`。

格式示例：

```text
Tr_velo_to_map: 1 0 0 10 0 1 0 20 0 0 1 30
```

兼容 CLI 的 `--pose` 还可以读取包含 `matrix`、`T_map_lidar` 或 `initial_T_map_lidar` 字段的 4×4 YAML。

## 配置文件

仓库样例为 `config.example.yaml`。推荐复制为 Git 已忽略的 `config.yaml`：

**Windows PowerShell｜项目根目录：**

```powershell
Copy-Item .\config.example.yaml .\config.yaml
```

配置中的相对路径以配置 YAML 所在目录为基准，而不是以当前 PowerShell 目录为基准。

### 路径参数

| 参数 | 推荐目录模式含义 | 是否必须 |
|---|---|---|
| `global_map_path` | 全局地图文件 | 加载帧时必须 |
| `frame_cloud_map_path` | 包含 `<frame_id>.pcd` 的目录 | 加载帧时必须 |
| `initial_pose_path` | 包含 `<frame_id>.txt` 的目录 | 加载帧时必须 |
| `frame_id` | GUI 启动后的初始 Frame ID 文本 | 可选，可手动选择或输入 |
| `output_path_yaml` | YAML 输出目录 | 导出时必须，且目录必须预先存在 |
| `output_path_pcd` | 可选 PCD 输出目录 | 勾选“导出调整后 PCD”时必须，且目录必须预先存在 |

### 显示参数

| 参数 | 样例/默认值 | 含义 |
|---|---:|---|
| `display.map_roi_radius_m` | `35.0` | 以初始 LiDAR 原点 `C0` 为中心的全局地图 3D ROI 半径 |
| `display.display_voxel_m` | `0.05` | 3D 和剖面显示用体素尺寸；不改变导出使用的原始单帧点集合 |
| `display.slice_half_length_m` | `20.0` | 四个剖面从当前 LiDAR 原点向两侧的半长 |
| `display.slice_thickness_m` | `0.20` | 剖面带总厚度 |

### 交互步长

| 参数 | 样例/默认值 | 含义 |
|---|---:|---|
| `interaction.translation_step_m` | `0.01` | 普通平移步长 |
| `interaction.translation_large_step_m` | `0.10` | 按住 Shift 时的平移步长 |
| `interaction.rotation_step_deg` | `0.05` | 普通旋转步长 |
| `interaction.rotation_large_step_deg` | `0.50` | 按住 Shift 时的旋转步长 |

### 最小配置模板

下面内容来自仓库 `config.example.yaml`。它只有在相应数据文件和目录存在时才能运行：

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

创建样例输出目录：

**Windows PowerShell｜项目根目录：**

```powershell
New-Item -ItemType Directory -Force .\alignment_output\yaml, .\alignment_output\pcd
```

## 输出文件

### 命名规则

若实际加载文件为：

```text
data/frames/1781158324500077000.pcd
```

则输出为：

```text
alignment_output/yaml/1781158324500077000.yaml
alignment_output/pcd/1781158324500077000.pcd   # 仅在勾选 PCD 导出且写入成功时存在
```

同名输出已存在时，GUI 会要求明确确认覆盖。YAML 和可选 PCD 先写临时文件，再进行带回滚的提交。

### YAML 字段

| 字段 | 含义 |
|---|---|
| `frame_id` | 实际加载 PCD 的 stem |
| `input.global_map_path` | 全局地图绝对路径 |
| `input.frame_cloud_map_path` | 单帧点云绝对路径 |
| `input.initial_pose_path` | 初始位姿绝对路径 |
| `initial_T_map_lidar` | 初始 4×4 位姿 |
| `manual_delta_about_lidar_origin` | map 坐标系中的平移/旋转增量及初始旋转中心 |
| `T_manual_map` | 围绕初始 LiDAR 原点构造的人工调整矩阵 |
| `corrected_T_map_lidar` | `T_manual_map @ initial_T_map_lidar` |
| `quality` | 最近一次有效质量结果；未执行 ICP 或人工调整使质量失效时，相应值可能为 `null` |
| `output.adjusted_pcd_written` | 调整后 PCD 是否实际写入 |
| `output.adjusted_pcd_path` | 实际写入的 PCD 绝对路径；未写入时为 `null` |

默认质量字段包括：

- `nn_residual_median_m`
- `nn_residual_p95_m`
- `icp_rmse_m`
- `icp_fitness`

### 调整后 PCD

可选 PCD 是对完整单帧点集合应用一次人工/ICP 调整矩阵后的结果，不使用仅供显示的体素点集，也不会再次应用初始位姿。如果输入单帧点云包含颜色，默认 Open3D 写出路径会携带调整前的颜色。

# 使用方法

## 推荐方式：配置文件 + 目录式帧浏览

### 步骤 1：准备数据

确保以下路径真实存在，并且 Frame ID 的 PCD 和 TXT 精确同名：

```text
data/global_map.pcd
data/frames/1781158324500077000.pcd
data/poses/1781158324500077000.txt
```

以上相对路径和 Frame ID 来自 `config.example.yaml`；仓库不附带这些数据。

### 步骤 2：创建和编辑配置

**Windows PowerShell｜项目根目录：**

```powershell
Copy-Item .\config.example.yaml .\config.yaml
New-Item -ItemType Directory -Force .\alignment_output\yaml, .\alignment_output\pcd
```

用文本编辑器修改 `config.yaml`，使其中地图、帧目录、位姿目录和输出目录指向真实文件。

### 步骤 3：启动 GUI

**Windows PowerShell｜项目虚拟环境解释器：**

```powershell
.\.venv\Scripts\python.exe .\src\frame_align_6dof.py --config .\config.yaml
```

预期结果：打开标题为“6DoF 单帧点云 / 全局地图剖面配准”的窗口。左侧是 3D 视图和四个剖面，右侧依次是 6DOF 控制、姿态矩阵、数据与输出面板。

### 步骤 4：扫描或输入 Frame ID

1. 在右侧确认全局地图、单帧点云目录和初始位姿目录。
2. 点击“刷新帧列表”。
3. 在 Frame ID 下拉框浏览，使用“上一帧/下一帧”，或直接输入 Frame ID。
4. 检查“已标注/未标注”和 `已标注数/总数（已标注/总量）`。

切换或输入 Frame ID 不会加载点云，这是设计行为。

### 步骤 5：加载当前帧

点击“加载当前帧”。程序会精确查找：

```text
<单帧点云目录>/<frame_id>.pcd
<初始位姿目录>/<frame_id>.txt
```

加载成功后：

- 状态栏显示 Frame ID、地图/单帧/位姿路径、原始点数、`C0` 和加载耗时；
- 3D 相机在地图 ROI 非空时对准初始 LiDAR 原点；
- 右侧矩阵区显示 `C0`、当前原点、人工矩阵、校正矩阵和质量值；
- 左侧四个剖面显示 Reference Map 和 Adjusted Frame。

加载失败时会弹出错误信息，并保留之前成功加载的帧和调整状态。

### 步骤 6：人工调整

可以使用右侧按钮、数值框或键盘：

| 操作 | 负向键 | 正向键 | 坐标系 |
|---|---|---|---|
| ΔX | `A` | `D` | map |
| ΔY | `S` | `W` | map |
| ΔZ | `E` | `Q` | map |
| roll | `9` | `7` | map 轴，绕初始 `C0` |
| pitch | `6` | `4` | map 轴，绕初始 `C0` |
| yaw | `1` | `3` | map 轴，绕初始 `C0` |

按住 Shift 使用配置中的大步长。其他操作：

- `R`：重置当前帧人工调整。
- “撤销”/“重做”：恢复调整历史。
- 修改“剖面半长”和“剖面厚度”：立即刷新四个剖面。
- 人工调整、重置、撤销或重做后，当前质量指标会失效并恢复为空值。

3D 视角不会因人工微调、剖面参数变化、撤销或重做而自动重置。

### 步骤 7：可选 ICP

点击“ICP”执行受限 point-to-plane ICP。当前代码要求 ICP ROI 中源点和目标点各不少于 10 个；如果建议修正超过 `0.5 m` 或 `3°`，会拒绝该结果并显示警告。

ICP 成功后，矩阵区和 `quality` 会显示最近邻残差、ICP RMSE 和 fitness。随后如果再做人工调整，质量值会被清空，因为它们不再对应当前姿态。

### 步骤 8：导出

1. 确认 YAML 输出目录已经存在。
2. 如需 PCD，勾选“导出调整后 PCD”，并确认 PCD 输出目录已经存在。
3. 点击“导出当前帧”或按 `G`。
4. 如目标文件已存在，确认是否覆盖。

预期结果：

- YAML 成功后弹出完成信息，状态栏显示 `Exported: <yaml path>`；
- 当前 Frame ID 的状态立即变为“已标注”，计数同步更新；
- 如果 PCD 写入失败但 YAML 成功，会显示警告，YAML 中 `adjusted_pcd_written` 为 `false`。

## 可选方式一：空白 GUI 启动

不提供配置也可以打开空白 GUI，然后通过右侧面板选择路径。

**Windows PowerShell｜项目根目录：**

```powershell
.\.venv\Scripts\python.exe .\src\frame_align_6dof.py
```

预期结果：GUI 打开，但不会自动加载任何帧。

## 可选方式二：兼容单文件 CLI

兼容模式要求 `--map`、`--source` 和 `--pose` 三个参数同时提供。以下示例路径与 `config.example.yaml` 一致，执行前必须准备对应文件：

**Windows PowerShell｜项目根目录：**

```powershell
$frameId = '1781158324500077000'
.\.venv\Scripts\python.exe .\src\frame_align_6dof.py `
  --map .\data\global_map.pcd `
  --source ".\data\frames\$frameId.pcd" `
  --pose ".\data\poses\$frameId.txt" `
  --output-dir .\alignment_output
```

兼容模式会创建 `--output-dir`。省略该参数时使用当前目录下的 `alignment_output`。它会预填 YAML 和 PCD 输出目录，但是否写出 PCD仍由 GUI 中的复选框决定。

只提供上述三个参数中的一部分会直接退出，并提示三者必须同时提供。

## 查看日志、结果和可视化

项目没有持久化日志文件配置。运行信息分布在：

- GUI 状态栏：加载路径、点数、`C0`、耗时和最近导出 YAML 路径；
- 消息框：扫描、加载、ICP 和导出的错误、警告或完成信息；
- 右侧矩阵区：当前变换矩阵和质量指标；
- 左侧 3D/剖面：配准的可视化结果；
- YAML/PCD 输出目录：持久化结果。

# 常见问题与排查

## 依赖缺失

### `ModuleNotFoundError`

先确认没有误用系统 Python：

**Windows PowerShell｜项目根目录：**

```powershell
.\.venv\Scripts\python.exe -c "import sys; print(sys.executable)"
.\.venv\Scripts\python.exe -m pip install -r .\requirements.txt
```

输出解释器路径应位于当前项目的 `.venv\Scripts\python.exe`。

### `No module named pytest`

pytest 是 README 使用但依赖文件未声明的可选测试依赖。版本策略为“需确认”；检查 `README.md`、`requirements.txt` 和维护约定。内置 `--self-test` 不依赖 pytest。

## 环境变量或路径错误

### `FileNotFoundError: config.yaml`

- 从项目根目录执行命令，或为 `--config` 提供正确路径。
- 首次使用可复制 `config.example.yaml` 为 `config.yaml`。
- 配置中的相对路径以配置文件所在目录为基准。

### 选定目录后仍不能导出

- YAML 输出目录必须已经存在。
- 勾选 PCD 导出后，PCD 输出目录也必须已经存在。
- 界面只有在成功加载帧且必需输出目录有效时才启用导出按钮。

### Frame ID 无法加载

- 确认 `<frame_id>.pcd` 和 `<frame_id>.txt` 精确同名。
- 扫描不递归；PCD 必须直接位于单帧目录最外层。
- Frame ID 不能包含空格、斜杠、反斜杠或路径穿越片段。
- 输入 `.pcd`/`.txt` 后缀是允许的，程序会去掉后缀。

## 输入数据不匹配

### `rotation is not orthonormal` 或 `rotation determinant is not +1`

初始位姿旋转块不是合法刚体旋转。检查 `<frame_id>.txt` 中唯一的 `Tr_velo_to_map`，确认恰好有 12 个有限数值，且 3×3 旋转矩阵正交、行列式为 `+1`。

不要通过放宽校验掩盖错误位姿；应修正数据源。

### 点云为空或格式不支持

- 全局地图支持 PCD、PLY、LAS、LAZ。
- 目录模式单帧必须为 PCD。
- Open3D 读取到空 PCD/PLY 会报错。
- 不支持的扩展名会提示 `Supported point clouds: PCD, PLY, LAS, LAZ`。

### 单帧显示位置明显错误

确认单帧 PCD 的坐标已经是 `map` 坐标。程序设计上不会再次应用初始位姿。如果输入仍是 LiDAR 局部坐标，本工具不会自动把它转换到地图坐标。

## 程序无输出、运行卡住或结果异常

### 点击加载后界面暂时无响应

点云读取、地图 ROI 和体素处理完成后，状态栏才会更新。先核对地图文件大小、格式和路径；再用 `--self-test` 判断是否为核心环境问题。仓库没有后台加载进度条或持久化日志文件。

### 3D 视图没有地图点

- 检查初始位姿的 `C0 = T_map_lidar[:3, 3]` 是否落在地图附近。
- 检查 `display.map_roi_radius_m` 是否过小。
- 地图 ROI 为空时，程序不会自动把相机移到 `C0`。
- 查看状态栏中的原始地图点数与 `C0`，并核对输入坐标系。

### 某个剖面显示“当前剖面无点”

- 适当增大“剖面半长”或“剖面厚度”。
- 检查当前 LiDAR 原点和点云坐标是否一致。
- 四个方向分别是 X-Z/0°、Y-Z/90°、`+45°` 和 `-45°`，某一方向无点不代表其他方向也无点。

### ICP 报 ROI 点不足

受限 ICP 要求源/目标 ROI 各至少 10 个点。先确认当前配准大致正确、ROI 覆盖了有效重叠区域，再重试。

### ICP 修正被拒绝

当前实现拒绝超过 `0.5 m` 或 `3°` 的 ICP 建议。先人工调整到较接近位置，或检查输入坐标系和初始位姿。

### YAML 已导出，但 PCD 导出失败

这是允许的部分成功结果。查看警告消息，并打开 YAML：

```yaml
output:
  adjusted_pcd_written: false
  adjusted_pcd_path: null
```

然后检查 PCD 输出目录、磁盘空间和 Open3D 写入错误。不要仅凭 YAML 文件存在就假设 PCD 已写入。

### 标注数量不正确

- “已标注”只根据 `<output_path_yaml>/<frame_id>.yaml` 的精确文件是否存在判断。
- 修改 YAML 输出目录、Frame ID 或点击“刷新帧列表”会重新统计。
- 总量只包括成功扫描到的单帧目录最外层 PCD。

# 验收与自检

## 判断项目已正确安装

满足以下条件可认为基础安装可用：

1. 项目虚拟环境解释器能够启动。
2. `--help` 能列出真实 CLI 参数。
3. `--self-test` 输出 `Self-test passed`。
4. GUI 能打开，且没有 PySide6、PyQtGraph、PyOpenGL 或 Open3D 导入错误。

推荐最小命令：

**Windows PowerShell｜项目根目录：**

```powershell
.\.venv\Scripts\python.exe .\src\frame_align_6dof.py --help
.\.venv\Scripts\python.exe .\src\frame_align_6dof.py --self-test
.\.venv\Scripts\python.exe -m unittest tests.test_cli_entry tests.test_frame_requests
```

最后一条使用 Python 标准库 `unittest`，不要求安装 pytest。

## 判断一次任务已正确完成

逐帧操作完成后检查：

- 状态栏显示的 Frame ID 和实际 PCD/TXT 一致。
- 3D 视图和至少所需方向的剖面有数据，Reference 与 Adjusted 的位置符合预期。
- 右侧 `T_corrected_map_lidar` 和 6DOF 增量符合人工调整。
- YAML 输出文件名与实际加载 PCD stem 一致。
- YAML 中 `input` 路径、`manual_delta_about_lidar_origin`、`T_manual_map` 和 `corrected_T_map_lidar` 完整。
- 如要求 PCD，确认 `output.adjusted_pcd_written: true` 且 `adjusted_pcd_path` 对应文件真实存在。
- GUI 中当前帧显示“已标注”，计数同步增加。

## 可选 pytest 验证

只有在项目环境已安装 pytest 时，才执行 README 中的测试方式：

**Windows PowerShell｜项目根目录：**

```powershell
$env:QT_QPA_PLATFORM = 'offscreen'
.\.venv\Scripts\python.exe -m pytest -q
```

真实样本测试 `tests/test_real_sample_rendering.py` 会检查以下固定路径：

```text
D:/1_data/map_seg/global_map/jinhua/colored_map_global_voxel_blue_filled.pcd
D:/1_data/map_seg/global_map/jinhua/velodyne_map/1781158324500077000.pcd
```

文件存在时测试真实 3D ROI、四剖面（含 X-Z）和相机中心；文件不存在时该测试自动跳过。以上路径是测试代码中的环境路径，不是项目运行的强制数据位置。

## 仍需项目维护者确认的信息

- 正式支持的 Python 版本范围。
- WSL 和 Linux 的正式支持状态及系统级图形依赖。
- pytest 是否应加入依赖文件，以及应锁定的 pytest 版本。
- `README.md` 中旧依赖文件名 `requirements-align-tool.txt` 是否应改为当前的 `requirements.txt`。
- LAZ 解压 backend 的选型和版本要求。
- 是否需要发布脱离源码目录的安装包或启动脚本；当前仓库只确认源码入口 `src/frame_align_6dof.py`。
