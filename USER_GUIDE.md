# 项目名称与作用

## 项目名称

单帧点云与全局点云 6DOF 剖面配准工具。

项目的推荐入口是 `src/frame_align_6dof.py`，核心功能代码位于 `frame_alignment/`。`src/frame_register_manual.py` 是保留的历史脚本，不属于本指南的推荐工作流。

## 项目解决什么问题

本项目提供一个桌面 GUI，用于逐帧检查和修正单帧点云与全局地图的配准结果。单帧 PCD 与全局地图需要已经处于相同的 `map` 坐标范围；程序围绕初始 LiDAR 原点显示局部地图、单帧点云和多个垂直剖面，允许人工调整 6DOF，也可运行受限 ICP 辅助优化。

一次典型任务的流程是：

```text
选择全局地图和帧目录
        ↓
按 Frame ID 读取 PCD 与初始位姿
        ↓
显示 3D ROI、LiDAR 坐标轴和 4～6 个剖面
        ↓
人工 6DOF 调整 / 可选受限 ICP
        ↓
导出校正 YAML / 可选校正后 PCD
```

## 适用场景

- 已有全局地图和位于同一 `map` 坐标范围的逐帧 PCD，需要人工复核配准质量。
- 每帧都有同名初始位姿 TXT，可从 `Tr_velo_to_map` 得到 LiDAR 在地图中的初始位置和方向。
- 需要通过与 LiDAR 航向对齐的 XZ、YZ 和斜向剖面检查立面、路面、杆状物等结构。
- 需要记录人工调整量、校正矩阵、质量指标和标注完成状态。
- 全局地图/单帧点云已经整体平移，需要通过 `map_anchor.yaml` 把仍在 world 范围的位姿转换到点云范围。

## 核心输入、处理和输出

| 类别 | 内容 |
|---|---|
| 核心输入 | 全局地图文件、单帧 PCD 目录、逐帧初始位姿目录、Frame ID、输出目录 |
| 点云处理 | 以初始 LiDAR 原点裁剪地图 ROI，显示体素降采样，保持完整单帧点用于最终导出 |
| 人工调整 | 在 `map` 坐标系调整平移和旋转，旋转中心固定为初始 LiDAR 原点 |
| 剖面 | 默认 4 个、最多 6 个；水平方向随校正后 LiDAR yaw，高度使用 `map Z` |
| 自动辅助 | 带修正幅度限制的 point-to-plane ICP |
| 核心输出 | `<实际加载的 PCD stem>.yaml` 和可选的同名校正后 `.pcd` |

## 不在本项目范围内的能力

- 不生成全局地图、单帧点云或缺失的初始位姿。
- 不把原始 LiDAR 坐标系点云自动转换到 `map` 坐标系；输入单帧必须已完成该转换。
- 不提供 Scan Context、回环检测或全局粗配准。
- 不提供无人值守批处理；切换 Frame ID 后仍需明确点击“加载当前帧”。
- 不提供 Web 服务、数据库或协作标注平台。
- 仓库未提供 Docker 镜像、WSL/Linux 启动脚本或安装包。

## 本指南核对的仓库信息

本指南已核对：

- `README.md`、`requirements.txt`、`config.yaml`、`.gitignore`
- `src/frame_align_6dof.py`
- `frame_alignment/contracts.py`
- `frame_alignment/app/`、`frame_alignment/core/`、`frame_alignment/io/`、`frame_alignment/ui/`
- `tests/` 中的 CLI、加载、坐标、剖面、界面、导出和真实样本测试

仓库当前没有 `pyproject.toml`、`package.json`、`environment.yml`、`Dockerfile*` 或独立的 `.bat`、`.cmd`、`.ps1`、`.sh` 启动脚本。

# 系统与环境要求

## 操作系统

| 环境 | 状态 | 说明 |
|---|---|---|
| Windows + PowerShell | 已确认 | 仓库和现有虚拟环境采用 Windows 路径与命令 |
| WSL/Linux | 需确认 | 仓库没有对应安装、图形转发或启动说明 |
| Docker | 未提供 | 仓库没有 Dockerfile 或容器启动入口 |

本文命令均在 **Windows PowerShell** 中执行；除非特别说明，当前目录都是项目根目录。

## Python 版本

仓库没有声明正式 Python 版本范围，因此正式支持版本为“需确认”。选择 Python 时需要保证 `requirements.txt` 中的 PySide6、Open3D、NumPy、SciPy 等依赖都能安装。

可执行以下命令查看实际解释器版本：

```powershell
.\.venv\Scripts\python.exe --version
```

不要把某台机器现有 `.venv` 的版本视为项目长期兼容承诺；更换 Python 后应重新安装依赖并运行本指南的最小验证。

## 图形和硬件要求

- GUI 需要可用的 Windows 桌面会话。
- 3D 视图依赖 PyQtGraph OpenGL 和 PyOpenGL，因此显卡驱动必须能提供可用的 OpenGL 环境。
- 仓库没有声明 CUDA、专用 GPU 或最低显存要求；ICP 由 Open3D 执行，当前代码未提供 CUDA 配置项。
- 大型地图会占用较多内存和加载时间。界面只显示 ROI 和体素降采样结果，但加载器仍需读取完整全局地图。

## 外部工具与系统依赖

- Python、`venv` 和 `pip`。
- Open3D：读取 PCD/PLY、写出 PCD、执行 ICP。
- laspy：读取 LAS/LAZ。仓库未声明 LAZ 解压 backend；若 laspy 报 backend 缺失，所需包和版本需根据实际错误确认。
- PySide6、PyQtGraph、PyOpenGL：GUI、2D 剖面和 3D 场景。
- 仓库不依赖模型文件、数据库、网络服务或必须设置的代理。

# 依赖安装与环境配置

## 推荐方式：项目内虚拟环境

### 1. 进入项目目录

```powershell
cd D:\0_code\frame_register_manual
```

如果项目位于其他位置，请替换为实际路径。

### 2. 创建虚拟环境

当 `.venv` 不存在或已经失效时：

```powershell
python -m venv .venv
```

`.gitignore` 已排除 `.venv/`，不要把虚拟环境提交到 Git。

### 3. 安装运行依赖

```powershell
.\.venv\Scripts\python.exe -m pip install --upgrade pip
.\.venv\Scripts\python.exe -m pip install -r .\requirements.txt
```

仓库当前声明：

| 包 | 版本约束 | 用途 |
|---|---:|---|
| NumPy | `numpy>=1.24` | 点云数组和矩阵计算 |
| SciPy | `scipy>=1.10` | 欧拉角/旋转、KD-tree 和统计 |
| Open3D | `open3d>=0.19` | 点云读写和 point-to-plane ICP |
| PySide6 | `PySide6==6.6.3.1` | Qt GUI |
| PyQtGraph | `pyqtgraph==0.13.3` | 2D 和 OpenGL 可视化 |
| PyOpenGL | `PyOpenGL==3.1.7` | OpenGL 接口 |
| PyYAML | `PyYAML==6.0.1` | 配置和结果 YAML |
| laspy | `laspy>=2.5` | LAS/LAZ 读取 |

`pytest` 未列入 `requirements.txt`，只在需要运行 pytest 测试时另行安装；版本策略需确认。

### 4. 验证依赖导入

```powershell
.\.venv\Scripts\python.exe -c "import numpy, scipy, open3d, PySide6, pyqtgraph, OpenGL, yaml, laspy; print('runtime imports OK')"
```

预期输出：

```text
runtime imports OK
```

### 5. 验证入口

```powershell
.\.venv\Scripts\python.exe .\src\frame_align_6dof.py --help
.\.venv\Scripts\python.exe .\src\frame_align_6dof.py --self-test
```

`--help` 应列出 `--config`、`--map`、`--source`、`--pose`、`--output-dir`、`--self-test`；自检应输出 `Self-test passed`。

## 环境变量和代理

正常启动 GUI 不要求环境变量、模型目录或网络代理。

自动化 GUI 测试可使用 Qt 无显示模式：

```powershell
$env:QT_QPA_PLATFORM='offscreen'
```

不要在需要人工交互和检查真实 3D 渲染时使用 `offscreen`。

## 常见安装失败

| 现象 | 排查方式 |
|---|---|
| `python` 不是命令 | 使用已安装 Python 的完整路径，或正确配置 PATH |
| `ModuleNotFoundError` | 确认命令使用 `\.venv\Scripts\python.exe`，再重新安装 `requirements.txt` |
| PySide6/Open3D 找不到匹配版本 | 核对 Python 版本、64 位架构和依赖支持范围；仓库正式 Python 范围需确认 |
| `Activate.ps1` 被执行策略阻止 | 不需要激活，直接使用项目解释器完整路径即可 |
| LAZ 读取提示缺少 backend | 仓库没有声明 LAZ backend；保留完整错误，按 laspy 提示确认需要安装的 backend |
| OpenGL 初始化失败 | 更新显卡驱动，确认在真实桌面会话中运行，并先执行 `--self-test` 区分核心环境与图形环境问题 |

# 数据与配置说明

## 推荐目录结构

仓库不附带可运行的点云样本。使用者需要准备类似结构：

```text
frame_register_manual/
├─ config.yaml
├─ data/
│  ├─ global_map.pcd
│  ├─ map_anchor.yaml             # 可选，必须与全局地图同目录
│  ├─ frames/
│  │  ├─ 1001.pcd
│  │  ├─ 1002.pcd
│  │  └─ ...
│  └─ poses/
│     ├─ 1001.txt
│     ├─ 1002.txt
│     └─ ...
└─ alignment_output/
   ├─ yaml/
   └─ pcd/
```

单帧目录扫描是非递归的，只扫描目录最外层扩展名为 `.pcd`（大小写不敏感）的文件。

## 全局地图格式

全局地图支持：

- `.pcd`
- `.ply`
- `.las`
- `.laz`

PCD/PLY 由 Open3D 读取；LAS/LAZ 由 laspy 读取。点必须能转换为 `N×3` XYZ 数据，空点云会被拒绝。颜色不是必需输入；3D 视图使用固定颜色区分全局地图和单帧，单帧自带颜色可传递到可选输出 PCD。

## 单帧点云格式和坐标要求

目录模式固定读取：

```text
<frame_cloud_map_path>/<frame_id>.pcd
```

关键约定：

1. 单帧点坐标已经是 `P_map`，不是原始 `P_lidar`。
2. 单帧点云与全局地图必须位于相同坐标范围。
3. 程序不会把 `initial_T_map_lidar` 再乘到单帧点上。
4. 校正时只应用人工/ICP 得到的 `T_manual_map`：

   ```text
   P_map_corrected = T_manual_map @ P_map
   ```

如果输入仍是原始 LiDAR 坐标系点云，必须先在本项目之外转换到 `map` 坐标系。

## Frame ID

Frame ID：

- 可包含字母、数字、下划线 `_`、连字符 `-` 和点 `.`；
- 输入末尾 `.pcd` 或 `.txt` 时会自动去掉扩展名；
- 不能为空，不能是 `.`、`..`，不能包含 `/`、`\`、空格或其他路径字符；
- 最终输出文件名取实际加载 PCD 的 stem，而不是未验证的输入文本。

## 初始位姿 TXT

目录模式固定读取：

```text
<initial_pose_path>/<frame_id>.txt
```

文件中必须且只能出现一条 `Tr_velo_to_map`，后面恰好有 12 个有限数值，按行组成 3×4 矩阵：

```text
Tr_velo_to_map: r00 r01 r02 tx r10 r11 r12 ty r20 r21 r22 tz
```

最小示例：

```text
Tr_velo_to_map: 1 0 0 10 0 1 0 20 0 0 1 30
```

程序补齐最后一行 `[0, 0, 0, 1]`，并检查：

- 矩阵为 `4×4`；
- 所有值有限；
- 旋转块正交；
- 旋转行列式为 `+1`；
- 最后一行为 `[0, 0, 0, 1]`。

兼容单文件入口的 `--pose` 也支持 YAML；根内容可以直接是 4×4 数组，或通过 `matrix`、`T_map_lidar`、`initial_T_map_lidar` 字段提供矩阵。

## 可选 map_anchor.yaml

当全局地图和单帧 PCD 已经从 world 坐标整体平移，而位姿 TXT 仍保留原 world 平移量时，在**全局地图文件所在目录**创建固定文件名：

```text
map_anchor.yaml
```

内容必须是包含三个有限数值的 YAML：

```yaml
map_translation_offset_xyz: [50000.0, 10000.0, 0.0]
```

加载器会自动查找：

```text
<global_map_path 的父目录>/map_anchor.yaml
```

并执行：

```text
pose_map[:3, 3] = pose_world[:3, 3] - map_translation_offset_xyz
```

注意：

- 无需在 `config.yaml` 或 GUI 中单独填写 map anchor 路径；
- 文件不存在时，位姿原样使用；
- 它只调整初始位姿的平移，不修改旋转，也不变换全局地图或单帧点；
- PCD 之间仍必须已经处于相同坐标范围；
- 文件存在但缺少字段、长度不是 3 或包含非有限值时，加载会失败并显示文件路径。

## config.yaml

仓库根目录的真实配置样例是 `config.yaml`。配置中的相对路径以该 YAML 文件所在目录为基准，而不是以 PowerShell 当前目录为基准。

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

### 路径字段

| 字段 | 含义 |
|---|---|
| `global_map_path` | 全局地图文件 |
| `frame_cloud_map_path` | 目录模式下为包含 `<frame_id>.pcd` 的目录 |
| `initial_pose_path` | 目录模式下为包含 `<frame_id>.txt` 的目录 |
| `frame_id` | GUI 启动时预填的 Frame ID，可留空后扫描/输入 |
| `output_path_yaml` | YAML 输出目录，导出前必须存在 |
| `output_path_pcd` | 可选 PCD 输出目录，勾选导出 PCD 时必须存在 |

### 显示字段

| 字段 | 样例值 | 行为 |
|---|---:|---|
| `display.map_roi_radius_m` | `35.0` | 以初始 LiDAR 原点 `C0` 为中心裁剪球形全局地图 ROI |
| `display.display_voxel_m` | `0.05` | 3D/剖面显示降采样尺寸，不影响完整点云导出 |
| `display.slice_half_length_m` | `20.0` | 剖面水平半长；GUI 允许 `10～35 m`，非法配置回退到 `20 m` |
| `display.slice_thickness_m` | `0.20` | 剖面带的总厚度 |

### 交互字段

| 字段 | 样例值 | 行为 |
|---|---:|---|
| `interaction.translation_step_m` | `0.01` | 普通平移步长 |
| `interaction.translation_large_step_m` | `0.10` | 按住 Shift 时的平移步长 |
| `interaction.rotation_step_deg` | `0.05` | 普通旋转步长 |
| `interaction.rotation_large_step_deg` | `0.50` | 按住 Shift 时的旋转步长 |

## 输出目录和文件

先创建输出目录：

```powershell
New-Item -ItemType Directory -Force .\alignment_output\yaml, .\alignment_output\pcd
```

若加载的是：

```text
data/frames/1001.pcd
```

则输出为：

```text
alignment_output/yaml/1001.yaml
alignment_output/pcd/1001.pcd   # 仅勾选且写入成功时存在
```

同名文件存在时 GUI 会要求确认覆盖。YAML 和可选 PCD 先写临时文件，再整体提交；提交中途失败时会尝试恢复旧文件。若 PCD 写出失败但 YAML 可以写出，YAML 仍会生成，并把 PCD 状态记录为未写入，同时 GUI 给出警告。

### YAML 字段

| 字段 | 含义 |
|---|---|
| `frame_id` | 实际加载 PCD 的 stem |
| `input.global_map_path` | 全局地图绝对路径 |
| `input.frame_cloud_map_path` | 单帧 PCD 绝对路径 |
| `input.initial_pose_path` | 初始位姿文件绝对路径 |
| `initial_T_map_lidar` | 加载并经过可选 map anchor 修正的初始 4×4 位姿 |
| `manual_delta_about_lidar_origin` | 平移/旋转坐标系、旋转中心和六自由度增量 |
| `T_manual_map` | 围绕初始 LiDAR 原点构造的校正矩阵 |
| `corrected_T_map_lidar` | `T_manual_map @ initial_T_map_lidar` |
| `quality` | 最近邻残差和 ICP 指标；无有效结果时可能为 `null` |
| `output.adjusted_pcd_written` | 可选 PCD 是否实际写入 |
| `output.adjusted_pcd_path` | 写入成功时的绝对路径，否则为 `null` |

质量字段包括：

- `nn_residual_median_m`
- `nn_residual_p95_m`
- `icp_rmse_m`
- `icp_fitness`

# 使用方法

## 推荐方式：配置文件 + 目录式帧浏览

### 第一步：准备数据

确认同一 Frame ID 同时存在：

```text
<单帧目录>/<frame_id>.pcd
<位姿目录>/<frame_id>.txt
```

确认全局地图与单帧点云的 XYZ 范围一致；如果 PCD 已平移而位姿未平移，按前述规则准备 `map_anchor.yaml`。

### 第二步：编辑配置和创建输出目录

编辑 `config.yaml` 中的路径和显示参数，然后执行：

```powershell
cd D:\0_code\frame_register_manual
New-Item -ItemType Directory -Force .\alignment_output\yaml, .\alignment_output\pcd
```

路径可使用绝对路径，也可使用相对 `config.yaml` 的路径。

### 第三步：启动 GUI

```powershell
.\.venv\Scripts\python.exe .\src\frame_align_6dof.py --config .\config.yaml
```

预期看到：左侧上方为 3D 场景，左侧下方默认四个剖面；右侧滚动栏依次为 6DOF 控制、剖面设置、姿态矩阵和数据与输出。

### 第四步：扫描和加载帧

1. 核对全局地图、单帧目录、位姿目录和输出目录。
2. 点击“刷新帧列表”。
3. 从 Frame ID 下拉框选择，或手动输入 Frame ID。
4. 点击“加载当前帧”。

切换、输入或扫描 Frame ID 都不会自动读取点云。只有点击“加载当前帧”才会替换当前帧。

加载成功后：

- 状态栏显示 Frame ID、输入文件、点数、位姿路径、初始原点 `C0` 和耗时；
- 3D 相机在地图 ROI 非空时自动对准初始 LiDAR 原点；
- 后续人工调整、剖面修改、撤销/重做不会强制重置用户相机视角；
- 全局地图路径不变时，后续帧复用已经读取的地图；
- 加载失败时弹出错误，上一帧的有效状态继续保留。

### 第五步：理解 3D 和剖面显示

3D 场景包含：

- Reference Map：全局地图在 `C0` 周围的 ROI；
- Adjusted Frame：当前校正矩阵作用后的单帧；
- LiDAR Origin：当前 LiDAR 原点；
- 红、绿、蓝短轴：校正后 LiDAR 的 X、Y、Z 方向；
- 彩色矩形：各剖面在 3D 中的平面轮廓。

默认剖面布局：

| 位置 | 剖面 | 是否可改位置 | 是否可删除 |
|---|---|---:|---:|
| `(0,0)` | XZ / `0°` | 否 | 否 |
| `(1,0)` | YZ / `90°` | 否 | 否 |
| `(0,1)` | `+45°` | 是 | 否 |
| `(1,1)` | `-45°` | 是 | 否 |

剖面水平方向以校正后 LiDAR yaw 为基准，高度始终沿 `map Z`。因此 yaw 改变后剖面会跟随红/绿水平轴转动，pitch/roll 不会把剖面高度轴倾斜。

2D 剖面横轴是沿剖面方向、相对当前 LiDAR 原点的距离；纵轴是相对剖面中心的 `map Z` 高度。灰蓝色为全局地图，黄色为校正后单帧。

### 第六步：调整剖面

右侧“剖面设置”中可选择可编辑剖面：

1. **与 XZ 夹角**：范围 `-180°～180°`；正方向从 LiDAR 水平 X 轴转向水平 Y 轴。
2. **平行偏移**：选择 XZ 或 YZ，并输入有符号距离。

符号约定：

- 平行 XZ 的正距离沿 LiDAR yaw 相关的 `+Y`；
- 平行 YZ 的正距离沿 LiDAR yaw 相关的 `-X`。

点击“新增剖面”最多增加两个：

- 第一个位于 `(0,2)`，默认 `+90°`；
- 第二个位于 `(1,2)`，默认 `-90°`；
- 有新增剖面时布局变为 2×3；
- 只有新增剖面可以删除；全部删除后恢复 2×2；
- 所有新增和编辑状态只在当前 GUI 会话有效，重新启动后恢复默认四剖面。

“剖面半长”允许 `10～35 m`，默认 `20 m`；“剖面厚度”表示选取带的总厚度。修改后 2D 剖面和 3D 轮廓立即刷新。

### 第七步：人工 6DOF 调整

所有人工平移和旋转轴都是 `map` 轴；旋转中心固定为初始 LiDAR 原点：

```text
C0 = initial_T_map_lidar[:3, 3]
```

快捷键：

| 参数 | 负方向 | 正方向 | 坐标系 |
|---|---:|---:|---|
| ΔX | `A` | `D` | map X |
| ΔY | `S` | `W` | map Y |
| ΔZ | `E` | `Q` | map Z |
| yaw | `3` | `1` | 绕 map Z |
| pitch | `6` | `4` | 绕 map Y |
| roll | `9` | `7` | 绕 map X |

按住 Shift 使用配置中的大步长。也可在右侧数值框直接输入。按钮和快捷键：

- “重置”或 `R`：回到零增量；
- “撤销”：恢复上一次姿态增量；
- “重做”：重做被撤销的增量；
- `G`：导出当前帧。

欧拉角按 `ZYX` 顺序从 yaw、pitch、roll 构造旋转。矩阵关系为：

```text
T_manual_map = [R, C0 - R @ C0 + Δt]
corrected_T_map_lidar = T_manual_map @ initial_T_map_lidar
P_map_corrected = T_manual_map @ P_map
```

### 第八步：可选受限 ICP

点击“ICP”运行 point-to-plane ICP。当前源码固定使用：

| 参数 | 值 |
|---|---:|
| ROI 半径 | `25.0 m` |
| 体素尺寸 | `0.08 m` |
| 最大对应距离 | `0.35 m` |
| 最大迭代次数 | `50` |
| 允许的原点平移修正 | 不超过 `0.5 m` |
| 允许的旋转修正 | 不超过 `3.0°` |

源和目标 ROI 各少于 10 个点时 ICP 会拒绝执行。ICP 结果超过限制时也会被拒绝，不会应用该修正。成功后姿态、视图和质量指标更新。

任何后续人工姿态修改、重置、撤销或重做都会把已有质量指标标记为无效（显示为 `null`），需要重新运行 ICP 才能得到新的 ICP 指标。

### 第九步：导出和继续下一帧

1. 确认 YAML 输出目录存在。
2. 如需校正后 PCD，勾选“导出调整后 PCD”并确认 PCD 目录存在。
3. 点击“导出当前帧”或按 `G`。
4. 同名文件存在时确认是否覆盖。

YAML 成功写出后，当前 Frame ID 立即显示“已标注”，计数更新为 `已标注数/总量`。PCD 是对完整单帧点应用一次 `T_manual_map` 的结果，不使用显示体素点，也不会再次应用初始位姿；输入颜色存在时会传递到输出。

使用“下一帧”选择 Frame ID 后，再点击“加载当前帧”继续。

## 可选方式：空 GUI 启动

```powershell
cd D:\0_code\frame_register_manual
.\.venv\Scripts\python.exe .\src\frame_align_6dof.py
```

界面不预填配置；使用右侧浏览按钮选择路径。

## 可选方式：兼容单文件启动

```powershell
cd D:\0_code\frame_register_manual
.\.venv\Scripts\python.exe .\src\frame_align_6dof.py `
  --map .\data\global_map.pcd `
  --source .\data\frames\1001.pcd `
  --pose .\data\poses\1001.txt `
  --output-dir .\alignment_output\yaml
```

规则：

- `--map`、`--source`、`--pose` 必须同时提供；
- `--source` 是已经位于 `map` 坐标系的单个点云文件；
- `--output-dir` 不存在时该兼容入口会创建它，并同时用作 YAML 与 PCD 输出目录；
- 省略 `--output-dir` 时使用当前目录下的 `alignment_output`；
- `--config` 也可和这三个参数一起使用，CLI 单文件路径会覆盖相应加载输入。

## 日志和运行状态

项目没有持久化日志文件。运行信息通过以下位置查看：

- 弹窗：路径、解析、加载、ICP、覆盖和导出错误；
- 窗口状态栏：成功加载的文件、点数、LiDAR 原点、耗时和导出路径；
- 右侧姿态矩阵框：`C0`、当前原点、`T_manual_map`、校正位姿和质量指标；
- PowerShell：未由 GUI 捕获的启动错误和 Python traceback。

# 常见问题与排查

## 依赖缺失

现象：

```text
ModuleNotFoundError: No module named 'PySide6'
```

处理：

```powershell
cd D:\0_code\frame_register_manual
.\.venv\Scripts\python.exe -m pip install -r .\requirements.txt
```

确认启动和安装使用的是同一个 `.venv` 解释器。

## 配置文件不存在

现象：

```text
FileNotFoundError: ... config.yaml
```

处理：确认仓库根目录存在 `config.yaml`，并从项目根目录执行：

```powershell
.\.venv\Scripts\python.exe .\src\frame_align_6dof.py --config .\config.yaml
```

也可以把 `--config` 后的路径改成实际配置文件绝对路径。

## 相对路径指向错误

配置中的相对路径相对于配置 YAML 所在目录。可暂时改用绝对路径确认问题。GUI 加载成功后，状态栏会显示解析后的真实输入路径。

## Frame ID 找不到文件

程序要求同时存在：

```text
<frame_cloud_map_path>/<frame_id>.pcd
<initial_pose_path>/<frame_id>.txt
```

检查文件名 stem 是否完全对应。扫描只查看单帧目录顶层，不会进入子目录。

## 位姿矩阵不是合法旋转

典型错误包括：

```text
rotation is not orthonormal
rotation determinant is not +1
wrong count for Tr_velo_to_map
```

检查 TXT 中是否只有一条 `Tr_velo_to_map`、是否恰好 12 个有限数值，以及 3×3 旋转块是否为合法旋转。不要把缩放、剪切或缺失的小数复制进位姿矩阵。

## Jinhua 可显示，其他地图为空

项目没有按数据集名称做匹配。依次检查：

1. 全局地图和单帧 PCD 的坐标范围是否一致；
2. 初始位姿平移是否落在该范围；
3. 若 PCD 已整体平移而位姿仍在 world 范围，是否在全局地图同目录放置了正确的 `map_anchor.yaml`；
4. `map_translation_offset_xyz` 的符号是否满足 `pose_map = pose_world - offset`；
5. `display.map_roi_radius_m` 是否足够覆盖局部地图。

## 3D 点云不显示

- 检查状态栏中的全局地图点数、单帧点数和 `C0`。
- 若地图总点数非零但 ROI 为零，通常是位姿原点与地图坐标范围不一致。
- 检查是否需要 `map_anchor.yaml`，以及它是否放在全局地图文件的父目录。
- 确认桌面 OpenGL 可用；`QT_QPA_PLATFORM=offscreen` 不适合人工检查真实渲染。
- 调整 3D 相机视角；程序只在成功加载且地图 ROI 非空时自动聚焦一次。

## 某个剖面没有点

- 增大“剖面厚度”。
- 在允许范围内增大“剖面半长”。
- 检查剖面角度或平行偏移是否穿过目标结构。
- 检查 LiDAR yaw 和初始位姿是否正确；XZ/YZ 跟随的是校正后 LiDAR 航向，不是固定屏幕方向。
- 参考点来自加载时围绕 `C0` 裁剪的地图 ROI；过大的人工位移可能移出该 ROI，需要重新加载或调整 `map_roi_radius_m`。

## ICP 无法运行或修正被拒绝

- 源/目标 ROI 必须各至少 10 个点。
- 初始误差过大时，`0.35 m` 最大对应距离可能不足；先人工粗调。
- ICP 增量若使当前原点移动超过 `0.5 m` 或旋转超过 `3°`，程序会拒绝结果。
- 当前限制在源码 `frame_alignment/core/registration.py` 中固定，GUI/配置没有对应字段。

## 导出按钮不可用

- 必须先成功加载一帧；
- YAML 输出目录必须已存在；
- 勾选 PCD 导出时，PCD 输出目录也必须已存在。

创建目录：

```powershell
New-Item -ItemType Directory -Force .\alignment_output\yaml, .\alignment_output\pcd
```

## 已导出但仍显示“未标注”

状态按精确文件判断：

```text
<output_path_yaml>/<frame_id>.yaml
```

确认 YAML 输出目录和当前 Frame ID。点击“刷新帧列表”会重新统计当前扫描到的所有帧。

## 程序启动无输出或长时间卡住

- 大地图读取、Open3D 解析、ROI 和体素降采样都在当前 GUI 启动/加载流程中执行，仓库没有后台进度条。
- 先运行 `--self-test`；它不读取真实点云，可区分核心矩阵逻辑与数据加载问题。
- 使用较小的有效数据副本验证格式；不要用空文件冒充点云。
- 查看启动 PowerShell 是否有 traceback。
- 若窗口“未响应”，先等待磁盘读取完成；持续异常时核对文件大小、磁盘速度和可用内存。

## GPU、CUDA 或驱动问题

本项目没有 CUDA 配置项，也没有声明必须使用专用 GPU。3D 界面仍依赖 OpenGL 驱动。若 `--self-test` 通过而 GUI/3D 初始化失败，应优先检查桌面会话、显卡驱动、PyQtGraph 和 PyOpenGL，而不是安装 CUDA。

## 权限、网络和代理问题

- 安装依赖需要访问 Python 包索引；网络受限时按组织规范设置 pip 镜像或代理，仓库没有固定代理配置。
- 输出目录需要当前用户具有创建临时文件、替换文件和删除备份文件的权限。
- 不要把输出目录设置到只读介质或无权限的共享目录。
- 项目没有 Docker 方式，因此没有已验证的容器挂载命令。

# 验收与自检

## 判断安装正确

在 Windows PowerShell、项目根目录执行：

```powershell
.\.venv\Scripts\python.exe -c "import numpy, scipy, open3d, PySide6, pyqtgraph, OpenGL, yaml, laspy; print('runtime imports OK')"
.\.venv\Scripts\python.exe .\src\frame_align_6dof.py --help
.\.venv\Scripts\python.exe .\src\frame_align_6dof.py --self-test
```

验收标准：

1. 依赖导入输出 `runtime imports OK`。
2. `--help` 显示所有已记录参数。
3. `--self-test` 输出 `Self-test passed` 且退出码为 0。

## 判断 GUI 和数据正确加载

1. GUI 正常打开，中文和 Δ/° 字符显示正常。
2. 扫描后 Frame ID 数量符合单帧目录顶层 PCD 数量。
3. 点击“加载当前帧”后状态栏显示正确 PCD/TXT 路径和非零点数。
4. 3D 中能看到 Reference、Adjusted、LiDAR 原点和红绿蓝轴。
5. 默认四个剖面都有符合数据分布的点；若没有，应能用厚度、长度和坐标排查定位原因。
6. 调整 yaw 时剖面轮廓跟随 LiDAR 水平轴旋转，高度轴保持 `map Z`。

## 判断一次任务正确完成

1. 导出的 YAML 文件名与实际加载 PCD stem 相同。
2. GUI 显示“已标注”，计数增加。
3. YAML 的 `input` 路径指向本次输入。
4. `initial_T_map_lidar`、六自由度增量、`T_manual_map` 和 `corrected_T_map_lidar` 齐全。
5. 矩阵满足 `corrected_T_map_lidar = T_manual_map @ initial_T_map_lidar`。
6. 若导出 PCD，`output.adjusted_pcd_written` 为 `true` 且文件存在；否则应为 `false` 且路径为 `null`。
7. 重新选择同一 Frame ID 时，标注状态仍能从 YAML 文件正确识别。

## 推荐最小测试

不安装 pytest 时，可使用 Python 标准库 `unittest` 运行与入口、地图锚点和动态剖面直接相关的测试：

```powershell
$env:QT_QPA_PLATFORM='offscreen'
.\.venv\Scripts\python.exe -m unittest `
  tests.test_cli_entry `
  tests.test_map_anchor `
  tests.test_map_anchor_integration `
  tests.test_dynamic_profile_geometry `
  tests.test_dynamic_profile_layout
```

如环境已经安装 pytest，可运行完整测试：

```powershell
$env:QT_QPA_PLATFORM='offscreen'
.\.venv\Scripts\python.exe -m pytest -q
```

真实样本测试依赖代码中指定的本机数据路径；样本不存在时相应测试会跳过。运行完整测试可能读取大型点云并耗时较长。

## 仍需维护者确认的信息

- 项目正式支持的 Python 版本范围。
- Windows 之外的正式支持范围。
- LAZ 解压 backend 的推荐包和版本。
- pytest 是否应加入开发依赖及其版本策略。
- 是否需要发布安装包、Docker 镜像或独立启动脚本。
