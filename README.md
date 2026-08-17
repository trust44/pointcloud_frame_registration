# 单帧点云与全局点云 6DOF 剖面配准工具

启动入口是 `src/frame_align_6dof.py`；原有的 `src/frame_register_manual.py` 保持不变。

## 使用方法
见`USER_GUIDE.md`

## 安装与启动

```powershell
python -m pip install -r requirements.txt
python .\src\frame_align_6dof.py
```

也可由配置文件预填路径和显示参数：

```powershell
python .\src\frame_align_6dof.py --config .\config.example.yaml
```

配置中的相对路径以 YAML 文件所在目录为基准。旧式的单文件参数仍可使用：

```powershell
python .\src\frame_align_6dof.py --map global_map.laz --source frame.pcd --pose initial_pose.txt
```

`--pose` 可为含 `Tr_velo_to_map` 的 TXT，或含 `matrix` 的 YAML。

## 加载与帧浏览

主视图区在左侧；右侧滚动栏从上到下依次为 6DOF 调整、姿态矩阵和“数据与输出”。

- 选择全局地图文件、单帧点云目录、初始位姿目录和输出目录。
- 点击“刷新帧列表”扫描单帧目录最外层的 `.pcd` 文件；Frame ID 下拉框可浏览、上一帧/下一帧切换，也可直接手动输入。
- 浏览或输入 Frame ID **不会自动加载**；点击“加载当前帧”后才读取以下文件：

  ```text
  <frame_cloud_map_path>/<frame_id>.pcd
  <initial_pose_path>/<frame_id>.txt
  ```

- 加载失败时，当前已成功加载的帧和配准状态会保留。成功加载后，3D 相机会自动对准初始 LiDAR 原点；手动调整、切片更新、撤销和重做不会改变当前观察视角。

右侧“已标注/未标注”依据精确文件
`<output_path_yaml>/<frame_id>.yaml` 判定；旁边的计数显示为
`已标注/总量`。切换 YAML 输出目录、切换 Frame ID 或点击“刷新帧列表”时都会刷新；导出 YAML 成功后状态也会立即更新。

## 交互

- `A/D`、`W/S`、`Q/E`：在 map 坐标系调整 ΔX、ΔY、ΔZ；按住 Shift 使用大步长。
- `1/3`、`4/6`、`7/9`：调整 yaw、pitch、roll；按住 Shift 使用大步长。
- `R` 重置；`G` 导出；右侧控制面板还提供撤销、重做和受限 ICP。

旋转始终围绕初始 LiDAR 原点 `C0 = T_map_lidar[:3, 3]`。单帧 PCD 已位于 map 坐标系，不会再次应用 `T0`。左侧同时显示 Reference、Adjusted、LiDAR Origin、四个剖面矩形和四个 2D 剖面（X-Z、Y-Z、+45°、-45°）。

## 导出

YAML 是必选输出，文件名使用实际加载 PCD 的 stem，例如 `123.pcd` 输出为 `123.yaml`。勾选后还可导出同名的调整后 PCD。写入采用临时文件后原子替换；同名文件存在时会请求确认。

导出前，YAML 输出目录必须已存在；如勾选“导出调整后 PCD”，PCD 输出目录也必须已存在。YAML 包含输入路径、初始位姿、6DOF 增量、手动变换、校正位姿、质量指标和 PCD 实际写入状态。

## 测试

```powershell
$env:QT_QPA_PLATFORM='offscreen'
python -m pytest -q
python .\src\frame_align_6dof.py --self-test
```

若本机存在 Jinhua 真实样本，`tests/test_real_sample_rendering.py` 会额外读取它们，验证高坐标 3D ROI、四方向剖面（含 X-Z）和相机中心；样本文件不存在时该测试会自动跳过。
