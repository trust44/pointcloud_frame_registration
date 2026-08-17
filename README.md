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

主视图区在左侧；右侧滚动栏从上到下依次为 6DOF 调整、剖面设置、姿态矩阵和“数据与输出”。

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

旋转始终围绕初始 LiDAR 原点 `C0 = T_map_lidar[:3, 3]`。单帧 PCD 已位于 map 坐标系，不会再次应用 `T0`。左侧同时显示 Reference、Adjusted、LiDAR Origin，以及 4～6 个对应一致的 3D 剖面轮廓和 2D 剖面。

## 剖面设置

- 默认四个剖面按 2×2 排列：XZ、YZ、相对 XZ 的 `+45°` 和 `-45°`。XZ、YZ 固定；两个默认对角剖面可修改位置但不可删除。
- 所有剖面的水平方向只跟随当前校正位姿的 LiDAR yaw；高度方向始终使用 map Z，因此 pitch/roll 不会使剖面倾斜。
- 可编辑剖面支持两种定位方式：

  - “与 XZ 夹角”：输入 `-180°～180°`；正角方向为从 LiDAR 水平 X 轴向水平 Y 轴旋转。
  - “平行偏移”：选择 XZ 或 YZ，并输入有符号距离；XZ 正距离沿 LiDAR 水平 `+Y`，YZ 正距离沿 LiDAR 水平 `-X`。

- 最多新增两个剖面。第一个位于右上格，默认 `+90°`；第二个位于右下格，默认 `-90°`。新增后布局自动变为 2×3，新增剖面可删除。
- “剖面半长”默认 `20m`，允许范围为 `10～35m`；修改后同步刷新全部 2D 剖面和 3D 轮廓。
- 剖面编辑、新增和删除只在当前 GUI 会话中有效；关闭后再次启动会恢复默认四剖面和 2×2 布局。

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
