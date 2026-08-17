# 地图坐标锚点兼容设计

## 目标

让全局点云与单帧点云已经处于同一坐标范围的数据集，能够使用同目录
`map_anchor.yaml` 将 calib 中仍处于 world 坐标的 `Tr_velo_to_map`
转换到 PCD 坐标范围，从而正确加载地图 ROI、定位相机并显示剖面。

本次只解决坐标锚点兼容，不修改点云内容、ICP、手动 6DOF 语义、导出格式、
动态剖面布局或 `src/frame_register_manual.py`。

## 已确认根因

生产代码没有 Jinhua 路径硬编码。`MainWindow` 使用初始位姿平移作为 ROI、相机和
剖面中心。Jinhua 的位姿原点已位于 PCD 范围；Yinxiu 和 Xiangxue 的 PCD 已平移，
但 calib 位姿仍为 world 坐标，因此原始 ROI 点数为零。

三组真实样本已验证：

| 数据集 | `map_translation_offset_xyz` | 原始 ROI | 修正后 ROI |
|---|---:|---:|---:|
| Jinhua | `[0, 0, 0]` | `906578` | `906578` |
| Yinxiu | `[-10000, -10000, 0]` | `0` | `915928` |
| Xiangxue | `[50000, 10000, 0]` | `0` | `1262008` |

## 最小实现

新增一个只负责地图锚点的 I/O 模块。它在全局地图文件同目录查找
`map_anchor.yaml`：

```text
<global_map_path parent>/map_anchor.yaml
```

若文件不存在，返回原位姿副本。若文件存在，则读取有限三维向量
`map_translation_offset_xyz`，只修正平移：

```text
T_map_lidar_corrected[:3, 3]
    = T_map_lidar_world[:3, 3] - map_translation_offset_xyz
```

旋转块和齐次矩阵最后一行保持不变。字段缺失、维度错误或含非有限值时抛出包含
锚点文件路径的 `ValueError`，避免静默使用错误坐标。

目录式 GUI 加载和旧式 `--map/--source/--pose` 加载共用该函数，保证入口一致。

## 测试边界

只执行以下最小测试：

1. 临时文件单元测试：无锚点保持不变；有效锚点只修正平移；非法锚点被拒绝。
2. 帧加载测试：确认加载器使用修正后的初始位姿。
3. 三组真实样本测试：实际读取全局/单帧 PCD，确认修正后 35m ROI 非空，四个默认
   剖面的地图点和单帧点都非空。
4. 入口自检、编译检查和原脚本未修改检查。

不运行全量测试，不进行 ICP 或导出回归。
