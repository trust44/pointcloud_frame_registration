# 匹配误差阶段与独立点云渲染设计

## 目标

在当前 6DOF 配准 GUI 中增加不改变位姿的匹配误差评估，保留 ICP 阶段快照指标，并为全局地图和单帧点云分别提供可配置、可运行时调整的 3D 渲染模式；2D 剖面配色保持现状。

## 匹配误差

在现有 ICP 按钮旁增加“计算误差”按钮。按钮使用当前姿态评估点云，不修改 `PoseModel`。

默认配置：

```yaml
error:
  method: nearest_neighbor
  evaluation_radius_m: 25.0
  match_threshold_m: 0.2
```

`evaluation_radius_m` 和 `match_threshold_m` 均可由配置调整。第一版提供三种方法：

- `nearest_neighbor`：调整后单帧到地图的单向最近邻距离；默认方法。
- `symmetric_chamfer`：单帧到地图、地图到单帧的对称最近邻距离。
- `point_to_plane`：基于地图法向的点到平面距离。

每种方法输出 `median_m`、`mean_m`、`rmse_m`、`p95_m` 和 `match_ratio`。匹配比例为距离不超过 `match_threshold_m` 的有效点比例。评估仅使用当前原点附近的评估 ROI。

## ICP 阶段

加载新帧时清空所有阶段指标。第一次成功 ICP 时保存不可被手动调整覆盖的 `quality.initial_icp`，内容包括 ICP 自身的 RMSE、fitness、平移修正量和旋转修正量。ICP 完成后立即计算一次几何匹配误差，保存为 `quality.icp_error`。用户后续点击“计算误差”时，将当前人工调整结果保存为 `quality.manual_error`。

手动调整不清除 `initial_icp` 或 `icp_error`。重复成功执行 ICP 时保留第一次成功 ICP 的 `initial_icp`，更新当前 `icp_error`。所有阶段结果写入 YAML，并在矩阵/质量显示区域区分展示。

## 独立点云渲染

配置按点云分别设置：

```yaml
display:
  clouds:
    global:
      mode: uniform
      color: "#90A4AE"
      cmap: viridis
      scalar: z
      range: auto
    source:
      mode: uniform
      color: "#F6C445"
      cmap: plasma
      scalar: z
      range: auto
```

每个点云支持 `uniform`、`native` 和 `cmap` 模式。`uniform` 使用单色；`native` 使用输入点云 RGB（不可用时回退单色）；`cmap` 根据 `z` 或到当前 LiDAR 原点的 `distance` 计算颜色。cmap 第一版支持 `viridis`、`plasma`、`inferno`、`turbo`、`gray`。`range: auto` 使用当前显示点自动归一化，也支持 `[min, max]` 固定范围。

配置提供启动默认值；GUI 提供全局地图和单帧点云各自的会话内覆盖，不自动写回配置文件。颜色刷新不重新读取点云、不改变位姿、ICP 或误差状态。

## 剖面约束

2D 剖面和 3D 剖面轮廓保持现有固定配色与图例，不随 3D 点云的 mode、color 或 cmap 改变。

## 测试范围

- 误差方法的统计量、阈值和空/不足点处理；
- “计算误差”不改变位姿；
- ICP 首次快照、重复 ICP、手动调整后的阶段隔离和 YAML 导出；
- 配置解析、GUI 会话覆盖、三种渲染模式、cmap 和固定/自动范围；
- 现有剖面配色和既有功能回归。
