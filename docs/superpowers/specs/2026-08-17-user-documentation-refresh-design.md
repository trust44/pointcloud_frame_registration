# 用户文档更新设计

## 目标

基于当前 `develop` 分支的真实实现，更新项目根目录的 `README.md` 和 `USER_GUIDE.md`，使第一次接触项目的工程师能够理解项目用途、准备正确输入、安装依赖、启动 GUI、完成逐帧配准并识别输出结果。

## 文档分工

- `README.md`：项目首页。包含项目定位、核心功能、输入输出、快速安装、推荐启动方式、核心坐标约定和详细指南链接。
- `USER_GUIDE.md`：完整用户手册。包含环境、依赖、数据与目录格式、配置字段、`map_anchor.yaml`、GUI 操作、动态剖面、6DOF/ICP、导出字段、故障排查和最小自检。

## 事实来源

所有描述以仓库中的 `requirements.txt`、`config.yaml`、`src/frame_align_6dof.py`、`frame_alignment/` 和 `tests/` 为准。仓库中不存在的入口、依赖版本、容器方式和数据文件不写成已支持能力；无法从仓库确认的内容标记为“需确认”。

## 关键内容

1. 说明单帧 PCD 已处于 map 坐标系，程序只对其施加人工/ICP 校正矩阵，不再次应用初始 `T_map_lidar`。
2. 说明目录模式下每个 Frame ID 对应 `<frame_id>.pcd` 和 `<frame_id>.txt`，TXT 必须包含唯一合法的 `Tr_velo_to_map`。
3. 说明全局地图同目录下可选 `map_anchor.yaml` 的自动发现规则，以及 `map_translation_offset_xyz` 从位姿平移中扣除的行为。
4. 说明默认四剖面、最多两个会话内新增剖面、2×2/2×3 布局、LiDAR yaw 跟随、map Z 高度轴、角度/平行偏移和半长/厚度设置。
5. 说明 map 坐标系 6DOF、初始 LiDAR 原点旋转中心、撤销/重做、受限 point-to-plane ICP 及质量指标失效规则。
6. 说明帧扫描、显式加载、标注状态与计数，以及 YAML 必选、调整后 PCD 可选、同名覆盖确认和原子写入。
7. 修正现有文档中不存在的 `config.example.yaml` 和过期依赖文件引用。

## 验证

- 检查 `README.md` 和 `USER_GUIDE.md` 不再引用不存在的文件。
- 使用项目 `.venv` 执行 `src/frame_align_6dof.py --help` 和 `--self-test`。
- 执行与配置、map anchor、CLI 和文档所述动态剖面直接相关的最小测试集，不运行真实大点云测试。
- 执行 `git diff --check`，确认只更新约定的用户文档（设计记录除外），且不改业务代码。
