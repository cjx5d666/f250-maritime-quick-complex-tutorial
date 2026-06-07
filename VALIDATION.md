# 验证与保留证据

本文说明本目录式教程仓库保留了哪些已验证证据，以及重新运行前仍需要确认的事项。所有路径均为仓库内相对路径。

## 验证范围

- 固定任务：2026-05-30 P0-P8 hard-requirement quick-complex 路线。
- 固定车辆：F250。
- 固定地图 authority：`data/map_authority/p0_p8_hard_requirement_20260530/`。
- 路线证据：`evidence/expected_route/`。
- FC Metric 3.10 证据：`evidence/expected_fc_3_10/`。

路线/规划验收不包含 Metric 3.10，也不包含 yaw pass/fail。Metric 3.10 是独立的 FC-only 稳态证据。动态船只 clearance 是 telemetry；当前路线安全 gate 以静态障碍物安全为准。

## 视觉展示层

RViz/Gazebo 展示层保留主要视觉对象，同时调整风机与动态船只的 planner
cloud geometry；路线点和其它静态障碍物不变：

- 动态 WAM-V planner obstacle size：`[12.0, 6.0, 6.0]`，center z：`3.5`。
- 动态 WAM-V visual mesh scale：`4.0`，mesh pose z：`-3.1`。
- WAM-V name、yaw 和 sinusoid motion 逻辑保持不变。
- RViz `/maritime/scene_markers` 发布静态 mesh markers：起飞邮轮/母船、
  终点油轮、岛/山、桥、风机、救生圈。
- RViz 同 topic 发布动态 WAM-V mesh marker。
- RViz raw planner CUBE/CYLINDER markers 默认关闭；需要调试原始规划几何时，
  设置 `MARITIME_SHOW_RAW_PLANNER_SHAPES=true`。
- deck、起降区、ship hull、dock、visual_boxes 等 reference primitives 默认关闭；
  需要调试参考几何时，设置 `MARITIME_SHOW_REFERENCE_PRIMITIVES=true`。
- Wind-channel planner cloud boxes 几何不变，size：`[10.0, 8.0, 18.0]`，
  center z：`9.0`；默认 RViz 画面不显示 raw boxes。

## 地图 Authority

地图 authority 保留在：

```text
data/map_authority/p0_p8_hard_requirement_20260530/
```

关键文件：

- `route_waypoints.csv`：P0-P8 路线点。
- `planner_obstacles.csv`：规划侧障碍物。
- `visual_mesh_footprints.csv`：可视模型 footprint。
- `hard_requirement_metrics.json`：硬需求几何指标。
- `planned_clearance_after_hard_requirements.json`：规划路线 clearance 审计。
- `map_manifest.json`：地图来源和输出清单。
- `01_clean_visual_base.png`、`02_visual_planner_obstacles.png`、`03_visual_planner_route_p0_p8.png`：展示图，只作审核辅助。

几何真值来自 CSV/JSON/YAML/SDF，不来自 PNG。

## 路线证据

路线证据保留在：

```text
evidence/expected_route/
```

最终选择：

- 家族：R4_H。
- 选中运行：`runs/lidar_lidar_R4_H_r2/`。
- 选择摘要：`final_selection.json`、`selection_summary.csv`。
- 选中轨迹：`selected_actual_trajectory.csv`。
- 审核展示图：`f250_historical_planned_vs_flown.png`。

代表结果：

- P8 reached：true。
- stop reason：`final_hold_reached`。
- 静态安全：SAFE。
- 静态 geometry entry：0。
- 静态 cloud entry：0。
- 静态 collision：false。
- 选中运行静态 clearance：0.8086800027841847 m。
- P1-P7 keypoint error mean：0.238261136879 m。
- P1-P7 keypoint error max：0.458080235519 m。
- P8 endpoint error：0.5952538810440474 m。
- P0-P8 任务时长：113.361 s。

`evidence/expected_route/zyaw_metrics/` 保留 yaw 相关复核材料，但 yaw pass/fail 不进入当前路线验收口径。

## FC Metric 3.10 证据

FC 证据保留在：

```text
evidence/expected_fc_3_10/
```

关键文件：

- `fc_3_10_summary.json`：正式汇总。
- `fc_3_10_samples.csv`：采样数据。
- `fc_3_10_phases.csv`：phase/window 数据。
- `fc_3_10_geometry_audit.json`：A/B 与 decagon 几何安全审计。
- `fc_3_10_decagon_points.csv`：位置项 decagon 点。

正式保留值：

```text
E3.10_selected=2.261625026799532%
```

分项：

- 所有 30 个正式 metric windows settled。
- `E_pos=0.727682193133525%`。
- `E_vel_2mps=2.261625026799532%`。
- `E_vel_selected=2.261625026799532%`。
- `E_yaw=0.28562253585743336%`。
- 选中速度项为 2.0 m/s。
- 几何审计结论：FC 3.10 A/B 和 decagon 几何 SAFE。

同日重复 FC 运行 settled，但数值存在波动；本仓库保留值是正式参考证据，不代表每次重跑会逐字节一致。

## Caveats

- 本仓库不 vend PX4、ROS、MAVROS、Gazebo、EGO-Planner 或系统依赖。
- P0 悬停 smoke 会经过 `maritime_ego_planner.launch`，因此也会验证本仓库内的 F250 专用 EGO-Planner launch wrapper 是否能接收 quick-complex 的 tunable 参数并启动外部 `ego_planner` 节点。
- 重新运行前必须确认 PX4 overlay 已安装，并且 PX4 airframes `CMakeLists.txt` 已注册 `10020_gazebo-classic_f250`。
- 部分保留 JSON 中的 `cache/...` 字段是历史 provenance，不是当前仓库默认运行输出路径；当前默认输出路径为 `runs/f250_human_scripts/`。
- 大型 OBJ mesh 原样保留，便于 `git clone` 后直接使用；这些文件低于 GitHub CLI push 的 100 MiB 单文件限制，但高于浏览器上传舒适范围。
- 仿真重跑会受到实时调度、图形环境、PX4/Gazebo 状态和依赖状态影响，数值允许存在小幅波动。
