# 验证与保留证据

本文说明本目录式教程仓库保留了哪些已验证证据，以及重新运行前仍需要确认的事项。所有路径均为仓库内相对路径。

## 验证范围

- 固定任务：2026-05-30 P0-P8 hard-requirement quick-complex 路线。
- 固定车辆：F250。
- 固定地图 authority：`data/map_authority/p0_p8_hard_requirement_20260530/`。
- 路线证据：`evidence/expected_route/`。
- FC Metric 3.10 证据：`evidence/expected_fc_3_10/`。

路线/规划验收终端只显示当前汇报使用的 `3.6`、`3.8` 和 `3.9`。Metric 3.10 是独立的 FC-only 稳态证据，不进入路线通过/失败判定；yaw pass/fail、Dsafe 和静态安全 gate 不再作为路线终端指标显示。动态船只 clearance 与静态 clearance 仍写入 JSON/日志，作为审计与调试材料。

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

仓库内 `evidence/expected_route/` 仍保留历史 R4_H 参考证据，用于说明路线几何和早期选择依据。2026-06-09 VM 实机重跑的当前终端验收结果为：

```text
run: runs/f250_human_scripts/f250_p0_p8_route_20260609_031417
3.6 keypoint arrival error mean = 1.664%
3.6 keypoint arrival error max  = 5.074%
3.8 planning / route success    = 8/8 = 100.0%
3.9 final target error          = 0.180%
result PASS
```

对应 JSON 值：

- `metric_3_6.mean_error_ratio=0.016637490559317753`。
- `metric_3_6.max_error_ratio=0.05073716207912651`。
- `metric_3_8.max_active_goal_index=8`，`required_final_goal_index=8`。
- `metric_3_9.final_error_m=0.559071258848178`。
- `metric_3_9.final_error_ratio=0.0018037512663880706`。
- `route.total_p0_p8_length_m=309.9491982437759`。

路线终端会在 P1-P8 逐点显示 `reached n/8` 和成功率。每个 keypoint 的 3.6 百分比与 P8 的 3.9 百分比在任务结束后由离线 `metric_summary.json` 打印，因此不会受途中 active-nearest 临时值影响。`evidence/expected_route/zyaw_metrics/` 保留 yaw 相关复核材料，但 yaw pass/fail 不进入当前路线验收口径。

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

2026-06-09 VM 实机重跑值：

```text
run: runs/f250_human_scripts/f250_fc_3_10_steady_state_20260609_032735
3.10 e_pos mean = 0.371%
3.10 e_vel mean = 2.099%
3.10 e_att mean = 0.122%
3.10 control stability error = 2.099%
result PASS
```

分项：

- 所有 30 个正式 metric windows settled。
- `E_pos=0.37096258341386323%`。
- `E_vel_2mps=2.098646302206202%`。
- `E_vel_selected=2.098646302206202%`。
- `E_yaw=0.12206005408457488%`，终端显示为 `e_att`。
- 选中速度项为 2.0 m/s。
- 几何审计结论：FC 3.10 A/B 和 decagon 几何 SAFE。

3.10 现在使用每类 10 个正式窗口的均值，再取 `max(mean e_pos, mean e_vel, mean e_att)` 作为最终 `3.10 control stability error`。同日重复 FC 运行 settled，但数值存在波动；不要求每次重跑逐字节一致。

## Caveats

- 本仓库不 vend PX4、ROS、MAVROS、Gazebo、EGO-Planner 或系统依赖。
- P0 悬停 smoke 会经过 `maritime_ego_planner.launch`，因此也会验证本仓库内的 F250 专用 EGO-Planner launch wrapper 是否能接收 quick-complex 的 tunable 参数并启动外部 `ego_planner` 节点。
- 重新运行前必须确认 PX4 overlay 已安装，并且 PX4 airframes `CMakeLists.txt` 已注册 `10020_gazebo-classic_f250`。
- 部分保留 JSON 中的 `cache/...` 字段是历史 provenance，不是当前仓库默认运行输出路径；当前默认输出路径为 `runs/f250_human_scripts/`。
- 大型 OBJ mesh 原样保留，便于 `git clone` 后直接使用；这些文件低于 GitHub CLI push 的 100 MiB 单文件限制，但高于浏览器上传舒适范围。
- 仿真重跑会受到实时调度、图形环境、PX4/Gazebo 状态和依赖状态影响，数值允许存在小幅波动。
