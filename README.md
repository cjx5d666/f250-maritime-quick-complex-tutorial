# F250 海事 Quick-Complex 教程

这个仓库是 F250 海事 PX4/Gazebo/MAVROS/EGO-Planner quick-complex 教程本体。它不是根目录压缩包仓库；克隆后直接在仓库目录内检查、安装 overlay、构建 catkin 工作空间并运行脚本。

已验证范围固定为 2026-05-30 P0-P8 hard-requirement quick-complex 路线、F250 参考机体、Gazebo Classic SITL、P0 悬停、独立 FC Metric 3.10 稳态检查、P0-P8 路线记录和最终 planned-vs-flown 绘图。路线几何、静态障碍物和地图 authority 固定，不在本教程中重新设计。

本包在 VM 上的已知兼容环境为 Ubuntu 20.04.6、ROS Noetic、Gazebo Classic 11.15.1、MAVROS 1.20.1、PX4 v1.16.0 风格源码树。PNG 仅作展示审核，CSV/JSON/YAML/SDF 才是几何和验收依据。

## 仓库结构

```text
.
├── README.md
├── VALIDATION.md
├── env.example
├── scripts/
│   ├── check_package.sh
│   ├── install_px4_overlay.sh
│   └── build_catkin_ws.sh
├── catkin_ws/src/f250_maritime_uav_sim/
├── px4_overlay/
├── data/map_authority/p0_p8_hard_requirement_20260530/
└── evidence/
    ├── expected_route/
    └── expected_fc_3_10/
```

关键内容：

- `catkin_ws/src/f250_maritime_uav_sim`：ROS 包、launch、world、模型、运行脚本和后处理脚本。
- `catkin_ws/src/f250_maritime_uav_sim/launch/f250_ego_advanced_param_px4_native_pose.xml`：本教程随包携带的 F250 专用 EGO-Planner launch wrapper，用于声明并下发 quick-complex 需要的 planner tunable 参数。
- `px4_overlay`：F250 airframe `10020_gazebo-classic_f250` 和 Gazebo Classic `f250` 模型。
- `data/map_authority/p0_p8_hard_requirement_20260530`：固定地图、路线点、障碍物和渲染依据。
- `evidence/expected_route`：保留的 R4_H 路线证据、轨迹、静态安全和指标摘要。
- `evidence/expected_fc_3_10`：保留的 FC Metric 3.10 稳态证据。
- `scripts/check_package.sh`：公开包静态检查。

## 1. 基线环境

这一部分是基础仿真栈，不是本仓库的自定义内容。下面命令只作为同学搭环境时的参考步骤，未在本仓库候选构建任务中重新安装验证；请按自己的课程镜像、老师要求或已有工作区调整。

需要准备：

- Ubuntu 20.04。
- ROS Noetic 和 catkin 工具。
- Gazebo Classic。
- MAVROS 和 GeographicLib 数据。
- PX4-Autopilot v1.16.0 兼容源码树。
- EGO-Planner 相关依赖，至少需要能提供 `ego_planner`、`traj_server`、`waypoint_generator`、`quadrotor_msgs` 和对应 ROS 依赖；F250 专用可调 launch wrapper 已在本仓库内提供，不要求外部 EGO-Planner 仓库预先带有这份 F250 参数补丁。
- `screen`，用于本教程的人机可见运行脚本。
- 图形桌面/display，用于 Gazebo GUI 或 RViz。

参考安装形态：

```bash
sudo apt update
sudo apt install ros-noetic-desktop-full ros-noetic-mavros ros-noetic-mavros-extras
sudo apt install python3-catkin-tools python3-rosdep geographiclib-tools screen
```

MAVROS GeographicLib 数据按本机 MAVROS 安装方式处理，例如使用 ROS 包自带脚本或系统 `geographiclib-tools`。PX4 和 EGO-Planner 的源码、子模块、编译方式请以你的课程环境为准。

用户旧仓库可作为基线搭建思路的参考链接：

- <https://github.com/cjx5d666/PX4-v1.16.0-F250>
- <https://github.com/cjx5d666/PX4-Gazebo-Egoplanner>

本教程的自定义部分从下一节开始。除 PX4 overlay、地图、模型和运行脚本外，本仓库还携带 `f250_ego_advanced_param_px4_native_pose.xml`，因此 `maritime_ego_planner.launch` 会从 `f250_maritime_uav_sim` 包内 include F250 参数化 wrapper；外部 EGO-Planner 仍作为节点和消息依赖使用。

## 2. 克隆并检查本教程

```bash
git clone https://github.com/cjx5d666/f250-maritime-quick-complex-tutorial.git
cd f250-maritime-quick-complex-tutorial

scripts/check_package.sh
```

不要在仓库根目录放教程压缩包，也不要按“解压根压缩包”的方式使用本教程。如果以后需要冻结版归档，应放在 GitHub Release asset 中，而不是仓库根目录。

## 3. 设置环境变量

```bash
cp env.example env.local
```

编辑 `env.local`，至少设置 PX4 路径：

```bash
export F250_PROJECT_ROOT="/path/to/f250-maritime-quick-complex-tutorial"
export F250_PX4_ROOT="/path/to/PX4-Autopilot"
```

每次运行前加载：

```bash
source env.local
source /opt/ros/noetic/setup.bash
```

如果 EGO-Planner、`traj_server`、`waypoint_generator` 或 `quadrotor_msgs` 来自另一个 catkin 工作空间，运行仿真脚本前也要 source 那个工作空间的 `devel/setup.bash`。构建本仓库 `catkin_ws` 时，推荐把该 setup 文件显式传给 `scripts/build_catkin_ws.sh`，见第 5 节。F250 quick-complex 的可调参数 launch wrapper 在本仓库内，不依赖外部 EGO-Planner launch 文件已被改过。

## 4. 安装 PX4 Overlay

先做无副作用检查：

```bash
export F250_PX4_ROOT="/path/to/PX4-Autopilot"
scripts/install_px4_overlay.sh --dry-run
```

确认目标 PX4 树正确后再安装：

```bash
scripts/install_px4_overlay.sh
```

脚本会：

- 复制 `px4_overlay/ROMFS/.../10020_gazebo-classic_f250` 到 PX4 airframes 目录。
- 复制 `px4_overlay/Tools/simulation/gazebo-classic/.../models/f250` 到 PX4 Gazebo Classic models 目录。
- 如果 `ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt` 尚未包含 `10020_gazebo-classic_f250`，自动注册。
- 如果目标文件已经相同，则不重复写入；如果目标文件不同，会先备份到 PX4 树下 `.f250_overlay_backups/<timestamp>/`。

安装后按你的 PX4 工作流重新编译 SITL，例如在 PX4 根目录执行对应的 `make px4_sitl_default`。这一步属于 PX4 基线编译，不由本仓库保证。

## 5. 构建 Catkin 工作空间

本仓库提供轻量构建脚本。它会先 source ROS Noetic，再 source 你显式传入的依赖工作空间 setup 文件，最后在本仓库 `catkin_ws` 中运行 `catkin_make`。

如果 EGO-Planner、`quadrotor_msgs` 或其他消息依赖来自外部 catkin 工作空间，使用 `--dependency-setup`：

```bash
scripts/build_catkin_ws.sh --dry-run --dependency-setup /path/to/ego_planner_ws/devel/setup.bash
scripts/build_catkin_ws.sh --dependency-setup /path/to/ego_planner_ws/devel/setup.bash
source catkin_ws/devel/setup.bash
```

也可以使用环境变量；多个 setup 文件用冒号分隔，或重复传入 `--dependency-setup`：

```bash
export F250_DEPENDENCY_SETUP="/path/to/ego_planner_ws/devel/setup.bash"
scripts/build_catkin_ws.sh --dry-run
scripts/build_catkin_ws.sh
source catkin_ws/devel/setup.bash
```

如果不传 `--dependency-setup` 且不设置 `F250_DEPENDENCY_SETUP`，EGO-Planner、`quadrotor_msgs`、MAVROS 和 ROS 消息依赖必须已经能在 source ROS 后被找到；否则 `catkin_make` 会失败。这属于基线依赖未就绪。

手动等价命令：

```bash
cd catkin_ws
source /opt/ros/noetic/setup.bash
source /path/to/ego_planner_ws/devel/setup.bash
catkin_make
source devel/setup.bash
```

## 6. 运行顺序

下面脚本都从仓库根目录运行。运行前确保已 source `env.local`、ROS 和本仓库 `catkin_ws/devel/setup.bash`。

1. 启动 F250 并保持 P0 悬停：

```bash
catkin_ws/src/f250_maritime_uav_sim/scripts/f250_start_to_p0_hover.sh
```

P0 目标为：

```text
55.0,16.0,10.0,0.469929
```

2. 运行 FC Metric 3.10 稳态检查：

```bash
catkin_ws/src/f250_maritime_uav_sim/scripts/f250_run_fc_3_10_steady_state.sh
```

3. 从当前 P0 状态释放并记录 P0-P8 路线：

```bash
catkin_ws/src/f250_maritime_uav_sim/scripts/f250_run_p0_p8_route.sh
```

4. 对最新路线生成 planned-vs-flown 图：

```bash
catkin_ws/src/f250_maritime_uav_sim/scripts/f250_plot_latest_run.sh
```

5. 停止 PX4/Gazebo/ROS/RViz/EGO-Planner 链路：

```bash
catkin_ws/src/f250_maritime_uav_sim/scripts/f250_stop_all.sh
```

默认运行输出在：

```text
runs/f250_human_scripts/
```

## 7. 验收口径

路线/规划验收只看 P0-P8 路线任务，不把 Metric 3.10 和 yaw pass/fail 纳入路线通过/失败判定。Metric 3.10 是飞控 FC-only 稳态证据。动态船只 clearance 在当前路线决策中是 telemetry，不作为路线安全 gate。

保留的路线证据族为 R4_H，选中运行为 `lidar_lidar_R4_H_r2`。代表值：

- P8 到达，显示为 P8/8。
- 静态安全 SAFE，无静态 geometry entry、无静态 cloud entry、无静态碰撞。
- 选中运行静态 clearance：0.8086800027841847 m。
- P1-P7 keypoint error：mean 0.238261136879 m，max 0.458080235519 m。
- P8 endpoint error：0.5952538810440474 m。
- P0 到 P8 路线时长：113.361 s。

保留的 FC Metric 3.10 正式值：

```text
E3.10_selected=2.261625026799532%
```

其中 `E_pos=0.727682193133525%`，`E_vel_selected=2.261625026799532%`，`E_yaw=0.28562253585743336%`。同日重复 FC 运行均 settled，但数值会有波动；以 `evidence/expected_fc_3_10/` 中保留值作为正式参考。

详细证据见 [VALIDATION.md](VALIDATION.md)。

## 8. 常见问题

- `f250_start_to_p0_hover.sh` 提示已有运行：先执行 `f250_stop_all.sh`，再重试。
- PX4 找不到 F250 airframe：确认 `scripts/install_px4_overlay.sh --dry-run` 输出路径正确，并检查 PX4 airframes `CMakeLists.txt` 是否包含 `10020_gazebo-classic_f250`。
- Gazebo 找不到模型：确认 overlay 中 `models/f250` 已复制到 PX4 Gazebo Classic models 目录，并重新 source/重启仿真。
- `catkin_make` 缺包：确认 ROS Noetic、MAVROS、EGO-Planner 依赖和 `quadrotor_msgs` 已 source 或已安装。
- 脚本找不到 `CURRENT_STATUS`：先运行 P0 hover，或检查 `RUN_ROOT` 是否仍为当前仓库下 `runs/f250_human_scripts`。
- 绘图结果不应使用历史 PNG 当几何真值；应由 `data/map_authority/...` 和当前轨迹 CSV 生成。
