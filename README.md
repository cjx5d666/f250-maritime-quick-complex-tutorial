# F250 海上复杂场景避障复现实验教程

本仓库提供一个可复现的 F250 无人机海上复杂场景避障教程包。教程目标是让同学在已经配置好 PX4 / ROS / Gazebo / EGO-Planner 的 Linux 环境中，复现固定的 P0-P8 航线任务、生成规划航线与实际飞行轨迹对比图，并查看独立的 FC 指标 3.10 结果。

> 注意：本仓库不是虚拟机镜像，也不包含完整 PX4、ROS、Gazebo 或 EGO-Planner 安装包。它提供的是复现实验所需的脚本、F250 模型覆盖文件、地图/场景配置、权威航点与保留验证证据。

## 仓库文件

| 文件 | 用途 |
| --- | --- |
| `f250_quick_complex_tutorial_20260605.tar.gz` | 教程主压缩包 |
| `SHA256SUMS.txt` | 压缩包 SHA256 校验值 |
| `f250_tutorial_validation_manifest_20260605.md` | 已验证结果与注意事项 |
| `README.md` | 本中文操作说明 |

压缩包 SHA256 应为：

```text
FDB915EC40A8662E157DAF8548AE3473B7DB5BBB7C325E5F3015E98DA2AA788B
```

## 已验证结果

该教程包来自已经通过检查的 F250 版本：

- P0 悬停检查通过。
- P0-P8 航线完成到 `P8/8`。
- 静态障碍物安全，未发生静态碰撞。
- 航线终点误差约 `0.605 m`。
- FC 指标 3.10 保留正式通过结果：`E3.10_selected = 2.261625026799532%`。

FC 3.10 是独立飞控稳态指标，不作为 P0-P8 避障航线是否通过的判定条件。

## 环境前提

请先准备一台 Linux 机器，推荐使用与实验环境相近的 Ubuntu / ROS Noetic 环境。需要已经具备：

- ROS Noetic 和 catkin 工作区工具。
- Gazebo Classic。
- PX4 SITL 源码树。
- MAVROS。
- EGO-Planner 运行依赖。
- `screen` 命令。
- 图形桌面环境，用于显示 Gazebo 和 RViz。

如果你的机器还没有完整仿真环境，请先完成 PX4、ROS、Gazebo、MAVROS 和 EGO-Planner 的基础安装，再使用本教程包。

## 下载与校验

可以直接在 GitHub 页面点击压缩包下载，也可以克隆仓库：

```bash
git clone https://github.com/cjx5d666/f250-maritime-quick-complex-tutorial.git
cd f250-maritime-quick-complex-tutorial
```

校验压缩包：

```bash
sha256sum -c SHA256SUMS.txt
```

如果输出包含 `OK`，说明下载的压缩包未损坏。

## 解压教程包

选择一个工作目录，例如：

```bash
mkdir -p ~/f250_tutorial
tar -xzf f250_quick_complex_tutorial_20260605.tar.gz -C ~/f250_tutorial
cd ~/f250_tutorial/f250_quick_complex_tutorial
```

先运行静态检查：

```bash
scripts/check_package.sh
```

该脚本会检查包结构、脚本语法、Python 文件、符号链接、缓存目录、旧车辆标记和本地路径泄漏。

## 配置 PX4 路径

教程脚本需要知道你本机 PX4-Autopilot 源码树的位置。假设你的 PX4 源码在：

```bash
~/PX4-Autopilot-v1.16.0-src-main
```

则设置：

```bash
export F250_PX4_ROOT=~/PX4-Autopilot-v1.16.0-src-main
```

如果你的 PX4 路径不同，请改成自己的实际路径。

## 安装覆盖文件与构建

进入教程包根目录后，按照包内说明安装 ROS 包与 PX4 覆盖文件。通常需要把：

- `catkin_ws/src/f250_maritime_uav_sim`
- `px4_overlay/ROMFS/.../10020_gazebo-classic_f250`
- `px4_overlay/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/f250`

放到对应的 catkin 工作区和 PX4 源码树中。

完成后重新构建 catkin 工作区，并确保 ROS 环境已加载：

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
```

如果你的工作区路径不是 `~/catkin_ws`，请改成自己的路径。

## 第一步：启动到 P0 悬停

进入脚本目录：

```bash
cd ~/catkin_ws/src/f250_maritime_uav_sim/scripts
```

启动仿真并飞到 P0 悬停：

```bash
./f250_start_to_p0_hover.sh
```

成功后应能看到 Gazebo / RViz 相关窗口，并且无人机进入 P0 附近的悬停状态。

## 第二步：运行 FC 指标 3.10

在 P0 悬停稳定后运行：

```bash
./f250_run_fc_3_10_steady_state.sh
```

脚本会执行独立的飞控稳态指标检查。该指标用于查看飞控跟踪性能，不用于判定 P0-P8 避障航线是否通过。

## 第三步：运行 P0-P8 航线

继续运行固定 P0-P8 海上复杂场景航线：

```bash
./f250_run_p0_p8_route.sh
```

期望结果是完成到 `P8/8`，并保持静态障碍物安全。

## 第四步：生成轨迹图

航线运行完成后生成最新的计划航线与实际飞行轨迹图：

```bash
./f250_plot_latest_run.sh
```

最终图应使用包内权威地图、规划航点和实际轨迹数据绘制。PNG 只是展示结果，CSV / JSON / YAML / SDF 才是几何与场景来源。

## 第五步：停止仿真

实验结束后停止相关进程：

```bash
./f250_stop_all.sh
```

建议每次实验结束后都执行停止脚本，避免残留 ROS、Gazebo、PX4 或 MAVROS 进程影响下一次运行。

## 常见注意事项

- 本教程复现的是同一任务配置、同一场景、同一地图、同一 F250 模型和同一脚本流程。
- 不保证不同机器上的 Gazebo / RViz 画面像素完全一致。
- FC 3.10 多次运行可能有数值波动，正式记录以验证清单中的保留通过结果为准。
- P0-P8 航线验收重点是完成 P8 和静态障碍物安全。
- 如果脚本找不到 PX4，请先检查 `F250_PX4_ROOT` 是否设置正确。
- 如果看不到 Gazebo 或 RViz，请检查图形桌面、显示变量和相关依赖。

## 验证清单

更多已验证结果、保留证据和 FC 3.10 重跑说明见：

```text
f250_tutorial_validation_manifest_20260605.md
```

建议同学先读完本 README，再查看验证清单中的结果数值和注意事项。
