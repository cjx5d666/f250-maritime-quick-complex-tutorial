# F250 Quick-Complex Maritime Tutorial

This package is a portable tutorial bundle for replaying the F250
quick-complex maritime P0-P8 route and the independent FC Metric 3.10
steady-state check on a compatible Linux PX4/ROS system. It contains the ROS
package subset, PX4 overlay files, authoritative map inputs, retained expected
evidence, and a static package checker.

The route geometry, static obstacles, and map authority are fixed to the
2026-05-30 P0-P8 hard-requirement package. PNG files are review artifacts only;
CSV, JSON, YAML, and SDF inputs are the source of truth.

## Package Contents Checklist

- ROS package: `catkin_ws/src/f250_maritime_uav_sim`
- PX4 overlay: `px4_overlay/ROMFS/.../10020_gazebo-classic_f250` and
  `px4_overlay/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/f250`
- Authoritative map files:
  `data/map_authority/p0_p8_hard_requirement_20260530`
- Expected route evidence: `evidence/expected_route`
- Expected FC 3.10 evidence: `evidence/expected_fc_3_10`
- Static check script: `scripts/check_package.sh`

## Assumptions And Prerequisites

This bundle is for a same-system Linux rebuild where the simulation stack is
already installed. It does not vendor PX4, ROS, MAVROS, EGO-Planner, Gazebo, or
GPU/desktop dependencies.

Required environment:

- ROS Noetic and catkin tools available on the target Linux system.
- Gazebo Classic and PX4 SITL already available.
- A compatible PX4-Autopilot tree supplied by the user through
  `F250_PX4_ROOT`.
- MAVROS and the EGO-Planner runtime dependencies already installed on the same
  Linux system.
- `screen` available for the human-facing launch scripts, unless the scripts are
  adapted for a local foreground workflow.
- A graphical desktop/display if using Gazebo GUI or RViz.

## Install And Unpack

Choose a project location on the target Linux system, then unpack the archive:

```bash
mkdir -p ~/f250_tutorial
tar -xzf f250_quick_complex_tutorial_20260605.tar.gz -C ~/f250_tutorial
cd ~/f250_tutorial/f250_quick_complex_tutorial
```

Run the static package check before installing overlays or starting anything:

```bash
scripts/check_package.sh
```

The check verifies package structure, script syntax, Python compileability,
broken symlinks, cache directories, legacy vehicle markers, and local VM path
leaks.

## PX4 Overlay

Set `F250_PX4_ROOT` to the compatible PX4-Autopilot tree, then copy the F250
airframe and Gazebo Classic model overlay:

```bash
export F250_PROJECT_ROOT="$(pwd -P)"
export F250_PX4_ROOT="/path/to/PX4-Autopilot"

test -d "$F250_PX4_ROOT" || { echo "missing F250_PX4_ROOT"; exit 2; }

cp -av \
  px4_overlay/ROMFS/px4fmu_common/init.d-posix/airframes/10020_gazebo-classic_f250 \
  "$F250_PX4_ROOT/ROMFS/px4fmu_common/init.d-posix/airframes/"

mkdir -p \
  "$F250_PX4_ROOT/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models"
cp -av \
  px4_overlay/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/f250 \
  "$F250_PX4_ROOT/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/"
```

If your PX4 tree uses a different Gazebo Classic model path, copy the `f250`
model directory to the equivalent SITL Gazebo Classic models directory in that
tree.

## Catkin Build

Build the included catkin workspace from this package root:

```bash
cd "$F250_PROJECT_ROOT/catkin_ws"
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

If your system uses `catkin build`, the equivalent is:

```bash
cd "$F250_PROJECT_ROOT/catkin_ws"
source /opt/ros/noetic/setup.bash
catkin build
source devel/setup.bash
```

## Environment Setup

Copy `env.example` to a local environment file and edit the PX4 path:

```bash
cd "$F250_PROJECT_ROOT"
cp env.example env.local
```

Set these explicitly in `env.local`:

```bash
export F250_PROJECT_ROOT="/path/to/f250_quick_complex_tutorial"
export F250_PX4_ROOT="/path/to/PX4-Autopilot"
```

Then source the environment and workspace setup:

```bash
source env.local
source /opt/ros/noetic/setup.bash
source "$F250_PROJECT_ROOT/catkin_ws/devel/setup.bash"
```

Useful defaults from `env.example`:

- `MAP_AUTHORITY="$F250_PROJECT_ROOT/data/map_authority/p0_p8_hard_requirement_20260530"`
- `RUN_ROOT="$F250_PROJECT_ROOT/runs/f250_human_scripts"`
- `CURRENT_STATUS="$RUN_ROOT/current/status.env"`
- `ENABLE_RVIZ=true`
- `PX4_GUI=true`

## Run Sequence

Use the scripts under
`catkin_ws/src/f250_maritime_uav_sim/scripts`. The route acceptance and FC
Metric 3.10 checks are separate. Stop the first stack before starting the FC
run so the FC run starts from a clean P0 hover.

1. Start and hold at P0 for route readiness:

```bash
cd "$F250_PROJECT_ROOT"
source env.local
source /opt/ros/noetic/setup.bash
source "$F250_PROJECT_ROOT/catkin_ws/devel/setup.bash"

catkin_ws/src/f250_maritime_uav_sim/scripts/f250_start_to_p0_hover.sh
```

2. Release and record the P0-P8 route:

```bash
catkin_ws/src/f250_maritime_uav_sim/scripts/f250_run_p0_p8_route.sh
```

3. Generate the final planned-vs-flown plot for the latest route run:

```bash
catkin_ws/src/f250_maritime_uav_sim/scripts/f250_plot_latest_run.sh
```

4. Stop the route simulation stack:

```bash
catkin_ws/src/f250_maritime_uav_sim/scripts/f250_stop_all.sh
```

5. Start and hold at P0 again for the FC-only test:

```bash
catkin_ws/src/f250_maritime_uav_sim/scripts/f250_start_to_p0_hover.sh
```

6. Run FC Metric 3.10 steady-state evidence:

```bash
catkin_ws/src/f250_maritime_uav_sim/scripts/f250_run_fc_3_10_steady_state.sh
```

7. Final stop:

```bash
catkin_ws/src/f250_maritime_uav_sim/scripts/f250_stop_all.sh
```

## Expected Outputs And Acceptance

Runs are written under:

```text
$RUN_ROOT
```

With the default `env.example`, this resolves to:

```text
$F250_PROJECT_ROOT/runs/f250_human_scripts
```

The current run symlink/status is:

```text
$CURRENT_STATUS
```

### P0 Readiness

The P0 script should create a run directory, write `status.env`, start the
F250 stack paused at P0, and report a hover target of:

```text
55.0,16.0,10.0,0.469929
```

The route script expects that current P0 status to identify vehicle `f250`,
the fixed quick-complex scene, and the fixed world.

### Route Acceptance

Expected retained route family: `R4_H`.

The selected retained route run is `lidar_lidar_R4_H_r2`. Representative
expected values:

- Route progress: P8 reached, displayed as P8/8.
- Static safety: SAFE, with zero static geometry entries, zero static cloud
  entries, and no static collision.
- Selected-run static clearance: 0.8086800027841847 m.
- Keypoint error over P1-P7: mean 0.238261136879 m, max
  0.458080235519 m.
- Endpoint error at P8: 0.5952538810440474 m.
- Route task duration from P0 to P8: 113.361 s for the selected run.

Route acceptance excludes FC Metric 3.10 and yaw pass/fail. Dynamic boat
clearance is telemetry only for this route decision.

### Plot Acceptance

The plot script should write these files in the selected route run directory:

- `latest_planned_vs_flown.png`
- `latest_plot_summary.json`
- `latest_plot_points.csv`
- `latest_plot_summary.md`

The plot must be generated from authoritative CSV/JSON/map inputs on the same
axes. Do not use a cached PNG as geometry truth or as a trajectory background.
The retained package includes `evidence/expected_route/f250_historical_planned_vs_flown.png`
as a review artifact only.

### FC Metric 3.10 Acceptance

Expected final retained FC formal value:

```text
E3.10_selected=2.261625026799532%
```

The retained FC evidence has:

- All 40 formal metric windows settled.
- `E_pos=0.727682193133525%`.
- `E_vel_selected=2.261625026799532%`.
- `E_yaw=0.28562253585743336%`.
- Selected velocity speed: 2.0 m/s.
- Geometry audit conclusion: SAFE for the FC 3.10 A/B and decagon geometry.

Same-day duplicate FC re-runs settled, but the values varied. Those duplicate
re-runs were not used as the final pass evidence; use the retained value above
as the formal expected result.

## Cleanup And Troubleshooting

Use the stop script whenever a run fails, a launch blocks on existing runtime,
or you need a clean restart:

```bash
catkin_ws/src/f250_maritime_uav_sim/scripts/f250_stop_all.sh
```

The stop script targets the F250 maritime PX4/Gazebo/ROS/RViz/EGO-Planner
runtime chain and F250 script screen sessions. It preserves run/result
directories.

Troubleshooting basics:

- If `f250_start_to_p0_hover.sh` reports existing runtime, run
  `f250_stop_all.sh`, then retry.
- If route or FC scripts cannot find ROS topics, confirm the P0 hover stack is
  still active and `CURRENT_STATUS` points to the current P0 `status.env`.
- If catkin packages are missing, re-source `/opt/ros/noetic/setup.bash` and
  `$F250_PROJECT_ROOT/catkin_ws/devel/setup.bash`.
- If PX4 cannot find the F250 airframe or model, re-check the PX4 overlay copy
  and `F250_PX4_ROOT`.
- Keep concise successful run outputs, structured JSON/CSV summaries, and final
  review plots. Do not retain high-volume logs, failed tuning sweeps, cache
  directories, or temporary runtime log folders in the tutorial package.
