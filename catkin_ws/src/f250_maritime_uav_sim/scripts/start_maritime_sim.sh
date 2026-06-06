#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
# shellcheck source=/dev/null
source "${SCRIPT_DIR}/f250_paths.sh"
PROJECT_ROOT="$(f250_resolve_project_root "${SCRIPT_DIR}")"
WS="${PROJECT_ROOT}/catkin_ws"
PKG="$(f250_resolve_package_root "${SCRIPT_DIR}" "${PROJECT_ROOT}")"

MODE="${1:-quick-complex}"
if [ "${MODE}" = "--help" ] || [ "${MODE}" = "-h" ]; then
  cat <<'EOF'
Usage:
  ./start_maritime_sim.sh [quick-complex|quick-complex-depth|quick-complex-gazebo-cloud|quick-complex-lidar]

Default mode:
  quick-complex  F250 integrated quick-complex route with lidar perception.

Operator-facing modes:
  quick-complex  integrated quick-complex route; default perception is lidar
  quick-complex-depth  same route with depth-camera planner cloud requested for fallback comparison
  quick-complex-gazebo-cloud  same route with Gazebo-state-gated planner cloud
  quick-complex-lidar  same route with Gazebo lidar planner cloud

Stop the visible simulation:
  ./f250_stop_all.sh

Common environment overrides:
  F250_PROJECT_ROOT=...
  F250_PX4_ROOT=...
  PX4_BOOT_WAIT_SEC=12
  AUTO_OFFBOARD_ARM=true
  REQUIRE_PLANNER_COMMAND_FOR_OFFBOARD=false
EOF
  exit 0
fi

if [ ! -d "${WS}" ] || [ ! -d "${PKG}" ]; then
  echo "Missing workspace: ${WS}" >&2
  exit 2
fi
PX4_ROOT="$(f250_resolve_px4_root "${PROJECT_ROOT}")"
SITL_GAZEBO="${PX4_ROOT}/Tools/simulation/gazebo-classic/sitl_gazebo-classic"
PX4_BUILD="${PX4_ROOT}/build/px4_sitl_default"

cd "${WS}"
export ROS_DISTRO="${ROS_DISTRO:-noetic}"
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://localhost:11311}"
source /opt/ros/noetic/setup.bash
if [ -f devel/setup.bash ]; then
  source devel/setup.bash
else
  echo "Missing catkin devel setup. Run catkin_make in ${WS} first." >&2
  exit 2
fi

export F250_PX4_ROOT="${PX4_ROOT}"
export ROS_PACKAGE_PATH="${PX4_ROOT}:${SITL_GAZEBO}:${ROS_PACKAGE_PATH:-}"
export GAZEBO_MODEL_PATH="${PKG}/models:${SITL_GAZEBO}/models:${GAZEBO_MODEL_PATH:-}"
export GAZEBO_PLUGIN_PATH="/opt/ros/noetic/lib:${PX4_BUILD}/build_gazebo-classic:${GAZEBO_PLUGIN_PATH:-}"
export LD_LIBRARY_PATH="${PX4_BUILD}/build_gazebo-classic:${LD_LIBRARY_PATH:-}"
export DISPLAY="${DISPLAY:-:0}"
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
export QT_X11_NO_MITSHM="${QT_X11_NO_MITSHM:-1}"
export DISABLE_ROS1_EOL_WARNINGS="${DISABLE_ROS1_EOL_WARNINGS:-1}"
export AUTO_OFFBOARD_ARM="${AUTO_OFFBOARD_ARM:-true}"
export MARITIME_VEHICLE="f250"
export ENABLE_RVIZ="${ENABLE_RVIZ:-true}"

launch_visible() {
  local scene_level="$1"
  local perception_source="$2"
  local dynamic_mode="$3"
  local landing_mode="$4"
  local scene_config="${PKG}/config/scenes/${scene_level}.yaml"
  local world="${PKG}/worlds/maritime_${scene_level}.world"
  echo "[maritime] visible mode: ${MODE}"
  echo "[maritime] scene=${scene_level} perception=${perception_source} dynamic=${dynamic_mode} landing=${landing_mode}"
  echo "[maritime] close with: ${PKG}/scripts/f250_stop_all.sh"
  exec roslaunch f250_maritime_uav_sim maritime_visual_acceptance.launch \
    vehicle:="${MARITIME_VEHICLE}" \
    scene_level:="${scene_level}" \
    scene_config:="${scene_config}" \
    perception_source:="${perception_source}" \
    dynamic_mode:="${dynamic_mode}" \
    world:="${world}" \
    px4_gui:="${PX4_GUI:-true}" \
    auto_offboard_arm:="${AUTO_OFFBOARD_ARM}" \
    require_planner_command_for_offboard:="${REQUIRE_PLANNER_COMMAND_FOR_OFFBOARD:-true}" \
    landing_enabled:="${landing_mode}" \
    enable_rviz:="${ENABLE_RVIZ}"
}


use_quick_map() {
  export MAP_SIZE_X="${MAP_SIZE_X:-760.0}"
  export MAP_SIZE_Y="${MAP_SIZE_Y:-320.0}"
  export MAP_SIZE_Z="${MAP_SIZE_Z:-18.0}"
}

use_quick_complex_ego_map() {
  use_quick_map
  export EGO_FEASIBILITY_TOLERANCE="${EGO_FEASIBILITY_TOLERANCE:-0.0}"
  export EGO_GRID_MAP_RESOLUTION="${EGO_GRID_MAP_RESOLUTION:-0.35}"
  export EGO_MAX_VEL="${EGO_MAX_VEL:-3.55}"
  export EGO_MAX_ACC="${EGO_MAX_ACC:-4.90}"
  export EGO_MAX_JERK="${EGO_MAX_JERK:-6.3}"
  export EGO_CONTROL_POINTS_DISTANCE="${EGO_CONTROL_POINTS_DISTANCE:-0.35}"
  export EGO_PLANNING_HORIZON="${EGO_PLANNING_HORIZON:-15.0}"
  export EGO_LOCAL_UPDATE_RANGE_X="${EGO_LOCAL_UPDATE_RANGE_X:-18.0}"
  export EGO_LOCAL_UPDATE_RANGE_Y="${EGO_LOCAL_UPDATE_RANGE_Y:-18.0}"
  export EGO_LOCAL_UPDATE_RANGE_Z="${EGO_LOCAL_UPDATE_RANGE_Z:-9.0}"
  export EGO_OBSTACLES_INFLATION="${EGO_OBSTACLES_INFLATION:-0.50}"
  export EGO_COLLISION_DIST0="${EGO_COLLISION_DIST0:-1.25}"
  export EGO_LAMBDA_SMOOTH="${EGO_LAMBDA_SMOOTH:-1.40}"
  export EGO_LAMBDA_COLLISION="${EGO_LAMBDA_COLLISION:-6.0}"
  export EGO_LAMBDA_FEASIBILITY="${EGO_LAMBDA_FEASIBILITY:-0.15}"
  export EGO_LAMBDA_FITNESS="${EGO_LAMBDA_FITNESS:-1.35}"
}

use_quick_spawn() {
  export PX4_SPAWN_X="${PX4_SPAWN_X:-$1}"
  export PX4_SPAWN_Y="${PX4_SPAWN_Y:-$2}"
  export PX4_SPAWN_Z="${PX4_SPAWN_Z:-${3:-1.10}}"
  export PX4_SPAWN_YAW="${PX4_SPAWN_YAW:-${4:-0.0}}"
}

use_f250_quick_metrics() {
  local metric_root="${RUN_ROOT:-${PROJECT_ROOT}/runs/f250_human_scripts}"
  export MARITIME_ENABLE_METRIC_MONITOR="${MARITIME_ENABLE_METRIC_MONITOR:-true}"
  export MARITIME_METRIC_OUTPUT_DIR="${MARITIME_METRIC_OUTPUT_DIR:-${metric_root}/live_metric_runs}"
  export MARITIME_METRIC_RUN_LABEL="${MARITIME_METRIC_RUN_LABEL:-f250_quick_complex_$(date +%Y%m%d_%H%M%S)}"
}


case "${MODE}" in
  quick-complex|complex-quick|quick_complex|default)
    use_quick_complex_ego_map
    use_quick_spawn 55.0 16.0 4.82 0.469929
    use_f250_quick_metrics
    export MARITIME_START_PAUSED="${MARITIME_START_PAUSED:-false}"
    launch_visible "${SCENE_LEVEL:-level_m_gps_assets_quick_complex}" "${PERCEPTION_SOURCE:-lidar}" "${DYNAMIC_MODE:-auto}" "${LANDING_MODE:-false}"
    ;;
  quick-complex-depth|complex-depth|quick_complex_depth)
    use_quick_complex_ego_map
    use_quick_spawn 55.0 16.0 4.82 0.469929
    use_f250_quick_metrics
    export MARITIME_START_PAUSED="${MARITIME_START_PAUSED:-false}"
    launch_visible "${SCENE_LEVEL:-level_m_gps_assets_quick_complex}" "${PERCEPTION_SOURCE:-depth}" "${DYNAMIC_MODE:-auto}" "${LANDING_MODE:-false}"
    ;;
  quick-complex-gazebo-cloud|complex-gazebo-cloud|quick_complex_gazebo_cloud)
    use_quick_complex_ego_map
    use_quick_spawn 55.0 16.0 4.82 0.469929
    use_f250_quick_metrics
    export MARITIME_START_PAUSED="${MARITIME_START_PAUSED:-false}"
    launch_visible "${SCENE_LEVEL:-level_m_gps_assets_quick_complex}" "${PERCEPTION_SOURCE:-gazebo_cloud}" "${DYNAMIC_MODE:-auto}" "${LANDING_MODE:-false}"
    ;;
  quick-complex-lidar|complex-lidar|quick_complex_lidar)
    use_quick_complex_ego_map
    use_quick_spawn 55.0 16.0 4.82 0.469929
    use_f250_quick_metrics
    export MARITIME_START_PAUSED="${MARITIME_START_PAUSED:-false}"
    launch_visible "${SCENE_LEVEL:-level_m_gps_assets_quick_complex}" "${PERCEPTION_SOURCE:-lidar}" "${DYNAMIC_MODE:-auto}" "${LANDING_MODE:-false}"
    ;;
  *)
    echo "Unknown mode: ${MODE}" >&2
    echo "Run: ${0} --help" >&2
    exit 2
    ;;
esac
