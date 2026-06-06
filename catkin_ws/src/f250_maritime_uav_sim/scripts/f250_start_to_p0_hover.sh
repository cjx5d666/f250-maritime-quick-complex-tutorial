#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
# shellcheck source=/dev/null
source "${SCRIPT_DIR}/f250_paths.sh"
PROJECT_ROOT="$(f250_resolve_project_root "${SCRIPT_DIR}")"
WS="${PROJECT_ROOT}/catkin_ws"
PKG="$(f250_resolve_package_root "${SCRIPT_DIR}" "${PROJECT_ROOT}")"
START_SCRIPT="${PKG}/scripts/start_maritime_sim.sh"
STOP_SCRIPT="${PKG}/scripts/f250_stop_all.sh"
LAYOUT_SCRIPT="${PKG}/scripts/f250_layout_windows.py"
SCENE_LEVEL_FIXED="level_m_gps_assets_quick_complex"
SCENE_CONFIG_FIXED="${PKG}/config/scenes/${SCENE_LEVEL_FIXED}.yaml"
WORLD_FIXED="${PKG}/worlds/maritime_${SCENE_LEVEL_FIXED}.world"
MAP_AUTHORITY="${MAP_AUTHORITY:-${PROJECT_ROOT}/data/map_authority/p0_p8_hard_requirement_20260530}"
R4H_BASELINE="${R4H_BASELINE:-${PROJECT_ROOT}/evidence/expected_route}"
R4H_METRICS="${R4H_METRICS:-${PROJECT_ROOT}/evidence/expected_route/zyaw_metrics}"

usage() {
  cat <<EOF
Usage:
  ${0} [--dry-run]
  ${0} --help

Starts the F250 quick-complex stack and holds at P0. The waypoint sequence is
left paused; a later script can publish /maritime/demo/start_waypoints to run
P0-P8.

Fixed task inputs:
  vehicle: f250
  scene: ${SCENE_LEVEL_FIXED}
  perception: lidar
  dynamic obstacles: auto
  hover target: P0 = (55.0, 16.0, 10.0), yaw 0.469929
  EGO defaults: F250 R4_H

Useful environment overrides:
  RUN_ROOT=...                  default: ${PROJECT_ROOT}/runs/f250_human_scripts
  RUN_LABEL=...                 default: f250_p0_hover_<timestamp>
  SCREEN_NAME=...               default: f250_p0_hover_<timestamp>
  ENABLE_RVIZ=true|false        default: true
  PX4_GUI=true|false            default: true
  F250_PX4_ROOT=...             override PX4-Autopilot root
  DISPLAY=:0                    default: current DISPLAY or :0
  F250_PRESTOP=true             run f250_stop_all.sh before starting
  F250_ALLOW_EXISTING_RUNTIME=true
  F250_ALLOW_RUN_DIR_REUSE=true
  F250_PROJECT_ROOT=...         override project root detected from script path
  MAP_AUTHORITY=...             override authoritative map directory

Stop:
  ${STOP_SCRIPT}

Release route later:
  ${PKG}/scripts/start_demo_waypoints.sh
EOF
}

DRY_RUN="${F250_DRY_RUN:-false}"
while [ "$#" -gt 0 ]; do
  case "$1" in
    --help|-h)
      usage
      exit 0
      ;;
    --dry-run)
      DRY_RUN="true"
      shift
      ;;
    *)
      echo "Unknown argument: $1" >&2
      echo "Run ${0} --help" >&2
      exit 2
      ;;
  esac
done

STAMP="$(date +%Y%m%d_%H%M%S)"
RUN_ROOT="${RUN_ROOT:-${PROJECT_ROOT}/runs/f250_human_scripts}"
RUN_LABEL="${RUN_LABEL:-f250_p0_hover_${STAMP}}"
SCREEN_NAME="${SCREEN_NAME:-f250_p0_hover_${STAMP}}"
RUN_DIR_INPUT="${RUN_DIR:-}"

fail() {
  echo "f250_start_to_p0_hover: $*" >&2
  exit 2
}

require_path() {
  [ -e "$1" ] || fail "missing required path: $1"
}

mkdir -p "${RUN_ROOT}"
RUN_ROOT="$(cd "${RUN_ROOT}" && pwd -P)"
if [ -z "${RUN_DIR_INPUT}" ]; then
  RUN_DIR="${RUN_ROOT}/${RUN_LABEL}"
else
  RUN_DIR="${RUN_DIR_INPUT}"
  case "${RUN_DIR}" in
    /*) ;;
    *) RUN_DIR="${RUN_ROOT}/${RUN_DIR}" ;;
  esac
fi
RUN_DIR_PARENT="$(dirname "${RUN_DIR}")"
mkdir -p "${RUN_DIR_PARENT}"
RUN_DIR="${RUN_DIR_PARENT}/$(basename "${RUN_DIR}")"
case "${RUN_DIR}" in
  "${RUN_ROOT}"/*) ;;
  *) fail "RUN_DIR must be inside RUN_ROOT=${RUN_ROOT}: ${RUN_DIR}" ;;
esac
LOG_DIR="${RUN_DIR}/logs"
METRIC_DIR="${RUN_DIR}/live_metric_runs"
STATUS_FILE="${RUN_DIR}/status.env"
PROVENANCE_FILE="${RUN_DIR}/provenance.txt"
PARAMS_FILE="${RUN_DIR}/params.env"
LAUNCH_ENV="${RUN_DIR}/launch_env.sh"
LAUNCH_CMD="${RUN_DIR}/launch_in_screen.sh"
START_LOG="${LOG_DIR}/start_maritime_sim.log"
PREFLIGHT_LOG="${LOG_DIR}/preflight_processes.txt"
METRIC_DISPLAY_LOG="${RUN_DIR}/realtime_metric_live.txt"

if [[ ! "${RUN_LABEL}" =~ ^[A-Za-z0-9_.-]+$ ]]; then
  fail "RUN_LABEL must use only letters, numbers, dot, underscore, or dash: ${RUN_LABEL}"
fi

if [[ ! "${SCREEN_NAME}" =~ ^[A-Za-z0-9_.-]+$ ]]; then
  fail "SCREEN_NAME must use only letters, numbers, dot, underscore, or dash: ${SCREEN_NAME}"
fi

require_path "${WS}"
require_path "${PKG}"
require_path "${START_SCRIPT}"
require_path "${STOP_SCRIPT}"
require_path "${LAYOUT_SCRIPT}"
require_path "${SCENE_CONFIG_FIXED}"
require_path "${WORLD_FIXED}"
require_path "${MAP_AUTHORITY}"
require_path "${R4H_BASELINE}"
require_path "${R4H_METRICS}"

if [ -e "${RUN_DIR}" ] && [ "${F250_ALLOW_RUN_DIR_REUSE:-false}" != "true" ]; then
  fail "run directory already exists: ${RUN_DIR}; set F250_ALLOW_RUN_DIR_REUSE=true to reuse"
fi

if [ "${DRY_RUN}" != "true" ] && ! command -v screen >/dev/null 2>&1; then
  fail "screen is required to start a detachable human-facing session"
fi

mkdir -p "${LOG_DIR}" "${METRIC_DIR}"
: >"${PREFLIGHT_LOG}"
: >"${METRIC_DISPLAY_LOG}"

export DISPLAY="${DISPLAY:-:0}"
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
export QT_X11_NO_MITSHM="${QT_X11_NO_MITSHM:-1}"
export DISABLE_ROS1_EOL_WARNINGS="${DISABLE_ROS1_EOL_WARNINGS:-1}"
export F250_PROJECT_ROOT="${PROJECT_ROOT}"
export F250_PX4_ROOT="${F250_PX4_ROOT:-}"

export MARITIME_VEHICLE="f250"
export SCENE_LEVEL="${SCENE_LEVEL_FIXED}"
export SCENE_CONFIG="${SCENE_CONFIG_FIXED}"
export WORLD="${WORLD_FIXED}"
export PERCEPTION_SOURCE="lidar"
export DYNAMIC_MODE="auto"
export LANDING_MODE="false"
export AUTO_OFFBOARD_ARM="${AUTO_OFFBOARD_ARM:-true}"
export REQUIRE_PLANNER_COMMAND_FOR_OFFBOARD="false"
export MARITIME_START_PAUSED="true"
export MARITIME_START_TOPIC="${MARITIME_START_TOPIC:-/maritime/demo/start_waypoints}"
export ENABLE_RVIZ="${ENABLE_RVIZ:-true}"
export PX4_GUI="${PX4_GUI:-true}"
export RAW_CLOUD_TOPIC="/maritime/lidar_points"
export MARITIME_ENABLE_LIDAR_DEBUG_MARKERS="${MARITIME_ENABLE_LIDAR_DEBUG_MARKERS:-true}"
export MARITIME_LIDAR_DEBUG_RAY_STRIDE="${MARITIME_LIDAR_DEBUG_RAY_STRIDE:-2}"
export MARITIME_LIDAR_DEBUG_RAY_MAX="${MARITIME_LIDAR_DEBUG_RAY_MAX:-90}"
export MARITIME_LIDAR_VIS_RANGE_M="${MARITIME_LIDAR_VIS_RANGE_M:-18.0}"

export PX4_SPAWN_X="55.0"
export PX4_SPAWN_Y="16.0"
export PX4_SPAWN_Z="4.82"
export PX4_SPAWN_YAW="0.469929"
export HOVER_X="55.0"
export HOVER_Y="16.0"
export HOVER_Z="10.0"
export HOVER_YAW="0.469929"

export MAP_SIZE_X="760.0"
export MAP_SIZE_Y="320.0"
export MAP_SIZE_Z="18.0"
export EGO_FEASIBILITY_TOLERANCE="0.0"
export EGO_GRID_MAP_RESOLUTION="0.35"
export EGO_MAX_VEL="3.55"
export EGO_MAX_ACC="4.90"
export EGO_MAX_JERK="6.3"
export EGO_CONTROL_POINTS_DISTANCE="0.35"
export EGO_PLANNING_HORIZON="15.0"
export EGO_LOCAL_UPDATE_RANGE_X="18.0"
export EGO_LOCAL_UPDATE_RANGE_Y="18.0"
export EGO_LOCAL_UPDATE_RANGE_Z="9.0"
export EGO_OBSTACLES_INFLATION="0.50"
export EGO_COLLISION_DIST0="1.25"
export EGO_LAMBDA_SMOOTH="1.40"
export EGO_LAMBDA_COLLISION="6.0"
export EGO_LAMBDA_FEASIBILITY="0.15"
export EGO_LAMBDA_FITNESS="1.35"

export MARITIME_ENABLE_METRIC_MONITOR="${MARITIME_ENABLE_METRIC_MONITOR:-true}"
export MARITIME_METRIC_OUTPUT_DIR="${METRIC_DIR}"
export MARITIME_METRIC_RUN_LABEL="${RUN_LABEL}"
export MARITIME_METRIC_DISPLAY_LOG="${METRIC_DISPLAY_LOG}"

runtime_patterns=(
  "[r]oscore"
  "[r]osmaster"
  "[r]osout"
  "[r]oslaunch f250_maritime_uav_sim maritime_visual_acceptance.launch"
  "[r]oslaunch f250_maritime_uav_sim maritime_px4_sitl.launch"
  "[r]oslaunch f250_maritime_uav_sim maritime_ego_planner.launch"
  "[g]zclient"
  "[g]zserver"
  "[r]viz .*maritime_visual_acceptance.rviz"
  "[p]x4 .*/build/px4_sitl_default/bin/px4"
  "[m]avros_node"
  "[g]azebo_truth_to_mavros_vision.py"
  "[p]osition_cmd_to_mavros_setpoint.py"
  "[m]aritime_dynamic_obstacles.py"
  "[m]aritime_laser_scan_adapter.py"
  "[m]aritime_sensor_cloud_adapter.py"
  "[m]aritime_gazebo_cloud.py"
  "[m]aritime_cloud_adapter.py"
  "[m]aritime_goal_sequence.py"
  "[m]aritime_metric_monitor.py"
  "[m]aritime_scene_markers.py"
  "[m]aritime_flight_path.py"
  "[e]go_planner_node"
  "[t]raj_server"
  "[w]aypoint_generator"
)

screen_session_exists() {
  screen -ls 2>/dev/null | awk '{print $1}' | grep -Eq "(^|[.])${SCREEN_NAME}([[:space:]]|$)"
}

runtime_busy() {
  local busy=0
  : >"${PREFLIGHT_LOG}"
  for pat in "${runtime_patterns[@]}"; do
    if pgrep -af "${pat}" >>"${PREFLIGHT_LOG}" 2>/dev/null; then
      busy=1
    fi
  done
  if command -v screen >/dev/null 2>&1; then
    screen -ls 2>/dev/null | awk '{print $1}' | grep -E '(^|[.])f250_(p0_hover|human|metrics|terminal_showcase)' >>"${PREFLIGHT_LOG}" || true
    if [ -s "${PREFLIGHT_LOG}" ]; then
      busy=1
    fi
  fi
  [ "${busy}" -ne 0 ]
}

write_status() {
  local state="$1"
  {
    echo "state=${state}"
    echo "updated_at=$(date -Is)"
    echo "run_dir=${RUN_DIR}"
    echo "screen_name=${SCREEN_NAME}"
    echo "start_log=${START_LOG}"
    echo "metric_output_dir=${METRIC_DIR}"
    echo "metric_display_log=${METRIC_DISPLAY_LOG}"
    echo "scene_config=${SCENE_CONFIG}"
    echo "world=${WORLD}"
    echo "vehicle=${MARITIME_VEHICLE}"
    echo "start_paused=${MARITIME_START_PAUSED}"
    echo "hover_target=${HOVER_X},${HOVER_Y},${HOVER_Z},${HOVER_YAW}"
    echo "stop_script=${STOP_SCRIPT}"
  } >"${STATUS_FILE}"
}

write_export() {
  printf 'export %s=%q\n' "$1" "$2" >>"${LAUNCH_ENV}"
}

write_snapshots() {
  {
    echo "created_at=$(date -Is)"
    echo "project_root=${PROJECT_ROOT}"
    echo "workspace=${WS}"
    echo "package=${PKG}"
    echo "start_script=${START_SCRIPT}"
    echo "stop_script=${STOP_SCRIPT}"
    echo "map_authority=${MAP_AUTHORITY}"
    echo "r4h_baseline=${R4H_BASELINE}"
    echo "r4h_metrics=${R4H_METRICS}"
    echo "run_dir=${RUN_DIR}"
    echo "screen_name=${SCREEN_NAME}"
    echo "host=$(hostname)"
    echo "user=$(id -un)"
  } >"${PROVENANCE_FILE}"

  {
    echo "MARITIME_VEHICLE=${MARITIME_VEHICLE}"
    echo "SCENE_LEVEL=${SCENE_LEVEL}"
    echo "SCENE_CONFIG=${SCENE_CONFIG}"
    echo "WORLD=${WORLD}"
    echo "PERCEPTION_SOURCE=${PERCEPTION_SOURCE}"
    echo "DYNAMIC_MODE=${DYNAMIC_MODE}"
    echo "LANDING_MODE=${LANDING_MODE}"
    echo "MARITIME_START_PAUSED=${MARITIME_START_PAUSED}"
    echo "MARITIME_START_TOPIC=${MARITIME_START_TOPIC}"
    echo "AUTO_OFFBOARD_ARM=${AUTO_OFFBOARD_ARM}"
    echo "REQUIRE_PLANNER_COMMAND_FOR_OFFBOARD=${REQUIRE_PLANNER_COMMAND_FOR_OFFBOARD}"
    echo "ENABLE_RVIZ=${ENABLE_RVIZ}"
    echo "PX4_GUI=${PX4_GUI}"
    echo "F250_PROJECT_ROOT=${F250_PROJECT_ROOT}"
    echo "F250_PX4_ROOT=${F250_PX4_ROOT}"
    echo "RAW_CLOUD_TOPIC=${RAW_CLOUD_TOPIC}"
    echo "MARITIME_ENABLE_LIDAR_DEBUG_MARKERS=${MARITIME_ENABLE_LIDAR_DEBUG_MARKERS}"
    echo "MARITIME_LIDAR_DEBUG_RAY_STRIDE=${MARITIME_LIDAR_DEBUG_RAY_STRIDE}"
    echo "MARITIME_LIDAR_DEBUG_RAY_MAX=${MARITIME_LIDAR_DEBUG_RAY_MAX}"
    echo "MARITIME_LIDAR_VIS_RANGE_M=${MARITIME_LIDAR_VIS_RANGE_M}"
    echo "PX4_SPAWN_X=${PX4_SPAWN_X}"
    echo "PX4_SPAWN_Y=${PX4_SPAWN_Y}"
    echo "PX4_SPAWN_Z=${PX4_SPAWN_Z}"
    echo "PX4_SPAWN_YAW=${PX4_SPAWN_YAW}"
    echo "HOVER_X=${HOVER_X}"
    echo "HOVER_Y=${HOVER_Y}"
    echo "HOVER_Z=${HOVER_Z}"
    echo "HOVER_YAW=${HOVER_YAW}"
    echo "MAP_SIZE_X=${MAP_SIZE_X}"
    echo "MAP_SIZE_Y=${MAP_SIZE_Y}"
    echo "MAP_SIZE_Z=${MAP_SIZE_Z}"
    echo "EGO_FEASIBILITY_TOLERANCE=${EGO_FEASIBILITY_TOLERANCE}"
    echo "EGO_GRID_MAP_RESOLUTION=${EGO_GRID_MAP_RESOLUTION}"
    echo "EGO_MAX_VEL=${EGO_MAX_VEL}"
    echo "EGO_MAX_ACC=${EGO_MAX_ACC}"
    echo "EGO_MAX_JERK=${EGO_MAX_JERK}"
    echo "EGO_CONTROL_POINTS_DISTANCE=${EGO_CONTROL_POINTS_DISTANCE}"
    echo "EGO_PLANNING_HORIZON=${EGO_PLANNING_HORIZON}"
    echo "EGO_LOCAL_UPDATE_RANGE_X=${EGO_LOCAL_UPDATE_RANGE_X}"
    echo "EGO_LOCAL_UPDATE_RANGE_Y=${EGO_LOCAL_UPDATE_RANGE_Y}"
    echo "EGO_LOCAL_UPDATE_RANGE_Z=${EGO_LOCAL_UPDATE_RANGE_Z}"
    echo "EGO_OBSTACLES_INFLATION=${EGO_OBSTACLES_INFLATION}"
    echo "EGO_COLLISION_DIST0=${EGO_COLLISION_DIST0}"
    echo "EGO_LAMBDA_SMOOTH=${EGO_LAMBDA_SMOOTH}"
    echo "EGO_LAMBDA_COLLISION=${EGO_LAMBDA_COLLISION}"
    echo "EGO_LAMBDA_FEASIBILITY=${EGO_LAMBDA_FEASIBILITY}"
    echo "EGO_LAMBDA_FITNESS=${EGO_LAMBDA_FITNESS}"
    echo "MARITIME_ENABLE_METRIC_MONITOR=${MARITIME_ENABLE_METRIC_MONITOR}"
    echo "MARITIME_METRIC_OUTPUT_DIR=${MARITIME_METRIC_OUTPUT_DIR}"
    echo "MARITIME_METRIC_RUN_LABEL=${MARITIME_METRIC_RUN_LABEL}"
    echo "MARITIME_METRIC_DISPLAY_LOG=${MARITIME_METRIC_DISPLAY_LOG}"
  } >"${PARAMS_FILE}"

  : >"${LAUNCH_ENV}"
  for name in \
    DISPLAY LIBGL_ALWAYS_SOFTWARE QT_X11_NO_MITSHM DISABLE_ROS1_EOL_WARNINGS \
    F250_PROJECT_ROOT F250_PX4_ROOT \
    MARITIME_VEHICLE SCENE_LEVEL SCENE_CONFIG WORLD PERCEPTION_SOURCE DYNAMIC_MODE \
    LANDING_MODE AUTO_OFFBOARD_ARM REQUIRE_PLANNER_COMMAND_FOR_OFFBOARD \
    MARITIME_START_PAUSED MARITIME_START_TOPIC ENABLE_RVIZ PX4_GUI RAW_CLOUD_TOPIC \
    MARITIME_ENABLE_LIDAR_DEBUG_MARKERS MARITIME_LIDAR_DEBUG_RAY_STRIDE \
    MARITIME_LIDAR_DEBUG_RAY_MAX MARITIME_LIDAR_VIS_RANGE_M \
    PX4_SPAWN_X PX4_SPAWN_Y PX4_SPAWN_Z PX4_SPAWN_YAW HOVER_X HOVER_Y HOVER_Z HOVER_YAW \
    MAP_SIZE_X MAP_SIZE_Y MAP_SIZE_Z EGO_FEASIBILITY_TOLERANCE EGO_GRID_MAP_RESOLUTION \
    EGO_MAX_VEL EGO_MAX_ACC EGO_MAX_JERK EGO_CONTROL_POINTS_DISTANCE EGO_PLANNING_HORIZON \
    EGO_LOCAL_UPDATE_RANGE_X EGO_LOCAL_UPDATE_RANGE_Y EGO_LOCAL_UPDATE_RANGE_Z \
    EGO_OBSTACLES_INFLATION EGO_COLLISION_DIST0 EGO_LAMBDA_SMOOTH EGO_LAMBDA_COLLISION \
    EGO_LAMBDA_FEASIBILITY EGO_LAMBDA_FITNESS MARITIME_ENABLE_METRIC_MONITOR \
    MARITIME_METRIC_OUTPUT_DIR MARITIME_METRIC_RUN_LABEL MARITIME_METRIC_DISPLAY_LOG
  do
    write_export "${name}" "${!name}"
  done
  chmod 0644 "${LAUNCH_ENV}"

  cat >"${LAUNCH_CMD}" <<EOF
#!/usr/bin/env bash
set -euo pipefail
source "${LAUNCH_ENV}"
exec > >(tee -a "${START_LOG}") 2>&1
echo "[f250-p0-hover] started at \$(date -Is)"
echo "[f250-p0-hover] run_dir=${RUN_DIR}"
echo "[f250-p0-hover] screen_name=${SCREEN_NAME}"
echo "[f250-p0-hover] stop=${STOP_SCRIPT}"
echo "[f250-p0-hover] route release=${PKG}/scripts/start_demo_waypoints.sh"
exec "${START_SCRIPT}" quick-complex
EOF
  chmod 0755 "${LAUNCH_CMD}"
}

write_snapshots
write_status "prepared"

if [ "${DRY_RUN}" = "true" ]; then
  write_status "prepared_dry_run"
  cat <<EOF
F250 P0 hover dry run.
Run: ${RUN_LABEL}
Results: ${RUN_DIR}
Launch plan: ${LAUNCH_CMD}
Status: ${STATUS_FILE}
EOF
  exit 0
fi

if [ "${F250_PRESTOP:-false}" = "true" ]; then
  "${STOP_SCRIPT}"
elif runtime_busy && [ "${F250_ALLOW_EXISTING_RUNTIME:-false}" != "true" ]; then
  write_status "blocked_existing_runtime"
  echo "Existing maritime runtime was detected. Details: ${PREFLIGHT_LOG}" >&2
  echo "Run ${STOP_SCRIPT} first, or set F250_PRESTOP=true / F250_ALLOW_EXISTING_RUNTIME=true." >&2
  exit 2
fi

if screen_session_exists; then
  write_status "blocked_existing_screen"
  fail "screen session already exists: ${SCREEN_NAME}"
fi

screen -dmS "${SCREEN_NAME}" "${LAUNCH_CMD}"
sleep "${F250_SCREEN_CONFIRM_SEC:-2}"

if ! screen_session_exists; then
  write_status "screen_exited_early"
  echo "screen session exited early. Check ${START_LOG}" >&2
  exit 2
fi

write_status "screen_started"
ln -sfn "${RUN_DIR}" "${RUN_ROOT}/current"

cat <<EOF
F250 P0 hover is starting.
Run: ${RUN_LABEL}
View: RViz main, Gazebo support
Results: ${RUN_DIR}
Startup log: ${START_LOG}
Next:
  Route: ${PKG}/scripts/f250_run_p0_p8_route.sh
  FC 3.10: ${PKG}/scripts/f250_run_fc_3_10_steady_state.sh
  Stop: ${STOP_SCRIPT}
EOF
if [ "${F250_AUTO_LAYOUT:-true}" = "true" ]; then
  (DISPLAY="${DISPLAY:-:0}" "${LAYOUT_SCRIPT}" --kind visual --wait-sec "${F250_LAYOUT_WAIT_SEC:-8}" >>"${RUN_DIR}/logs/window_layout.log" 2>&1 || true) &
fi
