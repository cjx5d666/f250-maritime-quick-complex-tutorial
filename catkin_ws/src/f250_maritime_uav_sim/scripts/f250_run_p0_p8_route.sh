#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
# shellcheck source=/dev/null
source "${SCRIPT_DIR}/f250_paths.sh"
PROJECT_ROOT="$(f250_resolve_project_root "${SCRIPT_DIR}")"
WS="${PROJECT_ROOT}/catkin_ws"
PKG="$(f250_resolve_package_root "${SCRIPT_DIR}" "${PROJECT_ROOT}")"
SCRIPTS="${PKG}/scripts"
SCENE_LEVEL_FIXED="level_m_gps_assets_quick_complex"
SCENE_CONFIG_FIXED="${PKG}/config/scenes/${SCENE_LEVEL_FIXED}.yaml"
WORLD_FIXED="${PKG}/worlds/maritime_${SCENE_LEVEL_FIXED}.world"
MAP_AUTHORITY="${MAP_AUTHORITY:-$(f250_first_existing_or_first "${PROJECT_ROOT}/maritime_quick_complex/map_authority/p0_p8_hard_requirement_20260530" "${PROJECT_ROOT}/data/map_authority/p0_p8_hard_requirement_20260530")}"
R4H_BASELINE="${R4H_BASELINE:-$(f250_first_existing_or_first "${PROJECT_ROOT}/maritime_quick_complex/results/f250/r4h_selected_20260530" "${PROJECT_ROOT}/evidence/expected_route")}"

RECORDER="${SCRIPTS}/f250_quick_complex_record.py"
CLEARANCE_EVALUATOR="${SCRIPTS}/maritime_clearance_evaluator.py"
METRIC_MONITOR="${SCRIPTS}/maritime_metric_monitor.py"
POSTPROCESS="${SCRIPTS}/f250_quick_complex_postprocess.py"
DISPLAY_HELPER="${SCRIPTS}/f250_route_human_summary.py"
START_DEMO="${SCRIPTS}/start_demo_waypoints.sh"

RUN_ROOT="${RUN_ROOT:-${PROJECT_ROOT}/runs/f250_human_scripts}"
DEFAULT_CURRENT_STATUS="${RUN_ROOT}/current/status.env"
CURRENT_STATUS="${CURRENT_STATUS:-}"
STAMP="$(date +%Y%m%d_%H%M%S)"
SCRIPT_PATH="$(readlink -f "$0")"

usage() {
  cat <<EOF
Usage:
  ${0} [--dry-run]
  ${0} [--foreground]
  ${0} --help

Runs the F250-only maritime_quick_complex P0-P8 route from an already active
P0 hover stack. It releases /maritime/demo/start_waypoints, records the actual
trajectory, replays route metrics offline, and prints only the current formal
human route indicators below:
  3.6 keypoint error
  3.8 planning / route success
  3.9 final target error

Fixed task inputs:
  vehicle: f250
  route/map/scene: 2026-05-30 hard-requirement P0-P8 quick-complex
  baseline/defaults: F250 R4_H
  terminal policy: no Metric 3.7 / 3.10 display, no yaw pass/fail display
  obstacle clearance: telemetry/debug only, not a current terminal metric

Default human mode:
  The route worker runs in a background screen/nohup worker and opens a
  separate metrics terminal or screen that follows route_terminal.log. The
  calling terminal only prints run/log locations. Use --foreground for the old
  blocking behavior.

Useful environment overrides:
  RUN_ROOT=...                  default: ${RUN_ROOT}
  RUN_LABEL=...                 default: f250_p0_p8_route_<timestamp>
  RUN_DIR=...                   explicit output directory under RUN_ROOT
  CURRENT_STATUS=...            default: ${DEFAULT_CURRENT_STATUS}
  ROUTE_MAX_DURATION_SEC=...    default: 360
  F250_ALLOW_RUN_DIR_REUSE=true allow reusing RUN_DIR
  F250_ROUTE_ARGS='...'         extra args passed to recorder
  F250_ROUTE_BACKGROUND=true|false
  F250_OPEN_METRICS_TERMINAL=true|false
  F250_PROJECT_ROOT=...         override project root detected from script path
  MAP_AUTHORITY=...             override authoritative map directory
  R4H_BASELINE=...              override expected route baseline directory

Dry-run:
  ${0} --dry-run
  Does not require ROS master and writes synthetic successful route outputs.
EOF
}

fail() {
  echo "f250_run_p0_p8_route: $*" >&2
  exit 2
}

env_value() {
  local key="$1"
  local file="$2"
  [ -f "${file}" ] || return 0
  awk -F= -v key="${key}" '$1 == key {sub(/^[^=]*=/, ""); print; exit}' "${file}"
}

require_path() {
  [ -e "$1" ] || fail "missing required path: $1"
}

write_status() {
  local state="$1"
  {
    echo "state=${state}"
    echo "updated_at=$(date -Is)"
    echo "run_dir=${RUN_DIR}"
    echo "run_label=${RUN_LABEL}"
    echo "vehicle=f250"
    echo "dry_run=${DRY_RUN}"
    echo "source_p0_status=${CURRENT_STATUS}"
    echo "source_p0_run_dir=${P0_RUN_DIR}"
    echo "scene_config=${SCENE_CONFIG_FIXED}"
    echo "world=${WORLD_FIXED}"
    echo "map_authority=${MAP_AUTHORITY}"
    echo "r4h_baseline=${R4H_BASELINE}"
    echo "actual_trajectory_csv=${TRAJECTORY_CSV}"
    echo "summary_json=${SUMMARY_JSON}"
    echo "metrics_json=${METRICS_JSON}"
    echo "metric_summary_json=${METRIC_SUMMARY_JSON}"
    echo "metric_waypoints_csv=${METRIC_WAYPOINTS_CSV}"
    echo "clearance_static_gate_json=${CLEARANCE_STATIC_JSON}"
    echo "clearance_dynamic_telemetry_json=${CLEARANCE_DYNAMIC_JSON}"
    echo "route_terminal_log=${ROUTE_TERMINAL_LOG}"
    echo "route_status_env=${ROUTE_STATUS_ENV}"
    echo "route_acceptance_excludes_metric_3_10=true"
    echo "route_acceptance_excludes_yaw=true"
    echo "dynamic_boat_clearance_role=telemetry_only"
  } >"${STATUS_FILE}"
}

write_params_json() {
  python3 - "${PARAMS_JSON}" "${SCENE_CONFIG_FIXED}" <<'PY'
import json
import os
import sys

path, scene_config = sys.argv[1], sys.argv[2]
payload = {
  "description": "F250 human P0-P8 route script using current R4_H defaults",
  "vehicle": "f250",
  "family_id": "R4_H",
  "scene_level": "level_m_gps_assets_quick_complex",
  "scene_config": os.path.abspath(scene_config),
  "perception_source": "lidar",
  "dynamic_mode": "auto",
  "params": {
    "map_size_x": 760.0,
    "map_size_y": 320.0,
    "map_size_z": 18.0,
    "max_vel": 3.55,
    "max_acc": 4.90,
    "max_jerk": 6.3,
    "control_points_distance": 0.35,
    "feasibility_tolerance": 0.0,
    "planning_horizon": 15.0,
    "local_update_range_x": 18.0,
    "local_update_range_y": 18.0,
    "local_update_range_z": 9.0,
    "obstacles_inflation": 0.50,
    "collision_dist0": 1.25,
    "lambda_smooth": 1.40,
    "lambda_collision": 6.0,
    "lambda_feasibility": 0.15,
    "lambda_fitness": 1.35,
    "grid_map_resolution": 0.35
  },
  "route_acceptance_policy": {
    "date": "2026-06-04",
    "route_acceptance_excludes_metric_3_10": True,
    "route_acceptance_excludes_yaw": True,
    "dynamic_boat_clearance_role": "telemetry_only"
  }
}
os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
with open(path, "w", encoding="utf-8") as handle:
    json.dump(payload, handle, indent=2, sort_keys=True)
    handle.write("\n")
PY
}

ensure_ros_available() {
  if ! command -v rostopic >/dev/null 2>&1; then
    fail "rostopic is unavailable; source ROS or use --dry-run"
  fi
  if ! rostopic list >/dev/null 2>&1; then
    fail "ROS master is unavailable; start f250_start_to_p0_hover.sh first or use --dry-run"
  fi
  if ! rostopic list | grep -qx "/mavros/local_position/odom"; then
    fail "missing /mavros/local_position/odom; P0 hover stack does not look active"
  fi
  if ! rostopic list | grep -qx "/maritime/active_goal"; then
    fail "missing /maritime/active_goal; fixed quick-complex route stack does not look active"
  fi
}

stop_pid() {
  local pid="$1"
  [ -n "${pid}" ] || return 0
  if kill -0 "${pid}" >/dev/null 2>&1; then
    kill -TERM "${pid}" >/dev/null 2>&1 || true
    local waited=0
    while kill -0 "${pid}" >/dev/null 2>&1 && [ "${waited}" -lt 30 ]; do
      sleep 0.2
      waited=$((waited + 1))
    done
    if kill -0 "${pid}" >/dev/null 2>&1; then
      kill -KILL "${pid}" >/dev/null 2>&1 || true
    fi
  fi
  wait "${pid}" >/dev/null 2>&1 || true
}

terminal_command() {
  local logfile="$1"
  local heading="${2:-F250 Route Metrics}"
  local awk_script
  awk_script='{
    line=$0
    if (line ~ /^=+/) {
      printf "\033[1;36m%s\033[0m\n", line
    } else if (line ~ /^\[[^]]+\]/) {
      printf "\033[1;33m%s\033[0m\n", line
    } else if (line ~ /result PASS/) {
      sub(/PASS/, "\033[1;32mPASS\033[0m", line)
      print line
    } else if (line ~ /result FAIL/) {
      sub(/FAIL/, "\033[1;31mFAIL\033[0m", line)
      print line
    } else {
      print line
    }
    fflush()
  }'
  printf "touch %q; tail -n +1 -F %q | awk %q" "${logfile}" "${logfile}" "${awk_script}"
}

append_terminal_over() {
  [ -n "${ROUTE_TERMINAL_LOG:-}" ] || return 0
  mkdir -p "$(dirname "${ROUTE_TERMINAL_LOG}")"
  if [ -f "${ROUTE_TERMINAL_LOG}" ] && [ "$(tail -n 1 "${ROUTE_TERMINAL_LOG}" 2>/dev/null || true)" = "OVER" ]; then
    return 0
  fi
  printf "\nOVER\n" >>"${ROUTE_TERMINAL_LOG}"
}

open_metrics_terminal() {
  local title="$1"
  local logfile="$2"
  local screen_name="$3"
  [ "${F250_OPEN_METRICS_TERMINAL:-true}" = "true" ] || return 0
  local cmd
  cmd="$(terminal_command "${logfile}" "${title}")"
  local metric_window="${SCRIPTS}/f250_metric_window.py"
  if [ -n "${DISPLAY:-}" ] && [ -f "${metric_window}" ] && python3 - <<'PY' >/dev/null 2>&1
import tkinter
PY
  then
    nohup python3 "${metric_window}" \
      --title "${title}" \
      --log "${logfile}" \
      --width "${F250_METRICS_WINDOW_WIDTH:-980}" \
      --height "${F250_METRICS_WINDOW_HEIGHT:-520}" \
      --font-size "${F250_METRICS_FONT_SIZE:-14}" \
      >/dev/null 2>&1 &
    echo "metrics_window=${title}"
  elif [ -n "${DISPLAY:-}" ] && command -v x-terminal-emulator >/dev/null 2>&1; then
    nohup x-terminal-emulator -T "${title}" -e bash -lc "${cmd}" >/dev/null 2>&1 &
    echo "metrics_terminal=${title}"
  elif [ -n "${DISPLAY:-}" ] && command -v gnome-terminal >/dev/null 2>&1; then
    nohup gnome-terminal --title="${title}" -- bash -lc "${cmd}" >/dev/null 2>&1 &
    echo "metrics_terminal=${title}"
  elif [ -n "${DISPLAY:-}" ] && command -v xterm >/dev/null 2>&1; then
    nohup xterm -T "${title}" -e bash -lc "${cmd}" >/dev/null 2>&1 &
    echo "metrics_terminal=${title}"
  elif command -v screen >/dev/null 2>&1; then
    screen -dmS "${screen_name}" bash -lc "${cmd}"
    echo "metrics_screen=${screen_name}"
    echo "attach_metrics=screen -r ${screen_name}"
  else
    echo "metrics_terminal_unavailable=true"
    echo "manual_metrics=tail -n +1 -F ${logfile}"
  fi
}

launch_route_worker() {
  mkdir -p "${RUN_DIR}/logs"
  local worker_screen="f250_route_worker_${RUN_LABEL}"
  local metrics_screen="f250_route_metrics_${RUN_LABEL}"
  local worker_log="${RUN_DIR}/logs/route_worker.log"
  local metrics_info
  metrics_info="$(open_metrics_terminal "F250 Route Metrics" "${ROUTE_TERMINAL_LOG}" "${metrics_screen}")"
  if [ "${F250_AUTO_LAYOUT:-true}" = "true" ]; then
    (DISPLAY="${DISPLAY:-:0}" "${SCRIPTS:-${PKG}/scripts}/f250_layout_windows.py" --kind metrics --wait-sec "${F250_METRICS_LAYOUT_WAIT_SEC:-1}" >>"${RUN_DIR}/logs/window_layout.log" 2>&1 || true) &
  fi
  local worker_info=""
  if command -v screen >/dev/null 2>&1; then
    screen -dmS "${worker_screen}" env \
      F250_ROUTE_BACKGROUND=false \
      F250_OPEN_METRICS_TERMINAL=false \
      RUN_ROOT="${RUN_ROOT}" \
      RUN_DIR="${RUN_DIR}" \
      RUN_LABEL="${RUN_LABEL}" \
      CURRENT_STATUS="${CURRENT_STATUS}" \
      ROUTE_MAX_DURATION_SEC="${ROUTE_MAX_DURATION_SEC}" \
      F250_ROUTE_ARGS="${F250_ROUTE_ARGS:-}" \
      F250_ROUTE_RECORD_PRESTART_SEC="${F250_ROUTE_RECORD_PRESTART_SEC:-}" \
      F250_ALLOW_RUN_DIR_REUSE=true \
      F250_ROUTE_TERMINAL_LOG_READY=true \
      bash "${SCRIPT_PATH}" --foreground --run-dir "${RUN_DIR}" --current-status "${CURRENT_STATUS}"
    worker_info="worker_screen=${worker_screen}"
  else
    nohup env \
      F250_ROUTE_BACKGROUND=false \
      F250_OPEN_METRICS_TERMINAL=false \
      RUN_ROOT="${RUN_ROOT}" \
      RUN_DIR="${RUN_DIR}" \
      RUN_LABEL="${RUN_LABEL}" \
      CURRENT_STATUS="${CURRENT_STATUS}" \
      ROUTE_MAX_DURATION_SEC="${ROUTE_MAX_DURATION_SEC}" \
      F250_ROUTE_ARGS="${F250_ROUTE_ARGS:-}" \
      F250_ROUTE_RECORD_PRESTART_SEC="${F250_ROUTE_RECORD_PRESTART_SEC:-}" \
      F250_ALLOW_RUN_DIR_REUSE=true \
      F250_ROUTE_TERMINAL_LOG_READY=true \
      bash "${SCRIPT_PATH}" --foreground --run-dir "${RUN_DIR}" --current-status "${CURRENT_STATUS}" \
      >"${worker_log}" 2>&1 &
    worker_info="worker_pid=$!"
  fi
  {
    printf "%s\n" "${metrics_info}"
    echo "${worker_info}"
    echo "worker_log=${worker_log}"
    echo "run_dir=${RUN_DIR}"
    echo "route_status_env=${ROUTE_STATUS_ENV}"
    echo "route_terminal_log=${ROUTE_TERMINAL_LOG}"
    echo "stop_script=${SCRIPTS}/f250_stop_all.sh"
  } >"${RUN_DIR}/logs/background_worker.env"
  cat <<EOF
F250 P0-P8 route started.
Metrics: F250 Route Metrics
Results: ${RUN_DIR}
Status: ${ROUTE_STATUS_ENV}
Stop: ${SCRIPTS}/f250_stop_all.sh
EOF
  if printf "%s\n" "${metrics_info}" | grep -q "manual_metrics"; then
    echo "Metrics log: ${ROUTE_TERMINAL_LOG}"
  fi
}

DRY_RUN="${F250_ROUTE_DRY_RUN:-false}"
RUN_IN_BACKGROUND="${F250_ROUTE_BACKGROUND:-true}"
RUN_LABEL="${RUN_LABEL:-${RUN_LABEL_OVERRIDE:-f250_p0_p8_route_${STAMP}}}"
RUN_DIR="${RUN_DIR:-}"

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
    --foreground)
      RUN_IN_BACKGROUND="false"
      shift
      ;;
    --run-label)
      RUN_LABEL="$2"
      shift 2
      ;;
    --run-dir)
      RUN_DIR="$2"
      RUN_LABEL="$(basename "$2")"
      shift 2
      ;;
    --current-status)
      CURRENT_STATUS="$2"
      shift 2
      ;;
    *)
      fail "unknown argument: $1; run ${0} --help"
      ;;
  esac
done

require_path "${WS}"
require_path "${PKG}"
require_path "${SCENE_CONFIG_FIXED}"
require_path "${WORLD_FIXED}"
require_path "${MAP_AUTHORITY}"
require_path "${R4H_BASELINE}"
require_path "${RECORDER}"
require_path "${CLEARANCE_EVALUATOR}"
require_path "${METRIC_MONITOR}"
require_path "${POSTPROCESS}"
require_path "${DISPLAY_HELPER}"
require_path "${START_DEMO}"

mkdir -p "${RUN_ROOT}"
RUN_ROOT="$(cd "${RUN_ROOT}" && pwd -P)"
DEFAULT_CURRENT_STATUS="${RUN_ROOT}/current/status.env"
if [ -z "${CURRENT_STATUS:-}" ]; then
  CURRENT_STATUS="${DEFAULT_CURRENT_STATUS}"
fi

if [ -z "${RUN_DIR}" ]; then
  RUN_DIR="${RUN_ROOT}/${RUN_LABEL}"
else
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
  *) fail "RUN_DIR must stay under RUN_ROOT=${RUN_ROOT}: ${RUN_DIR}" ;;
esac

if [[ ! "${RUN_LABEL}" =~ ^[A-Za-z0-9_.-]+$ ]]; then
  fail "RUN_LABEL must use only letters, numbers, dot, underscore, or dash: ${RUN_LABEL}"
fi

if [ -e "${RUN_DIR}" ] && [ "${F250_ALLOW_RUN_DIR_REUSE:-false}" != "true" ]; then
  fail "run directory already exists: ${RUN_DIR}; set F250_ALLOW_RUN_DIR_REUSE=true to reuse"
fi

P0_RUN_DIR=""
P0_VEHICLE=""
P0_STATE=""
P0_SCENE=""
P0_WORLD=""
if [ -f "${CURRENT_STATUS}" ]; then
  P0_RUN_DIR="$(env_value run_dir "${CURRENT_STATUS}")"
  P0_VEHICLE="$(env_value vehicle "${CURRENT_STATUS}")"
  P0_STATE="$(env_value state "${CURRENT_STATUS}")"
  P0_SCENE="$(env_value scene_config "${CURRENT_STATUS}")"
  P0_WORLD="$(env_value world "${CURRENT_STATUS}")"
fi

if [ "${DRY_RUN}" != "true" ]; then
  [ -f "${CURRENT_STATUS}" ] || fail "missing current P0 hover status: ${CURRENT_STATUS}; run f250_start_to_p0_hover.sh first"
  [ "${P0_VEHICLE}" = "f250" ] || fail "current status is not F250: ${CURRENT_STATUS} vehicle=${P0_VEHICLE:-<empty>}"
  if [ -n "${P0_SCENE}" ] && [ "${P0_SCENE}" != "${SCENE_CONFIG_FIXED}" ]; then
    fail "current status scene mismatch: ${P0_SCENE}"
  fi
  if [ -n "${P0_WORLD}" ] && [ "${P0_WORLD}" != "${WORLD_FIXED}" ]; then
    fail "current status world mismatch: ${P0_WORLD}"
  fi
fi

ROUTE_MAX_DURATION_SEC="${ROUTE_MAX_DURATION_SEC:-360}"
mkdir -p "${RUN_DIR}" "${RUN_DIR}/logs"

STATUS_FILE="${RUN_DIR}/status.env"
ROUTE_STATUS_ENV="${RUN_DIR}/route_status.env"
PROVENANCE_FILE="${RUN_DIR}/provenance.txt"
PARAMS_JSON="${RUN_DIR}/params.json"
TRAJECTORY_CSV="${RUN_DIR}/actual_trajectory.csv"
SUMMARY_JSON="${RUN_DIR}/summary.json"
CLEARANCE_STATIC_JSON="${RUN_DIR}/clearance_static_gate.json"
CLEARANCE_DYNAMIC_JSON="${RUN_DIR}/clearance_dynamic_telemetry.json"
METRIC_SUMMARY_JSON="${RUN_DIR}/metric_summary.json"
METRIC_WAYPOINTS_CSV="${RUN_DIR}/metric_waypoints.csv"
METRICS_JSON="${RUN_DIR}/metrics.json"
ROUTE_TERMINAL_LOG="${RUN_DIR}/route_terminal.log"
RECORDER_LOG="${RUN_DIR}/logs/recorder.log"
DISPLAY_LOG="${RUN_DIR}/logs/display_helper.log"
RELEASE_LOG="${RUN_DIR}/logs/release.log"
METRIC_REPLAY_LOG="${RUN_DIR}/logs/metric_replay.log"
CLEARANCE_STATIC_LOG="${RUN_DIR}/logs/clearance_static.log"
CLEARANCE_DYNAMIC_LOG="${RUN_DIR}/logs/clearance_dynamic.log"
POSTPROCESS_LOG="${RUN_DIR}/logs/postprocess.log"

if [ "${F250_ROUTE_TERMINAL_LOG_READY:-false}" != "true" ]; then
  : >"${ROUTE_TERMINAL_LOG}"
fi

{
  echo "created_at=$(date -Is)"
  echo "project_root=${PROJECT_ROOT}"
  echo "workspace=${WS}"
  echo "package=${PKG}"
  echo "script=${0}"
  echo "run_dir=${RUN_DIR}"
  echo "run_label=${RUN_LABEL}"
  echo "source_p0_status=${CURRENT_STATUS}"
  echo "source_p0_run_dir=${P0_RUN_DIR}"
  echo "source_p0_state=${P0_STATE}"
  echo "scene_config=${SCENE_CONFIG_FIXED}"
  echo "world=${WORLD_FIXED}"
  echo "map_authority=${MAP_AUTHORITY}"
  echo "r4h_baseline=${R4H_BASELINE}"
  echo "vehicle=f250"
  echo "route_policy=excludes_metric_3_10_and_yaw;dynamic_boat_clearance_telemetry_only"
  echo "host=$(hostname)"
  echo "user=$(id -un)"
} >"${PROVENANCE_FILE}"

write_params_json
write_status "prepared"

HELPER_COMMON=(
  --run-dir "${RUN_DIR}"
  --run-label "${RUN_LABEL}"
  --scene-config "${SCENE_CONFIG_FIXED}"
  --dynamic-mode auto
  --max-duration-sec "${ROUTE_MAX_DURATION_SEC}"
  --terminal-log "${ROUTE_TERMINAL_LOG}"
  --route-status-env "${ROUTE_STATUS_ENV}"
  --status-env "${STATUS_FILE}"
)

if [ "${DRY_RUN}" = "true" ]; then
  python3 "${DISPLAY_HELPER}" dry-run "${HELPER_COMMON[@]}"
  exit $?
fi

source /opt/ros/noetic/setup.bash
if [ -f "${WS}/devel/setup.bash" ]; then
  source "${WS}/devel/setup.bash"
else
  export ROS_PACKAGE_PATH="${WS}/src:${ROS_PACKAGE_PATH:-}"
fi

export MARITIME_VEHICLE="f250"
export SCENE_LEVEL="${SCENE_LEVEL_FIXED}"
export SCENE_CONFIG="${SCENE_CONFIG_FIXED}"
export WORLD="${WORLD_FIXED}"
export PERCEPTION_SOURCE="lidar"
export DYNAMIC_MODE="auto"
export MARITIME_START_TOPIC="${MARITIME_START_TOPIC:-/maritime/demo/start_waypoints}"

ensure_ros_available

if [ "${RUN_IN_BACKGROUND}" = "true" ]; then
  write_status "background_worker_starting"
  launch_route_worker
  exit 0
fi

RECORDER_PID=""
DISPLAY_PID=""
cleanup() {
  stop_pid "${DISPLAY_PID}"
  stop_pid "${RECORDER_PID}"
  append_terminal_over
}
trap cleanup EXIT

write_status "recording"

python3 "${RECORDER}" \
  --scene-config "${SCENE_CONFIG_FIXED}" \
  --output-csv "${TRAJECTORY_CSV}" \
  --summary-json "${SUMMARY_JSON}" \
  --max-duration-sec "${ROUTE_MAX_DURATION_SEC}" \
  ${F250_ROUTE_ARGS:-} \
  >"${RECORDER_LOG}" 2>&1 &
RECORDER_PID="$!"

python3 "${DISPLAY_HELPER}" live-monitor "${HELPER_COMMON[@]}" \
  2>"${DISPLAY_LOG}" &
DISPLAY_PID="$!"

sleep "${F250_ROUTE_RECORD_PRESTART_SEC:-1.0}"

"${START_DEMO}" "${MARITIME_START_TOPIC}" >"${RELEASE_LOG}" 2>&1

set +e
wait "${RECORDER_PID}"
RECORDER_STATUS=$?
RECORDER_PID=""
set -e

stop_pid "${DISPLAY_PID}"
DISPLAY_PID=""

if [ "${RECORDER_STATUS}" -ne 0 ]; then
  write_status "recorder_failed"
  echo "[f250-route] recorder failed status=${RECORDER_STATUS}; see ${RECORDER_LOG}" >&2
  exit "${RECORDER_STATUS}"
fi

write_status "postprocessing"

set +e
python3 "${METRIC_MONITOR}" --offline \
  --scene-config "${SCENE_CONFIG_FIXED}" \
  --trajectory-csv "${TRAJECTORY_CSV}" \
  --output-dir "${RUN_DIR}" \
  --run-label "${RUN_LABEL}" \
  --dynamic-mode auto \
  >"${METRIC_REPLAY_LOG}" 2>&1
METRIC_STATUS=$?

python3 "${CLEARANCE_EVALUATOR}" \
  --scene-config "${SCENE_CONFIG_FIXED}" \
  --trajectory-csv "${TRAJECTORY_CSV}" \
  --summary-json "${CLEARANCE_STATIC_JSON}" \
  --dynamic-mode none \
  >"${CLEARANCE_STATIC_LOG}" 2>&1
CLEARANCE_STATIC_STATUS=$?

python3 "${CLEARANCE_EVALUATOR}" \
  --scene-config "${SCENE_CONFIG_FIXED}" \
  --trajectory-csv "${TRAJECTORY_CSV}" \
  --summary-json "${CLEARANCE_DYNAMIC_JSON}" \
  --dynamic-mode auto \
  >"${CLEARANCE_DYNAMIC_LOG}" 2>&1
CLEARANCE_DYNAMIC_STATUS=$?

python3 "${POSTPROCESS}" \
  --candidate-id "${RUN_LABEL}" \
  --run-dir "${RUN_DIR}" \
  --params-json "${PARAMS_JSON}" \
  --summary-json "${SUMMARY_JSON}" \
  --clearance-static-json "${CLEARANCE_STATIC_JSON}" \
  --clearance-dynamic-json "${CLEARANCE_DYNAMIC_JSON}" \
  --metric-summary-json "${METRIC_SUMMARY_JSON}" \
  --output-json "${METRICS_JSON}" \
  --source lidar \
  --vehicle f250 \
  >"${POSTPROCESS_LOG}" 2>&1
POSTPROCESS_STATUS=$?
set -e

{
  echo "metric_replay_status=${METRIC_STATUS}"
  echo "clearance_static_status=${CLEARANCE_STATIC_STATUS}"
  echo "clearance_dynamic_status=${CLEARANCE_DYNAMIC_STATUS}"
  echo "postprocess_status=${POSTPROCESS_STATUS}"
  echo "metric_replay_log=${METRIC_REPLAY_LOG}"
  echo "clearance_static_log=${CLEARANCE_STATIC_LOG}"
  echo "clearance_dynamic_log=${CLEARANCE_DYNAMIC_LOG}"
  echo "postprocess_log=${POSTPROCESS_LOG}"
} >"${RUN_DIR}/postprocess_status.env"

set +e
python3 "${DISPLAY_HELPER}" finalize "${HELPER_COMMON[@]}" --print-final
FINAL_STATUS=$?
set -e

exit "${FINAL_STATUS}"
