#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
# shellcheck source=/dev/null
source "${SCRIPT_DIR}/f250_paths.sh"
PROJECT_ROOT="$(f250_resolve_project_root "${SCRIPT_DIR}")"
WS="${PROJECT_ROOT}/catkin_ws"
PKG="$(f250_resolve_package_root "${SCRIPT_DIR}" "${PROJECT_ROOT}")"
HELPER="${PKG}/scripts/f250_fc_3_10_steady_state.py"
RUN_ROOT="${RUN_ROOT:-${PROJECT_ROOT}/maritime_quick_complex/runs/f250_human_scripts}"
DEFAULT_CURRENT_STATUS="${RUN_ROOT}/current/status.env"
CURRENT_STATUS="${CURRENT_STATUS:-}"
STAMP="$(date +%Y%m%d_%H%M%S)"
SCRIPT_PATH="$(readlink -f "$0")"

usage() {
  cat <<EOF
Usage:
  ${0} [--dry-run]
  ${0} [--geometry-check]
  ${0} [--foreground]
  ${0} --help

Runs the F250-only Metric 3.10 FC steady-state test from an already running P0
hover stack. It does not start PX4, Gazebo, ROS, RViz, or route waypoints. It
derives safe FC geometry from the authoritative map package, sends position,
velocity, and yaw targets through the existing /planning/pos_cmd ->
/mavros/setpoint_position/local chain, samples actual state from
/mavros/local_position/odom, evaluates only steady-state windows, then commands
P0 hover again.

Default tasks:
  geometry: P0 from route_waypoints.csv, z fixed at 10 m, u is opposite the
            ship-to-three-islands direction.
  velocity: 2 m/s, 10 windows = 5 AB round trips. Default geometry uses
            A=P0, B=P0+60u. AB endpoints/segments are audited against
            authoritative planner obstacles, and L can be adjusted for
            settle/eval time or clearance.
  position: 10 decagon vertices, P0 as the nearest-obstacle/start vertex,
            C=P0+10u, denominator R=10 m.
  yaw: +90/+180/+270/+360, -90/-180/-270/-360, +180, 0.
  Metric 3.10 is FC-only steady-state evidence, not route/planner acceptance.

Default human mode:
  The FC test worker runs in a background screen/nohup worker and opens a
  separate metrics terminal or screen that tails fc_3_10_terminal.log. The
  calling terminal only prints run/log locations. Use --foreground for the old
  blocking behavior.

Outputs:
  fc_3_10_summary.json
  fc_3_10_samples.csv
  fc_3_10_phases.csv
  fc_3_10_geometry_audit.json
  fc_3_10_decagon_points.csv
  fc_3_10_terminal.log
  status.env

Useful environment overrides:
  RUN_ROOT=...                  default: ${RUN_ROOT}
  RUN_LABEL=...                 default: f250_fc_3_10_steady_state_<timestamp>
  RUN_DIR=...                   explicit output directory under RUN_ROOT
  CURRENT_STATUS=...            default: ${DEFAULT_CURRENT_STATUS}
  HOVER_TARGET=x,y,z,yaw        default: read from current/status.env, else P0
  F250_ALLOW_RUN_DIR_REUSE=true allow reusing RUN_DIR
  F250_FC_3_10_ARGS='...'       extra helper args
  F250_FC_BACKGROUND=true|false
  F250_OPEN_METRICS_TERMINAL=true|false
  F250_PROJECT_ROOT=...         override project root detected from script path
  MAP_AUTHORITY=...             override authoritative map directory for helper

Metric formulas written to JSON:
  Each phase ignores transient time, searches for a stationary window, then
  computes error only on the following eval window. A phase with no stationary
  window is not_settled and is not a formal steady-state metric.
  E_pos_i = norm(mean(eval actual_pos_xy) - target_xy) / 10 * 100
  E_vel_i = norm(mean(eval actual_vel_world_xy) - desired_vel_xy) / commanded_speed * 100
  E_yaw_i = abs(mean(eval wrap(actual_yaw - target_yaw))) / yaw_denominator * 100
  E_vel_2mps is the formal velocity item.
  E_vel_selected = E_vel_2mps
  E3.10_selected = max(E_pos, E_vel_selected, E_yaw)
  E3.10_2mps is retained for comparison.
  MAVROS odom twist is rotated body -> world with current yaw before velocity comparison.

Dry-run:
  ${0} --dry-run
  Does not require ROS master and writes synthetic demo outputs.

Geometry check:
  ${0} --geometry-check
  Does not require ROS master; writes authoritative geometry audit only plus empty metric CSV/JSON.
EOF
}

fail() {
  echo "f250_run_fc_3_10_steady_state: $*" >&2
  exit 2
}

env_value() {
  local key="$1"
  local file="$2"
  [ -f "${file}" ] || return 0
  awk -F= -v key="${key}" '$1 == key {sub(/^[^=]*=/, ""); print; exit}' "${file}"
}

write_status() {
  local state="$1"
  {
    echo "state=${state}"
    echo "updated_at=$(date -Is)"
    echo "run_dir=${RUN_DIR}"
    echo "run_label=${RUN_LABEL}"
    echo "vehicle=f250"
    echo "metric=3.10_fc_only_steady_state"
    echo "dry_run=${DRY_RUN}"
    echo "geometry_check=${GEOMETRY_CHECK}"
    echo "source_p0_status=${CURRENT_STATUS}"
    echo "source_p0_run_dir=${P0_RUN_DIR}"
    echo "hover_target=${HOVER_TARGET_RESOLVED}"
    echo "summary_json=${SUMMARY_JSON}"
    echo "samples_csv=${SAMPLES_CSV}"
    echo "phase_csv=${PHASE_CSV}"
    echo "geometry_audit_json=${GEOMETRY_JSON}"
    echo "decagon_csv=${DECAGON_CSV}"
    echo "terminal_display_log=${DISPLAY_LOG}"
    echo "route_acceptance_written=false"
  } >"${STATUS_FILE}"
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
  if ! rostopic list | grep -qx "/planning/pos_cmd"; then
    fail "missing /planning/pos_cmd; setpoint chain does not look active"
  fi
}

DRY_RUN="${F250_FC_3_10_DRY_RUN:-false}"
GEOMETRY_CHECK="${F250_FC_3_10_GEOMETRY_CHECK:-false}"
RUN_IN_BACKGROUND="${F250_FC_BACKGROUND:-true}"

terminal_command() {
  local logfile="$1"
  local heading="${2:-F250 FC 3.10 Metrics}"
  printf "touch %q; tail -n +1 -F %q" "${logfile}" "${logfile}"
}

append_terminal_over() {
  [ -n "${DISPLAY_LOG:-}" ] || return 0
  mkdir -p "$(dirname "${DISPLAY_LOG}")"
  if [ -f "${DISPLAY_LOG}" ] && [ "$(tail -n 1 "${DISPLAY_LOG}" 2>/dev/null || true)" = "OVER" ]; then
    return 0
  fi
  printf "\nOVER\n" >>"${DISPLAY_LOG}"
}

open_metrics_terminal() {
  local title="$1"
  local logfile="$2"
  local screen_name="$3"
  [ "${F250_OPEN_METRICS_TERMINAL:-true}" = "true" ] || return 0
  local cmd
  cmd="$(terminal_command "${logfile}" "${title}")"
  if [ -n "${DISPLAY:-}" ] && command -v x-terminal-emulator >/dev/null 2>&1; then
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
    echo "manual_metrics=tail -F ${logfile}"
  fi
}

launch_fc_worker() {
  local worker_screen="f250_fc310_worker_${RUN_LABEL}"
  local metrics_screen="f250_fc310_metrics_${RUN_LABEL}"
  local worker_log="${RUN_DIR}/fc_3_10_worker.log"
  local metrics_info
  metrics_info="$(open_metrics_terminal "F250 FC 3.10 Metrics" "${DISPLAY_LOG}" "${metrics_screen}")"
  if [ "${F250_AUTO_LAYOUT:-true}" = "true" ]; then
    (DISPLAY="${DISPLAY:-:0}" "${SCRIPTS:-${PKG}/scripts}/f250_layout_windows.py" --kind metrics --wait-sec "${F250_METRICS_LAYOUT_WAIT_SEC:-1}" >>"${RUN_DIR}/logs/window_layout.log" 2>&1 || true) &
  fi
  local worker_info=""
  if command -v screen >/dev/null 2>&1; then
    screen -dmS "${worker_screen}" env \
      F250_FC_BACKGROUND=false \
      F250_OPEN_METRICS_TERMINAL=false \
      RUN_ROOT="${RUN_ROOT}" \
      RUN_DIR="${RUN_DIR}" \
      RUN_LABEL="${RUN_LABEL}" \
      CURRENT_STATUS="${CURRENT_STATUS}" \
      HOVER_TARGET="${HOVER_TARGET_RESOLVED}" \
      F250_FC_3_10_ARGS="${F250_FC_3_10_ARGS:-}" \
      F250_ALLOW_RUN_DIR_REUSE=true \
      F250_FC_TERMINAL_LOG_READY=true \
      bash "${SCRIPT_PATH}" --foreground --run-dir "${RUN_DIR}" --current-status "${CURRENT_STATUS}"
    worker_info="worker_screen=${worker_screen}"
  else
    nohup env \
      F250_FC_BACKGROUND=false \
      F250_OPEN_METRICS_TERMINAL=false \
      RUN_ROOT="${RUN_ROOT}" \
      RUN_DIR="${RUN_DIR}" \
      RUN_LABEL="${RUN_LABEL}" \
      CURRENT_STATUS="${CURRENT_STATUS}" \
      HOVER_TARGET="${HOVER_TARGET_RESOLVED}" \
      F250_FC_3_10_ARGS="${F250_FC_3_10_ARGS:-}" \
      F250_ALLOW_RUN_DIR_REUSE=true \
      F250_FC_TERMINAL_LOG_READY=true \
      bash "${SCRIPT_PATH}" --foreground --run-dir "${RUN_DIR}" --current-status "${CURRENT_STATUS}" \
      >"${worker_log}" 2>&1 &
    worker_info="worker_pid=$!"
  fi
  {
    printf "%s\n" "${metrics_info}"
    echo "${worker_info}"
    echo "worker_log=${worker_log}"
    echo "run_dir=${RUN_DIR}"
    echo "status_env=${STATUS_FILE}"
    echo "terminal_display_log=${DISPLAY_LOG}"
    echo "stop_script=${PKG}/scripts/f250_stop_all.sh"
  } >"${RUN_DIR}/background_worker.env"
  cat <<EOF
F250 FC 3.10 started.
Metrics: F250 FC 3.10 Metrics
Results: ${RUN_DIR}
Status: ${STATUS_FILE}
Stop: ${PKG}/scripts/f250_stop_all.sh
EOF
  if printf "%s\n" "${metrics_info}" | grep -q "manual_metrics"; then
    echo "Metrics log: ${DISPLAY_LOG}"
  fi
}

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
    --geometry-check)
      GEOMETRY_CHECK="true"
      RUN_IN_BACKGROUND="false"
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

[ -d "${WS}" ] || fail "missing workspace: ${WS}"
[ -d "${PKG}" ] || fail "missing package: ${PKG}"
[ -x "${HELPER}" ] || [ -f "${HELPER}" ] || fail "missing helper: ${HELPER}"

mkdir -p "${RUN_ROOT}"
RUN_ROOT="$(cd "${RUN_ROOT}" && pwd -P)"
DEFAULT_CURRENT_STATUS="${RUN_ROOT}/current/status.env"
if [ -z "${CURRENT_STATUS:-}" ]; then
  CURRENT_STATUS="${DEFAULT_CURRENT_STATUS}"
fi

RUN_LABEL="${RUN_LABEL:-${RUN_LABEL_OVERRIDE:-f250_fc_3_10_steady_state_${STAMP}}}"
if [ -z "${RUN_DIR:-}" ]; then
  RUN_DIR="${RUN_ROOT}/${RUN_LABEL}"
else
  case "${RUN_DIR}" in
    /*) ;;
    *) RUN_DIR="${RUN_ROOT}/${RUN_DIR}" ;;
  esac
  RUN_LABEL="$(basename "${RUN_DIR}")"
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
P0_HOVER_TARGET=""
if [ -f "${CURRENT_STATUS}" ]; then
  P0_RUN_DIR="$(env_value run_dir "${CURRENT_STATUS}")"
  P0_VEHICLE="$(env_value vehicle "${CURRENT_STATUS}")"
  P0_STATE="$(env_value state "${CURRENT_STATUS}")"
  P0_HOVER_TARGET="$(env_value hover_target "${CURRENT_STATUS}")"
fi

if [ -n "${P0_VEHICLE}" ] && [ "${P0_VEHICLE}" != "f250" ]; then
  fail "current status is not F250: ${CURRENT_STATUS} vehicle=${P0_VEHICLE}"
fi

HOVER_TARGET_RESOLVED="${HOVER_TARGET:-${P0_HOVER_TARGET:-55.0,16.0,10.0,0.469929}}"

mkdir -p "${RUN_DIR}" "${RUN_DIR}/logs"
STATUS_FILE="${RUN_DIR}/status.env"
SUMMARY_JSON="${RUN_DIR}/fc_3_10_summary.json"
SAMPLES_CSV="${RUN_DIR}/fc_3_10_samples.csv"
PHASE_CSV="${RUN_DIR}/fc_3_10_phases.csv"
GEOMETRY_JSON="${RUN_DIR}/fc_3_10_geometry_audit.json"
DECAGON_CSV="${RUN_DIR}/fc_3_10_decagon_points.csv"
DISPLAY_LOG="${RUN_DIR}/fc_3_10_terminal.log"
PROVENANCE_FILE="${RUN_DIR}/provenance.txt"

if [ "${F250_FC_TERMINAL_LOG_READY:-false}" != "true" ]; then
  : >"${DISPLAY_LOG}"
fi

{
  echo "created_at=$(date -Is)"
  echo "project_root=${PROJECT_ROOT}"
  echo "workspace=${WS}"
  echo "package=${PKG}"
  echo "script=${0}"
  echo "helper=${HELPER}"
  echo "run_dir=${RUN_DIR}"
  echo "run_label=${RUN_LABEL}"
  echo "source_p0_status=${CURRENT_STATUS}"
  echo "source_p0_run_dir=${P0_RUN_DIR}"
  echo "source_p0_state=${P0_STATE}"
  echo "hover_target=${HOVER_TARGET_RESOLVED}"
  echo "vehicle=f250"
  echo "metric_policy=3.10 independent FC-only steady-state test; no route acceptance writes"
  echo "geometry_policy=authoritative map package; no PNG coordinate truth"
  echo "velocity_policy=settled standard, speed 2 mps, 10 windows, 5 AB round trips"
  echo "position_policy=10 decagon vertices, R=10m by default"
  echo "yaw_policy=+90,+180,+270,+360,-90,-180,-270,-360,+180,0"
  echo "host=$(hostname)"
  echo "user=$(id -un)"
} >"${PROVENANCE_FILE}"

write_status "prepared"

HELPER_ARGS=(
  --run-dir "${RUN_DIR}"
  --run-label "${RUN_LABEL}"
  --summary-json "${SUMMARY_JSON}"
  --samples-csv "${SAMPLES_CSV}"
  --phase-csv "${PHASE_CSV}"
  --geometry-audit-json "${GEOMETRY_JSON}"
  --decagon-csv "${DECAGON_CSV}"
  --display-log "${DISPLAY_LOG}"
  --p0-status "${CURRENT_STATUS}"
  --p0-run-dir "${P0_RUN_DIR}"
  --hover-target "${HOVER_TARGET_RESOLVED}"
)
if [ -n "${MAP_AUTHORITY:-}" ]; then
  HELPER_ARGS+=(--map-authority "${MAP_AUTHORITY}")
fi

if [ "${GEOMETRY_CHECK}" = "true" ]; then
  HELPER_ARGS+=(--geometry-check)
elif [ "${DRY_RUN}" = "true" ]; then
  HELPER_ARGS+=(--dry-run)
else
  source /opt/ros/noetic/setup.bash
  if [ -f "${WS}/devel/setup.bash" ]; then
    source "${WS}/devel/setup.bash"
  else
    export ROS_PACKAGE_PATH="${WS}/src:${ROS_PACKAGE_PATH:-}"
  fi
  ensure_ros_available
fi

if [ -n "${F250_FC_3_10_ARGS:-}" ]; then
  # shellcheck disable=SC2206
  EXTRA_ARGS=(${F250_FC_3_10_ARGS})
  HELPER_ARGS+=("${EXTRA_ARGS[@]}")
fi

if [ "${DRY_RUN}" != "true" ] && [ "${GEOMETRY_CHECK}" != "true" ] && [ "${RUN_IN_BACKGROUND}" = "true" ]; then
  write_status "background_worker_starting"
  launch_fc_worker
  exit 0
fi

trap append_terminal_over EXIT

set +e
python3 "${HELPER}" "${HELPER_ARGS[@]}"
STATUS=$?
set -e

if [ "${STATUS}" -eq 0 ]; then
  write_status "complete"
else
  write_status "failed"
  echo "F250 FC 3.10 failed with status ${STATUS}; see ${RUN_DIR}" >&2
fi
exit "${STATUS}"
