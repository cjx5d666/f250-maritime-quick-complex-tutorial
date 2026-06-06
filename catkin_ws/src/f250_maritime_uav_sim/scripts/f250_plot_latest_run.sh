#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
# shellcheck source=/dev/null
source "${SCRIPT_DIR}/f250_paths.sh"
PROJECT_ROOT="$(f250_resolve_project_root "${SCRIPT_DIR}")"
PKG="$(f250_resolve_package_root "${SCRIPT_DIR}" "${PROJECT_ROOT}")"
SCRIPTS="${PKG}/scripts"
HELPER="${SCRIPTS}/f250_render_latest_planned_vs_flown.py"

RUN_ROOT_DEFAULT="${PROJECT_ROOT}/runs/f250_human_scripts"
MAP_AUTHORITY_DEFAULT="${PROJECT_ROOT}/data/map_authority/p0_p8_hard_requirement_20260530"

usage() {
  cat <<EOF
Usage:
  ${0} [--dry-run] [--list] [--run-dir DIR] [--output-dir DIR]
  ${0} --help

Offline F250-only postprocess for the fixed maritime_quick_complex
2026-05-30 P0-P8 route. It selects the latest suitable F250 route run and
writes one planned-vs-flown figure plus structured JSON/CSV/Markdown outputs.

Default outputs in the selected run directory:
  latest_planned_vs_flown.png
  latest_plot_summary.json
  latest_plot_points.csv
  latest_plot_summary.md

Selection:
  - RUN_DIR or --run-dir selects an explicit run.
  - Otherwise scans RUN_ROOT, default:
    ${RUN_ROOT_DEFAULT}
  - Auto-selection requires actual_trajectory.csv and route/summary/metrics JSON.
  - FC 3.10-only runs are skipped during auto-selection.

Inputs:
  authoritative map package:
    ${MAP_AUTHORITY_DEFAULT}
  geometry is read from authoritative CSV/JSON files, not from cached PNGs.

Useful environment overrides:
  RUN_DIR=...       explicit run directory
  RUN_ROOT=...      scan root, default shown above
  MAP_AUTHORITY=... authoritative map directory
  PYTHON=...        Python executable, default python3
  F250_PROJECT_ROOT=... override project root detected from script path

Modes:
  --dry-run         print selected target and planned outputs; do not render
  --list            list suitable candidates; do not render
EOF
}

fail() {
  echo "f250_plot_latest_run: $*" >&2
  exit 2
}

open_plot_if_possible() {
  local figure="$1"
  [ -n "${figure}" ] || return 0
  [ -f "${figure}" ] || return 0
  [ "${F250_OPEN_PLOT:-true}" = "true" ] || return 0
  local display_value="${DISPLAY:-:0}"
  if command -v xdg-open >/dev/null 2>&1; then
    (DISPLAY="${display_value}" nohup xdg-open "${figure}" >/dev/null 2>&1 &)
    echo "Opened: ${figure}"
  elif command -v eog >/dev/null 2>&1; then
    (DISPLAY="${display_value}" nohup eog "${figure}" >/dev/null 2>&1 &)
    echo "Opened: ${figure}"
  else
    echo "Open manually: ${figure}"
  fi
}

RUN_ROOT="${RUN_ROOT:-${RUN_ROOT_DEFAULT}}"
MAP_AUTHORITY="${MAP_AUTHORITY:-${MAP_AUTHORITY_DEFAULT}}"
PYTHON_BIN="${PYTHON:-python3}"

ARGS=()
SAW_RUN_DIR="false"
SAW_RUN_ROOT="false"
SAW_MAP_AUTHORITY="false"
MODE="render"

while [ "$#" -gt 0 ]; do
  case "$1" in
    --help|-h)
      usage
      exit 0
      ;;
    --dry-run)
      MODE="dry-run"
      ARGS+=("$1")
      shift
      ;;
    --list)
      MODE="list"
      ARGS+=("$1")
      shift
      ;;
    --run-dir)
      [ "$#" -ge 2 ] || fail "--run-dir requires DIR"
      SAW_RUN_DIR="true"
      ARGS+=("$1" "$2")
      shift 2
      ;;
    --run-root)
      [ "$#" -ge 2 ] || fail "--run-root requires DIR"
      SAW_RUN_ROOT="true"
      RUN_ROOT="$2"
      ARGS+=("$1" "$2")
      shift 2
      ;;
    --map-authority)
      [ "$#" -ge 2 ] || fail "--map-authority requires DIR"
      SAW_MAP_AUTHORITY="true"
      MAP_AUTHORITY="$2"
      ARGS+=("$1" "$2")
      shift 2
      ;;
    --output-dir)
      [ "$#" -ge 2 ] || fail "--output-dir requires DIR"
      ARGS+=("$1" "$2")
      shift 2
      ;;
    *)
      fail "unknown argument: $1; run ${0} --help"
      ;;
  esac
done

[ -f "${HELPER}" ] || fail "missing helper: ${HELPER}"
[ -d "${RUN_ROOT}" ] || fail "missing RUN_ROOT: ${RUN_ROOT}"
[ -d "${MAP_AUTHORITY}" ] || fail "missing MAP_AUTHORITY: ${MAP_AUTHORITY}"

if [ "${SAW_RUN_DIR}" = "false" ] && [ -n "${RUN_DIR:-}" ]; then
  ARGS+=(--run-dir "${RUN_DIR}")
fi
if [ "${SAW_RUN_ROOT}" = "false" ]; then
  ARGS+=(--run-root "${RUN_ROOT}")
fi
if [ "${SAW_MAP_AUTHORITY}" = "false" ]; then
  ARGS+=(--map-authority "${MAP_AUTHORITY}")
fi

set +e
HELPER_OUTPUT="$("${PYTHON_BIN}" "${HELPER}" "${ARGS[@]}" 2>&1)"
STATUS=$?
set -e

if [ "${STATUS}" -ne 0 ]; then
  printf "%s\n" "${HELPER_OUTPUT}" >&2
  exit "${STATUS}"
fi

if [ "${MODE}" = "render" ]; then
  printf "%s\n" "${HELPER_OUTPUT}" | "${PYTHON_BIN}" -c '
import json
import sys

text = sys.stdin.read().strip()
try:
    data = json.loads(text)
except Exception:
    print(text)
    raise SystemExit(0)
print("F250 plot ready.")
print("Route OK: %s" % str(data.get("route_acceptance_ok")).lower())
print("Figure: %s" % data.get("planned_vs_flown_png", ""))
print("Summary: %s" % data.get("summary_json", ""))
'
  FIGURE_PATH="$(printf "%s\n" "${HELPER_OUTPUT}" | "${PYTHON_BIN}" -c 'import json, sys; print(json.loads(sys.stdin.read()).get("planned_vs_flown_png", ""))' 2>/dev/null || true)"
  open_plot_if_possible "${FIGURE_PATH}"
elif [ "${MODE}" = "dry-run" ]; then
  printf "%s\n" "${HELPER_OUTPUT}" | awk -F= '
    $1 == "selected_run_dir" {run=$2}
    $1 == "dry_run" {dry=$2}
    $1 == "would_write" {outputs[++n]=$2}
    END {
      print "F250 plot dry run."
      if (run != "") print "Run: " run
      for (i = 1; i <= n; i++) print "Would write: " outputs[i]
    }'
else
  printf "%s\n" "${HELPER_OUTPUT}" | awk -F= '
    $1 == "selected_run_dir" {run=$2}
    /^  [*-] / {count++}
    END {
      print "F250 route plot candidates."
      if (run != "") print "Selected: " run
      print "Candidates: " count
    }'
fi
