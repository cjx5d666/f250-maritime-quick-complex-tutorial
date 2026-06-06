#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
PX4_ROOT="${F250_PX4_ROOT:-}"
DRY_RUN="false"
AIRFRAME_NAME="10020_gazebo-classic_f250"
AIRFRAME_SRC="${ROOT}/px4_overlay/ROMFS/px4fmu_common/init.d-posix/airframes/${AIRFRAME_NAME}"
MODEL_SRC="${ROOT}/px4_overlay/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/f250"

usage() {
  cat <<EOF
Usage:
  F250_PX4_ROOT=/path/to/PX4-Autopilot ${0} [--dry-run]
  ${0} --help

Installs the F250 PX4 overlay into a PX4 v1.16.0 style tree:
  - ROMFS/px4fmu_common/init.d-posix/airframes/${AIRFRAME_NAME}
  - Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/f250
  - airframes/CMakeLists.txt registration for ${AIRFRAME_NAME}

The script is idempotent. Existing changed target files are backed up under
<PX4 root>/.f250_overlay_backups/<timestamp>/ before being replaced.
EOF
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    --dry-run)
      DRY_RUN="true"
      shift
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      echo "install_px4_overlay: unknown argument: $1" >&2
      echo "Run ${0} --help" >&2
      exit 2
      ;;
  esac
done

fail() {
  echo "install_px4_overlay: $*" >&2
  exit 2
}

run() {
  if [ "${DRY_RUN}" = "true" ]; then
    printf '[dry-run] '
    printf '%q ' "$@"
    printf '\n'
  else
    "$@"
  fi
}

[ -n "${PX4_ROOT}" ] || fail "set F250_PX4_ROOT to the target PX4-Autopilot root"
PX4_ROOT="$(cd "${PX4_ROOT}" 2>/dev/null && pwd -P)" || fail "F250_PX4_ROOT does not exist: ${F250_PX4_ROOT}"

AIRFRAME_DST_DIR="${PX4_ROOT}/ROMFS/px4fmu_common/init.d-posix/airframes"
AIRFRAME_DST="${AIRFRAME_DST_DIR}/${AIRFRAME_NAME}"
CMAKE_FILE="${AIRFRAME_DST_DIR}/CMakeLists.txt"
MODEL_DST_ROOT="${PX4_ROOT}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models"
MODEL_DST="${MODEL_DST_ROOT}/f250"
BACKUP_ROOT="${PX4_ROOT}/.f250_overlay_backups/$(date +%Y%m%d_%H%M%S)"

[ -f "${AIRFRAME_SRC}" ] || fail "missing overlay airframe: ${AIRFRAME_SRC}"
[ -d "${MODEL_SRC}" ] || fail "missing overlay model directory: ${MODEL_SRC}"
[ -d "${AIRFRAME_DST_DIR}" ] || fail "missing PX4 airframe directory: ${AIRFRAME_DST_DIR}"
[ -f "${CMAKE_FILE}" ] || fail "missing PX4 airframes CMakeLists.txt: ${CMAKE_FILE}"
[ -d "${MODEL_DST_ROOT}" ] || fail "missing PX4 Gazebo Classic models directory: ${MODEL_DST_ROOT}"

backup_target_file() {
  local target="$1"
  local rel="${target#${PX4_ROOT}/}"
  [ -f "${target}" ] || return 0
  if cmp -s "${target}" "${2}"; then
    return 0
  fi
  run mkdir -p "${BACKUP_ROOT}/$(dirname "${rel}")"
  run cp -a "${target}" "${BACKUP_ROOT}/${rel}"
}

install_file_if_needed() {
  local src="$1"
  local dst="$2"
  if [ -f "${dst}" ] && cmp -s "${src}" "${dst}"; then
    echo "unchanged: ${dst}"
    return 0
  fi
  backup_target_file "${dst}" "${src}"
  run mkdir -p "$(dirname "${dst}")"
  run cp -a "${src}" "${dst}"
}

install_model_tree() {
  local src_file rel dst_file
  while IFS= read -r -d '' src_file; do
    rel="${src_file#${MODEL_SRC}/}"
    dst_file="${MODEL_DST}/${rel}"
    install_file_if_needed "${src_file}" "${dst_file}"
  done < <(find "${MODEL_SRC}" -type f -print0 | sort -z)
}

register_airframe() {
  if grep -qE "^[[:space:]]*${AIRFRAME_NAME}([[:space:]]|$)" "${CMAKE_FILE}"; then
    echo "already registered: ${AIRFRAME_NAME}"
    return 0
  fi

  local tmp
  tmp="$(mktemp)"
  if ! awk -v entry="${AIRFRAME_NAME}" '
    BEGIN { in_block = 0; inserted = 0 }
    /^[[:space:]]*px4_add_romfs_files[[:space:]]*\(/ { in_block = 1 }
    in_block && !inserted && /^[[:space:]]*\)[[:space:]]*$/ {
      print "\t" entry
      inserted = 1
      in_block = 0
    }
    { print }
    END { if (!inserted) exit 3 }
  ' "${CMAKE_FILE}" > "${tmp}"; then
    rm -f "${tmp}"
    fail "could not find px4_add_romfs_files() insertion point in ${CMAKE_FILE}"
  fi

  if [ "${DRY_RUN}" = "true" ]; then
    echo "[dry-run] would insert ${AIRFRAME_NAME} into ${CMAKE_FILE}"
    rm -f "${tmp}"
    return 0
  fi

  run mkdir -p "${BACKUP_ROOT}/$(dirname "${CMAKE_FILE#${PX4_ROOT}/}")"
  run cp -a "${CMAKE_FILE}" "${BACKUP_ROOT}/${CMAKE_FILE#${PX4_ROOT}/}"
  cat "${tmp}" > "${CMAKE_FILE}"
  rm -f "${tmp}"
  echo "registered: ${AIRFRAME_NAME}"
}

install_file_if_needed "${AIRFRAME_SRC}" "${AIRFRAME_DST}"
install_model_tree
register_airframe

echo "F250 PX4 overlay check/install complete: ${PX4_ROOT}"
