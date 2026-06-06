#!/usr/bin/env bash
# Shared path resolution for the F250 tutorial entry scripts.

f250_abs_dir() {
  local path="$1"
  cd "${path}" 2>/dev/null && pwd -P
}

f250_resolve_project_root() {
  local script_dir="$1"
  local candidate

  if [ -n "${F250_PROJECT_ROOT:-}" ]; then
    candidate="$(f250_abs_dir "${F250_PROJECT_ROOT}")" || {
      echo "F250_PROJECT_ROOT does not exist: ${F250_PROJECT_ROOT}" >&2
      return 2
    }
    echo "${candidate}"
    return 0
  fi

  for candidate in \
    "${script_dir}/../../../.." \
    "${script_dir}/../../../../.." \
    "$(pwd -P)"; do
    candidate="$(f250_abs_dir "${candidate}")" || continue
    if [ -d "${candidate}/catkin_ws/src/f250_maritime_uav_sim" ]; then
      echo "${candidate}"
      return 0
    fi
  done

  echo "Unable to resolve F250 project root; set F250_PROJECT_ROOT." >&2
  return 2
}

f250_resolve_package_root() {
  local script_dir="$1"
  local project_root="$2"
  local candidate

  for candidate in \
    "${script_dir}/.." \
    "${project_root}/catkin_ws/src/f250_maritime_uav_sim"; do
    candidate="$(f250_abs_dir "${candidate}")" || continue
    if [ -f "${candidate}/package.xml" ] && grep -q '<name>f250_maritime_uav_sim</name>' "${candidate}/package.xml"; then
      echo "${candidate}"
      return 0
    fi
  done

  if command -v rospack >/dev/null 2>&1; then
    candidate="$(rospack find f250_maritime_uav_sim 2>/dev/null || true)"
    if [ -n "${candidate}" ] && [ -d "${candidate}" ]; then
      f250_abs_dir "${candidate}"
      return 0
    fi
  fi

  echo "Unable to resolve f250_maritime_uav_sim package root." >&2
  return 2
}

f250_resolve_px4_root() {
  local project_root="$1"
  local candidate

  if [ -n "${F250_PX4_ROOT:-}" ]; then
    candidate="$(f250_abs_dir "${F250_PX4_ROOT}")" || {
      echo "F250_PX4_ROOT does not exist: ${F250_PX4_ROOT}" >&2
      return 2
    }
    echo "${candidate}"
    return 0
  fi

  for candidate in \
    "${project_root}/PX4-Autopilot" \
    "${project_root}/PX4-Autopilot-src-main" \
    "${HOME:-}/PX4-Autopilot" \
    "${HOME:-}"/PX4-Autopilot*; do
    [ -n "${candidate}" ] || continue
    candidate="$(f250_abs_dir "${candidate}")" || continue
    if [ -f "${candidate}/launch/mavros_posix_sitl.launch" ]; then
      echo "${candidate}"
      return 0
    fi
  done

  echo "Unable to resolve PX4 root; set F250_PX4_ROOT." >&2
  return 2
}

f250_first_existing_or_first() {
  local first=""
  local candidate
  for candidate in "$@"; do
    [ -n "${first}" ] || first="${candidate}"
    if [ -e "${candidate}" ]; then
      echo "${candidate}"
      return 0
    fi
  done
  echo "${first}"
}
