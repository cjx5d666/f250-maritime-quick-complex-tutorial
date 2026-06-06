#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
PKG="${ROOT}/catkin_ws/src/f250_maritime_uav_sim"
SCRIPTS="${PKG}/scripts"

fail() {
  echo "check_package: $*" >&2
  exit 2
}

[ -d "${PKG}" ] || fail "missing package: ${PKG}"
[ -d "${ROOT}/data/map_authority/p0_p8_hard_requirement_20260530" ] || fail "missing map authority"
[ -d "${ROOT}/evidence/expected_route" ] || fail "missing expected route evidence"
[ -d "${ROOT}/evidence/expected_fc_3_10" ] || fail "missing expected FC 3.10 evidence"
[ -f "${ROOT}/README.md" ] || fail "missing README.md"
[ -f "${ROOT}/VALIDATION.md" ] || fail "missing VALIDATION.md"
[ -f "${ROOT}/.gitignore" ] || fail "missing .gitignore"
[ -f "${ROOT}/.gitattributes" ] || fail "missing .gitattributes"
[ -x "${ROOT}/scripts/install_px4_overlay.sh" ] || fail "missing executable scripts/install_px4_overlay.sh"
[ -f "${ROOT}/px4_overlay/ROMFS/px4fmu_common/init.d-posix/airframes/10020_gazebo-classic_f250" ] || fail "missing F250 PX4 airframe overlay"
[ -d "${ROOT}/px4_overlay/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/f250" ] || fail "missing F250 PX4 Gazebo model overlay"
[ -f "${PKG}/launch/f250_ego_advanced_param_px4_native_pose.xml" ] || fail "missing F250 EGO-Planner launch wrapper"

MARITIME_EGO_LAUNCH="${PKG}/launch/maritime_ego_planner.launch"
grep -Fq '$(find f250_maritime_uav_sim)/launch/f250_ego_advanced_param_px4_native_pose.xml' "${MARITIME_EGO_LAUNCH}" || \
  fail "maritime_ego_planner.launch does not include the repo-local F250 EGO launch wrapper"
if grep -Fq '$(find ego_planner)/launch/advanced_param_px4_native_pose.xml' "${MARITIME_EGO_LAUNCH}"; then
  fail "maritime_ego_planner.launch still includes the external EGO advanced launch file"
fi

if find "${ROOT}" -maxdepth 1 -type f -name "$(printf '*.tar.%s' gz)" | grep -q .; then
  find "${ROOT}" -maxdepth 1 -type f -name "$(printf '*.tar.%s' gz)" >&2
  fail "root archive found"
fi

sha_sums_file="$(printf 'SHA256SUMS.%s' txt)"
if [ -f "${ROOT}/${sha_sums_file}" ]; then
  fail "root ${sha_sums_file} is not used by the directory-style repo"
fi

if find -L "${ROOT}" -xtype l | grep -q .; then
  find -L "${ROOT}" -xtype l >&2
  fail "broken symlink found"
fi

if find "${ROOT}" -type d -name __pycache__ | grep -q .; then
  find "${ROOT}" -type d -name __pycache__ >&2
  fail "__pycache__ found"
fi

if find "${ROOT}" -type d -name .pytest_cache | grep -q .; then
  find "${ROOT}" -type d -name .pytest_cache >&2
  fail ".pytest_cache found"
fi

legacy_marker="$(printf '%s_%s' uav 2m)"
if find "${ROOT}" -path "*${legacy_marker}*" | grep -q .; then
  find "${ROOT}" -path "*${legacy_marker}*" >&2
  fail "legacy alternate-vehicle path exposed"
fi

if grep -R --binary-files=without-match --line-number "${legacy_marker}" "${ROOT}" >&2; then
  fail "legacy alternate-vehicle text exposed"
fi

if grep -R --binary-files=without-match --line-number "$(printf '/home/%s/f250_maritime_uav_sim' 'adminpc')" "${ROOT}" >&2; then
  fail "VM root hardcode found"
fi

for forbidden in \
  "$(printf 'AGENTS.%s' md)" \
  "$(printf 'WORKSPACE_STATE.%s' md)" \
  "$(printf 'PROJECT_MEMORY.%s' md)" \
  "$(printf 'tar -xz%s' f)" \
  "$(printf 'f250_%s_tutorial_20260605.tar.%s' 'quick_complex' gz)"; do
  if grep -R --binary-files=without-match --line-number "${forbidden}" "${ROOT}" >&2; then
    fail "forbidden public-package marker found: ${forbidden}"
  fi
done

CMAKE="${PKG}/CMakeLists.txt"
missing_cmake_scripts=()
while IFS= read -r relpath; do
  if [ ! -e "${PKG}/${relpath}" ]; then
    missing_cmake_scripts+=("${relpath}")
  fi
done < <(grep -Eo 'scripts/[A-Za-z0-9_./-]+' "${CMAKE}" | sort -u)
if [ "${#missing_cmake_scripts[@]}" -gt 0 ]; then
  printf 'missing CMake script reference: %s\n' "${missing_cmake_scripts[@]}" >&2
  fail "CMake references missing script files"
fi

mapfile -d '' shell_files < <(find "${ROOT}" -type f -name '*.sh' -print0 | sort -z)
if [ "${#shell_files[@]}" -gt 0 ]; then
  bash -n "${shell_files[@]}"
fi

PYCACHE_ROOT="$(mktemp -d)"
trap 'rm -rf "${PYCACHE_ROOT}"' EXIT
mapfile -d '' python_files < <(find "${ROOT}" -type f -name '*.py' -print0 | sort -z)
if [ "${#python_files[@]}" -gt 0 ]; then
  PYTHONPYCACHEPREFIX="${PYCACHE_ROOT}" python3 -m py_compile "${python_files[@]}"
fi

echo "F250 tutorial directory package static check passed: ${ROOT}"
