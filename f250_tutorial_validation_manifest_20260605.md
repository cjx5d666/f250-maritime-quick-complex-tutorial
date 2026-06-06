# F250 Tutorial Validation Manifest - 2026-06-05

## Retained Evidence

- P0 hover support: `f250_tutorial_p0_hover_verify_20260605_142348`
  - F250 root: `/home/adminpc/PX4-Autopilot-v1.16.0-src-main`
  - Scene: `level_m_gps_assets_quick_complex`
  - Readiness probe: OFFBOARD, armed, P0 distance 0.089 m, speed 0.112 m/s
- P0-P8 route: `f250_tutorial_route_verify_20260605_142512`
  - `route_acceptance_summary.json`: `ok=true`
  - Progress: P8/8, P8 completed
  - Static obstacle safety: SAFE, min clearance 0.9695893370572004 m
  - Keypoint error: mean 0.27793934485364125 m, max 0.5164169495580425 m
  - Endpoint error: 0.6049447587844858 m
  - Route policy excludes Metric 3.10 and yaw pass/fail
  - Plot: `latest_planned_vs_flown.png`
  - Plot geometry: authoritative CSV/JSON map package; rendered PNG not used as geometry truth
- FC 3.10 retained formal pass: `f250_fc_3_10_settled_formal_20260605`
  - Script: `catkin_ws/src/f250_maritime_uav_sim/scripts/f250_run_fc_3_10_steady_state.sh`
  - Source P0 support: `f250_p0_hover_fc310_settled_20260605`
  - All metric windows settled: true, 40/40
  - Selected velocity speed: 2.0 m/s
  - E_pos: 0.727682193133525 %
  - E_vel_selected: 2.261625026799532 %
  - E_yaw: 0.28562253585743336 %
  - E3.10_selected: 2.261625026799532 %
  - PX4 model source resolved through the workspace symlink to `/home/adminpc/PX4-Autopilot-v1.16.0-src-main`; the F250 SDF hash matches the explicit-root runs.

## FC 3.10 Re-Runs Cleaned As Duplicates

These runs were executed with the normalized F250 tutorial entry scripts and explicit `F250_PX4_ROOT=/home/adminpc/PX4-Autopilot-v1.16.0-src-main`. They completed and all 40 metric windows settled, but did not reproduce the retained formal E3.10 value.

- `f250_tutorial_fc310_verify_20260605_143215`
  - GUI/RViz enabled
  - E3.10_selected: 7.994643534211648 %
  - Selected velocity speed: 2.0 m/s
- `f250_tutorial_fc310_verify_headless_20260605_145126`
  - `ENABLE_RVIZ=false`, `PX4_GUI=false`
  - E3.10_selected: 2.710169340749831 %
  - Selected velocity speed: 2.0 m/s
- `f250_tutorial_fc310_verify_lowload_20260605_151045`
  - `ENABLE_RVIZ=false`, `PX4_GUI=false`, `MARITIME_ENABLE_METRIC_MONITOR=false`
  - E3.10_selected: 3.38346202262051 %
  - Selected velocity speed: 2.0 m/s

Associated duplicate P0 support runs were also cleaned:

- `f250_tutorial_p0_hover_fc310_verify_20260605_143133`
- `f250_tutorial_p0_hover_fc310_verify_headless_20260605_145017`
- `f250_tutorial_p0_hover_fc310_verify_lowload_20260605_150932`

## Cleanup Notes

- Retained successful route, route plot, P0 route support, formal FC support, formal FC run, final stop log, and this manifest.
- Removed duplicate FC re-run directories and transient Python `__pycache__` directories.
- Did not modify map, route, scene, static obstacles, or cached plot geometry sources.
