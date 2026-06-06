# Quick-Complex Final Selection

Current selected tuning: `R4_H`.

## Retained Evidence

- `final_selection.json`: selected candidate, scoring scope, safety result,
  telemetry counts, motion metrics, and compact comparison notes.
- `selection_summary.csv`: compact ranking/comparison table.
- `selected_actual_trajectory.csv`: selected run trajectory and planner command
  telemetry used for the final plot.
- `render_f250_historical_planned_vs_flown.py`: retained generator for the F250
  historical final plot.
- `f250_historical_planned_vs_flown.png`: F250 historical planned-vs-flown
  review figure.
- `runs/lidar_lidar_R4_H_r1/`, `runs/lidar_lidar_R4_H_r2/`,
  `runs/lidar_lidar_R4_H_r3/`: retained selected-family repeat evidence.

## Plot Standard

The final figure is regenerated from retained CSV/JSON inputs and the
authoritative map drawing code. The grey segment is the preparation segment;
the red segment starts at the recorded P0 task start time. Timing and scoring
still evaluate P0 to P8 only.

Do not use standalone rendered PNGs as geometry truth or trajectory
backgrounds. Future final plots should call this generator or use the same
same-axes method.
