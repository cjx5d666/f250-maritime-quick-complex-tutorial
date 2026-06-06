#!/usr/bin/env python3
import argparse
import csv
import glob
import json
import math
import os
import re
from statistics import mean


def timestamp():
    import time

    return time.strftime("%Y%m%d_%H%M%S")


def safe_float(value):
    if value is None or value == "":
        return None
    try:
        return float(value)
    except Exception:
        return None


def percentile(values, pct):
    if not values:
        return None
    ordered = sorted(float(v) for v in values)
    if len(ordered) == 1:
        return ordered[0]
    rank = (len(ordered) - 1) * pct
    lower = math.floor(rank)
    upper = math.ceil(rank)
    if lower == upper:
        return ordered[int(rank)]
    low_value = ordered[lower]
    high_value = ordered[upper]
    mix = rank - lower
    return low_value + (high_value - low_value) * mix


def spread(values):
    if not values:
        return None
    return max(values) - min(values)


def read_json(path):
    with open(path, "r", encoding="utf-8") as handle:
        return json.load(handle)


def rel_or_abs(base_dir, path_value, workspace_root=None):
    if not path_value:
        return None
    if os.path.isabs(path_value):
        return path_value
    candidates = [os.path.abspath(os.path.join(base_dir, path_value))]
    if workspace_root:
        candidates.append(os.path.abspath(os.path.join(workspace_root, path_value)))
    for candidate in candidates:
        if os.path.exists(candidate):
            return candidate
    return candidates[0]


def discover_metrics(metrics_roots, patterns):
    paths = []
    for root in metrics_roots:
        if os.path.isfile(root):
            paths.append(os.path.abspath(root))
            continue
        for pattern in patterns:
            paths.extend(glob.glob(os.path.join(root, pattern)))
    unique = []
    seen = set()
    for path in sorted(os.path.abspath(p) for p in paths):
        if path in seen:
            continue
        seen.add(path)
        unique.append(path)
    return unique


def candidate_family(candidate_id):
    if not candidate_id:
        return "unknown"
    value = str(candidate_id)
    value = re.sub(r"_\d{8}$", "", value)
    value = re.sub(r"_r\d+$", "", value)
    return value


def read_trajectory(path):
    rows = []
    if not path or not os.path.exists(path):
        return rows
    with open(path, "r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            rows.append({
                "wall_time": safe_float(row.get("wall_time")),
                "x": safe_float(row.get("x")),
                "y": safe_float(row.get("y")),
                "z": safe_float(row.get("z")),
                "vx": safe_float(row.get("vx")),
                "vy": safe_float(row.get("vy")),
                "vz": safe_float(row.get("vz")),
                "cross_track_m": safe_float(row.get("cross_track_m")),
                "along_track_m": safe_float(row.get("along_track_m")),
                "active_goal_distance_m": safe_float(row.get("active_goal_distance_m")),
                "mode": row.get("mode") or "",
                "armed": str(row.get("armed") or "").strip().lower() == "true",
                "active_goal_index": row.get("active_goal_index"),
            })
    return rows


def derive_motion(rows):
    if len(rows) < 3:
        return {}
    speed = []
    accel = []
    jerk = []
    prev = None
    prev_acc = None
    for row in rows:
        if None in (row["vx"], row["vy"], row["vz"]):
            continue
        velocity = [row["vx"], row["vy"], row["vz"]]
        speed.append(math.sqrt(sum(v * v for v in velocity)))
        if prev is not None and row["wall_time"] is not None and prev["wall_time"] is not None:
            dt = row["wall_time"] - prev["wall_time"]
            if dt > 1e-6:
                acc_vec = [(velocity[i] - prev["velocity"][i]) / dt for i in range(3)]
                acc_norm = math.sqrt(sum(v * v for v in acc_vec))
                accel.append(acc_norm)
                if prev_acc is not None:
                    jerk_vec = [(acc_vec[i] - prev_acc["acc_vec"][i]) / dt for i in range(3)]
                    jerk_norm = math.sqrt(sum(v * v for v in jerk_vec))
                    jerk.append(jerk_norm)
                prev_acc = {"acc_vec": acc_vec}
        prev = {"wall_time": row["wall_time"], "velocity": velocity}
    return {
        "speed_mean_mps": mean(speed) if speed else None,
        "speed_p95_mps": percentile(speed, 0.95),
        "speed_max_mps": max(speed) if speed else None,
        "acc_p95_mps2": percentile(accel, 0.95),
        "acc_p99_mps2": percentile(accel, 0.99),
        "acc_max_mps2": max(accel) if accel else None,
        "jerk_p95_proxy": percentile(jerk, 0.95),
        "jerk_p99_proxy": percentile(jerk, 0.99),
        "jerk_max_proxy": max(jerk) if jerk else None,
    }


def derive_smoothness(rows):
    active_goal_distances = [row["active_goal_distance_m"] for row in rows if row["active_goal_distance_m"] is not None]
    cross_track = [row["cross_track_m"] for row in rows if row["cross_track_m"] is not None]
    stop_samples = 0
    moving_samples = 0
    largest_stop_band = 0
    current_stop_band = 0
    for row in rows:
        if None in (row["vx"], row["vy"], row["vz"]):
            continue
        speed = math.sqrt(row["vx"] ** 2 + row["vy"] ** 2 + row["vz"] ** 2)
        if speed <= 0.18:
            stop_samples += 1
            current_stop_band += 1
            largest_stop_band = max(largest_stop_band, current_stop_band)
        else:
            moving_samples += 1
            current_stop_band = 0
    return {
        "cross_track_max_m": max(cross_track) if cross_track else None,
        "cross_track_p95_m": percentile(cross_track, 0.95),
        "goal_distance_p95_m": percentile(active_goal_distances, 0.95),
        "goal_distance_max_m": max(active_goal_distances) if active_goal_distances else None,
        "low_speed_sample_count": stop_samples,
        "moving_sample_count": moving_samples,
        "largest_low_speed_band_samples": largest_stop_band,
    }


def summarize_run(metrics_path):
    data = read_json(metrics_path)
    base_dir = os.path.dirname(metrics_path)
    workspace_root = os.path.abspath(os.getcwd())
    artifacts = data.get("artifacts") or {}
    trajectory_path = rel_or_abs(base_dir, artifacts.get("actual_trajectory_csv"), workspace_root=workspace_root)
    trajectory_rows = read_trajectory(trajectory_path)
    motion = derive_motion(trajectory_rows)
    smoothness = derive_smoothness(trajectory_rows)
    params = data.get("params") or {}
    clearance = data.get("clearance") or {}
    route = data.get("route") or {}
    counts = data.get("counts") or {}
    motion_defaults = {
        "speed_mean_mps": None,
        "speed_p95_mps": None,
        "speed_max_mps": None,
        "acc_p95_mps2": None,
        "acc_p99_mps2": None,
        "acc_max_mps2": None,
        "jerk_p95_proxy": None,
        "jerk_p99_proxy": None,
        "jerk_max_proxy": None,
    }
    smoothness_defaults = {
        "cross_track_max_m": None,
        "cross_track_p95_m": None,
        "goal_distance_p95_m": None,
        "goal_distance_max_m": None,
        "low_speed_sample_count": None,
        "moving_sample_count": None,
        "largest_low_speed_band_samples": None,
    }
    motion_defaults.update(motion)
    smoothness_defaults.update(smoothness)
    return {
        "metrics_path": os.path.abspath(metrics_path),
        "trajectory_csv": trajectory_path,
        "candidate_id": data.get("candidate_id"),
        "candidate_family": candidate_family(data.get("candidate_id")),
        "ok": bool(data.get("ok")),
        "summary_ok": bool(data.get("summary_ok")),
        "route_metric_ok": bool(data.get("route_metric_ok", data.get("ok"))),
        "monitor_status": data.get("monitor_status"),
        "reached_p8": bool(data.get("reached_p8")),
        "stop_reason": data.get("stop_reason"),
        "source": data.get("source"),
        "duration_sec": safe_float((data.get("task") or {}).get("duration_sec")),
        "static_min_clearance_m": safe_float(clearance.get("actual_static_min_m")),
        "dynamic_min_clearance_m": safe_float(clearance.get("actual_dynamic_min_m")),
        "static_collision": bool(clearance.get("static_collision")),
        "static_geometry_entry_count": clearance.get("static_geometry_entry_count"),
        "static_cloud_entry_count": clearance.get("static_cloud_entry_count"),
        "dynamic_geometry_entry_count": clearance.get("dynamic_geometry_entry_count"),
        "dynamic_cloud_entry_count": clearance.get("dynamic_cloud_entry_count"),
        "final_hold_seen_sec": safe_float(route.get("final_hold_seen_sec")),
        "max_active_goal_index": route.get("max_active_goal_index"),
        "waypoint_count": route.get("waypoint_count"),
        "counts_odom": counts.get("odom"),
        "counts_pos_cmd": counts.get("pos_cmd"),
        "counts_planner_cloud": counts.get("planner_cloud"),
        "counts_lidar_scan": counts.get("lidar_scan"),
        "grid_map_resolution": safe_float(params.get("grid_map_resolution")),
        "obstacles_inflation": safe_float(params.get("obstacles_inflation")),
        "planning_horizon": safe_float(params.get("planning_horizon")),
        "control_points_distance": safe_float(params.get("control_points_distance")),
        "max_vel": safe_float(params.get("max_vel")),
        "max_acc": safe_float(params.get("max_acc")),
        "max_jerk": safe_float(params.get("max_jerk")),
        "setpoint_max_speed_mps": safe_float(params.get("setpoint_max_speed_mps")),
        "setpoint_max_yaw_rate_rad_s": safe_float(params.get("setpoint_max_yaw_rate_rad_s")),
        "setpoint_max_z_rate_mps": safe_float(params.get("setpoint_max_z_rate_mps")),
        "setpoint_max_lead_m": safe_float(params.get("setpoint_max_lead_m")),
        **motion_defaults,
        **smoothness_defaults,
    }


def rank_failures(run):
    failures = []
    if not run["ok"]:
        failures.append("ok=false")
    if not run["summary_ok"]:
        failures.append("summary_ok=false")
    if not run["route_metric_ok"]:
        failures.append("route_metric_ok=false")
    if not run["reached_p8"]:
        failures.append("p8_not_reached")
    if run["static_collision"]:
        failures.append("static_collision")
    if (run["static_geometry_entry_count"] or 0) > 0:
        failures.append("static_geometry_entry_count=%s" % run["static_geometry_entry_count"])
    if (run["static_cloud_entry_count"] or 0) > 0:
        failures.append("static_cloud_entry_count=%s" % run["static_cloud_entry_count"])
    if run["max_active_goal_index"] is not None and run["waypoint_count"] is not None:
        if int(run["max_active_goal_index"]) < int(run["waypoint_count"]) - 1:
            failures.append("active_goal_index=%s_of_%s" % (run["max_active_goal_index"], int(run["waypoint_count"]) - 1))
    if run["final_hold_seen_sec"] is not None and run["final_hold_seen_sec"] < 0.2:
        failures.append("final_hold_below_0.2")
    return failures


def score_sort_key(run):
    hard_fail = int(bool(rank_failures(run)))
    duration = run["duration_sec"] if run["duration_sec"] is not None else 1e12
    jerk = run["jerk_p95_proxy"] if run["jerk_p95_proxy"] is not None else 1e12
    low_speed = run["largest_low_speed_band_samples"] if run["largest_low_speed_band_samples"] is not None else 1e12
    return (
        hard_fail,
        duration,
        jerk,
        low_speed,
        -(run["static_min_clearance_m"] if run["static_min_clearance_m"] is not None else -1e12),
    )


def aggregate_family(family, runs):
    successful = [run for run in runs if not rank_failures(run)]
    durations = [run["duration_sec"] for run in successful if run["duration_sec"] is not None]
    jerk_p95 = [run["jerk_p95_proxy"] for run in successful if run["jerk_p95_proxy"] is not None]
    low_speed = [run["largest_low_speed_band_samples"] for run in successful if run["largest_low_speed_band_samples"] is not None]
    clearances = [run["static_min_clearance_m"] for run in successful if run["static_min_clearance_m"] is not None]
    return {
        "candidate_family": family,
        "run_count": len(runs),
        "successful_run_count": len(successful),
        "all_successful": len(successful) == len(runs) and bool(runs),
        "duration_mean_sec": mean(durations) if durations else None,
        "duration_spread_sec": spread(durations),
        "worst_static_min_clearance_m": min(clearances) if clearances else None,
        "worst_jerk_p95_proxy": max(jerk_p95) if jerk_p95 else None,
        "worst_low_speed_band_samples": max(low_speed) if low_speed else None,
        "best_run": min(runs, key=score_sort_key),
        "runs": sorted(runs, key=score_sort_key),
    }


def write_json(path, payload):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True)
        handle.write("\n")


def write_csv(path, families):
    fieldnames = [
        "rank",
        "candidate_family",
        "run_count",
        "successful_run_count",
        "all_successful",
        "duration_mean_sec",
        "duration_spread_sec",
        "worst_static_min_clearance_m",
        "worst_jerk_p95_proxy",
        "worst_low_speed_band_samples",
        "best_run_candidate_id",
        "best_run_duration_sec",
        "best_run_static_min_clearance_m",
        "best_run_jerk_p95_proxy",
        "best_run_cross_track_max_m",
        "best_run_low_speed_band_samples",
        "best_run_failures",
    ]
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for rank, family_row in enumerate(families, start=1):
            best = family_row["best_run"]
            writer.writerow({
                "rank": rank,
                "candidate_family": family_row["candidate_family"],
                "run_count": family_row["run_count"],
                "successful_run_count": family_row["successful_run_count"],
                "all_successful": family_row["all_successful"],
                "duration_mean_sec": family_row["duration_mean_sec"],
                "duration_spread_sec": family_row["duration_spread_sec"],
                "worst_static_min_clearance_m": family_row["worst_static_min_clearance_m"],
                "worst_jerk_p95_proxy": family_row["worst_jerk_p95_proxy"],
                "worst_low_speed_band_samples": family_row["worst_low_speed_band_samples"],
                "best_run_candidate_id": best["candidate_id"],
                "best_run_duration_sec": best["duration_sec"],
                "best_run_static_min_clearance_m": best["static_min_clearance_m"],
                "best_run_jerk_p95_proxy": best["jerk_p95_proxy"],
                "best_run_cross_track_max_m": best["cross_track_max_m"],
                "best_run_low_speed_band_samples": best["largest_low_speed_band_samples"],
                "best_run_failures": "; ".join(rank_failures(best)),
            })


def parse_args():
    parser = argparse.ArgumentParser(description="Aggregate/rank F250 quick-complex candidate metrics.")
    parser.add_argument(
        "metrics_roots",
        nargs="+",
        help="One or more directories or metrics.json files.",
    )
    parser.add_argument(
        "--pattern",
        action="append",
        default=["*/metrics.json", "metrics.json"],
        help="Glob pattern relative to a metrics root directory. Repeatable.",
    )
    parser.add_argument(
        "--output-dir",
        default=None,
        help="Output directory for aggregate JSON/CSV. Defaults to the first metrics root when it is a directory.",
    )
    parser.add_argument(
        "--output-prefix",
        default="f250_quick_complex_candidate_summary",
        help="Output file prefix for JSON/CSV artifacts.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    metrics_paths = discover_metrics(args.metrics_roots, args.pattern)
    if not metrics_paths:
        raise SystemExit("no metrics.json files found")

    output_dir = args.output_dir
    if output_dir is None:
        first_root = args.metrics_roots[0]
        output_dir = first_root if os.path.isdir(first_root) else os.path.dirname(os.path.abspath(first_root))

    runs = [summarize_run(path) for path in metrics_paths]
    family_map = {}
    for run in runs:
        family_map.setdefault(run["candidate_family"], []).append(run)

    families = [aggregate_family(family, family_runs) for family, family_runs in family_map.items()]
    families.sort(key=lambda row: score_sort_key(row["best_run"]))

    payload = {
        "generated_at": timestamp(),
        "family_count": len(families),
        "run_count": len(runs),
        "families": families,
        "runs": sorted(runs, key=score_sort_key),
    }

    json_path = os.path.join(output_dir, "%s.json" % args.output_prefix)
    csv_path = os.path.join(output_dir, "%s.csv" % args.output_prefix)
    write_json(json_path, payload)
    write_csv(csv_path, families)
    print(json_path)
    print(csv_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
