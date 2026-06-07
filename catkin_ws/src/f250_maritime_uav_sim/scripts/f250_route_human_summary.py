#!/usr/bin/env python3
import argparse
import csv
import json
import math
import os
import sys
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)

from maritime_clearance import evaluate_clearance
from maritime_metric_core import MetricAccumulator, match_waypoint_index, run_offline
from maritime_scene_utils import load_scene, scene_waypoints


CSV_FIELDS = [
    "wall_time", "ros_time", "x", "y", "z", "vx", "vy", "vz", "actual_yaw",
    "expected_x", "expected_y", "expected_z", "expected_vx", "expected_vy", "expected_vz",
    "expected_ax", "expected_ay", "expected_az", "expected_yaw", "expected_yaw_dot",
    "pos_cmd_age_sec", "setpoint_x", "setpoint_y", "setpoint_z", "setpoint_yaw",
    "setpoint_age_sec", "position_error_to_expected_m", "position_error_to_setpoint_m",
    "velocity_error_to_expected_m", "yaw_error_rad", "yaw_error_to_setpoint_rad",
    "mode", "armed", "active_goal_index", "active_goal_x", "active_goal_y",
    "active_goal_z", "active_goal_distance_m", "cross_track_m", "along_track_m",
]

ROUTE_POLICY = {
    "date": "2026-06-04",
    "vehicle": "f250",
    "fixed_route": "maritime_quick_complex_p0_p8_hard_requirement_20260530",
    "baseline": "F250_R4_H",
    "route_acceptance_excludes_metric_3_10": True,
    "route_acceptance_excludes_yaw": True,
    "dynamic_boat_clearance_role": "telemetry_only",
    "route_acceptance_components": [
        "p8_completion",
        "static_obstacle_safety",
        "metric_3_6_keypoint_error",
        "metric_3_7_static_obstacle_gate",
        "metric_3_8_route_progress",
        "metric_3_9_endpoint_error",
    ],
}


def read_json(path, default=None):
    if not path or not os.path.exists(path):
        return default
    with open(path, "r", encoding="utf-8") as handle:
        return json.load(handle)


def write_json(path, payload):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True)
        handle.write("\n")


def env_quote(value):
    text = "" if value is None else str(value)
    if text == "":
        return ""
    return text.replace("\n", "_").replace(" ", "_")


def safe_float(value):
    if value is None or value == "":
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def bool_text(value):
    return "true" if bool(value) else "false"


def fmt_m(value):
    if value is None:
        return "--"
    return "%.3f m" % float(value)


def fmt_float(value):
    if value is None:
        return ""
    return "%.6g" % float(value)


def route_line(status):
    p8_text = "P8 done" if status["p8_completed"] else "P8 wait"
    key_text = "%s / %s" % (
        fmt_m(status["keypoint_error_mean_m"]),
        fmt_m(status["keypoint_error_max_m"]),
    )
    return "Route  %-4s | %-7s | Static %-7s | Key %s | End %s" % (
        status["progress"],
        p8_text,
        status["static"],
        key_text,
        fmt_m(status["endpoint_error_m"]),
    )


def route_state_text(state):
    text = str(state or "").strip().lower()
    if text in ("complete", "completed", "dry_run_complete"):
        return "COMPLETE"
    if text in ("failed", "failure", "recorder_failed"):
        return "FAILED"
    if text == "timeout":
        return "TIMEOUT"
    return "RUNNING"


def route_keypoint_text(status):
    mean_value = status.get("keypoint_error_mean_m")
    max_value = status.get("keypoint_error_max_m")
    if mean_value is None and max_value is None:
        return "waiting"
    return "mean %s   max %s" % (fmt_m(mean_value), fmt_m(max_value))


def route_endpoint_text(status):
    if not status.get("p8_completed") or status.get("endpoint_error_m") is None:
        return "waiting for P8"
    return fmt_m(status.get("endpoint_error_m"))


def route_display_block(status, state):
    progress_index = int(status.get("progress_index") or 0)
    final_index = int(status.get("final_index") or 0)
    progress_text = "P%d reached   %d / %d" % (progress_index, progress_index, final_index)
    lines = [
        "F250 P0 -> P8 Route",
        "",
        "%-15s %s" % ("Progress", progress_text),
        "%-15s %s" % ("Static Safety", status.get("static") or "UNKNOWN"),
        "%-15s %s" % ("Keypoint Error", route_keypoint_text(status)),
        "%-15s %s" % ("Endpoint Error", route_endpoint_text(status)),
        "",
        "%-15s %s" % ("Status", route_state_text(state)),
    ]
    return "\n".join(lines)


def terminal_header_block():
    return "\n".join([
        "F250 P0-P8 Route started",
        "checks OK",
        "plan: P0 -> P8, LiDAR perception, R4_H route params",
        "",
        "===== route progress =====",
        "",
        "[route]",
        "START from P0",
    ])


def waypoint_name(index):
    return "P%d" % int(index)


def waypoint_xyz(waypoints, index):
    if index < 0 or index >= len(waypoints):
        return None
    pos = waypoints[index].get("position", [])
    if len(pos) < 3:
        return None
    return [float(pos[0]), float(pos[1]), float(pos[2])]


def waypoint_start_block(waypoints, index):
    pos = waypoint_xyz(waypoints, index)
    lines = ["[waypoint %s]" % waypoint_name(index)]
    if pos is not None:
        lines.append("target x=%.3f y=%.3f z=%.3f" % (pos[0], pos[1], pos[2]))
    else:
        lines.append("target unavailable")
    lines.append("START")
    return "\n".join(lines)


def waypoint_stat(metric_summary, index):
    for item in (metric_summary or {}).get("waypoints") or []:
        try:
            if int(item.get("index", -1)) == int(index):
                return item
        except (TypeError, ValueError):
            continue
    return {}


def waypoint_reached_block(metric_summary, status, index):
    stat = waypoint_stat(metric_summary, index)
    lines = ["[waypoint %s]" % waypoint_name(index), "reached"]
    if index == int(status.get("final_index") or 0):
        lines.append("endpoint_error=%s" % fmt_m(status.get("endpoint_error_m")))
    elif index > 0:
        lines.append("keypoint_error=%s" % fmt_m(safe_float(stat.get("nearest_distance_m"))))
    else:
        lines.append("distance=%s" % fmt_m(safe_float(stat.get("nearest_distance_m"))))
    lines.append("END")
    return "\n".join(lines)


def static_details(clearance_static):
    actual = ((((clearance_static or {}).get("metrics") or {}).get("actual_trajectory") or {}).get("static") or {})
    return {
        "geometry_entry_count": int(actual.get("geometry_entry_count") or 0),
        "cloud_entry_count": int(actual.get("cloud_entry_count") or 0),
        "collision": bool(actual.get("collision")),
    }


def route_final_block(status, state, summary=None, clearance_static=None):
    details = static_details(clearance_static)
    duration = safe_float(((summary or {}).get("task") or {}).get("duration_sec"))
    lines = [
        "===== safety =====",
        "static safety %s" % (status.get("static") or "UNKNOWN"),
        "static geometry entry %d" % details["geometry_entry_count"],
        "static cloud entry %d" % details["cloud_entry_count"],
        "static collision %s" % bool_text(details["collision"]),
        "",
        "===== FINAL =====",
        "progress P%d reached %d/%d" % (
            int(status.get("progress_index") or 0),
            int(status.get("progress_index") or 0),
            int(status.get("final_index") or 0),
        ),
        "static safety %s" % (status.get("static") or "UNKNOWN"),
        "keypoint error mean %s max %s" % (
            fmt_m(status.get("keypoint_error_mean_m")),
            fmt_m(status.get("keypoint_error_max_m")),
        ),
        "endpoint error %s" % fmt_m(status.get("endpoint_error_m")),
    ]
    if duration is not None:
        lines.append("duration %.3f s" % duration)
    if route_state_text(state) != "COMPLETE":
        lines.append("status %s" % route_state_text(state))
    return "\n".join(lines)


def waypoint_count(metric_summary, fallback=9):
    route = (metric_summary or {}).get("route") or {}
    count = route.get("waypoint_count")
    if count is None:
        waypoints = (metric_summary or {}).get("waypoints") or []
        count = len(waypoints) if waypoints else fallback
    try:
        return int(count)
    except (TypeError, ValueError):
        return fallback


def keypoint_errors(metric_summary):
    waypoints = (metric_summary or {}).get("waypoints") or []
    if not waypoints:
        return None, None
    last_index = max(int(stat.get("index", -1)) for stat in waypoints)
    values = []
    for stat in waypoints:
        index = int(stat.get("index", -1))
        if 1 <= index <= last_index - 1:
            value = safe_float(stat.get("nearest_distance_m"))
            if value is not None:
                values.append(value)
    if not values:
        return None, None
    return sum(values) / len(values), max(values)


def static_safe_from_clearance(clearance_static):
    actual = ((((clearance_static or {}).get("metrics") or {}).get("actual_trajectory") or {}).get("static") or {})
    if not actual:
        return None
    collision = bool(actual.get("collision"))
    geometry_entries = int(actual.get("geometry_entry_count") or 0)
    cloud_entries = int(actual.get("cloud_entry_count") or 0)
    return (not collision) and geometry_entries == 0 and cloud_entries == 0


def static_min_from_clearance(clearance_static):
    actual = ((((clearance_static or {}).get("metrics") or {}).get("actual_trajectory") or {}).get("static") or {})
    return safe_float(actual.get("min_clearance_m"))


def dynamic_telemetry(clearance_dynamic):
    actual = ((((clearance_dynamic or {}).get("metrics") or {}).get("actual_trajectory") or {}).get("dynamic") or {})
    return {
        "min_clearance_m": safe_float(actual.get("min_clearance_m")),
        "geometry_entry_count": int(actual.get("geometry_entry_count") or actual.get("dynamic_geometry_entry_count") or 0),
        "cloud_entry_count": int(actual.get("cloud_entry_count") or actual.get("dynamic_cloud_entry_count") or 0),
        "role": "telemetry_only",
    }


def terminal_status(metric_summary, summary=None, clearance_static=None):
    metric_summary = metric_summary or {}
    summary = summary or {}
    count = waypoint_count(metric_summary)
    final_index = max(0, count - 1)
    progress_index = metric_summary.get("max_active_goal_index")
    if progress_index is None:
        progress_index = metric_summary.get("active_goal_index")
    route = summary.get("route") or {}
    if progress_index is None:
        progress_index = route.get("max_active_goal_index")
    try:
        progress_index = int(progress_index)
    except (TypeError, ValueError):
        progress_index = 0
    progress_index = max(0, min(final_index, progress_index))

    p8_completed = bool(metric_summary.get("p8_completed") or route.get("final_reached_ever"))
    if p8_completed:
        progress_index = final_index

    m37 = metric_summary.get("metric_3_7") or {}
    static_safe = m37.get("safe_so_far")
    if static_safe is None:
        static_safe = static_safe_from_clearance(clearance_static)
    static_text = "UNKNOWN"
    if static_safe is not None:
        static_text = "SAFE" if bool(static_safe) else "UNSAFE"

    key_mean, key_max = keypoint_errors(metric_summary)
    m39 = metric_summary.get("metric_3_9") or {}
    final_err = safe_float(m39.get("final_error_m"))
    if final_err is None:
        task = summary.get("task") or {}
        final_err = safe_float(task.get("p8_nearest_distance_m"))

    status = {
        "progress_index": progress_index,
        "final_index": final_index,
        "progress": "P%d/%d" % (progress_index, final_index),
        "p8_completed": p8_completed,
        "static": static_text,
        "static_safe": static_safe,
        "static_min_clearance_m": static_min_from_clearance(clearance_static),
        "keypoint_error_mean_m": key_mean,
        "keypoint_error_max_m": key_max,
        "endpoint_error_m": final_err,
    }
    status["line"] = route_line(status)
    return status


def route_acceptance(metric_summary, summary=None, clearance_static=None):
    metric_summary = metric_summary or {}
    summary = summary or {}
    status = terminal_status(metric_summary, summary, clearance_static)
    m36 = metric_summary.get("metric_3_6") or {}
    m37 = metric_summary.get("metric_3_7") or {}
    m38 = metric_summary.get("metric_3_8") or {}
    m39 = metric_summary.get("metric_3_9") or {}
    summary_ok = bool(summary.get("ok", True))
    p8_completed = bool(status["p8_completed"])
    static_safe = bool(status["static_safe"])
    components = {
        "p8_completion": p8_completed,
        "static_obstacle_safety": static_safe,
        "metric_3_6_keypoint_error": bool(m36.get("passed")),
        "metric_3_7_static_obstacle_gate": bool(m37.get("passed")),
        "metric_3_8_route_progress": bool(m38.get("passed")),
        "metric_3_9_endpoint_error": bool(m39.get("passed")),
        "recorder_summary": summary_ok,
    }
    ok = all(components.values())
    return ok, components, status


def write_env_file(path, fields):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", encoding="utf-8") as handle:
        for key, value in fields:
            handle.write("%s=%s\n" % (key, env_quote(value)))


def write_status_files(args, state, metric_summary, summary=None, clearance_static=None):
    ok, components, status = route_acceptance(metric_summary, summary, clearance_static)
    fields = [
        ("state", state),
        ("updated_at", time.strftime("%Y-%m-%dT%H:%M:%S%z")),
        ("run_dir", args.run_dir),
        ("run_label", args.run_label),
        ("vehicle", "f250"),
        ("progress", status["progress"]),
        ("p8_completed", bool_text(status["p8_completed"])),
        ("static_obstacle_safety", status["static"]),
        ("keypoint_error_mean_m", fmt_float(status["keypoint_error_mean_m"])),
        ("keypoint_error_max_m", fmt_float(status["keypoint_error_max_m"])),
        ("endpoint_error_m", fmt_float(status["endpoint_error_m"])),
        ("route_acceptance_ok", bool_text(ok)),
        ("route_acceptance_excludes_metric_3_10", "true"),
        ("route_acceptance_excludes_yaw", "true"),
        ("dynamic_boat_clearance_role", "telemetry_only"),
        ("route_terminal_log", args.terminal_log),
        ("route_acceptance_summary_json", os.path.join(args.run_dir, "route_acceptance_summary.json")),
    ]
    if args.route_status_env:
        write_env_file(args.route_status_env, fields)
    if args.status_env:
        write_env_file(args.status_env, fields)
    return ok, components, status


def append_terminal_line(path, line):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    text = line.rstrip("\n")
    try:
        with open(path, "r", encoding="utf-8") as handle:
            previous = [item.rstrip("\n") for item in handle.readlines() if item.rstrip("\n")]
        if previous and previous[-1] == text:
            return
    except OSError:
        pass
    with open(path, "a", encoding="utf-8") as handle:
        handle.write(text + "\n")


def append_terminal_block(path, block):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    text = block.rstrip("\n")
    with open(path, "a", encoding="utf-8") as handle:
        handle.write(text + "\n\n")


def append_terminal_over(path):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    try:
        with open(path, "r", encoding="utf-8") as handle:
            lines = [item.rstrip("\n") for item in handle.readlines() if item.rstrip("\n")]
        if lines and lines[-1] == "OVER":
            return
    except OSError:
        pass
    with open(path, "a", encoding="utf-8") as handle:
        handle.write("OVER\n")


def write_route_acceptance_summary(args, metric_summary, summary, clearance_static, clearance_dynamic, metrics=None):
    ok, components, status = route_acceptance(metric_summary, summary, clearance_static)
    payload = {
        "ok": ok,
        "vehicle": "f250",
        "run_dir": args.run_dir,
        "run_label": args.run_label,
        "terminal": status,
        "components": components,
        "policy": ROUTE_POLICY,
        "static_obstacle_safety": {
            "status": status["static"],
            "min_clearance_m": status["static_min_clearance_m"],
        },
        "dynamic_boat_clearance": dynamic_telemetry(clearance_dynamic),
        "metric_summary_json": os.path.join(args.run_dir, "metric_summary.json"),
        "metric_waypoints_csv": os.path.join(args.run_dir, "metric_waypoints.csv"),
        "metrics_json": os.path.join(args.run_dir, "metrics.json"),
    }
    if metrics is not None:
        payload["postprocess_ok"] = bool(metrics.get("ok"))
    out_path = os.path.join(args.run_dir, "route_acceptance_summary.json")
    write_json(out_path, payload)
    return payload


def update_artifact_policy(args, metric_summary, summary, clearance_static, clearance_dynamic):
    metrics_path = os.path.join(args.run_dir, "metrics.json")
    metrics = read_json(metrics_path, default=None)
    acceptance = write_route_acceptance_summary(
        args, metric_summary, summary or {}, clearance_static or {}, clearance_dynamic or {}, metrics)

    summary_path = os.path.join(args.run_dir, "summary.json")
    if summary is not None:
        summary["route_acceptance_policy"] = ROUTE_POLICY
        summary["route_terminal"] = acceptance["terminal"]
        summary["route_acceptance_ok"] = acceptance["ok"]
        write_json(summary_path, summary)

    if metrics is not None:
        policy = metrics.get("metric_policy") if isinstance(metrics.get("metric_policy"), dict) else {}
        policy.update({
            "date": ROUTE_POLICY["date"],
            "route_ok_excludes_metric_3_10": True,
            "route_ok_excludes_yaw": True,
            "dynamic_boat_clearance_role": "telemetry_only",
            "components": ROUTE_POLICY["route_acceptance_components"],
        })
        metrics["metric_policy"] = policy
        metrics["route_terminal"] = acceptance["terminal"]
        metrics["route_acceptance_summary_json"] = os.path.join(args.run_dir, "route_acceptance_summary.json")
        write_json(metrics_path, metrics)
    return acceptance


def write_params_json(path, args):
    payload = {
        "description": "F250 human P0-P8 route script using current R4_H defaults",
        "vehicle": "f250",
        "family_id": "R4_H",
        "scene_level": "level_m_gps_assets_quick_complex",
        "scene_config": os.path.abspath(args.scene_config),
        "perception_source": "lidar",
        "dynamic_mode": args.dynamic_mode,
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
            "grid_map_resolution": 0.35,
        },
        "route_acceptance_policy": ROUTE_POLICY,
    }
    write_json(path, payload)


def yaw_from_waypoint(waypoint):
    try:
        return float(waypoint.get("yaw", 0.0))
    except (TypeError, ValueError):
        return 0.0


def make_trajectory_row(time_sec, waypoint, index, total_along):
    pos = [float(value) for value in waypoint["position"][:3]]
    yaw = yaw_from_waypoint(waypoint)
    return {
        "wall_time": time_sec,
        "ros_time": time_sec,
        "x": pos[0],
        "y": pos[1],
        "z": pos[2],
        "vx": 0.0,
        "vy": 0.0,
        "vz": 0.0,
        "actual_yaw": yaw,
        "expected_x": pos[0],
        "expected_y": pos[1],
        "expected_z": pos[2],
        "expected_vx": 0.0,
        "expected_vy": 0.0,
        "expected_vz": 0.0,
        "expected_ax": 0.0,
        "expected_ay": 0.0,
        "expected_az": 0.0,
        "expected_yaw": yaw,
        "expected_yaw_dot": 0.0,
        "pos_cmd_age_sec": 0.0,
        "setpoint_x": pos[0],
        "setpoint_y": pos[1],
        "setpoint_z": pos[2],
        "setpoint_yaw": yaw,
        "setpoint_age_sec": 0.0,
        "position_error_to_expected_m": 0.0,
        "position_error_to_setpoint_m": 0.0,
        "velocity_error_to_expected_m": 0.0,
        "yaw_error_rad": 0.0,
        "yaw_error_to_setpoint_rad": 0.0,
        "mode": "OFFBOARD",
        "armed": "true",
        "active_goal_index": index,
        "active_goal_x": pos[0],
        "active_goal_y": pos[1],
        "active_goal_z": pos[2],
        "active_goal_distance_m": 0.0,
        "cross_track_m": 0.0,
        "along_track_m": total_along,
    }


def waypoint_distance(a, b):
    return math.sqrt(sum((float(a[i]) - float(b[i])) ** 2 for i in range(3)))


def synthesize_trajectory(scene_config, output_csv):
    scene = load_scene(scene_config)
    waypoints = scene_waypoints(scene)
    if not waypoints:
        raise RuntimeError("scene has no waypoints")
    rows = []
    route_advancements = []
    total_along = 0.0
    time_sec = 0.0
    previous_pos = None
    for index, waypoint in enumerate(waypoints):
        pos = waypoint["position"][:3]
        if previous_pos is not None:
            total_along += waypoint_distance(previous_pos, pos)
        rows.append(make_trajectory_row(time_sec, waypoint, index, total_along))
        route_advancements.append({
            "index": index,
            "name": waypoint.get("name", "waypoint_%d" % index),
            "position": [float(value) for value in pos],
            "wall_time": time_sec,
        })
        time_sec += 0.4
        rows.append(make_trajectory_row(time_sec, waypoint, index, total_along))
        time_sec += 0.6
        previous_pos = pos
    final_hold = float((scene.get("acceptance") or {}).get("final_zone_hold_sec", 0.2))
    time_sec += final_hold + 0.4
    rows.append(make_trajectory_row(time_sec, waypoints[-1], len(waypoints) - 1, total_along))

    os.makedirs(os.path.dirname(os.path.abspath(output_csv)), exist_ok=True)
    with open(output_csv, "w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)
    return scene, waypoints, rows, route_advancements, final_hold


def write_synthetic_summary(args, scene, waypoints, rows, route_advancements, final_hold):
    output_csv = os.path.join(args.run_dir, "actual_trajectory.csv")
    payload = {
        "ok": True,
        "errors": [],
        "duration_requested_sec": args.max_duration_sec,
        "duration_observed_sec": rows[-1]["wall_time"] if rows else 0.0,
        "scene": os.path.abspath(args.scene_config),
        "state": {
            "connected": True,
            "mode": "OFFBOARD",
            "armed": True,
            "seen_offboard": True,
            "seen_armed": True,
            "seen_disarmed_after_armed": False,
        },
        "counts": {
            "odom": len(rows),
            "pos_cmd": len(rows),
            "setpoint": len(rows),
            "state": len(rows),
            "active_goal": len(waypoints),
            "planner_cloud": 1,
            "raw_cloud": 1,
            "lidar_scan": 1,
            "landing_status": 0,
        },
        "clouds": {
            "planner_cloud_topic": "/maritime/obstacles_cloud",
            "raw_cloud_topic": "/maritime/lidar_points",
            "lidar_scan_topic": "/maritime/lidar_scan",
            "max_planner_cloud_points": 1,
            "max_raw_cloud_points": 1,
            "max_lidar_scan_ranges": 1,
        },
        "route": {
            "active_goal_advancements": route_advancements,
            "max_active_goal_index": len(waypoints) - 1,
            "waypoint_count": len(waypoints),
            "final_reached_ever": True,
            "final_hold_required_sec": final_hold,
            "final_hold_seen_sec": final_hold + 0.4,
        },
        "metrics": {
            "best_along_track_m": rows[-1]["along_track_m"] if rows else None,
            "max_cross_track_m": 0.0,
            "cross_track_samples": len(rows),
            "max_cross_track_allowed_m": 120.0,
            "last_active_goal_distance_m": 0.0,
            "min_active_goal_distance_m": 0.0,
            "max_altitude_m": max((row["z"] for row in rows), default=None),
            "min_altitude_armed_m": min((row["z"] for row in rows), default=None),
            "max_stall_allowed_sec": 90.0,
            "max_stall_seen_sec": 0.0,
        },
        "task": {
            "duration_sec": rows[-1]["wall_time"] if rows else 0.0,
            "cross_track_max_m": 0.0,
            "p0_wall_sec": rows[0]["wall_time"] if rows else None,
            "p0_nearest_distance_m": 0.0,
            "p0_nearest_wall_sec": rows[0]["wall_time"] if rows else None,
            "p0_first_within_radius_m": float(waypoints[0].get("radius", 1.0)),
            "p8_nearest_distance_m": 0.0,
            "p8_nearest_wall_sec": rows[-1]["wall_time"] if rows else None,
            "p8_first_within_wall_sec": rows[-1]["wall_time"] if rows else None,
            "p8_first_within_radius_m": float(waypoints[-1].get("radius", 1.0)),
        },
        "landing": {
            "required": False,
            "require_disarmed": False,
            "status_messages": 0,
            "states_seen": [],
            "goaround_seen": False,
            "goaround_reasons": [],
            "touchdown_seen": False,
            "disarm_success_seen": False,
            "last_status": None,
        },
        "clearance": None,
        "clearance_failure_reasons": [],
        "output_csv": os.path.abspath(output_csv),
        "stop_reason": "final_hold_reached",
        "route_acceptance_policy": ROUTE_POLICY,
    }
    write_json(os.path.join(args.run_dir, "summary.json"), payload)
    return payload


def compose_metrics_json(args, summary, metric_summary, clearance_static, clearance_dynamic):
    actual_static = ((((clearance_static.get("metrics") or {}).get("actual_trajectory") or {}).get("static")) or {})
    actual_dynamic = ((((clearance_dynamic.get("metrics") or {}).get("actual_trajectory") or {}).get("dynamic")) or {})
    ok, components, status = route_acceptance(metric_summary, summary, clearance_static)
    payload = {
        "candidate_id": args.run_label,
        "run_dir": args.run_dir,
        "vehicle": "f250",
        "source": "lidar",
        "params": (read_json(os.path.join(args.run_dir, "params.json"), {}) or {}).get("params", {}),
        "summary_ok": bool(summary.get("ok")),
        "monitor_status": 0 if bool(summary.get("ok")) else 2,
        "route_metric_ok": bool(
            components["metric_3_6_keypoint_error"]
            and components["metric_3_7_static_obstacle_gate"]
            and components["metric_3_8_route_progress"]
            and components["metric_3_9_endpoint_error"]
        ),
        "ok": ok,
        "reached_p8": bool(status["p8_completed"]),
        "stop_reason": summary.get("stop_reason"),
        "state": summary.get("state"),
        "counts": summary.get("counts"),
        "clouds": summary.get("clouds"),
        "route": summary.get("route"),
        "task": summary.get("task"),
        "clearance": {
            "actual_static_min_m": actual_static.get("min_clearance_m"),
            "actual_static_min_cloud_distance_m": actual_static.get("min_cloud_distance_m"),
            "actual_dynamic_min_m": actual_dynamic.get("min_clearance_m"),
            "actual_dynamic_min_cloud_distance_m": actual_dynamic.get("min_cloud_distance_m"),
            "static_collision": actual_static.get("collision"),
            "static_geometry_entry_count": actual_static.get("geometry_entry_count"),
            "static_cloud_entry_count": actual_static.get("cloud_entry_count"),
            "dynamic_geometry_entry_count": actual_dynamic.get("geometry_entry_count"),
            "dynamic_cloud_entry_count": actual_dynamic.get("cloud_entry_count"),
            "dynamic_role": "telemetry_only",
        },
        "formal_metrics": metric_summary,
        "metric_policy": {
            "date": ROUTE_POLICY["date"],
            "route_ok_excludes_metric_3_10": True,
            "route_ok_excludes_yaw": True,
            "dynamic_boat_clearance_role": "telemetry_only",
            "components": ROUTE_POLICY["route_acceptance_components"],
        },
        "failures": [] if ok else [name for name, passed in components.items() if not passed],
    }
    write_json(os.path.join(args.run_dir, "metrics.json"), payload)
    return payload


def dry_run(args):
    os.makedirs(args.run_dir, exist_ok=True)
    write_params_json(os.path.join(args.run_dir, "params.json"), args)
    scene, waypoints, rows, route_advancements, final_hold = synthesize_trajectory(
        args.scene_config, os.path.join(args.run_dir, "actual_trajectory.csv"))
    summary = write_synthetic_summary(args, scene, waypoints, rows, route_advancements, final_hold)
    metric_summary = run_offline(
        args.scene_config,
        os.path.join(args.run_dir, "actual_trajectory.csv"),
        args.run_dir,
        run_label=args.run_label,
        dynamic_mode=args.dynamic_mode,
        actual_filter="armed_offboard",
        clearance_sample_period_sec=0.0,
    )
    clearance_static = evaluate_clearance(
        args.scene_config,
        trajectory_csv=os.path.join(args.run_dir, "actual_trajectory.csv"),
        dynamic_mode="none",
        actual_filter="armed_offboard",
        sample_spacing_m=2.0,
        dynamic_samples=8,
    )
    write_json(os.path.join(args.run_dir, "clearance_static_gate.json"), clearance_static)
    clearance_dynamic = evaluate_clearance(
        args.scene_config,
        trajectory_csv=os.path.join(args.run_dir, "actual_trajectory.csv"),
        dynamic_mode=args.dynamic_mode,
        actual_filter="armed_offboard",
        sample_spacing_m=2.0,
        dynamic_samples=8,
    )
    write_json(os.path.join(args.run_dir, "clearance_dynamic_telemetry.json"), clearance_dynamic)
    compose_metrics_json(args, summary, metric_summary, clearance_static, clearance_dynamic)
    acceptance = update_artifact_policy(args, metric_summary, summary, clearance_static, clearance_dynamic)
    write_status_files(args, "complete", metric_summary, summary, clearance_static)
    state = "complete" if acceptance["ok"] else "failed"
    append_terminal_block(args.terminal_log, terminal_header_block())
    for index, _waypoint in enumerate(waypoints):
        if index > 0:
            append_terminal_block(args.terminal_log, waypoint_start_block(waypoints, index))
        append_terminal_block(args.terminal_log, waypoint_reached_block(metric_summary, acceptance["terminal"], index))
    block = route_final_block(acceptance["terminal"], state, summary, clearance_static)
    append_terminal_block(args.terminal_log, block)
    append_terminal_over(args.terminal_log)
    print(block, flush=True)
    print("OVER", flush=True)
    return 0 if acceptance["ok"] else 1


def live_monitor(args):
    import rospy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry

    class Monitor:
        def __init__(self):
            self.waypoints = scene_waypoints(load_scene(args.scene_config))
            self.accumulator = MetricAccumulator(
                args.scene_config,
                dynamic_mode=args.dynamic_mode,
                clearance_sample_period_sec=args.clearance_sample_period_sec,
            )
            self.active_goal_index = None
            self.last_progress_index = None
            self.last_p8_completed = None
            self.last_started_index = None
            self.start_wall = time.time()
            self.deadline_wall = self.start_wall + float(args.max_duration_sec)
            self.exit_code = 1
            append_terminal_block(args.terminal_log, terminal_header_block())
            self.timer = rospy.Timer(rospy.Duration(max(0.2, args.display_period_sec)), self.timer_cb)
            self.odom_sub = rospy.Subscriber(args.odom_topic, Odometry, self.odom_cb, queue_size=1)
            self.active_sub = rospy.Subscriber(args.active_goal_topic, PoseStamped, self.active_goal_cb, queue_size=1)

        def active_goal_cb(self, msg):
            pos = msg.pose.position
            position = [float(pos.x), float(pos.y), float(pos.z)]
            matched = match_waypoint_index(position, self.accumulator.stats, tolerance_m=0.35)
            if matched is not None:
                if matched > 0 and matched != self.last_started_index:
                    append_terminal_block(args.terminal_log, waypoint_start_block(self.waypoints, matched))
                    self.last_started_index = matched
                self.active_goal_index = matched

        def odom_cb(self, msg):
            pos = msg.pose.pose.position
            stamp = msg.header.stamp.to_sec() if msg.header.stamp and msg.header.stamp.to_sec() > 0.0 else rospy.Time.now().to_sec()
            self.accumulator.observe(
                [float(pos.x), float(pos.y), float(pos.z)],
                yaw_rad=None,
                time_sec=stamp,
                active_goal_index=self.active_goal_index,
            )
            if self.accumulator.p8_completed:
                self.emit(force=True, state="complete")
                self.exit_code = 0
                rospy.signal_shutdown("p8 completed")

        def timer_cb(self, _event):
            if time.time() >= self.deadline_wall:
                self.emit(force=True, state="timeout")
                rospy.signal_shutdown("route monitor timeout")
                return
            self.emit(force=False, state="running")

        def emit(self, force=False, state="running"):
            summary = self.accumulator.summary()
            status = terminal_status(summary)
            progress_changed = status["progress_index"] != self.last_progress_index
            p8_changed = status["p8_completed"] != self.last_p8_completed
            if progress_changed:
                previous = 0 if self.last_progress_index is None else int(self.last_progress_index)
                current = int(status["progress_index"])
                if self.last_progress_index is None and current == 0:
                    append_terminal_block(args.terminal_log, waypoint_reached_block(summary, status, 0))
                elif current > previous:
                    for index in range(previous + 1, current + 1):
                        append_terminal_block(args.terminal_log, waypoint_reached_block(summary, status, index))
                self.last_progress_index = status["progress_index"]
                self.last_p8_completed = status["p8_completed"]
            elif p8_changed:
                self.last_p8_completed = status["p8_completed"]
            write_status_files(args, state, summary, summary=None, clearance_static=None)

    os.makedirs(args.run_dir, exist_ok=True)
    rospy.init_node("f250_route_human_summary", anonymous=True)
    monitor = Monitor()
    rospy.spin()
    return monitor.exit_code


def finalize(args):
    summary = read_json(os.path.join(args.run_dir, "summary.json"), default={})
    metric_summary = read_json(os.path.join(args.run_dir, "metric_summary.json"), default={})
    clearance_static = read_json(os.path.join(args.run_dir, "clearance_static_gate.json"), default={})
    clearance_dynamic = read_json(os.path.join(args.run_dir, "clearance_dynamic_telemetry.json"), default={})
    acceptance = update_artifact_policy(args, metric_summary, summary, clearance_static, clearance_dynamic)
    state = "complete" if acceptance["ok"] else "failed"
    write_status_files(args, state, metric_summary, summary, clearance_static)
    block = route_final_block(acceptance["terminal"], state, summary, clearance_static)
    append_terminal_block(args.terminal_log, block)
    append_terminal_over(args.terminal_log)
    if args.print_final:
        print(block, flush=True)
        print("OVER", flush=True)
    return 0 if acceptance["ok"] else 1


def add_common(parser):
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--run-label", required=True)
    parser.add_argument("--scene-config", required=True)
    parser.add_argument("--dynamic-mode", default="auto")
    parser.add_argument("--max-duration-sec", type=float, default=360.0)
    parser.add_argument("--terminal-log", required=True)
    parser.add_argument("--route-status-env", required=True)
    parser.add_argument("--status-env", required=True)


def parse_args():
    parser = argparse.ArgumentParser(description="F250 P0-P8 route terminal display and summary helper.")
    sub = parser.add_subparsers(dest="command", required=True)

    dry = sub.add_parser("dry-run", help="write synthetic successful F250 route artifacts without ROS")
    add_common(dry)

    live = sub.add_parser("live-monitor", help="print restricted live F250 route terminal lines")
    add_common(live)
    live.add_argument("--odom-topic", default="/mavros/local_position/odom")
    live.add_argument("--active-goal-topic", default="/maritime/active_goal")
    live.add_argument("--clearance-sample-period-sec", type=float, default=0.5)
    live.add_argument("--display-period-sec", type=float, default=1.0)

    final = sub.add_parser("finalize", help="write route acceptance summary from run artifacts")
    add_common(final)
    final.add_argument("--print-final", action="store_true")

    return parser.parse_args()


def main():
    args = parse_args()
    if args.command == "dry-run":
        return dry_run(args)
    if args.command == "live-monitor":
        return live_monitor(args)
    if args.command == "finalize":
        return finalize(args)
    raise RuntimeError("unknown command: %s" % args.command)


if __name__ == "__main__":
    raise SystemExit(main())
