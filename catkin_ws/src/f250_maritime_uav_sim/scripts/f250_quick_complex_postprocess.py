#!/usr/bin/env python3
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from f250_quick_complex_candidate_summary import derive_motion, derive_smoothness, read_trajectory


def read_json(path):
    with open(path, "r", encoding="utf-8") as handle:
        return json.load(handle)


def write_json(path, payload):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True)
        handle.write("\n")


def metric_passed(metric_summary, metric_name):
    metric = metric_summary.get(metric_name)
    if not isinstance(metric, dict):
        return False
    return bool(metric.get("passed"))


def main():
    parser = argparse.ArgumentParser(description="Compose F250 quick-complex metrics.json from run artifacts.")
    parser.add_argument("--candidate-id", required=True)
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--params-json", required=True)
    parser.add_argument("--summary-json", required=True)
    parser.add_argument("--clearance-static-json", required=True)
    parser.add_argument("--clearance-dynamic-json", required=True)
    parser.add_argument("--metric-summary-json", required=True)
    parser.add_argument("--output-json", required=True)
    parser.add_argument("--source", default="lidar")
    parser.add_argument("--vehicle", default="f250")
    args = parser.parse_args()

    params_payload = read_json(args.params_json)
    summary = read_json(args.summary_json)
    clearance_static = read_json(args.clearance_static_json)
    clearance_dynamic = read_json(args.clearance_dynamic_json)
    metric_summary = read_json(args.metric_summary_json)
    trajectory_rows = read_trajectory(summary["output_csv"])
    motion = derive_motion(trajectory_rows)
    smoothness = derive_smoothness(trajectory_rows)

    actual_static = ((((clearance_static.get("metrics") or {}).get("actual_trajectory") or {}).get("static")) or {})
    actual_dynamic = ((((clearance_dynamic.get("metrics") or {}).get("actual_trajectory") or {}).get("dynamic")) or {})
    failure_reasons = []
    failure_reasons.extend(summary.get("errors") or [])

    actual_clearance_ok = (
        not actual_static.get("collision")
        and int(actual_static.get("geometry_entry_count") or 0) == 0
        and int(actual_static.get("cloud_entry_count") or 0) == 0
    )
    if not actual_clearance_ok:
        failure_reasons.append("actual trajectory has static collision/entry")

    route = summary.get("route") or {}
    final_hold_required = float(route.get("final_hold_required_sec") or 0.2)
    final_hold_seen = float(route.get("final_hold_seen_sec") or 0.0)
    reached_p8 = bool(route.get("final_reached_ever"))
    if not reached_p8:
        failure_reasons.append("P8 not reached")
    if final_hold_seen < final_hold_required:
        failure_reasons.append(
            "final hold %.3f below %.3f" % (final_hold_seen, final_hold_required)
        )

    summary_ok = bool(summary.get("ok"))
    metric36_ok = metric_passed(metric_summary, "metric_3_6")
    metric37_ok = metric_passed(metric_summary, "metric_3_7")
    metric38_ok = metric_passed(metric_summary, "metric_3_8")
    metric39_ok = metric_passed(metric_summary, "metric_3_9")
    metric_route_ok = bool(metric36_ok and metric37_ok and metric38_ok and metric39_ok)
    route_ok = bool(
        summary_ok and metric_route_ok and actual_clearance_ok and reached_p8 and
        final_hold_seen >= final_hold_required
    )
    if not metric36_ok:
        failure_reasons.append("metric_3_6 route keypoint gate failed")
    if not metric37_ok:
        failure_reasons.append("metric_3_7 static obstacle-entry/P8 gate failed")
    if not metric38_ok:
        failure_reasons.append("metric_3_8 planning success gate failed")
    if not metric39_ok:
        failure_reasons.append("metric_3_9 final target gate failed")

    metrics = {
        "candidate_id": args.candidate_id,
        "run_dir": args.run_dir,
        "vehicle": args.vehicle,
        "source": args.source,
        "params": params_payload.get("params", {}),
        "params_context": {
            "description": params_payload.get("description"),
            "scene_level": params_payload.get("scene_level"),
            "scene_config": params_payload.get("scene_config"),
            "perception_source": params_payload.get("perception_source"),
            "dynamic_mode": params_payload.get("dynamic_mode"),
            "px4_spawn_x": params_payload.get("px4_spawn_x"),
            "px4_spawn_y": params_payload.get("px4_spawn_y"),
            "px4_spawn_z": params_payload.get("px4_spawn_z"),
            "px4_spawn_yaw": params_payload.get("px4_spawn_yaw"),
        },
        "artifacts": {
            "actual_trajectory_csv": os.path.join(args.run_dir, "actual_trajectory.csv"),
            "summary_json": os.path.join(args.run_dir, "summary.json"),
            "clearance_static_gate_json": os.path.join(args.run_dir, "clearance_static_gate.json"),
            "clearance_dynamic_telemetry_json": os.path.join(args.run_dir, "clearance_dynamic_telemetry.json"),
            "metric_summary_json": os.path.join(args.run_dir, "metric_summary.json"),
            "metric_waypoints_csv": os.path.join(args.run_dir, "metric_waypoints.csv"),
            "planned_vs_actual_png": None,
        },
        "summary_ok": summary_ok,
        "monitor_status": 0 if summary_ok else 2,
        "route_metric_ok": metric_route_ok,
        "ok": route_ok,
        "reached_p8": reached_p8,
        "stop_reason": summary.get("stop_reason"),
        "state": summary.get("state"),
        "counts": summary.get("counts"),
        "clouds": summary.get("clouds"),
        "route": route,
        "task": summary.get("task"),
        "clearance": {
            "actual_static_min_m": actual_static.get("min_clearance_m"),
            "actual_static_min_cloud_distance_m": actual_static.get("min_cloud_distance_m"),
            "actual_dynamic_min_m": actual_dynamic.get("min_clearance_m"),
            "actual_dynamic_min_cloud_distance_m": actual_dynamic.get("min_cloud_distance_m"),
            "static_collision": actual_static.get("collision"),
            "static_geometry_entry_count": actual_static.get("geometry_entry_count"),
            "static_cloud_entry_count": actual_static.get("cloud_entry_count"),
            "dynamic_geometry_entry_count": actual_dynamic.get("dynamic_geometry_entry_count"),
            "dynamic_cloud_entry_count": actual_dynamic.get("dynamic_cloud_entry_count"),
        },
        "motion": {
            "speed_mean": motion.get("speed_mean_mps"),
            "speed_p95": motion.get("speed_p95_mps"),
            "speed_max": motion.get("speed_max_mps"),
            "acc_p95": motion.get("acc_p95_mps2"),
            "acc_p99": motion.get("acc_p99_mps2"),
            "acc_max": motion.get("acc_max_mps2"),
            "jerk_p95": motion.get("jerk_p95_proxy"),
            "jerk_p99": motion.get("jerk_p99_proxy"),
            "jerk_max": motion.get("jerk_max_proxy"),
        },
        "smoothness": smoothness,
        "formal_metrics": metric_summary,
        "metric_policy": {
            "date": "2026-06-04",
            "route_ok_excludes_metric_3_10": True,
            "route_ok_excludes_yaw": True,
            "clearance_threshold_shortfall_is_telemetry": True,
            "components": ["summary_ok", "actual_static_no_entry", "metric_3_6", "metric_3_7", "metric_3_8", "metric_3_9"],
        },
        "clearance_failure_reasons": clearance_static.get("failure_reasons") or [],
        "dynamic_clearance_failure_reasons": clearance_dynamic.get("failure_reasons") or [],
        "failures": failure_reasons,
    }
    write_json(args.output_json, metrics)
    print(os.path.abspath(args.output_json))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
