#!/usr/bin/env python3
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from maritime_clearance import evaluate_clearance


def parse_args():
    parser = argparse.ArgumentParser(
        description="Offline planner-visible obstacle clearance evaluator for maritime scenes.")
    parser.add_argument("--scene-config", required=True)
    parser.add_argument("--trajectory-csv", default=None,
                        help="Optional trajectory CSV for actual-flight clearance.")
    parser.add_argument("--summary-json", default=None,
                        help="Optional output path. JSON is printed to stdout when omitted.")
    parser.add_argument("--dynamic-mode", default=os.environ.get("DYNAMIC_MODE", "auto"))
    parser.add_argument("--sample-spacing-m", type=float, default=0.25)
    parser.add_argument("--dynamic-samples", type=int, default=64)
    parser.add_argument("--dynamic-horizon-sec", type=float, default=None)
    parser.add_argument("--min-obstacle-distance-m", type=float, default=None)
    parser.add_argument("--min-dynamic-obstacle-distance-m", type=float, default=None)
    parser.add_argument("--actual-filter", default="armed_offboard",
                        choices=("all", "armed", "armed_offboard"))
    parser.add_argument("--cloud-entry-radius-m", type=float, default=None)
    parser.add_argument("--cloud-search-radius-m", type=float, default=None)
    parser.add_argument("--skip-cloud-distance", action="store_true",
                        help="Use scene geometry only. Geometry still represents planner-visible obstacles.")
    return parser.parse_args()


def main():
    args = parse_args()
    report = evaluate_clearance(
        args.scene_config,
        trajectory_csv=args.trajectory_csv,
        dynamic_mode=args.dynamic_mode,
        sample_spacing_m=args.sample_spacing_m,
        dynamic_samples=args.dynamic_samples,
        dynamic_horizon_sec=args.dynamic_horizon_sec,
        min_obstacle_distance_m=args.min_obstacle_distance_m,
        min_dynamic_obstacle_distance_m=args.min_dynamic_obstacle_distance_m,
        actual_filter=args.actual_filter,
        include_cloud_distance=not args.skip_cloud_distance,
        cloud_entry_radius_m=args.cloud_entry_radius_m,
        cloud_search_radius_m=args.cloud_search_radius_m,
    )

    if args.summary_json:
        os.makedirs(os.path.dirname(os.path.abspath(args.summary_json)), exist_ok=True)
        with open(args.summary_json, "w", encoding="utf-8") as handle:
            json.dump(report, handle, indent=2, sort_keys=True)
            handle.write("\n")
        print(os.path.abspath(args.summary_json))
    else:
        print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report.get("ok") else 2


if __name__ == "__main__":
    sys.exit(main())
