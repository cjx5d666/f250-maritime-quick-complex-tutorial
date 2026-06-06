#!/usr/bin/env python3
import argparse
import csv
import importlib.util
import json
import math
import os
import sys
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, Ellipse, Rectangle
from matplotlib.transforms import Affine2D


def resolve_project_root():
    env_root = os.environ.get("F250_PROJECT_ROOT")
    if env_root:
        return os.path.abspath(os.path.expanduser(env_root))
    script_dir = os.path.dirname(os.path.abspath(__file__))
    for rel in ("../../../..", "../../../../.."):
        candidate = os.path.abspath(os.path.join(script_dir, rel))
        if os.path.isdir(os.path.join(candidate, "catkin_ws", "src", "f250_maritime_uav_sim")):
            return candidate
    return os.getcwd()


def env_or_project_path(env_name, *parts):
    value = os.environ.get(env_name)
    if value:
        return os.path.abspath(os.path.expanduser(value))
    return os.path.join(PROJECT_ROOT, *parts)


def first_existing_or_first(*paths):
    for path in paths:
        if os.path.exists(path):
            return path
    return paths[0]


PROJECT_ROOT = resolve_project_root()
DEFAULT_RUN_ROOT = env_or_project_path(
    "RUN_ROOT", "runs", "f250_human_scripts")
DEFAULT_MAP_AUTHORITY = (
    os.path.abspath(os.path.expanduser(os.environ["MAP_AUTHORITY"]))
    if os.environ.get("MAP_AUTHORITY") else
    os.path.join(
        PROJECT_ROOT,
        "data",
        "map_authority",
        "p0_p8_hard_requirement_20260530",
    )
)
FIXED_ROUTE_ID = "maritime_quick_complex_p0_p8_hard_requirement_20260530"
PHASE_C_FALLBACK = os.path.join(DEFAULT_RUN_ROOT, "phase_c_static_check")
AUTHORITY_RENDERER_NAME = "render_quick_complex_authoritative_map.py"

OUTPUT_PNG = "latest_planned_vs_flown.png"
OUTPUT_JSON = "latest_plot_summary.json"
OUTPUT_CSV = "latest_plot_points.csv"
OUTPUT_MD = "latest_plot_summary.md"

ROUTE_JSON_NAMES = (
    "route_acceptance_summary.json",
    "summary.json",
    "metrics.json",
)

ROUTE_ACCEPTANCE_COMPONENTS = (
    "p8_completion",
    "static_obstacle_safety",
    "metric_3_6_keypoint_error",
    "metric_3_7_static_obstacle_gate",
    "metric_3_8_route_progress",
    "metric_3_9_endpoint_error",
    "recorder_summary",
)


def abspath(path):
    return os.path.abspath(os.path.expanduser(path))


def read_json(path, default=None):
    if not path or not os.path.exists(path):
        return default
    with open(path, "r", encoding="utf-8") as handle:
        return json.load(handle)


def write_json(path, payload):
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True)
        handle.write("\n")


def safe_float(value, default=None):
    if value is None or value == "":
        return default
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def safe_int(value, default=None):
    if value is None or value == "":
        return default
    try:
        return int(float(value))
    except (TypeError, ValueError):
        return default


def file_size(path):
    try:
        return os.path.getsize(path)
    except OSError:
        return 0


def iso_from_mtime(mtime):
    return time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime(mtime))


def read_csv_dicts(path):
    with open(path, "r", encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def write_csv_dicts(path, rows, fieldnames):
    with open(path, "w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def route_json_paths(run_dir):
    return [os.path.join(run_dir, name) for name in ROUTE_JSON_NAMES]


def legacy_vehicle_marker():
    return "uav" + "_" + "2m"


def is_under_legacy_vehicle_path(path):
    parts = abspath(path).split(os.sep)
    return legacy_vehicle_marker() in parts


def route_artifact_mtime(run_dir):
    paths = [os.path.join(run_dir, "actual_trajectory.csv")]
    paths.extend(route_json_paths(run_dir))
    mtimes = []
    for path in paths:
        if os.path.exists(path):
            mtimes.append(os.path.getmtime(path))
    if not mtimes and os.path.isdir(run_dir):
        mtimes.append(os.path.getmtime(run_dir))
    return max(mtimes) if mtimes else 0.0


def has_fc_only_shape(run_dir):
    base = os.path.basename(os.path.normpath(run_dir)).lower()
    if "fc_3_10" in base:
        return True
    fc_summary = os.path.join(run_dir, "fc_3_10_summary.json")
    route_summary = os.path.join(run_dir, "route_acceptance_summary.json")
    actual_csv = os.path.join(run_dir, "actual_trajectory.csv")
    return os.path.exists(fc_summary) and not (
        os.path.exists(route_summary) and os.path.exists(actual_csv)
    )


def extract_vehicle(json_payloads):
    for payload in json_payloads:
        if not isinstance(payload, dict):
            continue
        vehicle = payload.get("vehicle")
        if vehicle:
            return str(vehicle)
        params = payload.get("params")
        if isinstance(params, dict) and params.get("vehicle"):
            return str(params.get("vehicle"))
    return None


def classify_run_dir(run_dir, auto_select=False):
    run_dir = abspath(run_dir)
    actual_csv = os.path.join(run_dir, "actual_trajectory.csv")
    if is_under_legacy_vehicle_path(run_dir):
        return None, "reject_legacy_vehicle_path"
    if not os.path.isdir(run_dir):
        return None, "not_a_directory"
    if not os.path.exists(actual_csv) or file_size(actual_csv) <= 0:
        return None, "missing_actual_trajectory_csv"
    json_paths = [path for path in route_json_paths(run_dir) if file_size(path) > 0]
    if not json_paths:
        return None, "missing_route_summary_json"
    if auto_select and has_fc_only_shape(run_dir):
        return None, "skip_fc_3_10_only"

    payloads = []
    for path in json_paths:
        try:
            payloads.append(read_json(path, default={}))
        except Exception:
            payloads.append({})
    vehicle = extract_vehicle(payloads)
    if vehicle and vehicle.lower() != "f250":
        return None, "reject_non_f250_vehicle_%s" % vehicle

    has_acceptance = os.path.exists(os.path.join(run_dir, "route_acceptance_summary.json"))
    has_summary = os.path.exists(os.path.join(run_dir, "summary.json"))
    has_metrics = os.path.exists(os.path.join(run_dir, "metrics.json"))
    completeness = int(has_acceptance) + int(has_summary) + int(has_metrics)
    mtime = route_artifact_mtime(run_dir)
    candidate = {
        "run_dir": run_dir,
        "run_label": os.path.basename(os.path.normpath(run_dir)),
        "actual_trajectory_csv": actual_csv,
        "mtime": mtime,
        "mtime_iso": iso_from_mtime(mtime),
        "has_route_acceptance_summary": has_acceptance,
        "has_summary_json": has_summary,
        "has_metrics_json": has_metrics,
        "completeness": completeness,
    }
    return candidate, None


def list_run_candidates(run_root):
    run_root = abspath(run_root)
    candidates = []
    rejected = []
    if not os.path.isdir(run_root):
        return candidates, [{"run_dir": run_root, "reason": "run_root_missing"}]
    for name in sorted(os.listdir(run_root)):
        path = os.path.join(run_root, name)
        if not os.path.isdir(path):
            continue
        candidate, reason = classify_run_dir(path, auto_select=True)
        if candidate:
            candidates.append(candidate)
        else:
            rejected.append({"run_dir": path, "reason": reason})
    if not candidates and os.path.isdir(PHASE_C_FALLBACK):
        candidate, reason = classify_run_dir(PHASE_C_FALLBACK, auto_select=False)
        if candidate:
            candidate["fallback_reason"] = "phase_c_static_check"
            candidates.append(candidate)
        else:
            rejected.append({"run_dir": PHASE_C_FALLBACK, "reason": reason})
    candidates.sort(
        key=lambda item: (
            int(item["has_route_acceptance_summary"]),
            item["completeness"],
            item["mtime"],
            item["run_dir"],
        ),
        reverse=True,
    )
    return candidates, rejected


def select_run(args):
    if args.run_dir:
        candidate, reason = classify_run_dir(args.run_dir, auto_select=False)
        if not candidate:
            raise RuntimeError("RUN_DIR is not a suitable F250 route run: %s (%s)" % (
                abspath(args.run_dir), reason))
        return candidate, [candidate], []
    candidates, rejected = list_run_candidates(args.run_root)
    if not candidates:
        raise RuntimeError("no suitable F250 route run found under %s" % abspath(args.run_root))
    return candidates[0], candidates, rejected


def require_map_inputs(map_authority):
    map_authority = abspath(map_authority)
    paths = {
        "map_authority": map_authority,
        "map_manifest_json": os.path.join(map_authority, "map_manifest.json"),
        "route_waypoints_csv": os.path.join(map_authority, "route_waypoints.csv"),
        "planner_obstacles_csv": os.path.join(map_authority, "planner_obstacles.csv"),
        "visual_mesh_footprints_csv": os.path.join(map_authority, "visual_mesh_footprints.csv"),
        "map_layer_index_csv": os.path.join(map_authority, "map_layer_index.csv"),
    }
    missing = [path for path in paths.values() if not os.path.exists(path)]
    if missing:
        raise RuntimeError("missing authoritative map input(s): %s" % ", ".join(missing))
    return paths


def load_authority_renderer(map_authority):
    renderer_path = os.path.join(abspath(map_authority), AUTHORITY_RENDERER_NAME)
    if not os.path.exists(renderer_path):
        raise RuntimeError("missing authoritative map renderer: %s" % renderer_path)
    spec = importlib.util.spec_from_file_location("quick_complex_authoritative_map", renderer_path)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot import authoritative map renderer: %s" % renderer_path)
    renderer = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(renderer)
    required = (
        "setup_ax",
        "draw_visual_layer",
        "text_with_halo",
        "WAYPOINT_LABEL_OFFSETS",
        "BUOY_LABEL_OFFSETS",
        "rotated_rect",
        "load_visual_mesh",
        "SCENE_PATH",
        "yaml",
    )
    missing = [name for name in required if not hasattr(renderer, name)]
    if missing:
        raise RuntimeError(
            "authoritative map renderer missing required helper(s): %s" % ", ".join(missing)
        )
    return renderer


def load_authority_scene_and_meshes(renderer):
    with open(renderer.SCENE_PATH, "r", encoding="utf-8") as handle:
        scene = renderer.yaml.safe_load(handle) or {}
    meshes = [
        renderer.load_visual_mesh(item)
        for item in scene.get("visual_vessels", []) or []
        if item.get("mesh_uri")
    ]
    return scene, meshes


def waypoint_label(row, fallback_index):
    label = row.get("label") or ""
    if label:
        return label
    name = row.get("name") or ""
    if len(name) >= 2 and name[0].lower() == "p" and name[1].isdigit():
        chars = []
        for char in name[1:]:
            if not char.isdigit():
                break
            chars.append(char)
        if chars:
            return "P%s" % "".join(chars)
    return "P%d" % fallback_index


def load_waypoints(path):
    waypoints = []
    for index, row in enumerate(read_csv_dicts(path)):
        x = safe_float(row.get("x"))
        y = safe_float(row.get("y"))
        z = safe_float(row.get("z"))
        if x is None or y is None or z is None:
            continue
        waypoints.append({
            "index": safe_int(row.get("index"), index),
            "label": waypoint_label(row, index),
            "name": row.get("name") or "",
            "x": x,
            "y": y,
            "z": z,
            "yaw_rad": safe_float(row.get("yaw_rad")),
            "radius_m": safe_float(row.get("radius_m")),
        })
    if not waypoints:
        raise RuntimeError("no waypoints parsed from %s" % path)
    return waypoints


def load_actual_points(path):
    points = []
    for row_index, row in enumerate(read_csv_dicts(path)):
        x = safe_float(row.get("x"))
        y = safe_float(row.get("y"))
        z = safe_float(row.get("z"))
        if x is None or y is None:
            continue
        time_sec = safe_float(row.get("ros_time"))
        if time_sec is None:
            time_sec = safe_float(row.get("wall_time"))
        point = {
            "index": row_index,
            "time_sec": time_sec,
            "x": x,
            "y": y,
            "z": z,
            "active_goal_index": safe_int(row.get("active_goal_index")),
            "mode": row.get("mode") or "",
            "armed": row.get("armed") or "",
        }
        points.append(point)
    if not points:
        raise RuntimeError("no actual trajectory points parsed from %s" % path)
    return points


def axis_limits(manifest, waypoints, actual_points):
    extent = (manifest or {}).get("coordinate_extent") or {}
    xs = [point["x"] for point in actual_points] + [point["x"] for point in waypoints]
    ys = [point["y"] for point in actual_points] + [point["y"] for point in waypoints]
    x_extent = extent.get("x") or []
    y_extent = extent.get("y") or []
    if len(x_extent) == 2:
        xs.extend([safe_float(x_extent[0], 0.0), safe_float(x_extent[1], 300.0)])
    if len(y_extent) == 2:
        ys.extend([safe_float(y_extent[0], -120.0), safe_float(y_extent[1], 120.0)])
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    margin_x = max(10.0, (max_x - min_x) * 0.04)
    margin_y = max(10.0, (max_y - min_y) * 0.04)
    return (min_x - margin_x, max_x + margin_x), (min_y - margin_y, max_y + margin_y)


def rotated_rect(ax, center, length, width, yaw, **kwargs):
    transform = Affine2D().rotate(yaw).translate(center[0], center[1]) + ax.transData
    patch = Rectangle((-length / 2.0, -width / 2.0), length, width, transform=transform, **kwargs)
    ax.add_patch(patch)
    return patch


def draw_text(ax, x, y, text, color="#202124", size=7.5, dx=1.8, dy=1.8, weight="normal"):
    ax.text(float(x) + dx, float(y) + dy, text, color=color, fontsize=size,
            ha="left", va="bottom", weight=weight, zorder=50)


def draw_visual_footprints(ax, rows):
    style_colors = {
        "oasis": ("#b7c7d3", "#4c5a64"),
        "tanker": ("#c7cbd2", "#434850"),
        "island": ("#8f9f75", "#526344"),
        "red_bridge": ("#b85b4f", "#74332d"),
        "white_bridge": ("#d9d7cc", "#737067"),
        "wind": ("#f6f7f2", "#59636a"),
        "other": ("#f2f2ef", "#8c1e16"),
    }
    for row in rows:
        style = (row.get("style") or "other").lower()
        face, edge = style_colors.get(style, style_colors["other"])
        cx = safe_float(row.get("center_x"))
        cy = safe_float(row.get("center_y"))
        min_x = safe_float(row.get("min_x"))
        max_x = safe_float(row.get("max_x"))
        min_y = safe_float(row.get("min_y"))
        max_y = safe_float(row.get("max_y"))
        if None in (cx, cy, min_x, max_x, min_y, max_y):
            continue
        width = max_x - min_x
        height = max_y - min_y
        if style == "island":
            patch = Ellipse((cx, cy), width, height, facecolor=face, edgecolor=edge,
                            linewidth=0.8, alpha=0.45, zorder=2)
            ax.add_patch(patch)
        elif style == "wind":
            radius = max(width, height) * 0.25
            ax.add_patch(Circle((cx, cy), max(radius, 1.2), facecolor=face,
                                edgecolor=edge, linewidth=0.8, alpha=0.75, zorder=5))
        elif style == "other":
            ax.add_patch(Circle((cx, cy), max(width, height) / 2.0, facecolor=face,
                                edgecolor=edge, linewidth=0.8, alpha=0.75, zorder=5))
        else:
            patch = Rectangle((min_x, min_y), width, height, facecolor=face,
                              edgecolor=edge, linewidth=0.9, alpha=0.35, zorder=3)
            ax.add_patch(patch)

        name = row.get("name") or ""
        label = None
        lowered = name.lower()
        if style == "oasis":
            label = "H ship"
        elif style == "tanker":
            label = "S ship"
        elif style == "red_bridge":
            label = "B1"
        elif style == "white_bridge":
            label = "B2"
        elif style == "wind":
            if lowered.endswith("_1"):
                label = "W1"
            elif lowered.endswith("_2"):
                label = "W2"
            elif lowered.endswith("_3"):
                label = "W3"
            elif lowered.endswith("_4"):
                label = "W4"
        if label:
            draw_text(ax, cx, cy, label, size=7.2, weight="bold")


def obstacle_label(name):
    lowered = str(name).lower()
    if lowered.startswith("o"):
        token = lowered.split("_", 1)[0]
        return token.upper()
    if lowered.startswith("w"):
        token = lowered.split("_", 1)[0]
        return token.upper()
    if lowered.startswith("d"):
        token = lowered.split("_", 1)[0]
        return token.upper()
    return None


def draw_planner_obstacles(ax, rows):
    for row in rows:
        kind = (row.get("type") or "").lower()
        name = row.get("name") or ""
        cx = safe_float(row.get("center_x"))
        cy = safe_float(row.get("center_y"))
        yaw = safe_float(row.get("yaw_rad"), 0.0)
        if cx is None or cy is None:
            continue
        if "buoy" in kind or (row.get("shape") or "").lower() == "cylinder":
            radius = safe_float(row.get("radius_m"), 1.0)
            ax.add_patch(Circle((cx, cy), radius, facecolor="#e9553f",
                                edgecolor="#8c1e16", linewidth=1.0,
                                alpha=0.88, zorder=20))
        else:
            sx = safe_float(row.get("size_x"), 1.0)
            sy = safe_float(row.get("size_y"), 1.0)
            dynamic = "dynamic" in kind
            color = "#2f9eb3" if dynamic else "#ef9f4b"
            edge = "#0c5360" if dynamic else "#623b18"
            patch = rotated_rect(
                ax, (cx, cy), sx, sy, yaw,
                facecolor=color, edgecolor=edge, linewidth=1.1,
                alpha=0.78, zorder=22 if dynamic else 18)
            if not dynamic:
                patch.set_hatch("//")
            if dynamic and (row.get("motion_type") or "").lower() == "sinusoid":
                amp = safe_float(row.get("motion_amplitude_m"), 0.0)
                ax_x = safe_float(row.get("motion_axis_x"), 1.0)
                ax_y = safe_float(row.get("motion_axis_y"), 0.0)
                norm = math.hypot(ax_x, ax_y) or 1.0
                dx = amp * ax_x / norm
                dy = amp * ax_y / norm
                ax.plot([cx - dx, cx + dx], [cy - dy, cy + dy],
                        color=edge, linestyle="--", linewidth=1.0,
                        alpha=0.8, zorder=21)
        label = obstacle_label(name)
        if label:
            draw_text(ax, cx, cy, label, size=7.0, color="#222222", weight="bold")


def draw_presentation_planner_layer(ax, renderer, obstacle_rows, include_wind_boxes=False):
    for row in obstacle_rows:
        if str(row.get("include_in_cloud", "True")).lower() == "false":
            continue
        name = row.get("name") or ""
        kind = (row.get("type") or "").lower()
        shape = (row.get("shape") or "").lower()
        if not include_wind_boxes and "wind_channel_cloud" in name:
            continue
        cx = safe_float(row.get("center_x"))
        cy = safe_float(row.get("center_y"))
        yaw = safe_float(row.get("yaw_rad"), 0.0)
        if cx is None or cy is None:
            continue

        label = obstacle_label(name)
        if "buoy" in kind or shape == "cylinder":
            radius = safe_float(row.get("radius_m"), 1.0)
            ax.add_patch(Circle((cx, cy), radius, facecolor="#f6b84d",
                                edgecolor="#b06a00", linewidth=1.1,
                                alpha=0.78, zorder=18))
            if label:
                dx, dy = renderer.BUOY_LABEL_OFFSETS.get(label, (2.0, 2.0))
                renderer.text_with_halo(ax, cx + dx, cy + dy, label,
                                        fontsize=7.2, color="#7a4300", zorder=36)
            continue

        sx = safe_float(row.get("size_x"), 1.0)
        sy = safe_float(row.get("size_y"), 1.0)
        dynamic = "dynamic" in kind
        patch = renderer.rotated_rect(
            ax, (cx, cy), (sx, sy), yaw,
            facecolor="#3aa4c7" if dynamic else "#7d8790",
            edgecolor="#176073" if dynamic else "#40484f",
            linewidth=1.0,
            alpha=0.24 if dynamic else 0.12,
            zorder=16 if dynamic else 15,
        )
        if not dynamic:
            patch.set_hatch("///")
        if dynamic and (row.get("motion_type") or "").lower() == "sinusoid":
            amp = safe_float(row.get("motion_amplitude_m"), 0.0)
            ax_x = safe_float(row.get("motion_axis_x"), 1.0)
            ax_y = safe_float(row.get("motion_axis_y"), 0.0)
            norm = math.hypot(ax_x, ax_y) or 1.0
            dx = amp * ax_x / norm
            dy = amp * ax_y / norm
            ax.plot([cx - dx, cx + dx], [cy - dy, cy + dy],
                    color="#176073", linestyle="--", linewidth=0.8,
                    alpha=0.34, zorder=17)
        if label:
            renderer.text_with_halo(ax, cx, cy, label, ha="center", va="center",
                                    fontsize=7.0, color="#0b3945", zorder=36)


def draw_route_and_actual(ax, renderer, waypoints, actual_points):
    route_x = [point["x"] for point in waypoints]
    route_y = [point["y"] for point in waypoints]
    actual_x = [point["x"] for point in actual_points]
    actual_y = [point["y"] for point in actual_points]
    ax.plot(route_x, route_y, color="#174ea6", linewidth=2.25,
            alpha=0.96, zorder=30)
    for point in waypoints:
        ax.scatter([point["x"]], [point["y"]], color="#174ea6",
                   edgecolor="white", linewidth=0.8, s=34, zorder=44)
        dx, dy = renderer.WAYPOINT_LABEL_OFFSETS.get(point["label"], (1.7, 1.7))
        renderer.text_with_halo(ax, point["x"] + dx, point["y"] + dy,
                                point["label"], fontsize=7.5,
                                color="#08306b", zorder=46)
    ax.plot(actual_x, actual_y, color="#d13f31", linewidth=1.8,
            alpha=0.96, zorder=42)


def render_png(output_png, map_authority, manifest, visual_rows, obstacle_rows, waypoints, actual_points):
    del manifest, visual_rows
    renderer = load_authority_renderer(map_authority)
    _scene, meshes = load_authority_scene_and_meshes(renderer)
    fig, ax = plt.subplots(figsize=(13.0, 8.2), dpi=180)
    renderer.setup_ax(ax, "F250 P0-P8 planned vs flown")
    renderer.draw_visual_layer(ax, meshes)
    draw_presentation_planner_layer(ax, renderer, obstacle_rows, include_wind_boxes=False)
    draw_route_and_actual(ax, renderer, waypoints, actual_points)
    handles = [
        Line2D([0], [0], color="#174ea6", lw=2.25, marker="o",
               markersize=5.0, label="planned P0-P8"),
        Line2D([0], [0], color="#d13f31", lw=1.8, label="flown P0-P8"),
    ]
    ax.legend(handles=handles, loc="upper right", fontsize=8.0, framealpha=0.92)
    fig.tight_layout()
    fig.savefig(output_png)
    plt.close(fig)


def route_length_3d(waypoints):
    total = 0.0
    previous = None
    for point in waypoints:
        current = (point["x"], point["y"], point["z"])
        if previous:
            total += math.sqrt(sum((current[i] - previous[i]) ** 2 for i in range(3)))
        previous = current
    return total


def bounds(points):
    if not points:
        return None
    xs = [point["x"] for point in points]
    ys = [point["y"] for point in points]
    zs = [point["z"] for point in points if point.get("z") is not None]
    return {
        "x": [min(xs), max(xs)],
        "y": [min(ys), max(ys)],
        "z": [min(zs), max(zs)] if zs else None,
    }


def time_bounds(points):
    values = [point["time_sec"] for point in points if point.get("time_sec") is not None]
    if not values:
        return None
    return [min(values), max(values)]


def route_acceptance_from_run(run_dir):
    acceptance_path = os.path.join(run_dir, "route_acceptance_summary.json")
    summary_path = os.path.join(run_dir, "summary.json")
    metrics_path = os.path.join(run_dir, "metrics.json")
    acceptance = read_json(acceptance_path, default={}) or {}
    summary = read_json(summary_path, default={}) or {}
    metrics = read_json(metrics_path, default={}) or {}

    terminal = acceptance.get("terminal") or summary.get("route_terminal") or metrics.get("route_terminal") or {}
    raw_components = acceptance.get("components") or {}
    components = {}
    for name in ROUTE_ACCEPTANCE_COMPONENTS:
        if name in raw_components:
            components[name] = raw_components[name]
    ok = acceptance.get("ok")
    if ok is None:
        ok = summary.get("route_acceptance_ok")
    if ok is None:
        ok = metrics.get("ok")
    return {
        "ok": ok,
        "components": components,
        "terminal": terminal,
        "source_json": acceptance_path if os.path.exists(acceptance_path) else None,
        "summary_json": summary_path if os.path.exists(summary_path) else None,
        "metrics_json": metrics_path if os.path.exists(metrics_path) else None,
    }


def make_points_rows(waypoints, actual_points, route_csv, actual_csv):
    rows = []
    for point in waypoints:
        rows.append({
            "kind": "planned_waypoint",
            "source_file": route_csv,
            "index": point["index"],
            "label": point["label"],
            "name": point["name"],
            "time_sec": "",
            "x": "%.9g" % point["x"],
            "y": "%.9g" % point["y"],
            "z": "%.9g" % point["z"],
            "yaw_rad": "" if point.get("yaw_rad") is None else "%.9g" % point["yaw_rad"],
            "radius_m": "" if point.get("radius_m") is None else "%.9g" % point["radius_m"],
            "active_goal_index": "",
            "mode": "",
            "armed": "",
        })
    for point in actual_points:
        rows.append({
            "kind": "actual_trajectory",
            "source_file": actual_csv,
            "index": point["index"],
            "label": "",
            "name": "",
            "time_sec": "" if point.get("time_sec") is None else "%.9g" % point["time_sec"],
            "x": "%.9g" % point["x"],
            "y": "%.9g" % point["y"],
            "z": "" if point.get("z") is None else "%.9g" % point["z"],
            "yaw_rad": "",
            "radius_m": "",
            "active_goal_index": "" if point.get("active_goal_index") is None else point["active_goal_index"],
            "mode": point.get("mode") or "",
            "armed": point.get("armed") or "",
        })
    return rows


def write_markdown(path, summary):
    actual = summary["actual_trajectory"]
    planned = summary["planned_route"]
    route = summary["route_acceptance"]
    lines = [
        "# F250 latest planned-vs-flown summary",
        "",
        "- run: `%s`" % summary["selected_run_dir"],
        "- vehicle: f250",
        "- fixed route: `%s`" % FIXED_ROUTE_ID,
        "- map authority: `%s`" % summary["map_authority"],
        "- planned waypoints: %d, length: %.3f m" % (
            planned["waypoint_count"], planned["route_length_m"]),
        "- actual samples: %d" % actual["sample_count"],
        "- route acceptance ok: %s" % route.get("ok"),
        "- terminal progress: %s" % ((route.get("terminal") or {}).get("progress")),
        "- outputs: `%s`, `%s`, `%s`, `%s`" % (
            OUTPUT_PNG, OUTPUT_JSON, OUTPUT_CSV, OUTPUT_MD),
        "",
        "Geometry basis: authoritative CSV/JSON map inputs plus `actual_trajectory.csv`; no rendered PNG is used as geometry truth.",
    ]
    with open(path, "w", encoding="utf-8") as handle:
        handle.write("\n".join(lines))
        handle.write("\n")


def render_latest(args, selected):
    run_dir = selected["run_dir"]
    map_paths = require_map_inputs(args.map_authority)
    output_dir = abspath(args.output_dir or run_dir)
    os.makedirs(output_dir, exist_ok=True)
    output_png = os.path.join(output_dir, OUTPUT_PNG)
    output_json = os.path.join(output_dir, OUTPUT_JSON)
    output_csv = os.path.join(output_dir, OUTPUT_CSV)
    output_md = os.path.join(output_dir, OUTPUT_MD)

    manifest = read_json(map_paths["map_manifest_json"], default={}) or {}
    waypoints = load_waypoints(map_paths["route_waypoints_csv"])
    obstacles = read_csv_dicts(map_paths["planner_obstacles_csv"])
    visual_rows = read_csv_dicts(map_paths["visual_mesh_footprints_csv"])
    actual_points = load_actual_points(selected["actual_trajectory_csv"])

    render_png(output_png, args.map_authority, manifest, visual_rows, obstacles, waypoints, actual_points)
    points_rows = make_points_rows(
        waypoints, actual_points,
        map_paths["route_waypoints_csv"],
        selected["actual_trajectory_csv"],
    )
    point_fields = [
        "kind", "source_file", "index", "label", "name", "time_sec",
        "x", "y", "z", "yaw_rad", "radius_m", "active_goal_index",
        "mode", "armed",
    ]
    write_csv_dicts(output_csv, points_rows, point_fields)

    route_acceptance = route_acceptance_from_run(run_dir)
    run_artifacts = {
        "actual_trajectory_csv": selected["actual_trajectory_csv"],
        "route_acceptance_summary_json": os.path.join(run_dir, "route_acceptance_summary.json")
        if os.path.exists(os.path.join(run_dir, "route_acceptance_summary.json")) else None,
        "summary_json": os.path.join(run_dir, "summary.json")
        if os.path.exists(os.path.join(run_dir, "summary.json")) else None,
        "metrics_json": os.path.join(run_dir, "metrics.json")
        if os.path.exists(os.path.join(run_dir, "metrics.json")) else None,
    }
    summary = {
        "generated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "phase": "D",
        "vehicle": "f250",
        "fixed_route": FIXED_ROUTE_ID,
        "selected_run_dir": run_dir,
        "selected_run_label": selected["run_label"],
        "selected_run_mtime": selected["mtime_iso"],
        "map_authority": map_paths["map_authority"],
        "source_inputs": {
            "authoritative_map_csv_json": {
                "map_manifest_json": map_paths["map_manifest_json"],
                "route_waypoints_csv": map_paths["route_waypoints_csv"],
                "planner_obstacles_csv": map_paths["planner_obstacles_csv"],
                "visual_mesh_footprints_csv": map_paths["visual_mesh_footprints_csv"],
                "map_layer_index_csv": map_paths["map_layer_index_csv"],
            },
            "run_artifacts": run_artifacts,
        },
        "geometry_truth": {
            "basis": "authoritative CSV/JSON map package plus actual trajectory CSV",
            "rendered_png_used_as_geometry_truth": False,
        },
        "outputs": {
            "planned_vs_flown_png": output_png,
            "summary_json": output_json,
            "points_csv": output_csv,
            "summary_md": output_md,
        },
        "planned_route": {
            "waypoint_count": len(waypoints),
            "labels": [point["label"] for point in waypoints],
            "route_length_m": route_length_3d(waypoints),
            "bounds": bounds(waypoints),
        },
        "actual_trajectory": {
            "sample_count": len(actual_points),
            "time_bounds_sec": time_bounds(actual_points),
            "bounds": bounds(actual_points),
            "active_goal_index_max": max(
                [point["active_goal_index"] for point in actual_points
                 if point.get("active_goal_index") is not None] or [None]
            ),
        },
        "route_acceptance": route_acceptance,
        "plot_points_csv_rows": len(points_rows),
        "offline_only": True,
        "runtime_processes_started": [],
        "route_acceptance_pass_fail_components_written": list(route_acceptance["components"].keys()),
    }
    write_json(output_json, summary)
    write_markdown(output_md, summary)
    return summary


def print_candidate_table(selected, candidates, rejected=None):
    print("selected_run_dir=%s" % selected["run_dir"])
    print("selected_run_mtime=%s" % selected["mtime_iso"])
    print("candidates:")
    for item in candidates:
        marker = "*" if item["run_dir"] == selected["run_dir"] else "-"
        print(
            "  %s %s complete=%d route_acceptance=%s summary=%s metrics=%s %s" % (
                marker,
                item["mtime_iso"],
                item["completeness"],
                str(item["has_route_acceptance_summary"]).lower(),
                str(item["has_summary_json"]).lower(),
                str(item["has_metrics_json"]).lower(),
                item["run_dir"],
            )
        )
    if rejected:
        print("rejected:")
        for item in rejected:
            print("  - %s reason=%s" % (item["run_dir"], item["reason"]))


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Render the latest F250 maritime_quick_complex P0-P8 planned-vs-flown plot offline.")
    parser.add_argument("--run-root", default=DEFAULT_RUN_ROOT,
                        help="F250 human script run root to scan when RUN_DIR is not provided.")
    parser.add_argument("--run-dir", default=os.environ.get("RUN_DIR"),
                        help="Explicit F250 run directory to render.")
    parser.add_argument("--map-authority", default=DEFAULT_MAP_AUTHORITY,
                        help="Authoritative 2026-05-30 P0-P8 map package.")
    parser.add_argument("--output-dir", default=None,
                        help="Output directory. Default: selected run directory.")
    parser.add_argument("--dry-run", action="store_true",
                        help="Select and print the target run without rendering.")
    parser.add_argument("--list", action="store_true",
                        help="List suitable run candidates and exit without rendering.")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv or sys.argv[1:])
    try:
        selected, candidates, rejected = select_run(args)
        if args.dry_run or args.list:
            print_candidate_table(selected, candidates, rejected if args.list else None)
            if args.dry_run:
                output_dir = abspath(args.output_dir or selected["run_dir"])
                print("dry_run=true")
                print("would_write=%s" % os.path.join(output_dir, OUTPUT_PNG))
                print("would_write=%s" % os.path.join(output_dir, OUTPUT_JSON))
                print("would_write=%s" % os.path.join(output_dir, OUTPUT_CSV))
                print("would_write=%s" % os.path.join(output_dir, OUTPUT_MD))
            return 0
        summary = render_latest(args, selected)
        print(json.dumps({
            "selected_run_dir": summary["selected_run_dir"],
            "planned_vs_flown_png": summary["outputs"]["planned_vs_flown_png"],
            "summary_json": summary["outputs"]["summary_json"],
            "points_csv": summary["outputs"]["points_csv"],
            "summary_md": summary["outputs"]["summary_md"],
            "actual_samples": summary["actual_trajectory"]["sample_count"],
            "route_acceptance_ok": summary["route_acceptance"]["ok"],
        }, indent=2, sort_keys=True))
        return 0
    except Exception as exc:
        print("f250_render_latest_planned_vs_flown: %s" % exc, file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
