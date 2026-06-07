#!/usr/bin/env python3
import csv
import json
import math
import os
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import patheffects as pe
import numpy as np
import trimesh
import yaml
from PIL import Image, ImageColor, ImageDraw
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, Rectangle, Polygon
from matplotlib.transforms import Affine2D


ROOT = Path(os.environ.get("F250_PROJECT_ROOT", Path(__file__).resolve().parents[3]))
SCENE_PATH = ROOT / "catkin_ws/src/f250_maritime_uav_sim/config/scenes/level_m_gps_assets_quick_complex.yaml"
OUT_DIR = Path(__file__).resolve().parent

X_RANGE = (0.0, 300.0)
Y_RANGE = (-120.0, 120.0)
PX_PER_M = 6.0
CANVAS_W = int(round((X_RANGE[1] - X_RANGE[0]) * PX_PER_M)) + 1
CANVAS_H = int(round((Y_RANGE[1] - Y_RANGE[0]) * PX_PER_M)) + 1

STANDARD_OUTPUTS = {
    "clean": "01_clean_visual_base.png",
    "planner": "02_visual_planner_obstacles.png",
    "route": "03_visual_planner_route_p0_p8.png",
}


VISUAL_STYLE = {
    "oasis": {"face": "#d5e0e7", "edge": "#4c5e68", "alpha": 0.92, "z": 8},
    "tanker": {"face": "#cbd5dc", "edge": "#46545f", "alpha": 0.92, "z": 8},
    "island": {"face": "#bfd4ae", "edge": "#708a69", "alpha": 0.86, "z": 4},
    "red_bridge": {"face": "#cf6f60", "edge": "#9f3d32", "alpha": 0.72, "z": 7},
    "white_bridge": {"face": "#e8eef2", "edge": "#98a6ae", "alpha": 0.74, "z": 7},
    "wind": {"face": "#f8faf8", "edge": "#66737a", "alpha": 0.88, "z": 9},
    "other": {"face": "#d7d7d7", "edge": "#666666", "alpha": 0.7, "z": 6},
}

LABEL_STROKE = [pe.withStroke(linewidth=2.8, foreground="white", alpha=0.92)]

WAYPOINT_LABEL_OFFSETS = {
    "P0": (2.2, -2.2),
    "P1": (1.6, 2.4),
    "P2": (1.6, 2.2),
    "P3": (1.6, -2.0),
    "P4": (1.8, 1.8),
    "P5": (1.6, 2.2),
    "P6": (1.6, -3.0),
    "P7": (1.8, 1.6),
    "P8": (1.7, 1.5),
}

BUOY_LABEL_OFFSETS = {
    "O1": (2.2, 2.2),
    "O2": (2.2, 1.0),
    "O3": (2.1, -2.0),
    "O4": (2.1, 2.1),
    "O5": (2.1, 1.3),
}


def as_scale3(value):
    if isinstance(value, (int, float)):
        return np.array([float(value), float(value), float(value)], dtype=float)
    if isinstance(value, (list, tuple)):
        if len(value) == 3:
            return np.array([float(value[0]), float(value[1]), float(value[2])], dtype=float)
        if len(value) == 1:
            return np.array([float(value[0]), float(value[0]), float(value[0])], dtype=float)
    return np.ones(3, dtype=float)


def yaw_of(item):
    if "yaw" in item:
        return float(item.get("yaw", 0.0))
    rpy = item.get("rpy") or [0.0, 0.0, 0.0]
    return float(rpy[2]) if len(rpy) >= 3 else 0.0


def resolve_model_uri(uri):
    if not uri.startswith("model://"):
        return Path(uri)
    rel = uri[len("model://"):]
    model, rest = rel.split("/", 1)
    return ROOT / "catkin_ws/src/f250_maritime_uav_sim/models" / model / rest


def package_display_path(path):
    path = Path(path)
    try:
        return "${F250_PROJECT_ROOT}/" + path.resolve().relative_to(ROOT.resolve()).as_posix()
    except ValueError:
        return str(path)


def transform_xyz(points, item):
    center = np.array(item.get("center", [0.0, 0.0, 0.0]), dtype=float)
    scale = as_scale3(item.get("scale", item.get("mesh_scale", 1.0)))
    local = points * scale
    yaw = yaw_of(item)
    c, s = math.cos(yaw), math.sin(yaw)
    rot = np.array([[c, -s], [s, c]], dtype=float)
    xy = local[:, :2] @ rot.T + center[:2]
    z = local[:, 2] + center[2]
    return np.column_stack([xy, z])


def convex_hull(points):
    pts = sorted(set((float(x), float(y)) for x, y in points))
    if len(pts) <= 1:
        return np.array(pts, dtype=float)

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return np.array(lower[:-1] + upper[:-1], dtype=float)


def classify_visual(name):
    n = name.lower()
    if "oasis" in n or "carrier" in n:
        return "oasis"
    if "tanker" in n:
        return "tanker"
    if "island" in n or "kauai" in n:
        return "island"
    if "golden" in n:
        return "red_bridge"
    if "helix" in n:
        return "white_bridge"
    if "wind" in n:
        return "wind"
    return "other"


def load_visual_mesh(item):
    mesh_path = resolve_model_uri(item["mesh_uri"])
    mesh = trimesh.load(mesh_path, force="mesh", process=False)
    vertices = np.asarray(mesh.vertices, dtype=float)
    faces = np.asarray(mesh.faces, dtype=int)
    world = transform_xyz(vertices, item)
    hull = convex_hull(world[:, :2])
    return {
        "name": item["name"],
        "mesh_path": package_display_path(mesh_path),
        "style": classify_visual(item["name"]),
        "yaw": yaw_of(item),
        "vertices": int(len(vertices)),
        "faces": faces,
        "world_xy": world[:, :2],
        "hull": hull,
        "center": [float(item.get("center", [0.0, 0.0, 0.0])[0]), float(item.get("center", [0.0, 0.0, 0.0])[1])],
        "bounds": {
            "min_x": float(np.min(world[:, 0])),
            "max_x": float(np.max(world[:, 0])),
            "min_y": float(np.min(world[:, 1])),
            "max_y": float(np.max(world[:, 1])),
            "min_z": float(np.min(world[:, 2])),
            "max_z": float(np.max(world[:, 2])),
        },
    }


def text_with_halo(ax, x, y, text, **kwargs):
    default = {
        "fontsize": 7.4,
        "color": "#253238",
        "weight": "bold",
        "zorder": 35,
    }
    default.update(kwargs)
    t = ax.text(x, y, text, **default)
    t.set_path_effects(LABEL_STROKE)
    return t


def world_to_pixel(points):
    pts = np.asarray(points, dtype=float)
    cols = (pts[:, 0] - X_RANGE[0]) * PX_PER_M
    rows = (Y_RANGE[1] - pts[:, 1]) * PX_PER_M
    return [(float(c), float(r)) for c, r in zip(cols, rows)]


def rasterize_visual_meshes(meshes, skip_styles=None):
    skip_styles = set(skip_styles or [])
    base = Image.new("RGBA", (CANVAS_W, CANVAS_H), (0, 0, 0, 0))
    for rec in meshes:
        if rec["style"] in skip_styles:
            continue
        style = VISUAL_STYLE[rec["style"]]
        mask = Image.new("L", (CANVAS_W, CANVAS_H), 0)
        draw = ImageDraw.Draw(mask)
        xy = rec["world_xy"]
        for face in rec["faces"]:
            pts = xy[face]
            if (
                np.max(pts[:, 0]) < X_RANGE[0]
                or np.min(pts[:, 0]) > X_RANGE[1]
                or np.max(pts[:, 1]) < Y_RANGE[0]
                or np.min(pts[:, 1]) > Y_RANGE[1]
            ):
                continue
            draw.polygon(world_to_pixel(pts), fill=255)

        mask_np = np.asarray(mask, dtype=bool)
        if rec["style"] in {"island", "oasis", "tanker"}:
            layer = Image.fromarray(textured_visual_layer(rec, style, mask_np), mode="RGBA")
        else:
            fill_rgba = ImageColor.getrgb(style["face"]) + (int(255 * style["alpha"]),)
            layer = Image.new("RGBA", (CANVAS_W, CANVAS_H), fill_rgba)
            layer.putalpha(mask)
        base = Image.alpha_composite(base, layer)

        if mask_np.any():
            eroded = np.zeros_like(mask_np, dtype=bool)
            eroded[1:-1, 1:-1] = (
                mask_np[1:-1, 1:-1]
                & mask_np[:-2, 1:-1]
                & mask_np[2:, 1:-1]
                & mask_np[1:-1, :-2]
                & mask_np[1:-1, 2:]
            )
            boundary = mask_np & ~eroded
            edge_rgba = ImageColor.getrgb(style["edge"]) + (230,)
            edge_arr = np.zeros((CANVAS_H, CANVAS_W, 4), dtype=np.uint8)
            edge_arr[boundary] = edge_rgba
            base = Image.alpha_composite(base, Image.fromarray(edge_arr, mode="RGBA"))
    return np.asarray(base)


_WORLD_GRIDS = None


def world_grids():
    global _WORLD_GRIDS
    if _WORLD_GRIDS is None:
        cols = np.arange(CANVAS_W, dtype=float)
        rows = np.arange(CANVAS_H, dtype=float)
        xs = X_RANGE[0] + cols / PX_PER_M
        ys = Y_RANGE[1] - rows / PX_PER_M
        _WORLD_GRIDS = np.meshgrid(xs, ys)
    return _WORLD_GRIDS


def principal_frame(rec):
    pts = np.asarray(rec["world_xy"], dtype=float)
    center = np.asarray(rec["center"], dtype=float)
    shifted = pts - center
    cov = np.cov(shifted.T)
    vals, vecs = np.linalg.eigh(cov)
    tangent = vecs[:, int(np.argmax(vals))]
    if tangent[0] < 0:
        tangent = -tangent
    normal = np.array([-tangent[1], tangent[0]])
    u = shifted @ tangent
    v = shifted @ normal
    half_len = max(1.0, float(np.percentile(np.abs(u), 98.5)))
    half_width = max(1.0, float(np.percentile(np.abs(v), 98.5)))
    return center, tangent, normal, half_len, half_width


def mix_rgb(a, b, t):
    t = np.asarray(t, dtype=float)
    if t.ndim == 0:
        return np.asarray(a, dtype=float) * (1.0 - float(t)) + np.asarray(b, dtype=float) * float(t)
    return np.asarray(a, dtype=float) * (1.0 - t[:, None]) + np.asarray(b, dtype=float) * t[:, None]


def textured_visual_layer(rec, style, mask_np):
    arr = np.zeros((CANVAS_H, CANVAS_W, 4), dtype=np.uint8)
    if not mask_np.any():
        return arr

    xs_grid, ys_grid = world_grids()
    xs = xs_grid[mask_np]
    ys = ys_grid[mask_np]
    base_rgb = np.array(ImageColor.getrgb(style["face"]), dtype=float)
    colors = np.tile(base_rgb, (len(xs), 1))

    if rec["style"] == "island":
        cx, cy = rec["center"]
        sx = max(1.0, rec["bounds"]["max_x"] - rec["bounds"]["min_x"])
        sy = max(1.0, rec["bounds"]["max_y"] - rec["bounds"]["min_y"])
        dx = (xs - cx) / sx
        dy = (ys - cy) / sy
        elev = (
            0.62 * np.exp(-((dx * 2.3) ** 2 + (dy * 2.0) ** 2))
            + 0.24 * np.exp(-(((dx - 0.18) * 3.1) ** 2 + ((dy + 0.12) * 2.7) ** 2))
            + 0.16 * np.exp(-(((dx + 0.24) * 3.5) ** 2 + ((dy - 0.20) * 2.9) ** 2))
        )
        elev += 0.045 * np.sin(xs * 0.21 + ys * 0.17)
        elev = np.clip(elev, 0.0, 1.0)
        low = np.array(ImageColor.getrgb("#b8d0a8"), dtype=float)
        mid = np.array(ImageColor.getrgb("#8fb27a"), dtype=float)
        high = np.array(ImageColor.getrgb("#d4cf9c"), dtype=float)
        colors = mix_rgb(low, mid, np.clip(elev * 1.25, 0.0, 1.0))
        high_mask = elev > 0.58
        if np.any(high_mask):
            colors[high_mask] = mix_rgb(colors[high_mask], high, np.clip((elev[high_mask] - 0.58) / 0.42, 0.0, 1.0))
        contour = np.abs((elev * 8.0) % 1.0 - 0.5) < 0.035
        colors[contour] *= 0.88
    elif rec["style"] in {"oasis", "tanker"}:
        center, tangent, normal, half_len, half_width = principal_frame(rec)
        shifted = np.column_stack([xs, ys]) - center
        u = shifted @ tangent / half_len
        v = shifted @ normal / half_width
        hull = np.array(ImageColor.getrgb("#c9d5dd" if rec["style"] == "oasis" else "#c4cfd6"), dtype=float)
        deck = np.array(ImageColor.getrgb("#eef5f8" if rec["style"] == "oasis" else "#e7ecef"), dtype=float)
        shadow = np.array(ImageColor.getrgb("#8fa0aa"), dtype=float)
        colors = np.tile(hull, (len(xs), 1))
        side = np.clip((np.abs(v) - 0.34) / 0.55, 0.0, 1.0)
        colors = mix_rgb(colors, shadow, side * 0.38)
        deck_mask = (np.abs(v) < 0.22) & (np.abs(u) < 0.86)
        colors[deck_mask] = mix_rgb(colors[deck_mask], deck, 0.78)
        center_line = (np.abs(v) < 0.035) & (np.abs(u) < 0.84)
        colors[center_line] = np.array(ImageColor.getrgb("#5f7581"), dtype=float)
        if rec["style"] == "oasis":
            windows = (np.abs(v) > 0.34) & (np.abs(v) < 0.47) & (np.abs(u) < 0.68)
            colors[windows] = mix_rgb(colors[windows], ImageColor.getrgb("#6f8795"), 0.70)
            pad = (u > 0.58) & (u < 0.80) & (np.abs(v) < 0.20)
            colors[pad] = mix_rgb(colors[pad], ImageColor.getrgb("#5fbf8b"), 0.70)
        else:
            hatches = (np.abs(v) < 0.26) & (np.abs(u) < 0.76) & (np.abs(np.sin((u + 1.0) * 18.0)) > 0.88)
            colors[hatches] = mix_rgb(colors[hatches], ImageColor.getrgb("#7c8a92"), 0.45)

    arr[mask_np, :3] = np.clip(colors, 0, 255).astype(np.uint8)
    arr[mask_np, 3] = int(255 * style["alpha"])
    return arr


def mesh_centerline(rec, bins=36):
    pts = np.asarray(rec["world_xy"], dtype=float)
    if len(pts) < 4:
        return pts
    center = np.mean(pts, axis=0)
    shifted = pts - center
    cov = np.cov(shifted.T)
    vals, vecs = np.linalg.eigh(cov)
    tangent = vecs[:, int(np.argmax(vals))]
    if tangent[0] < 0:
        tangent = -tangent
    normal = np.array([-tangent[1], tangent[0]])
    u = shifted @ tangent
    v = shifted @ normal
    lo, hi = np.percentile(u, [1.0, 99.0])
    edges = np.linspace(lo, hi, bins + 1)
    line = []
    for a, b in zip(edges[:-1], edges[1:]):
        mask = (u >= a) & (u <= b)
        if int(np.count_nonzero(mask)) < 6:
            continue
        um = float(np.median(u[mask]))
        vm = float(np.median(v[mask]))
        line.append(center + um * tangent + vm * normal)
    if len(line) < 2:
        hull = rec["hull"]
        return hull if len(hull) else pts
    return np.asarray(line)


def draw_bridge_icon(ax, rec):
    line = mesh_centerline(rec)
    if len(line) < 2:
        return
    if rec["style"] == "white_bridge":
        edge, face, rail = "#758b95", "#f8fcff", "#b9c7ce"
        edge_w, face_w, rail_w = 8.0, 5.8, 1.0
        z = 24
    else:
        edge, face, rail = "#974c44", "#df7668", "#b7554b"
        edge_w, face_w, rail_w = 9.2, 7.2, 1.0
        z = 23
    ax.plot(line[:, 0], line[:, 1], color=edge, linewidth=edge_w,
            alpha=0.88, zorder=z, solid_capstyle="round")
    ax.plot(line[:, 0], line[:, 1], color=face, linewidth=face_w,
            alpha=0.96, zorder=z + 1, solid_capstyle="round")
    ax.plot(line[:, 0], line[:, 1], color=rail, linewidth=rail_w,
            alpha=0.75, zorder=z + 2, solid_capstyle="round")


def draw_ship_icon(ax, rec):
    hull = rec["hull"]
    if len(hull) < 3:
        return
    center, tangent, normal, half_len, half_width = principal_frame(rec)
    edge = "#40515d" if rec["style"] == "oasis" else "#3f4a53"
    hull_face = "#dfe8ee" if rec["style"] == "oasis" else "#d4dde3"
    deck = "#f7fbfd" if rec["style"] == "oasis" else "#eef3f5"
    steel = "#9dafb9" if rec["style"] == "oasis" else "#91a0a8"

    hull_patch = Polygon(hull, closed=True, facecolor=hull_face, edgecolor=edge,
                         linewidth=1.0, alpha=0.80, zorder=20)
    ax.add_patch(hull_patch)

    def xy(points):
        pts = np.asarray(points, dtype=float)
        return center + pts[:, 0:1] * half_len * tangent + pts[:, 1:2] * half_width * normal

    def poly(points, facecolor, edgecolor=None, linewidth=0.55, alpha=0.90, zorder=21):
        patch = Polygon(xy(points), closed=True, facecolor=facecolor,
                        edgecolor=edgecolor or facecolor, linewidth=linewidth,
                        alpha=alpha, zorder=zorder)
        patch.set_clip_path(hull_patch)
        ax.add_patch(patch)
        return patch

    def line(points, color, linewidth=0.75, alpha=0.90, zorder=22, style="-"):
        pts = xy(points)
        ln, = ax.plot(pts[:, 0], pts[:, 1], color=color, linewidth=linewidth,
                      alpha=alpha, zorder=zorder, linestyle=style,
                      solid_capstyle="round")
        ln.set_clip_path(hull_patch)
        return ln

    shadow = xy([[-0.94, -0.70], [0.94, -0.70]])
    ax.plot(shadow[:, 0], shadow[:, 1], color="#71818b", linewidth=1.4,
            alpha=0.34, zorder=20.5, solid_capstyle="round")

    if rec["style"] == "oasis":
        poly([[-0.82, -0.20], [0.84, -0.20], [0.84, 0.20], [-0.82, 0.20]],
             deck, edgecolor="#8ba0aa", linewidth=0.55, alpha=0.96, zorder=21)
        poly([[-0.58, -0.08], [0.30, -0.08], [0.30, 0.08], [-0.58, 0.08]],
             "#d2e2ea", edgecolor="#8da2ad", linewidth=0.45, alpha=0.92, zorder=22)
        poly([[0.44, -0.16], [0.72, -0.16], [0.72, 0.16], [0.44, 0.16]],
             "#7fc4a0", edgecolor="#4f9273", linewidth=0.45, alpha=0.86, zorder=22)
        for side in (-1, 1):
            poly([[-0.72, side * 0.35], [0.62, side * 0.35],
                  [0.62, side * 0.48], [-0.72, side * 0.48]],
                 "#7892a0", edgecolor="#5a7280", linewidth=0.28, alpha=0.72, zorder=22)
            for u0 in np.linspace(-0.62, 0.48, 7):
                poly([[u0, side * 0.55], [u0 + 0.11, side * 0.55],
                      [u0 + 0.11, side * 0.68], [u0, side * 0.68]],
                     "#f0a640", edgecolor="#b87826", linewidth=0.22, alpha=0.70, zorder=23)
        for u0 in np.linspace(-0.45, 0.18, 4):
            poly([[u0, -0.16], [u0 + 0.12, -0.16],
                  [u0 + 0.12, 0.16], [u0, 0.16]],
                 "#f9fcfd", edgecolor="#aab8bf", linewidth=0.30, alpha=0.80, zorder=23)
        line([[-0.88, 0.0], [0.88, 0.0]], "#536a77", linewidth=0.8, alpha=0.72, zorder=24)
        line([[-0.78, -0.28], [0.76, -0.28]], "#9aadba", linewidth=0.55, alpha=0.70, zorder=23)
        line([[-0.78, 0.28], [0.76, 0.28]], "#9aadba", linewidth=0.55, alpha=0.70, zorder=23)
    else:
        poly([[-0.82, -0.24], [0.74, -0.24], [0.74, 0.24], [-0.82, 0.24]],
             deck, edgecolor="#83939b", linewidth=0.55, alpha=0.94, zorder=21)
        for u0 in np.linspace(-0.62, 0.32, 5):
            poly([[u0, -0.18], [u0 + 0.16, -0.18],
                  [u0 + 0.16, 0.18], [u0, 0.18]],
                 "#b8c5cc", edgecolor="#6d7c84", linewidth=0.40, alpha=0.86, zorder=22)
        poly([[0.52, -0.30], [0.78, -0.30], [0.78, 0.30], [0.52, 0.30]],
             "#e8edf0", edgecolor="#74848c", linewidth=0.45, alpha=0.92, zorder=22)
        poly([[0.60, -0.18], [0.72, -0.18], [0.72, 0.18], [0.60, 0.18]],
             "#b5c3ca", edgecolor="#75858d", linewidth=0.35, alpha=0.82, zorder=23)
        line([[-0.78, 0.0], [0.50, 0.0]], "#61727b", linewidth=0.75, alpha=0.78, zorder=23)
        line([[-0.72, -0.34], [0.42, -0.34]], steel, linewidth=0.55, alpha=0.70, zorder=22)
        line([[-0.72, 0.34], [0.42, 0.34]], steel, linewidth=0.55, alpha=0.70, zorder=22)
        for u0 in (-0.42, -0.05, 0.30):
            line([[u0, -0.20], [u0 + 0.18, 0.20]], "#7b8c94",
                 linewidth=0.45, alpha=0.62, zorder=23)


def draw_wind_icon(ax, rec):
    cx, cy = rec["center"]
    yaw = float(rec.get("yaw", 0.0))
    blade_len = 4.8
    mast_len = 5.2
    mast_dir = yaw + math.pi / 2.0
    ax.plot([cx - math.cos(mast_dir) * mast_len * 0.35, cx + math.cos(mast_dir) * mast_len * 0.35],
            [cy - math.sin(mast_dir) * mast_len * 0.35, cy + math.sin(mast_dir) * mast_len * 0.35],
            color="#7b878d", linewidth=1.2, alpha=0.75, zorder=26)
    for k in range(3):
        a = yaw + math.pi / 2.0 + k * 2.0 * math.pi / 3.0
        ax.plot([cx, cx + math.cos(a) * blade_len],
                [cy, cy + math.sin(a) * blade_len],
                color="#58676e", linewidth=1.35, alpha=0.96, zorder=28)
    ax.add_patch(Circle((cx, cy), 1.25, facecolor="#f9fbfa",
                        edgecolor="#40515a", linewidth=1.0, zorder=29))


def draw_readable_overlays(ax, meshes):
    bridges = [rec for rec in meshes if rec["style"] in {"red_bridge", "white_bridge"}]
    ships = [rec for rec in meshes if rec["style"] in {"oasis", "tanker"}]
    winds = [rec for rec in meshes if rec["style"] == "wind"]
    for rec in ships:
        draw_ship_icon(ax, rec)
    for rec in bridges:
        draw_bridge_icon(ax, rec)
    if winds:
        centers = np.asarray([rec["center"] for rec in winds], dtype=float)
        tangent = np.array([math.cos(winds[0]["yaw"]), math.sin(winds[0]["yaw"])])
        order = np.argsort(centers @ tangent)
        sorted_centers = centers[order]
        ax.plot(sorted_centers[:, 0], sorted_centers[:, 1], color="#6f8187",
                linewidth=1.0, alpha=0.48, zorder=25)
        for rec in winds:
            draw_wind_icon(ax, rec)


def rotated_rect(ax, center, size, yaw, **kwargs):
    sx, sy = float(size[0]), float(size[1])
    transform = Affine2D().rotate(float(yaw)).translate(float(center[0]), float(center[1])) + ax.transData
    patch = Rectangle((-sx / 2.0, -sy / 2.0), sx, sy, transform=transform, **kwargs)
    ax.add_patch(patch)
    return patch


def setup_ax(ax, title):
    ax.set_facecolor("#e9f5fb")
    ax.set_xlim(*X_RANGE)
    ax.set_ylim(*Y_RANGE)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, color="#cfe2ec", linewidth=0.55, alpha=0.62)
    ax.set_xlabel("local X east (m)")
    ax.set_ylabel("local Y north (m)")
    ax.set_title(title)


def draw_visual_layer(ax, meshes, labels=True, readable_icons=True):
    visual_rgba = rasterize_visual_meshes(meshes, skip_styles={"wind"} if readable_icons else set())
    ax.imshow(visual_rgba, extent=[X_RANGE[0], X_RANGE[1], Y_RANGE[0], Y_RANGE[1]], origin="upper", zorder=3)
    if readable_icons:
        draw_readable_overlays(ax, meshes)
    if labels:
        for rec in meshes:
            short = short_label(rec["name"])
            if not short:
                continue
            cx, cy = rec["center"]
            dy = 8.0 if rec["style"] == "wind" else 0.0
            text_with_halo(ax, cx, cy + dy, short, ha="center", va="center",
                           fontsize=7.3, color="#263238", zorder=34)


def short_label(name):
    mapping = {
        "oasis_of_the_seas_static_carrier": "S1",
        "tanker_ship_static_visual": "S2",
        "kauai_left_island_visual": "Island-L",
        "kauai_center_island_visual": "Island-C",
        "kauai_right_island_visual": "Island-R",
        "golden_gate_bridge_visual": "B1",
        "helix_bridge_visual": "B2",
        "task_wind_channel_1": "W1",
        "task_wind_channel_2": "W2",
        "task_wind_channel_3": "W3",
        "task_wind_channel_4": "W4",
    }
    return mapping.get(name, "")


def draw_planner_layer(ax, scene, include_wind_boxes=True):
    for item in scene.get("buoys", []) or []:
        if not item.get("include_in_cloud", True):
            continue
        x, y = item["center"][:2]
        r = float(item.get("radius", 1.0))
        ax.add_patch(Circle((x, y), r, facecolor="#f6b84d", edgecolor="#b06a00",
                            linewidth=1.25, alpha=0.85, zorder=18))
        label = item["name"].split("_")[0].upper()
        dx, dy = BUOY_LABEL_OFFSETS.get(label, (2.0, 2.0))
        text_with_halo(ax, x + dx, y + dy, label, fontsize=7.2,
                       color="#7a4300", zorder=33)

    for item in scene.get("box_obstacles", []) or []:
        if not item.get("include_in_cloud", True):
            continue
        if not include_wind_boxes and "wind_channel_cloud" in item.get("name", ""):
            continue
        patch = rotated_rect(
            ax,
            item.get("center", [0.0, 0.0, 0.0]),
            item.get("size", [1.0, 1.0, 1.0])[:2],
            item.get("yaw", 0.0),
            facecolor="#7d8790",
            edgecolor="#40484f",
            linewidth=1.2,
            alpha=0.22,
            zorder=15,
        )
        patch.set_hatch("///")

    for item in scene.get("dynamic_obstacles", []) or []:
        center = item.get("center", [0.0, 0.0, 0.0])
        size = item.get("size", [1.0, 1.0, 1.0])
        patch = rotated_rect(
            ax, center, size[:2], item.get("yaw", 0.0),
            facecolor="#3aa4c7", edgecolor="#176073", linewidth=1.2,
            alpha=0.55, zorder=16,
        )
        patch.set_hatch("\\\\")
        text_with_halo(ax, center[0], center[1], "D1", ha="center", va="center",
                       fontsize=7.0, color="#0b3945", zorder=33)


def draw_waypoints(ax, scene):
    points = []
    for item in scene.get("waypoints", []) or []:
        name = item["name"].split("_")[0].upper()
        x, y = item["position"][:2]
        points.append((x, y))
        ax.scatter([x], [y], color="#174ea6", edgecolor="white",
                   linewidth=0.8, s=34, zorder=31)
        dx, dy = WAYPOINT_LABEL_OFFSETS.get(name, (1.7, 1.7))
        text_with_halo(ax, x + dx, y + dy, name, fontsize=7.5,
                       color="#08306b", zorder=36)
    if points:
        ax.plot([p[0] for p in points], [p[1] for p in points],
                color="#174ea6", linewidth=2.25, alpha=0.96, zorder=30)


def save_figure(path, title, draw_planner=False, draw_points=False, show_title=True, show_legend=True, include_wind_boxes=True):
    scene = yaml.safe_load(SCENE_PATH.read_text(encoding="utf-8"))
    meshes = [load_visual_mesh(item) for item in scene.get("visual_vessels", []) or [] if item.get("mesh_uri")]
    fig, ax = plt.subplots(figsize=(13.0, 8.2), dpi=180)
    setup_ax(ax, title if show_title else "")
    draw_visual_layer(ax, meshes)
    if draw_planner:
        draw_planner_layer(ax, scene, include_wind_boxes=include_wind_boxes)
    if draw_points:
        draw_waypoints(ax, scene)

    if show_legend:
        handles = [
            Line2D([0], [0], color="#7c9674", lw=2, label="visual mesh footprint + readable icons"),
        ]
        if draw_planner:
            handles.append(Line2D([0], [0], color="#b06a00", marker="o", linestyle="", label="planner buoy/cylinder"))
            handles.append(Line2D([0], [0], color="#40484f", lw=2, label="planner box/cloud footprint"))
        if draw_points:
            handles.append(Line2D([0], [0], color="#174ea6", lw=2, marker="o", label="current P0-P8"))
        ax.legend(handles=handles, loc="upper right", fontsize=7.5)

    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)
    return scene, meshes


def write_manifest(scene, meshes):
    visual_rows = []
    item_by_name = {item.get("name"): item for item in scene.get("visual_vessels", []) or []}
    for rec in meshes:
        item = item_by_name.get(rec["name"], {})
        center = item.get("center", [0.0, 0.0, 0.0])
        scale = as_scale3(item.get("scale", item.get("mesh_scale", 1.0)))
        row = {
            "name": rec["name"],
            "style": rec["style"],
            "mesh_path": rec["mesh_path"],
            "center_x": float(center[0]) if len(center) > 0 else 0.0,
            "center_y": float(center[1]) if len(center) > 1 else 0.0,
            "center_z": float(center[2]) if len(center) > 2 else 0.0,
            "yaw_rad": yaw_of(item),
            "scale_x": float(scale[0]),
            "scale_y": float(scale[1]),
            "scale_z": float(scale[2]),
            "vertices": rec["vertices"],
        }
        row.update(rec["bounds"])
        visual_rows.append(row)
    with (OUT_DIR / "visual_mesh_footprints.csv").open("w", encoding="utf-8", newline="") as fh:
        fields = [
            "name", "style", "mesh_path", "center_x", "center_y", "center_z",
            "yaw_rad", "scale_x", "scale_y", "scale_z", "vertices",
            "min_x", "max_x", "min_y", "max_y", "min_z", "max_z",
        ]
        writer = csv.DictWriter(fh, fieldnames=fields)
        writer.writeheader()
        writer.writerows(visual_rows)

    with (OUT_DIR / "route_waypoints.csv").open("w", encoding="utf-8", newline="") as fh:
        fields = [
            "index", "label", "name", "x", "y", "z", "yaw_rad", "radius_m",
            "hold_time_sec", "max_duration_sec",
        ]
        writer = csv.DictWriter(fh, fieldnames=fields)
        writer.writeheader()
        for idx, item in enumerate(scene.get("waypoints", []) or []):
            pos = item.get("position", [0.0, 0.0, 0.0])
            name = item.get("name", f"p{idx}")
            writer.writerow({
                "index": idx,
                "label": name.split("_")[0].upper(),
                "name": name,
                "x": float(pos[0]),
                "y": float(pos[1]),
                "z": float(pos[2]) if len(pos) > 2 else 0.0,
                "yaw_rad": float(item.get("yaw", 0.0)),
                "radius_m": float(item.get("radius", 0.0)),
                "hold_time_sec": float(item.get("hold_time", 0.0)),
                "max_duration_sec": float(item.get("max_duration_sec", 0.0)),
            })

    obstacle_rows = []
    for item in scene.get("buoys", []) or []:
        center = item.get("center", [0.0, 0.0, 0.0])
        obstacle_rows.append({
            "type": "buoy_cylinder",
            "name": item.get("name", ""),
            "include_in_cloud": bool(item.get("include_in_cloud", True)),
            "visual": bool(item.get("visual", True)),
            "center_x": float(center[0]),
            "center_y": float(center[1]),
            "center_z": float(center[2]) if len(center) > 2 else 0.0,
            "radius_m": float(item.get("radius", 0.0)),
            "height_m": float(item.get("height", 0.0)),
            "size_x": "",
            "size_y": "",
            "size_z": "",
            "yaw_rad": float(item.get("yaw", 0.0)),
            "shape": "cylinder",
            "motion_type": "",
            "motion_axis_x": "",
            "motion_axis_y": "",
            "motion_axis_z": "",
            "motion_amplitude_m": "",
            "motion_period_sec": "",
            "motion_phase_rad": "",
        })
    for item in scene.get("box_obstacles", []) or []:
        center = item.get("center", [0.0, 0.0, 0.0])
        size = item.get("size", [0.0, 0.0, 0.0])
        obstacle_rows.append({
            "type": "static_box",
            "name": item.get("name", ""),
            "include_in_cloud": bool(item.get("include_in_cloud", True)),
            "visual": bool(item.get("visual", True)),
            "center_x": float(center[0]),
            "center_y": float(center[1]),
            "center_z": float(center[2]) if len(center) > 2 else 0.0,
            "radius_m": "",
            "height_m": "",
            "size_x": float(size[0]) if len(size) > 0 else 0.0,
            "size_y": float(size[1]) if len(size) > 1 else 0.0,
            "size_z": float(size[2]) if len(size) > 2 else 0.0,
            "yaw_rad": float(item.get("yaw", 0.0)),
            "shape": "box",
            "motion_type": "",
            "motion_axis_x": "",
            "motion_axis_y": "",
            "motion_axis_z": "",
            "motion_amplitude_m": "",
            "motion_period_sec": "",
            "motion_phase_rad": "",
        })
    for item in scene.get("dynamic_obstacles", []) or []:
        center = item.get("center", [0.0, 0.0, 0.0])
        size = item.get("size", [0.0, 0.0, 0.0])
        motion = item.get("motion", {}) or {}
        axis = motion.get("axis_vector", ["", "", ""])
        obstacle_rows.append({
            "type": "dynamic_obstacle",
            "name": item.get("name", ""),
            "include_in_cloud": bool(item.get("include_in_cloud", True)),
            "visual": bool(item.get("visual", True)),
            "center_x": float(center[0]),
            "center_y": float(center[1]),
            "center_z": float(center[2]) if len(center) > 2 else 0.0,
            "radius_m": "",
            "height_m": "",
            "size_x": float(size[0]) if len(size) > 0 else 0.0,
            "size_y": float(size[1]) if len(size) > 1 else 0.0,
            "size_z": float(size[2]) if len(size) > 2 else 0.0,
            "yaw_rad": float(item.get("yaw", 0.0)),
            "shape": item.get("shape", "box"),
            "motion_type": motion.get("type", ""),
            "motion_axis_x": axis[0] if len(axis) > 0 else "",
            "motion_axis_y": axis[1] if len(axis) > 1 else "",
            "motion_axis_z": axis[2] if len(axis) > 2 else "",
            "motion_amplitude_m": motion.get("amplitude", ""),
            "motion_period_sec": motion.get("period_sec", ""),
            "motion_phase_rad": motion.get("phase_rad", ""),
        })
    with (OUT_DIR / "planner_obstacles.csv").open("w", encoding="utf-8", newline="") as fh:
        fields = [
            "type", "name", "include_in_cloud", "visual", "center_x",
            "center_y", "center_z", "radius_m", "height_m", "size_x",
            "size_y", "size_z", "yaw_rad", "shape", "motion_type",
            "motion_axis_x", "motion_axis_y", "motion_axis_z",
            "motion_amplitude_m", "motion_period_sec", "motion_phase_rad",
        ]
        writer = csv.DictWriter(fh, fieldnames=fields)
        writer.writeheader()
        writer.writerows(obstacle_rows)

    layer_rows = [
        {
            "file": STANDARD_OUTPUTS["clean"],
            "role": "01_clean_visual_base",
            "has_axes": True,
            "has_title": False,
            "has_legend": False,
            "visual_mesh_csv": "visual_mesh_footprints.csv",
            "planner_obstacle_csv": "",
            "waypoint_csv": "",
            "include_wind_boxes": False,
            "notes": "Clean visual base only; no planner obstacles or route points.",
        },
        {
            "file": STANDARD_OUTPUTS["planner"],
            "role": "02_visual_plus_planner_obstacles",
            "has_axes": True,
            "has_title": True,
            "has_legend": True,
            "visual_mesh_csv": "visual_mesh_footprints.csv",
            "planner_obstacle_csv": "planner_obstacles.csv",
            "waypoint_csv": "",
            "include_wind_boxes": True,
            "notes": "Visual base plus planner-visible obstacle footprints; no route points.",
        },
        {
            "file": STANDARD_OUTPUTS["route"],
            "role": "03_visual_planner_route_p0_p8",
            "has_axes": True,
            "has_title": True,
            "has_legend": True,
            "visual_mesh_csv": "visual_mesh_footprints.csv",
            "planner_obstacle_csv": "planner_obstacles.csv",
            "waypoint_csv": "route_waypoints.csv",
            "include_wind_boxes": True,
            "notes": "Current P0-P8 route review on authoritative geometry.",
        },
    ]
    with (OUT_DIR / "map_layer_index.csv").open("w", encoding="utf-8", newline="") as fh:
        fields = [
            "file", "role", "has_axes", "has_title", "has_legend",
            "visual_mesh_csv", "planner_obstacle_csv", "waypoint_csv",
            "include_wind_boxes", "notes",
        ]
        writer = csv.DictWriter(fh, fieldnames=fields)
        writer.writeheader()
        writer.writerows(layer_rows)

    meta = {
        "scene_path": package_display_path(SCENE_PATH),
        "coordinate_extent": {"x": list(X_RANGE), "y": list(Y_RANGE)},
        "source": "generated from active scene YAML plus transformed mesh/model geometry",
        "outputs": [
            STANDARD_OUTPUTS["clean"],
            STANDARD_OUTPUTS["planner"],
            STANDARD_OUTPUTS["route"],
            "visual_mesh_footprints.csv",
            "route_waypoints.csv",
            "planner_obstacles.csv",
            "map_layer_index.csv",
        ],
    }
    (OUT_DIR / "map_manifest.json").write_text(json.dumps(meta, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    readme = [
        "# Quick-Complex Authoritative Map",
        "",
        "Generated from the active quick-complex scene YAML and transformed mesh/model geometry.",
        "The retained PNGs are review figures with axes/borders, not raw coordinate rasters.",
        "Geometry comes from the retained CSV/YAML/mesh chain; visual styling is",
        "cosmetic and clipped to the same footprints.",
        "",
        "## Outputs",
        "",
        "- `01_clean_visual_base.png`: clean visual base; no planner obstacles, no route points.",
        "- `02_visual_planner_obstacles.png`: same visual base plus planner-visible obstacle footprints; no route points.",
        "- `03_visual_planner_route_p0_p8.png`: same visual/planner layers plus the current P0-P8 route.",
        "- `visual_mesh_footprints.csv`: transformed mesh centers and footprint bounds used to render visual objects.",
        "- `route_waypoints.csv`: current P0-P8 route point positions, yaw, radius, hold, and timeout data.",
        "- `planner_obstacles.csv`: planner-visible static/dynamic obstacle positions and dimensions.",
        "- `map_layer_index.csv`: PNG-to-CSV layer/source index. No PNG should be treated as standalone evidence.",
        "- `map_manifest.json`: source paths and coordinate extent.",
        "",
        "Legacy descriptive PNG names were removed from the retained display path to",
        "avoid old-noise lookup mistakes. Use only the numbered outputs above for review.",
        "",
        "## Current Drawing Standard",
        "",
        "- Use the real YAML/mesh/planner geometry for position, scale, yaw, and footprint.",
        "- Keep readable top-down ship details and island contour shading inside those existing footprints.",
        "- Keep visual-world styling separate from planner-collision geometry and metrics.",
        "- Regenerate figures from this script instead of editing PNGs by hand.",
        "",
        "## Layer Meaning",
        "",
        "- Visual layer: transformed OBJ/SDF mesh footprint, used for visual semantics such as ship deck / shoreline placement.",
        "- Readable icon overlay: drawn from the same YAML centers/yaws and mesh-derived bridge centerlines; it is cosmetic only and does not replace the geometry source.",
        "- Planner layer: YAML buoys, boxes, and dynamic obstacle footprints that are visible to the planner/evaluator.",
        "",
        "No scene YAML, route waypoint, obstacle, or planner parameter was changed by this render step.",
        "",
        "## Plotting Rule",
        "",
        "For final planned-vs-flown plots, draw the map layers, planned P0-P8 route,",
        "and selected trajectory on the same axes from retained CSV/JSON inputs.",
        "Use numbered current outputs for display. This F250 tutorial package keeps its",
        "retained display plot at",
        "`../../../evidence/expected_route/f250_historical_planned_vs_flown.png`.",
        "Do not overlay trajectories onto a previously rendered figure PNG.",
        "",
        "## RViz Display Layer",
        "",
        "RViz scene markers may publish additional static mesh context from the same",
        "scene YAML `visual_vessels` entries, such as ships, islands, bridges, and wind",
        "turbines. Those markers are visual context only and do not change planner cloud",
        "geometry, route geometry, static obstacle safety gates, or this map authority.",
    ]
    (OUT_DIR / "README.md").write_text("\n".join(readme) + "\n", encoding="utf-8")


def main():
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    clean_path = OUT_DIR / STANDARD_OUTPUTS["clean"]
    planner_path = OUT_DIR / STANDARD_OUTPUTS["planner"]
    route_path = OUT_DIR / STANDARD_OUTPUTS["route"]

    _scene, _meshes = save_figure(clean_path, "", show_title=False, show_legend=False)
    _scene, _meshes = save_figure(
        planner_path,
        "Quick-complex visual map + planner obstacles",
        draw_planner=True,
        include_wind_boxes=True,
    )
    scene, meshes = save_figure(
        route_path,
        "Quick-complex P0-P8 route on authoritative map",
        draw_planner=True,
        draw_points=True,
        include_wind_boxes=True,
    )
    write_manifest(scene, meshes)
    print(OUT_DIR)


if __name__ == "__main__":
    main()
