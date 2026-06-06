#!/usr/bin/env python3
import argparse
import os
import subprocess
import sys
import time


def have_display():
    return bool(os.environ.get("DISPLAY"))


def screen_size():
    try:
        out = subprocess.check_output(["xrandr"], stderr=subprocess.DEVNULL, text=True)
        for line in out.splitlines():
            if " connected" in line and "+" in line:
                for token in line.split():
                    if "x" in token and "+" in token:
                        size = token.split("+")[0]
                        w, h = size.split("x", 1)
                        return int(w), int(h)
    except Exception:
        pass
    return 2358, 1248


def layout_geometry(width, height):
    left = 64
    top = 28
    margin = 10
    main_w = max(800, width - left - margin)
    gazebo_h = min(int((height - top - margin) * 0.63), 760)
    bottom_y = top + gazebo_h + margin
    bottom_h = max(260, height - bottom_y - margin)
    rviz_w = int(main_w * 0.66)
    metrics_x = left + rviz_w + margin
    metrics_w = max(360, width - metrics_x - margin)
    return {
        "gazebo": (left, top, main_w, gazebo_h),
        "rviz": (left, bottom_y, rviz_w, bottom_h),
        "metrics": (metrics_x, bottom_y, metrics_w, bottom_h),
    }


def wnck_layout(kind):
    import gi
    gi.require_version("Gdk", "3.0")
    gi.require_version("Wnck", "3.0")
    from gi.repository import Wnck

    screen = Wnck.Screen.get_default()
    if screen is None:
        return False
    screen.force_update()
    width, height = screen_size()
    geometry = layout_geometry(width, height)
    mask = (Wnck.WindowMoveResizeMask.X | Wnck.WindowMoveResizeMask.Y |
            Wnck.WindowMoveResizeMask.WIDTH | Wnck.WindowMoveResizeMask.HEIGHT)
    moved = []
    for window in screen.get_windows():
        name = window.get_name() or ""
        target = None
        label = None
        if kind in ("all", "visual") and ("RViz" in name or "maritime_visual_acceptance" in name):
            target = geometry["rviz"]
            label = "rviz"
        elif kind in ("all", "visual") and "Gazebo" in name:
            target = geometry["gazebo"]
            label = "gazebo"
        elif kind in ("all", "metrics") and ("F250 Route Metrics" in name or "F250 FC 3.10 Metrics" in name):
            target = geometry["metrics"]
            label = "metrics"
        if target:
            try:
                window.unmaximize()
                window.set_geometry(Wnck.WindowGravity.CURRENT, mask, *target)
                window.activate(int(time.time()))
                moved.append((label, name, target))
            except Exception as exc:
                print("layout_failed %s %s: %s" % (label, name, exc), file=sys.stderr)
    screen.force_update()
    for label, name, target in moved:
        print("layout_%s=%s %s" % (label, name, target))
    return bool(moved)


def main():
    parser = argparse.ArgumentParser(description="Arrange F250 demo windows on a horizontal desktop.")
    parser.add_argument("--kind", choices=["all", "visual", "metrics"], default="all")
    parser.add_argument("--wait-sec", type=float, default=0.0)
    args = parser.parse_args()
    if not have_display():
        print("layout_skipped=no_DISPLAY")
        return 0
    if args.wait_sec > 0:
        time.sleep(args.wait_sec)
    try:
        moved = wnck_layout(args.kind)
    except Exception as exc:
        print("layout_skipped=%s" % exc)
        return 0
    if not moved:
        print("layout_no_matching_windows")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
