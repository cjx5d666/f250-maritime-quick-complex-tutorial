#!/usr/bin/env python3
import argparse
import os
import re
import subprocess
import tkinter as tk
from tkinter import font as tkfont


def screen_size():
    try:
        out = subprocess.check_output(["xrandr"], stderr=subprocess.DEVNULL, text=True)
        for line in out.splitlines():
            if " connected" not in line:
                continue
            for token in line.split():
                if "x" in token and "+" in token:
                    size = token.split("+", 1)[0]
                    width, height = size.split("x", 1)
                    return int(width), int(height)
    except Exception:
        pass
    return 2358, 1248


def default_geometry(width, height):
    screen_w, screen_h = screen_size()
    margin = 28
    x = max(0, screen_w - int(width) - margin)
    y = max(0, screen_h - int(height) - margin)
    return "%dx%d+%d+%d" % (int(width), int(height), x, y)


def classify(line):
    if line.startswith("=====") or line.startswith("=========="):
        return "major"
    if re.match(r"^\[[^]]+\]", line):
        return "minor"
    if "PASS" in line:
        return "pass"
    if "FAIL" in line:
        return "fail"
    if not line.strip():
        return "dim"
    return None


def main():
    parser = argparse.ArgumentParser(description="Show a colorized F250 metrics log window.")
    parser.add_argument("--title", required=True)
    parser.add_argument("--log", required=True)
    parser.add_argument("--geometry", default="")
    parser.add_argument("--width", type=int, default=980)
    parser.add_argument("--height", type=int, default=520)
    parser.add_argument("--font-size", type=int, default=14)
    parser.add_argument("--no-topmost", action="store_true")
    args = parser.parse_args()

    root = tk.Tk()
    root.title(args.title)
    root.geometry(args.geometry or default_geometry(args.width, args.height))
    root.configure(bg="#101820")
    if not args.no_topmost:
        root.attributes("-topmost", True)

    text = tk.Text(
        root,
        bg="#101820",
        fg="#f1f5f9",
        insertbackground="#f1f5f9",
        wrap="none",
        padx=14,
        pady=10,
        borderwidth=0,
    )
    text.pack(fill="both", expand=True)
    text.configure(font=tkfont.Font(family="DejaVu Sans Mono", size=args.font_size))
    text.tag_configure(
        "major",
        foreground="#22d3ee",
        font=tkfont.Font(family="DejaVu Sans Mono", size=args.font_size, weight="bold"),
    )
    text.tag_configure(
        "minor",
        foreground="#fbbf24",
        font=tkfont.Font(family="DejaVu Sans Mono", size=args.font_size, weight="bold"),
    )
    text.tag_configure(
        "pass",
        foreground="#22c55e",
        font=tkfont.Font(family="DejaVu Sans Mono", size=args.font_size, weight="bold"),
    )
    text.tag_configure(
        "fail",
        foreground="#ef4444",
        font=tkfont.Font(family="DejaVu Sans Mono", size=args.font_size, weight="bold"),
    )
    text.tag_configure("dim", foreground="#94a3b8")
    state = {"last": None}

    def refresh():
        try:
            with open(args.log, "r", encoding="utf-8", errors="replace") as handle:
                data = handle.read()
        except OSError:
            data = "waiting for log: %s\n" % args.log
        if data != state["last"]:
            state["last"] = data
            text.configure(state="normal")
            text.delete("1.0", "end")
            for line in data.splitlines():
                tag = classify(line)
                text.insert("end", line + "\n", tag if tag else ())
            text.see("end")
            text.configure(state="disabled")
        root.after(500, refresh)

    refresh()
    root.mainloop()


if __name__ == "__main__":
    main()
