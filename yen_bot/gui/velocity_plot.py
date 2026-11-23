#!/usr/bin/env python3
"""Scrolling plot of the four joint velocities (last 10 s)."""
import tkinter as tk
from collections import deque
from tkinter import ttk

import numpy as np

from .theme import C, JOINT_COLORS, MONO


class VelocityPlot:
    WINDOW_S = 10.0

    def __init__(self, parent):
        self.history = deque(maxlen=500)          # (time, v1, v2, v3, v4)
        self._flat_sig = None                     # last drawn "all zero" picture (skip identical redraws)

        frame = ttk.Frame(parent, style="Card.TFrame", padding=(10, 4))
        frame.pack(side="bottom", fill="x")
        head = ttk.Frame(frame, style="Inner.TFrame")
        head.pack(fill="x", pady=(0, 4))
        ttk.Label(head, text="JOINT VELOCITY HISTORY (°/s  ·  10s WINDOW)", style="Title.TLabel").pack(side="left")
        legend = ttk.Frame(head, style="Inner.TFrame")
        legend.pack(side="right")
        for i in range(4):
            tk.Label(legend, text=f"■ J{i + 1}", bg=C["card"], fg=JOINT_COLORS[i],
                     font=("Segoe UI", 9, "bold")).pack(side="left", padx=6)

        self.canvas = tk.Canvas(frame, bg=C["bg"], highlightthickness=0, height=70)
        self.canvas.pack(fill="x")

    def record(self, t, velocities):
        self.history.append((t, *velocities))

    def draw(self, ang_limit):
        cv = self.canvas
        w, h = max(cv.winfo_width(), 300), max(cv.winfo_height(), 50)
        lim = max(ang_limit * 1.15, 10.0)

        d = None
        if len(self.history) >= 2:
            d = np.array(self.history)
            d = d[(d[:, 0] - d[-1, 0]) >= -self.WINDOW_S]
            if len(d) < 2:
                d = None

        # idle (every velocity 0) -> the picture is identical, so do not redraw it
        flat = d is None or not np.any(np.abs(d[:, 1:]) > 1e-6)
        sig = (w, h, round(lim, 1))
        if flat and self._flat_sig == sig:
            return
        self._flat_sig = sig if flat else None

        cv.delete("all")
        pad_l, pad_r, pad_t, pad_b = 46, 14, 8, 18
        pw, ph = max(w - pad_l - pad_r, 10), max(h - pad_t - pad_b, 10)

        for frac in (-1.0, -0.5, 0.0, 0.5, 1.0):                 # horizontal grid + labels
            y = pad_t + ph * (0.5 - 0.5 * frac)
            cv.create_line(pad_l, y, pad_l + pw, y, fill=C["edge"] if frac == 0.0 else C["grid"])
            cv.create_text(pad_l - 6, y, text=f"{frac * lim:4.0f}", fill=C["muted"], anchor="e", font=(MONO, 8))

        for sec in range(-10, 1, 2):                             # vertical grid (-10 s ... 0 s)
            x = pad_l + pw * ((sec + 10.0) / 10.0)
            cv.create_line(x, pad_t, x, pad_t + ph, fill=C["grid"])
            cv.create_text(x, pad_t + ph + 11, text=f"{sec}s", fill=C["muted"], font=(MONO, 8))

        if d is None:
            return
        xs = pad_l + pw * (((d[:, 0] - d[-1, 0]) + 10.0) / 10.0)
        for j in range(4):
            ys = pad_t + ph * (0.5 - 0.5 * np.clip(d[:, j + 1] / lim, -1.0, 1.0))
            cv.create_line(*np.column_stack((xs, ys)).flatten().tolist(), fill=JOINT_COLORS[j], width=2)
