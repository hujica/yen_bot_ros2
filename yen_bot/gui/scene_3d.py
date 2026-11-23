#!/usr/bin/env python3
"""3D kinematic view of the arm (orthographic, drag with the mouse to orbit)."""
import math
import tkinter as tk
from collections import deque
from tkinter import ttk

import numpy as np

from .theme import C, MONO


class Scene3D:
    CENTER = np.array([0.10, 0.0, 0.16])          # point the camera looks at (m)

    def __init__(self, parent, controller):
        self.ctrl = controller
        self.azim, self.elev = -55.0, 24.0        # camera angles (deg)
        self.trail = deque(maxlen=200)            # end-effector path
        self._drag_xy = None
        self._sig = None                          # last drawn state: skip redraw if unchanged

        frame = ttk.Frame(parent, style="Card.TFrame", padding=(10, 8))
        frame.pack(fill="both", expand=True, pady=(5, 6))
        head = ttk.Frame(frame, style="Inner.TFrame")
        head.pack(fill="x", pady=(0, 4))
        ttk.Label(head, text="3D KINEMATIC VISUALIZER", style="Title.TLabel").pack(side="left")
        ttk.Label(head, text="Drag canvas to rotate view", style="Muted.TLabel").pack(side="right")

        self.canvas = tk.Canvas(frame, bg=C["bg"], highlightthickness=0, height=120)
        self.canvas.pack(fill="both", expand=True)
        self.canvas.bind("<ButtonPress-1>", self._on_press)
        self.canvas.bind("<B1-Motion>", self._on_drag)

    def record_trail(self):
        """Call every control step: remembers the tip position while the arm is moving."""
        if np.linalg.norm(self.ctrl.current_tip_velocity) > 1e-5:
            self.trail.append(self.ctrl.current_end_effector_pose[:3])

    # ----------------------------------------------------------------- camera
    def _on_press(self, event):
        self._drag_xy = (event.x, event.y)

    def _on_drag(self, event):
        if self._drag_xy is None:
            return
        dx, dy = event.x - self._drag_xy[0], event.y - self._drag_xy[1]
        self._drag_xy = (event.x, event.y)
        self.azim = (self.azim - dx * 0.5) % 360
        self.elev = float(np.clip(self.elev + dy * 0.4, -15.0, 85.0))

    def _project(self, pts, w, h):
        """3D points (m) -> 2D screen points (px)."""
        az, el = math.radians(self.azim), math.radians(self.elev)
        ca, sa, ce, se = math.cos(az), math.sin(az), math.cos(el), math.sin(el)

        p = np.asarray(pts, dtype=float) - self.CENTER
        x, y, z = p[:, 0], p[:, 1], p[:, 2]
        xr = -x * sa + y * ca                     # rotate about Z (azimuth) ...
        yr = x * ca + y * sa
        zr = z * ce - yr * se                     # ... then about X (elevation)

        scale = min(w * 1.0, h * 1.45)            # fit the arm + base inside the canvas
        return np.column_stack((w * 0.5 + xr * scale, h * 0.52 - zr * scale))

    # ---------------------------------------------------------------- drawing
    def draw(self):
        cv = self.canvas
        c = self.ctrl
        w, h = max(cv.winfo_width(), 50), max(cv.winfo_height(), 50)   # real canvas size (never larger, or the drawing is clipped)
        sig = (tuple(np.round(c.current_joint_angles, 2)),
               tuple(np.round(c.current_tip_velocity, 4)),
               round(self.azim, 1), round(self.elev, 1),
               len(self.trail), bool(c.workspace_alarm), w, h)
        if sig == self._sig:
            return                                # nothing changed -> no redraw (no flicker)
        self._sig = sig

        cv.delete("all")
        self._draw_floor(cv, w, h)
        self._draw_arm(cv, w, h)

    def _draw_floor(self, cv, w, h):
        def segment(a, b, **kw):
            p = self._project([a, b], w, h)
            cv.create_line(*p[0], *p[1], **kw)

        for g in np.linspace(-0.25, 0.35, 7):                    # grid
            segment([g, -0.25, 0.0], [g, 0.25, 0.0], fill=C["grid"])
        for g in np.linspace(-0.25, 0.25, 6):
            segment([-0.25, g, 0.0], [0.35, g, 0.0], fill=C["grid"])
        for tip, color, name in (([0.08, 0, 0], C["danger"], "X"),   # axes
                                 ([0, 0.08, 0], C["ok"], "Y"),
                                 ([0, 0, 0.08], C["accent"], "Z")):
            segment([0, 0, 0], tip, fill=color, width=2)
            tx, ty = self._project([tip], w, h)[0]
            cv.create_text(tx + 6, ty, text=name, fill=color, font=(MONO, 9, "bold"))

        a = np.linspace(0, 2 * np.pi, 36)                            # base ring
        ring = self._project(np.column_stack((0.045 * np.cos(a), 0.045 * np.sin(a), np.zeros_like(a))), w, h)
        cv.create_polygon(*ring.flatten(), outline=C["muted"], fill="", width=2)

        kin = self.ctrl.kinematics                                   # workspace limit rings
        for rad in (kin.WS_R_MIN, kin.WS_R_MAX):
            wr = self._project(np.column_stack((rad * np.cos(a), rad * np.sin(a), np.zeros_like(a))), w, h)
            cv.create_line(*wr.flatten(), fill=C["warn"], dash=(4, 4))

    def _draw_arm(self, cv, w, h):
        pts_3d = np.asarray(self.ctrl.kinematics.joint_positions(self.ctrl.current_joint_angles), float)
        pts_2d = self._project(pts_3d, w, h)
        arm_color = C["danger"] if self.ctrl.workspace_alarm else C["accent"]

        shadow = pts_3d.copy()
        shadow[:, 2] = 0.0
        cv.create_line(*self._project(shadow, w, h).flatten(), fill="#222b3a", width=5, capstyle="round")
        if len(self.trail) > 1:
            cv.create_line(*self._project(np.array(self.trail), w, h).flatten(), fill=C["violet"], width=2)

        cv.create_line(*pts_2d.flatten(), fill=arm_color, width=8, capstyle="round", joinstyle="round")
        for x, y in pts_2d[:-1]:
            cv.create_oval(x - 6, y - 6, x + 6, y + 6, fill="white", outline=arm_color, width=2)

        tx, ty = pts_2d[-1]                                          # tip + velocity arrow
        v = np.array(self.ctrl.current_tip_velocity)
        if np.linalg.norm(v) > 1e-4:
            ex, ey = self._project([pts_3d[-1] + v * 2.0], w, h)[0]
            cv.create_line(tx, ty, ex, ey, fill=C["ok"], width=3, arrow="last")
        cv.create_oval(tx - 7, ty - 7, tx + 7, ty + 7, fill=C["danger"], outline="white", width=1.5)
