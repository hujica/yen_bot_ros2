#!/usr/bin/env python3
"""All the controls on the left: pose read-out, jog pad, speed inputs, joint sliders, torque and E-stop buttons."""
import tkinter as tk
from tkinter import ttk

import numpy as np

from .theme import C, JOINT_COLORS, JOINT_NAMES, card, mono_label


class PosePanel:
    """End-effector pose read-out, direct target pose input (IK), and tip velocity."""

    def __init__(self, parent, on_go=None):
        self.on_go = on_go
        self._init_done = False
        b = card(parent, "End-effector pose & velocity")
        self.pose_lbls, self.pose_vars, self.pose_entries = [], [], []
        for col, (name, unit) in enumerate([("X", "mm"), ("Y", "mm"), ("Z", "mm"), ("PITCH", "°")]):
            ttk.Label(b, text=f"{name} ({unit})", style="Muted.TLabel").grid(row=0, column=col, padx=6, sticky="w")
            lbl = mono_label(b, "0.0", 16, C["accent"], width=7)
            lbl.grid(row=1, column=col, padx=4, pady=(0, 2))
            self.pose_lbls.append(lbl)
            var = tk.StringVar(value="0.0")
            ent = tk.Entry(b, textvariable=var, width=7, justify="right",
                           bg=C["btn"], fg=C["text"], insertbackground=C["text"],
                           relief="flat", font=("Consolas", 11, "bold"))
            ent.grid(row=2, column=col, padx=4, pady=(2, 4), ipady=3)
            ent.bind("<Return>", lambda e: self._submit())
            self.pose_vars.append(var)
            self.pose_entries.append(ent)
            b.columnconfigure(col, weight=1)
        self.btn_go = ttk.Button(b, text="GO", style="Home.TButton", width=5, command=self._submit)
        self.btn_go.grid(row=2, column=4, padx=(4, 2), pady=(2, 4), sticky="nsew")
        self.lbl_feedback = mono_label(b, "", 9, C["warn"])
        self.lbl_feedback.grid(row=3, column=0, columnspan=5, padx=6, sticky="w")
        ttk.Label(b, text="TIP VELOCITY", style="Muted.TLabel"
                  ).grid(row=4, column=0, columnspan=5, padx=2, sticky="w")
        self.lbl_tip_v = mono_label(b, "", 10, C["ok"])
        self.lbl_tip_v.grid(row=5, column=0, columnspan=5, padx=8, sticky="w")

    def _submit(self):
        if self.on_go is None:
            return
        try:
            vals = [float(v.get()) for v in self.pose_vars]
        except ValueError:
            self.lbl_feedback.configure(text="Invalid number", fg=C["danger"])
            return
        ok = self.on_go(vals[0] / 1000.0, vals[1] / 1000.0, vals[2] / 1000.0, vals[3])
        if ok:
            self.lbl_feedback.configure(text=f"Moving to ({vals[0]:.1f}, {vals[1]:.1f}, {vals[2]:.1f}) mm", fg=C["ok"])
        else:
            self.lbl_feedback.configure(text="Unreachable / outside +/-70° limit", fg=C["danger"])

    def set_enabled(self, enabled: bool):
        st = "normal" if enabled else "disabled"
        for ent in self.pose_entries:
            ent.configure(state=st)
        self.btn_go.state(["!disabled"] if enabled else ["disabled"])

    def refresh(self, pose, tip_velocity):
        vals = (pose[0] * 1000, pose[1] * 1000, pose[2] * 1000, pose[3])
        for lbl, v in zip(self.pose_lbls, vals):
            lbl.configure(text=f"{v:7.1f}")
        if not self._init_done:
            for var, v in zip(self.pose_vars, vals):
                var.set(f"{v:.1f}")
            self._init_done = True
        v = np.array(tip_velocity) * 1000
        self.lbl_tip_v.configure(
            text=f"vx{v[0]:+6.1f}  vy{v[1]:+6.1f}  vz{v[2]:+6.1f}   |v| {np.linalg.norm(v):5.1f} mm/s")


class JogPanel:
    """Cartesian jog pad (hold a button to move) + HOME.  `direction` is the held (x, y, z) or None."""

    PAD = [("▲  +X", (1, 0, 0), 0, 1), ("▼  −X", (-1, 0, 0), 2, 1),
           ("◀  −Y", (0, 1, 0), 1, 0), ("+Y  ▶", (0, -1, 0), 1, 2),
           ("⬆  +Z", (0, 0, 1), 0, 4), ("⬇  −Z", (0, 0, -1), 2, 4)]

    def __init__(self, parent, on_home):
        self.direction = None
        b = card(parent, "Cartesian jog  ·  hold to move")
        self.buttons = []
        for text, vec, r, c in self.PAD:
            btn = ttk.Button(b, text=text, style="Jog.TButton", width=8)
            btn.grid(row=r, column=c, padx=3, pady=2, sticky="nsew")
            btn.bind("<ButtonPress-1>", lambda e, v=vec: self._hold(v))
            btn.bind("<ButtonRelease-1>", lambda e: self._hold(None))
            self.buttons.append(btn)
        home = ttk.Button(b, text="HOME", style="Home.TButton", width=8, command=on_home)
        home.grid(row=1, column=1, padx=3, pady=2, sticky="nsew")
        self.buttons.append(home)
        b.columnconfigure(3, minsize=18)
        ttk.Label(b, text="X forward · Y right · Z up   (resolved-rate control)", style="Muted.TLabel"
                  ).grid(row=3, column=0, columnspan=5, pady=(2, 0), sticky="w")

    def _hold(self, vec):
        self.direction = vec

    def release(self):
        self.direction = None

    def set_enabled(self, enabled: bool):
        """Torque OFF -> the jog pad is locked."""
        for btn in self.buttons:
            btn.state(["!disabled"] if enabled else ["disabled"])
        if not enabled:
            self.direction = None


class SpeedPanel:
    """Linear tip speed and max joint speed inputs."""

    def __init__(self, parent):
        b = card(parent, "Velocity input")
        self.lin_var = tk.DoubleVar(value=40)
        self.ang_var = tk.DoubleVar(value=30)
        rows = [("Linear (tip)", self.lin_var, 5, 150, 5, "mm/s"),
                ("Joint max", self.ang_var, 5, 120, 5, "°/s")]
        for r, (name, var, lo, hi, inc, unit) in enumerate(rows):
            ttk.Label(b, text=name, style="Card.TLabel").grid(row=r, column=0, sticky="w", pady=2)
            ttk.Scale(b, from_=lo, to=hi, variable=var, length=170,
                      command=lambda v, var=var: var.set(round(float(v)))).grid(row=r, column=1, padx=10)
            ttk.Spinbox(b, from_=lo, to=hi, increment=inc, textvariable=var, width=5).grid(row=r, column=2)
            ttk.Label(b, text=unit, style="Muted.TLabel").grid(row=r, column=3, padx=(2, 0))

    @staticmethod
    def _read(var, lo, hi, default):
        try:
            return float(np.clip(float(var.get()), lo, hi))
        except (tk.TclError, ValueError):
            return default

    def read(self):
        """Returns (linear mm/s, joint deg/s), always inside the allowed range."""
        return self._read(self.lin_var, 5, 150, 40), self._read(self.ang_var, 5, 120, 30)


class JointPanel:
    """One target slider per joint + actual angle and velocity read-outs."""

    def __init__(self, parent, limit, initial_angles, on_target):
        self.on_target = on_target          # called as on_target(joint_index, angle_deg)
        self.dragging = set()               # sliders held by the mouse (do not overwrite them)
        self.syncing = False                # True while refresh() moves the sliders itself
        self.enabled = True                 # False while torque is OFF
        self.sliders, self.lbl_act, self.lbl_vel = [], [], []

        b = card(parent, f"Joint targets  ·  −{limit:.0f}° … +{limit:.0f}°")
        for col, t in ((2, "ACTUAL"), (3, "VELOCITY")):
            ttk.Label(b, text=t, style="Muted.TLabel").grid(row=0, column=col, sticky="w", padx=4)
        for i in range(4):
            r = i + 1
            tk.Label(b, text=JOINT_NAMES[i], bg=C["card"], fg=JOINT_COLORS[i],
                     font=("Segoe UI", 10, "bold"), width=11, anchor="w").grid(row=r, column=0, pady=3)
            sc = ttk.Scale(b, from_=-limit, to=limit, length=130,
                           command=lambda v, i=i: self._on_slider(i, v))
            sc.set(initial_angles[i])
            sc.grid(row=r, column=1, padx=6)
            sc.bind("<ButtonPress-1>", lambda e, i=i: self.dragging.add(i))
            sc.bind("<ButtonRelease-1>", lambda e, i=i: self.dragging.discard(i))
            act = mono_label(b, "", 10, width=8)
            act.grid(row=r, column=2, padx=4)
            vel = mono_label(b, "", 10, C["muted"], width=10)
            vel.grid(row=r, column=3, padx=4)
            self.sliders.append(sc)
            self.lbl_act.append(act)
            self.lbl_vel.append(vel)
        self.limit = limit

    def _on_slider(self, i, value):
        if not self.syncing and self.enabled:
            self.on_target(i, round(float(value), 1))

    def set_enabled(self, enabled: bool):
        """Torque OFF -> the sliders are locked."""
        self.enabled = enabled
        if not enabled:
            self.dragging.clear()
        for sc in self.sliders:
            sc.state(["!disabled"] if enabled else ["disabled"])

    def refresh(self, angles, velocities, targets):
        self.syncing = True
        for i in range(4):
            at_limit = abs(angles[i]) >= self.limit - 0.5
            self.lbl_act[i].configure(text=f"{angles[i]:+6.1f}°", fg=C["danger"] if at_limit else C["text"])
            self.lbl_vel[i].configure(text=f"{velocities[i]:+6.1f}°/s")
            if i not in self.dragging:
                sc = self.sliders[i]
                if self.enabled:
                    sc.set(targets[i])
                else:                                    # a disabled ttk.Scale ignores set()
                    sc.state(["!disabled"])
                    sc.set(targets[i])
                    sc.state(["disabled"])
        self.syncing = False


class TorqueButton:
    """Torque ON / OFF toggle."""

    def __init__(self, parent, on_click):
        self.button = tk.Button(parent, font=("Segoe UI", 12, "bold"), relief="flat", bd=0,
                                pady=6, cursor="hand2", command=on_click)
        self.button.pack(side="left", fill="x", expand=True, padx=(0, 6))
        self.show(True)

    def show(self, on: bool, enabled: bool = True):
        if on:
            self.button.configure(text="●   TORQUE  ON   -   click to turn OFF",
                                  bg=C["ok"], fg=C["bg"], activebackground=C["ok"])
        else:
            self.button.configure(text="○   TORQUE  OFF   -   click to turn ON",
                                  bg=C["btn"], fg=C["warn"], activebackground=C["edge"])
        self.button.configure(state="normal" if enabled else "disabled")


class EStopButton:
    """Big red EMERGENCY STOP button that turns into RESET E-STOP."""

    def __init__(self, parent, on_click):
        self.stopped = False
        self.button = tk.Button(parent, bg=C["danger"], fg="white", activebackground="#ff7a90",
                                activeforeground="white", font=("Segoe UI", 12, "bold"), relief="flat",
                                bd=0, pady=6, cursor="hand2", command=on_click)
        self.button.pack(side="left", fill="x", expand=True, padx=(6, 0))
        self.show(False)

    def show(self, stopped: bool):
        self.stopped = stopped
        if stopped:
            self.button.configure(text="↺   RESET E-STOP", bg=C["warn"], fg=C["bg"])
        else:
            self.button.configure(text="⏻   EMERGENCY STOP", bg=C["danger"], fg="white")
