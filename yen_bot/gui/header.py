#!/usr/bin/env python3
"""Top bar: title, hardware/simulation tag, mode and status chip, workspace-alarm banner."""
import time
import tkinter as tk
from tkinter import ttk

from .theme import C


class StatusHeader:
    LEVEL_COLORS = {"ok": C["ok"], "warn": C["warn"], "error": C["danger"]}

    def __init__(self, root, hardware_live: bool):
        self.root = root
        head = ttk.Frame(root)
        head.pack(fill="x", padx=20, pady=(14, 4))
        ttk.Label(head, text="SO-ARM101", style="Head.TLabel").pack(side="left")
        ttk.Label(head, text="   4-DOF  ·  Velocity Kinematics Console", style="Sub.TLabel"
                  ).pack(side="left", pady=(8, 0))

        self.chip = tk.Label(head, text="READY", bg=C["ok"], fg=C["bg"], font=("Segoe UI", 10, "bold"),
                             padx=14, pady=4)
        self.chip.pack(side="right")
        self.lbl_mode = ttk.Label(head, text="IDLE", style="Sub.TLabel")
        self.lbl_mode.pack(side="right", padx=16)
        tk.Label(head, text="● HARDWARE LIVE" if hardware_live else "● SIMULATION", bg=C["bg"],
                 fg=C["ok"] if hardware_live else C["warn"], font=("Segoe UI", 9, "bold")
                 ).pack(side="right", padx=8)

        # alarm banner: always packed (fixed height) so the layout never jumps
        self.banner = tk.Label(root, text=" ", bg=C["bg"], fg="white",
                               font=("Segoe UI", 12, "bold"), pady=4)
        self.banner.pack(fill="x", padx=20, pady=(0, 2))
        self._alarmed = False
        self._banner_state = None

    def update(self, status: str, level: str, mode: str):
        self.chip.configure(text=status.upper(), bg=self.LEVEL_COLORS[level])
        self.lbl_mode.configure(text=mode)

    def set_alarm(self, text: str):
        """text = '' -> banner hidden.  Otherwise a flashing red banner (+ one bell)."""
        if text:
            bg = C["danger"] if int(time.time() * 2) % 2 == 0 else "#7a1f2e"
            msg = f"⚠  WORKSPACE ALARM  ·  {text}"
            if not self._alarmed:
                try:
                    self.root.bell()
                except tk.TclError:
                    pass
        else:
            bg, msg = C["bg"], " "
        self._alarmed = bool(text)
        if (msg, bg) != self._banner_state:              # configure only when it changed
            self._banner_state = (msg, bg)
            self.banner.configure(text=msg, bg=bg)

    def show_gui_error(self):
        self.chip.configure(text="GUI ERROR - see console", bg=C["danger"])
