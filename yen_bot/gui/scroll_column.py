#!/usr/bin/env python3
"""A vertically scrolling column, so the controls stay reachable on a small screen."""
import tkinter as tk
from tkinter import ttk

from .theme import C


class ScrollColumn:
    """Put widgets into `.body`; call `.pack(...)` on the column itself."""

    def __init__(self, parent, width=430):
        self.outer = ttk.Frame(parent)
        self.canvas = tk.Canvas(self.outer, bg=C["bg"], highlightthickness=0, width=width)
        bar = ttk.Scrollbar(self.outer, orient="vertical", command=self.canvas.yview)
        self.canvas.configure(yscrollcommand=bar.set)
        bar.pack(side="right", fill="y")
        self.canvas.pack(side="left", fill="both", expand=True)

        self.body = ttk.Frame(self.canvas)
        window = self.canvas.create_window((0, 0), window=self.body, anchor="nw")
        self.body.bind("<Configure>",
                       lambda e: self.canvas.configure(scrollregion=self.canvas.bbox("all")))
        self.canvas.bind("<Configure>",
                         lambda e: self.canvas.itemconfigure(window, width=e.width))

        # mouse wheel: Windows/macOS = <MouseWheel>, Linux (the VNC desktop) = Button-4/5
        for sequence in ("<MouseWheel>", "<Button-4>", "<Button-5>"):
            self.canvas.bind_all(sequence, self._on_wheel, add="+")

    def pack(self, **options):
        self.outer.pack(**options)

    def _on_wheel(self, event):
        under = self.outer.winfo_containing(event.x_root, event.y_root)
        if under is None or not str(under).startswith(str(self.outer)):
            return                                   # pointer is not over this column
        up = event.num == 4 or getattr(event, "delta", 0) > 0
        self.canvas.yview_scroll(-1 if up else 1, "units")
