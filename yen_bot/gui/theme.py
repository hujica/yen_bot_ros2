#!/usr/bin/env python3
"""Colours, fonts and small widget helpers shared by every GUI file."""
import tkinter as tk
from tkinter import ttk

C = dict(
    bg="#0d1117", panel="#161b22", card="#1c2330", btn="#263248", edge="#2b3445",
    text="#e6edf3", muted="#8b98a9", accent="#4fc3f7", violet="#9d7cff",
    ok="#3ddc97", warn="#ffb74d", danger="#ff5370", grid="#1f2736",
)
JOINT_COLORS = ["#4fc3f7", "#9d7cff", "#3ddc97", "#ffb74d"]
JOINT_NAMES = ["J1  Base", "J2  Shoulder", "J3  Elbow", "J4  Wrist"]
MONO = "Consolas"


def apply_style(root):
    """Dark ttk theme."""
    root.configure(bg=C["bg"])
    s = ttk.Style(root)
    s.theme_use("clam")
    s.configure(".", background=C["bg"], foreground=C["text"], font=("Segoe UI", 10))
    s.configure("Card.TFrame", background=C["card"], bordercolor=C["edge"], borderwidth=1, relief="solid")
    s.configure("Inner.TFrame", background=C["card"])
    s.configure("Card.TLabel", background=C["card"], foreground=C["text"])
    s.configure("Title.TLabel", background=C["card"], foreground=C["accent"], font=("Segoe UI", 9, "bold"))
    s.configure("Muted.TLabel", background=C["card"], foreground=C["muted"], font=("Segoe UI", 9))
    s.configure("Head.TLabel", font=("Segoe UI", 20, "bold"))
    s.configure("Sub.TLabel", foreground=C["muted"], font=("Segoe UI", 10))
    s.configure("Jog.TButton", background=C["btn"], foreground=C["text"], bordercolor=C["edge"],
                focusthickness=0, padding=(6, 9), font=("Segoe UI", 10, "bold"))
    s.map("Jog.TButton", background=[("pressed", C["accent"]), ("active", "#324260")],
          foreground=[("pressed", C["bg"])])
    s.configure("Home.TButton", background=C["violet"], foreground="white", bordercolor=C["violet"],
                focusthickness=0, padding=(6, 9), font=("Segoe UI", 10, "bold"))
    s.map("Home.TButton", background=[("pressed", "#7d5fe0"), ("active", "#b199ff")])
    s.configure("Horizontal.TScale", background=C["card"], troughcolor=C["bg"], bordercolor=C["edge"],
                lightcolor=C["accent"], darkcolor=C["accent"])
    s.configure("TSpinbox", fieldbackground=C["bg"], foreground=C["text"], background=C["btn"],
                arrowcolor=C["text"], bordercolor=C["edge"], insertcolor=C["text"])


def card(parent, title):
    """A titled card; returns the inner frame to put widgets in."""
    outer = ttk.Frame(parent, style="Card.TFrame", padding=(14, 10))
    outer.pack(fill="x", pady=5)
    ttk.Label(outer, text=title.upper(), style="Title.TLabel").pack(anchor="w", pady=(0, 8))
    body = ttk.Frame(outer, style="Inner.TFrame")
    body.pack(fill="x")
    return body


def mono_label(parent, text="", size=11, fg=None, width=None):
    return tk.Label(parent, text=text, bg=C["card"], fg=fg or C["text"],
                    font=(MONO, size, "bold"), width=width, anchor="w")
