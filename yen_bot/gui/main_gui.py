#!/usr/bin/env python3
"""Main window + 50 Hz control loop.  The widgets themselves live in the other gui/*.py files."""
import signal
import time
import traceback
import tkinter as tk
from tkinter import ttk

import numpy as np

from .control_panels import EStopButton, JogPanel, JointPanel, PosePanel, SpeedPanel, TorqueButton
from .header import StatusHeader
from .scroll_column import ScrollColumn
from .scene_3d import Scene3D
from .theme import apply_style
from .velocity_plot import VelocityPlot

try:
    import rclpy
except ImportError:
    rclpy = None


class SOArm101GUI:
    TICK_MS = 20          # control loop: 50 Hz
    DRAW_EVERY = 3        # redraw every 3rd tick (~17 Hz) - gentler on the VNC desktop
    ROS_EVERY = 2         # publish /joint_states every 2nd tick (~25 Hz)

    def __init__(self, root_window: tk.Tk, controller, ros_node):
        self.root = root_window
        self.ctrl = controller
        self.ros_node = ros_node
        self.limit = float(controller.kinematics.upper[0])      # joint window (+/- deg)

        self._closing = False
        self.t0 = self.last = time.perf_counter()
        self.n = 0
        self.last_err = None
        self._last_status = None
        self._ui_state = None              # (torque_on, estopped) last pushed to the widgets
        self.ang_limit = 30.0              # joint speed limit from the speed panel (deg/s)

        self._setup_window()
        apply_style(self.root)
        self._build_layout()
        self.tick()

    # ================================================================== build
    def _setup_window(self):
        self.root.title("SO-ARM101  |  Velocity Kinematics Console")
        # fixed size from the screen: more stable than "zoomed" on the VNC desktop
        sw, sh = self.root.winfo_screenwidth(), self.root.winfo_screenheight()
        ww, wh = int(sw * 0.96), int(sh * 0.90)
        self.root.geometry(f"{ww}x{wh}+{(sw - ww) // 2}+{(sh - wh) // 2}")
        self.root.minsize(min(1000, ww), min(640, wh))

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        # Esc = emergency stop (reset only with the button)
        self.root.bind("<Escape>", lambda e: None if self.ctrl.estopped else self._toggle_estop())
        try:
            signal.signal(signal.SIGINT, lambda sig, frame: self.on_close())
        except Exception:
            pass

    def _build_layout(self):
        self.header = StatusHeader(self.root, self.ctrl.hardware.is_connected)

        # Torque + E-stop bar: at the TOP (right under the header), side by side
        bar = ttk.Frame(self.root)
        bar.pack(side="top", fill="x", padx=20, pady=(0, 6))
        self.torque = TorqueButton(bar, on_click=self._toggle_torque)
        self.estop = EStopButton(bar, on_click=self._toggle_estop)

        body = ttk.Frame(self.root)
        body.pack(fill="both", expand=True, padx=20, pady=8)
        column = ScrollColumn(body, width=430)           # scrolls when the screen is short
        column.pack(side="left", fill="y", padx=(0, 16))
        left = column.body
        right = ttk.Frame(body)
        right.pack(side="right", fill="both", expand=True)

        # left column: controls
        self.pose = PosePanel(left, on_go=self.ctrl.move_to_pose)
        self.jog = JogPanel(left, on_home=self.ctrl.home_arm)
        self.speed = SpeedPanel(left)
        self.joints = JointPanel(left, self.limit, self.ctrl.current_joint_angles,
                                 on_target=self.ctrl.set_joint_target)
        # right column: graph at the bottom first, the 3D view takes the remaining space
        self.vel_plot = VelocityPlot(right)
        self.scene = Scene3D(right, self.ctrl)

    # ============================================================== callbacks
    def _toggle_torque(self):
        if self.ctrl.estopped:
            return                                  # reset the E-stop first
        self.jog.release()
        self.ctrl.set_torque_enabled(not self.ctrl.torque_on)

    def _toggle_estop(self):
        if self.ctrl.estopped:
            self.ctrl.reset_estop()
            self.estop.show(False)
        else:
            self.jog.release()
            self.ctrl.emergency_stop()
            self.estop.show(True)

    def on_close(self):
        if self._closing:
            return
        self._closing = True
        self.ros_node = None
        try:
            if rclpy is not None and rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        try:
            self.root.quit()
            self.root.destroy()
        except Exception:
            pass

    # =============================================================== main loop
    def tick(self):
        if self._closing:
            return
        if rclpy is not None and self.ros_node is not None and not rclpy.ok():
            self.on_close()
            return

        try:
            self._control_step()
            if self.n % self.DRAW_EVERY == 0:
                self._refresh_views()
            self.n += 1
        except Exception:
            msg = traceback.format_exc()
            if msg != self.last_err:
                print(msg)
                self.last_err = msg
            try:
                self.header.show_gui_error()
            except Exception:
                pass
        finally:
            if not self._closing:
                self.root.after(self.TICK_MS, self.tick)

    def _control_step(self):
        """One 50 Hz step: read inputs -> controller.update -> log -> publish."""
        c = self.ctrl
        now = time.perf_counter()
        dt = min(max(now - self.last, 1e-3), 0.1)
        self.last = now

        lin_mm_s, self.ang_limit = self.speed.read()
        c.set_speed_limits(lin_mm_s / 1000.0, self.ang_limit)
        if self.jog.direction is not None and not c.estopped and c.torque_on:
            c.set_cartesian_velocity(*(np.array(self.jog.direction, float) * c.max_linear_speed))
        else:
            c.set_cartesian_velocity(0.0, 0.0, 0.0)
        c.update(dt)

        if c.estopped and not self.estop.stopped:       # controller stopped itself (servo comm lost)
            self.jog.release()
            self.estop.show(True)
        if c.status != self._last_status:
            if c.status_level != "ok":
                print(f"[WARN] {c.status}")
            self._last_status = c.status

        self.vel_plot.record(now - self.t0, c.current_joint_velocities)
        self.scene.record_trail()
        if self.ros_node and self.n % self.ROS_EVERY == 0:
            self._publish_ros()

    def _refresh_views(self):
        c = self.ctrl
        state = (c.torque_on, c.estopped)
        if state != self._ui_state:                 # touch the widgets only when it changes
            self._ui_state = state
            self.torque.show(c.torque_on, enabled=not c.estopped)
            self.joints.set_enabled(c.torque_on)
            self.jog.set_enabled(c.torque_on)
            self.pose.set_enabled(c.torque_on)
        self.header.set_alarm(" · ".join(c.workspace_alarm))
        self.pose.refresh(c.current_end_effector_pose, c.current_tip_velocity)
        self.joints.refresh(c.current_joint_angles, c.current_joint_velocities, c.joint_targets)
        self.header.update(c.status, c.status_level, c.mode)
        self.scene.draw()
        self.vel_plot.draw(self.ang_limit)

    def _publish_ros(self):
        if rclpy is not None and not rclpy.ok():
            self.ros_node = None
            return
        try:
            ok = self.ros_node.publish_joint_states(self.ctrl.current_joint_angles,
                                                    self.ctrl.current_joint_velocities)
        except Exception as e:
            ok = False
            print(f"[GUI] ROS publish failed: {e}")
        if ok is False:
            print("[GUI] ROS context is not valid any more - ROS publishing disabled.")
            self.ros_node = None
