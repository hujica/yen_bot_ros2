#!/usr/bin/env python3
import numpy as np
from ..hardware.sts3215_driver import STS3215Driver
from ..kinematics.arm_kinematics import ArmKinematics

from .motion_planner import MotionPlanner
from .safety_monitor import SafetyMonitor


class ArmController:
    """Ties the parts together: kinematics + MotionPlanner + SafetyMonitor + servo driver.
    Cartesian jogging (resolved-rate) and joint moves, both inside a +/-70 deg joint window."""

    HOME_POSE = [0.0, 0.0, 0.0, 0.0]     # L-shape: all servos at 1024 steps
    JOINT_LIMIT_DEG = 70.0

    def __init__(self, port: str = "/dev/ttyACM0", baudrate: int = 1000000):
        self.hardware = STS3215Driver(port, baudrate)
        self.kinematics = ArmKinematics(self.JOINT_LIMIT_DEG)
        self.planner = MotionPlanner(self.kinematics)
        self.safety = SafetyMonitor()

        # state (read by the GUI)
        self.current_joint_angles = list(self.HOME_POSE)       # deg
        self.joint_targets = list(self.HOME_POSE)              # deg
        self.current_joint_velocities = [0.0] * 4              # deg/s
        self.current_tip_velocity = [0.0] * 3                  # m/s
        self.current_end_effector_pose = self.kinematics.forward_kinematics(self.HOME_POSE)

        # commands / limits
        self.cart_cmd = np.zeros(3)                            # tip velocity command (m/s)
        self.max_linear_speed = 0.04                           # m/s
        self.max_joint_speed = 30.0                            # deg/s

        # status
        self.mode = "IDLE"
        self.status, self.status_level = "Ready", "ok"
        self.sim_mode = True                                   # until real servos are found
        self.torque_on = True                                  # torque button state
        self.workspace_alarm = []                              # workspace violations, [] = OK

    @property
    def estopped(self) -> bool:
        return self.safety.estopped

    @property
    def estop_reason(self) -> str:
        return self.safety.reason

    # ------------------------------------------------------------------ setup
    def initialize_arm(self) -> bool:
        """Connect to the servos. Returns True if physical hardware was found."""
        connected = self.hardware.connect()
        self.sim_mode = not connected

        if connected:
            q = [self.hardware.get_servo_angle(i) for i in range(1, 5)]
            self.current_joint_angles = q
            self.joint_targets = list(q)
            self.current_end_effector_pose = self.kinematics.forward_kinematics(q)
            if not self.kinematics.within_limits(q):
                print("[WARN] Arm is outside the +/-70 deg window at start-up.")
        else:
            print("[WARN] Arm not connected - running in SIMULATION mode.")

        self._set_torque(True)
        self._apply_servo_speed()
        return connected

    def shutdown(self) -> None:
        self.cart_cmd[:] = 0.0
        self.hardware.disconnect()

    # --------------------------------------------------------------- commands
    def set_speed_limits(self, linear_m_s: float, joint_deg_s: float) -> None:
        self.max_linear_speed = max(1e-3, float(linear_m_s))
        joint_deg_s = max(1.0, float(joint_deg_s))
        if joint_deg_s != self.max_joint_speed:
            self.max_joint_speed = joint_deg_s
            self._apply_servo_speed()

    def set_cartesian_velocity(self, vx: float, vy: float, vz: float) -> None:
        self.cart_cmd = np.array([vx, vy, vz], dtype=float)

    def set_joint_target(self, idx: int, angle_deg: float) -> None:
        if not self.torque_on:                # torque OFF: ignore every command (hold)
            return
        k = self.kinematics
        self.joint_targets[idx] = float(np.clip(angle_deg, k.lower[idx], k.upper[idx]))

    def move_cartesian(self, dx: float, dy: float, dz: float) -> bool:
        """Go to a nearby Cartesian pose (IK) at the current joint speed."""
        if not self.torque_on:
            return False
        p = self.current_end_effector_pose
        q = self.kinematics.inverse_kinematics([p[0] + dx, p[1] + dy, p[2] + dz, p[3]])
        if q is None:
            return False
        self.joint_targets = q
        return True

    def move_to_pose(self, x_m: float, y_m: float, z_m: float, pitch_deg: float) -> bool:
        """Go to an absolute Cartesian pose [x, y, z (m), pitch (deg)] via IK."""
        if self.estopped or not self.torque_on:
            return False
        q = self.kinematics.inverse_kinematics([x_m, y_m, z_m, pitch_deg])
        if q is None:
            return False
        self.cart_cmd[:] = 0.0
        self.joint_targets = q
        return True

    def home_arm(self) -> None:
        if not self.torque_on:
            return
        self.cart_cmd[:] = 0.0
        self.joint_targets = list(self.HOME_POSE)

    def emergency_stop(self, reason: str = "manual") -> None:
        print(f"[ArmController] EMERGENCY STOP ({reason})")
        self.safety.trigger(reason)
        self.cart_cmd[:] = 0.0
        self.joint_targets = list(self.current_joint_angles)
        self.current_joint_velocities = [0.0] * 4
        self.current_tip_velocity = [0.0] * 3
        self._set_torque(False)

    def reset_estop(self) -> None:
        self._sync_from_servos()              # the arm was limp: take its real pose first
        self._set_torque(True)
        self.joint_targets = list(self.current_joint_angles)
        self.safety.reset()
        self.status, self.status_level = "Ready", "ok"

    def set_torque_enabled(self, enable: bool) -> None:
        """Torque button.  OFF: servos limp, commands ignored, pose held in software.
        ON: goal := present position first (driver does it), so the arm never jumps."""
        if self.estopped or enable == self.torque_on:
            return
        self.cart_cmd[:] = 0.0
        self.current_joint_velocities = [0.0] * 4
        self.current_tip_velocity = [0.0] * 3
        self._sync_from_servos()
        self._set_torque(enable)

    # ------------------------------------------------------------ main update
    def update(self, dt: float) -> None:
        """Advance the arm by dt seconds (call at a fixed rate, e.g. 50 Hz)."""
        if self.estopped:
            self.mode = "E-STOP"
            self._refresh_status(blocked=False)
            return

        if not self.torque_on:                # torque OFF: hold, only read the real pose
            self._hold_step()
            return

        q = np.array(self.current_joint_angles, dtype=float)
        plan = self.planner.plan(q, self.cart_cmd, self.joint_targets, self.max_joint_speed, dt)
        self.mode = plan.mode
        self.joint_targets = plan.targets

        self._apply(q + plan.step, plan.step / dt)
        if not self.estopped:                # _apply may auto E-stop (servo comm lost)
            self._refresh_status(plan.blocked)

    def _apply(self, new_q: np.ndarray, qdot: np.ndarray) -> None:
        """Store the new state and send it to the servos."""
        moved = bool(np.any(np.abs(new_q - np.array(self.current_joint_angles)) > 1e-9))
        self.current_joint_angles = new_q.tolist()
        self.current_joint_velocities = qdot.tolist()
        self.current_end_effector_pose = self.kinematics.forward_kinematics(new_q)
        self.current_tip_velocity = self.kinematics.tip_velocity(new_q, qdot).tolist()
        if not moved:
            return

        # list (not generator): every servo gets its command even if one fails
        results = [self.hardware.set_servo_angle(i + 1, a) for i, a in enumerate(self.current_joint_angles)]
        if not self.sim_mode and self.safety.servo_write_failed(results):
            self.emergency_stop("servo communication lost")

    # ---------------------------------------------------------------- helpers
    def _refresh_status(self, blocked: bool) -> None:
        self.workspace_alarm = self.kinematics.workspace_violations(self.current_joint_angles)
        self.status, self.status_level = self.safety.status(
            self.mode, blocked, self.sim_mode,
            self.kinematics.within_limits(self.current_joint_angles),
            " / ".join(self.workspace_alarm), not self.torque_on)

    def _set_torque(self, enable: bool) -> None:
        for i in range(1, 5):
            self.hardware.enable_torque(i, enable)
        self.torque_on = enable

    def _sync_from_servos(self) -> None:
        """Read the real joint angles; targets follow them."""
        q = [self.hardware.get_servo_angle(i) for i in range(1, 5)]
        self.current_joint_angles = q
        self.joint_targets = list(q)
        self.current_end_effector_pose = self.kinematics.forward_kinematics(q)

    def _hold_step(self) -> None:
        """Torque OFF: nothing is commanded; show the real pose (arm may be moved by hand)."""
        self.mode = "TORQUE OFF"
        self.cart_cmd[:] = 0.0
        self._sync_from_servos()
        self.current_joint_velocities = [0.0] * 4
        self.current_tip_velocity = [0.0] * 3
        self._refresh_status(blocked=False)

    def _apply_servo_speed(self) -> None:
        ceiling = self.max_joint_speed * 1.5     # servo may run a bit faster than our ramp
        for i in range(1, 5):
            self.hardware.set_servo_speed(i, ceiling)
