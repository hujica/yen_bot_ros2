#!/usr/bin/env python3
import numpy as np


class ArmGeometry:
    """Link lengths, joint offsets and joint limits of the SO-ARM101.
    Lengths in metres, servo angles in degrees.

    Servo angle q = 0 on every joint is the physical L-shape home pose.
    The maths works with model angles  th = q + OFFSET  (radians):

        th1  base yaw            (0 deg points along +X)
        th2  shoulder pitch      (0 = upper arm horizontal)
        th3  elbow pitch         (= q3 - 90 deg: forearm hangs down at home)
        th4  wrist pitch         (orientation only - wrist length is 0)

    DH table:   i | a (m)    | alpha | d (m)
                1 | 0.0388   | +90   | 0.030
                2 | 0.1159   |   0   | 0
                3 | 0.1350   |   0   | 0
                4 | 0        |   0   | 0

    A1/A2/A3 follow the SO-101 URDF (pan offset 38.8 mm, upper arm 115.9 mm,
    forearm 135.0 mm).  If the tip should be the gripper tip instead of the wrist,
    use A3 = 0.164.

    If a joint turns the wrong way, flip it ONLY in hardware/servo_map.py (DIRECTIONS).
    """

    A1, D1, A2, A3 = 0.0388, 0.030, 0.1159, 0.1350
    OFFSET_DEG = np.array([0.0, 90.0, -90.0, 0.0])

    # ---- workspace (alarm zone), derived from the geometry above -----------
    WS_R_MAX = 0.98 * (A1 + A2 + A3)    # full stretch (2% safety margin)
    WS_R_MIN = 0.02                     # tip must not go behind / into the base axis
    WS_Z_MIN = 0.0                      # shoulder / elbow / tip stay above the table

    def __init__(self, joint_limit_deg: float = 70.0):
        self.lower = np.full(4, -float(joint_limit_deg))
        self.upper = np.full(4, float(joint_limit_deg))

    def _model_angles(self, q_deg) -> np.ndarray:
        """Servo angles (deg) -> model angles (rad)."""
        return np.radians(np.asarray(q_deg, dtype=float) + self.OFFSET_DEG)

    def within_limits(self, q_deg) -> bool:
        q = np.asarray(q_deg, dtype=float)
        return bool(np.all(q >= self.lower - 1e-9) and np.all(q <= self.upper + 1e-9))

    def clamp(self, q_deg) -> np.ndarray:
        return np.clip(np.asarray(q_deg, dtype=float), self.lower, self.upper)
