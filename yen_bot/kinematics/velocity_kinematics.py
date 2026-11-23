#!/usr/bin/env python3
import numpy as np

from .arm_geometry import ArmGeometry


class VelocityKinematics(ArmGeometry):
    """Jacobian, forward velocity and resolved-rate (inverse velocity) kinematics."""

    def jacobian(self, q_deg) -> np.ndarray:
        """3x4 Jacobian: [vx vy vz] (m/s) = J @ [dq1..dq4] (rad/s).  The wrist column is 0."""
        th1, th2, th3, _ = self._model_angles(q_deg)
        s1, c1 = np.sin(th1), np.cos(th1)
        s2, c2 = np.sin(th2), np.cos(th2)
        s23, c23 = np.sin(th2 + th3), np.cos(th2 + th3)

        r = self.A1 + self.A2 * c2 + self.A3 * c23
        dr2, dr3 = -self.A2 * s2 - self.A3 * s23, -self.A3 * s23
        dz2, dz3 = self.A2 * c2 + self.A3 * c23, self.A3 * c23
        return np.array([[-s1 * r, c1 * dr2, c1 * dr3, 0.0],
                         [c1 * r, s1 * dr2, s1 * dr3, 0.0],
                         [0.0, dz2, dz3, 0.0]])

    def tip_velocity(self, q_deg, qdot_deg) -> np.ndarray:
        """Joint speeds (deg/s) -> tip velocity (m/s)."""
        return self.jacobian(q_deg) @ np.radians(qdot_deg)

    def joint_velocity_from_cartesian(self, q_deg, v_cart, pitch_rate: float = 0.0,
                                      damping: float = 0.003):
        """Resolved-rate control with damped least squares (safe near singularities).
        v_cart in m/s.  Returns (joint speeds deg/s, manipulability)."""
        J = np.vstack([self.jacobian(q_deg), [0.0, 1.0, 1.0, 1.0]])   # + tool-pitch row
        xdot = np.array([*v_cart, pitch_rate], dtype=float)

        JJt = J @ J.T
        w = float(np.sqrt(max(np.linalg.det(JJt), 0.0)))              # manipulability
        w0 = 3e-4                                                     # "near singular" below this
        lam2 = damping ** 2 + (0.01 * (1.0 - w / w0) ** 2 if w < w0 else 0.0)
        qdot = J.T @ np.linalg.solve(JJt + lam2 * np.eye(4), xdot)
        return np.degrees(qdot), w
