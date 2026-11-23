#!/usr/bin/env python3
import numpy as np

from .arm_geometry import ArmGeometry


class PositionKinematics(ArmGeometry):
    """Forward / inverse kinematics and the points used to draw the arm."""

    def forward_kinematics(self, q_deg) -> list:
        """Returns [x, y, z, pitch_deg]  (metres, degrees)."""
        th1, th2, th3, th4 = self._model_angles(q_deg)
        r = self.A1 + self.A2 * np.cos(th2) + self.A3 * np.cos(th2 + th3)
        z = self.D1 + self.A2 * np.sin(th2) + self.A3 * np.sin(th2 + th3)
        return [float(r * np.cos(th1)), float(r * np.sin(th1)), float(z),
                float(np.degrees(th2 + th3 + th4))]

    def joint_positions(self, q_deg) -> np.ndarray:
        """5x3 points for drawing: base, column top, shoulder, elbow, tip."""
        th1, th2, th3, _ = self._model_angles(q_deg)
        c1, s1 = np.cos(th1), np.sin(th1)
        r_sh = self.A1
        r_el, z_el = r_sh + self.A2 * np.cos(th2), self.D1 + self.A2 * np.sin(th2)
        r_tip, z_tip = r_el + self.A3 * np.cos(th2 + th3), z_el + self.A3 * np.sin(th2 + th3)
        return np.array([[0, 0, 0],
                         [0, 0, self.D1],
                         [r_sh * c1, r_sh * s1, self.D1],
                         [r_el * c1, r_el * s1, z_el],
                         [r_tip * c1, r_tip * s1, z_tip]])

    def workspace_violations(self, q_deg) -> list:
        """[] = inside the workspace, otherwise short messages (shown in the alarm banner)."""
        th1 = self._model_angles(q_deg)[0]
        pts = self.joint_positions(q_deg)
        tip = pts[-1]
        r = float(tip[0] * np.cos(th1) + tip[1] * np.sin(th1))      # signed reach from the base axis
        msgs = []
        if r > self.WS_R_MAX:
            msgs.append(f"reach {r * 1000:.0f} mm > {self.WS_R_MAX * 1000:.0f} mm")
        elif r < self.WS_R_MIN:
            msgs.append(f"too close to base ({r * 1000:.0f} mm)")
        zmin = float(np.min(pts[2:, 2]))                            # shoulder, elbow, tip
        if zmin < self.WS_Z_MIN - 1e-9:
            msgs.append(f"below table ({zmin * 1000:.0f} mm)")
        return msgs

    def inverse_kinematics(self, pose):
        """Elbow-down IK.  pose = [x, y, z, pitch_deg].
        Returns servo angles [q1..q4] (deg), or None if unreachable / outside the limits."""
        x, y, z = pose[:3]
        phi = np.radians(pose[3] if len(pose) > 3 else 0.0)

        th1 = np.arctan2(y, x)
        rw = np.hypot(x, y) - self.A1            # shoulder -> wrist, horizontal
        zw = z - self.D1                         # shoulder -> wrist, vertical

        cos3 = (rw ** 2 + zw ** 2 - self.A2 ** 2 - self.A3 ** 2) / (2.0 * self.A2 * self.A3)
        if abs(cos3) > 1.0 + 1e-9:
            return None
        cos3 = np.clip(cos3, -1.0, 1.0)

        th3 = np.arctan2(-np.sqrt(1.0 - cos3 ** 2), cos3)     # elbow-down (th3 < 0)
        th2 = np.arctan2(zw, rw) - np.arctan2(self.A3 * np.sin(th3), self.A2 + self.A3 * np.cos(th3))
        th4 = phi - th2 - th3

        q = np.degrees([th1, th2, th3, th4]) - self.OFFSET_DEG
        return q.tolist() if self.within_limits(q) else None
