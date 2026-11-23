#!/usr/bin/env python3


class ServoMap:
    """Joint angle (deg)  <->  STS3215 step count, with servo direction and the +/-60 deg window."""

    CENTER_COUNT = 1024                       # 0 deg = L-shape home
    STEPS_PER_DEG = 2047 / 180.0              # 180 deg = 2047 steps
    JOINT_LIMIT_DEG = 70.0
    MIN_COUNT = int(round(CENTER_COUNT - JOINT_LIMIT_DEG * STEPS_PER_DEG))   # 342
    MAX_COUNT = int(round(CENTER_COUNT + JOINT_LIMIT_DEG * STEPS_PER_DEG))   # 1706

    # Direction per servo:  1 -> +angle raises the step count,  -1 -> lowers it.
    # A joint moves the wrong way?  Flip ONLY that joint here (1 <-> -1).
    DIRECTIONS = {1: -1, 2: -1, 3: -1, 4: 1}

    def __init__(self, directions=None):
        self.directions = dict(self.DIRECTIONS if directions is None else directions)

    @classmethod
    def clamp_angle(cls, angle_deg: float) -> float:
        return max(-cls.JOINT_LIMIT_DEG, min(cls.JOINT_LIMIT_DEG, float(angle_deg)))

    def to_count(self, servo_id: int, angle_deg: float) -> int:
        angle = self.clamp_angle(angle_deg)
        count = int(round(self.CENTER_COUNT + self.directions[servo_id] * angle * self.STEPS_PER_DEG))
        return max(self.MIN_COUNT, min(self.MAX_COUNT, count))

    def to_deg(self, servo_id: int, count: float) -> float:
        """Not clamped, so a position outside the window stays visible."""
        return self.directions[servo_id] * (float(count) - self.CENTER_COUNT) / self.STEPS_PER_DEG
