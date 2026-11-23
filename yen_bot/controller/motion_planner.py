#!/usr/bin/env python3
from collections import namedtuple

import numpy as np

# step: joint increment (deg) | mode: text for the GUI | blocked: hit a joint limit | targets: joint targets
MotionPlan = namedtuple("MotionPlan", "step mode blocked targets")


class MotionPlanner:
    """Decides how far every joint moves in one time step.  No hardware, no stored state."""

    def __init__(self, kinematics):
        self.kin = kinematics

    def plan(self, q, cart_cmd, joint_targets, max_joint_speed, dt) -> MotionPlan:
        if np.any(cart_cmd):
            return self._cartesian(q, cart_cmd, max_joint_speed, dt)
        return self._joint(q, joint_targets, max_joint_speed, dt)

    def _cartesian(self, q, cart_cmd, max_joint_speed, dt) -> MotionPlan:
        """Resolved-rate: tip velocity -> joint velocity (damped least squares)."""
        qdot, _ = self.kin.joint_velocity_from_cartesian(q, cart_cmd)
        peak = float(np.max(np.abs(qdot)))
        if peak > max_joint_speed:
            qdot *= max_joint_speed / peak              # keep direction, respect speed limit
        step = qdot * dt

        new_q = q + step
        blocked = bool(np.any(new_q < self.kin.lower) or np.any(new_q > self.kin.upper))
        if blocked:
            step = np.zeros(4)
        return MotionPlan(step, "CARTESIAN VELOCITY", blocked, (q + step).tolist())

    def _joint(self, q, joint_targets, max_joint_speed, dt) -> MotionPlan:
        """Move towards the joint targets at no more than max_joint_speed."""
        err = self.kin.clamp(joint_targets) - q
        biggest = float(np.max(np.abs(err)))
        if biggest < 1e-3:
            return MotionPlan(np.zeros(4), "IDLE", False, joint_targets)
        max_step = max_joint_speed * dt
        step = err if biggest <= max_step else err * (max_step / biggest)
        return MotionPlan(step, "JOINT MOVE", False, joint_targets)
