#!/usr/bin/env python3
from .position_kinematics import PositionKinematics
from .velocity_kinematics import VelocityKinematics


class ArmKinematics(PositionKinematics, VelocityKinematics):
    """Everything about the arm's maths in one object:
         geometry / limits  -> arm_geometry.py
         FK, IK, drawing    -> position_kinematics.py
         Jacobian, DLS      -> velocity_kinematics.py
    """
