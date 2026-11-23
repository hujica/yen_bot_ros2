#!/usr/bin/env python3
"""Kinematics node: forward kinematics + Jacobian tip velocity from /joint_states.

Topics
  sub  /joint_states         sensor_msgs/JointState     joint_1..joint_4 (rad, rad/s)
  pub  /end_effector/pose    geometry_msgs/PoseStamped  tip position (m); tool pitch as rotation about Y
  pub  /end_effector/twist   geometry_msgs/TwistStamped tip linear velocity (m/s), if velocities are given
Parameters: frame_id (base_link), joint_limit_deg (60.0)
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .arm_kinematics import ArmKinematics

JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4"]


class KinematicsNode(Node):
    def __init__(self):
        super().__init__("yen_bot_kinematics")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("joint_limit_deg", 70.0)

        self.frame_id = self.get_parameter("frame_id").value
        self.kin = ArmKinematics(float(self.get_parameter("joint_limit_deg").value))
        self.pose_pub = self.create_publisher(PoseStamped, "/end_effector/pose", 10)
        self.twist_pub = self.create_publisher(TwistStamped, "/end_effector/twist", 10)
        self.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)

    @staticmethod
    def _ordered(names, values):
        """Values in joint_1..joint_4 order (by name if given, otherwise as received)."""
        if names:
            lookup = dict(zip(names, values))
            if not all(n in lookup for n in JOINT_NAMES):
                return None
            return [lookup[n] for n in JOINT_NAMES]
        return list(values) if len(values) >= 4 else None

    def _on_joint_states(self, msg: JointState) -> None:
        q_rad = self._ordered(msg.name, msg.position)
        if q_rad is None:
            return
        q_deg = np.degrees(q_rad)
        x, y, z, pitch_deg = self.kin.forward_kinematics(q_deg)

        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = self.frame_id
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = x, y, z
        half = math.radians(pitch_deg) / 2.0
        pose.pose.orientation.y = math.sin(half)
        pose.pose.orientation.w = math.cos(half)
        self.pose_pub.publish(pose)

        qd_rad = self._ordered(msg.name, msg.velocity) if msg.velocity else None
        if qd_rad is not None:
            v = self.kin.tip_velocity(q_deg, np.degrees(qd_rad))
            twist = TwistStamped()
            twist.header = pose.header
            twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z = (float(c) for c in v)
            self.twist_pub.publish(twist)

        for text in self.kin.workspace_violations(q_deg):
            self.get_logger().warn(f"workspace: {text}", throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = KinematicsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
