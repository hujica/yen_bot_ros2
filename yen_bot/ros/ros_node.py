#!/usr/bin/env python3
"""ROS node: relays the hardware node's joint state onto the public /joint_states topic.

Topics
  sub  /hardware/joint_states  sensor_msgs/JointState
  pub  /joint_states           sensor_msgs/JointState
"""
import rclpy
from sensor_msgs.msg import JointState

from .ros2_bridge import ROS2BridgeNode


class RosNode(ROS2BridgeNode):
    def __init__(self):
        super().__init__("yen_bot_ros")
        self.create_subscription(JointState, "/hardware/joint_states", self.joint_pub.publish, 10)


def main(args=None):
    rclpy.init(args=args)
    node = RosNode()
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
