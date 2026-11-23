#!/usr/bin/env python3
import threading

import rclpy
from rclpy.executors import SingleThreadedExecutor

from .ros2_bridge import ROS2BridgeNode


class RosRuntime:
    """Starts rclpy, the bridge node and a background spin thread; shuts them down again."""

    def __init__(self):
        rclpy.init()
        self.node = ROS2BridgeNode()
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        threading.Thread(target=self.executor.spin, daemon=True).start()

    def shutdown(self) -> None:
        steps = (self.executor.shutdown, self.node.destroy_node,
                 lambda: rclpy.ok() and rclpy.shutdown())
        for step in steps:                      # one failing step must not stop the others
            try:
                step()
            except Exception:
                pass
