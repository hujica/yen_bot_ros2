#!/usr/bin/env python3
"""Tools node: one-shot, READ-ONLY servo count reader (never writes to a servo).

Parameters: port (/dev/ttyACM0), baudrate (1000000)
Close the GUI / hardware node first - only one program may use the serial port.
"""
import rclpy
from rclpy.node import Node

from .read_servo_counts import read_counts


class ToolsNode(Node):
    def __init__(self):
        super().__init__("yen_bot_tools")
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 1000000)

    def run(self) -> None:
        try:
            read_counts(self.get_parameter("port").value,
                        int(self.get_parameter("baudrate").value),
                        emit=lambda line: self.get_logger().info(line))
        except Exception as e:
            self.get_logger().error(str(e))


def main(args=None):
    rclpy.init(args=args)
    node = ToolsNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
