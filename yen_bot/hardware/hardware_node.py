#!/usr/bin/env python3
"""Hardware node: STS3215 serial bus + Services (/set_torque, /home_arm) + Action (/move_arm)."""
import math
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, Trigger

from .sts3215_driver import STS3215Driver

JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4"]


class HardwareNode(Node):
    def __init__(self):
        super().__init__("yen_bot_hardware")
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 1000000)
        self.declare_parameter("rate_hz", 25.0)
        self.declare_parameter("enable_torque", True)

        self.driver = STS3215Driver(self.get_parameter("port").value,
                                    int(self.get_parameter("baudrate").value))
        connected = self.driver.connect()
        self.get_logger().info("physical servos found" if connected else "SIMULATION (mock) mode")
        if self.get_parameter("enable_torque").value:
            for sid in self.driver.SERVO_IDS:
                self.driver.enable_torque(sid, True)

        self._last_deg = None
        self._last_time = None
        self.state_pub = self.create_publisher(JointState, "/hardware/joint_states", 10)
        self.create_subscription(JointState, "/hardware/joint_command", self._on_command, 10)
        self.create_timer(1.0 / float(self.get_parameter("rate_hz").value), self._publish_state)

        # Services
        self.create_service(SetBool, "/set_torque", self._srv_set_torque)
        self.create_service(Trigger, "/home_arm", self._srv_home_arm)

        # Action Server
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            "/move_arm",
            self._execute_move_arm,
        )

    def _srv_set_torque(self, request, response):
        for sid in self.driver.SERVO_IDS:
            self.driver.enable_torque(sid, request.data)
        response.success = True
        response.message = f"Torque {'ENABLED' if request.data else 'DISABLED'}"
        self.get_logger().info(response.message)
        return response

    def _srv_home_arm(self, request, response):
        for sid in self.driver.SERVO_IDS:
            self.driver.set_servo_angle(sid, 0.0)
        response.success = True
        response.message = "Arm commanded to HOME (0 deg)"
        self.get_logger().info(response.message)
        return response

    def _execute_move_arm(self, goal_handle):
        self.get_logger().info("Executing /move_arm action...")
        traj = goal_handle.request.trajectory
        feedback = FollowJointTrajectory.Feedback()
        feedback.joint_names = list(JOINT_NAMES)

        for pt in traj.points:
            if traj.joint_names:
                pairs = [(JOINT_NAMES.index(n) + 1, p)
                         for n, p in zip(traj.joint_names, pt.positions) if n in JOINT_NAMES]
            else:
                pairs = list(zip(self.driver.SERVO_IDS, pt.positions))
            for sid, rad in pairs:
                self.driver.set_servo_angle(sid, math.degrees(rad))

            time.sleep(0.5)
            actual_rad = [math.radians(self.driver.get_servo_angle(sid)) for sid in self.driver.SERVO_IDS]
            feedback.actual.positions = actual_rad
            feedback.desired.positions = list(pt.positions)
            goal_handle.publish_feedback(feedback)

        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = 0
        self.get_logger().info("/move_arm action completed.")
        return result

    def _on_command(self, msg: JointState) -> None:
        if msg.name:
            pairs = [(JOINT_NAMES.index(n) + 1, p) for n, p in zip(msg.name, msg.position)
                     if n in JOINT_NAMES]
        else:
            pairs = list(zip(self.driver.SERVO_IDS, msg.position))
        for sid, rad in pairs:
            self.driver.set_servo_angle(sid, math.degrees(rad))

    def _publish_state(self) -> None:
        now = self.get_clock().now()
        deg = [self.driver.get_servo_angle(sid) for sid in self.driver.SERVO_IDS]
        vel = [0.0] * 4
        if self._last_deg is not None:
            dt = (now - self._last_time).nanoseconds * 1e-9
            if dt > 0.0:
                vel = [(a - b) / dt for a, b in zip(deg, self._last_deg)]
        self._last_deg, self._last_time = deg, now

        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.name = list(JOINT_NAMES)
        msg.position = [math.radians(a) for a in deg]
        msg.velocity = [math.radians(v) for v in vel]
        self.state_pub.publish(msg)

    def destroy_node(self):
        self.driver.disconnect()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HardwareNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
