#!/usr/bin/env python3
"""GUI node: Tk console + 50 Hz control loop (the original single-process app)."""
import tkinter as tk

from yen_bot.controller.arm_controller import ArmController
from yen_bot.gui.main_gui import SOArm101GUI
from yen_bot.ros.ros_runtime import RosRuntime

SERIAL_PORT = "/dev/ttyACM0"
BAUDRATE = 1000000


def main(args=None):
    ros = RosRuntime()
    controller = ArmController(port=SERIAL_PORT, baudrate=BAUDRATE)
    controller.initialize_arm()                  # falls back to SIMULATION if no servos

    root = tk.Tk()
    app = SOArm101GUI(root, controller, ros.node)

    try:
        root.mainloop()
    except KeyboardInterrupt:
        app.on_close()
    finally:
        for step in (controller.shutdown, ros.shutdown):    # zero commands + close port, stop ROS
            try:
                step()
            except Exception:
                pass


if __name__ == "__main__":
    main()
