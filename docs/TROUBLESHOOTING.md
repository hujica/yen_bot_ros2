# Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `ros2 run: error: the following arguments are required: executable_name` | Executable name missing | `ros2 run yen_bot gui_node` |
| `NameError: name 'RosRuntime' is not defined` | `gui_node.py` did not import it | `from yen_bot.ros.ros_runtime import RosRuntime` |
| `no such option: --break-system-packages` | Old pip (Ubuntu 22.04) | Omit the flag: `sudo python3 -m pip install feetech-servo-sdk pyserial` |
| `No module named 'scservo_sdk'` | Feetech SDK missing | `bash ~/src/setup.sh` or `sudo python3 -m pip install feetech-servo-sdk pyserial` |
| `Cannot open /dev/ttyACM0` / `SIMULATION (MOCK) MODE` | Port missing in container, no permission, or wrong baud rate | See *Serial port* below |
| `Port opened but servo ID 1 did not answer` | Servo power off, wrong baud rate, or ID 1 not set | Check power and cabling; servo IDs must be 1-4 at 1,000,000 baud |
| `python3: can't open file '/home/ubuntu/src/main.py'` | `run.sh` is the legacy launcher | Use `ros2 run yen_bot gui_node` (see below) |
| Code change has no effect | Built without `--symlink-install` | Rebuild, or build with `colcon build --symlink-install` |
| GUI panels cut off at the bottom | Screen too short | Scroll the left column, enlarge the window, or raise the VNC `RESOLUTION` |
| A joint moves the wrong way | Servo direction | Flip that joint in `ServoMap.DIRECTIONS` |
| Auto E-stop "servo communication lost" | 5 failed writes in a row | Check cable, power and baud rate, then RESET E-STOP |
| Status "Arm outside ±60° window" | Arm was outside the window at start-up | Move it inside by hand (torque OFF) or with HOME |
| Jog hold does nothing | Torque is OFF or E-stop active | Turn torque ON / reset E-stop |

## Serial port

```bash
ls -l /dev/ttyACM*                  # inside the same shell that runs the app
```

- **Not listed inside the container**: plug in the arm, then on the host `docker compose down && docker compose up -d`,
  and run `bash ~/src/setup.sh` again (pip packages do not survive container re-creation).
- **Listed but permission denied**: `sudo chmod a+rw /dev/ttyACM0` (container), or add your user to `dialout`
  and log in again (native Linux).
- **Windows / macOS Docker Desktop**: USB is not passed through. Use a Linux host, or WSL2 with `usbipd-win`.
- **Different port name**: edit `SERIAL_PORT` in `yen_bot/gui/gui_node.py` and `devices:` in `docker-compose.yml`.

## About `run.sh` and `setup.sh`

`src/run.sh` was written for the older layout and runs `python3 main.py` from `~/src`; that file now lives
inside the package. Launch the package instead:

```bash
cd ~/src
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run yen_bot gui_node
```

The port-permission step in `run.sh` (`sudo chmod a+rw /dev/ttyACM0`) is still useful; run it by hand if needed.
`setup.sh` is still valid for installing dependencies.

## Reading the console output

| Message | Meaning |
|---|---|
| `[STS3215Driver] PHYSICAL HARDWARE DETECTED …` | Real servos found |
| `[STS3215Driver Mock] Servo ID n torque ENABLED/DISABLED.` | Simulation only; no real servo was touched |
| `[WARN] <status>` | Status changed to a warning / error level |
| `[STS3215Driver] WARNING: servo n target … clamped …` | A target outside ±60° was clamped (printed once per servo) |
| `[ArmController] EMERGENCY STOP (reason)` | E-stop triggered |
| `[GUI] ROS context is not valid any more …` | rclpy was shut down; ROS publishing is disabled |

## Still stuck?

Collect the full terminal output from `ros2 run yen_bot gui_node` (including the traceback), the output of
`ls -l /dev/ttyACM*`, and whether you are in Docker or native, then open an issue with those.
