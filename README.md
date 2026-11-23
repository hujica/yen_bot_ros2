# SO-ARM101 Velocity Kinematics Console

A ROS 2 (Humble) Python package that drives a 4-DOF **SO-ARM101** robot arm built from
**Feetech STS3215** bus servos. It provides a desktop GUI with Cartesian jogging
(resolved-rate velocity control), joint sliders, a live 3D view, a joint-velocity plot,
and a torque switch / emergency stop. Joint states are published on `/joint_states`.

If no servo board is found, the app starts automatically in **simulation (mock) mode**,
so you can develop and test the GUI and kinematics without hardware.

```
Tk GUI (50 Hz loop) ─► ArmController ─► MotionPlanner ─► Kinematics (FK / IK / Jacobian)
                            │
                            ├─► SafetyMonitor (E-stop, comm watchdog, status text)
                            ├─► STS3215Driver ─► Feetech SDK ─► /dev/ttyACM0 ─► servos 1-4
                            └─► ROS2BridgeNode ─► /joint_states
```

## Features

- **Cartesian jog** (±X, ±Y, ±Z, hold a button to move) using damped-least-squares resolved-rate control
- **Joint targets**: one slider per joint, limited to ±60°
- **Live read-outs**: end-effector pose (X/Y/Z mm, pitch °), tip velocity, per-joint angle and velocity
- **3D kinematic view** (drag to orbit) with workspace rings and end-effector trail
- **10 s joint-velocity history** plot
- **Safety**: E-stop (button or `Esc`), torque ON/OFF, ±60° joint window, workspace alarm,
  servo-communication watchdog (auto E-stop), slow start-up speed
- **ROS 2**: publishes `sensor_msgs/JointState` on `/joint_states` (~25 Hz)
- **Simulation fallback** when the serial port or SDK is unavailable

## Requirements

| Item | Version / note |
|---|---|
| OS / ROS | Ubuntu 22.04, ROS 2 Humble (the provided Docker image is `tiryoh/ros2-desktop-vnc:humble`) |
| Python | 3.10 (ships with Humble) |
| System packages | `python3-tk`, `python3-pip`, `python3-numpy` |
| Python packages | `feetech-servo-sdk` (provides `scservo_sdk`), `pyserial` |
| Hardware (optional) | SO-ARM101 with STS3215 servos (IDs 1-4) on a USB serial board, 1,000,000 baud |

## Quick start

### 1. Docker (recommended)

```bash
# on the host, in the project folder (where docker-compose.yml is)
docker compose up -d          # connect the arm by USB BEFORE this, so the device is passed through
# open the desktop in a browser:  http://localhost:8888   (VNC password: soarm123)
```

Open a terminal in the desktop (or `docker exec -it so_arm101_env bash`), then:

```bash
cd ~/src
bash setup.sh                 # one time per container: installs tk, feetech-servo-sdk, pyserial
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 run yen_bot gui_node
```

> `--symlink-install` means Python edits take effect without rebuilding.
> Without it, rebuild after every change.

### 2. Native install

```bash
sudo apt install python3-tk python3-pip python3-numpy
sudo python3 -m pip install feetech-servo-sdk pyserial
sudo usermod -aG dialout $USER         # then log out and back in

mkdir -p ~/ws/src && cp -r yen_bot ~/ws/src/
cd ~/ws && colcon build --symlink-install
source install/setup.bash
ros2 run yen_bot gui_node
```

### What you should see

```
[STS3215Driver] PHYSICAL HARDWARE DETECTED on /dev/ttyACM0 @ 1000000 bps.   # real arm
[STS3215Driver] No physical servos ... SIMULATION (MOCK) MODE.              # no arm / no SDK
```

## Safety first

With real hardware the servos **move for real**. Before enabling torque:

1. Clear the area around the arm.
2. Keep the **EMERGENCY STOP** button (or the `Esc` key) within reach.
3. The app limits all joints to ±60° around the L-shaped home pose and starts the servos at 30°/s.
   Do not rely on software limits as your only protection.

## Repository layout

```
Project/
├── docker-compose.yml          ROS 2 Humble desktop container (VNC on :8888, passes /dev/ttyACM0)
├── docs/                       detailed documentation (see below)
└── src/
    ├── setup.sh                one-time dependency install inside the container
    ├── run.sh                  legacy launcher (see docs/TROUBLESHOOTING.md)
    ├── 4dof_so_arm/            legacy pre-package version of the code (not used by ros2 run)
    └── yen_bot/             the ROS 2 package
        ├── package.xml, setup.py, setup.cfg
        └── yen_bot/
            ├── gui/gui_node.py, hardware/hardware_node.py, kinematics/kinematics_node.py,
            │   ros/ros_node.py, tools/tools_node.py   (one ROS 2 node per folder)
            ├── controller/     ArmController, MotionPlanner, SafetyMonitor
            ├── kinematics/     geometry, FK/IK, Jacobian + resolved-rate
            ├── hardware/       STS3215 driver, angle <-> servo-count map
            ├── ros/            rclpy runtime + /joint_states bridge
            └── gui/            Tk widgets, 3D scene, velocity plot, theme
```

## Documentation

| Document | Contents |
|---|---|
| [docs/USER_GUIDE.md](docs/USER_GUIDE.md) | Using the GUI: controls, torque, E-stop, speeds, status messages |
| [docs/HARDWARE_SETUP.md](docs/HARDWARE_SETUP.md) | Wiring, serial port, Docker passthrough, servo IDs, calibration, direction flipping |
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | Modules, control loop, data flow, ROS interface, extension points |
| [docs/KINEMATICS.md](docs/KINEMATICS.md) | DH parameters, FK, IK, Jacobian, damped least squares, workspace limits |
| [docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md) | Common errors and fixes |

## Configuration cheat-sheet

| What | Where | Default |
|---|---|---|
| Serial port, baud rate | `yen_bot/gui/gui_node.py` (`SERIAL_PORT`, `BAUDRATE`) | `/dev/ttyACM0`, 1,000,000 |
| Joint limit | `controller/arm_controller.py` (`JOINT_LIMIT_DEG`), `hardware/servo_map.py` | ±60° |
| Link lengths / offsets | `kinematics/arm_geometry.py` | SO-101 URDF values |
| Servo direction per joint | `hardware/servo_map.py` (`DIRECTIONS`) | `{1:-1, 2:-1, 3:-1, 4:1}` |
| Start-up servo speed | `hardware/sts3215_driver.py` (`STARTUP_SPEED_DEG_S`) | 30°/s |
| Comm-failure auto E-stop | `controller/safety_monitor.py` (`COMM_FAIL_LIMIT`) | 5 failed writes in a row |
| Control loop rate | `gui/main_gui.py` (`TICK_MS`) | 20 ms (50 Hz) |

## Known limitations

- Package metadata in `package.xml` / `setup.py` (description, maintainer e-mail, **license**) is still a template placeholder.
- Only the lint tests from the ROS 2 package template exist; there are no unit tests for the kinematics yet.
- The node only **publishes** `/joint_states`; it does not subscribe to any command topic.
- Wrist pitch (joint 4) has zero length in the model: the tip is the wrist, not the gripper tip.

## License

Not specified yet. Add a license to `package.xml` and `setup.py` before sharing the code.

## Nodes

| Executable | Folder | Role |
|---|---|---|
| `gui_node` | `gui/` | Tk console + control loop (the original all-in-one app; publishes `/joint_states`) |
| `hardware_node` | `hardware/` | STS3215 bus: publishes `/hardware/joint_states`, listens to `/hardware/joint_command` |
| `kinematics_node` | `kinematics/` | `/joint_states` → `/end_effector/pose`, `/end_effector/twist` |
| `ros_node` | `ros/` | relays `/hardware/joint_states` → `/joint_states` |
| `tools_node` | `tools/` | one-shot read-only servo count reader |

`gui_node` is self-contained. Do not run it together with `hardware_node` (both open the serial port).
