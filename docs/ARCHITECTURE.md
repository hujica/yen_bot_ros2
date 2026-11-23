# Architecture

## Layers

```
┌────────────────────────────  gui/  ───────────────────────────┐
│ main_gui.SOArm101GUI   50 Hz tick, owns all widgets            │
│ control_panels · header · scene_3d · velocity_plot · theme     │
└───────────────┬───────────────────────────────┬───────────────┘
                │ commands / read state         │ publish
┌───────────────▼────────────────┐    ┌─────────▼────────────────┐
│ controller/ArmController       │    │ ros/ROS2BridgeNode       │
│  ├ MotionPlanner (pure maths)  │    │  /joint_states           │
│  ├ SafetyMonitor (E-stop/status)│   │ ros/RosRuntime           │
│  └ kinematics/ArmKinematics    │    │  rclpy + spin thread     │
└───────────────┬────────────────┘    └──────────────────────────┘
                │ angles (deg)
┌───────────────▼────────────────┐
│ hardware/STS3215Driver         │──► scservo_sdk ──► /dev/ttyACM0
│  └ ServoMap (deg ↔ counts)     │    (mock mode if no hardware)
└────────────────────────────────┘
```

Dependency direction is strictly downward: the GUI knows the controller, the controller knows the planner,
safety monitor, kinematics and driver, and none of those import the GUI.

## Start-up sequence (`gui/gui_node.py`)

1. `RosRuntime()`: `rclpy.init()`, create `ROS2BridgeNode`, start a `SingleThreadedExecutor` in a daemon thread.
2. `ArmController(port, baudrate)`, then `initialize_arm()`: open the port, ping servo 1, read the real
   angles, enable torque, apply the servo speed limit. On failure it runs in simulation.
3. `tk.Tk()` and `SOArm101GUI(root, controller, ros.node)`.
4. `root.mainloop()`. On exit, `controller.shutdown()` (zero commands, close port) then `ros.shutdown()`.

## Control loop (`SOArm101GUI.tick`)

Runs every `TICK_MS = 20` ms (50 Hz) via `root.after`:

```
_control_step():
    read speed panel  ─► controller.set_speed_limits(...)
    jog button held?  ─► controller.set_cartesian_velocity(dir × max_linear_speed)
    controller.update(dt)
        ├ E-stop?       → hold, status only
        ├ torque off?   → read real pose, no commands
        └ planner.plan(...) → new joint angles → driver.set_servo_angle(1..4)
    record velocity history, record 3D trail
    every 2nd tick → publish /joint_states   (~25 Hz)
every 3rd tick → _refresh_views(): pose, joints, header, 3D scene, plot (~17 Hz)
```

`dt` is measured with `perf_counter` and clamped to 1 ms … 100 ms. Exceptions inside a tick are printed once
(deduplicated), the header shows *GUI ERROR*, and the loop keeps running.

## Modules

### `controller/arm_controller.py` – `ArmController`
Owns the state the GUI reads (`current_joint_angles`, `joint_targets`, `current_joint_velocities`,
`current_end_effector_pose`, `current_tip_velocity`, `mode`, `status`, `workspace_alarm`, `torque_on`).
Commands: `set_joint_target`, `set_cartesian_velocity`, `move_cartesian`, `home_arm`, `set_speed_limits`,
`set_torque_enabled`, `emergency_stop`, `reset_estop`.

### `controller/motion_planner.py` – `MotionPlanner`
Stateless. `plan(q, cart_cmd, joint_targets, max_joint_speed, dt)` returns
`MotionPlan(step, mode, blocked, targets)`:
- non-zero Cartesian command → **CARTESIAN VELOCITY** (resolved-rate, speed-scaled, blocked at limits)
- otherwise → **JOINT MOVE** towards targets at ≤ `max_joint_speed` (or **IDLE** when within 0.001°)

### `controller/safety_monitor.py` – `SafetyMonitor`
E-stop flag and reason, write-failure counter (`COMM_FAIL_LIMIT = 5`), and the prioritised status text:
E-stop → workspace alarm → joint limit → torque off → simulation → outside window → Ready / Moving.

### `kinematics/`
See [KINEMATICS.md](KINEMATICS.md).

### `hardware/`
- `STS3215Driver`: `connect`, `disconnect`, `set_servo_angle`, `set_servo_speed`, `get_servo_angle`,
  `enable_torque`. Supports both flavours of the Feetech Python SDK (`PacketHandler` style and
  `sms_sts` style). If not connected, all calls succeed against `simulated_angles` and torque changes are printed
  as `[STS3215Driver Mock]`.
- `ServoMap`: angle ↔ step conversion, per-servo direction, ±60° clamping.

### `ros/`
- `RosRuntime`: lifecycle of rclpy, node, executor and spin thread. `shutdown()` runs each step in its own
  try/except so one failure cannot block the others.
- `ROS2BridgeNode` (`so_arm101_gui_bridge`): `publish_joint_states(angles_deg, velocities_deg_s)` converts to
  radians and publishes `sensor_msgs/JointState` on `/joint_states` with names `joint_1…joint_4`.

### `gui/`
| File | Contents |
|---|---|
| `main_gui.py` | `SOArm101GUI`: window setup, layout, callbacks, the control loop |
| `control_panels.py` | `PosePanel`, `JogPanel`, `SpeedPanel`, `JointPanel`, `TorqueButton`, `EStopButton` |
| `header.py` | `StatusHeader`: title, hardware tag, status chip, flashing alarm banner |
| `scene_3d.py` | `Scene3D`: orthographic projection on a Tk canvas, mouse orbit, floor grid, workspace rings, trail |
| `velocity_plot.py` | `VelocityPlot`: 10 s scrolling plot, skips redraws while all velocities are zero |
| `scroll_column.py` | `ScrollColumn`: scrollable left column (mouse wheel on Windows/macOS/Linux) |
| `theme.py` | Colours, fonts, ttk dark style, small widget helpers |

The window is sized to 96 % × 90 % of the screen (a fixed size is more stable than "zoomed" in a VNC desktop).
The Torque/E-stop bar is packed directly under the header so it is always visible.

## Units cheat-sheet

| Place | Units |
|---|---|
| GUI joint angles / velocities | degrees, deg/s |
| GUI pose | mm and degrees |
| Controller joint state | degrees, deg/s |
| Controller pose / tip velocity | metres, m/s |
| Kinematics internals | radians |
| `/joint_states` | radians, rad/s |

## Extending the project

- **Add a command topic**: add a subscriber in `ROS2BridgeNode`, and call `controller.set_joint_target` or
  `set_cartesian_velocity` from it. Remember the callback runs on the spin thread, not the Tk thread.
- **Add a joint (e.g. gripper)**: extend `SERVO_IDS`, `ServoMap.DIRECTIONS`, the `[0.0]*4` state arrays, the
  joint panel and the `/joint_states` names.
- **Different arm geometry**: see "Changing the arm" in [KINEMATICS.md](KINEMATICS.md).
- **Unit tests**: `MotionPlanner` and the kinematics classes have no hardware or GUI dependencies, so they are
  straightforward to test with `pytest` (for example `forward_kinematics(inverse_kinematics(p)) ≈ p`).
