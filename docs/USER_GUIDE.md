# User Guide

## Window layout

| Area | Purpose |
|---|---|
| **Header** | Title, `HARDWARE LIVE` / `SIMULATION` tag, current mode, status chip. A flashing red banner appears on a workspace alarm. |
| **Torque / E-stop bar** (top) | `TORQUE ON/OFF` on the left, `EMERGENCY STOP` / `RESET E-STOP` on the right. |
| **Left column** (scrolls) | End-effector pose, Cartesian jog pad, velocity inputs, joint targets. |
| **Right column** | 3D kinematic visualizer and the 10 s joint-velocity history. |

If the screen is short, the left column scrolls (mouse wheel or scroll bar).

## Starting up

1. Launch with `ros2 run yen_bot gui_node`.
2. The app tries to open the serial port and ping servo ID 1.
   - Success: header shows **HARDWARE LIVE**, servos start with a 30°/s speed limit and torque on.
   - Failure: header shows **SIMULATION** and everything runs against a software mock.
3. On a real arm, the app reads the current joint angles first and uses them as the targets,
   so the arm does **not** jump at start-up.

## Controls

### Torque button
- **ON** (green): servos are powered and hold position. When re-enabling, the goal is first set to the
  current position, so the arm never jumps to an old target.
- **OFF**: servos go limp so you can move the arm by hand. All jog / slider / HOME commands are ignored and
  locked in the GUI. The display keeps following the real pose.

### Emergency stop
- Click **EMERGENCY STOP** or press **`Esc`**.
- Effect: commands cleared, targets set to the current pose, torque disabled on all servos.
- To resume, click **RESET E-STOP**. The app first reads the real pose, then re-enables torque.
- The Torque button is disabled while E-stopped.
- The app also triggers an E-stop itself if 5 servo writes in a row fail (communication lost).

### Cartesian jog (hold to move)
Hold a button to move the tip at the **Linear (tip)** speed; release to stop.

| Button | Intended direction |
|---|---|
| `▲ +X` / `▼ −X` | forward / back |
| `◀ −Y` / `+Y ▶` | left / right (see note below) |
| `⬆ +Z` / `⬇ −Z` | up / down |
| `HOME` | all joints back to 0° (L-shape pose) |

Motion is resolved-rate control (tip velocity → joint velocity). The solver also holds the tool pitch,
and is damped near singularities, so the arm slows rather than flailing near full stretch.
If any joint would exceed ±60° the motion is blocked and the status shows *Joint limit – motion blocked*.

> **Note on Y:** the jog buttons send `−Y` for the left button and `+Y` for the right one in code
> (`gui/control_panels.py`, `JogPanel.PAD`), so the pose read-out's Y value decreases when you press `+Y ▶`.
> With X forward and Z up, a right-handed frame has +Y to the left. Check this on your arm and swap the two
> vectors if you want the labels and the sign of Y to agree.

### Velocity input
| Field | Range | Default | Meaning |
|---|---|---|---|
| Linear (tip) | 5-150 mm/s | 40 | Tip speed while a jog button is held |
| Joint max | 5-120 °/s | 30 | Joint speed ceiling for jogging and slider moves |

The servo speed register is set to 1.5× the joint max so the servo does not fall behind the software ramp.

### Joint targets
One slider per joint (−60° … +60°). Dragging sets the target; the arm moves toward it at no more than
**Joint max**. Next to each slider are the actual angle (turns red at the limit) and the joint velocity.

## Read-outs

- **End-effector pose**: X, Y, Z in mm (from the base, `X` forward) and tool pitch in degrees.
- **Tip velocity**: vx, vy, vz and |v| in mm/s.
- **3D view**: drag with the left mouse button to orbit. The arm turns red on a workspace alarm.
  Dashed rings are the min / max reach limits; the violet line is the tip trail; the green arrow is tip velocity.
- **Velocity history**: joint velocities J1-J4 over the last 10 s (scale follows the Joint max setting).

## Status messages

| Message | Level | Meaning |
|---|---|---|
| Ready / Moving | ok | Normal |
| SIMULATION – arm not connected | warn | No hardware; running the mock |
| TORQUE OFF – position held | warn | Servos limp, commands ignored |
| Joint limit (±60°) – motion blocked | warn | Jog would leave the joint window |
| Arm outside ±60° window | warn | Arm started or was moved outside the window |
| WORKSPACE ALARM – … | error | Reach too long / too close to base / below table (also flashes the banner and rings the bell) |
| EMERGENCY STOP (reason) | error | E-stop active; reset with the button |
| GUI ERROR – see console | error | An exception in the GUI loop; read the terminal output |

## ROS 2 interface

```bash
ros2 topic echo /joint_states
```

| Item | Value |
|---|---|
| Node | `so_arm101_gui_bridge` |
| Topic | `/joint_states` (`sensor_msgs/msg/JointState`), ~25 Hz |
| Names | `joint_1` … `joint_4` |
| Units | position in **radians**, velocity in **rad/s** (the GUI itself works in degrees) |

The node is publish-only; there is no command topic yet.
