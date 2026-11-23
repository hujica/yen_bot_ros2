# Hardware Setup

## Bill of materials

- SO-ARM101 arm, 4 joints used by this software (base yaw, shoulder, elbow, wrist pitch)
- 4 × Feetech **STS3215** bus servos, IDs **1, 2, 3, 4** (J1 base → J4 wrist)
- Feetech USB servo-bus adapter (appears as `/dev/ttyACM0`)
- Servo power supply matching your servo voltage rating (see the SO-101 build guide)

## Servo IDs and communication

| Setting | Value |
|---|---|
| Servo IDs | 1 = base, 2 = shoulder, 3 = elbow, 4 = wrist pitch |
| Baud rate | 1,000,000 |
| Port | `/dev/ttyACM0` |
| Protocol | Feetech SCS/STS (via `scservo_sdk`) |

Control-table addresses used (`hardware/sts3215_driver.py`):

| Function | Address | Size |
|---|---|---|
| Torque enable | 40 | 1 byte |
| Goal position | 42 | 2 bytes |
| Goal speed | 46 | 2 bytes |
| Present position | 56 | 2 bytes |

On connect, the driver pings servo ID 1. If it answers, the arm is treated as live; otherwise the app
falls back to simulation.

## Angle ↔ servo count mapping

Defined in `hardware/servo_map.py`:

- **0° = 1024 steps** (the L-shaped home pose, all servos centred)
- 180° = 2047 steps, so 1° ≈ 11.37 steps
- Joint window ±60° → counts **342 … 1706**
- Per-servo direction `DIRECTIONS = {1: -1, 2: -1, 3: -1, 4: +1}`

### Home (L-shape) pose

At all-zero angles the upper arm is horizontal and the forearm hangs down (an "L"). Put the physical arm in
this pose, with every servo at count 1024, before first use. If your arm's zero differs, re-centre the servos
with the Feetech tools before running the software.

### A joint moves the wrong way

Flip the sign for **only that joint** in `ServoMap.DIRECTIONS` (1 ↔ −1) and restart. Do not change the
kinematics for this.

## Serial port access

### Native Linux
```bash
ls -l /dev/ttyACM*
sudo usermod -aG dialout $USER     # log out and back in afterwards
```

### Docker (provided `docker-compose.yml`)
The container maps `/dev/ttyACM0` and runs `privileged`. Important points:

1. **Plug the arm in before** `docker compose up`. A device added afterwards is not visible in the container.
2. If the port is missing inside the container, recreate it:
   ```bash
   docker compose down && docker compose up -d
   ```
   Your code is safe: `./src` is a bind-mounted volume. Packages installed with `pip` inside the old container
   are lost, so run `bash ~/src/setup.sh` again.
3. Docker Desktop on Windows / macOS cannot pass USB devices through directly. Use a Linux host, or WSL2 with
   `usbipd-win`.
4. If the port exists but is not readable: `sudo chmod a+rw /dev/ttyACM0`.

If the port name differs (for example `/dev/ttyUSB0`), edit `SERIAL_PORT` in `yen_bot/gui/gui_node.py` and the
`devices:` entry in `docker-compose.yml`.

## Safety behaviour in hardware mode

| Mechanism | Detail |
|---|---|
| Start-up speed | After connecting, every servo is limited to **30°/s** before anything moves |
| Joint window | Targets are clamped to ±60° in the controller, planner **and** the driver |
| No jump on torque-on | Goal position is set to the present position before torque is enabled |
| Speed register | Never written as 0 (0 means *unlimited* on the STS3215); minimum is 1 step |
| Comm watchdog | 5 consecutive failed write rounds → automatic E-stop (torque off) |
| E-stop | Button or `Esc`: torque disabled on all servos, targets = current pose |
| Shutdown | On exit the controller zeroes its commands and closes the serial port |

## First power-up checklist

1. Arm in the L-shaped home pose, area clear, power supply on.
2. Start the app; confirm `PHYSICAL HARDWARE DETECTED`.
3. With **low** Joint max (≈ 10-20 °/s), move each joint slider a few degrees and confirm each joint moves
   the right way (otherwise flip `DIRECTIONS`).
4. Jog ±X, ±Y, ±Z in small steps and compare with the pose read-out.
5. Test the E-stop (`Esc`) while the arm is moving slowly.
