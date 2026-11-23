# Kinematics

All maths lives in `yen_bot/kinematics/`. Lengths are in metres, servo angles in degrees,
internal model angles in radians.

| File | Class | Role |
|---|---|---|
| `arm_geometry.py` | `ArmGeometry` | Link lengths, joint offsets, joint limits, workspace bounds |
| `position_kinematics.py` | `PositionKinematics` | Forward / inverse kinematics, drawing points, workspace check |
| `velocity_kinematics.py` | `VelocityKinematics` | Jacobian, tip velocity, resolved-rate control |
| `arm_kinematics.py` | `ArmKinematics` | Combines the two above into the one object the controller uses |

## Model

Servo angle `q = 0` on every joint is the physical L-shape home pose. The maths uses model angles
`θ = q + OFFSET` with `OFFSET = [0°, 90°, −90°, 0°]`:

| Joint | Model angle | Meaning |
|---|---|---|
| 1 | θ1 | Base yaw (0° points along +X) |
| 2 | θ2 | Shoulder pitch (0 = upper arm horizontal) |
| 3 | θ3 | Elbow pitch (= q3 − 90°: forearm hangs down at home) |
| 4 | θ4 | Wrist pitch (orientation only, wrist length 0) |

DH table:

| i | a (m) | α | d (m) |
|---|---|---|---|
| 1 | 0.0388 | +90° | 0.030 |
| 2 | 0.1159 | 0 | 0 |
| 3 | 0.1350 | 0 | 0 |
| 4 | 0 | 0 | 0 |

`A1 = 0.0388` (pan offset), `A2 = 0.1159` (upper arm), `A3 = 0.1350` (forearm), `D1 = 0.030` (base height),
following the SO-101 URDF. The tip is the **wrist**; use `A3 = 0.164` if you want the gripper tip instead.

## Forward kinematics

```
r = A1 + A2·cos θ2 + A3·cos(θ2 + θ3)
z = D1 + A2·sin θ2 + A3·sin(θ2 + θ3)
x = r·cos θ1,   y = r·sin θ1
pitch = θ2 + θ3 + θ4
```

`forward_kinematics(q)` returns `[x, y, z, pitch_deg]`. `joint_positions(q)` returns the 5 points
(base, column top, shoulder, elbow, tip) used by the 3D view.

## Inverse kinematics (elbow-down, closed form)

Given `[x, y, z, pitch]`:

1. `θ1 = atan2(y, x)`
2. Wrist point relative to the shoulder: `rw = hypot(x, y) − A1`, `zw = z − D1`
3. `cos θ3 = (rw² + zw² − A2² − A3²) / (2·A2·A3)`. If |cos θ3| > 1 the pose is unreachable → `None`.
4. `θ3 = atan2(−√(1 − cos²θ3), cos θ3)` (elbow-down, θ3 < 0)
5. `θ2 = atan2(zw, rw) − atan2(A3·sin θ3, A2 + A3·cos θ3)`
6. `θ4 = pitch − θ2 − θ3`
7. Convert back to servo angles (`− OFFSET`). If any joint is outside ±60° the result is `None`.

`ArmController.move_cartesian()` uses this for point-to-point moves; the GUI jog pad uses the velocity method below.

## Jacobian and resolved-rate control

`jacobian(q)` is the 3×4 matrix mapping joint rates (rad/s) to tip velocity (m/s). The wrist column is zero
because the wrist has no length.

`joint_velocity_from_cartesian(q, v_cart, pitch_rate=0, damping=0.003)`:

1. Stack a tool-pitch row `[0, 1, 1, 1]` under the Jacobian, giving a 4×4 system with
   `ẋ = [vx, vy, vz, pitch_rate]`. With `pitch_rate = 0` the tool pitch is held constant.
2. Manipulability `w = √det(J·Jᵀ)`.
3. Damped least squares: `q̇ = Jᵀ (J·Jᵀ + λ²I)⁻¹ ẋ`, with `λ² = damping²`, plus an extra term
   `0.01·(1 − w/w0)²` when `w < w0 = 3·10⁻⁴`, so damping increases near singularities.
4. Returns `q̇` in deg/s and `w`.

`MotionPlanner` then scales `q̇` so the fastest joint does not exceed **Joint max**, integrates one step
(`q̇·dt`), and blocks the whole step if any joint would leave ±60°.

## Workspace limits

`workspace_violations(q)` returns a list of messages (empty = OK):

| Check | Limit | Message |
|---|---|---|
| Max reach | `WS_R_MAX = 0.98·(A1 + A2 + A3)` ≈ 0.284 m | `reach … mm > … mm` |
| Min reach | `WS_R_MIN = 0.02 m` (signed reach from the base axis) | `too close to base` |
| Table | shoulder, elbow and tip z ≥ 0 | `below table` |

A violation turns the status red and shows the flashing banner. It is an **alarm**, not a hard stop.
Joint limits (±60°) are what actually block motion.

## Changing the arm

| Change | Edit |
|---|---|
| Different link lengths | `A1, D1, A2, A3` in `arm_geometry.py` |
| Gripper tip as end point | set `A3 = 0.164` |
| Different joint window | `JOINT_LIMIT_DEG` in `arm_controller.py` and `servo_map.py` (keep them equal) |
| Different zero pose | `OFFSET_DEG` in `arm_geometry.py` and `CENTER_COUNT` in `servo_map.py` |
