# se3_control — native onboard SE(3) control

**[Flight examples and commands](EXAMPLES.md)**

Geometric SE(3) position/yaw control for an **already airborne** multicopter,
registered with Commander as one external flight mode named `SE3`. The module
computes the reference (hold or a firmware-defined trajectory) and the
geometric attitude reference onboard at 100 Hz, then commands PX4 through one
of three control paths selected by `SE3_CTRL` at boot. It never arms, takes
off or lands. PX4 keeps actuator output, failsafes and flight-mode ownership.

This README describes the implementation. Operating procedures for the lab
toolkit (SITL stack, recording, analysis) are documented in that toolkit.

## Architecture

```
            vehicle_local_position, vehicle_attitude, vehicle_angular_velocity
                                        │
   trajectory/ ──► Run() @ 100 Hz: reference ─► Controller::compute_geometry
   (hold, setpoint,       (Runtime.cpp)          A/m, R_d, Ω_d, Ω̇_d, collective
    steps5, circle,                                       │
    figure8)                      ┌───────────────────────┼────────────────────────┐
                                  ▼                       ▼                        ▼
                    SE3_CTRL=0 ATTITUDE_PX4    SE3_CTRL=1 RATE_PX4      SE3_CTRL=2 PHYSICAL
                    vehicle_attitude_setpoint  vehicle_rates_setpoint   reference → inner loop
                             │                          │               (gyro-triggered)
                       mc_att_control                   │               se3_wrench_setpoint
                             │                          │                         │
                       mc_rate_control ◄────────────────┘                         │
                             │                                                    │
                       control_allocator (native) ◄───────────────────► control_allocator
                             │                                         (physical allocation)
                             └──────────────► actuator_motors ◄───────────────────┘
```

All three paths share the same trajectories, geometric reference (including
the position integral), hold, Commander lifecycle and fault handling. Only
what is commanded differs.

## Control paths

`SE3_CTRL` is read once in `init()` and is fixed until reboot
(`reboot_required`). The mapping and the PX4 controllers each path enables
are defined in `ControlPath.hpp`.

| `SE3_CTRL` | Path | Mode enables (`config_control_setpoints`) | SE3 publishes as command |
|---|---|---|---|
| 0 | ATTITUDE_PX4 | attitude, rates, allocation | `vehicle_attitude_setpoint` |
| **1** (default) | RATE_PX4 | rates, allocation | `vehicle_rates_setpoint` |
| 2 | PHYSICAL | allocation | `se3_wrench_setpoint` |

Position, velocity and altitude control are never enabled, so Commander keeps
multicopter position control off while SE3 owns the reference. An unknown
`SE3_CTRL` value stops the module from starting.

SE3 publishes setpoints only while its mode is the current navigation state
**and** Commander's active controllers are the ones its path requires
(`controllers_match()`).

**Numbering change.** An earlier experimental build used `0 = RATE_PX4` and
`1 = PHYSICAL`. PX4 never stores a parameter at its default, so the only value
that build could have persisted is `1`, which now selects RATE_PX4. The active
path is printed at boot, sent to the GCS (`SE3 registered: <path> control`) and
shown by `se3_control status`.

### Common reference (all paths, 100 Hz)

`Controller.cpp`, mass-normalized (gains are `kx/m`, `kv/m`, `ki/m`):

```
e_x   = p - p_ref,  e_v = v - v_ref                  local NED
I    += sat(e_x, 0.5 m) dt                           position integral [m s]
a     = a_ref - Kx e_x - Kv e_v - Ki I               acceleration command
A/m   = a - g e3                                     specific force
u     = -(A/m) · (R e3)                              collective (rejected if <= 0)
```

**Position integral** (`Controller::update_integral()`, `Integral.hpp`).
`Ki = diag(SE3_KIXY, SE3_KIXY, SE3_KIZ)`. It is added to the acceleration
command, so it shapes the desired force, `R_d` and the thrust of every path.
It is updated once per 100 Hz cycle, before the translation law.

- **Bounded:** `|Ki I| <= 1 m/s²` per horizontal axis and `2 m/s²`
  vertically. The integrated error is saturated at 0.5 m per axis, so a large
  step contributes little while it is transient.
- **Anti-windup:** nothing is integrated while the previous output saturated:
  - RATE_PX4 and ATTITUDE_PX4: normalized thrust outside [0, 1], a clamped
    body rate (RATE_PX4), or `control_allocator_status` reporting unallocated
    thrust or roll/pitch torque.
  - PHYSICAL: `se3_allocation_status` with `thrust_limited` or
    `roll_pitch_scale < 1`.

  At its bound the integral holds, and an error back inside acts at once.
  `se3_trajectory_status.integral_frozen` logs the freeze.
- **Reset** on mode entry and exit, on every fault and therefore on estimator
  resets (a controller fault), and per axis when its gain is 0. Trajectory
  and hold commands keep it: it represents a model offset, not a reference.
- **Derivatives:** the force derivatives use the rates of the latest update,
  so `Ω_d` and `Ω̇_d` stay consistent with the integral actually applied.
  `dI/dt` is the rate actually applied: `sat(e_x)` inside the bound, only the
  part up to the bound on the step that reaches it, and zero when frozen or
  held at the bound. `d²I/dt² = e_v` only while `I` is strictly inside its
  bound and `|e_x|` inside the 0.5 m saturation.

`AttitudeReference.cpp` builds `R_d` from `b3_d = -A/‖A‖` and the reference
heading (Lee et al., 2010). It also computes `Ω_d` and `Ω̇_d` analytically from
the force derivatives (trajectory jerk and snap), with no finite differencing.
Attitude errors (`AttitudeLaws.cpp`):

```
e_R = ½ (R_dᵀ R − Rᵀ R_d)^∨          e_Ω = Ω − Rᵀ R_d Ω_d
```

The normalized thrust for the PX4 paths is `MPC_THR_HOVER · u / g`, clamped
to [0, 1].

### RATE_PX4 (`SE3_CTRL=1`, default)

The original SE3 controller:

```
Ω_sp = Rᵀ R_d Ω_d − K_R e_R          K_R = SE3_KRX/KRY/KRZ, clamped to ±SE3_RATE_MAX
```

SE3 publishes `vehicle_rates_setpoint` at 100 Hz. `mc_rate_control`
(native `MC_*RATE*` gains) runs at the gyro rate and native allocation follows.
`vehicle_attitude_setpoint` is also published, as an observation only;
Commander keeps `mc_att_control` disabled.

### ATTITUDE_PX4 (`SE3_CTRL=0`)

SE3 publishes `R_d` (as `q_d`), the normalized thrust and the reference yaw
rate (`yaw_sp_move_rate`) on `vehicle_attitude_setpoint` at 100 Hz.
`mc_att_control` runs on every `vehicle_attitude` update and publishes the
rate setpoint; `mc_rate_control` and native allocation follow. Native attitude
and rate gains are used unchanged. SE3 publishes **no** rate setpoint, and the
SE3 body-rate gains and limit are not used or checked.

### PHYSICAL (`SE3_CTRL=2`)

Run() stores `A/m` and `(R_d, Ω_d, Ω̇_d)` at 100 Hz. A gyro-triggered work
item (`run_inner()`, one wrench per new `vehicle_angular_velocity` sample, on
the same work queue as Run()) computes the Lee wrench in SI units:

```
f = m · ( −(A/m) · (R e3) )                                    [N]
M = −K_R e_R − K_Ω e_Ω − K_I e_I + Ω × JΩ + J α_ff             [N m]
α_ff = −Ω × (Rᵀ R_d Ω_d) + Rᵀ R_d Ω̇_d,   e_I = ∫ e_R dt
K_R = SE3_PKR{X,Y,Z} [N m/rad],  K_Ω = SE3_PKW{X,Y,Z} [N m s/rad]
K_I = diag(SE3_PKI_XY, SE3_PKI_XY, SE3_PKI_Z) [N m/(rad s)]
m = SE3_MASS,  J = diag(SE3_JXX, SE3_JYY, SE3_JZZ)
```

The geometric attitude integral `e_I` is integrated in the inner loop, over
each gyro-sample interval, after the moment for that sample is computed. It is
bounded to `|K_I e_I| <= J · 8 rad/s²` (roll/pitch) and `J_zz · 2 rad/s²`
(yaw). The allocator's reply to the previous wrench freezes the roll/pitch
axes while `roll_pitch_scale < 1`, and yaw while `yaw_scale < 1`. It is reset
on mode entry and exit, on faults and whenever the neutral wrench is used.
`se3_wrench_setpoint.moment_integral` logs `−K_I e_I`.

The inner loop publishes `se3_wrench_setpoint` only while the vehicle is armed
in the SE3 mode with allocation enabled and the PX4 rate and attitude loops
disabled (`owns_allocation()`, re-checked at the gyro rate). Before the first
reference, and after a fault until Commander acts, it publishes a **neutral**
wrench: `R_d = R`, `Ω_d = 0`, which gives rate damping plus `Ω × JΩ`. Rotation
is stopped, not levelled. Thrust is weight-compensating in the air and zero on
the ground.

It also publishes the hover-scaled normalized collective on
`vehicle_thrust_setpoint` for observers (land detector, logs). The allocator
ignores it in this path. The physical path does not depend on the body-rate
law, its gains or `SE3_RATE_MAX`.

## Physical model and allocation (`physical/`)

### Vehicle and rotor model

The PHYSICAL path uses no separate vehicle model. It uses:

- **Mass and inertia:** `SE3_MASS` and `SE3_JXX/JYY/JZZ`, about the centre of
  mass. The defaults are the Gazebo F450: 2.0736 kg and
  diag(0.022078, 0.022078, 0.043301) kg m².
- **Rotor model:** PX4's own rotor effectiveness, which `control_allocator`
  builds from `CA_ROTOR*_PX/PY/PZ`, `CA_ROTOR*_AX/AY/AZ`, `CA_ROTOR*_CT` and
  `CA_ROTOR*_KM` (`ActuatorEffectivenessRotors`). Column `i` is the wrench of
  rotor `i` per unit motor command:

  ```
  thrust  CT_i n_i                       [N]
  moment  CT_i (r_i × n_i − KM_i n_i)    [N m]
  ```

  With `CT` given in newtons (thrust at full command), this matrix is the SI
  rotor model: rotor thrust is `CT_i u_i`, where `u_i` is the thrust-linear
  `actuator_motors` command. Native allocation normalizes the same matrix, so
  `CT`'s scale does not change native control.
- **Thrust curve:** applied once, by the output driver (`FunctionMotors`,
  `THR_MDL_FAC`), exactly as for native allocation. The physical allocator
  never applies a curve of its own.

The F450 airframe `4022_gz_f450` sets the Gazebo model's geometry. Rotors are
in `actuator_motors` order (FRD), with hubs at ±0.16263456 m, `PZ = −0.0112 m`
and thrust axes (0, 0, −1):

| Rotor | Position | `KM` |
|---|---|---|
| 0 | front right | +0.0137 (CCW) |
| 1 | back left | +0.0137 (CCW) |
| 2 | front left | −0.0137 (CW) |
| 3 | back right | −0.0137 (CW) |

`CT = 12` N is the Gazebo thrust at full command,
`k_f (150 + 850)² = 1.2e-5 · 1000²`. `THR_MDL_FAC` keeps its default of 0.
Other vehicles configure their own `CA_ROTOR*`, `THR_MDL_FAC` and `SE3_*`
values; generic airframe defaults are unchanged.

**Thrust-model accuracy.** `T = CT u` is exact only if `THR_MDL_FAC`
linearizes the real motor curve. The Gazebo F450 curve is
`T(u) = 0.27 + 3.06 u + 8.67 u²` N. With `THR_MDL_FAC = 0` the linear model
over-predicts thrust below full command (5.08 N per rotor at hover is
commanded as `u = 0.42`, which produces 3.1 N). PHYSICAL hover then depends
on the vertical position integral (bounded at 2 m/s²) and a steady
position error: in SITL the integral sits at its bound, the request is
28.2 N for a 20.3 N weight, and the vehicle holds about 0.45 m low. The
Gazebo curve normalized to `CT` is `(0.15 + 0.85 x)²`, which
`THR_MDL_FAC = 0.72` matches to within 0.023 of full thrust; with it (and the
thrust-linear `MPC_THR_HOVER = 0.42`) the request is 19.9 N and the vertical
integral about 0.2 m/s². These are runtime test values only; see Validation.

### Compatibility requirements

PHYSICAL refuses to run (SE3 reports itself unhealthy and cannot be selected)
unless the `SE3_PK*` attitude gains are positive, `SE3_PKI_*` are
non-negative, and `SE3_MASS` and `SE3_J*` are positive and finite.

In `control_allocator` (`se3_motors_compatible()`) the following must also
hold; otherwise the wrench is reported as `compatible = false`:
- exactly one effectiveness matrix;
- exactly 4 motors and no servos;
- non-reversible motors with the [0, 1] range.

`Allocation::configure()` additionally rejects (also `compatible = false`):
- lateral or downward rotor thrust;
- a singular or ill-conditioned geometry, including a failed motor's zeroed
  column.

The effectiveness is captured in `update_effectiveness_matrix_if_needed()`
after failed motors are zeroed but **before** the native weak-authority filter
(rows with all entries ≤ 0.05 set to zero), so small but physically valid SI
coefficients, such as `CT·KM < 0.05 N m` on a small vehicle, are kept. Native
allocation still uses the filtered matrix.

### Motor-command allocation (`PhysicalAllocation.{hpp,cpp}`)

The wrench `W = [f, Mx, My, Mz]` is linear in the motor commands:

```
W = B u,   B_0i = −E_(z thrust),i,   B_(1..3)i = E_(moment),i,   u = B⁻¹ W
```

`B` is rebuilt after every effectiveness update. Feasibility is enforced in
command coordinates, where the motor limits form a box and the map is exactly
linear. Desaturation priorities (the PX4 `MC_AIRMODE = 0` policy, applied
regardless of `MC_AIRMODE`):

1. **Roll/pitch:** scaled as a vector (direction preserved), only if no
   collective at or below the request makes it feasible.
2. **Collective:** reduced as needed; never raised, except to the
   all-motors-minimum value.
3. **Yaw:** scaled to fit without changing 1 or 2. Unlike native PX4, no yaw
   margin is bought by reducing collective.

### Integration in control_allocator

Compiled only when `CONFIG_MODULES_SE3_CONTROL` is set. It links `se3_physical`
and includes `<modules/se3_control/physical/PhysicalAllocation.hpp>`.

**Optional input.** If the `se3_wrench_setpoint` callback cannot be
registered, the allocator logs a warning and native allocation is unaffected.
The physical path then never owns the motors, and SE3 faults on its allocation
timeout.

**Ownership** (`physical_owns_allocation()`). The physical path owns the
motors exactly while the vehicle is armed in the navigation state named by the
wrench, with allocation enabled and the PX4 rate controller disabled. In that
configuration the native normalized path is never used, so only one source
commands the motors.

**No stale commands.**
- A physical command is published only while its wrench is at most 20 ms old
  (`kWrenchTimeout`). After that, nothing is republished.
- After physical ownership ends, the stored command is never republished. Only
  a fresh native allocation drives the motors again.
- An incompatible or rejected wrench allocates and publishes nothing, and the
  previous physical command is no longer republished either.

**Native protections reused.** After allocation the command goes through the
native slew-rate limit (`CA_R*_SLEW`), clipping, stopped-motor and
motor-failure handling, and `actuator_motors` publication.

**Reporting.** `se3_allocation_status` describes the commands actually
published (`Allocation::apply_commands()`): after slew limiting and clipping,
with stopped or failed motors contributing zero thrust. The achieved wrench,
rotor thrusts and `thrust_limited` follow from those commands. Each scale is
capped at `1 − ‖M_achieved − M_requested‖ / ‖M_requested‖` for its moment
(roll/pitch pair, yaw), clamped to [0, 1]: the allocated fraction when the
final moment is the request scaled down, and lower for a direction error, an
overshoot or a moment produced for a zero request. Errors within
`1e-4 · max(1 N m, ‖M_requested‖)` (the tolerance of `thrust_limited`) are
rounding and leave the scale unchanged. `valid = false` means not
all rotors are running. The roll/pitch and yaw scales and `thrust_limited` are
the anti-windup inputs of both SE3 integrals.

## Source organization

| Path | Contents |
|---|---|
| `Runtime.cpp` | `se3_control` module: Commander registration, 100 Hz loop, hold/trajectory lifecycle, path selection, fault handling, shell commands, PHYSICAL inner-loop glue |
| `ControlPath.hpp` | `SE3_CTRL` mapping, per-path PX4 controllers, publish/ownership predicate (pure, unit-tested) |
| `Controller.{hpp,cpp}` | translation law with the position integral, force derivatives, geometric output |
| `Integral.hpp` | bounded, freezable integration step shared by both integrals |
| `AttitudeReference.{hpp,cpp}` | `R_d`, `Ω_d`, `Ω̇_d` from the specific force and heading |
| `AttitudeLaws.{hpp,cpp}` | `e_R`/`e_Ω`; body-rate law and physical Lee moment |
| `Types.hpp` | shared state, gain and output types |
| `trajectory/` | `Trajectory.hpp` interface, `Catalog.cpp` (named plans: entries, patterns, returns), `Parameters.cpp` (`TRJ_*` access), `Sampler.cpp` (Hold, Step, Line, Yaw, Circle and FigureEight segments, analytic derivatives through snap). No dependency on SE3 gains, flight modes or uORB. |
| `physical/` | `se3_physical` library: `PhysicalAllocation` (shared with `control_allocator`). `PhysicalPath` holds the module's PHYSICAL runtime state (mass, inertia, reference, attitude integral), built only into `se3_control`. |
| `Se3MathTest.cpp`, `physical/PhysicalAllocationTest.cpp` | unit tests |
| `module.yaml` | `SE3_*` and `TRJ_*` parameters |

Outside the module:
- `msg/Se3WrenchSetpoint.msg`, `msg/Se3AllocationStatus.msg`, `msg/Se3TrajectoryStatus.msg`
- the physical input in `control_allocator`
- the F450 rotor geometry in `ROMFS/px4fmu_common/init.d-posix/airframes/4022_gz_f450`
- `logger/logged_topics.cpp`
- `boards/px4/fmu-v6c/se3.px4board`
- startup lines in `boards/px4/fmu-v6c/init/rc.board_extras` and `ROMFS/px4fmu_common/init.d-posix/rcS`

## Parameters

| Parameter | Default | Used by | Meaning |
|---|---|---|---|
| `SE3_ENABLE` | 1 | all | allow selection (registers regardless) |
| `SE3_CTRL` | 1 | all | control path, reboot required |
| `SE3_KX`, `SE3_KV` | 4.0, 3.0 | all | position/velocity gain divided by mass |
| `SE3_KIXY`, `SE3_KIZ` | 0.3, 0.6 1/s³ | all | position-integral gain divided by mass |
| `SE3_KRX/KRY/KRZ` | 5, 5, 2 | RATE_PX4 | attitude-to-rate gain |
| `SE3_RATE_MAX` | 3.0 rad/s | RATE_PX4 | body-rate limit |
| `SE3_MASS` | 2.0736 kg | PHYSICAL | vehicle mass |
| `SE3_JXX/JYY/JZZ` | 0.022078, 0.022078, 0.043301 kg m² | PHYSICAL | principal inertia |
| `SE3_PKRX/PKRY/PKRZ` | 5, 5, 1.1 N m/rad | PHYSICAL | `K_R` |
| `SE3_PKWX/PKWY/PKWZ` | 0.53, 0.53, 0.35 N m s/rad | PHYSICAL | `K_Ω` |
| `SE3_PKI_XY`, `SE3_PKI_Z` | 0.5, 0.1 N m/(rad s) | PHYSICAL | `K_I` |
| `MPC_THR_HOVER` | airframe | all | thrust normalization, neutral thrust |
| `TRJ_*` | below | all | trajectory definitions |

ATTITUDE_PX4 and RATE_PX4 additionally use the native `MC_*` attitude and rate
gains through PX4's own controllers. PHYSICAL additionally uses the native
`CA_ROTOR*` effectiveness, motor limits, slew rates and `THR_MDL_FAC`.

Integral gain choice: with `Kx = 4`, `Kv = 3` the translational loop
`s³ + Kv s² + Kx s + Ki` is stable for `Ki < Kv Kx = 12`. The defaults sit far
below that (integral time `Kx/Ki` of 13 s horizontally, 7 s vertically). The
attitude integral is similarly far below its bound `K_Ω K_R / J`.

## Operation

### Shell commands (`pxh>` / MAVLink console)

```
se3_control status                 # path, active/faulted, trajectory, integral, fault response, PHYSICAL perf
se3_control select                 # request the SE3 mode from Commander (may be rejected)
se3_control hold                   # cancel any trajectory; hold the measured position and heading
se3_control list                   # setpoint, steps5, circle, figure8
se3_control trajectory <name>      # start now; replaces any running trajectory
se3_control fault controller|state|allocation   # SITL only, needs SYS_FAILURE_EN=1
```

`se3_control stop` is accepted only while disarmed.

### Hold and trajectories

- **Mode entry is passive:** SE3 captures the measured position and heading and
  holds them until a command arrives. Commands sent before activation are
  discarded.
- **Direct switching.** `trajectory <name>` and `hold` are accepted whenever
  SE3 is active, including during a running trajectory or its entry. The
  latest command wins and replaces the current plan on the next 100 Hz cycle;
  no `hold`, completion or reselection is needed in between. There is no
  blending: `setpoint` and `steps5` step immediately, while `circle` and
  `figure8` start from the position and yaw the current plan has at that
  instant **at rest**, so replacing a moving reference changes its velocity
  discontinuously. `hold` cancels the trajectory and captures the measured
  pose.
- **References are absolute local NED** (`TRJ_HOME_*` is the pattern centre,
  altitude and heading, and the return point).

| Trajectory | Behavior | Parameters (defaults) |
|---|---|---|
| `setpoint` | immediate step to `TRJ_SP_*`, held indefinitely; live parameter edits are immediate steps | `TRJ_SP_X/Y/Z/YAW` (0, 0, −1.0, 0) |
| `steps5` | five true steps, each held `TRJ_STEP_HOLD`, then a Line home and hold | `TRJ_STEP_HOLD` (5 s) |
| `circle` | Line to the +X edge, `TRJ_CIR_TURNS` turns of radius `TRJ_CIR_R` around home in exactly `TRJ_CIR_T` at constant rate, Line home | `TRJ_CIR_R/T/TURNS` (1.0 m, 60 s, 5) |
| `figure8` | Line to home, then `x = a_x sin φ`, `y = a_y sin 2φ` around home, `TRJ_F8_TURNS` cycles in exactly `TRJ_F8_T` at constant rate, ending at home | `TRJ_F8_AX/AY/T/TURNS` (0.8 m, 1.2 m, 90 s, 5) |

`TRJ_HOME_X/Y/Z/YAW` default to (0, 0, −1.2, 0). The `steps5` references are
fixed: (0, 0, −1.2), (1, 0, −1.2), (1, 1, −1.2), (0, 1, −1.2) and
(0, 0, −0.9). The first is taken immediately when the command is applied.

**Plans** (`trajectory/Catalog.cpp`, sampled by `trajectory/Sampler.cpp`).
A plan is a short sequence of segments; each starts at the pose where the
previous one ended.

| Segment | Motion | Time scale |
|---|---|---|
| Hold | stationary | — |
| Step | jumps by its offset at its start, then stationary | — |
| Line, Yaw | position offset or yaw rotation, from rest to rest | ninth-order C4 (derivatives through snap vanish at both ends) |
| Circle, FigureEight | `turns` revolutions/cycles in exactly the segment duration | linear: constant phase rate |

- **steps5:** a hold at the first point and four steps, each lasting
  `TRJ_STEP_HOLD`, at `TRJ_HOME_YAW`; then a Line home.
- **circle:** a Yaw rotation to `TRJ_HOME_YAW` in place (if needed), a Line
  from the current reference position to the circle's +X edge, the circle
  (`TRJ_CIR_T`), and a Line home.
- **figure8:** the same Yaw and Line entry, to `TRJ_HOME_*` (the pattern
  centre), then the figure-eight (`TRJ_F8_T`), which ends at home.
- **Line and Yaw durations** are fixed by the distance: `max(2 s, d / 0.25 m/s)`
  and `max(2 s, |Δψ| / 0.25 rad/s)`, so they peak at about 0.62 m/s and
  0.62 rad/s. Moves shorter than 1 mm (1 mrad) are omitted.
- **Intentional discontinuities:** the patterns start and end with a velocity
  step between rest and the pattern speed (`r ω` = 0.52 m/s for the default
  circle, 0.88 m/s for the default figure-eight). Entry, pattern and return
  are each analytic; their joins are continuous in position only.
- **Parameter snapshot:** all `TRJ_*` values except the live setpoint are
  taken when the command is accepted.
- **Completion:** a finished plan reports `completed` and holds home at rest.

### Commander integration

- **Registration.** At boot SE3 registers a mode and an arming check, without
  activating the mode. It requires a disarmed vehicle at start.
- **Health replies.** It answers Commander's health requests with
  `can_arm_and_run` and the mode requirements: angular velocity, attitude,
  local position, and prevent-arming.
- **Selection** is accepted only when airborne, armed, not in failsafe, on a
  rotary-wing non-VTOL vehicle, with fresh finite estimates and a valid
  configuration for the selected path.

### Faults (`enter_fault()`)

A fault is latched until disarm; a faulted SE3 cannot be re-selected in flight.

| Class | Examples | Response |
|---|---|---|
| Controller | invalid configuration, estimator reset, trajectory or geometry failure, stale reference, allocator incompatible / not replying / motor failure (PHYSICAL) | request Position mode; if rejected or unanswered within 100 ms, report unhealthy so PX4's failsafe acts |
| State | invalid or stale estimate, landed, failsafe, disarmed | report unhealthy immediately; PX4's failsafe acts |

Until Commander leaves the mode, SE3 publishes a neutral command for its path:

- **RATE_PX4:** zero body rates with hover thrust.
- **ATTITUDE_PX4:** a level attitude at the current heading with hover thrust.
- **PHYSICAL:** SE3 republishes its mode configuration with the PX4 rate
  controller enabled. Torque production then returns to `mc_rate_control`
  (tracking zero rates and hover thrust), and the allocator leaves physical
  ownership. The PHYSICAL configuration is restored on disarm.

**Returning to Position mode** is a normal pilot or Commander mode change
(`commander mode posctl` in SITL). SE3 stops publishing as soon as its mode or
controllers are no longer active.

## Building and uploading FMUv6C firmware

SE3 is **not** in the standard FMUv6C firmware. The SE3 firmware is a label
variant of the same board:

```
make px4_fmu-v6c_se3                 # build/px4_fmu-v6c_se3/px4_fmu-v6c_se3.px4
make px4_fmu-v6c_se3 upload          # standard PX4 uploader (px_uploader.py) over USB
make px4_fmu-v6c_default             # standard firmware, without SE3
```

`boards/px4/fmu-v6c/se3.px4board` inherits `default.px4board`.

- **Adds:** `CONFIG_MODULES_SE3_CONTROL=y`.
- **Removes:**
  - fixed-wing attitude/rate/position control and FW autotune;
  - VTOL attitude control, rover position control, MC autotune;
  - SIH, together with the HITL simulation drivers it selects;
  - the landing-target estimator, camera trigger/capture/feedback, gimbal;
  - the airspeed selector and differential-pressure drivers.

  Each of these starts only behind a parameter check in the startup scripts.
- **Retains** everything else: MC attitude, rate and position control,
  FlightModeManager, control allocator, EKF2 and sensors, Commander, land
  detector, battery monitoring, PWM/DShot/PX4IO outputs, MAVLink,
  uXRCE-DDS, logger, RC/manual control and navigator.

Both variants have board id 56 and the same 1,966,080-byte image limit, so the
standard bootloader and uploader accept either. Startup runs
`se3_control start` only when its parameters exist
(`param greater -s SE3_ENABLE -1`).

Measured flash use (1920 KiB limit):

| Build | Image | Used | Free |
|---|---|---|---|
| `px4_fmu-v6c_default` | 1,919,772 B | 97.64% | 46,308 B |
| `px4_fmu-v6c_se3` | 1,739,868 B | 88.49% | 226,212 B |

Both build without compiler warnings. Upload has **not** been run on
hardware.

## Logging

| Topic | Rate in the default profile | Content |
|---|---|---|
| `se3_trajectory_status` | 10 ms | reference (position through snap, yaw), run id, plan CRC, trajectory id (−1 hold), segment, phase, position-integral acceleration and freeze flag |
| `trajectory_setpoint` | 200 ms | same reference on the standard topic (observation; `mc_pos_control` is off) |
| `vehicle_attitude_setpoint` | 50 ms | geometric `R_d` and thrust (the command in ATTITUDE_PX4) |
| `se3_wrench_setpoint` | 10 ms | PHYSICAL requested wrench, attitude-integral moment, neutral flag, gyro sample time |
| `se3_allocation_status` | 10 ms | requested/achieved wrench, roll/pitch and yaw scale, thrust limit, rotor thrusts, motor commands, valid/compatible |

**High rate.** Setting the high-rate bit (`SDLOG_PROFILE` |= 16) logs
`se3_wrench_setpoint` and `se3_allocation_status` on every update, together
with `vehicle_angular_velocity`, `vehicle_rates_setpoint`, `actuator_motors`
and the thrust/torque setpoints. The physical topics are published only while
PHYSICAL owns allocation, so other modes carry no extra load.

**Measuring the real logged rate.** On hardware the logger loop runs every
3.5 ms (~285 Hz) and logs at most one message per topic per loop. Gyro-rate
topics above that rate are therefore decimated; `timestamp_sample` gaps in the
ULog show the actual rate.

**SITL.** SITL logs honour the per-topic intervals above (measured:
`se3_trajectory_status` 98–100 Hz, `vehicle_attitude_setpoint` 19.5 Hz,
`trajectory_setpoint` 4.9 Hz, unlisted topics such as `actuator_motors` at
about 49 Hz). Use `uorb top` for the actual publication rates.

## Validation

### Unit tests

```
make tests TESTFILTER=Se3Math
make tests TESTFILTER=PhysicalAllocation
```

- **Se3Math** (29 tests):
  - trajectories: the catalog, agreed defaults and plan layouts, true steps
    at 5 s, exactly five circle turns and figure-eight cycles at constant
    rate, analytic derivatives against finite differences, the Yaw/Line
    entry, direct replacement (position kept, velocity discontinuity
    explicit), measured-pose hold and invalid settings;
  - the bounded integral (applied rate equals the actual change, bounds,
    freeze and resume) and the position integral (signs, bound step, reset,
    zero-gain equivalence with the non-integral controller), and force
    derivatives including the integral;
  - attitude-reference derivatives against finite differences, body-rate and
    Lee moment terms with and without the attitude integral;
  - `SE3_CTRL` mapping, per-path controllers, an exhaustive check that the
    RATE_PX4 publish predicate equals the original, and allocator ownership
    for each path, including the fault fallback.
- **PhysicalAllocation** (16 tests): effectiveness signs, linear thrust,
  invalid models, small SI coefficients below the native filter threshold,
  exact reproduction of feasible wrenches, desaturation priorities,
  saturation feedback from the published (slew-limited or stopped) commands,
  including off-axis, overshooting and reversed moments, moments for a zero
  request and rounding-level changes, ownership, an unavailable wrench input
  and stale-command freshness.

The native `ControlAllocation*` and `ActuatorEffectiveness*` tests also pass.

### Gazebo F450 SITL

Headless `gz_f450` SITL with external-vision EKF2, default `SE3_*`/`TRJ_*`
values, `MIS_TAKEOFF_ALT = 1.2` (so the hover matches `TRJ_HOME_Z`), driven
from the PX4 shell. Each path flew one campaign: native takeoff, Position,
`select` and passive hold, live setpoint edits (x 0.5 m, yaw 0.5 rad,
z −0.3 m, then back), `steps5`, `circle` to completion, `figure8` replaced
directly by `circle` and then by `hold`, `figure8` to completion, return to
Position, re-selection and a controller fault; then a second flight with a
state fault (PHYSICAL: allocation fault). Results are from the ULogs
(`se3_trajectory_status` reference against `vehicle_local_position`).

| Position error RMS | ATTITUDE_PX4 | RATE_PX4 | PHYSICAL | PHYSICAL, matched thrust curve¹ |
|---|---|---|---|---|
| Hold (12 s) | 0.036 m | 0.028 m | 0.616 m (vertical) | 0.021 m |
| Circle run (68 s, 4.999 turns) | 0.025 m | 0.035 m | 0.452 m (0.046 m horizontal) | 0.028 m |
| Figure-eight run (94 s) | 0.027 m | 0.039 m | 0.456 m (0.055 m horizontal) | 0.032 m |

¹ Runtime-only `THR_MDL_FAC = 0.72` and `MPC_THR_HOVER = 0.42` (see
Thrust-model accuracy); hold, circle and figure-eight only.

- **References:** `steps5` visited exactly the five points, changing at 5.01,
  10.01, 15.01 and 20.01 s. The circle swept 4.999 turns at a constant
  0.524 m/s for 59.99 s; the figure-eight had amplitudes 0.800 × 1.200 m over
  89.99 s. Velocity steps occurred only where intended: 0.52 m/s at circle
  start and end, 0.88 m/s at figure-eight start, and 0.44–0.52 m/s on direct
  replacement of a moving reference. Position stayed continuous (≤ 5 mm) at
  every circle/figure-eight entry.
- **Publications in SE3** (`uorb top`):
  - ATTITUDE_PX4: SE3 `vehicle_attitude_setpoint` at 100 Hz; `vehicle_rates_setpoint` at 250 Hz from `mc_att_control`.
  - RATE_PX4: SE3 `vehicle_rates_setpoint` at 100 Hz; torque, thrust and `actuator_motors` at 250 Hz.
  - PHYSICAL: `se3_wrench_setpoint`, `se3_allocation_status` and `actuator_motors` at 250 Hz (inner interval 4.000 ms); no `vehicle_torque_setpoint`, which resumed at 250 Hz as soon as SE3 was left.
- **Allocation (PHYSICAL):** every wrench compatible and valid. The
  achieved wrench equalled the request except during the 90° yaw step of the
  first setpoint (yaw scaled in 0.16 % of samples, as intended by the yaw
  priority); roll/pitch were never scaled and thrust never limited.
- **Integrals:** the position integral froze only in ATTITUDE_PX4 during that
  yaw step (0.4 % of the setpoint run). The PHYSICAL attitude integral stayed
  below 0.033 N m.
- **Fault responses** (last SE3 reference to the mode change): controller
  fault to Position in 32–36 ms (PHYSICAL also switched to the PX4 rate
  controller first); state fault to Commander's Land failsafe in 68–72 ms;
  PHYSICAL allocation fault to Position in 28 ms. Each fault stayed latched
  until disarm.

### Known limitations

- **All closed-loop evidence is from Gazebo SITL with the Gazebo F450 model.**
  Nothing has been flown or bench-tested on hardware. The FMUv6C SE3 image
  builds but has not been uploaded or run.
- **PHYSICAL needs a correct thrust curve.** With the `4022_gz_f450` default
  `THR_MDL_FAC = 0`, PHYSICAL holds about 0.45 m below its reference with the
  vertical integral at its bound (see Validation). Matching `THR_MDL_FAC`
  (0.72 for this model) removes the error but changes native control of the
  airframe too (`MPC_THR_HOVER` becomes about 0.42); the airframe defaults
  have not been changed.
- **PHYSICAL vehicle parameters** (`SE3_MASS`, `SE3_J*`, `CA_ROTOR*_CT/KM`,
  `THR_MDL_FAC`) are known only for the Gazebo F450.
- **Trajectory discontinuities are intentional:** steps, the velocity steps at
  pattern start and end, and on direct replacement of a moving reference.
- **Not exercised in SITL:** a work-queue stall, gyro loss, a forced disarm in
  flight and motor failure.
- **Allocation:** yaw has the lowest priority with no thrust-for-yaw margin,
  and the physical path ignores `MC_AIRMODE`.
- On hardware, the gyro-triggered rates follow `IMU_GYRO_RATEMAX` and the
  attitude loop follows the estimator rate; neither has been measured.
