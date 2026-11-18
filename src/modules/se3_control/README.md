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

All three paths share the same trajectories, geometric reference, hold,
Commander lifecycle and fault handling. Only what is commanded differs.

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

`Controller.cpp`, mass-normalized (gains are `kx/m`, `kv/m`):

```
a     = a_ref - Kx (p - p_ref) - Kv (v - v_ref)      local NED
A/m   = a - g e3                                     specific force
u     = -(A/m) · (R e3)                              collective (rejected if <= 0)
```

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
M = −K_R e_R − K_Ω e_Ω + Ω × JΩ + J α_ff                       [N m]
α_ff = −Ω × (Rᵀ R_d Ω_d) + Rᵀ R_d Ω̇_d
K_R = SE3_PKR{X,Y,Z} [N m/rad],  K_Ω = SE3_PKW{X,Y,Z} [N m s/rad]
```

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

### Vehicle model (`PhysicalModel.{hpp,cpp}`)

Models are plain data selected by `SE3_MODEL`. Each wrench carries the model
id, so the controller (mass, inertia) and the allocator (geometry, thrust
curves) always use the same description. Only one model exists:

| `SE3_MODEL` | Model | Source |
|---|---|---|
| 0 | Gazebo F450 | `Tools/simulation/gz/models/f450/model.sdf`, airframe `4022_gz_f450` |

- **Mass:** 2.0736 kg (base 2.0232 kg + 4 × 0.0126 kg rotors).
- **Inertia:** diag(0.022078, 0.022078, 0.043301) kg m² about the composite
  centre of mass.
- **Rotors** (`actuator_motors` order, FRD), arm 0.16263 m, plane 0.0112 m
  above the CoM:
  - 0: front right, CCW
  - 1: back left, CCW
  - 2: front left, CW
  - 3: back right, CW
- **Reaction coefficient:** 0.0137 m (the SDF `momentConstant`).
- **Thrust curve:** Gazebo `T = k_f ω²` with `k_f = 1.2e-5`, and PX4 commands
  `ω = 150 + 850 u` (`SIM_GZ_EC_MIN/MAX`). Therefore
  `T(u) = 0.27 + 3.06 u + 8.67 u²` N on `u ∈ [0, 1]`.
- **Not modelled:** rotor drag and rolling moment, the 12.5 ms motor time
  constant, airspeed thrust loss, and Gazebo's 1 rad/s speed quantization.

These are simulation coefficients. **No hardware motor model exists.** A
hardware model needs measured coefficients and must not reuse these.

`valid()` requires:
- positive mass and a symmetric positive-definite inertia;
- vertical unit thrust axes;
- convex, strictly increasing thrust curves.

`rotor_command()` inverts `T(u)` in closed form, using the larger root in a
cancellation-free form, and rejects thrusts outside `[T(u_min), T(u_max)]`.

### Compatibility requirements

PHYSICAL refuses to run (SE3 reports itself unhealthy and cannot be selected)
unless all of the following hold. They are checked at boot and on every
parameter update; none is ever changed automatically.

- `THR_MDL_FAC = 0`. Otherwise PX4 would apply a second thrust curve to the
  same command.
- The model's required parameters, for the Gazebo F450:
  - `SYS_AUTOSTART=4022`
  - `CA_AIRFRAME=0`, `CA_ROTOR_COUNT=4`
  - `SIM_GZ_EC_FUNC1..4 = 101..104`
  - `SIM_GZ_EC_MIN1..4 = 150`, `SIM_GZ_EC_MAX1..4 = 1000`
- Positive, finite `SE3_PK*` gains.

In `control_allocator` (`se3_motors_compatible()`) the following must also
hold; otherwise the wrench is reported as `compatible = false`:
- exactly one effectiveness matrix;
- the model's 4 motors and no servos;
- non-reversible motors with the [0, 1] range.

### Rotor-thrust allocation (`PhysicalAllocation.{hpp,cpp}`)

The wrench `W = [f, Mx, My, Mz]` is linear in the rotor thrusts:

```
W = B T,   B_0i = −n_i,z,   B_(1..3)i = r_i × n_i − k_i n_i,   T = B⁻¹ W
```

Feasibility is enforced in thrust coordinates, where the rotor limits form a
box and the map is exactly linear. Each feasible thrust is then inverted
through the thrust curve, so no linearization of the motor model is involved.
Desaturation priorities (the PX4 `MC_AIRMODE = 0` policy, applied regardless
of `MC_AIRMODE`):

1. **Roll/pitch:** scaled as a vector (direction preserved), only if no
   collective at or below the request makes it feasible.
2. **Collective:** reduced as needed; never raised, except to the all-rotors-idle
   minimum.
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
- An incompatible wrench allocates and publishes nothing.

**Native protections reused.** After allocation the command goes through the
native slew-rate limit (`CA_R*_SLEW`), clipping, stopped-motor and
motor-failure handling, and `actuator_motors` publication.

**Reporting.** `se3_allocation_status` reports the wrench of the commands
actually published: after slew limiting, with stopped or failed motors
contributing zero thrust. `valid = false` means not all rotors are running.

## Source organization

| Path | Contents |
|---|---|
| `Runtime.cpp` | `se3_control` module: Commander registration, 100 Hz loop, hold/trajectory lifecycle, path selection, fault handling, shell commands, PHYSICAL inner-loop glue |
| `ControlPath.hpp` | `SE3_CTRL` mapping, per-path PX4 controllers, publish/ownership predicate (pure, unit-tested) |
| `Controller.{hpp,cpp}` | translation law, force derivatives, geometric output |
| `AttitudeReference.{hpp,cpp}` | `R_d`, `Ω_d`, `Ω̇_d` from the specific force and heading |
| `AttitudeLaws.{hpp,cpp}` | `e_R`/`e_Ω`; body-rate law, physical Lee moment, and a normalized-torque law that the runtime does not use |
| `Types.hpp` | shared state, gain and output types |
| `trajectory/` | `Trajectory.hpp` interface, `Catalog.cpp` (named trajectories, `TRJ_*` parameters), `Sampler.cpp` (analytic segment sampling, ninth-order time scaling, derivatives through snap). No dependency on SE3 gains, flight modes or uORB. |
| `physical/` | `se3_physical` library: `PhysicalModel`, `PhysicalAllocation` (shared with `control_allocator`). `PhysicalPath` holds the module's PHYSICAL runtime state, built only into `se3_control`. |
| `Se3MathTest.cpp`, `physical/PhysicalAllocationTest.cpp` | unit tests |
| `module.yaml` | `SE3_*` and `TRJ_*` parameters |

Outside the module:
- `msg/Se3WrenchSetpoint.msg`, `msg/Se3AllocationStatus.msg`, `msg/Se3TrajectoryStatus.msg`
- the physical input in `control_allocator`
- `logger/logged_topics.cpp`
- `boards/px4/fmu-v6c/se3.px4board`
- startup lines in `boards/px4/fmu-v6c/init/rc.board_extras` and `ROMFS/px4fmu_common/init.d-posix/rcS`

## Parameters

| Parameter | Default | Used by | Meaning |
|---|---|---|---|
| `SE3_ENABLE` | 1 | all | allow selection (registers regardless) |
| `SE3_CTRL` | 1 | all | control path, reboot required |
| `SE3_KX`, `SE3_KV` | 4.0, 3.0 | all | position/velocity gain divided by mass |
| `SE3_KRX/KRY/KRZ` | 5, 5, 2 | RATE_PX4 | attitude-to-rate gain |
| `SE3_RATE_MAX` | 3.0 rad/s | RATE_PX4 | body-rate limit |
| `SE3_MODEL` | 0 | PHYSICAL | physical model, reboot required |
| `SE3_PKRX/PKRY/PKRZ` | 5, 5, 1.1 N m/rad | PHYSICAL | `K_R` |
| `SE3_PKWX/PKWY/PKWZ` | 0.53, 0.53, 0.35 N m s/rad | PHYSICAL | `K_Ω` |
| `MPC_THR_HOVER` | airframe | all | thrust normalization, neutral thrust |
| `TRJ_*` | see `module.yaml` | all | trajectory definitions |

ATTITUDE_PX4 and RATE_PX4 additionally use the native `MC_*` attitude and rate
gains through PX4's own controllers.

## Operation

### Shell commands (`pxh>` / MAVLink console)

```
se3_control status                 # path, active/faulted, trajectory, fault response, PHYSICAL perf
se3_control select                 # request the SE3 mode from Commander (may be rejected)
se3_control hold                   # capture and hold the measured position and heading
se3_control list                   # setpoint, steps5, circle, figure8
se3_control trajectory <name>      # start; requires SE3 active and holding
se3_control fault controller|state|allocation   # SITL only, needs SYS_FAILURE_EN=1
```

`se3_control stop` is accepted only while disarmed.

### Hold and trajectories

- **Mode entry is passive:** SE3 captures the measured position and heading and
  holds them. `hold` re-captures at any time.
- **References are absolute local NED.** No alignment to the current pose
  occurs.
  - **circle:** starts at `(TRJ_HOME_X + TRJ_CIR_R, TRJ_HOME_Y, TRJ_HOME_Z)`.
  - **figure8:** starts at `TRJ_HOME_*`.
  - **steps5:** five fixed points around `(0, 0, −0.75)`, each held for
    `TRJ_STEP_HOLD`.
  - **setpoint:** follows `TRJ_SP_*` live; edits become immediate steps.
- All other values are snapshotted at start. A completed trajectory holds its
  final reference.

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
| `px4_fmu-v6c_default` | 1,919,748 B | 97.64% | 46,332 B |
| `px4_fmu-v6c_se3` | 1,736,780 B | 88.34% | 229,300 B |

Upload was verified only to the extent that the target exists and the image
metadata matches. It has **not** been run on hardware.

## Logging

| Topic | Rate in the default profile | Content |
|---|---|---|
| `se3_trajectory_status` | 10 ms | reference (position through snap, yaw), run id, plan CRC, phase |
| `trajectory_setpoint` | 200 ms | same reference on the standard topic (observation; `mc_pos_control` is off) |
| `vehicle_attitude_setpoint` | 50 ms | geometric `R_d` and thrust (the command in ATTITUDE_PX4) |
| `se3_wrench_setpoint` | 10 ms | PHYSICAL requested wrench, model, neutral flag, gyro sample time |
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

**SITL caveat.** SITL starts the logger with `-p ekf2_timestamps`, which
ignores per-topic intervals. SITL logs record everything at the estimator rate
(~250 Hz) and do not reflect hardware logging rates.

## Validation

### Unit tests

```
make tests TESTFILTER=Se3Math
make tests TESTFILTER=PhysicalAllocation
```

- **Se3Math:**
  - trajectory derivatives;
  - attitude-reference derivatives against finite differences;
  - body-rate and Lee moment terms;
  - `SE3_CTRL` mapping, per-path controllers, and an exhaustive check that the
    RATE_PX4 publish predicate equals the original;
  - allocator ownership for each path, including the fault fallback.
- **PhysicalAllocation:**
  - model constants and thrust inversion;
  - invalid models;
  - effectiveness signs;
  - exact reproduction of feasible wrenches and the desaturation priorities;
  - ownership, an unavailable wrench input, and stale-command freshness.

### Gazebo F450 SITL

Headless `gz_f450` SITL with external-vision EKF2. Each run flew: takeoff,
Position, select, hold, circle, figure8, live position and 2.5 rad yaw steps,
pilot return to Position, controller fault, state fault.

| | ATTITUDE_PX4 | RATE_PX4 | PHYSICAL |
|---|---|---|---|
| Hold, position error RMS | 0.057 m | 0.041 m | 0.008 m |
| Circle, position error RMS | 0.111 m | 0.104 m | 0.092 m |
| Figure-8, position error RMS | 0.086 m | 0.093 m | 0.076 m |
| Rate-setpoint source | `mc_att_control` at the attitude rate | SE3 at 100 Hz | none (rate loop off) |
| Allocation | native | native | physical, 249.9 Hz (gyro rate) |

- **RATE_PX4 regression.** Identical flights on the pre-change implementation
  gave the same mode timeline, fault behavior and topic rates, with
  rate-setpoint RMS within 0.005 rad/s. The tracking differences are run-to-run
  noise.
- **Fault responses:**
  - Controller fault: Position accepted after about 24 ms.
  - State fault: PX4 failsafe (Land).
  - PHYSICAL allocation fault (unknown model): native rate control after
    12 ms, Position after 24 ms, no motor-command gap above 8 ms.
  - PHYSICAL total gyro loss (`sensors stop`): detected 36 ms after the last
    sample; Position was rejected, and Commander terminated the flight.
- **Position handoff:** under 4 cm drift over 5 s in every path.
- **Model checks against Gazebo:**
  - Roll/pitch angular acceleration fits the achieved moment with a gain of
    about 0.91 and about 20 ms delay. The delay is the motor time constant,
    one simulation step and the gyro filter.
  - Yaw fits with a gain of 0.998.
  - Yaw authority at hover is about 0.27 N m, so large yaw steps saturate the
    yaw axis (`yaw_scale < 1`).

### Known limitations

- **All closed-loop evidence is from Gazebo SITL with the Gazebo F450 model.**
  Nothing has been flown or bench-tested on hardware.
  - PHYSICAL has no hardware model. On hardware the Gazebo model's required
    parameters (`SYS_AUTOSTART=4022`, `SIM_GZ_EC_*`) do not match, so SE3
    reports itself unhealthy and PHYSICAL cannot be selected.
  - On hardware, the gyro-triggered rates follow `IMU_GYRO_RATEMAX`, and the
    attitude loop follows the estimator rate; neither has been measured.
- **Not exercised in SITL:** a work-queue stall and a forced disarm in flight.
  After the gyro-loss test the SITL logger stops with the IMU, so motor
  outputs after that event are not in the log.
- **Allocation:** yaw has the lowest priority with no thrust-for-yaw margin,
  and the physical path ignores `MC_AIRMODE`.
- **Gazebo model gaps:** motor dynamics, rotor drag and speed quantization are
  not in the model and appear as a reduced effective roll/pitch gain with delay.
