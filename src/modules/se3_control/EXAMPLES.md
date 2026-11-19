# Onboard SE3 — Flight Examples

Practical commands for configuring and operating the native PX4 SE3 controller.

For architecture, mathematics, gains, physical allocation, and implementation
details, see [README.md](README.md).

The module does not arm, take off, or land the vehicle. Start from a stable,
airborne PX4 Position-mode hover. Maintain a working pilot-controlled return
to Position mode throughout each experiment.

## 1. Build and upload firmware

On the Ubuntu build workstation:

```bash
cd ~/px4_env/px4/PX4-Autopilot
make px4_fmu-v6c_se3 upload
```

Use only with a confirmed FMUv6C flight controller. Remove propellers before
firmware servicing. Connect the controller by USB when the uploader requests
its bootloader. Verify the completed upload.

The standard `px4_fmu-v6c_default` firmware does not contain SE3.

## 2. Inspect the configuration

In QGroundControl's MAVLink Console (`pxh>`), while disarmed:

```text
ver hw
ver git
se3_control status
se3_control list
param show SE3_*
param show TRJ_*
param show MPC_THR_HOVER
```

Control paths:

| SE3_CTRL | Controller |
|---|---|
| 0 | ATTITUDE_PX4 |
| 1 | RATE_PX4 (default) |
| 2 | PHYSICAL |

PHYSICAL uses `SE3_MASS`, `SE3_JXX/JYY/JZZ` and the vehicle's
`CA_ROTOR*` geometry, `CT` (newtons at full command), `KM` and
`THR_MDL_FAC`. The defaults describe the Gazebo F450. No real-hardware
values have been identified or validated: do not use PHYSICAL on the
physical F450 before measuring them.

## 3. Select the controller

While disarmed, choose ATTITUDE_PX4:

```text
param set SE3_CTRL 0
param save
reboot
```

Or choose RATE_PX4:

```text
param set SE3_CTRL 1
param save
reboot
```

After reboot:

```text
se3_control status
param show SE3_CTRL
```

SE3_CTRL is read at module initialization. Changing it requires a reboot;
it cannot be switched during flight.

## 4. Configure trajectories

All positions are absolute PX4 local NED coordinates, in metres.
Negative Z is above the local origin. Verify the current position and
available flight volume before entering coordinates.

Example parameter names for the circle and figure-eight:

```text
param show TRJ_HOME_X
param show TRJ_HOME_Y
param show TRJ_HOME_Z
param show TRJ_HOME_YAW
param show TRJ_CIR_R
param show TRJ_CIR_T
param show TRJ_CIR_TURNS
param show TRJ_F8_AX
param show TRJ_F8_AY
param show TRJ_F8_T
param show TRJ_F8_TURNS
```

Defaults: home (0, 0, −1.2) m with yaw 0; circle 1.0 m radius, 5 turns
in 60 s; figure-eight amplitudes 0.8 m (X) and 1.2 m (Y), 5 cycles in
90 s. Set only values suitable for your actual local-NED origin and
flight area, for example:

```text
param set TRJ_HOME_Z -1.2
param set TRJ_CIR_R 1.0
param set TRJ_CIR_T 60
param set TRJ_F8_AX 0.8
param set TRJ_F8_AY 1.2
param set TRJ_F8_T 90
```

The numbers above are examples, not universally safe flight targets.
Both patterns are centred on `TRJ_HOME_*`. The reference first turns to
`TRJ_HOME_YAW` in place (if needed) and moves along a straight line from
its current position to the pattern start (the circle's +X edge, or home
for the figure-eight), at rest at both ends. It then flies the configured
turns at a constant rate in exactly `TRJ_CIR_T` / `TRJ_F8_T`; the
reference velocity steps from rest to the pattern speed (0.52 m/s and
0.88 m/s with the defaults) at the start and back at the end. The circle
then returns home along a line; the figure-eight already ends at home.
Each entry or return line takes `max(2 s, distance / 0.25 m/s)`. Check
that the path from the current reference to the pattern and back home is
clear.

Live setpoint parameters:

```text
param show TRJ_SP_X
param show TRJ_SP_Y
param show TRJ_SP_Z
param show TRJ_SP_YAW
```

Set these near the intended hover position before starting a live-setpoint
trajectory. Parameter edits become immediate reference steps.

The `steps5` trajectory steps through five fixed points, each held for
`TRJ_STEP_HOLD` (5 s): `(0, 0, -1.2)`, `(1, 0, -1.2)`, `(1, 1, -1.2)`,
`(0, 1, -1.2)` and `(0, 0, -0.9)`. These are true steps; the first is
taken immediately. Afterwards it returns along a line to `TRJ_HOME_*`.

## 5. Take off and enter SE3

Arm and take off normally using the pilot controls. Establish a stable
hover in native PX4 Position mode and verify the estimator.

In the PX4 console, once airborne:

```text
se3_control select
se3_control status
```

Confirm:

- `active=1`
- `faulted=0`
- `trajectory: measured-pose hold, holding`

SE3 initially captures and holds the measured position and heading.
No trajectory starts automatically.

## 6. Execute and interrupt trajectories

Any command takes effect immediately while SE3 is active, including
during a running trajectory: no `hold`, completion or reselection is
needed in between.

```text
se3_control trajectory circle
se3_control status
se3_control trajectory figure8     # replaces the circle; starts from the current position at rest
se3_control trajectory steps5      # steps at once to (0, 0, -1.2)
se3_control hold                   # cancels and holds the measured pose
```

Verify every `steps5` target before running it.

For a live setpoint, first configure TRJ_SP_* to a safe position:

```text
se3_control trajectory setpoint
se3_control status
```

While that trajectory is running, modifying TRJ_SP_* immediately
changes the commanded reference:

```text
param set TRJ_SP_X 0.25
se3_control hold
```

The X value above is only an example and must be replaced if unsafe.

The setpoint stays active indefinitely. Finite trajectories return
home and hold it when completed (`completed (holding home)` in
`se3_control status`); `se3_control hold` instead captures the
current measured pose.

## 7. Return to PX4 Position mode

In the PX4 console:

```text
commander mode posctl
commander status
```

Confirm that native Position mode has taken over. Land normally using
pilot controls. Do not stop SE3 or required estimator/communication
services while airborne.

If SE3 reports a fault, prioritize the established pilot recovery and
PX4 failsafe procedures rather than attempting to restart the module
in flight.

## 8. Retrieve and analyze flight logs

After landing and disarming, use:

QGroundControl → Analyze Tools → Log Download → Refresh → Download

Save the original `.ulg` file.

On the toolkit workstation:

```bash
cd ~/px4_env
./tools/px4_analyze /path/to/flight.ulg --profile onboard_se3
```

The toolkit provides ULog processing, tracking plots, and analysis.
Its specialized SE3 analysis currently has limitations for live-setpoint
runs and physical-wrench diagnostics.

For complete toolkit usage, see:
https://ancl.github.io/px4-dev-toolkit/
