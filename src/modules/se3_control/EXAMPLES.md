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
| 2 | PHYSICAL (Gazebo F450 only) |

PHYSICAL has no validated real-hardware motor model. Do not use it on
the physical F450.

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

Set only values suitable for your actual local-NED origin and flight area:

```text
param set TRJ_HOME_X 0
param set TRJ_HOME_Y 0
param set TRJ_HOME_Z -0.75
param set TRJ_CIR_R 0.5
param set TRJ_CIR_T 20
param set TRJ_F8_AX 0.5
param set TRJ_F8_AY 0.5
param set TRJ_F8_T 30
```

The numbers above are examples, not universally safe flight targets.
Circle starts at `TRJ_HOME_X + TRJ_CIR_R`; figure-eight starts at
`TRJ_HOME_*`. Neither starts automatically at the current aircraft position.

Live setpoint parameters:

```text
param show TRJ_SP_X
param show TRJ_SP_Y
param show TRJ_SP_Z
param show TRJ_SP_YAW
```

Set these near the intended hover position before starting a live-setpoint
trajectory. Parameter edits become immediate reference steps.

The `steps5` trajectory contains five fixed points around
`(0, 0, -0.75)` and does not use `TRJ_HOME_*`.

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
- `trajectory: holding`

SE3 initially captures and holds the measured position and heading.
No trajectory starts automatically.

## 6. Execute and interrupt trajectories

Start a configured circle:

```text
se3_control trajectory circle
se3_control status
```

Interrupt it and hold the measured position:

```text
se3_control hold
se3_control status
```

Start a configured figure-eight:

```text
se3_control trajectory figure8
se3_control status
se3_control hold
```

Run the five predefined setpoints only after verifying every target:

```text
se3_control trajectory steps5
se3_control status
se3_control hold
```

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

A new trajectory can be started after returning to holding.
Finite trajectories hold their final commanded reference when completed;
`se3_control hold` instead captures the current measured pose.

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
