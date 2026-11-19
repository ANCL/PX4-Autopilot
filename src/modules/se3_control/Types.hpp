// Vehicle state and controller-specific data structures.
//
// Position, velocity, and acceleration use PX4 local NED.
// Attitude maps the body FRD frame to local NED.
// Angular velocity is expressed in the body FRD frame.

#pragma once

#include <matrix/matrix/math.hpp>

namespace se3_control
{

using Vector3 = matrix::Vector3f;
using Rotation = matrix::Dcmf;

struct State
{
  Vector3 position{};
  Vector3 velocity{};
  Rotation attitude{};
  Vector3 angular_velocity{};
};

struct TranslationGains
{
  // Mass-normalized translational gains: kx/m [1/s^2], kv/m [1/s] and the
  // per-axis integral gain ki/m [1/s^3] (x, y share SE3_KIXY; z is SE3_KIZ).
  float position{0.0f};
  float velocity{0.0f};
  Vector3 integral{};
};

// Bounded integral of the position error, I = int sat(e_x) dt [m s], with the
// time derivatives it had in its latest update. Derivatives are zero on axes
// that were frozen, held at the bound or not integrated, so the analytic
// desired-force derivatives match the integral actually applied.
struct PositionIntegral
{
  Vector3 value{};
  Vector3 rate{};             // dI/dt [m].
  Vector3 rate_derivative{};  // d^2I/dt^2 [m/s].
};

struct TranslationOutput
{
  // Kinematic acceleration command in local NED [m/s^2].
  Vector3 acceleration{};

  // Integral part -K_i I of the acceleration command [m/s^2].
  Vector3 integral{};

  // Specific force A/m in NED [m/s^2], including gravity compensation.
  // Not a normalized PX4 thrust setpoint.
  Vector3 specific_force{};
};

// Desired attitude and its analytic derivatives.
//
// The rotation maps desired body FRD to local NED. Angular velocity and
// acceleration are expressed in the desired body FRD frame.
struct DesiredAttitudeDynamics
{
  Rotation attitude{};
  Vector3 angular_velocity{};       // [rad/s].
  Vector3 angular_acceleration{};   // [rad/s^2].
};

// PX4-normalized geometric attitude gains.
// The feedforward gain multiplies angular acceleration [rad/s^2].
struct NormalizedAttitudeGains
{
  Vector3 attitude{};
  Vector3 angular_velocity{};
  Vector3 angular_acceleration{};
};

struct NormalizedTorqueOutput
{
  Vector3 attitude_feedback{};
  Vector3 angular_velocity_feedback{};
  Vector3 acceleration_feedforward{};
  Vector3 torque{};
};

// Physical geometric attitude gains (SE3_CTRL = PHYSICAL), per body axis:
// K_R [N m/rad], K_Omega [N m s/rad] and K_I [N m/(rad s)].
struct PhysicalAttitudeGains
{
  Vector3 attitude{};
  Vector3 angular_velocity{};
  Vector3 integral{};
};

// Terms of the physical Lee moment [N m] in the current body FRD frame.
struct PhysicalMomentOutput
{
  Vector3 attitude_error{};             // e_R, input of the attitude integral
  Vector3 attitude_feedback{};          // -K_R e_R
  Vector3 angular_velocity_feedback{};  // -K_Omega e_Omega
  Vector3 integral_feedback{};          // -K_I e_I
  Vector3 gyroscopic{};                 // Omega x J Omega
  Vector3 inertial_feedforward{};       // J alpha_ff
  Vector3 moment{};
};

// First/second derivatives of the desired specific force in NED.
// Units are m/s^3 and m/s^4; these drive desired-attitude dynamics.
struct ForceDerivatives
{
  Vector3 first{};
  Vector3 second{};
};

// Common geometric reference calculated before any attitude law.
//
// The runtime commands the desired attitude directly (SE3_CTRL =
// ATTITUDE_PX4), or uses the body-rate law (RATE_PX4) or the physical moment
// law (PHYSICAL). The normalized torque law in AttitudeLaws
// uses the same reference but is not selected by the runtime.
// Collective acceleration is the positive body-Z projection of A/m.
struct GeometricOutput
{
  TranslationOutput translation{};
  ForceDerivatives force_derivatives{};
  DesiredAttitudeDynamics desired_attitude{};
  float collective_acceleration{0.0f};
};

}  // namespace se3_control
