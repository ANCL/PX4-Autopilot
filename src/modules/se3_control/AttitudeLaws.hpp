// Geometric attitude control laws.
//
// Both control laws share the same geometric SO(3) tracking errors.

#pragma once

#include "Types.hpp"

namespace se3_control
{

class AttitudeLaws
{
public:
  // Geometric attitude-to-body-rate handoff [rad/s].
  // Omega_sp = R^T R_d Omega_d - K_R * e_R.
  // mc_rate_control owns normalized torque production.
  static bool body_rate(
    const State & state,
    const DesiredAttitudeDynamics & desired,
    const Vector3 & attitude_gain,
    Vector3 & rate_setpoint);

  // Physical Lee moment [N m] for inertia J [kg m^2] (Lee et al., 2010):
  // M = -K_R e_R - K_Omega e_Omega + Omega x J Omega + J alpha_ff.
  // With R_d = R and Omega_d = 0 this is pure rate damping plus the
  // gyroscopic term, which the runtime uses as its neutral moment.
  static bool physical(
    const State & state,
    const DesiredAttitudeDynamics & desired,
    const matrix::Matrix3f & inertia,
    const PhysicalAttitudeGains & gains,
    PhysicalMomentOutput & output);

  // Direct PX4-normalized body torque command.
  static bool normalized(
    const State & state,
    const DesiredAttitudeDynamics & desired,
    const NormalizedAttitudeGains & gains,
    NormalizedTorqueOutput & output);

};

}  // namespace se3_control
