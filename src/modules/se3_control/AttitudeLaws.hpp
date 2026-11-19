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

  // Physical Lee moment [N m] for inertia J [kg m^2] (Lee et al., 2010),
  // with a geometric integral term e_I = int e_R dt [rad s] maintained by
  // the caller (zero for the pure Lee law):
  // M = -K_R e_R - K_Omega e_Omega - K_I e_I + Omega x J Omega + J alpha_ff.
  // With R_d = R, Omega_d = 0 and e_I = 0 this is pure rate damping plus the
  // gyroscopic term, which the runtime uses as its neutral moment.
  static bool physical(
    const State & state,
    const DesiredAttitudeDynamics & desired,
    const matrix::Matrix3f & inertia,
    const PhysicalAttitudeGains & gains,
    const Vector3 & attitude_integral,
    PhysicalMomentOutput & output);

};

}  // namespace se3_control
