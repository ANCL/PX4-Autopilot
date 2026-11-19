// Geometric SE(3) controller.
//
// Mathematical calculations are independent of Commander, uORB,
// scheduling, trajectory generation, and motor allocation.

#pragma once

#include "trajectory/Trajectory.hpp"

#include "Types.hpp"

namespace se3_control
{

class Controller
{
public:
  // Advance the bounded position-error integral by dt [s] (see Controller.cpp).
  //
  // freeze stops integration on every axis (output saturation). Returns false
  // for invalid inputs, leaving the integral unchanged.
  static bool update_integral(
    const State & state,
    const trajectory_generator::Reference & reference,
    const TranslationGains & gains,
    float dt,
    bool freeze,
    PositionIntegral & integral);

  // Compute translation feedback, integral and feedforward.
  //
  // Returns false for invalid state, reference, or parameters.
  // On failure, output remains unchanged.
  static bool compute_translation(
    const State & state,
    const trajectory_generator::Reference & reference,
    const TranslationGains & gains,
    const PositionIntegral & integral,
    TranslationOutput & output);

  // Calculate analytic desired-force derivatives using the rigid-body model.
  //
  // The translation output must come from compute_translation() with the
  // same vehicle state, trajectory reference, and controller gains.
  //
  // Returns false for invalid inputs or non-finite calculations.
  // On failure, output remains unchanged.
  static bool compute_force_derivatives(
    const State & state,
    const trajectory_generator::Reference & reference,
    const TranslationGains & gains,
    const PositionIntegral & integral,
    const TranslationOutput & translation,
    ForceDerivatives & output);

  // Calculate the complete geometric reference used by the attitude laws.
  //
  // The function does not select a control law or normalize motor commands.
  // On failure, output remains unchanged.
  static bool compute_geometry(
    const State & state,
    const trajectory_generator::Reference & reference,
    const TranslationGains & gains,
    const PositionIntegral & integral,
    GeometricOutput & output);
};

}  // namespace se3_control
