// Controller-independent trajectory reference in PX4 local NED.
//
// This structure describes the commanded translational motion and heading.
// Any native controller can use the derivatives required by its control law.
//
// Units:
//   Position       m
//   Velocity       m/s
//   Acceleration   m/s^2
//   Jerk           m/s^3
//   Snap           m/s^4
//   Heading        rad
//   Heading rate   rad/s
//   Heading accel. rad/s^2

#pragma once

#include <array>

namespace trajectory_generator
{

using Vector3 = std::array<double, 3>;

struct Reference
{
  Vector3 position{};
  Vector3 velocity{};
  Vector3 acceleration{};
  Vector3 jerk{};
  Vector3 snap{};

  double yaw{0.0};
  double yaw_rate{0.0};
  double yaw_acceleration{0.0};
};

}  // namespace trajectory_generator
