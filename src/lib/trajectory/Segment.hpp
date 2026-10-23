// Controller-independent trajectory segments and sequences.
//
// Segments describe motion relative to a reference pose in local NED.
// A sequence combines segments without dynamic memory allocation.
//
// The mathematical sampler determines how each segment is evaluated.

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "Reference.hpp"

namespace trajectory_generator
{

struct Segment
{
  enum class Type : uint8_t
  {
    Hold,
    Step,
    Line,
    Circle,
    Helix,
    Yaw,
    FigureEight
  };

  Type type{Type::Hold};
  double duration_s{0.0};

  // Translation in local NED [m].
  Vector3 offset{};

  // Circular and helical trajectory parameters.
  double radius_m{0.0};
  double height_m{0.0};
  double turns{0.0};

  // Heading and figure-eight parameters.
  double yaw_offset_rad{0.0};
  double x_amplitude_m{0.0};
  double y_amplitude_m{0.0};
};

// Fixed-capacity sequence for deterministic onboard execution.
// Only the first count segments are active.
constexpr std::size_t kMaxSegments = 32;

struct Sequence
{
  std::array<Segment, kMaxSegments> segments{};
  std::size_t count{0};
};

}  // namespace trajectory_generator
