// Analytic trajectory sampling interface.
//
// Sampling produces the same reference fields as the toolkit generator.
// The implementation must preserve the existing segment mathematics,
// interpolation rules, and sequence-boundary behavior.
//
// Invalid configurations are reported explicitly instead of throwing
// exceptions. The sampler does not depend on PX4 or ROS 2.

#pragma once

#include "Reference.hpp"
#include "Segment.hpp"

namespace trajectory_generator
{

// Validate all active segments before using a sequence.
bool validate(const Sequence & sequence);

// Evaluate the complete sequence at the requested elapsed time.
//
// The origin is the configured reference pose, not automatically the
// measured vehicle position. This preserves absolute NED trajectories.
//
// On failure, reference is left unchanged.
// After completion, the final position and yaw are held.
bool sample(
  const Sequence & sequence,
  double time_s,
  const Reference & origin,
  Reference & reference);

}  // namespace trajectory_generator
