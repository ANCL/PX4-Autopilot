// Firmware-resident named trajectories in absolute PX4 local NED.
// The analytic sampler stays independent of trajectory names and commands.

#pragma once

#include <cstddef>

#include "Plan.hpp"

namespace trajectory_generator
{

std::size_t builtin_plan_count();
const char *builtin_plan_name(std::size_t index);
int builtin_plan_index(const char *name); // -1 for unknown names.

// Constructs and validates the selected plan in caller-owned storage.
// No file access or dynamic allocation. Call outside the control update path.
bool build_builtin_plan(int index, Plan &plan);

} // namespace trajectory_generator
