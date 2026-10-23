// Shared, fixed-capacity trajectory plans for PX4 controllers.
//
// A selected plan contains absolute PX4 local NED start position and yaw.
// The active controller snapshots the selected plan on an explicit run request.

#pragma once

#include <cstddef>

#include "Sampler.hpp"

namespace trajectory_generator
{

struct Plan
{
  char name[32]{}; // Assigned by the onboard selection command.
  Reference origin{};
  Sequence sequence{};
  float duration_s{0.0f};
};

// Text format version 2, produced by tools/trajectory_lib/compiler.py:
// PX4_TRAJ_V2
// x y z yaw
// segment_count
// type duration offset_x offset_y offset_z radius height turns yaw_offset x_amp y_amp
// ... repeated segment_count times. Unused fields are zero.
// CRC32 <8 hex digits> over every preceding line, including newlines.
// A failed load preserves the previous plan.
bool load_plan_file(const char *path, Plan &plan, const char *&error);

// Thread-safe, bounded storage shared by the trajectory CLI and any onboard
// controller linked to trajectory_generator. Snapshot uses a short mutex hold;
// parsing and sampling never occur while the store is locked.
bool store_plan(const Plan &plan);
bool snapshot_plan(Plan &plan, unsigned *revision = nullptr);

}  // namespace trajectory_generator
