// Bounded trajectory plan and explicit SE3 execution handshake.
//
// Named plans are constructed in firmware; no file, SD card, or transfer
// service is involved. The controller consumes requests on its work queue.

#pragma once

#include <cstdint>

#include "Sampler.hpp"

namespace trajectory_generator
{

struct Plan
{
  char name[32]{};
  Reference origin{};
  Sequence sequence{};
  float duration_s{0.0f};
  uint32_t checksum{0}; // CRC32 over the effective plan values, for ULog identity.
};

// The SE3 controller grants permission only while selected and holding.
// Revoking permission discards any request not yet consumed.
void set_se3_start_allowed(bool allowed);

// Queue a builtin plan by name; never change the active flight mode.
// Returns false for an unknown name or when SE3 is not holding.
bool request_se3_start(const char *name);

// On the SE3 work queue, build and consume one requested plan.
// The running plan remains independent of future parameter changes.
bool consume_start_request(Plan &plan);

} // namespace trajectory_generator
