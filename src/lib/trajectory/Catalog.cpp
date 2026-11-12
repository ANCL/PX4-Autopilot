// Fixed onboard trajectory definitions, shared by the command and SE3.
//
// Offboard ROS 2 retains its own YAML configuration. These native defaults
// need no microSD plan files and can later be overridden by PX4 parameters.
// Each preset starts and finishes at [0, 0, -1] local NED and yaw zero.

#include "Catalog.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>

namespace trajectory_generator
{
namespace
{

constexpr char const *kNames[] = {
  "hover", "step_x", "step_y", "line_x", "line_y", "line_z",
  "circle_r05", "circle_r10", "figure_eight_tracking", "helix_r10",
  "yaw_sweep", "full_excitation"
};
constexpr std::size_t kCount = sizeof(kNames) / sizeof(kNames[0]);
constexpr float kHomeZ = -1.0f;
constexpr float kYaw90 = 1.5707963267948966f;

Vector3 position(float x, float y, float z)
{
  return {{x, y, z}};
}

Segment &append(Plan &plan, Segment::Type type, float duration_s)
{
  Segment &segment = plan.sequence.segments[plan.sequence.count++];
  segment = Segment{};
  segment.type = type;
  segment.duration_s = duration_s;
  plan.duration_s += duration_s;
  return segment;
}

void hold(Plan &plan, float duration_s)
{
  append(plan, Segment::Type::Hold, duration_s);
}

void move_to(Plan &plan, Segment::Type type, float duration_s,
             const Vector3 &target, Vector3 &cursor)
{
  Segment &segment = append(plan, type, duration_s);
  segment.offset = position(target[0] - cursor[0], target[1] - cursor[1],
                            target[2] - cursor[2]);
  cursor = target;
}

void line(Plan &plan, float seconds, const Vector3 &target, Vector3 &cursor)
{
  move_to(plan, Segment::Type::Line, seconds, target, cursor);
}

void step(Plan &plan, float seconds, const Vector3 &target, Vector3 &cursor)
{
  move_to(plan, Segment::Type::Step, seconds, target, cursor);
}

void yaw_to(Plan &plan, float seconds, float target, float &heading)
{
  Segment &segment = append(plan, Segment::Type::Yaw, seconds);
  segment.yaw_offset_rad = target - heading;
  heading = target;
}

void circle(Plan &plan, float seconds, float radius, float turns)
{
  Segment &segment = append(plan, Segment::Type::Circle, seconds);
  segment.radius_m = radius;
  segment.turns = turns;
}

void helix(Plan &plan, float seconds, float radius, float turns, float height)
{
  Segment &segment = append(plan, Segment::Type::Helix, seconds);
  segment.radius_m = radius;
  segment.turns = turns;
  segment.height_m = height;
}

void figure_eight(Plan &plan)
{
  Segment &segment = append(plan, Segment::Type::FigureEight, 60.0f);
  segment.x_amplitude_m = 0.75f;
  segment.y_amplitude_m = 1.2f;
  segment.turns = 3.0f;
}

void return_home(Plan &plan, Vector3 &cursor)
{
  line(plan, 5.0f, position(0.0f, 0.0f, kHomeZ), cursor);
}

// CRC32 covers explicitly serialized effective fields, never padding or files.
void crc_byte(uint32_t &crc, uint8_t byte)
{
  crc ^= byte;
  for (int bit = 0; bit < 8; ++bit) {
    crc = (crc >> 1) ^ ((crc & 1U) ? 0xEDB88320U : 0U);
  }
}

void crc_word(uint32_t &crc, uint32_t word)
{
  for (int byte = 0; byte < 4; ++byte) {
    crc_byte(crc, static_cast<uint8_t>(word >> (8 * byte)));
  }
}

void crc_float(uint32_t &crc, float value)
{
  static_assert(sizeof(float) == sizeof(uint32_t), "requires 32-bit float");
  uint32_t bits{};
  std::memcpy(&bits, &value, sizeof(bits));
  crc_word(crc, bits);
}

uint32_t fingerprint(const Plan &plan)
{
  uint32_t crc = 0xFFFFFFFFU;
  for (const char *p = plan.name; *p; ++p) {
    crc_byte(crc, static_cast<uint8_t>(*p));
  }
  crc_byte(crc, 0);
  for (int i = 0; i < 3; ++i) {
    crc_float(crc, plan.origin.position[i]);
  }
  crc_float(crc, plan.origin.yaw);
  crc_word(crc, static_cast<uint32_t>(plan.sequence.count));

  for (std::size_t i = 0; i < plan.sequence.count; ++i) {
    const Segment &s = plan.sequence.segments[i];
    crc_word(crc, static_cast<uint32_t>(s.type));
    crc_float(crc, s.duration_s);
    for (int j = 0; j < 3; ++j) {
      crc_float(crc, s.offset[j]);
    }
    crc_float(crc, s.radius_m);
    crc_float(crc, s.height_m);
    crc_float(crc, s.turns);
    crc_float(crc, s.yaw_offset_rad);
    crc_float(crc, s.x_amplitude_m);
    crc_float(crc, s.y_amplitude_m);
  }
  return crc ^ 0xFFFFFFFFU;
}

bool closed(const Plan &plan)
{
  Reference beginning{};
  Reference ending{};
  if (!sample(plan.sequence, 0.0f, plan.origin, beginning) ||
      !sample(plan.sequence, plan.duration_s + 1.0f, plan.origin, ending)) {
    return false;
  }
  for (int i = 0; i < 3; ++i) {
    if (std::fabs(beginning.position[i] - plan.origin.position[i]) > 0.01f ||
        std::fabs(ending.position[i] - plan.origin.position[i]) > 0.01f) {
      return false;
    }
  }
  const float yaw_error = ending.yaw - plan.origin.yaw;
  return std::fabs(std::atan2(std::sin(yaw_error), std::cos(yaw_error))) <= 0.01f;
}

} // namespace

std::size_t builtin_plan_count()
{
  return kCount;
}

const char *builtin_plan_name(std::size_t index)
{
  return index < kCount ? kNames[index] : nullptr;
}

int builtin_plan_index(const char *name)
{
  if (name) {
    for (std::size_t i = 0; i < kCount; ++i) {
      if (std::strcmp(name, kNames[i]) == 0) {
        return static_cast<int>(i);
      }
    }
  }
  return -1;
}

bool build_builtin_plan(int index, Plan &plan)
{
  if (index < 0 || static_cast<std::size_t>(index) >= kCount) {
    return false;
  }

  plan.origin = {};
  plan.origin.position = position(0.0f, 0.0f, kHomeZ);
  plan.origin.yaw = 0.0f;
  plan.sequence.count = 0;
  plan.duration_s = 0.0f;
  plan.checksum = 0;
  std::strncpy(plan.name, kNames[index], sizeof(plan.name) - 1);
  plan.name[sizeof(plan.name) - 1] = '\0';

  Vector3 cursor = plan.origin.position;
  float heading = plan.origin.yaw;
  const Vector3 home = plan.origin.position;

  switch (index) {
  case 0: // hover
    hold(plan, 10.0f);
    break;
  case 1: // step_x
    hold(plan, 2.0f);
    step(plan, 4.0f, position(1.0f, 0.0f, kHomeZ), cursor);
    hold(plan, 2.0f);
    return_home(plan, cursor);
    hold(plan, 2.0f);
    break;
  case 2: // step_y
    hold(plan, 2.0f);
    step(plan, 4.0f, position(0.0f, 1.0f, kHomeZ), cursor);
    hold(plan, 2.0f);
    return_home(plan, cursor);
    hold(plan, 2.0f);
    break;
  case 3: // line_x
    hold(plan, 2.0f);
    line(plan, 4.0f, position(0.5f, 0.0f, kHomeZ), cursor);
    hold(plan, 2.0f);
    return_home(plan, cursor);
    hold(plan, 2.0f);
    break;
  case 4: // line_y
    hold(plan, 2.0f);
    line(plan, 4.0f, position(0.0f, 0.5f, kHomeZ), cursor);
    hold(plan, 2.0f);
    return_home(plan, cursor);
    hold(plan, 2.0f);
    break;
  case 5: // line_z
    hold(plan, 2.0f);
    line(plan, 4.0f, position(0.0f, 0.0f, -1.25f), cursor);
    hold(plan, 2.0f);
    line(plan, 4.0f, home, cursor);
    hold(plan, 2.0f);
    break;
  case 6: // circle_r05
  case 7: // circle_r10
  {
    const float radius = index == 6 ? 0.5f : 1.0f;
    hold(plan, 2.0f);
    line(plan, index == 6 ? 5.0f : 10.0f, position(radius, 0.0f, kHomeZ), cursor);
    circle(plan, index == 6 ? 12.0f : 20.0f, radius, 3.0f);
    return_home(plan, cursor);
    hold(plan, 2.0f);
    break;
  }
  case 8: // figure_eight_tracking
    hold(plan, 2.0f);
    figure_eight(plan);
    hold(plan, 2.0f);
    break;
  case 9: // helix_r10
    hold(plan, 2.0f);
    line(plan, 10.0f, position(1.0f, 0.0f, kHomeZ), cursor);
    helix(plan, 20.0f, 1.0f, 3.0f, -0.5f);
    cursor = position(1.0f, 0.0f, -1.5f);
    line(plan, 8.0f, home, cursor);
    hold(plan, 2.0f);
    break;
  case 10: // yaw_sweep
    hold(plan, 2.0f);
    yaw_to(plan, 4.0f, kYaw90, heading);
    hold(plan, 2.0f);
    yaw_to(plan, 4.0f, 0.0f, heading);
    hold(plan, 2.0f);
    break;
  case 11: // full_excitation
    hold(plan, 2.0f);
    step(plan, 4.0f, position(1.0f, 0.0f, kHomeZ), cursor);
    hold(plan, 2.0f);
    return_home(plan, cursor);
    step(plan, 4.0f, position(0.0f, 1.0f, kHomeZ), cursor);
    hold(plan, 2.0f);
    return_home(plan, cursor);
    line(plan, 4.0f, position(0.0f, 0.0f, -1.25f), cursor);
    hold(plan, 2.0f);
    line(plan, 4.0f, home, cursor);
    yaw_to(plan, 4.0f, kYaw90, heading);
    hold(plan, 1.0f);
    yaw_to(plan, 4.0f, 0.0f, heading);
    figure_eight(plan);
    hold(plan, 2.0f);
    break;
  default:
    return false;
  }

  if (!validate(plan.sequence) || !closed(plan)) {
    return false;
  }

  plan.checksum = fingerprint(plan);
  return true;
}

} // namespace trajectory_generator
