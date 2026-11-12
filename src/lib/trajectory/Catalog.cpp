// Named trajectories in PX4 local NED. Parameters determine geometry and timing.

#include "Trajectory.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <parameters/param.h>

namespace trajectory_generator
{
namespace
{

constexpr char const *kNames[] = {
  "hover", "step_x", "step_y", "line_x", "line_y", "line_z",
  "circle_r05", "circle_r10", "figure_eight_tracking", "helix_r10",
  "yaw_sweep", "full_excitation"
};
enum class TrajectoryId : int {
  Hover,
  StepX,
  StepY,
  LineX,
  LineY,
  LineZ,
  Circle05,
  Circle10,
  FigureEight,
  Helix,
  Yaw,
  FullExcitation
};

constexpr std::size_t kCount = sizeof(kNames) / sizeof(kNames[0]);
static_assert(kCount == 12, "trajectory names must match TrajectoryId");
static_assert(kMaxSegments >= 15, "increase sequence capacity for the largest preset");

Vector3 position(float x, float y, float z)
{
  return {{x, y, z}};
}

Segment &append(Trajectory &plan, Segment::Type type, float duration_s)
{
  Segment &segment = plan.sequence.segments[plan.sequence.count++];
  segment = Segment{};
  segment.type = type;
  segment.duration_s = duration_s;
  plan.duration_s += duration_s;
  return segment;
}

void hold(Trajectory &plan, float duration_s)
{
  append(plan, Segment::Type::Hold, duration_s);
}

void move_to(Trajectory &plan, Segment::Type type, float duration_s,
             const Vector3 &target, Vector3 &cursor)
{
  Segment &segment = append(plan, type, duration_s);
  segment.offset = position(target[0] - cursor[0], target[1] - cursor[1],
                            target[2] - cursor[2]);
  cursor = target;
}

void line(Trajectory &plan, float seconds, const Vector3 &target, Vector3 &cursor)
{
  move_to(plan, Segment::Type::Line, seconds, target, cursor);
}

void step(Trajectory &plan, float seconds, const Vector3 &target, Vector3 &cursor)
{
  move_to(plan, Segment::Type::Step, seconds, target, cursor);
}

void yaw_to(Trajectory &plan, float seconds, float target, float &heading)
{
  Segment &segment = append(plan, Segment::Type::Yaw, seconds);
  segment.yaw_offset_rad = target - heading;
  heading = target;
}

void circle(Trajectory &plan, float seconds, float radius, float turns)
{
  Segment &segment = append(plan, Segment::Type::Circle, seconds);
  segment.radius_m = radius;
  segment.turns = turns;
}

void helix(Trajectory &plan, float seconds, float radius, float turns, float height)
{
  Segment &segment = append(plan, Segment::Type::Helix, seconds);
  segment.radius_m = radius;
  segment.turns = turns;
  segment.height_m = height;
}

void figure_eight(Trajectory &plan, const Settings &settings)
{
  Segment &segment = append(plan, Segment::Type::FigureEight, settings.figure_s);
  segment.x_amplitude_m = settings.figure_x;
  segment.y_amplitude_m = settings.figure_y;
  segment.turns = static_cast<float>(settings.turns);
}

void return_home(Trajectory &plan, Vector3 &cursor, const Settings &settings)
{
  line(plan, settings.return_s, plan.origin.position, cursor);
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

uint32_t fingerprint(const Trajectory &plan)
{
  uint32_t crc = 0xFFFFFFFFU;
  crc_word(crc, static_cast<uint32_t>(plan.id));
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

bool closed(const Trajectory &plan)
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

bool load_settings(Settings &settings)
{
  Settings candidate{};
  struct Field { const char *name; float *value; };
  const Field fields[] = {
    {"TRJ_HOME_X", &candidate.home_x},
    {"TRJ_HOME_Y", &candidate.home_y},
    {"TRJ_HOME_Z", &candidate.home_z},
    {"TRJ_HOME_YAW", &candidate.home_yaw},
    {"TRJ_STEP_X", &candidate.step_x},
    {"TRJ_STEP_Y", &candidate.step_y},
    {"TRJ_LINE_X", &candidate.line_x},
    {"TRJ_LINE_Y", &candidate.line_y},
    {"TRJ_LINE_DZ", &candidate.line_dz},
    {"TRJ_CIR05_R", &candidate.circle_r05},
    {"TRJ_CIR10_R", &candidate.circle_r10},
    {"TRJ_HELIX_R", &candidate.helix_radius},
    {"TRJ_HELIX_H", &candidate.helix_height},
    {"TRJ_F8_AX", &candidate.figure_x},
    {"TRJ_F8_AY", &candidate.figure_y},
    {"TRJ_YAW_ANG", &candidate.yaw_sweep},
    {"TRJ_HOVER_T", &candidate.hover_s},
    {"TRJ_HOLD_T", &candidate.hold_s},
    {"TRJ_STEP_T", &candidate.step_s},
    {"TRJ_LINE_T", &candidate.line_s},
    {"TRJ_RETURN_T", &candidate.return_s},
    {"TRJ_CIR05_T", &candidate.circle05_s},
    {"TRJ_CIR10_T", &candidate.circle10_s},
    {"TRJ_F8_T", &candidate.figure_s},
    {"TRJ_HELIX_T", &candidate.helix_s},
    {"TRJ_YAW_T", &candidate.yaw_s},
  };

  for (const Field &field : fields) {
    const param_t handle = param_find(field.name);
    if (handle == PARAM_INVALID || param_get(handle, field.value) != 0 ||
        !std::isfinite(*field.value)) {
      return false;
    }
  }

  const param_t turns_param = param_find("TRJ_TURNS");
  if (turns_param == PARAM_INVALID ||
      param_get(turns_param, &candidate.turns) != 0 ||
      candidate.turns < 1 || candidate.turns > 10) {
    return false;
  }

  settings = candidate;
  return true;
}

int trajectory_index(const char *name)
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

bool build_trajectory(int index, const Settings &settings, Trajectory &plan)
{
  if (index < 0 || static_cast<std::size_t>(index) >= kCount) {
    return false;
  }

  const float durations[] = {settings.hover_s, settings.hold_s, settings.step_s,
                             settings.line_s, settings.return_s, settings.circle05_s,
                             settings.circle10_s, settings.figure_s,
                             settings.helix_s, settings.yaw_s};
  for (const float value : durations) {
    if (!std::isfinite(value) || value < 0.01f) {
      return false;
    }
  }
  if (settings.turns < 1 || settings.turns > 10 ||
      !(settings.step_x > 0.0f && settings.step_y > 0.0f &&
        settings.line_x > 0.0f && settings.line_y > 0.0f &&
        settings.circle_r05 > 0.0f && settings.circle_r10 > 0.0f &&
        settings.helix_radius > 0.0f && settings.figure_x > 0.0f &&
        settings.figure_y > 0.0f)) {
    return false;
  }

  plan.origin = {};
  plan.origin.position = position(settings.home_x, settings.home_y, settings.home_z);
  plan.origin.yaw = settings.home_yaw;
  plan.sequence.count = 0;
  plan.duration_s = 0.0f;
  plan.checksum = 0;
  plan.id = index;

  Vector3 cursor = plan.origin.position;
  float heading = plan.origin.yaw;
  const Vector3 home = plan.origin.position;
  const float x = settings.home_x;
  const float y = settings.home_y;
  const float z = settings.home_z;
  const float turns = static_cast<float>(settings.turns);

  switch (static_cast<TrajectoryId>(index)) {
  case TrajectoryId::Hover:
    hold(plan, settings.hover_s);
    break;
  case TrajectoryId::StepX:
    hold(plan, settings.hold_s);
    step(plan, settings.step_s, position(x + settings.step_x, y, z), cursor);
    hold(plan, settings.hold_s);
    return_home(plan, cursor, settings);
    hold(plan, settings.hold_s);
    break;
  case TrajectoryId::StepY:
    hold(plan, settings.hold_s);
    step(plan, settings.step_s, position(x, y + settings.step_y, z), cursor);
    hold(plan, settings.hold_s);
    return_home(plan, cursor, settings);
    hold(plan, settings.hold_s);
    break;
  case TrajectoryId::LineX:
    hold(plan, settings.hold_s);
    line(plan, settings.line_s, position(x + settings.line_x, y, z), cursor);
    hold(plan, settings.hold_s);
    return_home(plan, cursor, settings);
    hold(plan, settings.hold_s);
    break;
  case TrajectoryId::LineY:
    hold(plan, settings.hold_s);
    line(plan, settings.line_s, position(x, y + settings.line_y, z), cursor);
    hold(plan, settings.hold_s);
    return_home(plan, cursor, settings);
    hold(plan, settings.hold_s);
    break;
  case TrajectoryId::LineZ:
    hold(plan, settings.hold_s);
    line(plan, settings.line_s, position(x, y, z + settings.line_dz), cursor);
    hold(plan, settings.hold_s);
    line(plan, settings.line_s, home, cursor);
    hold(plan, settings.hold_s);
    break;
  case TrajectoryId::Circle05:
  case TrajectoryId::Circle10:
  {
    const float radius = index == static_cast<int>(TrajectoryId::Circle05) ? settings.circle_r05 : settings.circle_r10;
    hold(plan, settings.hold_s);
    line(plan, index == static_cast<int>(TrajectoryId::Circle05) ? 5.0f : 10.0f, position(x + radius, y, z), cursor);
    circle(plan, index == static_cast<int>(TrajectoryId::Circle05) ? settings.circle05_s : settings.circle10_s, radius, turns);
    return_home(plan, cursor, settings);
    hold(plan, settings.hold_s);
    break;
  }
  case TrajectoryId::FigureEight:
    hold(plan, settings.hold_s);
    figure_eight(plan, settings);
    hold(plan, settings.hold_s);
    break;
  case TrajectoryId::Helix:
    hold(plan, settings.hold_s);
    line(plan, 10.0f, position(x + settings.helix_radius, y, z), cursor);
    helix(plan, settings.helix_s, settings.helix_radius, turns, settings.helix_height);
    cursor = position(x + settings.helix_radius, y, z + settings.helix_height);
    line(plan, 8.0f, home, cursor);
    hold(plan, settings.hold_s);
    break;
  case TrajectoryId::Yaw:
    hold(plan, settings.hold_s);
    yaw_to(plan, settings.yaw_s, settings.home_yaw + settings.yaw_sweep, heading);
    hold(plan, settings.hold_s);
    yaw_to(plan, settings.yaw_s, settings.home_yaw, heading);
    hold(plan, settings.hold_s);
    break;
  case TrajectoryId::FullExcitation:
    hold(plan, settings.hold_s);
    step(plan, settings.step_s, position(x + settings.step_x, y, z), cursor);
    hold(plan, settings.hold_s);
    return_home(plan, cursor, settings);
    step(plan, settings.step_s, position(x, y + settings.step_y, z), cursor);
    hold(plan, settings.hold_s);
    return_home(plan, cursor, settings);
    line(plan, settings.line_s, position(x, y, z + settings.line_dz), cursor);
    hold(plan, settings.hold_s);
    line(plan, settings.line_s, home, cursor);
    yaw_to(plan, settings.yaw_s, settings.home_yaw + settings.yaw_sweep, heading);
    hold(plan, 1.0f);
    yaw_to(plan, settings.yaw_s, settings.home_yaw, heading);
    figure_eight(plan, settings);
    hold(plan, settings.hold_s);
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
