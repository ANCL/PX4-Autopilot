// Analytic position and yaw references for the native SE3 controller.
//
// A trajectory is a short sequence of segments in absolute PX4 local NED. Each
// segment starts at the pose where the previous one ended (the plan origin for
// the first). Every reference has analytic derivatives through snap.
//
// Trajectory math has no dependency on SE3 gains, PX4 flight modes or uORB.
#pragma once

#include <cstddef>
#include <cstdint>

namespace trajectory_generator
{

struct Vector3
{
    float values[3]{};
    float &operator[](unsigned index) { return values[index]; }
    const float &operator[](unsigned index) const { return values[index]; }
};

struct Reference
{
    Vector3 position{};
    Vector3 velocity{};
    Vector3 acceleration{};
    Vector3 jerk{};
    Vector3 snap{};
    float yaw{0.0f};
    float yaw_rate{0.0f};
    float yaw_acceleration{0.0f};
};

// Hold and Step are stationary (a step jumps by offset at its start). Line
// and Yaw move by offset / yaw_offset_rad from rest to rest with a C4 time
// scale. Circle and FigureEight run at a constant phase rate, so `turns`
// take exactly duration_s; their velocity steps at entry and exit.
struct Segment
{
    enum class Type : uint8_t { Hold, Step, Line, Circle, Yaw, FigureEight };
    Type type{Type::Hold};
    float duration_s{0.0f};
    Vector3 offset{};
    float radius_m{0.0f};
    float turns{0.0f};
    float yaw_offset_rad{0.0f};
    float x_amplitude_m{0.0f};
    float y_amplitude_m{0.0f};
};

// Longest plan: steps5 (one hold, four steps and the return home).
constexpr std::size_t kMaxSegments = 8;

struct Sequence
{
    Segment segments[kMaxSegments]{};
    std::size_t count{0};
};

enum class TrajectoryId : int { Setpoint, Steps5, Circle, Figure8 };

// Plan identifier for the measured-pose hold (mode entry and `hold`).
constexpr int kHoldId = -1;

struct Trajectory
{
    int id{kHoldId};
    Reference origin{};       // Start pose of the first segment.
    Sequence sequence{};
    float duration_s{0.0f};   // After it, the final pose is held.
    uint32_t checksum{0};     // CRC32 of the plan, for onboard logs.
};

// Parameter snapshot taken at command time. TRJ_SP_* are also read live by
// the runtime while the setpoint trajectory is active.
struct Settings
{
    float sp_x{}, sp_y{}, sp_z{}, sp_yaw{};
    float step_hold{};
    float home_x{}, home_y{}, home_z{}, home_yaw{};
    float circle_radius{}, circle_duration{};
    int32_t circle_turns{};
    float figure_x{}, figure_y{}, figure_duration{};
    int32_t figure_turns{};
};

bool validate(const Sequence &sequence);

// Reference at time_s after the plan start. Before the start the first sample
// is returned; after the last segment its final pose is held at rest. At an
// exact boundary the ending segment owns the sample.
bool sample(const Sequence &sequence, float time_s, const Reference &origin, Reference &reference);

// Index of the segment active at time_s (count after the last segment).
std::size_t segment_index(const Sequence &sequence, float time_s);

std::size_t trajectory_count();
const char *trajectory_name(std::size_t index);
int trajectory_index(const char *name);

// Called once during SE3 initialization to expose TRJ_* in PX4/QGC.
void mark_parameters_used();

bool load_settings(Settings &settings);

// Parameter ranges required by trajectory index (no plan is built).
bool valid_settings(int index, const Settings &settings);

// Plan for trajectory index. setpoint and steps5 step to their first
// position immediately. circle and figure8 first move from the position and
// yaw of `start` (the current reference) with Yaw and Line segments from
// rest; only position and yaw are taken from it, so a moving reference is
// replaced with a velocity discontinuity.
bool build_trajectory(int index, const Settings &settings, const Reference &start, Trajectory &plan);

// Stationary plan at the given position and yaw (measured-pose hold).
bool build_hold(const Vector3 &position, float yaw, Trajectory &plan);

} // namespace trajectory_generator
