// Analytic position and yaw references for the native SE3 controller.
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

struct Segment
{
    enum class Type : uint8_t { Hold, Step, Line, Circle, Helix, Yaw, FigureEight };
    Type type{Type::Hold};
    float duration_s{0.0f};
    Vector3 offset{};
    float radius_m{0.0f};
    float height_m{0.0f};
    float turns{0.0f};
    float yaw_offset_rad{0.0f};
    float x_amplitude_m{0.0f};
    float y_amplitude_m{0.0f};
};

constexpr std::size_t kMaxSegments = 32;

struct Sequence
{
    Segment segments[kMaxSegments]{};
    std::size_t count{0};
};

struct Trajectory
{
    int id{-1};
    Reference origin{};
    Sequence sequence{};
    float duration_s{0.0f};
    uint32_t checksum{0};
};

// Setpoint values are read live by SE3; other values are snapshotted at start.
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

enum class TrajectoryId : int { Setpoint, Steps5, Circle, Figure8 };

bool validate(const Sequence &sequence);
bool sample(const Sequence &sequence, float time_s,
            const Reference &origin, Reference &reference);

std::size_t trajectory_count();
const char *trajectory_name(std::size_t index);
int trajectory_index(const char *name);

// Called once during SE3 initialization to expose TRJ_* in PX4/QGC.
void mark_parameters_used();

bool load_settings(Settings &settings);
bool build_trajectory(int index, const Settings &settings, Trajectory &plan);

} // namespace trajectory_generator
