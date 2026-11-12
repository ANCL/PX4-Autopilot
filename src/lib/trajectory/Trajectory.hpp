// Analytic trajectory references and firmware-resident trajectory definitions.
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

// Configuration is snapshotted when a trajectory is commanded.
struct Settings
{
    float home_x{};
    float home_y{};
    float home_z{};
    float home_yaw{};
    float step_x{};
    float step_y{};
    float line_x{};
    float line_y{};
    float line_dz{};
    float circle_r05{};
    float circle_r10{};
    float helix_radius{};
    float helix_height{};
    float figure_x{};
    float figure_y{};
    float yaw_sweep{};
    float hover_s{};
    float hold_s{};
    float step_s{};
    float line_s{};
    float return_s{};
    float circle05_s{};
    float circle10_s{};
    float figure_s{};
    float helix_s{};
    float yaw_s{};
    int32_t turns{};
 };

bool validate(const Sequence &sequence);
bool sample(const Sequence &sequence, float time_s,
            const Reference &origin, Reference &reference);

int trajectory_index(const char *name);
bool load_settings(Settings &settings);
bool build_trajectory(int index, const Settings &settings, Trajectory &trajectory);

} // namespace trajectory_generator
