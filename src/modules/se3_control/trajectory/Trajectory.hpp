// Analytic position and yaw references for the native SE3 controller.
//
// A trajectory is a short sequence of segments in absolute PX4 local NED.
// Each segment is a C4 Hermite polynomial in normalized time (position and
// yaw), plus an optional planar pattern (circle or figure-eight) driven by a
// Hermite phase. Every reference has analytic derivatives through snap, and
// segments join with continuous derivatives through snap unless a step is
// intended.
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

// Degree-9 polynomial on one segment, in normalized time u = t / T, written
// in the well-conditioned two-point Hermite form
//
//   p(u) = E(u) + (1 - u)^5 F(u),
//   E(u) = sum_m end[m] (u - 1)^m,  F(u) = sum_j correction[j] u^j  (m, j <= 4).
//
// E is the Taylor polynomial of the end state, so p and its first four
// derivatives equal it exactly at u = 1; F matches the start state at u = 0.
// Unlike power-basis coefficients, neither part cancels large terms near u = 1.
struct Hermite
{
    float end[5]{};
    float correction[5]{};
};

struct Segment
{
    // Planar pattern added to the x/y polynomials, as a function of the phase:
    //   Circle:      (a_x cos(phase), a_x sin(phase))
    //   FigureEight: (a_x sin(phase), a_y sin(2 phase))
    enum class Shape : uint8_t { None, Circle, FigureEight };

    float duration_s{0.0f};
    Hermite position[3]{};    // [m]; the pattern centre for a shaped segment.
    Hermite yaw{};            // [rad]
    Shape shape{Shape::None};
    Hermite phase{};          // [rad]
    float x_amplitude_m{0.0f}; // Circle radius or figure-eight x amplitude.
    float y_amplitude_m{0.0f};
};

// Longest plan: steps5 (five holds and the return home).
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

// Hermite polynomial on [0, T] from the start to the end state; start[m]
// and end[m] are value and time derivatives d^m/dt^m, m = 0..4.
Hermite hermite(const float start[5], const float end[5], float duration_s);

bool validate(const Sequence &sequence);

// Reference at time_s after the plan start. Before the start the first sample
// is returned; after the last segment its final pose is held at rest. At an
// exact boundary the ending segment owns the sample.
bool sample(const Sequence &sequence, float time_s, Reference &reference);

// Index of the segment active at time_s (count after the last segment).
std::size_t segment_index(const Sequence &sequence, float time_s);

// Reference of one segment at time_s in [0, duration_s] (clamped).
bool sample_segment(const Segment &segment, float time_s, Reference &reference);

std::size_t trajectory_count();
const char *trajectory_name(std::size_t index);
int trajectory_index(const char *name);

// Called once during SE3 initialization to expose TRJ_* in PX4/QGC.
void mark_parameters_used();

bool load_settings(Settings &settings);

// Parameter ranges required by trajectory index (no plan is built).
bool valid_settings(int index, const Settings &settings);

// Plan for trajectory index, starting from the current reference `start`
// (position, yaw and their derivatives). Continuous trajectories enter from
// it without a reference discontinuity; setpoint and steps5 step immediately.
bool build_trajectory(int index, const Settings &settings, const Reference &start, Trajectory &plan);

// Stationary plan at the given position and yaw (measured-pose hold).
bool build_hold(const Vector3 &position, float yaw, Trajectory &plan);

} // namespace trajectory_generator
