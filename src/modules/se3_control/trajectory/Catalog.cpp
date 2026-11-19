// Four firmware-resident SE3 reference types in absolute PX4 local NED.
//
//   setpoint  TRJ_SP_* as an immediate step, held indefinitely (the runtime
//             follows live parameter edits).
//   steps5    Five fixed step references, each held TRJ_STEP_HOLD, then a
//             smooth return to TRJ_HOME_*.
//   circle    TRJ_CIR_TURNS turns of radius TRJ_CIR_R in TRJ_CIR_T seconds.
//   figure8   TRJ_F8_TURNS cycles of amplitudes TRJ_F8_AX/AY in TRJ_F8_T.
//
// Both patterns are centred on TRJ_HOME_* and run at a constant phase rate
// for exactly their configured duration. They are entered smoothly from the
// current reference: a C4 transition to the pattern at rest, then a phase
// ramp along the pattern up to the nominal rate. Completion reverses this
// (phase ramp-down, then a C4 transition home). Entry and return are not part
// of the configured duration.

#include "Trajectory.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>

namespace trajectory_generator
{
namespace
{
constexpr const char *kNames[] = {"setpoint", "steps5", "circle", "figure8"};
constexpr std::size_t kCount = sizeof(kNames) / sizeof(kNames[0]);
constexpr float kPi = 3.14159265358979323846f;

// Exactly five absolute NED step references; each is entered as a true step.
constexpr float kStepPositions[5][3] = {
    {0.0f, 0.0f, -1.2f},
    {1.0f, 0.0f, -1.2f},
    {1.0f, 1.0f, -1.2f},
    {0.0f, 1.0f, -1.2f},
    {0.0f, 0.0f, -0.9f},
};

// Nominal length of a stationary segment; the final pose is held after it.
constexpr float kStationaryDuration = 1.0f;

// Transition limits. A transition takes the shortest duration (from 2 s, on a
// 25 % grid, at most 30 s) whose speed, acceleration and yaw rate stay within
// these values, or within 110 % of those at its start if larger.
constexpr float kTransitionSpeed = 0.6f;        // [m/s]
constexpr float kTransitionAcceleration = 0.6f; // [m/s^2]
constexpr float kTransitionYawRate = 0.5f;      // [rad/s]
constexpr float kMinTransition = 2.0f;          // [s]
constexpr float kMaxTransition = 30.0f;         // [s]
constexpr int kTransitionChecks = 32;           // Samples per duration check.

// Phase ramps last at least 2 s and keep the tangential acceleration within
// kTransitionAcceleration. A ramp is the C4 Hermite phase from rest to rate w
// (or back) advancing w T_r / 2; its rate is then w times the C3 smooth step
// 35u^4 - 84u^5 + 70u^6 - 20u^7, whose slope peaks at 35/16, so the
// tangential acceleration peaks at (35/16) v / T_r for pattern speed v.
constexpr float kMinRamp = 2.0f;                // [s]
constexpr float kRampPeak = 35.0f / 16.0f;

Vector3 position(float x, float y, float z)
{
    return {{x, y, z}};
}

Hermite constant(float value)
{
    const float state[5] {value};
    return hermite(state, state, 1.0f);
}

// Phase advancing from phase at rate0 to phase + distance at rate1 in T.
Hermite phase_profile(float phase, float rate0, float distance, float rate1, float duration)
{
    const float start[5] {phase, rate0};
    const float end[5] {phase + distance, rate1};
    return hermite(start, end, duration);
}

float wrap_pi(float angle)
{
    return std::atan2(std::sin(angle), std::cos(angle));
}

Segment &append(Trajectory &plan, float duration)
{
    Segment &segment = plan.sequence.segments[plan.sequence.count++];
    segment = Segment{};
    segment.duration_s = duration;
    plan.duration_s += duration;
    return segment;
}

void append_stationary(Trajectory &plan, const Vector3 &target, float yaw, float duration)
{
    Segment &segment = append(plan, duration);

    for (int axis = 0; axis < 3; ++axis) {
        segment.position[axis] = constant(target[axis]);
    }

    segment.yaw = constant(yaw);
}

float norm(const Vector3 &vector)
{
    return std::sqrt(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]);
}

// Hermite channels of a transition from the full start state to rest at target.
void transition_segment(const Reference &start, const Vector3 &target, float yaw, float duration,
                        Segment &segment)
{
    segment = Segment{};
    segment.duration_s = duration;

    for (int axis = 0; axis < 3; ++axis) {
        const float from[5] {start.position[axis], start.velocity[axis], start.acceleration[axis],
                             start.jerk[axis], start.snap[axis]};
        float to[5] {};
        to[0] = target[axis];
        segment.position[axis] = hermite(from, to, duration);
    }

    const float from_yaw[5] {start.yaw, start.yaw_rate, start.yaw_acceleration, 0.0f, 0.0f};
    float to_yaw[5] {};
    to_yaw[0] = yaw;
    segment.yaw = hermite(from_yaw, to_yaw, duration);
}

bool within_limits(const Segment &segment, float speed, float acceleration, float yaw_rate)
{
    for (int i = 1; i < kTransitionChecks; ++i) {
        Reference reference{};

        if (!sample_segment(segment, segment.duration_s * static_cast<float>(i) / kTransitionChecks, reference)) {
            return false;
        }

        if (norm(reference.velocity) > speed || norm(reference.acceleration) > acceleration ||
            std::fabs(reference.yaw_rate) > yaw_rate) {
            return false;
        }
    }

    return true;
}

// C4 transition from the current reference to rest at target. Nothing is
// appended if the reference is already there at rest.
void append_transition(Trajectory &plan, const Reference &start, const Vector3 &target, float yaw)
{
    const Vector3 offset = position(target[0] - start.position[0], target[1] - start.position[1],
                                    target[2] - start.position[2]);
    const float motion = norm(start.velocity) + norm(start.acceleration) + norm(start.jerk) + norm(start.snap) +
                         std::fabs(start.yaw_rate) + std::fabs(start.yaw_acceleration);

    if (norm(offset) < 1.0e-3f && std::fabs(yaw - start.yaw) < 1.0e-3f && motion < 1.0e-4f) {
        return;
    }

    const float speed = std::fmax(kTransitionSpeed, 1.1f * norm(start.velocity));
    const float acceleration = std::fmax(kTransitionAcceleration, 1.1f * norm(start.acceleration));
    const float yaw_rate = std::fmax(kTransitionYawRate, 1.1f * std::fabs(start.yaw_rate));

    Segment &segment = append(plan, 0.0f);
    float duration = kMinTransition;

    for (;;) {
        transition_segment(start, target, yaw, duration, segment);

        if (duration >= kMaxTransition || within_limits(segment, speed, acceleration, yaw_rate)) {
            break;
        }

        duration = std::fmin(1.25f * duration, kMaxTransition);
    }

    plan.duration_s += duration;
}

// Pattern segment centred at `centre` with the given phase.
void append_pattern(Trajectory &plan, Segment::Shape shape, const Vector3 &centre, float yaw,
                    float x_amplitude, float y_amplitude, const Hermite &phase, float duration)
{
    Segment &segment = append(plan, duration);

    for (int axis = 0; axis < 3; ++axis) {
        segment.position[axis] = constant(centre[axis]);
    }

    segment.yaw = constant(yaw);
    segment.shape = shape;
    segment.phase = phase;
    segment.x_amplitude_m = x_amplitude;
    segment.y_amplitude_m = y_amplitude;
}

// Pattern point at the given phase (see Segment::Shape).
Vector3 pattern_point(Segment::Shape shape, const Vector3 &centre, float x_amplitude, float y_amplitude,
                      float phase)
{
    if (shape == Segment::Shape::Circle) {
        return position(centre[0] + x_amplitude * std::cos(phase), centre[1] + x_amplitude * std::sin(phase),
                        centre[2]);
    }

    return position(centre[0] + x_amplitude * std::sin(phase), centre[1] + y_amplitude * std::sin(2.0f * phase),
                    centre[2]);
}

// Entry, constant-rate pattern and return for a circle or figure-eight.
//
// The pattern proper covers phase [0, 2 pi n] at rate w = 2 pi n / T. A ramp
// of duration T_r changes the rate between 0 and w along the pattern and
// advances the phase by w T_r / 2, so the ramp-up starts at -w T_r / 2 and the
// ramp-down ends at 2 pi n + w T_r / 2. The phase is C4 at every join.
void build_pattern(Trajectory &plan, const Reference &start, Segment::Shape shape, const Settings &settings,
                   float x_amplitude, float y_amplitude, int32_t turns, float duration)
{
    const Vector3 centre = position(settings.home_x, settings.home_y, settings.home_z);
    const float yaw = start.yaw + wrap_pi(settings.home_yaw - start.yaw);
    const float total_phase = 2.0f * kPi * static_cast<float>(turns);
    const float rate = total_phase / duration;

    // Peak speed: r w on the circle; at most w sqrt(a_x^2 + 4 a_y^2) on the figure-eight.
    const float peak_speed = shape == Segment::Shape::Circle ? x_amplitude * rate :
                             rate * std::sqrt(x_amplitude * x_amplitude + 4.0f * y_amplitude * y_amplitude);
    const float ramp = std::fmax(kMinRamp, kRampPeak * peak_speed / kTransitionAcceleration);
    const float ramp_phase = 0.5f * rate * ramp;

    append_transition(plan, start, pattern_point(shape, centre, x_amplitude, y_amplitude, -ramp_phase), yaw);
    append_pattern(plan, shape, centre, yaw, x_amplitude, y_amplitude,
                   phase_profile(-ramp_phase, 0.0f, ramp_phase, rate, ramp), ramp);
    append_pattern(plan, shape, centre, yaw, x_amplitude, y_amplitude,
                   phase_profile(0.0f, rate, total_phase, rate, duration), duration);
    append_pattern(plan, shape, centre, yaw, x_amplitude, y_amplitude,
                   phase_profile(total_phase, rate, ramp_phase, 0.0f, ramp), ramp);

    // Return home from rest at the end of the ramp-down.
    Reference stop{};
    stop.position = pattern_point(shape, centre, x_amplitude, y_amplitude, total_phase + ramp_phase);
    stop.yaw = yaw;
    append_transition(plan, stop, centre, yaw);
}

// The checksum identifies the effective trajectory in onboard logs.
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
    uint32_t bits{};
    static_assert(sizeof(bits) == sizeof(value), "requires 32-bit float");
    std::memcpy(&bits, &value, sizeof(bits));
    crc_word(crc, bits);
}

void crc_hermite(uint32_t &crc, const Hermite &hermite)
{
    for (int k = 0; k < 5; ++k) {
        crc_float(crc, hermite.end[k]);
        crc_float(crc, hermite.correction[k]);
    }
}

uint32_t fingerprint(const Trajectory &plan)
{
    uint32_t crc = 0xFFFFFFFFU;
    crc_word(crc, static_cast<uint32_t>(plan.id));
    crc_word(crc, static_cast<uint32_t>(plan.sequence.count));

    for (std::size_t i = 0; i < plan.sequence.count; ++i) {
        const Segment &s = plan.sequence.segments[i];
        crc_float(crc, s.duration_s);

        for (const Hermite &axis : s.position) { crc_hermite(crc, axis); }

        crc_hermite(crc, s.yaw);
        crc_word(crc, static_cast<uint32_t>(s.shape));
        crc_hermite(crc, s.phase);
        crc_float(crc, s.x_amplitude_m);
        crc_float(crc, s.y_amplitude_m);
    }

    return crc ^ 0xFFFFFFFFU;
}

bool finish(Trajectory &plan)
{
    if (!validate(plan.sequence)) { return false; }

    plan.checksum = fingerprint(plan);
    return true;
}

} // namespace

std::size_t trajectory_count()
{
    return kCount;
}

const char *trajectory_name(std::size_t index)
{
    return index < kCount ? kNames[index] : nullptr;
}

int trajectory_index(const char *name)
{
    if (name) {
        for (std::size_t i = 0; i < kCount; ++i) {
            if (std::strcmp(name, kNames[i]) == 0) { return static_cast<int>(i); }
        }
    }

    return -1;
}

bool valid_settings(int index, const Settings &s)
{
    const bool home = std::isfinite(s.home_x) && std::isfinite(s.home_y) && std::isfinite(s.home_z) &&
                      std::isfinite(s.home_yaw);

    switch (static_cast<TrajectoryId>(index)) {
    case TrajectoryId::Setpoint:
        return std::isfinite(s.sp_x) && std::isfinite(s.sp_y) && std::isfinite(s.sp_z) && std::isfinite(s.sp_yaw);

    case TrajectoryId::Steps5:
        return home && std::isfinite(s.step_hold) && s.step_hold >= 0.1f;

    case TrajectoryId::Circle:
        return home && s.circle_radius > 0.0f && s.circle_duration >= 0.1f &&
               s.circle_turns >= 1 && s.circle_turns <= 10;

    case TrajectoryId::Figure8:
        return home && s.figure_x > 0.0f && s.figure_y > 0.0f && s.figure_duration >= 0.1f &&
               s.figure_turns >= 1 && s.figure_turns <= 10;
    }

    return false;
}

bool build_trajectory(int index, const Settings &settings, const Reference &start, Trajectory &plan)
{
    if (index < 0 || static_cast<std::size_t>(index) >= kCount || !valid_settings(index, settings)) {
        return false;
    }

    plan = Trajectory{};
    plan.id = index;

    switch (static_cast<TrajectoryId>(index)) {
    case TrajectoryId::Setpoint:
        append_stationary(plan, position(settings.sp_x, settings.sp_y, settings.sp_z), settings.sp_yaw,
                          kStationaryDuration);
        break;

    case TrajectoryId::Steps5: {
        const float yaw = start.yaw + wrap_pi(settings.home_yaw - start.yaw);

        for (const auto &step : kStepPositions) {
            append_stationary(plan, position(step[0], step[1], step[2]), yaw, settings.step_hold);
        }

        Reference last{};
        last.position = position(kStepPositions[4][0], kStepPositions[4][1], kStepPositions[4][2]);
        last.yaw = yaw;
        append_transition(plan, last, position(settings.home_x, settings.home_y, settings.home_z), yaw);
        break;
    }

    case TrajectoryId::Circle:
        build_pattern(plan, start, Segment::Shape::Circle, settings, settings.circle_radius, 0.0f,
                      settings.circle_turns, settings.circle_duration);
        break;

    case TrajectoryId::Figure8:
        build_pattern(plan, start, Segment::Shape::FigureEight, settings, settings.figure_x, settings.figure_y,
                      settings.figure_turns, settings.figure_duration);
        break;
    }

    return finish(plan);
}

bool build_hold(const Vector3 &target, float yaw, Trajectory &plan)
{
    plan = Trajectory{};
    plan.id = kHoldId;
    append_stationary(plan, target, yaw, kStationaryDuration);
    return finish(plan);
}

} // namespace trajectory_generator
