// Four firmware-resident SE3 reference types in absolute PX4 local NED.
//
//   setpoint  TRJ_SP_* as an immediate step, held indefinitely (the runtime
//             follows live parameter edits).
//   steps5    Five fixed positions, each entered as an immediate step and held
//             TRJ_STEP_HOLD, then a Line back to TRJ_HOME_*.
//   circle    TRJ_CIR_TURNS turns of radius TRJ_CIR_R about TRJ_HOME_* in
//             exactly TRJ_CIR_T seconds, then a Line back to TRJ_HOME_*.
//   figure8   TRJ_F8_TURNS cycles of amplitudes TRJ_F8_AX/AY about TRJ_HOME_*
//             in exactly TRJ_F8_T seconds, ending at TRJ_HOME_*.
//
// circle and figure8 are entered from the current reference position with a
// Yaw rotation to TRJ_HOME_YAW (if needed) and a Line to the pattern start,
// both from rest to rest. The patterns themselves run at a constant phase
// rate, so their velocity steps where they start and end. Parameters are read
// at command time; only the live setpoint tracks edits.

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

// Exactly five absolute NED references; each is entered as a true step.
constexpr float kStepPositions[5][3] = {
    {0.0f, 0.0f, -1.2f},
    {1.0f, 0.0f, -1.2f},
    {1.0f, 1.0f, -1.2f},
    {0.0f, 1.0f, -1.2f},
    {0.0f, 0.0f, -0.9f},
};

// Nominal length of a stationary plan; the final pose is held after it.
constexpr float kStationaryDuration = 1.0f;

// Line and Yaw durations: distance / average rate, at least kMinMoveDuration.
// The C4 time scale peaks at 630/256 times the average rate (about 0.6 m/s
// and 0.6 rad/s here).
constexpr float kLineAverageSpeed = 0.25f;   // [m/s]
constexpr float kYawAverageRate = 0.25f;     // [rad/s]
constexpr float kMinMoveDuration = 2.0f;     // [s]
constexpr float kMinMoveDistance = 1.0e-3f;  // [m] or [rad]; shorter moves are omitted.

Vector3 position(float x, float y, float z)
{
    return {{x, y, z}};
}

Vector3 difference(const Vector3 &to, const Vector3 &from)
{
    return position(to[0] - from[0], to[1] - from[1], to[2] - from[2]);
}

float wrap_pi(float angle)
{
    return atan2f(sinf(angle), cosf(angle));
}

Segment &append(Trajectory &plan, Segment::Type type, float duration)
{
    Segment &segment = plan.sequence.segments[plan.sequence.count++];
    segment = Segment{};
    segment.type = type;
    segment.duration_s = duration;
    plan.duration_s += duration;
    return segment;
}

// Line by offset from rest to rest; omitted if negligible.
void append_line(Trajectory &plan, const Vector3 &offset)
{
    const float distance = sqrtf(offset[0] * offset[0] + offset[1] * offset[1] + offset[2] * offset[2]);

    if (distance >= kMinMoveDistance) {
        append(plan, Segment::Type::Line, fmaxf(kMinMoveDuration, distance / kLineAverageSpeed)).offset = offset;
    }
}

// Start at the current reference pose, turn to yaw, then move to target.
void append_entry(Trajectory &plan, const Reference &start, const Vector3 &target, float yaw)
{
    plan.origin.position = start.position;
    plan.origin.yaw = start.yaw;

    const float yaw_offset = wrap_pi(yaw - start.yaw);

    if (fabsf(yaw_offset) >= kMinMoveDistance) {
        append(plan, Segment::Type::Yaw, fmaxf(kMinMoveDuration, fabsf(yaw_offset) / kYawAverageRate))
        .yaw_offset_rad = yaw_offset;
    }

    append_line(plan, difference(target, start.position));
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

uint32_t fingerprint(const Trajectory &plan)
{
    uint32_t crc = 0xFFFFFFFFU;
    crc_word(crc, static_cast<uint32_t>(plan.id));

    for (int i = 0; i < 3; ++i) { crc_float(crc, plan.origin.position[i]); }

    crc_float(crc, plan.origin.yaw);
    crc_word(crc, static_cast<uint32_t>(plan.sequence.count));

    for (std::size_t i = 0; i < plan.sequence.count; ++i) {
        const Segment &s = plan.sequence.segments[i];
        crc_word(crc, static_cast<uint32_t>(s.type));
        crc_float(crc, s.duration_s);

        for (int axis = 0; axis < 3; ++axis) { crc_float(crc, s.offset[axis]); }

        crc_float(crc, s.radius_m);
        crc_float(crc, s.turns);
        crc_float(crc, s.yaw_offset_rad);
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
    if (index < 0 || static_cast<std::size_t>(index) >= kCount || !valid_settings(index, settings) ||
        !std::isfinite(start.position[0]) || !std::isfinite(start.position[1]) ||
        !std::isfinite(start.position[2]) || !std::isfinite(start.yaw)) {
        return false;
    }

    plan = Trajectory{};
    plan.id = index;
    const Vector3 home = position(settings.home_x, settings.home_y, settings.home_z);

    switch (static_cast<TrajectoryId>(index)) {
    case TrajectoryId::Setpoint:
        plan.origin.position = position(settings.sp_x, settings.sp_y, settings.sp_z);
        plan.origin.yaw = settings.sp_yaw;
        append(plan, Segment::Type::Hold, kStationaryDuration);
        break;

    case TrajectoryId::Steps5:
        plan.origin.position = position(kStepPositions[0][0], kStepPositions[0][1], kStepPositions[0][2]);
        plan.origin.yaw = settings.home_yaw;
        append(plan, Segment::Type::Hold, settings.step_hold);

        for (int i = 1; i < 5; ++i) {
            append(plan, Segment::Type::Step, settings.step_hold).offset = position(
                        kStepPositions[i][0] - kStepPositions[i - 1][0],
                        kStepPositions[i][1] - kStepPositions[i - 1][1],
                        kStepPositions[i][2] - kStepPositions[i - 1][2]);
        }

        append_line(plan, difference(home, position(kStepPositions[4][0], kStepPositions[4][1], kStepPositions[4][2])));
        break;

    case TrajectoryId::Circle: {
            // The circle sampler starts on the +X edge of its circle.
            const Vector3 entry = position(home[0] + settings.circle_radius, home[1], home[2]);
            append_entry(plan, start, entry, settings.home_yaw);
            Segment &circle = append(plan, Segment::Type::Circle, settings.circle_duration);
            circle.radius_m = settings.circle_radius;
            circle.turns = static_cast<float>(settings.circle_turns);
            append_line(plan, difference(home, entry));
            break;
        }

    case TrajectoryId::Figure8: {
            // The figure-eight sampler starts and ends at its centre.
            append_entry(plan, start, home, settings.home_yaw);
            Segment &figure = append(plan, Segment::Type::FigureEight, settings.figure_duration);
            figure.x_amplitude_m = settings.figure_x;
            figure.y_amplitude_m = settings.figure_y;
            figure.turns = static_cast<float>(settings.figure_turns);
            break;
        }
    }

    return finish(plan);
}

bool build_hold(const Vector3 &target, float yaw, Trajectory &plan)
{
    if (!std::isfinite(target[0]) || !std::isfinite(target[1]) || !std::isfinite(target[2]) ||
        !std::isfinite(yaw)) {
        return false;
    }

    plan = Trajectory{};
    plan.id = kHoldId;
    plan.origin.position = target;
    plan.origin.yaw = yaw;
    append(plan, Segment::Type::Hold, kStationaryDuration);
    return finish(plan);
}

} // namespace trajectory_generator
