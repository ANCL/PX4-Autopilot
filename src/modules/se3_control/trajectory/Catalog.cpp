// Four firmware-resident SE3 reference types in absolute PX4 local NED.
// Parameters are read at command time; only the live setpoint tracks edits.

#include "Trajectory.hpp"

#include <parameters/param.h>

#include <cmath>
#include <cstdint>
#include <cstring>

namespace trajectory_generator
{
namespace
{
constexpr const char *kNames[] = {"setpoint", "steps5", "circle", "figure8"};
constexpr std::size_t kCount = sizeof(kNames) / sizeof(kNames[0]);
constexpr float kLiveDuration = 86400.0f; // SE3 never auto-completes live setpoint.

// Exactly five absolute NED references, including the starting/ending hover.
// A step has zero commanded transition duration; hold time is configurable.
constexpr float kStepPositions[5][3] = {
    { 0.00f, 0.00f, -0.75f},
    { 0.25f, 0.00f, -0.75f},
    { 0.00f, 0.00f, -0.75f},
    { 0.00f, 0.25f, -0.75f},
    { 0.00f, 0.00f, -0.75f},
};

Vector3 position(float x, float y, float z)
{
    return {{x, y, z}};
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
        crc_float(crc, s.height_m);
        crc_float(crc, s.turns);
        crc_float(crc, s.yaw_offset_rad);
        crc_float(crc, s.x_amplitude_m);
        crc_float(crc, s.y_amplitude_m);
    }
    return crc ^ 0xFFFFFFFFU;
}

bool read_float(const char *name, float &value)
{
    const param_t p = param_find(name);
    return p != PARAM_INVALID && param_get(p, &value) == 0 && std::isfinite(value);
}

bool read_int(const char *name, int32_t &value)
{
    const param_t p = param_find(name);
    return p != PARAM_INVALID && param_get(p, &value) == 0;
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

void mark_parameters_used()
{
    // param_find marks these parameters used for the standard PX4/QGC UI.
    // This is in the existing SE3 initialization, not a separate boot command.
    constexpr const char *names[] = {
        "TRJ_SP_X", "TRJ_SP_Y", "TRJ_SP_Z", "TRJ_SP_YAW", "TRJ_STEP_HOLD",
        "TRJ_HOME_X", "TRJ_HOME_Y", "TRJ_HOME_Z", "TRJ_HOME_YAW",
        "TRJ_CIR_R", "TRJ_CIR_T", "TRJ_CIR_TURNS",
        "TRJ_F8_AX", "TRJ_F8_AY", "TRJ_F8_T", "TRJ_F8_TURNS"
    };
    for (const char *name : names) { (void)param_find(name); }
}

bool load_settings(Settings &settings)
{
    Settings next{};
    if (!read_float("TRJ_SP_X", next.sp_x) ||
        !read_float("TRJ_SP_Y", next.sp_y) ||
        !read_float("TRJ_SP_Z", next.sp_z) ||
        !read_float("TRJ_SP_YAW", next.sp_yaw) ||
        !read_float("TRJ_STEP_HOLD", next.step_hold) ||
        !read_float("TRJ_HOME_X", next.home_x) ||
        !read_float("TRJ_HOME_Y", next.home_y) ||
        !read_float("TRJ_HOME_Z", next.home_z) ||
        !read_float("TRJ_HOME_YAW", next.home_yaw) ||
        !read_float("TRJ_CIR_R", next.circle_radius) ||
        !read_float("TRJ_CIR_T", next.circle_duration) ||
        !read_int("TRJ_CIR_TURNS", next.circle_turns) ||
        !read_float("TRJ_F8_AX", next.figure_x) ||
        !read_float("TRJ_F8_AY", next.figure_y) ||
        !read_float("TRJ_F8_T", next.figure_duration) ||
        !read_int("TRJ_F8_TURNS", next.figure_turns)) {
        return false;
    }
    settings = next;
    return true;
}

bool build_trajectory(int index, const Settings &settings, Trajectory &plan)
{
    if (index < 0 || static_cast<std::size_t>(index) >= kCount) {
        return false;
    }
    plan = Trajectory{};
    plan.id = index;

    switch (static_cast<TrajectoryId>(index)) {
    case TrajectoryId::Setpoint:
        plan.origin.position = position(settings.sp_x, settings.sp_y, settings.sp_z);
        plan.origin.yaw = settings.sp_yaw;
        append(plan, Segment::Type::Hold, kLiveDuration);
        break;

    case TrajectoryId::Steps5: {
        if (!std::isfinite(settings.step_hold) || settings.step_hold <= 0.0f) {
            return false;
        }
        plan.origin.position = position(kStepPositions[0][0], kStepPositions[0][1], kStepPositions[0][2]);
        plan.origin.yaw = 0.0f;
        append(plan, Segment::Type::Hold, settings.step_hold);
        for (int i = 1; i < 5; ++i) {
            Segment &step = append(plan, Segment::Type::Step, settings.step_hold);
            step.offset = position(
                kStepPositions[i][0] - kStepPositions[i - 1][0],
                kStepPositions[i][1] - kStepPositions[i - 1][1],
                kStepPositions[i][2] - kStepPositions[i - 1][2]);
        }
        break;
    }

    case TrajectoryId::Circle: {
        if (!(settings.circle_radius > 0.0f) || !(settings.circle_duration > 0.0f) ||
            settings.circle_turns < 1 || settings.circle_turns > 10) { return false; }
        // The circle sampler starts on the +X edge of the circle.
        plan.origin.position = position(settings.home_x + settings.circle_radius,
                                        settings.home_y, settings.home_z);
        plan.origin.yaw = settings.home_yaw;
        Segment &circle = append(plan, Segment::Type::Circle, settings.circle_duration);
        circle.radius_m = settings.circle_radius;
        circle.turns = static_cast<float>(settings.circle_turns);
        break;
    }

    case TrajectoryId::Figure8: {
        if (!(settings.figure_x > 0.0f) || !(settings.figure_y > 0.0f) ||
            !(settings.figure_duration > 0.0f) ||
            settings.figure_turns < 1 || settings.figure_turns > 10) { return false; }
        plan.origin.position = position(settings.home_x, settings.home_y, settings.home_z);
        plan.origin.yaw = settings.home_yaw;
        Segment &figure = append(plan, Segment::Type::FigureEight, settings.figure_duration);
        figure.x_amplitude_m = settings.figure_x;
        figure.y_amplitude_m = settings.figure_y;
        figure.turns = static_cast<float>(settings.figure_turns);
        break;
    }
    }

    // No file I/O, heap allocation or second work queue.
    if (!validate(plan.sequence)) { return false; }
    Reference beginning{}, ending{};
    if (!sample(plan.sequence, 0.0f, plan.origin, beginning) ||
        !sample(plan.sequence, plan.duration_s + 1.0f, plan.origin, ending)) {
        return false;
    }
    for (int axis = 0; axis < 3; ++axis) {
        if (std::fabs(beginning.position[axis] - ending.position[axis]) > 0.01f) {
            return false;
        }
    }
    plan.checksum = fingerprint(plan);
    return true;
}

} // namespace trajectory_generator
