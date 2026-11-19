// TRJ_* parameter access for the trajectory catalog (Catalog.cpp).

#include "Trajectory.hpp"

#include <parameters/param.h>

#include <cmath>

namespace trajectory_generator
{
namespace
{

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

void mark_parameters_used()
{
    // param_find marks these parameters used for the standard PX4/QGC UI.
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

} // namespace trajectory_generator
