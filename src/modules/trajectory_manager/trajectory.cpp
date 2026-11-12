// PX4 console interface to firmware-resident trajectory definitions.
// No SD card, compiled files, or MAVLink transfer client is involved.

#include <trajectory/Catalog.hpp>
#include <trajectory/Plan.hpp>

#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>

#include <pthread.h>
#include <cstring>

namespace
{

// A command can be invoked on a small NuttX shell stack. Reuse a static
// inspection buffer rather than allocating a full 32-segment Plan there.
pthread_mutex_t g_inspect_mutex = PTHREAD_MUTEX_INITIALIZER;
trajectory_generator::Plan g_inspect_plan{};

} // namespace

extern "C" __EXPORT int trajectory_main(int argc, char *argv[])
{
    if (argc == 2 && std::strcmp(argv[1], "list") == 0) {
        PX4_INFO("onboard trajectories:");
        for (std::size_t i = 0; i < trajectory_generator::builtin_plan_count(); ++i) {
            PX4_INFO("  %s", trajectory_generator::builtin_plan_name(i));
        }
        return PX4_OK;
    }

    if (argc == 3 && std::strcmp(argv[1], "show") == 0) {
        const int index = trajectory_generator::builtin_plan_index(argv[2]);
        if (index < 0) {
            PX4_ERR("unknown trajectory: %s", argv[2]);
            return PX4_ERROR;
        }

        pthread_mutex_lock(&g_inspect_mutex);
        const bool valid = trajectory_generator::build_builtin_plan(index, g_inspect_plan);
        if (valid) {
            const auto &plan = g_inspect_plan;
            PX4_INFO("%s: %u segments, %.2f s, plan CRC32 %08x", plan.name,
                     static_cast<unsigned>(plan.sequence.count),
                     static_cast<double>(plan.duration_s),
                     static_cast<unsigned>(plan.checksum));
            PX4_INFO("start NED [%.2f, %.2f, %.2f] yaw %.2f",
                     static_cast<double>(plan.origin.position[0]),
                     static_cast<double>(plan.origin.position[1]),
                     static_cast<double>(plan.origin.position[2]),
                     static_cast<double>(plan.origin.yaw));
        }
        pthread_mutex_unlock(&g_inspect_mutex);
        if (!valid) { PX4_ERR("invalid builtin trajectory"); }
        return valid ? PX4_OK : PX4_ERROR;
    }

    if (argc == 3 && std::strcmp(argv[1], "start") == 0) {
        if (trajectory_generator::builtin_plan_index(argv[2]) < 0) {
            PX4_ERR("unknown trajectory: %s", argv[2]);
            return PX4_ERROR;
        }

        if (!trajectory_generator::request_se3_start(argv[2])) {
            PX4_ERR("SE3 must be active and holding; start not requested");
            return PX4_ERROR;
        }

        PX4_INFO("start requested: %s", argv[2]);
        return PX4_OK;
    }

    PX4_INFO("usage: trajectory list | show <name> | start <name>");
    return PX4_ERROR;
}
