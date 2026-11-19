// Geometric SE(3) control of an airborne multicopter as one registered
// Commander flight mode. Run() computes the position/yaw reference, hold,
// trajectory and geometric attitude reference at 100 Hz for every control
// path; SE3_CTRL (fixed until reboot, see ControlPath.hpp) selects what is
// commanded from it:
//
//   RATE_PX4     geometric body rate + normalized thrust -> mc_rate_control
//                -> control_allocator (native).
//   ATTITUDE_PX4 geometric attitude + normalized thrust (vehicle_attitude_
//                setpoint) -> mc_att_control -> mc_rate_control -> native
//                allocation; SE3 publishes no rate setpoint.
//   PHYSICAL     physical Lee wrench from a gyro-triggered inner loop
//                (run_inner(), physical/PhysicalPath) -> se3_wrench_setpoint,
//                allocated in rotor-thrust space by control_allocator, with
//                the PX4 rate loop disabled.
//
// The 100 Hz translation law, including its bounded position-error integral,
// is common to all three paths.
//
// Setpoints are published only while this registered flight mode is active
// with the controllers its path requires. Fault handling (Position-mode
// request or native PX4 failsafe) is described at enter_fault().

#include "AttitudeLaws.hpp"
#include "ControlPath.hpp"
#include "Controller.hpp"
#include "physical/PhysicalPath.hpp"

#include "trajectory/Trajectory.hpp"
#include <parameters/param.h>
#include <px4_platform_common/atomic.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <systemlib/mavlink_log.h>
#include <uORB/SubscriptionCallback.hpp>

#include <drivers/drv_hrt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/arming_check_reply.h>
#include <uORB/topics/arming_check_request.h>
#include <uORB/topics/control_allocator_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/register_ext_component_reply.h>
#include <uORB/topics/register_ext_component_request.h>
#include <uORB/topics/unregister_ext_component.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/se3_trajectory_status.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_status.h>

#include <matrix/matrix/math.hpp>

#include <cmath>
#include <pthread.h>
#include <cstdint>
#include <cstring>

using namespace time_literals;

namespace
{
constexpr char kModeName[] = "SE3";
constexpr float kGravity = 9.80665f;
constexpr hrt_abstime kPositionTimeout = 200_ms;
constexpr hrt_abstime kAttitudeTimeout = 200_ms;
constexpr hrt_abstime kGyroTimeout = 100_ms;

// Commander normally acknowledges a mode request within one 10 ms loop.
constexpr hrt_abstime kHandoffAckTimeout = 100_ms;

// Longest outer-loop interval integrated; a longer gap integrates nothing.
constexpr hrt_abstime kIntegralMaxInterval = 50_ms;

// Native allocation status older than this is not used for anti-windup.
constexpr hrt_abstime kAllocatorStatusTimeout = 100_ms;

using se3_control::ControlPath;

bool fresh(hrt_abstime now, hrt_abstime stamp, hrt_abstime limit)
{
    return stamp > 0 && now >= stamp && now - stamp <= limit;
}

// Navigation-state request handled by Commander like a user mode change:
// it is accepted only if the requested mode can run.
vehicle_command_s nav_state_command(uint8_t nav_state, const vehicle_status_s &status)
{
    vehicle_command_s command{};
    command.timestamp = hrt_absolute_time();
    command.command = vehicle_command_s::VEHICLE_CMD_SET_NAV_STATE;
    command.param1 = static_cast<float>(nav_state);
    command.source_system = status.system_id;
    command.source_component = status.component_id;
    command.target_system = status.system_id;
    command.target_component = status.component_id;
    return command;
}

// Fault classes. A controller fault (internal computation, trajectory,
// configuration, or estimator reference-frame reset with a valid estimate)
// requests Position mode. A state fault (invalid/stale estimate, landed,
// failsafe, disarmed) is left entirely to Commander's native failsafe.
enum class FaultClass : int32_t { None, Controller, State };
enum class FaultResponse : uint8_t { None, PositionRequested, PositionAccepted, NativeFailsafe };
} // namespace

class Se3Runtime final : public ModuleBase<Se3Runtime>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
    Se3Runtime() : ModuleParams(nullptr), ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl) {}

    ~Se3Runtime() override
    {
        // Unregister the flight mode when the controller shuts down.
        // Commander handles any required mode transition.
        if (_mode_id >= 0 && _arming_check_id >= 0) {
            unregister_ext_component_s unregister{};
            unregister.timestamp = hrt_absolute_time();
            std::strncpy(unregister.name, kModeName, sizeof(unregister.name) - 1);
            unregister.arming_check_id = _arming_check_id;
            unregister.mode_id = _mode_id;
            unregister.mode_executor_id = -1;
            _unregister_pub.publish(unregister);
        }

        delete _inner_gyro_sub; // Unregisters the callback before its work item goes.
        delete _inner;
    }

    bool init()
    {
        vehicle_status_s status{};

        if (_status_sub.copy(&status) && status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
            PX4_ERR("SE3 registration requires a disarmed vehicle");
            return false;
        }

        updateParams();
        trajectory_generator::mark_parameters_used();

        // The control path and physical model are fixed until reboot.
        if (!se3_control::control_path_from_parameter(_param_ctrl.get(), _path)) {
            PX4_ERR("SE3_CTRL %d is not a control path", (int)_param_ctrl.get());
            return false;
        }

        _physical = _path == ControlPath::Physical;

        if (_physical && !init_physical()) {
            return false;
        }

        PX4_INFO("control path %s (SE3_CTRL %d)", se3_control::control_path_name(_path), (int)_param_ctrl.get());

        _registration_request_id = hrt_absolute_time();

        register_ext_component_request_s request{};
        request.timestamp = _registration_request_id;
        request.request_id = _registration_request_id;
        request.px4_ros2_api_version = register_ext_component_request_s::LATEST_PX4_ROS2_API_VERSION;
        std::strncpy(request.name, kModeName, sizeof(request.name) - 1);
        request.register_arming_check = true;
        request.register_mode = true;
        request.register_mode_executor = false;
        request.enable_replace_internal_mode = false;
        request.activate_mode_immediately = false;
        _register_pub.publish(request);

        ScheduleOnInterval(10_ms); // SE(3) reference update at 100 Hz.
        return true;
    }

    int print_status() override
    {
        PX4_INFO("Commander mode id=%d arming check id=%d", (int)_mode_id, (int)_arming_check_id);
        PX4_INFO("enabled=%d active=%d faulted=%d", (int)(_param_enable.get() == 1),
                 (int)_active.load(), (int)_faulted);
        const int plan = _plan_id.load();
        PX4_INFO("trajectory: %s, %s", plan >= 0 ? trajectory_generator::trajectory_name(plan) : "measured-pose hold",
                 _executing.load() ? "running" : (_completed.load() ? "completed (holding home)" : "holding"));
        static constexpr const char *responses[] = {
            "none", "Position requested", "Position accepted", "PX4 failsafe"
        };
        PX4_INFO("fault response: %s", responses[static_cast<int>(_fault_response)]);
        PX4_INFO("position integral -K_i I: [%.3f, %.3f, %.3f] m/s^2",
                 (double)(-_param_kixy.get() * _integral.value(0)), (double)(-_param_kixy.get() * _integral.value(1)),
                 (double)(-_param_kiz.get() * _integral.value(2)));

        PX4_INFO("control: %s (SE3_CTRL %d)%s", se3_control::control_path_name(_path), (int)_param_ctrl.get(),
                 _native_rate_fallback ? ", fault fallback to PX4 rate control" : "");

        if (_physical) {
            _physical_path.print_status();
        }

        return 0;
    }

    static int task_spawn(int argc, char *argv[])
    {
        auto *instance = new Se3Runtime();

        if (instance) {
            _object.store(instance);
            _task_id = task_id_is_work_queue;

            if (instance->init()) {
                return PX4_OK;
            }
        }

        delete instance;
        _object.store(nullptr);
        _task_id = -1;
        return PX4_ERROR;
    }

    static int custom_command(int argc, char *argv[])
    {
        Se3Runtime *instance = get_instance();

        if (!instance) {
            PX4_ERR("start se3_control first");
            return PX4_ERROR;
        }

        if (argc == 1 && std::strcmp(argv[0], "select") == 0) {
            return instance->command_select();
        }

        if (argc == 1 && std::strcmp(argv[0], "hold") == 0) {
            return instance->command_hold();
        }

        if (argc == 1 && std::strcmp(argv[0], "list") == 0) {
            PX4_INFO("available trajectories:");
            for (std::size_t i = 0; i < trajectory_generator::trajectory_count(); ++i) {
                PX4_INFO("  %s", trajectory_generator::trajectory_name(i));
            }
            return PX4_OK;
        }

        if (argc == 2 && std::strcmp(argv[0], "trajectory") == 0) {
            return instance->command_trajectory(argv[1]);
        }

#if defined(CONFIG_ARCH_BOARD_PX4_SITL)
        if (argc == 2 && std::strcmp(argv[0], "fault") == 0) {
            return instance->command_fault(argv[1]);
        }
#endif

        return print_usage("unknown command");
    }

    static int print_usage(const char *reason = nullptr)
    {
        if (reason) { PX4_WARN("%s", reason); }
        PRINT_MODULE_DESCRIPTION(R"DESCR_STR(
### Description
Experimental SE(3) geometric controller for an already-flying multicopter.
Registered at boot on supported targets. Selection requires valid airborne state. No autonomous takeoff.
SE3_CTRL (fixed until reboot) selects the path: 0 ATTITUDE_PX4 (PX4 attitude and rate controllers),
1 RATE_PX4 (PX4 rate controller, default) or 2 PHYSICAL (physical wrench, SE3 allocation).
PX4 retains allocator, actuator output and flight-safety ownership.

Selecting SE3 holds the measured pose. While SE3 is active, `trajectory` starts a
trajectory immediately, preempting any running one, and `hold` holds the measured pose.
)DESCR_STR");
        PRINT_MODULE_USAGE_NAME("se3_control", "controller");
        PRINT_MODULE_USAGE_COMMAND("start");
        PRINT_MODULE_USAGE_COMMAND_DESCR("select", "Request the SE3 flight mode (holds the measured pose)");
        PRINT_MODULE_USAGE_COMMAND_DESCR("hold", "Cancel the trajectory and hold the measured pose");
        PRINT_MODULE_USAGE_COMMAND_DESCR("list", "List the trajectories");
        PRINT_MODULE_USAGE_COMMAND_DESCR("trajectory", "Start a trajectory now, preempting the current one");
        PRINT_MODULE_USAGE_ARG("setpoint|steps5|circle|figure8", "Trajectory", false);
#if defined(CONFIG_ARCH_BOARD_PX4_SITL)
        PRINT_MODULE_USAGE_COMMAND_DESCR("fault", "SITL test only: inject a fault (requires SYS_FAILURE_EN=1)");
        PRINT_MODULE_USAGE_ARG("controller|state|allocation", "Fault class (allocation: non-finite PHYSICAL wrench)", false);
#endif
        PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
        return 0;
    }

private:
    // CLI threads may request a transition, but only Run() modifies the
    // reference and trajectory clock on the controller work queue.
    int command_select()
    {
        // Commander owns mode transitions and can reject this request.
        const int mode_id = _registered_mode_id.load();
        if (mode_id < 0) {
            PX4_ERR("SE3 has not registered with Commander");
            return PX4_ERROR;
        }

        uORB::Subscription status_sub{ORB_ID(vehicle_status)};
        vehicle_status_s status{};
        if (!status_sub.copy(&status)) {
            PX4_ERR("vehicle status unavailable");
            return PX4_ERROR;
        }

        if (!_vehicle_command_pub.publish(nav_state_command(static_cast<uint8_t>(mode_id), status))) {
            PX4_ERR("mode request publication failed");
            return PX4_ERROR;
        }

        PX4_INFO("requested SE3 mode %d", mode_id);
        return PX4_OK;
    }

    int command_hold()
    {
        if (!_active.load()) {
            PX4_ERR("SE3 is not the active flight mode");
            return PX4_ERROR;
        }

        submit_request(trajectory_generator::kHoldId, trajectory_generator::Settings{});
        PX4_INFO("SE3 current-pose hold requested");
        return PX4_OK;
    }

#if defined(CONFIG_ARCH_BOARD_PX4_SITL)
    // SITL-only test hook, gated like PX4's `failure` command. The injected
    // fault enters exactly the response path of a detected fault of that class.
    int command_fault(const char *kind)
    {
        int32_t enabled = 0;

        if (param_get(param_find("SYS_FAILURE_EN"), &enabled) != PX4_OK || enabled != 1) {
            PX4_ERR("fault injection disabled by SYS_FAILURE_EN");
            return PX4_ERROR;
        }

        if (!_active.load()) {
            PX4_ERR("SE3 is not the active flight mode");
            return PX4_ERROR;
        }

        if (std::strcmp(kind, "controller") == 0) {
            _injected_fault.store(static_cast<int32_t>(FaultClass::Controller));

        } else if (std::strcmp(kind, "state") == 0) {
            _injected_fault.store(static_cast<int32_t>(FaultClass::State));

        } else if (std::strcmp(kind, "allocation") == 0 && _physical) {
            // Real allocator rejection path: non-finite wrenches until the
            // resulting fault is detected.
            _inject_invalid_wrench.store(true);

        } else {
            PX4_ERR("fault class must be controller, state or (PHYSICAL) allocation");
            return PX4_ERROR;
        }

        PX4_INFO("SE3 %s fault injection requested", kind);
        return PX4_OK;
    }
#endif

    void publish_arming_reply(uint8_t request_id, bool can_arm_and_run)
    {
        // Every reply carries the mode requirements; Commander applies them
        // from each reply, so the immediate fault reply must set them too.
        arming_check_reply_s reply{};
        reply.timestamp = hrt_absolute_time();
        reply.request_id = request_id;
        reply.registration_id = static_cast<uint8_t>(_arming_check_id);
        reply.can_arm_and_run = can_arm_and_run;
        reply.mode_req_angular_velocity = true;
        reply.mode_req_attitude = true;
        reply.mode_req_local_position = true;
        reply.mode_req_prevent_arming = true; // Airborne handoff only, no takeoff/arm.
        _arming_reply_pub.publish(reply);
    }

    // Latch a fault on the first failure while SE3 is selected.
    //
    // A controller fault requests Position mode; Commander accepts it only if
    // Position can run. A state fault, or a rejected/unanswered request, is
    // reported to Commander immediately (re-using the current request ID)
    // instead of at the next 300 ms health request, so its native failsafe
    // applies without delay. SE3 never reports itself healthy while faulted.
    //
    // PHYSICAL additionally re-enables the PX4 rate controller for this mode
    // at once, so torque production returns to mc_rate_control (tracking the
    // neutral rate setpoint) within a Commander cycle instead of depending on
    // the physical allocation that may have failed. The mode configuration
    // is restored on disarm.
    void enter_fault(FaultClass fault, const char *reason)
    {
        clear_request();
        _active.store(false);
        _executing.store(false);
        reset_integral();
        _faulted = true;
#if defined(CONFIG_ARCH_BOARD_PX4_SITL)
        _inject_invalid_wrench.store(false);
#endif

        if (_physical) {
            _physical_path.clear_reference();
            publish_mode_config(true);
        }

        if (fault == FaultClass::Controller) {
            PX4_WARN("SE3 fault: %s; requesting Position mode", reason);
            _position_request_time = hrt_absolute_time();
            _handoff_command_pub.publish(nav_state_command(vehicle_status_s::NAVIGATION_STATE_POSCTL, _status));
            _fault_response = FaultResponse::PositionRequested;

        } else {
            PX4_WARN("SE3 fault: %s; leaving to PX4 failsafe", reason);
            report_fault_to_commander();
        }

        continue_fault_handoff();
    }

    void report_fault_to_commander()
    {
        publish_arming_reply(_last_arming_request_id, false);
        _fault_response = FaultResponse::NativeFailsafe;
    }

    // Runs every cycle while SE3 is still selected after a fault.
    void continue_fault_handoff()
    {
        if (_fault_response == FaultResponse::PositionRequested) {
            vehicle_command_ack_s ack{};

            while (_command_ack_sub.update(&ack)) {
                if (ack.command == vehicle_command_s::VEHICLE_CMD_SET_NAV_STATE &&
                    ack.target_component == _status.component_id &&
                    ack.timestamp >= _position_request_time) {
                    if (ack.result == vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED) {
                        PX4_WARN("SE3 fault: Position mode accepted");
                        _fault_response = FaultResponse::PositionAccepted;

                    } else {
                        PX4_WARN("SE3 fault: Position mode rejected (%d); leaving to PX4 failsafe", (int)ack.result);
                        report_fault_to_commander();
                    }

                    break;
                }
            }

            if (_fault_response == FaultResponse::PositionRequested &&
                hrt_elapsed_time(&_position_request_time) > kHandoffAckTimeout) {
                PX4_WARN("SE3 fault: no Position mode reply; leaving to PX4 failsafe");
                report_fault_to_commander();
            }
        }

        publish_neutral_setpoint();
    }

    // Zero body rates and hover thrust replace the last SE3 command until
    // Commander deselects SE3; mc_rate_control otherwise keeps tracking the
    // last published setpoint. Rotation is stopped, not levelled. Nothing is
    // published unless airborne.
    //
    // ATTITUDE_PX4 instead replaces the last attitude setpoint with a level
    // attitude at the current heading and hover thrust for mc_att_control.
    void publish_neutral_setpoint()
    {
        if (!_landed_received || !flying()) {
            return;
        }

        const float hover = _param_hover.get();
        const bool hover_valid = std::isfinite(hover) && hover > 0.0f && hover < 1.0f;

        if (_path == ControlPath::AttitudePx4) {
            publish_neutral_attitude(hover_valid ? hover : _last_thrust);
            return;
        }

        vehicle_rates_setpoint_s setpoint{};
        setpoint.timestamp = hrt_absolute_time();
        setpoint.thrust_body[2] = -(hover_valid ? hover : _last_thrust);
        _rate_setpoint_pub.publish(setpoint);
    }

    void publish_neutral_attitude(float thrust)
    {
        const matrix::Quatf attitude{_attitude.q};
        float yaw = _last_yaw;

        if (attitude.isAllFinite() && fabsf(attitude.norm() - 1.0f) < 0.05f) {
            yaw = matrix::Eulerf{attitude}.psi();
        }

        vehicle_attitude_setpoint_s setpoint{};
        setpoint.timestamp = hrt_absolute_time();
        matrix::Quatf{matrix::Eulerf{0.0f, 0.0f, std::isfinite(yaw) ? yaw : 0.0f}}.copyTo(setpoint.q_d);
        setpoint.thrust_body[2] = -thrust;
        _attitude_setpoint_pub.publish(setpoint);
    }

    // Controllers of this mode, published at registration and when PHYSICAL
    // falls back to (or, on disarm, returns from) PX4 rate control.
    void publish_mode_config(bool native_rate_fallback)
    {
        const se3_control::ModeControllers controllers =
            se3_control::mode_controllers(_path, native_rate_fallback);

        vehicle_control_mode_s config{};
        config.timestamp = hrt_absolute_time();
        config.source_id = static_cast<uint8_t>(_mode_id);
        config.flag_control_attitude_enabled = controllers.attitude;
        config.flag_control_rates_enabled = controllers.rates;
        config.flag_control_allocation_enabled = controllers.allocation;
        _mode_control_config_pub.publish(config);
        _native_rate_fallback = native_rate_fallback;
    }

    // The shell supplies a named trajectory and a parameter snapshot. Any
    // command replaces a pending one; only the SE3 work queue changes the
    // active plan, so a command always preempts the running trajectory.
    int command_trajectory(const char *name)
    {
        const int index = trajectory_generator::trajectory_index(name);

        if (index < 0) {
            PX4_ERR("unknown trajectory: %s", name);
            return PX4_ERROR;
        }

        if (!_active.load()) {
            PX4_ERR("SE3 is not the active flight mode");
            return PX4_ERROR;
        }

        trajectory_generator::Settings settings{};

        if (!trajectory_generator::load_settings(settings) ||
            !trajectory_generator::valid_settings(index, settings)) {
            PX4_ERR("invalid trajectory parameters");
            return PX4_ERROR;
        }

        submit_request(index, settings);
        PX4_INFO("trajectory requested: %s", name);
        return PX4_OK;
    }

    void submit_request(int trajectory, const trajectory_generator::Settings &settings)
    {
        pthread_mutex_lock(&_request_mutex);
        _request.pending = true;
        _request.trajectory = trajectory;
        _request.settings = settings;
        pthread_mutex_unlock(&_request_mutex);
    }

    void clear_request()
    {
        pthread_mutex_lock(&_request_mutex);
        _request.pending = false;
        pthread_mutex_unlock(&_request_mutex);
    }

    // Stationary reference at the measured local pose (mode entry, `hold`).
    // Fails only for a non-finite estimate, which state_valid() excludes.
    bool capture_hold(hrt_abstime now)
    {
        if (!trajectory_generator::build_hold({{_position.x, _position.y, _position.z}}, _position.heading, _plan)) {
            return false;
        }

        _plan_start = now;
        _plan_id.store(_plan.id);
        _executing.store(false);
        _completed.store(false);
        return true;
    }

    // Reference of the active plan at time now; the live setpoint follows
    // TRJ_SP_* edits as immediate steps.
    bool current_reference(hrt_abstime now, trajectory_generator::Reference &reference) const
    {
        const float elapsed_s = static_cast<float>(now - _plan_start) * 1.0e-6f;

        if (!trajectory_generator::sample(_plan.sequence, elapsed_s, reference)) {
            return false;
        }

        if (_plan.id == static_cast<int>(trajectory_generator::TrajectoryId::Setpoint)) {
            reference = {};
            reference.position = {{_param_sp_x.get(), _param_sp_y.get(), _param_sp_z.get()}};
            reference.yaw = _param_sp_yaw.get();
        }

        return true;
    }

    // Apply the latest shell command at time now. A new trajectory starts
    // from the reference the current plan has at this instant, so
    // continuous entries have no reference discontinuity. False only if the
    // hold could not be captured.
    bool apply_request(hrt_abstime now)
    {
        pthread_mutex_lock(&_request_mutex);
        const Request request = _request;
        _request.pending = false;
        pthread_mutex_unlock(&_request_mutex);

        if (!request.pending) {
            return true;
        }

        if (request.trajectory == trajectory_generator::kHoldId) {
            return capture_hold(now);
        }

        trajectory_generator::Reference start{};

        if (!current_reference(now, start) ||
            !trajectory_generator::build_trajectory(request.trajectory, request.settings, start, _next_plan)) {
            PX4_ERR("trajectory %s could not be built; keeping the current reference",
                    trajectory_generator::trajectory_name(request.trajectory));
            return true;
        }

        _plan = _next_plan;
        _plan_start = now;
        _plan_id.store(_plan.id);
        // The live setpoint has no fixed plan, so no checksum identifies it.
        _plan_crc = _plan.id == static_cast<int>(trajectory_generator::TrajectoryId::Setpoint) ? 0 : _plan.checksum;
        ++_run_id;
        _executing.store(true);
        _completed.store(false);
        PX4_INFO("SE3 trajectory %s started (run %u, %.1f s)", trajectory_generator::trajectory_name(_plan.id),
                 static_cast<unsigned>(_run_id), static_cast<double>(_plan.duration_s));
        return true;
    }

    void reset_integral()
    {
        _integral = {};
        _integral_time = 0;
        _command_saturated = false;
    }

    // Anti-windup input for the translational integral: the previous normalized
    // command was clamped, or the allocator could not produce the collective or
    // roll/pitch command (native control_allocator_status, or the PHYSICAL
    // se3_allocation_status replies).
    bool output_saturated(hrt_abstime now)
    {
        if (_physical) {
            return _physical_path.force_saturated();
        }

        _allocator_status_sub.update(&_allocator_status);
        const bool roll_pitch_achieved = _allocator_status.unallocated_torque[0] * _allocator_status.unallocated_torque[0] +
                                         _allocator_status.unallocated_torque[1] * _allocator_status.unallocated_torque[1] < 1e-6f;
        const bool allocator_saturated = fresh(now, _allocator_status.timestamp, kAllocatorStatusTimeout) &&
                                         (!_allocator_status.thrust_setpoint_achieved || !roll_pitch_achieved);
        return _command_saturated || allocator_saturated;
    }

    void Run() override
    {
        if (should_exit()) {
            clear_request();
            ScheduleClear();

            if (_physical) {
                _inner_gyro_sub->unregisterCallback();
                _inner->clear();
            }

            exit_and_cleanup();
            return;
        }

        if (_parameter_update_sub.updated()) {
            parameter_update_s update{};
            _parameter_update_sub.copy(&update);
            updateParams();

            if (_physical) {
                update_vehicle();
            }
        }

        // Commander assigns both IDs. Matching the request ID prevents
        // accepting another component's registration reply.
        if (_mode_id < 0) {
            register_ext_component_reply_s response{};

            if (_register_reply_sub.update(&response) &&
                response.request_id == _registration_request_id &&
                std::strncmp(response.name, kModeName, sizeof(response.name)) == 0) {
                if (response.success && response.mode_id >= 0 && response.arming_check_id >= 0) {
                    _mode_id = response.mode_id;
                    _registered_mode_id.store(response.mode_id);
                    _arming_check_id = response.arming_check_id;

                    // RATE_PX4: PX4 rate loop and allocator. ATTITUDE_PX4:
                    // also the PX4 attitude loop. PHYSICAL: allocator only;
                    // its moment replaces the rate loop (as in Offboard
                    // thrust/torque). Commander derives multicopter-position
                    // control from the position/velocity/altitude flags,
                    // which all remain false.
                    publish_mode_config(false);
                    PX4_INFO("registered mode %d (arming check %d)", (int)_mode_id, (int)_arming_check_id);
                    mavlink_log_info(&_mavlink_log_pub, "SE3 registered: %s control\t",
                                     se3_control::control_path_name(_path));
                } else {
                    PX4_ERR("Commander mode registration rejected");
                }
            }
        }

        _status_sub.update(&_status);
        _control_mode_sub.update(&_control_mode);
        if (_landed_sub.update(&_landed)) { _landed_received = true; }
        _position_sub.update(&_position);
        _attitude_sub.update(&_attitude);
        _gyro_sub.update(&_gyro);

        const bool valid = ready();

        // Commander broadcasts health requests ~3 Hz. Reply even while
        // disabled; lack of a reply would cause a different timeout path.
        arming_check_request_s request{};
        if (_arming_check_id >= 0 && _arming_request_sub.update(&request)) {
            _last_arming_request_id = request.request_id;
            publish_arming_reply(request.request_id, valid);
        }

        // No publication at all while another flight mode owns the setpoint,
        // or while Commander's controllers are not the ones this path needs.
        se3_control::ControlModeFlags controllers{};
        controllers.attitude = _control_mode.flag_control_attitude_enabled;
        controllers.rates = _control_mode.flag_control_rates_enabled;
        controllers.multicopter_position = _control_mode.flag_multicopter_position_control_enabled;
        controllers.allocation = _control_mode.flag_control_allocation_enabled;
        const bool selected = _mode_id >= 0 && _status.nav_state == _mode_id &&
                              se3_control::controllers_match(_path, _faulted, controllers);

        if (!selected) {
            clear_request();
            _active.store(false);
            _executing.store(false);
            _completed.store(false);
            reset_integral();

            if (_physical) {
                _physical_path.clear_reference();
            }

            // Keep faults latched until disarm, including after a mode change.
            if (_status.arming_state == vehicle_status_s::ARMING_STATE_DISARMED) {
                _faulted = false;
                _fault_response = FaultResponse::None;

                if (_native_rate_fallback) {
                    publish_mode_config(false); // PHYSICAL configuration for the next flight.
                }
            }

            return;
        }

        // A latched fault is never cleared in flight: a recovered sample or a
        // briefly valid estimate does not resume control.
        if (_faulted) {
            continue_fault_handoff();
            return;
        }

#if defined(CONFIG_ARCH_BOARD_PX4_SITL)
        const int32_t injected = _injected_fault.load();

        if (injected != static_cast<int32_t>(FaultClass::None)) {
            _injected_fault.store(static_cast<int32_t>(FaultClass::None));
            enter_fault(static_cast<FaultClass>(injected), "injected for SITL testing");
            return;
        }
#endif

        if (!valid) {
            // Only invalid SE3 configuration with otherwise valid flight state
            // is a controller fault; everything else is left to PX4.
            const bool controller_fault = _landed_received && flying() && state_valid() && !configured();
            enter_fault(controller_fault ? FaultClass::Controller : FaultClass::State,
                        controller_fault ? "invalid SE3 configuration" : "vehicle state not valid for SE3");
            return;
        }

        if (!_active.load()) {
            // Mode entry is deliberately passive: capture the actual pose
            // and hold it until an explicit trajectory command arrives.
            // Commands issued before activation are discarded.
            clear_request();
            reset_integral();

            if (!capture_hold(hrt_absolute_time())) {
                enter_fault(FaultClass::State, "measured pose not valid for hold");
                return;
            }

            _xy_reset_counter = _position.xy_reset_counter;
            _z_reset_counter = _position.z_reset_counter;
            _quat_reset_counter = _attitude.quat_reset_counter;
            if (_physical) {
                _physical_path.start_monitoring(hrt_absolute_time());
            }

            _active.store(true);
        }

        // Estimator-frame discontinuities are not silently interpreted as
        // trajectory tracking errors. The estimate itself remains valid, so
        // this is a controller fault: PX4 Position control handles resets.
        if (_xy_reset_counter != _position.xy_reset_counter ||
            _z_reset_counter != _position.z_reset_counter ||
            _quat_reset_counter != _attitude.quat_reset_counter) {
            enter_fault(FaultClass::Controller, "estimator reset");
            return;
        }

        // PHYSICAL relies on control_allocator accepting its wrench with all
        // model rotors running; otherwise PX4 allocation takes over.
        const char *allocation_problem = _physical ? _physical_path.allocation_problem() : nullptr;

        if (allocation_problem != nullptr) {
            enter_fault(FaultClass::Controller, allocation_problem);
            return;
        }

        const hrt_abstime now = hrt_absolute_time();

        if (!apply_request(now)) {
            enter_fault(FaultClass::State, "measured pose not valid for hold");
            return;
        }

        const float elapsed_s = static_cast<float>(now - _plan_start) * 1.0e-6f;
        trajectory_generator::Reference reference{};

        if (!current_reference(now, reference)) {
            enter_fault(FaultClass::Controller, "trajectory sampling failed");
            return;
        }

        const bool live_setpoint = _plan.id == static_cast<int>(trajectory_generator::TrajectoryId::Setpoint);

        // Finite trajectories end at rest at home, which the sampler then holds.
        if (_executing.load() && !live_setpoint && elapsed_s >= _plan.duration_s) {
            _executing.store(false);
            _completed.store(true);
            PX4_INFO("SE3 trajectory %s completed; holding home", trajectory_generator::trajectory_name(_plan.id));
        }

        se3_control::State state{};
        state.position = se3_control::Vector3{_position.x, _position.y, _position.z};
        state.velocity = se3_control::Vector3{_position.vx, _position.vy, _position.vz};
        state.angular_velocity = se3_control::Vector3{_gyro.xyz[0], _gyro.xyz[1], _gyro.xyz[2]};

        const matrix::Quatf quat{_attitude.q};
        state.attitude = matrix::Dcmf{quat};

        // Mass-normalized translation law shared by all paths, with the
        // bounded position-error integral updated before it is applied.
        const se3_control::TranslationGains translation_gains = translation_gains_from_parameters();
        const bool integral_frozen = output_saturated(now);
        const float integral_dt = _integral_time > 0 && now > _integral_time &&
                                  now - _integral_time <= kIntegralMaxInterval ?
                                  static_cast<float>(now - _integral_time) * 1.0e-6f : 0.0f;
        _integral_time = now;

        if (!se3_control::Controller::update_integral(state, reference, translation_gains, integral_dt,
                integral_frozen, _integral)) {
            enter_fault(FaultClass::Controller, "position integral update failed");
            return;
        }

        se3_control::GeometricOutput geometric{};
        se3_control::Vector3 body_rate{};
        const se3_control::Vector3 attitude_gains{
            _param_krx.get(), _param_kry.get(), _param_krz.get()
        };

        // Only RATE_PX4 uses the SE3 body-rate law and its gains and limit;
        // the other paths command the geometric attitude (body rate stays 0).
        const bool rate_path = _path == ControlPath::RatePx4;

        if (!se3_control::Controller::compute_geometry(state, reference, translation_gains, _integral, geometric) ||
            (rate_path &&
             !se3_control::AttitudeLaws::body_rate(state, geometric.desired_attitude, attitude_gains, body_rate))) {
            enter_fault(FaultClass::Controller, "geometric control computation failed");
            return;
        }

        // Convert collective thrust to the normalized PX4 body-z convention.
        const float thrust = _param_hover.get() * geometric.collective_acceleration / kGravity;
        const float rate_limit = rate_path ? _param_rate_max.get() : 0.0f;

        if (!std::isfinite(thrust) || !body_rate.isAllFinite() || !std::isfinite(rate_limit)) {
            enter_fault(FaultClass::Controller, "non-finite rate or thrust command");
            return;
        }

        vehicle_rates_setpoint_s setpoint{};
        setpoint.timestamp = hrt_absolute_time();
        setpoint.roll = fminf(fmaxf(body_rate(0), -rate_limit), rate_limit);
        setpoint.pitch = fminf(fmaxf(body_rate(1), -rate_limit), rate_limit);
        setpoint.yaw = fminf(fmaxf(body_rate(2), -rate_limit), rate_limit);
        setpoint.thrust_body[0] = 0.f;
        setpoint.thrust_body[1] = 0.f;
        setpoint.thrust_body[2] = -fminf(fmaxf(thrust, 0.0f), 1.0f);

        // Clamped normalized commands freeze the integral next cycle. Only
        // the PX4 paths command them; PHYSICAL uses the allocation replies.
        _command_saturated = !_physical &&
                             (thrust > 1.0f || thrust < 0.0f ||
                              (rate_path && (fabsf(body_rate(0)) > rate_limit || fabsf(body_rate(1)) > rate_limit ||
                                             fabsf(body_rate(2)) > rate_limit)));

        if (_physical) {
            // The inner loop uses this reference until the next update. The
            // rate setpoint (zero rates) is ignored by mc_rate_control while
            // SE3 owns allocation; it is the neutral command for the first
            // PX4 rate-loop cycle after SE3 is deselected or falls back.
            _physical_path.set_reference(geometric.translation.specific_force, geometric.desired_attitude,
                                         setpoint.timestamp);
        }

        // ATTITUDE_PX4: mc_att_control alone publishes the rate setpoint.
        if (_path != ControlPath::AttitudePx4) {
            _rate_setpoint_pub.publish(setpoint);
        }

        _last_thrust = -setpoint.thrust_body[2];
        _last_yaw = reference.yaw;

        // The exact geometric attitude reference on the standard PX4 topic.
        // ATTITUDE_PX4: this is the command tracked by mc_att_control.
        // RATE_PX4 and PHYSICAL: an observation only; Commander keeps
        // mc_att_control disabled in this external flight mode.
        // Rotation maps body FRD to world NED, matching PX4 q_d convention.
        const matrix::Quatf desired_quaternion{geometric.desired_attitude.attitude};
        vehicle_attitude_setpoint_s attitude_sp{};
        attitude_sp.timestamp = setpoint.timestamp;
        for (int component = 0; component < 4; ++component) {
            attitude_sp.q_d[component] = desired_quaternion(component);
        }
        attitude_sp.yaw_sp_move_rate = reference.yaw_rate;
        for (int axis = 0; axis < 3; ++axis) {
            attitude_sp.thrust_body[axis] = setpoint.thrust_body[axis];
        }
        _attitude_setpoint_pub.publish(attitude_sp);

        // Record the exact reference sent to the geometric controller.
        se3_trajectory_status_s record{};
        record.timestamp = setpoint.timestamp;
        record.run_id = _run_id;
        record.plan_crc32 = _plan.id == trajectory_generator::kHoldId ? 0 : _plan_crc;
        record.trajectory = static_cast<int8_t>(_plan.id);
        // Stationary plans (hold, setpoint) stay in their only segment.
        const std::size_t segment = trajectory_generator::segment_index(_plan.sequence, elapsed_s);
        record.segment = static_cast<uint8_t>(_completed.load() ? _plan.sequence.count :
                                              (segment < _plan.sequence.count ? segment : _plan.sequence.count - 1));
        record.phase = _completed.load() ? se3_trajectory_status_s::PHASE_COMPLETED :
                       (_executing.load() ? se3_trajectory_status_s::PHASE_RUNNING :
                                            se3_trajectory_status_s::PHASE_HOLD);
        record.elapsed_s = _plan.id == trajectory_generator::kHoldId ? 0.0f :
                           (live_setpoint ? elapsed_s : fminf(elapsed_s, _plan.duration_s));
        for (int axis = 0; axis < 3; ++axis) {
            record.position[axis] = reference.position[axis];
            record.velocity[axis] = reference.velocity[axis];
            record.acceleration[axis] = reference.acceleration[axis];
            record.jerk[axis] = reference.jerk[axis];
            record.snap[axis] = reference.snap[axis];
        }
        record.yaw = reference.yaw;
        record.yaw_rate = reference.yaw_rate;
        record.yaw_acceleration = reference.yaw_acceleration;

        for (int axis = 0; axis < 3; ++axis) {
            record.integral_acceleration[axis] = geometric.translation.integral(axis);
        }

        record.integral_frozen = integral_frozen;
        // Publish the same reference used for SE3 control on PX4's standard
        // trajectory topic. The FlightModeManager has no flight task in an
        // external mode; mc_pos_control remains disabled by Commander.
        // No controller state, arm state, or trajectory lifecycle is changed.
        trajectory_setpoint_s trajectory_sp{};
        trajectory_sp.timestamp = record.timestamp;
        for (int axis = 0; axis < 3; ++axis) {
            trajectory_sp.position[axis] = reference.position[axis];
            trajectory_sp.velocity[axis] = reference.velocity[axis];
            trajectory_sp.acceleration[axis] = reference.acceleration[axis];
            trajectory_sp.jerk[axis] = reference.jerk[axis];
        }
        trajectory_sp.yaw = reference.yaw;
        trajectory_sp.yawspeed = reference.yaw_rate;
        _trajectory_pub.publish(trajectory_sp);

        _trajectory_status_pub.publish(record);
    }

    // PHYSICAL setup at boot: vehicle parameters and the gyro-triggered inner
    // loop on Run()'s work queue (same thread, so the inner loop shares state
    // with Run() without locking).
    bool init_physical()
    {
        _physical_path.init();
        update_vehicle();

        _inner = new InnerLoop(*this);
        _inner_gyro_sub = _inner ? new uORB::SubscriptionCallbackWorkItem(_inner, ORB_ID(vehicle_angular_velocity)) : nullptr;

        if (_inner_gyro_sub == nullptr || !_inner_gyro_sub->registerCallback()) {
            PX4_ERR("SE3 inner loop start failed");
            return false;
        }

        return true;
    }

    void update_vehicle()
    {
        _physical_path.set_vehicle(_param_mass.get(),
                                   se3_control::Vector3{_param_jxx.get(), _param_jyy.get(), _param_jzz.get()});
    }

    se3_control::TranslationGains translation_gains_from_parameters() const
    {
        se3_control::TranslationGains gains{};
        gains.position = _param_kx.get();
        gains.velocity = _param_kv.get();
        gains.integral = se3_control::Vector3{_param_kixy.get(), _param_kixy.get(), _param_kiz.get()};
        return gains;
    }

    se3_control::PhysicalAttitudeGains physical_gains() const
    {
        se3_control::PhysicalAttitudeGains gains{};
        gains.attitude = se3_control::Vector3{_param_pkrx.get(), _param_pkry.get(), _param_pkrz.get()};
        gains.angular_velocity = se3_control::Vector3{_param_pkwx.get(), _param_pkwy.get(), _param_pkwz.get()};
        gains.integral = se3_control::Vector3{_param_pki_xy.get(), _param_pki_xy.get(), _param_pki_z.get()};
        return gains;
    }

    // Gyro-triggered PHYSICAL inner loop: one wrench per new angular-velocity
    // sample while this mode owns allocation. A sample is never used twice.
    void run_inner()
    {
        vehicle_angular_velocity_s gyro{};

        if (!_inner_gyro_sub->update(&gyro) || gyro.timestamp_sample <= _inner_last_sample) {
            return;
        }

        _inner_last_sample = gyro.timestamp_sample;
        _physical_path.count_inner_start();

        // Fresh copies: ownership must follow Commander at gyro rate, not at
        // Run()'s 100 Hz, so SE3 and the PX4 rate loop never both command.
        _inner_status_sub.update(&_inner_status);
        _inner_control_mode_sub.update(&_inner_control_mode);
        _inner_attitude_sub.update(&_inner_attitude);

        if (owns_allocation()) {
            publish_wrench(gyro);
        }

        _physical_path.count_inner_end();
    }

    // Same conditions as control_allocator's se3_physical::physical_owns_allocation().
    bool owns_allocation() const
    {
        return _mode_id >= 0 && _inner_status.nav_state == _mode_id &&
               _inner_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED &&
               !_inner_control_mode.flag_control_rates_enabled &&
               !_inner_control_mode.flag_control_attitude_enabled &&
               !_inner_control_mode.flag_multicopter_position_control_enabled &&
               _inner_control_mode.flag_control_allocation_enabled;
    }

    // Lee wrench while active and unfaulted; a failure latches the fault. The
    // neutral wrench (PhysicalPath::neutral_wrench) is published before the
    // first reference and after a fault until Commander applies the PX4
    // rate-control fallback or leaves the mode.
    void publish_wrench(const vehicle_angular_velocity_s &gyro)
    {
        se3_control::State state{};
        state.angular_velocity = se3_control::Vector3{gyro.xyz[0], gyro.xyz[1], gyro.xyz[2]};
        state.attitude = matrix::Dcmf{matrix::Quatf{_inner_attitude.q}};

        const se3_control::PhysicalAttitudeGains gains = physical_gains();
        float thrust = 0.0f;
        se3_control::Vector3 moment{};
        se3_control::Vector3 moment_integral{};
        bool lee = false;

        if (_active.load() && !_faulted) {
            const char *reason = nullptr;
            const se3_control::PhysicalPath::Fault fault =
                _physical_path.lee_wrench(state, _inner_attitude.timestamp, gyro.timestamp_sample, gains,
                                          thrust, moment, moment_integral, reason);

            if (fault == se3_control::PhysicalPath::Fault::None) {
                lee = true;

            } else {
                enter_fault(fault == se3_control::PhysicalPath::Fault::State ? FaultClass::State : FaultClass::Controller,
                            reason);
            }
        }

        if (!lee && !_physical_path.neutral_wrench(state, _landed.landed, gains, thrust, moment)) {
            return;
        }

#if defined(CONFIG_ARCH_BOARD_PX4_SITL)
        _physical_path.inject_invalid_wrench(_inject_invalid_wrench.load());
#endif
        _physical_path.publish(gyro, static_cast<uint8_t>(_mode_id), !lee, thrust, moment, moment_integral,
                               _param_hover.get());
    }

    // Readiness is the conjunction of the vehicle, estimate and configuration
    // predicates below. They are separated only to classify faults.
    bool ready() const
    {
        const bool supported_airframe = _param_autostart.get() != 6010 && _param_autostart.get() != 6002;

        return _mode_id >= 0 && !_faulted && _landed_received && flying() &&
               supported_airframe && state_valid() && configured();
    }

    bool flying() const
    {
        return _status.arming_state == vehicle_status_s::ARMING_STATE_ARMED &&
               !_landed.landed && !_status.failsafe &&
               _status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING &&
               !_status.is_vtol;
    }

    // Finite, valid and fresh estimator outputs.
    bool state_valid() const
    {
        const hrt_abstime now = hrt_absolute_time();
        const bool valid_position = _position.xy_valid && _position.z_valid &&
                                    _position.v_xy_valid && _position.v_z_valid &&
                                    std::isfinite(_position.x) && std::isfinite(_position.y) &&
                                    std::isfinite(_position.z) && std::isfinite(_position.vx) &&
                                    std::isfinite(_position.vy) && std::isfinite(_position.vz) &&
                                    std::isfinite(_position.heading);
        const bool valid_attitude = std::isfinite(_attitude.q[0]) && std::isfinite(_attitude.q[1]) &&
                                    std::isfinite(_attitude.q[2]) && std::isfinite(_attitude.q[3]);
        const float norm2 = _attitude.q[0] * _attitude.q[0] +
                             _attitude.q[1] * _attitude.q[1] +
                             _attitude.q[2] * _attitude.q[2] +
                             _attitude.q[3] * _attitude.q[3];
        const bool valid_gyro = std::isfinite(_gyro.xyz[0]) && std::isfinite(_gyro.xyz[1]) &&
                                std::isfinite(_gyro.xyz[2]);

        return valid_position && valid_attitude &&
               norm2 > 0.9f && norm2 < 1.1f && valid_gyro &&
               fresh(now, _position.timestamp, kPositionTimeout) &&
               fresh(now, _attitude.timestamp, kAttitudeTimeout) &&
               fresh(now, _gyro.timestamp, kGyroTimeout);
    }

    // Each path requires only the parameters it uses. All share the
    // translation gains; RATE_PX4 adds the SE3 body-rate gains and limit,
    // ATTITUDE_PX4 uses PX4's MC_* attitude and rate gains, and PHYSICAL its
    // own gains, mass and inertia.
    bool configured() const
    {
        const bool integral = std::isfinite(_param_kixy.get()) && _param_kixy.get() >= 0.0f &&
                              std::isfinite(_param_kiz.get()) && _param_kiz.get() >= 0.0f;

        if (_path == ControlPath::RatePx4) {
            return _param_enable.get() == 1 && integral &&
                   _param_kx.get() > 0.0f && _param_kv.get() > 0.0f &&
                   _param_krx.get() > 0.0f && _param_kry.get() > 0.0f &&
                   _param_krz.get() > 0.0f &&
                   _param_rate_max.get() > 0.0f &&
                   _param_hover.get() > 0.0f && _param_hover.get() < 1.0f &&
                   std::isfinite(_param_kx.get()) && std::isfinite(_param_kv.get()) &&
                   std::isfinite(_param_krx.get()) && std::isfinite(_param_kry.get()) &&
                   std::isfinite(_param_krz.get()) &&
                   std::isfinite(_param_rate_max.get()) && std::isfinite(_param_hover.get());
        }

        const bool translation = _param_enable.get() == 1 && integral &&
                                 _param_kx.get() > 0.0f && _param_kv.get() > 0.0f &&
                                 _param_hover.get() > 0.0f && _param_hover.get() < 1.0f &&
                                 std::isfinite(_param_kx.get()) && std::isfinite(_param_kv.get()) &&
                                 std::isfinite(_param_hover.get());

        return translation && (!_physical || _physical_path.configured(physical_gains()));
    }

    // PHYSICAL inner loop work item. It runs on Run()'s work queue, so the
    // two never execute concurrently.
    class InnerLoop final : public px4::WorkItem
    {
    public:
        explicit InnerLoop(Se3Runtime &runtime) :
            px4::WorkItem("se3_control_inner", px4::wq_configurations::rate_ctrl), _runtime(runtime) {}

        void Run() override { _runtime.run_inner(); }
        void clear() { ScheduleClear(); }

    private:
        Se3Runtime &_runtime;
    };

    // Commander registration, flight-mode state, and rate setpoint interfaces.
    uORB::Publication<register_ext_component_request_s> _register_pub{ORB_ID(register_ext_component_request)};
    uORB::Subscription _register_reply_sub{ORB_ID(register_ext_component_reply)};
    uORB::Publication<vehicle_control_mode_s> _mode_control_config_pub{ORB_ID(config_control_setpoints)};
    uORB::Publication<unregister_ext_component_s> _unregister_pub{ORB_ID(unregister_ext_component)};
    uORB::Subscription _arming_request_sub{ORB_ID(arming_check_request)};
    uORB::Publication<arming_check_reply_s> _arming_reply_pub{ORB_ID(arming_check_reply)};

    uORB::Subscription _status_sub{ORB_ID(vehicle_status)};
    uORB::Subscription _control_mode_sub{ORB_ID(vehicle_control_mode)};
    uORB::Subscription _landed_sub{ORB_ID(vehicle_land_detected)};
    uORB::Subscription _position_sub{ORB_ID(vehicle_local_position)};
    uORB::Subscription _attitude_sub{ORB_ID(vehicle_attitude)};
    uORB::Subscription _gyro_sub{ORB_ID(vehicle_angular_velocity)};
    uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 100_ms};
    uORB::Publication<vehicle_attitude_setpoint_s> _attitude_setpoint_pub{ORB_ID(vehicle_attitude_setpoint)};
    uORB::Publication<vehicle_rates_setpoint_s> _rate_setpoint_pub{ORB_ID(vehicle_rates_setpoint)};
    uORB::Publication<se3_trajectory_status_s> _trajectory_status_pub{ORB_ID(se3_trajectory_status)};
    uORB::Publication<trajectory_setpoint_s> _trajectory_pub{ORB_ID(trajectory_setpoint)};

    vehicle_status_s _status{};
    vehicle_control_mode_s _control_mode{};
    vehicle_land_detected_s _landed{};
    bool _landed_received{false};
    bool _faulted{false};
    vehicle_local_position_s _position{};
    vehicle_attitude_s _attitude{};
    vehicle_angular_velocity_s _gyro{};

    // Active plan (work-queue thread only) and the latest shell command.
    trajectory_generator::Trajectory _plan{};
    trajectory_generator::Trajectory _next_plan{}; // Built here, so a failed build keeps _plan.
    hrt_abstime _plan_start{0};
    uint32_t _plan_crc{0};
    struct Request {
        bool pending{false};
        int trajectory{trajectory_generator::kHoldId};
        trajectory_generator::Settings settings{};
    };
    pthread_mutex_t _request_mutex = PTHREAD_MUTEX_INITIALIZER;
    Request _request{};

    // Translational integral (common to all paths) and its anti-windup inputs.
    se3_control::PositionIntegral _integral{};
    hrt_abstime _integral_time{0};
    bool _command_saturated{false};
    uORB::Subscription _allocator_status_sub{ORB_ID(control_allocator_status)};
    control_allocator_status_s _allocator_status{};

    uint64_t _registration_request_id{0};
    int8_t _mode_id{-1};
    int8_t _arming_check_id{-1};
    uint8_t _xy_reset_counter{0};
    uint8_t _z_reset_counter{0};
    uint8_t _quat_reset_counter{0};
    uint32_t _run_id{0};
    px4::atomic_int32_t _registered_mode_id{-1};
    uORB::Publication<vehicle_command_s> _vehicle_command_pub{ORB_ID(vehicle_command)}; // CLI thread.

    // Fault handoff (work-queue thread only, except the SITL injection flag).
    uORB::Publication<vehicle_command_s> _handoff_command_pub{ORB_ID(vehicle_command)};
    uORB::Subscription _command_ack_sub{ORB_ID(vehicle_command_ack)};
    FaultResponse _fault_response{FaultResponse::None};
    hrt_abstime _position_request_time{0};
    uint8_t _last_arming_request_id{0};
    float _last_thrust{0.0f};
#if defined(CONFIG_ARCH_BOARD_PX4_SITL)
    px4::atomic<int32_t> _injected_fault{static_cast<int32_t>(FaultClass::None)};
#endif
    // Control path (SE3_CTRL), fixed at boot.
    ControlPath _path{ControlPath::RatePx4};
    bool _native_rate_fallback{false}; // PHYSICAL fault: mode runs the PX4 rate loop.
    float _last_yaw{0.0f};
    orb_advert_t _mavlink_log_pub{nullptr};

    // PHYSICAL control (SE3_CTRL = 2); all unused with the other paths.
    bool _physical{false};
    se3_control::PhysicalPath _physical_path{};
    InnerLoop *_inner{nullptr};
    uORB::SubscriptionCallbackWorkItem *_inner_gyro_sub{nullptr};
    uORB::Subscription _inner_status_sub{ORB_ID(vehicle_status)};
    uORB::Subscription _inner_control_mode_sub{ORB_ID(vehicle_control_mode)};
    uORB::Subscription _inner_attitude_sub{ORB_ID(vehicle_attitude)};
    vehicle_status_s _inner_status{};
    vehicle_control_mode_s _inner_control_mode{};
    vehicle_attitude_s _inner_attitude{};
    hrt_abstime _inner_last_sample{0};
#if defined(CONFIG_ARCH_BOARD_PX4_SITL)
    px4::atomic_bool _inject_invalid_wrench{false};
#endif

    // Read by shell commands and status.
    px4::atomic_bool _active{false};
    px4::atomic_bool _executing{false};
    px4::atomic_bool _completed{false};
    px4::atomic_int32_t _plan_id{trajectory_generator::kHoldId};

    DEFINE_PARAMETERS(
        (ParamInt<px4::params::SE3_ENABLE>) _param_enable,
        (ParamFloat<px4::params::SE3_KX>) _param_kx,
        (ParamFloat<px4::params::SE3_KV>) _param_kv,
        (ParamFloat<px4::params::SE3_KIXY>) _param_kixy,
        (ParamFloat<px4::params::SE3_KIZ>) _param_kiz,
        (ParamFloat<px4::params::SE3_KRX>) _param_krx,
        (ParamFloat<px4::params::SE3_KRY>) _param_kry,
        (ParamFloat<px4::params::SE3_KRZ>) _param_krz,
        (ParamFloat<px4::params::SE3_RATE_MAX>) _param_rate_max,
        (ParamFloat<px4::params::TRJ_SP_X>) _param_sp_x,
        (ParamFloat<px4::params::TRJ_SP_Y>) _param_sp_y,
        (ParamFloat<px4::params::TRJ_SP_Z>) _param_sp_z,
        (ParamFloat<px4::params::TRJ_SP_YAW>) _param_sp_yaw,
        (ParamFloat<px4::params::MPC_THR_HOVER>) _param_hover,
        (ParamInt<px4::params::SYS_AUTOSTART>) _param_autostart,
        (ParamInt<px4::params::SE3_CTRL>) _param_ctrl,
        (ParamFloat<px4::params::SE3_MASS>) _param_mass,
        (ParamFloat<px4::params::SE3_JXX>) _param_jxx,
        (ParamFloat<px4::params::SE3_JYY>) _param_jyy,
        (ParamFloat<px4::params::SE3_JZZ>) _param_jzz,
        (ParamFloat<px4::params::SE3_PKRX>) _param_pkrx,
        (ParamFloat<px4::params::SE3_PKRY>) _param_pkry,
        (ParamFloat<px4::params::SE3_PKRZ>) _param_pkrz,
        (ParamFloat<px4::params::SE3_PKWX>) _param_pkwx,
        (ParamFloat<px4::params::SE3_PKWY>) _param_pkwy,
        (ParamFloat<px4::params::SE3_PKWZ>) _param_pkwz,
        (ParamFloat<px4::params::SE3_PKI_XY>) _param_pki_xy,
        (ParamFloat<px4::params::SE3_PKI_Z>) _param_pki_z
    )
};

extern "C" __EXPORT int se3_control_main(int argc, char *argv[])
{
    // Allow shutdown only when disarmed; Commander handles a lost mode.
    if (argc > 1 && std::strcmp(argv[1], "stop") == 0) {
        uORB::Subscription status_sub{ORB_ID(vehicle_status)};
        vehicle_status_s status{};

        if (!status_sub.copy(&status) ||
            status.arming_state != vehicle_status_s::ARMING_STATE_DISARMED) {
            PX4_ERR("SE3 can only be stopped while disarmed");
            return PX4_ERROR;
        }
    }

    return Se3Runtime::main(argc, argv);
}
