// Geometric SE(3) control with body-rate and collective-thrust setpoints.
//
// The controller tracks a position and yaw reference using vehicle state.
// mc_rate_control tracks commanded body rates; control_allocator produces
// actuator commands. Commander manages flight-mode transitions and failsafes.
//
// SE3_CTRL = PHYSICAL (fixed at boot) replaces the rate setpoint with the
// physical Lee wrench: Run() keeps the same 100 Hz reference, hold, trajectory
// and fault logic, and a gyro-triggered inner loop (run_inner()) computes the
// moment for every new angular-velocity sample and publishes
// se3_wrench_setpoint, which control_allocator allocates in rotor-thrust
// space. The registered mode then disables the PX4 rate loop.
//
// Setpoints are published only while this registered flight mode is active.
// Fault handling (Position-mode request or native PX4 failsafe) is described
// at enter_fault().

#include "AttitudeLaws.hpp"
#include "Controller.hpp"

#include "trajectory/Trajectory.hpp"
#include <parameters/param.h>
#include <px4_platform_common/atomic.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <lib/perf/perf_counter.h>
#include <lib/se3_physical/PhysicalModel.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/failure_detector_status.h>
#include <uORB/topics/se3_allocation_status.h>
#include <uORB/topics/se3_wrench_setpoint.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>

#include <drivers/drv_hrt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/arming_check_reply.h>
#include <uORB/topics/arming_check_request.h>
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

#include <float.h>
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

// PHYSICAL inner loop: the 100 Hz reference may miss at most two updates, and
// the attitude must come from the current estimator output (>= 200 Hz).
constexpr hrt_abstime kReferenceTimeout = 30_ms;
constexpr hrt_abstime kInnerAttitudeTimeout = 20_ms;

// control_allocator answers every wrench; silence means it does not accept them.
constexpr hrt_abstime kAllocationTimeout = 50_ms;

enum class ControlPath : int32_t { RatePx4 = 0, Physical = 1 };

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
        perf_free(_inner_perf);
        perf_free(_inner_interval_perf);
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
        _physical = _param_ctrl.get() == static_cast<int32_t>(ControlPath::Physical);

        if (_physical && !init_physical()) {
            return false;
        }

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
        PX4_INFO("trajectory: %s", _executing.load() ? "running" :
                 (_completed.load() ? "completed (holding final pose)" : "holding"));
        static constexpr const char *responses[] = {
            "none", "Position requested", "Position accepted", "PX4 failsafe"
        };
        PX4_INFO("fault response: %s", responses[static_cast<int>(_fault_response)]);

        if (_physical) {
            PX4_INFO("control: PHYSICAL (%s), parameters %s", _model.name,
                     _physical_parameters_ok ? "match model" : "DO NOT match model");
            perf_print_counter(_inner_perf);
            perf_print_counter(_inner_interval_perf);

        } else {
            PX4_INFO("control: RATE_PX4");
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
Experimental SE(3) geometric body-rate controller for an already-flying multicopter.
Registered at boot on supported targets. Selection requires valid airborne state. No autonomous takeoff.
PX4 retains rate PID, allocator, and output/flight-safety ownership.
)DESCR_STR");
        PRINT_MODULE_USAGE_NAME("se3_control", "controller");
        PRINT_MODULE_USAGE_COMMAND("start");
        PRINT_MODULE_USAGE_COMMAND("select");
        PRINT_MODULE_USAGE_COMMAND("hold");
        PRINT_MODULE_USAGE_COMMAND("list");
        PRINT_MODULE_USAGE_COMMAND("trajectory");
#if defined(CONFIG_ARCH_BOARD_PX4_SITL)
        PRINT_MODULE_USAGE_COMMAND_DESCR("fault", "SITL test only: inject a fault (requires SYS_FAILURE_EN=1)");
        PRINT_MODULE_USAGE_ARG("controller|state|allocation", "Fault class (allocation: PHYSICAL wrench with an unknown model)", false);
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

        _hold_requested.store(true);
        PX4_INFO("SE3 current-position hold requested");
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
            // Real allocator rejection path: wrenches name an unknown model
            // until the resulting fault is detected.
            _inject_unknown_model.store(true);

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
    void enter_fault(FaultClass fault, const char *reason)
    {
        clear_trajectory_request();
        _active.store(false);
        _executing.store(false);
        _faulted = true;
#if defined(CONFIG_ARCH_BOARD_PX4_SITL)
        _inject_unknown_model.store(false);
#endif

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
    void publish_neutral_setpoint()
    {
        if (!_landed_received || !flying()) {
            return;
        }

        const float hover = _param_hover.get();
        const bool hover_valid = std::isfinite(hover) && hover > 0.0f && hover < 1.0f;

        vehicle_rates_setpoint_s setpoint{};
        setpoint.timestamp = hrt_absolute_time();
        setpoint.thrust_body[2] = -(hover_valid ? hover : _last_thrust);
        _rate_setpoint_pub.publish(setpoint);
    }

    // The shell supplies a named trajectory and a parameter snapshot.
    // Only the SE3 work queue changes the active reference.
    int command_trajectory(const char *name)
    {
        const int index = trajectory_generator::trajectory_index(name);

        if (index < 0) {
            PX4_ERR("unknown trajectory: %s", name);
            return PX4_ERROR;
        }

        if (!_active.load() || _executing.load() || _hold_requested.load()) {
            PX4_ERR("SE3 must be active and holding");
            return PX4_ERROR;
        }

        trajectory_generator::Settings settings{};
        if (!trajectory_generator::load_settings(settings)) {
            PX4_ERR("invalid trajectory parameters");
            return PX4_ERROR;
        }

        pthread_mutex_lock(&_trajectory_mutex);
        const bool accepted = _requested_trajectory < 0 && _active.load() &&
                              !_executing.load() && !_hold_requested.load();

        if (accepted) {
            _requested_settings = settings;
            _requested_trajectory = index;
        }

        pthread_mutex_unlock(&_trajectory_mutex);

        if (!accepted) {
            PX4_ERR("SE3 is busy");
            return PX4_ERROR;
        }

        PX4_INFO("trajectory requested: %s", name);
        return PX4_OK;
    }

    void clear_trajectory_request()
    {
        pthread_mutex_lock(&_trajectory_mutex);
        _requested_trajectory = -1;
        pthread_mutex_unlock(&_trajectory_mutex);
    }

    void capture_hold()
    {
        // Initialize a stationary reference at the measured local pose.
        _origin = {};
        _origin.position = {_position.x, _position.y, _position.z};
        _origin.yaw = _position.heading;
        _sequence = trajectory_generator::Sequence{};
        _sequence.count = 1;
        _sequence.segments[0].type = trajectory_generator::Segment::Type::Hold;
        _sequence.segments[0].duration_s = 86400.0f;
        _start_time = hrt_absolute_time();
        _executing.store(false);
        _completed.store(false);
        _active_plan_crc = 0;
        _active_trajectory_index = -1;
    }

    void Run() override
    {
        if (should_exit()) {
            clear_trajectory_request();
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
                _physical_parameters_ok = physical_parameters_match();
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

                    vehicle_control_mode_s config{};
                    config.timestamp = hrt_absolute_time();
                    config.source_id = static_cast<uint8_t>(_mode_id);
                    // Only the PX4 rate loop and allocator are enabled.
                    // Commander derives multicopter-position control from
                    // the position/velocity/altitude flags (all remain false).
                    // PHYSICAL leaves only the allocator enabled: its moment
                    // replaces the rate loop (as in Offboard thrust/torque).
                    config.flag_control_rates_enabled = !_physical;
                    config.flag_control_allocation_enabled = true;
                    _mode_control_config_pub.publish(config);
                    PX4_INFO("registered mode %d (arming check %d)", (int)_mode_id, (int)_arming_check_id);
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

        // No publication at all while another flight mode owns the setpoint.
        // RATE_PX4 requires the PX4 rate loop; PHYSICAL requires it disabled.
        const bool rate_loop = _physical ? !_control_mode.flag_control_rates_enabled :
                               _control_mode.flag_control_rates_enabled;
        const bool selected = _mode_id >= 0 && _status.nav_state == _mode_id &&
                              rate_loop &&
                              !_control_mode.flag_control_attitude_enabled &&
                              !_control_mode.flag_multicopter_position_control_enabled &&
                              _control_mode.flag_control_allocation_enabled;

        if (!selected) {
            clear_trajectory_request();
            _active.store(false);
            _executing.store(false);
            _completed.store(false);
            _hold_requested.store(false);

            _reference.valid = false;

            // Keep faults latched until disarm, including after a mode change.
            if (_status.arming_state == vehicle_status_s::ARMING_STATE_DISARMED) {
                _faulted = false;
                _fault_response = FaultResponse::None;
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
            // and hold it until an explicit trajectory start arrives.
            capture_hold();
            _hold_requested.store(false);
            _xy_reset_counter = _position.xy_reset_counter;
            _z_reset_counter = _position.z_reset_counter;
            _quat_reset_counter = _attitude.quat_reset_counter;
            _active_time = hrt_absolute_time();
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
        const char *allocation_problem = _physical ? physical_allocation_problem() : nullptr;

        if (allocation_problem != nullptr) {
            enter_fault(FaultClass::Controller, allocation_problem);
            return;
        }

        bool hold_requested = true;
        if (_hold_requested.compare_exchange(&hold_requested, false)) {
            clear_trajectory_request();
            capture_hold();
        }

        int requested_index = -1;
        trajectory_generator::Settings settings{};
        pthread_mutex_lock(&_trajectory_mutex);

        if (_requested_trajectory >= 0 && !_executing.load()) {
            requested_index = _requested_trajectory;
            settings = _requested_settings;
            _requested_trajectory = -1;
        }

        pthread_mutex_unlock(&_trajectory_mutex);

        if (requested_index >= 0) {
            // The selected reference is absolute NED; no pose alignment occurs.
            auto &trajectory = _next_trajectory;
            if (!trajectory_generator::build_trajectory(requested_index, settings, trajectory)) {
                PX4_ERR("invalid trajectory definition");
            } else {
                _origin = trajectory.origin;
                _sequence = trajectory.sequence;
                _duration_s = trajectory.duration_s;
                // Live setpoint has no fixed-plan checksum because it can change in flight.
                _active_plan_crc = requested_index == static_cast<int>(trajectory_generator::TrajectoryId::Setpoint)
                    ? 0 : trajectory.checksum;
                _active_trajectory_index = requested_index;
                ++_run_id;
                _start_time = hrt_absolute_time();
                _executing.store(true);
                _completed.store(false);
                PX4_INFO("SE3 trajectory started (%u segments)",
                         static_cast<unsigned>(_sequence.count));
            }
        }

        const float elapsed_s =
            static_cast<float>(hrt_absolute_time() - _start_time) * 1.0e-6f;
        trajectory_generator::Reference reference{};

        if (!trajectory_generator::sample(_sequence, elapsed_s, _origin, reference)) {
            enter_fault(FaultClass::Controller, "trajectory sampling failed");
            return;
        }

        const bool live_setpoint = _active_trajectory_index ==
            static_cast<int>(trajectory_generator::TrajectoryId::Setpoint);
        // Live setpoint is the one reference that follows parameter edits.
        // No interpolation: changes become instantaneous command steps.
        if (live_setpoint) {
            reference = {};
            reference.position = {{_param_sp_x.get(), _param_sp_y.get(), _param_sp_z.get()}};
            reference.yaw = _param_sp_yaw.get();
        }

        if (_executing.load() && !live_setpoint && elapsed_s >= _duration_s) {
            _executing.store(false);
            _completed.store(true); // Sampler holds the exact final reference.
        }

        se3_control::State state{};
        state.position = se3_control::Vector3{_position.x, _position.y, _position.z};
        state.velocity = se3_control::Vector3{_position.vx, _position.vy, _position.vz};
        state.angular_velocity = se3_control::Vector3{_gyro.xyz[0], _gyro.xyz[1], _gyro.xyz[2]};

        const matrix::Quatf quat{_attitude.q};
        state.attitude = matrix::Dcmf{quat};

        // Use the same mass-normalized translation law as ROS Offboard.
        se3_control::TranslationGains translation_gains{};
        translation_gains.position = _param_kx.get();
        translation_gains.velocity = _param_kv.get();

        se3_control::GeometricOutput geometric{};
        se3_control::Vector3 body_rate{};
        const se3_control::Vector3 attitude_gains{
            _param_krx.get(), _param_kry.get(), _param_krz.get()
        };

        if (!se3_control::Controller::compute_geometry(state, reference, translation_gains, geometric) ||
            !se3_control::AttitudeLaws::body_rate(state, geometric.desired_attitude, attitude_gains, body_rate)) {
            enter_fault(FaultClass::Controller, "geometric control computation failed");
            return;
        }

        // Convert collective thrust to the normalized PX4 body-z convention.
        const float thrust = _param_hover.get() * geometric.collective_acceleration / kGravity;
        const float rate_limit = _param_rate_max.get();

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

        if (_physical) {
            // The inner loop uses this reference until the next update. The
            // rate setpoint is ignored by mc_rate_control while SE3 owns
            // allocation; zero rates make it a neutral command for the first
            // PX4 rate-loop cycle after SE3 is deselected.
            _reference.specific_force = geometric.translation.specific_force;
            _reference.desired = geometric.desired_attitude;
            _reference.timestamp = setpoint.timestamp;
            _reference.valid = true;
            setpoint.roll = 0.f;
            setpoint.pitch = 0.f;
            setpoint.yaw = 0.f;
        }

        _rate_setpoint_pub.publish(setpoint);
        _last_thrust = -setpoint.thrust_body[2];

        // Expose the exact geometric attitude reference through the standard
        // PX4 observation topic. SE3 still owns body-rate control: Commander
        // keeps mc_att_control disabled in this external flight mode.
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
        record.plan_crc32 = _active_plan_crc;
        record.phase = _completed.load() ? se3_trajectory_status_s::PHASE_COMPLETED :
                       (_executing.load() ? se3_trajectory_status_s::PHASE_RUNNING :
                                            se3_trajectory_status_s::PHASE_HOLD);
        record.elapsed_s = _active_trajectory_index >= 0 ? fminf(elapsed_s, _duration_s) : 0.0f;
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

    // PHYSICAL setup at boot: model, its parameter dependencies and the
    // gyro-triggered inner loop on Run()'s work queue (same thread, so the
    // inner loop shares state with Run() without locking).
    bool init_physical()
    {
        _model_id = static_cast<uint8_t>(_param_model.get());

        if (!se3_physical::get_model(_model_id, _model) || !se3_physical::valid(_model)) {
            PX4_ERR("SE3_MODEL %d is not a valid physical model", (int)_param_model.get());
            return false;
        }

        _physical_parameters_ok = physical_parameters_match();
        _inner = new InnerLoop(*this);
        _inner_gyro_sub = _inner ? new uORB::SubscriptionCallbackWorkItem(_inner, ORB_ID(vehicle_angular_velocity)) : nullptr;

        if (_inner_gyro_sub == nullptr || !_inner_gyro_sub->registerCallback()) {
            PX4_ERR("SE3 inner loop start failed");
            return false;
        }

        _inner_perf = perf_alloc(PC_ELAPSED, MODULE_NAME": physical inner");
        _inner_interval_perf = perf_alloc(PC_INTERVAL, MODULE_NAME": physical inner interval");
        PX4_INFO("PHYSICAL control with model %s", _model.name);
        return true;
    }

    // The model's thrust curve and geometry hold only for its airframe and
    // output mapping, and only if PX4 applies no thrust curve of its own.
    bool physical_parameters_match() const
    {
        float thrust_model_factor = NAN;

        // FunctionMotors applies its curve only for a factor above FLT_EPSILON.
        if (param_get(param_find("THR_MDL_FAC"), &thrust_model_factor) != PX4_OK ||
            !(fabsf(thrust_model_factor) <= FLT_EPSILON)) {
            return false;
        }

        for (int i = 0; i < _model.required_count; ++i) {
            const param_t handle = param_find(_model.required[i].name);
            int32_t value = 0;

            if (handle == PARAM_INVALID || param_get(handle, &value) != PX4_OK || value != _model.required[i].value) {
                return false;
            }
        }

        return true;
    }

    se3_control::PhysicalAttitudeGains physical_gains() const
    {
        se3_control::PhysicalAttitudeGains gains{};
        gains.attitude = se3_control::Vector3{_param_pkrx.get(), _param_pkry.get(), _param_pkrz.get()};
        gains.angular_velocity = se3_control::Vector3{_param_pkwx.get(), _param_pkwy.get(), _param_pkwz.get()};
        return gains;
    }

    bool physical_configured() const
    {
        const se3_control::PhysicalAttitudeGains gains = physical_gains();

        for (int axis = 0; axis < 3; ++axis) {
            if (!std::isfinite(gains.attitude(axis)) || gains.attitude(axis) <= 0.0f ||
                !std::isfinite(gains.angular_velocity(axis)) || gains.angular_velocity(axis) <= 0.0f) {
                return false;
            }
        }

        return _physical_parameters_ok;
    }

    // Checked every Run() while PHYSICAL SE3 is active; returns the fault
    // reason, or nullptr if control_allocator is allocating the wrench.
    const char *physical_allocation_problem()
    {
        se3_allocation_status_s status{};

        if (_allocation_status_sub.update(&status)) {
            _allocation_status = status;
        }

        failure_detector_status_s failure{};

        if (_failure_detector_sub.update(&failure)) {
            _motor_failure = failure.fd_motor;
        }

        if (_motor_failure) {
            return "motor failure detected";
        }

        // Only allocator replies since this activation count.
        if (_allocation_status.timestamp >= _active_time) {
            if (!_allocation_status.compatible) {
                return "physical allocation incompatible with model";
            }

            if (!_allocation_status.valid) {
                return "physical allocation without all rotors";
            }
        }

        const hrt_abstime now = hrt_absolute_time();

        if (now - _active_time > kAllocationTimeout &&
            !fresh(now, _allocation_status.timestamp, kAllocationTimeout)) {
            return "control_allocator not allocating the SE3 wrench";
        }

        return nullptr;
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
        perf_count(_inner_interval_perf);
        perf_begin(_inner_perf);

        // Fresh copies: ownership must follow Commander at gyro rate, not at
        // Run()'s 100 Hz, so SE3 and the PX4 rate loop never both command.
        _inner_status_sub.update(&_inner_status);
        _inner_control_mode_sub.update(&_inner_control_mode);
        _inner_attitude_sub.update(&_inner_attitude);

        if (owns_allocation()) {
            publish_wrench(gyro);
        }

        perf_end(_inner_perf);
    }

    bool owns_allocation() const
    {
        return _mode_id >= 0 && _inner_status.nav_state == _mode_id &&
               _inner_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED &&
               !_inner_control_mode.flag_control_rates_enabled &&
               !_inner_control_mode.flag_control_attitude_enabled &&
               !_inner_control_mode.flag_multicopter_position_control_enabled &&
               _inner_control_mode.flag_control_allocation_enabled;
    }

    // Lee wrench from the latest 100 Hz reference and the newest attitude and
    // gyro sample. On failure a fault is latched and false returned.
    bool lee_wrench(const se3_control::State &state, float &thrust, se3_control::Vector3 &moment)
    {
        const hrt_abstime now = hrt_absolute_time();

        if (!fresh(now, _inner_attitude.timestamp, kInnerAttitudeTimeout) ||
            !state.attitude.isAllFinite() || !state.angular_velocity.isAllFinite()) {
            enter_fault(FaultClass::State, "attitude or angular velocity not valid for SE3");
            return false;
        }

        if (!_reference.valid || !fresh(now, _reference.timestamp, kReferenceTimeout)) {
            enter_fault(FaultClass::Controller, "SE3 reference not updated");
            return false;
        }

        // Lee collective thrust f = -m (A/m) . (R e3) with the current attitude.
        const se3_control::Vector3 body_z{state.attitude(0, 2), state.attitude(1, 2), state.attitude(2, 2)};
        const float collective = -(_reference.specific_force * body_z);
        se3_control::PhysicalMomentOutput output{};

        if (!std::isfinite(collective) || collective <= 0.0f ||
            !se3_control::AttitudeLaws::physical(state, _reference.desired, _model.inertia,
                    physical_gains(), output)) {
            enter_fault(FaultClass::Controller, "physical wrench computation failed");
            return false;
        }

        thrust = _model.mass * collective;
        moment = output.moment;
        return true;
    }

    void publish_wrench(const vehicle_angular_velocity_s &gyro)
    {
        se3_control::State state{};
        state.angular_velocity = se3_control::Vector3{gyro.xyz[0], gyro.xyz[1], gyro.xyz[2]};
        state.attitude = matrix::Dcmf{matrix::Quatf{_inner_attitude.q}};

        float thrust = 0.0f;
        se3_control::Vector3 moment{};
        const bool lee = _active.load() && !_faulted && lee_wrench(state, thrust, moment);

        if (!lee) {
            // Neutral wrench while SE3 is selected without a valid Lee
            // reference (before the first reference, or after a fault until
            // Commander changes mode): with R_d = R and Omega_d = 0 the Lee
            // moment is rate damping plus Omega x J Omega, so rotation is
            // stopped, not levelled. Weight-compensating thrust in the air,
            // idle on the ground. Without a gyro sample nothing is published.
            if (!state.angular_velocity.isAllFinite()) {
                return;
            }

            if (!state.attitude.isAllFinite()) {
                state.attitude.setIdentity(); // R_d = R: e_R = 0 for any R.
            }

            se3_control::DesiredAttitudeDynamics current{};
            current.attitude = state.attitude;
            se3_control::PhysicalMomentOutput output{};

            if (!se3_control::AttitudeLaws::physical(state, current, _model.inertia, physical_gains(), output)) {
                return;
            }

            moment = output.moment;
            thrust = _landed.landed ? 0.0f : _model.mass * kGravity;
        }

        se3_wrench_setpoint_s setpoint{};
        setpoint.timestamp_sample = gyro.timestamp_sample;
        setpoint.nav_state = static_cast<uint8_t>(_mode_id);
        setpoint.model = _model_id;
#if defined(CONFIG_ARCH_BOARD_PX4_SITL)

        if (_inject_unknown_model.load()) {
            setpoint.model = 0xff;
        }

#endif
        setpoint.neutral = !lee;
        setpoint.thrust = thrust;

        for (int axis = 0; axis < 3; ++axis) {
            setpoint.moment[axis] = moment(axis);
        }

        setpoint.timestamp = hrt_absolute_time();
        _wrench_pub.publish(setpoint);

        // Normalized collective for the land detector and other observers
        // (the allocator ignores it in this path), on the same hover-scaled
        // convention as the RATE_PX4 thrust setpoint.
        vehicle_thrust_setpoint_s observation{};
        observation.timestamp_sample = gyro.timestamp_sample;
        observation.xyz[2] = -fminf(fmaxf(_param_hover.get() * thrust / (_model.mass * kGravity), 0.0f), 1.0f);
        observation.timestamp = setpoint.timestamp;
        _thrust_observation_pub.publish(observation);
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

    bool configured() const
    {
        return _param_enable.get() == 1 &&
               _param_kx.get() > 0.0f && _param_kv.get() > 0.0f &&
               _param_krx.get() > 0.0f && _param_kry.get() > 0.0f &&
               _param_krz.get() > 0.0f &&
               _param_rate_max.get() > 0.0f &&
               _param_hover.get() > 0.0f && _param_hover.get() < 1.0f &&
               std::isfinite(_param_kx.get()) && std::isfinite(_param_kv.get()) &&
               std::isfinite(_param_krx.get()) && std::isfinite(_param_kry.get()) &&
               std::isfinite(_param_krz.get()) &&
               std::isfinite(_param_rate_max.get()) && std::isfinite(_param_hover.get()) &&
               (!_physical || physical_configured());
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

    // Reference computed by Run() and held by the inner loop until the next.
    struct PhysicalReference
    {
        se3_control::Vector3 specific_force{}; // A/m in NED [m/s^2].
        se3_control::DesiredAttitudeDynamics desired{};
        hrt_abstime timestamp{0};
        bool valid{false};
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

    trajectory_generator::Reference _origin{};
    trajectory_generator::Sequence _sequence{};
    trajectory_generator::Trajectory _next_trajectory{};
    pthread_mutex_t _trajectory_mutex = PTHREAD_MUTEX_INITIALIZER;
    trajectory_generator::Settings _requested_settings{};
    int _requested_trajectory{-1};
    int _active_trajectory_index{-1};
    hrt_abstime _start_time{0};
    uint64_t _registration_request_id{0};
    int8_t _mode_id{-1};
    int8_t _arming_check_id{-1};
    uint8_t _xy_reset_counter{0};
    uint8_t _z_reset_counter{0};
    uint8_t _quat_reset_counter{0};
    float _duration_s{0.0f};
    uint32_t _run_id{0};
    uint32_t _active_plan_crc{0};
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
    // PHYSICAL control (SE3_CTRL = 1); all unused with RATE_PX4.
    bool _physical{false};
    uint8_t _model_id{0};
    se3_physical::Model _model{};
    bool _physical_parameters_ok{false};
    PhysicalReference _reference{};
    hrt_abstime _active_time{0};
    InnerLoop *_inner{nullptr};
    uORB::SubscriptionCallbackWorkItem *_inner_gyro_sub{nullptr};
    uORB::Subscription _inner_status_sub{ORB_ID(vehicle_status)};
    uORB::Subscription _inner_control_mode_sub{ORB_ID(vehicle_control_mode)};
    uORB::Subscription _inner_attitude_sub{ORB_ID(vehicle_attitude)};
    vehicle_status_s _inner_status{};
    vehicle_control_mode_s _inner_control_mode{};
    vehicle_attitude_s _inner_attitude{};
    hrt_abstime _inner_last_sample{0};
    uORB::Publication<se3_wrench_setpoint_s> _wrench_pub{ORB_ID(se3_wrench_setpoint)};
    uORB::Publication<vehicle_thrust_setpoint_s> _thrust_observation_pub{ORB_ID(vehicle_thrust_setpoint)};
    uORB::Subscription _allocation_status_sub{ORB_ID(se3_allocation_status)};
    uORB::Subscription _failure_detector_sub{ORB_ID(failure_detector_status)};
    se3_allocation_status_s _allocation_status{};
    bool _motor_failure{false};
    perf_counter_t _inner_perf{nullptr};
    perf_counter_t _inner_interval_perf{nullptr};
#if defined(CONFIG_ARCH_BOARD_PX4_SITL)
    px4::atomic_bool _inject_unknown_model{false};
#endif

    px4::atomic_bool _active{false};
    px4::atomic_bool _executing{false};
    px4::atomic_bool _completed{false};
    px4::atomic_bool _hold_requested{false};

    DEFINE_PARAMETERS(
        (ParamInt<px4::params::SE3_ENABLE>) _param_enable,
        (ParamFloat<px4::params::SE3_KX>) _param_kx,
        (ParamFloat<px4::params::SE3_KV>) _param_kv,
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
        (ParamInt<px4::params::SE3_MODEL>) _param_model,
        (ParamFloat<px4::params::SE3_PKRX>) _param_pkrx,
        (ParamFloat<px4::params::SE3_PKRY>) _param_pkry,
        (ParamFloat<px4::params::SE3_PKRZ>) _param_pkrz,
        (ParamFloat<px4::params::SE3_PKWX>) _param_pkwx,
        (ParamFloat<px4::params::SE3_PKWY>) _param_pkwy,
        (ParamFloat<px4::params::SE3_PKWZ>) _param_pkwz
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
