// PX4-native SE(3) body-rate handoff (initial SITL-only integration).
//
// Ownership contract:
//   This module owns ONLY the desired position/attitude -> body-rate setpoint.
//   mc_rate_control owns gyro-rate feedback, PID, and torque production.
//   control_allocator owns allocation; PX4 owns output limiting and failsafes.
//
// Flight-mode registration uses PX4's existing Commander external-mode
// registration mechanism locally over uORB: no ROS, DDS or new nav-state enum.
// The module is NOT automatically started or selected.

#include "AttitudeLaws.hpp"
#include "Controller.hpp"

#include <trajectory/Sampler.hpp>
#include <trajectory/Plan.hpp>
#include <px4_platform_common/atomic.h>
#include <uORB/topics/vehicle_command.h>

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
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>

#include <matrix/matrix/math.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>

using namespace time_literals;

namespace
{
constexpr char kModeName[] = "Onboard SE3";
constexpr float kGravity = 9.80665f;
constexpr hrt_abstime kPositionTimeout = 200_ms;
constexpr hrt_abstime kAttitudeTimeout = 200_ms;
constexpr hrt_abstime kGyroTimeout = 100_ms;
// Reject a commanded trajectory that would jump away from the hover at entry.
constexpr float kMaxStartDistanceM = 0.15f;
constexpr float kMaxStartYawErrorRad = 0.20f;

bool fresh(hrt_abstime now, hrt_abstime stamp, hrt_abstime limit)
{
    return stamp > 0 && now >= stamp && now - stamp <= limit;
}
} // namespace

class Se3Runtime final : public ModuleBase<Se3Runtime>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
    Se3Runtime() : ModuleParams(nullptr), ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl) {}

    ~Se3Runtime() override
    {
        // Inform Commander that this external mode no longer exists.
        // The selected failsafe action is PX4-owned, not set by this module.
        if (_mode_id >= 0 && _arming_check_id >= 0) {
            unregister_ext_component_s unregister{};
            unregister.timestamp = hrt_absolute_time();
            std::strncpy(unregister.name, kModeName, sizeof(unregister.name) - 1);
            unregister.arming_check_id = _arming_check_id;
            unregister.mode_id = _mode_id;
            unregister.mode_executor_id = -1;
            _unregister_pub.publish(unregister);
        }
    }

    bool init()
    {
        vehicle_status_s status{};

        if (!_status_sub.copy(&status) || status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
            PX4_ERR("start only while disarmed with Commander running");
            return false;
        }

        updateParams();
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

        ScheduleOnInterval(10_ms); // 100 Hz outer loop; PX4 gyro rate PID runs independently.
        return true;
    }

    int print_status() override
    {
        PX4_INFO("Commander mode id=%d arming check id=%d", (int)_mode_id, (int)_arming_check_id);
        PX4_INFO("enabled=%d active=%d faulted=%d", (int)(_param_enable.get() == 1),
                 (int)_active.load(), (int)_faulted);
        PX4_INFO("trajectory: %s", _executing.load() ? "running" :
                 (_completed.load() ? "completed (holding final pose)" : "holding"));
        PX4_INFO("PID/allocator: PX4 native (not reimplemented)");
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

        if (argc == 1 && std::strcmp(argv[0], "run") == 0) {
            return instance->command_run();
        }

        if (argc == 1 && std::strcmp(argv[0], "hold") == 0) {
            return instance->command_hold();
        }

        return print_usage("unknown command");
    }

    static int print_usage(const char *reason = nullptr)
    {
        if (reason) { PX4_WARN("%s", reason); }
        PRINT_MODULE_DESCRIPTION(R"DESCR_STR(
### Description
Experimental SE(3) geometric body-rate controller for an already-flying multicopter.
Manual SITL startup only. Disabled unless SE3_ENABLE=1. No autonomous takeoff.
PX4 retains rate PID, allocator, and output/flight-safety ownership.
)DESCR_STR");
        PRINT_MODULE_USAGE_NAME("se3_control", "controller");
        PRINT_MODULE_USAGE_COMMAND("start");
        PRINT_MODULE_USAGE_COMMAND("select");
        PRINT_MODULE_USAGE_COMMAND("run");
        PRINT_MODULE_USAGE_COMMAND("hold");
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

        vehicle_command_s command{};
        command.timestamp = hrt_absolute_time();
        command.command = vehicle_command_s::VEHICLE_CMD_SET_NAV_STATE;
        command.param1 = static_cast<float>(mode_id);
        command.source_system = status.system_id;
        command.source_component = status.component_id;
        command.target_system = status.system_id;
        command.target_component = status.component_id;

        if (!_vehicle_command_pub.publish(command)) {
            PX4_ERR("mode request publication failed");
            return PX4_ERROR;
        }

        PX4_INFO("SE3 mode %d requested; confirm Commander accepted", mode_id);
        return PX4_OK;
    }

    int command_run()
    {
        if (!_active.load() || _executing.load() ||
            _run_requested.load() || _hold_requested.load()) {
            PX4_ERR("SE3 must be holding in its selected flight mode");
            return PX4_ERROR;
        }

        trajectory_generator::Plan plan{};
        unsigned revision = 0;

        if (!trajectory_generator::snapshot_plan(plan, &revision)) {
            PX4_ERR("no trajectory selected (use trajectory select <name>)");
            return PX4_ERROR;
        }

        uORB::Subscription local_position{ORB_ID(vehicle_local_position)};
        vehicle_local_position_s position{};

        if (!local_position.copy(&position) ||
            !start_matches(plan, position)) {
            PX4_ERR("not at loaded absolute-NED trajectory start; staying in hold");
            return PX4_ERROR;
        }

        // A later load cannot silently replace the plan already checked.
        _run_revision.store(revision);
        _run_requested.store(true);
        PX4_INFO("trajectory run requested; awaiting onboard control tick");
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

    static bool start_matches(const trajectory_generator::Plan &plan,
                              const vehicle_local_position_s &position)
    {
        if (!position.xy_valid || !position.z_valid ||
            !position.v_xy_valid || !position.v_z_valid ||
            !std::isfinite(position.x) || !std::isfinite(position.y) ||
            !std::isfinite(position.z) || !std::isfinite(position.heading) ||
            position.timestamp == 0 ||
            !fresh(hrt_absolute_time(), position.timestamp, kPositionTimeout)) {
            return false;
        }

        const float dx = position.x - plan.origin.position[0];
        const float dy = position.y - plan.origin.position[1];
        const float dz = position.z - plan.origin.position[2];
        const float delta_yaw = position.heading - plan.origin.yaw;
        const float yaw_error = std::atan2(std::sin(delta_yaw), std::cos(delta_yaw));

        return dx * dx + dy * dy + dz * dz <=
               kMaxStartDistanceM * kMaxStartDistanceM &&
               std::fabs(yaw_error) <= kMaxStartYawErrorRad;
    }

    void capture_hold()
    {
        // Position mode or the previous SE3 trajectory has relinquished
        // the reference. Hold the latest measured NED pose, not a stale
        // waypoint or a translated copy of the loaded trajectory.
        _origin = {};
        _origin.position = {_position.x, _position.y, _position.z};
        _origin.yaw = _position.heading;
        _sequence = {};
        _sequence.count = 1;
        _sequence.segments[0].type = trajectory_generator::Segment::Type::Hold;
        _sequence.segments[0].duration_s = 86400.0f;
        _start_time = hrt_absolute_time();
        _executing.store(false);
        _completed.store(false);
    }

    void Run() override
    {
        if (should_exit()) {
            ScheduleClear();
            exit_and_cleanup();
            return;
        }

        if (_parameter_update_sub.updated()) {
            parameter_update_s update{};
            _parameter_update_sub.copy(&update);
            updateParams();
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
                    config.flag_control_rates_enabled = true;
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
            arming_check_reply_s reply{};
            reply.timestamp = hrt_absolute_time();
            reply.request_id = request.request_id;
            reply.registration_id = static_cast<uint8_t>(_arming_check_id);
            reply.can_arm_and_run = valid;
            reply.mode_req_angular_velocity = true;
            reply.mode_req_attitude = true;
            reply.mode_req_local_position = true;
            reply.mode_req_prevent_arming = true; // Airborne handoff only, no takeoff/arm.
            _arming_reply_pub.publish(reply);
        }

        // No publication at all while another flight mode owns the setpoint.
        const bool selected = _mode_id >= 0 && _status.nav_state == _mode_id &&
                              _control_mode.flag_control_rates_enabled &&
                              !_control_mode.flag_control_attitude_enabled &&
                              !_control_mode.flag_multicopter_position_control_enabled &&
                              _control_mode.flag_control_allocation_enabled;

        if (!selected) {
            _active.store(false);
            _executing.store(false);
            _completed.store(false);
            _run_requested.store(false);
            _hold_requested.store(false);

            // Keep any SE3 fault latched throughout the flight. Commander
            // owns recovery using its existing external-mode failsafe.
            // Reset only on the ground, never merely because the mode changed.
            if (_status.arming_state == vehicle_status_s::ARMING_STATE_DISARMED) {
                _faulted = false;
            }

            return;
        }

        // Do not resume control just because a stale sample recovered or
        // the estimator briefly became valid again. Force a mode exit.
        if (!valid) {
            _active.store(false);
            _executing.store(false);
            _faulted = true;
            return;
        }

        if (!_active.load()) {
            // Mode entry is deliberately passive: capture the actual pose
            // and hold it until a separate explicit run command arrives.
            capture_hold();
            _run_requested.store(false);
            _hold_requested.store(false);
            _xy_reset_counter = _position.xy_reset_counter;
            _z_reset_counter = _position.z_reset_counter;
            _quat_reset_counter = _attitude.quat_reset_counter;
            _active.store(true);
        }

        // Estimator-frame discontinuities are not silently interpreted as
        // trajectory tracking errors. Commander must reject this mode.
        if (_xy_reset_counter != _position.xy_reset_counter ||
            _z_reset_counter != _position.z_reset_counter ||
            _quat_reset_counter != _attitude.quat_reset_counter) {
            _active.store(false);
            _executing.store(false);
            _faulted = true;
            return;
        }

        // Process commands only on the controller work queue. A file may be
        // loaded in Position mode without moving or starting this controller.
        bool pending = true;

        if (_hold_requested.compare_exchange(&pending, false)) {
            _run_requested.store(false);
            capture_hold();
        }

        pending = true;

        if (_run_requested.compare_exchange(&pending, false)) {
            trajectory_generator::Plan plan{};
            unsigned revision = 0;

            if (trajectory_generator::snapshot_plan(plan, &revision) &&
                revision == _run_revision.load() && start_matches(plan, _position)) {
                _origin = plan.origin;
                _sequence = plan.sequence;
                _duration_s = plan.duration_s;
                _start_time = hrt_absolute_time();
                _executing.store(true);
                _completed.store(false);
                PX4_INFO("SE3 trajectory started (%u segments)",
                         static_cast<unsigned>(_sequence.count));

            } else {
                PX4_WARN("trajectory start rejected; SE3 remains in hold");
            }
        }

        const float elapsed_s =
            static_cast<float>(hrt_absolute_time() - _start_time) * 1.0e-6f;
        trajectory_generator::Reference reference{};

        if (!trajectory_generator::sample(_sequence, elapsed_s, _origin, reference)) {
            _active.store(false);
            _executing.store(false);
            _faulted = true;
            return;
        }

        if (_executing.load() && elapsed_s >= _duration_s) {
            _executing.store(false);
            _completed.store(true); // Sampler holds the exact final reference.
        }

        se3_control::State state{};
        state.position = se3_control::Vector3{_position.x, _position.y, _position.z};
        state.velocity = se3_control::Vector3{_position.vx, _position.vy, _position.vz};
        state.angular_velocity = se3_control::Vector3{_gyro.xyz[0], _gyro.xyz[1], _gyro.xyz[2]};

        const matrix::Quatf quat{_attitude.q};
        state.attitude = matrix::Dcmf{quat};

        const float mass = _param_mass.get();
        se3_control::TranslationGains translation_gains{};
        translation_gains.mass = mass;
        translation_gains.position = mass * _param_kx.get();
        translation_gains.velocity = mass * _param_kv.get();

        se3_control::GeometricOutput geometric{};
        se3_control::Vector3 body_rate{};
        const se3_control::Vector3 attitude_gains{
            _param_krx.get(), _param_kry.get(), _param_krz.get()
        };

        if (!se3_control::Controller::compute_geometry(state, reference, translation_gains, geometric) ||
            !se3_control::AttitudeLaws::body_rate(state, geometric.desired_attitude, attitude_gains, body_rate)) {
            _active.store(false);
            _executing.store(false);
            _faulted = true;
            return;
        }

        // The normalized conversion is the existing PX4 hover-thrust
        // convention; no physical-wrench uORB messages or new allocator.
        const float thrust = _param_hover.get() * geometric.collective_thrust_n / (mass * kGravity);
        const float rate_limit = _param_rate_max.get();

        if (!std::isfinite(thrust) || !body_rate.isAllFinite() || !std::isfinite(rate_limit)) {
            _active.store(false);
            _executing.store(false);
            _faulted = true;
            return;
        }

        vehicle_rates_setpoint_s setpoint{};
        setpoint.timestamp = hrt_absolute_time();
        setpoint.roll = std::min(std::max(body_rate(0), -rate_limit), rate_limit);
        setpoint.pitch = std::min(std::max(body_rate(1), -rate_limit), rate_limit);
        setpoint.yaw = std::min(std::max(body_rate(2), -rate_limit), rate_limit);
        setpoint.thrust_body[0] = 0.f;
        setpoint.thrust_body[1] = 0.f;
        setpoint.thrust_body[2] = -std::min(std::max(thrust, 0.0f), 1.0f);
        _rate_setpoint_pub.publish(setpoint);
    }

    bool ready() const
    {
        const hrt_abstime now = hrt_absolute_time();
        const bool flying = _status.arming_state == vehicle_status_s::ARMING_STATE_ARMED &&
                            !_landed.landed && !_status.failsafe &&
                            _status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING &&
                            !_status.is_vtol;
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
        const bool supported_airframe = _param_autostart.get() != 6010 && _param_autostart.get() != 6002;
        const bool configured = _param_enable.get() == 1 &&
                                _param_mass.get() > 0.0f &&
                                _param_kx.get() > 0.0f && _param_kv.get() > 0.0f &&
                                _param_krx.get() > 0.0f && _param_kry.get() > 0.0f &&
                                _param_krz.get() > 0.0f &&
                                _param_rate_max.get() > 0.0f &&
                                _param_hover.get() > 0.0f && _param_hover.get() < 1.0f &&
                                std::isfinite(_param_mass.get()) &&
                                std::isfinite(_param_kx.get()) && std::isfinite(_param_kv.get()) &&
                                std::isfinite(_param_krx.get()) && std::isfinite(_param_kry.get()) &&
                                std::isfinite(_param_krz.get()) &&
                                std::isfinite(_param_rate_max.get()) && std::isfinite(_param_hover.get());

        return _mode_id >= 0 && !_faulted && _landed_received && flying &&
               supported_airframe &&
               valid_position && valid_attitude &&
               norm2 > 0.9f && norm2 < 1.1f && valid_gyro && configured &&
               fresh(now, _position.timestamp, kPositionTimeout) &&
               fresh(now, _attitude.timestamp, kAttitudeTimeout) &&
               fresh(now, _gyro.timestamp, kGyroTimeout);
    }

    // Existing PX4 Commander external-mode protocol, not a new protocol.
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
    uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
    uORB::Publication<vehicle_rates_setpoint_s> _rate_setpoint_pub{ORB_ID(vehicle_rates_setpoint)};

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
    hrt_abstime _start_time{0};
    uint64_t _registration_request_id{0};
    int8_t _mode_id{-1};
    int8_t _arming_check_id{-1};
    uint8_t _xy_reset_counter{0};
    uint8_t _z_reset_counter{0};
    uint8_t _quat_reset_counter{0};
    float _duration_s{0.0f};
    px4::atomic_int32_t _registered_mode_id{-1};
    uORB::Publication<vehicle_command_s> _vehicle_command_pub{ORB_ID(vehicle_command)};
    px4::atomic_bool _active{false};
    px4::atomic_bool _executing{false};
    px4::atomic_bool _completed{false};
    px4::atomic_bool _run_requested{false};
    px4::atomic_bool _hold_requested{false};
    px4::atomic<uint32_t> _run_revision{0};

    DEFINE_PARAMETERS(
        (ParamInt<px4::params::SE3_ENABLE>) _param_enable,
        (ParamFloat<px4::params::SE3_MASS>) _param_mass,
        (ParamFloat<px4::params::SE3_KX>) _param_kx,
        (ParamFloat<px4::params::SE3_KV>) _param_kv,
        (ParamFloat<px4::params::SE3_KRX>) _param_krx,
        (ParamFloat<px4::params::SE3_KRY>) _param_kry,
        (ParamFloat<px4::params::SE3_KRZ>) _param_krz,
        (ParamFloat<px4::params::SE3_RATE_MAX>) _param_rate_max,
        (ParamFloat<px4::params::MPC_THR_HOVER>) _param_hover,
        (ParamInt<px4::params::SYS_AUTOSTART>) _param_autostart
    )
};

extern "C" __EXPORT int se3_control_main(int argc, char *argv[])
{
    // Do not deliberately stop a running controller while flying. An
    // unexpected crash is handled by PX4's existing external-mode timeout.
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
