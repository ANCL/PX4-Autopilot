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
        // On stop, Commander invalidates this registered mode and selects
        // its existing fallback if it was active. Never leave a ghost mode.
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
        PX4_INFO("enabled=%d active=%d", (int)(_param_enable.get() == 1), (int)_active);
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

    static int custom_command(int argc, char *argv[]) { return print_usage("unknown command"); }

    static int print_usage(const char *reason = nullptr)
    {
        if (reason) { PX4_WARN("%s", reason); }
        PRINT_MODULE_DESCRIPTION(R"DESCR_STR(
### Description
Experimental SE(3) geometric body-rate controller for an already-flying multicopter.
Manual SITL startup only. Disabled unless SE3_ENABLE=1.f No autonomous takeoff.
PX4 retains rate PID, allocator, and output/flight-safety ownership.
)DESCR_STR");
        PRINT_MODULE_USAGE_NAME("se3_control", "controller");
        PRINT_MODULE_USAGE_COMMAND("start");
        PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
        return 0;
    }

private:
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
            _active = false;
            _faulted = false; // Clear fault only after leaving the mode.
            return;
        }

        // Do not resume control just because a stale sample recovered or
        // the estimator briefly became valid again. Force a mode exit.
        if (!valid) {
            _active = false;
            _faulted = true;
            return;
        }

        if (!_active) {
            // Capture actual local NED position and yaw at the mode handoff.
            // The first supported reference is a HOLD, never an implicit
            // origin-relative step or automatic takeoff.
            _origin = {};
            _origin.position = {_position.x, _position.y, _position.z};
            _origin.yaw = _position.heading;
            _sequence = {};
            _sequence.count = 1;
            _sequence.segments[0].type = trajectory_generator::Segment::Type::Hold;
            _sequence.segments[0].duration_s = 86400.0f;
            _xy_reset_counter = _position.xy_reset_counter;
            _z_reset_counter = _position.z_reset_counter;
            _quat_reset_counter = _attitude.quat_reset_counter;
            _start_time = hrt_absolute_time();
            _active = true;
        }

        // Estimator-frame discontinuities are not silently interpreted as
        // trajectory tracking errors. Commander must reject this mode.
        if (_xy_reset_counter != _position.xy_reset_counter ||
            _z_reset_counter != _position.z_reset_counter ||
            _quat_reset_counter != _attitude.quat_reset_counter) {
            _active = false;
            _faulted = true;
            return;
        }

        trajectory_generator::Reference reference{};
        if (!trajectory_generator::sample(_sequence,
                                         static_cast<float>(hrt_absolute_time() - _start_time) * 1.0e-6f,
                                         _origin, reference)) {
            _active = false;
            _faulted = true;
            return;
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
            _active = false;
            _faulted = true;
            return;
        }

        // The normalized conversion is the existing PX4 hover-thrust
        // convention; no physical-wrench uORB messages or new allocator.
        const float thrust = _param_hover.get() * geometric.collective_thrust_n / (mass * kGravity);
        const float rate_limit = _param_rate_max.get();

        if (!std::isfinite(thrust) || !body_rate.isAllFinite() || !std::isfinite(rate_limit)) {
            _active = false;
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
    bool _active{false};

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
    return Se3Runtime::main(argc, argv);
}
