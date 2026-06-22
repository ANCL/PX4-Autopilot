#include "FAPositionControl.hpp"
#include <lib/matrix/matrix/math.hpp>
#include <drivers/drv_hrt.h>

using namespace matrix;

FAPositionControl::FAPositionControl() : 
    ModuleParams(nullptr), 
    WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
    updateParams();
}

FAPositionControl::~FAPositionControl()
{
    // cleanup
}

bool FAPositionControl::init() 
{
    // wake up the module whenever new angular velocity data is published
    if (!_vehicle_angular_velocity_sub.registerCallback()) {
    PX4_ERR("Callback registration failed");
    return false;
}

    _time_stamp_last_loop = hrt_absolute_time();
    return true;
}

void FAPositionControl::parameters_update(bool force)
{
    if (_parameter_update_sub.updated() || force) {
        parameter_update_s param_update;
        _parameter_update_sub.copy(&param_update);
        updateParams();
        
        _mass = _param_fa_mass.get();

        _k_p = Vector3f(_param_fa_p_x.get(), _param_fa_p_y.get(), _param_fa_p_z.get());
        _k_v = Vector3f(_param_fa_v_x.get(), _param_fa_v_y.get(), _param_fa_v_z.get());
        _k_r = Vector3f(_param_fa_r_r.get(), _param_fa_r_p.get(), _param_fa_r_y.get());
        _k_w = Vector3f(_param_fa_w_r.get(), _param_fa_w_p.get(), _param_fa_w_y.get()); 

        _thrust_maximums = Vector3f(_param_fa_thr_max_x.get(), _param_fa_thr_max_y.get(), _param_fa_thr_max_z.get());
        _torque_maximums = Vector3f(_param_fa_trq_max_r.get(), _param_fa_trq_max_p.get(), _param_fa_trq_max_y.get());

        _HOVER_THRUST_MIN = _param_fa_hover_min.get();
        _HOVER_THRUST_MAX = _param_fa_hover_max.get();

        _use_hover_thrust_estimate = _param_fa_hvr_thr_on.get() != 0;
    }
}

void FAPositionControl::Run()
{
    if (should_exit()) {
        _vehicle_angular_velocity_sub.unregisterCallback();
        exit_and_cleanup();
        return;
    }

    // check parameters
    parameters_update();

    // only run if we have new attitude data to process
    if (_vehicle_angular_velocity_sub.updated()) {
        
        // calculate dt
        const hrt_abstime time_stamp_now = hrt_absolute_time();
        float dt = (time_stamp_now - _time_stamp_last_loop) / 1e6f; // dt in sec
        
        // constrain dt to reasonable bounds [0.1ms, 50ms]
        dt = math::constrain(dt, 0.0001f, 0.05f);
        _time_stamp_last_loop = time_stamp_now;

        // run the control loop
        update(dt);
    }
}

bool FAPositionControl::update(const float dt)
{
    // fetch vehicle states
    vehicle_local_position_s local_pos{};
    _vehicle_local_position_sub.copy(&local_pos);

    vehicle_attitude_s attitude{};
    _vehicle_attitude_sub.copy(&attitude);

    vehicle_angular_velocity_s angular_vel{};
    _vehicle_angular_velocity_sub.copy(&angular_vel);

    // fetch desired setpoints (from FlightTaskFAPosition)
    trajectory_setpoint_s trajectory_sp{};
    _trajectory_setpoint_sub.copy(&trajectory_sp);

    vehicle_attitude_setpoint_s attitude_sp{};
    _vehicle_attitude_setpoint_sub.copy(&attitude_sp);
    
    vehicle_rates_setpoint_s angular_vel_sp{};
    _vehicle_rates_setpoint_sub.copy(&angular_vel_sp);

    // fetch hover thrust estimate
    hover_thrust_estimate_s hover_thrust_estimate{};
    _hover_thrust_estimate_sub.copy(&hover_thrust_estimate);

    // see land_detected status
    vehicle_land_detected_s land_detected{};
    _vehicle_land_detected_sub.copy(&land_detected);
    
    // setup state vectors
    Vector3f pos(local_pos.x, local_pos.y, local_pos.z);
    Vector3f vel(local_pos.vx, local_pos.vy, local_pos.vz);
    Quatf q(attitude.q);
    Vector3f w(angular_vel.xyz);
    
    Vector3f pos_sp(trajectory_sp.position);
    Vector3f vel_sp(trajectory_sp.velocity);
    Vector3f acc_sp(trajectory_sp.acceleration);

    float current_yaw = Eulerf(q).psi(); 

    float desired_yaw = PX4_ISFINITE(trajectory_sp.yaw) ? trajectory_sp.yaw : current_yaw;
    Quatf q_d(Eulerf(0.0f, 0.0f, desired_yaw)); 
    
    float desired_yawspeed = PX4_ISFINITE(trajectory_sp.yawspeed) ? trajectory_sp.yawspeed : 0.0f;
    Vector3f w_d(0.0f, 0.0f, desired_yawspeed);

    // compute position, velocity, and acceleration errors per-axis
    for (int i = 0; i < 3; i++) {
        _e_p(i) = PX4_ISFINITE(pos_sp(i)) ? (pos(i) - pos_sp(i)) : 0.0f;
        _e_v(i) = PX4_ISFINITE(vel_sp(i)) ? (vel(i) - vel_sp(i)) : 0.0f;
        acc_sp(i) = PX4_ISFINITE(acc_sp(i)) ? acc_sp(i) : 0.0f;
    }

    Dcmf R(q); // body attitude
    Dcmf R_d(q_d); // desired attitude

    // (R_d^T * R) - (R^T * R_d)
    Dcmf R_error_matrix = (R_d.transpose() * R) - (R.transpose() * R_d);

    // compute vee(R_error_matrix)
    _e_R(0) = R_error_matrix(2, 1) * 0.5f;
    _e_R(1) = R_error_matrix(0, 2) * 0.5f;
    _e_R(2) = R_error_matrix(1, 0) * 0.5f;

    // compute angular velocity error: e_w = w - [R^T][R_d]w_d
    _e_w = w - (R.transpose() * R_d) * w_d;

    // ---------------------------------------------------------
    // APPLY CONTROL LAW
    // ---------------------------------------------------------
    static constexpr float g = 9.81f;
    Vector3f z(0.0f, 0.0f, -1.0f);
    
    float hover_thrust = 0.5f;
    bool is_airborne = !land_detected.landed;

    // use the dynamic estimator if the user enabled it, we are airborne, and the data is valid
    if (_use_hover_thrust_estimate && is_airborne && 
        hover_thrust_estimate.valid && PX4_ISFINITE(hover_thrust_estimate.hover_thrust)) {
        
        hover_thrust = math::constrain(hover_thrust_estimate.hover_thrust, _HOVER_THRUST_MIN, _HOVER_THRUST_MAX);
    }
    
    // F_n = m*a_d - K_p*e_p - K_v*e_v + m*g*e_D
    Vector3f a_n = acc_sp + g * z - _k_p.emult(_e_p) - _k_v.emult(_e_v);
    
    // F_b = R^T * F_n
    Dcmf R_transpose(q.inversed()); 
    Vector3f a_b = R_transpose * a_n;


    // Normalize the force to [-1, 1] (N) for the PX4 mixer
    for (int i = 0; i < 3; ++i) {
        if (_use_hover_thrust_estimate) {
            // use the adaptive hover thrust fraction
            float max_thrust_ratio = _thrust_maximums(2) / _thrust_maximums(i); 
            _vehicle_thrust_setpoint(i) = a_b(i) * (hover_thrust / g) * max_thrust_ratio;
        } else {
            // direct force calculation based purely on mass and maximums
            float required_force = a_b(i) * _mass;
            _vehicle_thrust_setpoint(i) = required_force / _thrust_maximums(i);
        }

        // if landed, command 0 thrust to prevent spool-up (unless a takeoff accel is commanded)
        if (!is_airborne && vel_sp.length() < 0.1f) {
            _vehicle_thrust_setpoint(i) = 0.0f;
        } else {
            _vehicle_thrust_setpoint(i) = math::constrain(_vehicle_thrust_setpoint(i), -1.0f, 1.0f);
        }
    }

    // compute torque setpoint
    Vector3f tau_b = -_k_r.emult(_e_R) - _k_w.emult(_e_w);

    // normalize the torque to [-1, 1] (N*m) for the PX4 mixer
    for (int i = 0; i < 3; ++i) {
        _vehicle_torque_setpoint(i) = tau_b(i) / _torque_maximums(i);
    }

    // publish outputs
    vehicle_thrust_setpoint_s thrust_msg{};
    thrust_msg.timestamp = hrt_absolute_time();
    _vehicle_thrust_setpoint.copyTo(thrust_msg.xyz); // Copy normalized vector
    _vehicle_thrust_setpoint_pub.publish(thrust_msg);

    vehicle_torque_setpoint_s torque_msg{};
    torque_msg.timestamp = hrt_absolute_time();
    _vehicle_torque_setpoint.copyTo(torque_msg.xyz); // Copy normalized vector
    _vehicle_torque_setpoint_pub.publish(torque_msg);

    return true;
}

// =============================================================================
// REQUIRED PX4 MODULE BOILERPLATE
// =============================================================================

int FAPositionControl::task_spawn(int argc, char *argv[])
{
    FAPositionControl *instance = new FAPositionControl();

    if (instance) {
        _object.store(instance);
        _task_id = task_id_is_work_queue;

        if (instance->init()) {
            return PX4_OK;
        }
    } else {
        PX4_ERR("alloc failed");
    }

    delete instance;
    _object.store(nullptr);
    _task_id = -1;

    return PX4_ERROR;
}

int FAPositionControl::custom_command(int argc, char *argv[])
{
    return print_usage("unknown command");
}

int FAPositionControl::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
Fully actuated position controller.
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("fa_position", "controller");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

// entry point for the module
extern "C" __EXPORT int fa_pos_control_main(int argc, char *argv[]);

int fa_pos_control_main(int argc, char *argv[])
{
    // Assuming your class inherits from ModuleBase, 
    // it already has a built-in main() function to handle start/stop/status.
    return FAPositionControl::main(argc, argv);
}