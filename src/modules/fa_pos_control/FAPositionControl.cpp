#include "FAPositionControl.hpp"
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
    // wake up the module whenever new angular velocity data arrives
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
        
        // Physical Properties
        _mass = _param_fa_mass.get();
        _inertia = Vector3f(_param_fa_j_x.get(), _param_fa_j_y.get(), _param_fa_j_z.get());

        // Sliding Mode Alpha
        _alpha = _param_fa_alpha.get();

        // Position Control Gains
        _k_p = Vector3f(_param_fa_p_x.get(), _param_fa_p_y.get(), _param_fa_p_z.get());
        _k_v = Vector3f(_param_fa_v_x.get(), _param_fa_v_y.get(), _param_fa_v_z.get());
        _k_i = Vector3f(_param_fa_i_x.get(), _param_fa_i_y.get(), _param_fa_i_z.get());
        _k_i_r = Vector3f(_param_fa_i_r_r.get(), _param_fa_i_r_p.get(), _param_fa_i_r_y.get()); 
        _int_limit = _param_fa_int_lim.get();
        _int_limit_r = _param_fa_int_lim_r.get();

        // Attitude Control Gains
        _k_r = Vector3f(_param_fa_r_r.get(), _param_fa_r_p.get(), _param_fa_r_y.get());
        _k_w = Vector3f(_param_fa_w_r.get(), _param_fa_w_p.get(), _param_fa_w_y.get());
        
        // Sliding Mode Auxiliary Gains
        _k_3 = Vector3f(_param_fa_k3_r.get(), _param_fa_k3_p.get(), _param_fa_k3_y.get());
        _k_4 = Vector3f(_param_fa_k4_r.get(), _param_fa_k4_p.get(), _param_fa_k4_y.get());

        // Actuator Limits
        _thrust_maximums = Vector3f(_param_fa_thr_max_x.get(), _param_fa_thr_max_y.get(), _param_fa_thr_max_z.get());
        _torque_maximums = Vector3f(_param_fa_trq_max_r.get(), _param_fa_trq_max_p.get(), _param_fa_trq_max_y.get());
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

    // only run if we have new angular velocity to process
    if (_vehicle_angular_velocity_sub.updated()) {
        
        // calculate dt
        const hrt_abstime time_stamp_now = hrt_absolute_time();
        float dt = (time_stamp_now - _time_stamp_last_loop) / 1e6f; // dt in sec
        
        // constrain dt to reasonable bounds [0.0125ms, 10ms]
        dt = math::constrain(dt, 0.0000125f, 0.01f);
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

    // fetch desired setpoints
    trajectory_setpoint_s trajectory_sp{};
    _trajectory_setpoint_sub.copy(&trajectory_sp);

    vehicle_attitude_setpoint_s attitude_sp{};
    _vehicle_attitude_setpoint_sub.copy(&attitude_sp);
    
    vehicle_rates_setpoint_s angular_vel_sp{};
    _vehicle_rates_setpoint_sub.copy(&angular_vel_sp);

    // fetch hover thrust & land state
    hover_thrust_estimate_s hover_thrust_estimate{};
    _hover_thrust_estimate_sub.copy(&hover_thrust_estimate);

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
    
    // clean up incoming setpoints
    sanitize_vector(vel_sp);
    sanitize_vector(acc_sp);

    float desired_yaw = PX4_ISFINITE(trajectory_sp.yaw) ? trajectory_sp.yaw : Eulerf(q).psi();
    Quatf q_d(Eulerf(0.0f, 0.0f, desired_yaw)); // hard coded zero roll-pitch for now
    
    float desired_yawspeed = PX4_ISFINITE(trajectory_sp.yawspeed) ? trajectory_sp.yawspeed : 0.0f;
    Vector3f w_d(0.0f, 0.0f, desired_yawspeed); // hard coded zero roll-pitch for now

    _e_p = compute_error(pos, pos_sp);
    _e_v = compute_error(vel, vel_sp);

    Dcmf R(q);      // body attitude
    Dcmf R_d(q_d);  // desired attitude

    // R_{e,r} = R_d^T * R
    Dcmf R_er = R_d.transpose() * R;
    // R_{e,r}^T = R^T * R_d
    Dcmf R_er_T = R.transpose() * R_d;

    // e_R = 1/2 (R_{e,r} - R_{e,r}^T)^vee
    Dcmf R_error_matrix = R_er - R_er_T;
    _e_R(0) = R_error_matrix(2, 1) * 0.5f;
    _e_R(1) = R_error_matrix(0, 2) * 0.5f;
    _e_R(2) = R_error_matrix(1, 0) * 0.5f;

    // e_w = w - R_{e,r}^T * w_d
    Vector3f w_r = R_er_T * w_d; 
    _e_w = w - w_r;

    // Calculate auxiliary signals v_R and v_w
    Vector3f v_R;
    Vector3f v_w;
    for (int i = 0; i < 3; i++) {
        v_R(i) = frac_sgn(_e_R(i), _alpha);
        v_w(i) = frac_sgn(_e_w(i), _alpha);
    }

    // ---------------------------------------------------------
    // INTEGRATION & ANTI-WINDUP
    // ---------------------------------------------------------
    bool is_airborne = !land_detected.landed;

    if (is_airborne) {
        // accumulate error over time (Riemann sum)
        _e_p_int += _e_p * dt;
        _e_R_int += _e_R * dt;

        // clamp the accumulated error component-wise
        for (int i = 0; i < 3; i++) {
            _e_p_int(i) = math::constrain(_e_p_int(i), -_int_limit, _int_limit);
            _e_R_int(i) = math::constrain(_e_R_int(i), -_int_limit_r, _int_limit_r);
        }
    } else {
        // reset the integrator when on the ground to prevent takeoff spikes
        _e_p_int.zero();
        _e_R_int.zero();
    }

    // ---------------------------------------------------------
    // APPLY CONTROL LAW
    // ---------------------------------------------------------
    static constexpr float g = 9.81f;
    Vector3f z(0.0f, 0.0f, -1.0f);
    
    // Position Control
    // F_n = m*a_d - K_p*e_p - K_v*e_v - K_i*int(e_p) + m*g*z
    Vector3f F_n = _mass * (acc_sp + g * z) - _k_p.emult(_e_p) - _k_v.emult(_e_v) - _k_i.emult(_e_p_int);
    
    // F_b = R^T * F_n
    Dcmf R_transpose(q.inversed()); 
    Vector3f F_b = R_transpose * F_n; 

    // Attitude Control
    // k_R*e_R + k_w*e_w + k_3*v_R + k_4*v_w + k_i_r*int(e_R)
    Vector3f feedback = _k_r.emult(_e_R) + _k_w.emult(_e_w) + _k_3.emult(v_R) + _k_4.emult(v_w) + _k_i_r.emult(_e_R_int);
    
    // J * feedback
    Vector3f J_feedback = _inertia.emult(feedback);

    // Gyroscopic term: w x J*w
    Vector3f J_w = _inertia.emult(w);
    Vector3f gyroscopic = w % J_w; // % is the cross product operator in PX4 matrix lib

    // Feedforward terms: J * (w x w_r - R_er_T * w_d_dot)
    Vector3f w_d_dot(0.0f, 0.0f, 0.0f); 
    Vector3f feedforward = _inertia.emult((w % w_r) - (R_er_T * w_d_dot));

    // Full control law: u_tau = -J*(feedback) + w x J*w - J*(w x w_r - R_er_T*w_d_dot)
    Vector3f tau_b = -J_feedback + gyroscopic - feedforward;
    
    // Normalize about the maximums
    for (int i = 0; i < 3; ++i) {
        _vehicle_thrust_setpoint(i) = F_b(i) / _thrust_maximums(i);
        _vehicle_torque_setpoint(i) = tau_b(i) / _torque_maximums(i); 
    }

    // publish outputs using the generic template
    publish_actuator_setpoint<vehicle_thrust_setpoint_s>(_vehicle_thrust_setpoint_pub, _vehicle_thrust_setpoint);
    publish_actuator_setpoint<vehicle_torque_setpoint_s>(_vehicle_torque_setpoint_pub, _vehicle_torque_setpoint);

    return true;
}

// =============================================================================
// HELPER FUNCTIONS
// =============================================================================

// cleans up vectors that may contain NaNs from the setpoints
void FAPositionControl::sanitize_vector(Vector3f& vec) 
{
    for (int i = 0; i < 3; ++i) {
        if (!PX4_ISFINITE(vec(i))) vec(i) = 0.0f;
    }
}

// calculates the error between a state and a setpoint, defaulting to 0 if the setpoint is NaN
Vector3f FAPositionControl::compute_error(const Vector3f& state, const Vector3f& setpoint) 
{
    Vector3f error_vec;
    for (int i = 0; i < 3; ++i) {
        error_vec(i) = PX4_ISFINITE(setpoint(i)) ? (state(i) - setpoint(i)) : 0.0f;
    }
    return error_vec;
}

// a template to handle the identical boilerplate for publishing thrust and torque
template <typename MsgType, typename PubType>
void FAPositionControl::publish_actuator_setpoint(PubType& publisher, const Vector3f& data) 
{
    MsgType msg{};
    msg.timestamp = hrt_absolute_time();
    data.copyTo(msg.xyz);
    publisher.publish(msg);
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

    PRINT_MODULE_USAGE_NAME("fa_pos_control", "controller");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

// entry point for the module
extern "C" __EXPORT int fa_pos_control_main(int argc, char *argv[]);

int fa_pos_control_main(int argc, char *argv[])
{
    return FAPositionControl::main(argc, argv);
}