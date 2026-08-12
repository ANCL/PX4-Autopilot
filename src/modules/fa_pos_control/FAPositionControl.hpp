/**
 * @file FAPositionControl.hpp
 *
 * A custom position controller intended for a fully-actuated hexarotor
 */

#pragma once

#include <cmath> // Added for std::pow and std::abs
#include <matrix/matrix/math.hpp>
#include <lib/matrix/matrix/math.hpp>
#include <perf/perf_counter.h>

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/hover_thrust_estimate.h>

using namespace time_literals;

class FAPositionControl : public ModuleBase<FAPositionControl>, public ModuleParams, 
    public px4::WorkItem
{
public:

    FAPositionControl();
    ~FAPositionControl();

    static int task_spawn(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);

    bool init();

    /**
     * Apply control law
     * @param dt time in seconds since last iteration
     * @return true if update succeeded and output setpoint is executable, false if not
     */
    bool update(const float dt);

private:
    void Run() override;
    void parameters_update(bool force = false);

    // Helper Functions
    void sanitize_vector(matrix::Vector3f& vec);
    matrix::Vector3f compute_error(const matrix::Vector3f& state, const matrix::Vector3f& setpoint);
    matrix::Vector3f project_wrench(const matrix::Vector3f& command, const matrix::Vector3f& max_limits);
    template <typename MsgType, typename PubType> void publish_actuator_setpoint(PubType& publisher, const matrix::Vector3f& data);

    // Values
    float _mass;
    matrix::Vector3f _inertia{0.05f, 0.05f, 0.06f}; // Jxx, Jyy, Jzz
    float _alpha{0.8f};             // Fractional power for sliding mode

    // Limits
    matrix::Vector3f _thrust_maximums;
    matrix::Vector3f _torque_maximums;
    float _int_limit;               // maximum allowed value for the position integral state
    float _int_limit_r;             // maximum allowed value for the attitude integral state
    
    // Gains
    matrix::Vector3f _k_p;          // position P gain
    matrix::Vector3f _k_v;          // position D gain
    matrix::Vector3f _k_i;          // position I gain
    matrix::Vector3f _k_r;          // attitude P gain (k_R)
    matrix::Vector3f _k_w;          // attitude D gain (k_W)
    matrix::Vector3f _k_3;          // attitude sliding mode gain (k_3)
    matrix::Vector3f _k_4;          // attitude sliding mode gain (k_4)
    matrix::Vector3f _k_i_r;        // attitude I gain
    
    // Errors
    matrix::Vector3f _e_p;          // position error
    matrix::Vector3f _e_v;          // velocity error
    matrix::Vector3f _e_R;          // rotational error
    matrix::Vector3f _e_w;          // angular vel error
    matrix::Vector3f _e_p_int{};    // accumulated position error
    matrix::Vector3f _e_R_int{};    // accumulated attitude error

    // Setpoints
    matrix::Vector3f _vehicle_thrust_setpoint; // normalized [-1, 1]
    matrix::Vector3f _vehicle_torque_setpoint; // normalized [-1, 1]
    
    // Timestamp
    hrt_abstime _time_stamp_last_loop{0};

    // uORB Subscriptions
    uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

    uORB::Subscription _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint)};
    uORB::Subscription _vehicle_attitude_setpoint_sub{ORB_ID(vehicle_attitude_setpoint)};
    uORB::Subscription _vehicle_rates_setpoint_sub{ORB_ID(vehicle_rates_setpoint)};
    
    uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
    uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
    uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
    uORB::Subscription _hover_thrust_estimate_sub{ORB_ID(hover_thrust_estimate)};
    
    // uORB Callbacks
    uORB::SubscriptionCallbackWorkItem _vehicle_angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};

    // uORB Publishers
    uORB::Publication<vehicle_thrust_setpoint_s>  _vehicle_thrust_setpoint_pub{ORB_ID(vehicle_thrust_setpoint)};
    uORB::Publication<vehicle_torque_setpoint_s>  _vehicle_torque_setpoint_pub{ORB_ID(vehicle_torque_setpoint)};

    // PX4 Params
    DEFINE_PARAMETERS(
        (ParamFloat<px4::params::FA_MASS>) _param_fa_mass,
        
        // Inertia Parameters
        (ParamFloat<px4::params::FA_J_X>) _param_fa_j_x,
        (ParamFloat<px4::params::FA_J_Y>) _param_fa_j_y,
        (ParamFloat<px4::params::FA_J_Z>) _param_fa_j_z,
        
        // Sliding Mode Alpha
        (ParamFloat<px4::params::FA_ALPHA>) _param_fa_alpha,

        (ParamFloat<px4::params::FA_P_X>) _param_fa_p_x,
        (ParamFloat<px4::params::FA_P_Y>) _param_fa_p_y,
        (ParamFloat<px4::params::FA_P_Z>) _param_fa_p_z,

        (ParamFloat<px4::params::FA_V_X>) _param_fa_v_x,
        (ParamFloat<px4::params::FA_V_Y>) _param_fa_v_y,
        (ParamFloat<px4::params::FA_V_Z>) _param_fa_v_z,

        (ParamFloat<px4::params::FA_I_X>) _param_fa_i_x,
        (ParamFloat<px4::params::FA_I_Y>) _param_fa_i_y,
        (ParamFloat<px4::params::FA_I_Z>) _param_fa_i_z,

        (ParamFloat<px4::params::FA_INT_LIM>) _param_fa_int_lim,
        (ParamFloat<px4::params::FA_INT_LIM_R>) _param_fa_int_lim_r,

        (ParamFloat<px4::params::FA_R_R>) _param_fa_r_r,
        (ParamFloat<px4::params::FA_R_P>) _param_fa_r_p,
        (ParamFloat<px4::params::FA_R_Y>) _param_fa_r_y,

        (ParamFloat<px4::params::FA_W_R>) _param_fa_w_r,
        (ParamFloat<px4::params::FA_W_P>) _param_fa_w_p,
        (ParamFloat<px4::params::FA_W_Y>) _param_fa_w_y,
        
        // Attitude Integral Gains
        (ParamFloat<px4::params::FA_I_R_R>) _param_fa_i_r_r,
        (ParamFloat<px4::params::FA_I_R_P>) _param_fa_i_r_p,
        (ParamFloat<px4::params::FA_I_R_Y>) _param_fa_i_r_y,

        // New Sliding Mode Gains
        (ParamFloat<px4::params::FA_K3_R>) _param_fa_k3_r,
        (ParamFloat<px4::params::FA_K3_P>) _param_fa_k3_p,
        (ParamFloat<px4::params::FA_K3_Y>) _param_fa_k3_y,

        (ParamFloat<px4::params::FA_K4_R>) _param_fa_k4_r,
        (ParamFloat<px4::params::FA_K4_P>) _param_fa_k4_p,
        (ParamFloat<px4::params::FA_K4_Y>) _param_fa_k4_y,

        (ParamFloat<px4::params::FA_THR_MAX_X>) _param_fa_thr_max_x,
        (ParamFloat<px4::params::FA_THR_MAX_Y>) _param_fa_thr_max_y,
        (ParamFloat<px4::params::FA_THR_MAX_Z>) _param_fa_thr_max_z,

        (ParamFloat<px4::params::FA_TRQ_MAX_R>) _param_fa_trq_max_r,
        (ParamFloat<px4::params::FA_TRQ_MAX_P>) _param_fa_trq_max_p,
        (ParamFloat<px4::params::FA_TRQ_MAX_Y>) _param_fa_trq_max_y
    );

};

// helper to compute |x|^alpha * sign(x)
inline float frac_sgn(float x, float alpha) {
    if (std::abs(x) < 1e-9f) {
        return 0.0f;
    }

    float sign = (x > 0.0f) ? 1.0f : -1.0f;
    return std::pow(std::abs(x), alpha) * sign;
}