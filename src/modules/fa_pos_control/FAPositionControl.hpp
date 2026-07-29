/**
 * @file FAPositionControl.hpp
 *
 * A custom position controller intended for a fully-actuated hexarotor
 */

#pragma once

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

    // Limits
    matrix::Vector3f _thrust_maximums;
    matrix::Vector3f _torque_maximums;
    float _int_limit;               // maximum allowed value for the integral state
    float _int_limit_r;             // anti-windup limit for attitude
    
    // Gains
    matrix::Vector3f _k_p;          // position P gain
    matrix::Vector3f _k_v;          // position D gain
    matrix::Vector3f _k_i;          // position I gain
    matrix::Vector3f _k_r;          // attitude P gain
    matrix::Vector3f _k_w;          // attitude D gain
    matrix::Vector3f _k_i_r;        // attitude I gain
    
    // Errors
    matrix::Vector3f _e_p;          // position error
    matrix::Vector3f _e_v;          // velocitiy error
    matrix::Vector3f _e_R;          // rotational error
    matrix::Vector3f _e_w;          // angular vel error
    matrix::Vector3f _e_p_int{};    // accumulated position error
    matrix::Vector3f _e_R_int{};    // attitude error accumulator

    // Setpoints
    matrix::Vector3f _vehicle_thrust_setpoint; // normalized [-1, 1]
    matrix::Vector3f _vehicle_torque_setpoint; // normalized [-1, 1]
    
    // Timestamp
    hrt_abstime _time_stamp_last_loop{0};

    // uORB Subscriptions
    uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

    uORB::Subscription _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint)};
    uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
    uORB::Subscription _vehicle_attitude_setpoint_sub{ORB_ID(vehicle_attitude_setpoint)};
    uORB::Subscription _vehicle_rates_setpoint_sub{ORB_ID(vehicle_rates_setpoint)};
    
    uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
    uORB::Subscription _hover_thrust_estimate_sub{ORB_ID(hover_thrust_estimate)};
    uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
    
    // uORB Callbacks
    uORB::SubscriptionCallbackWorkItem _vehicle_local_position_sub{this, ORB_ID(vehicle_local_position)};

    // uORB Publishers
    uORB::Publication<vehicle_thrust_setpoint_s>  _vehicle_thrust_setpoint_pub{ORB_ID(vehicle_thrust_setpoint)};
    uORB::Publication<vehicle_torque_setpoint_s>  _vehicle_torque_setpoint_pub{ORB_ID(vehicle_torque_setpoint)};

    // PX4 Params
    DEFINE_PARAMETERS(
        (ParamFloat<px4::params::FA_MASS>) _param_fa_mass,

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

        (ParamFloat<px4::params::FA_I_R_R>) _param_fa_i_r_r,
        (ParamFloat<px4::params::FA_I_R_P>) _param_fa_i_r_p,
        (ParamFloat<px4::params::FA_I_R_Y>) _param_fa_i_r_y,

        (ParamFloat<px4::params::FA_THR_MAX_X>) _param_fa_thr_max_x,
        (ParamFloat<px4::params::FA_THR_MAX_Y>) _param_fa_thr_max_y,
        (ParamFloat<px4::params::FA_THR_MAX_Z>) _param_fa_thr_max_z,

        (ParamFloat<px4::params::FA_TRQ_MAX_R>) _param_fa_trq_max_r,
        (ParamFloat<px4::params::FA_TRQ_MAX_P>) _param_fa_trq_max_p,
        (ParamFloat<px4::params::FA_TRQ_MAX_Y>) _param_fa_trq_max_y
    );

};