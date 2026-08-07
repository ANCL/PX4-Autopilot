/**
 * @file fa_position_control_params.c
 *
 * Parameters for the fully actuated hexarotor position controller.
 *
 */

/**
 * Vehicle Mass
 *
 * Total physical mass of the hexarotor vehicle.
 *
 * @group Fully Actuated Control
 * @unit kg
 * @min 0.1
 * @max 50.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(FA_MASS, 3.70f);

/**
 * Vehicle Inertia X (Jxx)
 *
 * Moment of inertia around the body X axis.
 *
 * @group Fully Actuated Control
 * @min 0.001
 * @max 5.0
 * @decimal 4
 */
PARAM_DEFINE_FLOAT(FA_J_X, 0.05f);

/**
 * Vehicle Inertia Y (Jyy)
 *
 * Moment of inertia around the body Y axis.
 *
 * @group Fully Actuated Control
 * @min 0.001
 * @max 5.0
 * @decimal 4
 */
PARAM_DEFINE_FLOAT(FA_J_Y, 0.05f);

/**
 * Vehicle Inertia Z (Jzz)
 *
 * Moment of inertia around the body Z axis.
 *
 * @group Fully Actuated Control
 * @min 0.001
 * @max 5.0
 * @decimal 4
 */
PARAM_DEFINE_FLOAT(FA_J_Z, 0.06f);

/**
 * Sliding Mode Fractional Power (Alpha)
 *
 * Fractional power for the sliding mode attitude control law. 
 * Should be strictly between 0 and 1 for finite-time convergence.
 *
 * @group Fully Actuated Control
 * @min 0.1
 * @max 1.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(FA_ALPHA, 0.8f);

/**
 * Position P Gain X
 *
 * Proportional gain for body X-axis position error.
 *
 * @group Fully Actuated Control
 * @min 0.0
 * @max 50.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(FA_P_X, 8.0f);

/**
 * Position P Gain Y
 *
 * Proportional gain for body Y-axis position error.
 *
 * @group Fully Actuated Control
 * @min 0.0
 * @max 50.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(FA_P_Y, 8.0f);

/**
 * Position P Gain Z
 *
 * Proportional gain for body Z-axis position error.
 *
 * @group Fully Actuated Control
 * @min 0.0
 * @max 50.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(FA_P_Z, 4.0f);

/**
 * Velocity P Gain X
 *
 * Proportional gain for body X-axis velocity error (acts as D gain for position).
 *
 * @group Fully Actuated Control
 * @min 0.0
 * @max 20.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(FA_V_X, 8.0f);

/**
 * Velocity P Gain Y
 *
 * Proportional gain for body Y-axis velocity error (acts as D gain for position).
 *
 * @group Fully Actuated Control
 * @min 0.0
 * @max 20.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(FA_V_Y, 8.0f);

/**
 * Velocity P Gain Z
 *
 * Proportional gain for body Z-axis velocity error (acts as D gain for position).
 *
 * @group Fully Actuated Control
 * @min 0.0
 * @max 20.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(FA_V_Z, 4.0f);

/**
 * Position I Gain X
 *
 * Integral gain for body X-axis position error.
 *
 * @group Fully Actuated Control
 * @min 0.0
 * @max 10.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(FA_I_X, 1.0f);

/**
 * Position I Gain Y
 *
 * Integral gain for body Y-axis position error.
 *
 * @group Fully Actuated Control
 * @min 0.0
 * @max 10.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(FA_I_Y, 1.0f);

/**
 * Position I Gain Z
 *
 * Integral gain for body Z-axis position error.
 *
 * @group Fully Actuated Control
 * @min 0.0
 * @max 10.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(FA_I_Z, 1.0f);

/**
 * Position Integral Anti-Windup Limit
 *
 * Maximum allowed value for the position integral state.
 * Set to 0.0 to disable position integration.
 *
 * @group Fully Actuated Control
 * @min 0.0
 * @max 10.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(FA_INT_LIM, 0.0f); // DISABLED BY DEFAULT

/**
 * Attitude Integral Anti-Windup Limit
 *
 * Maximum allowed value for the attitude integral state.
 * Set to 0.0 to disable attitude integration.
 *
 * @group Fully Actuated Control
 * @min 0.0
 * @max 10.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(FA_INT_LIM_R, 1.0f); 

/**
 * Attitude P Gain Roll (k_R)
 *
 * Proportional gain for attitude matrix error around Roll axis.
 *
 * @group Fully Actuated Control
 * @min 0.0
 * @max 30.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(FA_R_R, 2.0f);

/**
 * Attitude P Gain Pitch (k_R)
 *
 * Proportional gain for attitude matrix error around Pitch axis.
 *
 * @group Fully Actuated Control
 * @min 0.0
 * @max 30.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(FA_R_P, 2.0f);

/**
 * Attitude P Gain Yaw (k_R)
 *
 * Proportional gain for attitude matrix error around Yaw axis.
 *
 * @group Fully Actuated Control
 * @min 0.0
 * @max 30.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(FA_R_Y, 2.0f);

/**
 * Angular Rate P Gain Roll (k_W)
 *
 * Proportional gain for angular velocity error around Roll axis.
 *
 * @group Fully Actuated Control
 * @min 0.0
 * @max 10.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(FA_W_R, 1.0f);

/**
 * Angular Rate P Gain Pitch (k_W)
 *
 * Proportional gain for angular velocity error around Pitch axis.
 *
 * @group Fully Actuated Control
 * @min 0.0
 * @max 10.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(FA_W_P, 1.0f);

/**
 * Angular Rate P Gain Yaw (k_W)
 *
 * Proportional gain for angular velocity error around Yaw axis.
 *
 * @group Fully Actuated Control
 * @min 0.0
 * @max 10.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(FA_W_Y, 1.0f);

/**
 * Attitude I Gain Roll
 *
 * Integral gain for attitude matrix error around Roll axis.
 *
 * @group Fully Actuated Control
 * @min 0.0
 * @max 10.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(FA_I_R_R, 0.0f);

/**
 * Attitude I Gain Pitch
 *
 * Integral gain for attitude matrix error around Pitch axis.
 *
 * @group Fully Actuated Control
 * @min 0.0
 * @max 10.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(FA_I_R_P, 0.0f);

/**
 * Attitude I Gain Yaw
 *
 * Integral gain for attitude matrix error around Yaw axis.
 *
 * @group Fully Actuated Control
 * @min 0.0
 * @max 10.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(FA_I_R_Y, 0.0f);

/**
 * Attitude Sliding Mode Gain Roll (k_3)
 *
 * Sliding mode auxiliary gain (k3) for attitude error around Roll axis.
 *
 * @group Fully Actuated Control
 * @min 0.0
 * @max 20.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(FA_K3_R, 0.5f);

/**
 * Attitude Sliding Mode Gain Pitch (k_3)
 *
 * Sliding mode auxiliary gain (k3) for attitude error around Pitch axis.
 *
 * @group Fully Actuated Control
 * @min 0.0
 * @max 20.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(FA_K3_P, 0.5f);

/**
 * Attitude Sliding Mode Gain Yaw (k_3)
 *
 * Sliding mode auxiliary gain (k3) for attitude error around Yaw axis.
 *
 * @group Fully Actuated Control
 * @min 0.0
 * @max 20.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(FA_K3_Y, 0.5f);

/**
 * Angular Rate Sliding Mode Gain Roll (k_4)
 *
 * Sliding mode auxiliary gain (k4) for angular velocity error around Roll axis.
 *
 * @group Fully Actuated Control
 * @min 0.0
 * @max 20.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(FA_K4_R, 0.5f);

/**
 * Angular Rate Sliding Mode Gain Pitch (k_4)
 *
 * Sliding mode auxiliary gain (k4) for angular velocity error around Pitch axis.
 *
 * @group Fully Actuated Control
 * @min 0.0
 * @max 20.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(FA_K4_P, 0.5f);

/**
 * Angular Rate Sliding Mode Gain Yaw (k_4)
 *
 * Sliding mode auxiliary gain (k4) for angular velocity error around Yaw axis.
 *
 * @group Fully Actuated Control
 * @min 0.0
 * @max 20.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(FA_K4_Y, 0.5f);

/**
 * Maximum Thrust X
 *
 * Absolute physical force limit achievable along the body X axis.
 *
 * @group Fully Actuated Control
 * @unit N
 * @min 0.1
 * @max 200.0
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(FA_THR_MAX_X, 61.2f);

/**
 * Maximum Thrust Y
 *
 * Absolute physical force limit achievable along the body Y axis.
 *
 * @group Fully Actuated Control
 * @unit N
 * @min 0.1
 * @max 200.0
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(FA_THR_MAX_Y, 61.2f);

/**
 * Maximum Thrust Z
 *
 * Absolute physical force limit achievable along the body Z axis.
 *
 * @group Fully Actuated Control
 * @unit N
 * @min 0.1
 * @max 500.0
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(FA_THR_MAX_Z, 61.2f);

/**
 * Maximum Torque Roll
 *
 * Absolute physical torque limit achievable around the body Roll axis.
 *
 * @group Fully Actuated Control
 * @unit Nm
 * @min 0.01
 * @max 20.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(FA_TRQ_MAX_R, 2.0f);

/**
 * Maximum Torque Pitch
 *
 * Absolute physical torque limit achievable around the body Pitch axis.
 *
 * @group Fully Actuated Control
 * @unit Nm
 * @min 0.01
 * @max 20.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(FA_TRQ_MAX_P, 2.0f);

/**
 * Maximum Torque Yaw
 *
 * Absolute physical torque limit achievable around the body Yaw axis.
 *
 * @group Fully Actuated Control
 * @unit Nm
 * @min 0.01
 * @max 20.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(FA_TRQ_MAX_Y, 2.0f);