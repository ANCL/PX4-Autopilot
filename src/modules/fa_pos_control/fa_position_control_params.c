/**
 * @file fa_position_control_params.c
 *
 * Parameters for the fully actuated hexarotor position controller.
 *
 * @author Your Name <ddwalton@ualberta.ca>
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
PARAM_DEFINE_FLOAT(FA_MASS, 2.84f);

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
PARAM_DEFINE_FLOAT(FA_P_X, 16.0f);

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
PARAM_DEFINE_FLOAT(FA_P_Y, 16.0f);

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
PARAM_DEFINE_FLOAT(FA_P_Z, 8.0f);

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
 * Attitude P Gain Roll
 *
 * Proportional gain for attitude matrix error around Roll axis.
 *
 * @group Fully Actuated Control
 * @min 0.0
 * @max 30.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(FA_R_R, 6.0f);

/**
 * Attitude P Gain Pitch
 *
 * Proportional gain for attitude matrix error around Pitch axis.
 *
 * @group Fully Actuated Control
 * @min 0.0
 * @max 30.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(FA_R_P, 6.0f);

/**
 * Attitude P Gain Yaw
 *
 * Proportional gain for attitude matrix error around Yaw axis.
 *
 * @group Fully Actuated Control
 * @min 0.0
 * @max 30.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(FA_R_Y, 12.0f);

/**
 * Angular Rate P Gain Roll
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
 * Angular Rate P Gain Pitch
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
 * Angular Rate P Gain Yaw
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
PARAM_DEFINE_FLOAT(FA_THR_MAX_X, 5.5f);

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
PARAM_DEFINE_FLOAT(FA_THR_MAX_Y, 4.8f);

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
PARAM_DEFINE_FLOAT(FA_THR_MAX_Z, 35.3f);

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
PARAM_DEFINE_FLOAT(FA_TRQ_MAX_R, 1.0f);

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
PARAM_DEFINE_FLOAT(FA_TRQ_MAX_P, 1.0f);

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
PARAM_DEFINE_FLOAT(FA_TRQ_MAX_Y, 1.0f);

/**
 * Minimum Hover Thrust
 *
 * Lower bound constraint for the adaptive hover thrust estimate.
 *
 * @group Fully Actuated Control
 * @min 0.01
 * @max 0.50
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(FA_HOVER_MIN, 0.1f);

/**
 * Maximum Hover Thrust
 *
 * Upper bound constraint for the adaptive hover thrust estimate.
 *
 * @group Fully Actuated Control
 * @min 0.50
 * @max 0.95
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(FA_HOVER_MAX, 0.9f);

/**
 * Use Hover Thrust Estimator
 *
 * If enabled, uses the system's adaptive hover thrust estimate for stabilization.
 * If disabled, relies strictly on the static FA_MASS and FA_THR_MAX_* parameters.
 *
 * @group Fully Actuated Control
 * @boolean
 */
PARAM_DEFINE_INT32(FA_HVR_THR_ON, 0);