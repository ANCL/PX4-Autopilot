#include "FlightTaskFAPosition.hpp"
#include "uORB/topics/vehicle_rates_setpoint.h"

bool FlightTaskFAPosition::activate(const trajectory_setpoint_s &last_setpoint)
{
    // ensure the parent class (ManualPosition) successfully activates first
    if (!FlightTaskManualPosition::activate(last_setpoint)) {
        return false;
    }
	// initialize smoothing states to prevent initial spikes
    _prev_velocity_sp = _velocity_setpoint;
    _smoothed_acceleration_sp.zero();
    
    // FA-specific initialization can be added here later.
    // First version keeps activation behavior identical to ManualPosition.
	    
	return true;
}

bool FlightTaskFAPosition::update()
{
    // update the parent class to calculate standard pos/vel/yaw setpoints
    if (!FlightTaskManualPosition::update()) {
        return false;
    }

    // apply any overrides or constraints specific to a Fully Actuated vehicle
    applyFullyActuatedSetpointPolicy();
    return true;
}

void FlightTaskFAPosition::applyFullyActuatedSetpointPolicy()
{    
    // Limit horizontal (XY) velocity to 0.5 m/s
    const float max_xy_vel = 0.5f;
    matrix::Vector2f vel_xy(_velocity_setpoint(0), _velocity_setpoint(1));
    if (vel_xy.length() > max_xy_vel) {
        vel_xy = vel_xy.normalized() * max_xy_vel;
        _velocity_setpoint.xy() = vel_xy;
    }

    // Limit vertical (Z) velocity to 0.5 m/s
    const float max_z_vel = 0.5f;
    _velocity_setpoint(2) = math::constrain(_velocity_setpoint(2), -max_z_vel, max_z_vel);

    // Limit yaw rate to 20 deg/s
    const float max_yaw_rate = math::radians(20.0f); 
    _yawspeed_setpoint = math::constrain(_yawspeed_setpoint, -max_yaw_rate, max_yaw_rate);

    // Calculate smooth acceleration setpoint
    matrix::Vector3f raw_accel = (_velocity_setpoint - _prev_velocity_sp) / _deltatime;
    _prev_velocity_sp = _velocity_setpoint;

    for (int i = 0; i < 3; ++i) {
        if (!PX4_ISFINITE(raw_accel(i))) {
            raw_accel(i) = 0.0f;
        }
    }

    float alpha = _deltatime / (_deltatime + ACCEL_FILTER_TAU);

    _smoothed_acceleration_sp = _smoothed_acceleration_sp + alpha * (raw_accel - _smoothed_acceleration_sp);

    _acceleration_setpoint = _smoothed_acceleration_sp;
    
    _jerk_setpoint.zero();
}