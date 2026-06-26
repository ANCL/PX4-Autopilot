#pragma once

#include "../ManualPosition/FlightTaskManualPosition.hpp"
#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_attitude_setpoint.h>

class FlightTaskFAPosition : public FlightTaskManualPosition
{
public:
    FlightTaskFAPosition() = default;
    virtual ~FlightTaskFAPosition() = default;

    bool activate(const trajectory_setpoint_s &last_setpoint) override;
    bool update() override;
    
private:
    void applyFullyActuatedSetpointPolicy();

	// state variables for acceleration smoothing
    matrix::Vector3f _smoothed_acceleration_sp{};
    matrix::Vector3f _prev_velocity_sp{};
    
    // time constant for the low-pass filter (seconds)
    static constexpr float ACCEL_FILTER_TAU = 0.6f;

    uORB::Publication<vehicle_attitude_setpoint_s> _vehicle_attitude_setpoint_pub{ORB_ID(vehicle_attitude_setpoint)};
};