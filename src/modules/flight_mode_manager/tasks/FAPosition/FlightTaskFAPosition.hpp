#pragma once

#include "../ManualPosition/FlightTaskManualPosition.hpp"

class FlightTaskFAPosition : public FlightTaskManualPosition
{
public:
	FlightTaskFAPosition() = default;
	virtual ~FlightTaskFAPosition() = default;

	bool activate(const trajectory_setpoint_s &last_setpoint) override;
	bool update() override;
	
private:
	void applyFullyActuatedSetpointPolicy();
}; 