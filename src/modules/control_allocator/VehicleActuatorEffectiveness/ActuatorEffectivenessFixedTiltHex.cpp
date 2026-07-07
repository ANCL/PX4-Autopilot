#include "ActuatorEffectivenessFixedTiltHex.hpp"

ActuatorEffectivenessFixedTiltHex::ActuatorEffectivenessFixedTiltHex(ModuleParams *parent) : 
    ActuatorEffectivenessCustom(parent)
{
}

bool ActuatorEffectivenessFixedTiltHex::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason external_update)
{
	if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
		return false;
	}

	// Motors
	_motors.enablePropellerTorque(true);
	const bool motors_added_successfully = _motors.addActuators(configuration);
	_motors_mask = _motors.getMotors();

	// Torque
	const bool torque_added_successfully = _torque.addActuators(configuration);

	return (motors_added_successfully && torque_added_successfully);
}

void ActuatorEffectivenessFixedTiltHex::updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp,
		int matrix_index, ActuatorVector &actuator_sp, const matrix::Vector<float, NUM_ACTUATORS> &actuator_min,
		const matrix::Vector<float, NUM_ACTUATORS> &actuator_max)
{
}
