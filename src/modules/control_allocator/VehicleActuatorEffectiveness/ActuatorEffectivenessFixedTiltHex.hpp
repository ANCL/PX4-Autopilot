#pragma once

#include "ActuatorEffectivenessCustom.hpp"

class ActuatorEffectivenessFixedTiltHex : public ActuatorEffectivenessCustom
{
public:
    explicit ActuatorEffectivenessFixedTiltHex(ModuleParams *parent);
    virtual ~ActuatorEffectivenessFixedTiltHex() = default;

    bool getEffectivenessMatrix(Configuration &configuration, EffectivenessUpdateReason external_update) override;

    const char *name() const override
    {
        return "FixedTiltHex";
    }

private:
    void updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp,
		int matrix_index, ActuatorVector &actuator_sp, const matrix::Vector<float, NUM_ACTUATORS> &actuator_min,
		const matrix::Vector<float, NUM_ACTUATORS> &actuator_max);
};