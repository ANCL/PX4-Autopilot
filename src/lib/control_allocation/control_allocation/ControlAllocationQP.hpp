#pragma once

#include "ControlAllocation.hpp"

class ControlAllocationQP : public ControlAllocation
{
public:
	ControlAllocationQP() = default;
	~ControlAllocationQP() override = default;

	void allocate() override;

	void setEffectivenessMatrix(
		const matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> &effectiveness,
		const ActuatorVector &actuator_trim,
		const ActuatorVector &linearization_point,
		int num_actuators,
		bool update_normalization_scale) override;

private:
	bool _matrix_consistent{false};
	bool _last_feasible{false};
	bool _last_valid_result{false};
	float _last_scaled_error{0.f};
	float _last_saturation_margin{0.f};
};
