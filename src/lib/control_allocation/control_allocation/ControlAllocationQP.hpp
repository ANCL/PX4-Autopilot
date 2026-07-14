#pragma once

#include "ControlAllocation.hpp"


#include "generated/fa_closest_workspace.h"
#include "generated/fa_margin_workspace.h"
#include "generated/fa_problem_data.h"
#include "generated/inc/public/osqp.h"

class ControlAllocationQP : public ControlAllocation
{
public:
    friend class ControlAllocationQPTest;
	
    ControlAllocationQP() = default;
	~ControlAllocationQP() override = default;

    // main allocation loop: tries closest projection, optimizes margin, maps outputs
	void allocate() override;

    // configure allocator and verify geometry matches fixed qp data
	void setEffectivenessMatrix(
		const matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> &effectiveness,
		const ActuatorVector &actuator_trim,
		const ActuatorVector &linearization_point,
		int num_actuators,
		bool update_normalization_scale) override;
    
    
    bool isLastResultValid() {
        return _last_valid_result;
    }
    
    float getLastClosestError() {
        return _last_closest_error;
    }

    float getLastMarginError() {
        return _last_closest_error;
    }
private:

    // ensure runtime px4 matrix and bounds match generated python constants
    bool checkMatrixConsistency(const matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> &effectiveness,
                                const ActuatorVector &actuator_trim,
                                const ActuatorVector &linearization_point);
    
    // extract requested wrench, handle nans, and apply axis weights
    void prepareControlSetpoints(OSQPFloat control_des[FA_WRENCH_DIM], OSQPFloat control_scaled[FA_WRENCH_DIM]);

    // project requested wrench into achievable physical limits using bounded osqp solver
    bool solveClosestControl(const OSQPFloat control_des[FA_WRENCH_DIM], 
                             OSQPFloat mu_projected[FA_NP], 
                             OSQPFloat& closest_error);

    // maximize distance to actuator limits for redundant geometries
    bool solveMarginControl(const OSQPFloat control_des[FA_WRENCH_DIM], 
                            const OSQPFloat control_scaled[FA_WRENCH_DIM], 
                            const OSQPFloat mu_projected[FA_NP], 
                            OSQPFloat& margin_error, 
                            OSQPFloat& saturation_margin);

    // strictly ensure qp solver respected physical constraints before passing to mixer
    bool verifyActuatorBounds();

    // convert optimal physical forces into normalized [0, 1] signals for the flight controller
    void mapToNormalizedOutputs();

	bool _matrix_consistent{false};
	bool _last_feasible{false};
	bool _last_valid_result{false};
	float _last_saturation_margin{0.f};
    // last scaled error from the closest point solver
	float _last_closest_error{0.f};
    // last scaled error from the margin solver
    float _last_margin_error{0.f};
};
