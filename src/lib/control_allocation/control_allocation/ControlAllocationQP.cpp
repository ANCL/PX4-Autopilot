#include "ControlAllocationQP.hpp"

#include <cmath>
#include <limits>
#include <algorithm>
#include <cstdio> 

#include "generated/fa_closest_workspace.h"
#include "generated/fa_margin_workspace.h"
#include "generated/fa_problem_data.h"
#include "generated/inc/public/osqp.h"

#include <px4_platform_common/log.h>

void
ControlAllocationQP::setEffectivenessMatrix(
    const matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> &effectiveness,
    const ActuatorVector &actuator_trim,
    const ActuatorVector &linearization_point,
    int num_actuators,
    bool update_normalization_scale)
{
    ControlAllocation::setEffectivenessMatrix(
        effectiveness, actuator_trim, linearization_point,
        num_actuators, update_normalization_scale);

    (void)update_normalization_scale;
    _matrix_consistent = checkMatrixConsistency(effectiveness, actuator_trim, linearization_point);
}

bool 
ControlAllocationQP::checkMatrixConsistency(
    const matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> &effectiveness,
    const ActuatorVector &actuator_trim,
    const ActuatorVector &linearization_point)
{
    if (_num_actuators != FA_NP) {
        return false;
    }

    // check that effectiveness matrix matches precomputed FA_B
    for (int axis = 0; axis < FA_WRENCH_DIM; ++axis) {
        for (int actuator = 0; actuator < FA_NP; ++actuator) {
            // FA_B uses unit vectors, but PX4's effectiveness matrix has max thrust (CT) baked in
            // multiply FA_B by FA_MU_MAX to scale it to N / Nm for a valid comparison
            float expected_effectiveness = FA_B[axis][actuator] * FA_MU_MAX[actuator];
            if (fabsf(effectiveness(axis, actuator) - expected_effectiveness) > FA_MATRIX_TOL) {
                printEffectivenessDifference(effectiveness); 
                return false;
            }
        }
    }

    // check trims & tolerances match
    for (int actuator = 0; actuator < FA_NP; ++actuator) {
        if (fabsf(actuator_trim(actuator)) > FA_MATRIX_TOL || fabsf(linearization_point(actuator)) > FA_MATRIX_TOL) {
            return false;
        }
    }

    return true;
}

void
ControlAllocationQP::allocate()
{
    _prev_actuator_sp = _actuator_sp;

    if (!_matrix_consistent || _num_actuators != FA_NP) {
        _actuator_sp = _prev_actuator_sp;
        _last_valid_result = false;
        PX4_ERR("QP: Control allocation matrix inconsistent");
        return;
    }

    OSQPFloat control_des[FA_WRENCH_DIM] = {0};
    OSQPFloat control_scaled[FA_WRENCH_DIM] = {0};
    prepareControlSetpoints(control_des, control_scaled);

    OSQPFloat mu_projected[FA_NP] = {0};
    OSQPFloat closest_error = 0.0f;
    
    // solve the bounded closest-control projection
    if (!solveClosestControl(control_des, mu_projected, closest_error)) {
        _actuator_sp = _prev_actuator_sp;
        _last_valid_result = false;
        return;
    }

    bool feasible = (closest_error <= FA_FEASIBILITY_TOL);
    OSQPFloat saturation_margin = std::numeric_limits<OSQPFloat>::quiet_NaN();
    OSQPFloat margin_error = 0.0f;

    // if feasible and we have redundant actuators, optimize saturation margin
    if (feasible && NUM_ACTUATORS > 6) {
        solveMarginControl(control_des, control_scaled, mu_projected, margin_error, saturation_margin);
    }

    // final safety checks and output mapping
    if (!verifyActuatorBounds()) {
        _actuator_sp = _prev_actuator_sp;
        _last_valid_result = false;
        return;
    }

    mapToNormalizedOutputs();

    _last_valid_result = true;
    _last_feasible = feasible;
    _last_closest_error = static_cast<float>(closest_error);
    _last_margin_error = static_cast<float>(margin_error);
    _last_saturation_margin = static_cast<float>(saturation_margin);
}

void ControlAllocationQP::prepareControlSetpoints(OSQPFloat control_des[FA_WRENCH_DIM], OSQPFloat control_scaled[FA_WRENCH_DIM])
{
    for (int axis = 0; axis < FA_WRENCH_DIM; ++axis) {
        control_des[axis] = static_cast<OSQPFloat>(_control_sp(axis));
        
        if (!PX4_ISFINITE(control_des[axis])) {
            control_des[axis] = 0.0f; 
            //PX4_ERR("QP: control_des[%d] is NaN", axis); disabled for now
        }
        
        control_scaled[axis] = FA_SW[axis] * control_des[axis];
    }
}

bool ControlAllocationQP::solveClosestControl(const OSQPFloat control_des[FA_WRENCH_DIM], OSQPFloat mu_projected[FA_NP], OSQPFloat& closest_error)
{
    OSQPFloat closest_q[FA_NP] = {0};

    // q = -C' Sw' Sw c_des
    for (int actuator = 0; actuator < FA_NP; ++actuator) {
        for (int axis = 0; axis < FA_WRENCH_DIM; ++axis) {
            closest_q[actuator] -= FA_C[axis][actuator] * FA_SW[axis] * FA_SW[axis] * control_des[axis];
        }
    }

    OSQPInt exitflag = osqp_update_data_vec(&fa_closest_solver, closest_q, nullptr, nullptr);
    if (exitflag != 0) return false;

    exitflag = osqp_solve(&fa_closest_solver);

    if (exitflag != 0 || (fa_closest_solver.info->status_val != OSQP_SOLVED && fa_closest_solver.info->status_val != OSQP_SOLVED_INACCURATE)) {
        PX4_ERR("QP: Closest projection failed. OSQP error: %d", fa_closest_solver.info->status_val);
        return false;
    }

    OSQPFloat control_achieved[FA_WRENCH_DIM] = {0};
    OSQPFloat scaled_error_squared = 0.0f;

    for (int actuator = 0; actuator < FA_NP; ++actuator) {
        mu_projected[actuator] = fa_closest_solver.solution->x[actuator];
        _actuator_sp(actuator) = static_cast<float>(mu_projected[actuator]);
    }

    for (int axis = 0; axis < FA_WRENCH_DIM; ++axis) {
        for (int actuator = 0; actuator < FA_NP; ++actuator) {
            control_achieved[axis] += FA_C[axis][actuator] * mu_projected[actuator];
        }
        const OSQPFloat error = FA_SW[axis] * (control_achieved[axis] - control_des[axis]);
        scaled_error_squared += error * error;
    }

    closest_error = sqrtf(scaled_error_squared);
	if (closest_error > FA_FEASIBILITY_TOL) {
		PX4_WARN("QP: Infeasible wrench received. Error: %.6f", static_cast<double>(closest_error));
	}

    return true;
}

bool ControlAllocationQP::solveMarginControl(
    const OSQPFloat control_des[FA_WRENCH_DIM], 
    const OSQPFloat control_scaled[FA_WRENCH_DIM], 
    const OSQPFloat mu_projected[FA_NP], 
    OSQPFloat& margin_error, 
    OSQPFloat& saturation_margin)
{
    OSQPFloat margin_l[FA_MARGIN_CONSTRAINTS] = {0};
    OSQPFloat margin_u[FA_MARGIN_CONSTRAINTS] = {0};

    for (int axis = 0; axis < FA_WRENCH_DIM; ++axis) {
        margin_l[axis] = control_scaled[axis];
        margin_u[axis] = control_scaled[axis];
    }

    for (int actuator = 0; actuator < FA_NP; ++actuator) {
        margin_l[FA_WRENCH_DIM + actuator] = -OSQP_INFTY;
        margin_u[FA_WRENCH_DIM + actuator] = -FA_MU_MIN[actuator];
        
        margin_l[FA_WRENCH_DIM + FA_NP + actuator] = -OSQP_INFTY;
        margin_u[FA_WRENCH_DIM + FA_NP + actuator] = FA_MU_MAX[actuator];
    }

    margin_l[FA_MARGIN_CONSTRAINTS - 1] = 0.0f;
    margin_u[FA_MARGIN_CONSTRAINTS - 1] = 0.5f;

    if (osqp_update_data_vec(&fa_margin_solver, nullptr, margin_l, margin_u) != 0) return false;
    if (osqp_solve(&fa_margin_solver) != 0) return false;

    if (fa_margin_solver.info->status_val == OSQP_SOLVED || fa_margin_solver.info->status_val == OSQP_SOLVED_INACCURATE) {
        OSQPFloat control_achieved[FA_WRENCH_DIM] = {0};
        OSQPFloat margin_error_squared = 0.0f;

        for (int actuator = 0; actuator < FA_NP; ++actuator) {
            _actuator_sp(actuator) = static_cast<float>(fa_margin_solver.solution->x[actuator]);
        }

        for (int axis = 0; axis < FA_WRENCH_DIM; ++axis) {
            for (int actuator = 0; actuator < FA_NP; ++actuator) {
                control_achieved[axis] += FA_C[axis][actuator] * _actuator_sp(actuator);
            }
            const OSQPFloat error = FA_SW[axis] * (control_achieved[axis] - control_des[axis]);
            margin_error_squared += error * error;
        }

        margin_error = sqrtf(margin_error_squared);

        if (margin_error <= FA_FEASIBILITY_TOL) {
            saturation_margin = fa_margin_solver.solution->x[FA_NP];
            return true;
        }
    }

    // if margin solve failed or error is too high, revert to closest projection
    for (int actuator = 0; actuator < FA_NP; ++actuator) {
        _actuator_sp(actuator) = static_cast<float>(mu_projected[actuator]);
    }
    return false;
}

bool ControlAllocationQP::verifyActuatorBounds()
{
    for (int actuator = 0; actuator < FA_NP; ++actuator) {
        if (_actuator_sp(actuator) < FA_MU_MIN[actuator] - FA_FEASIBILITY_TOL || 
            _actuator_sp(actuator) > FA_MU_MAX[actuator] + FA_FEASIBILITY_TOL) {
            
            PX4_ERR("QP: Actuator %d bounds outside of tolerance range (Value: %.6f)", 
                    actuator, static_cast<double>(_actuator_sp(actuator)));
            return false;
        }
    }
    return true;
}

void ControlAllocationQP::mapToNormalizedOutputs()
{
    for (int actuator = 0; actuator < FA_NP; ++actuator) {
        float force_n = std::max(0.0f, _actuator_sp(actuator));
        float max_thrust_n = FA_MU_MAX[actuator];

        if (max_thrust_n > 0.001f) {
            // apply inverse quadratic thrust model: u = sqrt(F_desired / F_max)
            float normalized_output = std::sqrt(force_n / max_thrust_n);
            
            // hard clamp to strictly [0.0, 1.0]
			// TODO: change this to clamp to [u_min, u_max]
            _actuator_sp(actuator) = std::max(0.0f, std::min(normalized_output, 1.0f));
        } else {
            _actuator_sp(actuator) = 0.0f;
        }
    }
}

void ControlAllocationQP::printEffectivenessDifference(const matrix::Matrix<float, NUM_AXES, NUM_ACTUATORS> &effectiveness)
{
    PX4_INFO("QP: Effectiveness vs FA_B Difference (effectiveness - expected_effectiveness):");
    
    for (int axis = 0; axis < FA_WRENCH_DIM; ++axis) {
        char buffer[256];
        int offset = 0;
        
        for (int actuator = 0; actuator < FA_NP; ++actuator) {
            float expected_effectiveness = FA_B[axis][actuator] * FA_MU_MAX[actuator];
            float diff = effectiveness(axis, actuator) - expected_effectiveness;
            
            offset += snprintf(buffer + offset, sizeof(buffer) - offset, "%9.5f ", static_cast<double>(diff));
        }
        
        PX4_INFO("  Axis %d: %s", axis, buffer);
    }
}