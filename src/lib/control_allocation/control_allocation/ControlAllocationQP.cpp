#include "ControlAllocationQP.hpp"

#include <cmath>
#include <limits>


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
		effectiveness,
		actuator_trim,
		linearization_point,
		num_actuators,
		update_normalization_scale);

	/* =====================================================================
	 * BLOCK 1: FIXED-PROBLEM CONSISTENCY CHECK
	 *
	 * Inputs:
	 *   PX4 effectiveness matrix, actuator trim, actuator bounds.
	 *
	 * Purpose:
	 *   Verify that the runtime PX4 configuration matches the fixed matrix and
	 *   bounds used by fa_wrench_codegen.py. The generated QP matrices are
	 *   valid only for this fixed problem family.
	 *
	 * Output:
	 *   _matrix_consistent
	 * ===================================================================== */

	(void)update_normalization_scale;
    _matrix_consistent = (_num_actuators == FA_NP);

    if (_matrix_consistent) {
        for (int axis = 0; axis < FA_WRENCH_DIM; ++axis) {
            for (int actuator = 0; actuator < FA_NP; ++actuator) {
                if (fabsf(_effectiveness(axis, actuator) - FA_B[axis][actuator]) > 0.01f) {
                    _matrix_consistent = false;
                }
            }
        }
    }

    if (_matrix_consistent) {
        for (int actuator = 0; actuator < FA_NP; ++actuator) {
            if (fabsf(actuator_trim(actuator)) > 0.01f
                || fabsf(linearization_point(actuator)) > 0.01f
                || fabsf(_actuator_min(actuator) - FA_MU_MIN[actuator]) > 0.01f
                || fabsf(_actuator_max(actuator) - FA_MU_MAX[actuator]) > 0.01f) {
                _matrix_consistent = false;
            }
        }
    }
}

void
ControlAllocationQP::allocate()
{
	_prev_actuator_sp = _actuator_sp;

	/* =====================================================================
	 * BLOCK 2: RUNTIME CONTROL INPUT AND OUTPUT STORAGE
	 *
	 * Input:
	 *   _control_sp = [tau_x, tau_y, tau_z, thrust_x, thrust_y, thrust_z].
	 *
	 * Purpose:
	 *   Prepare the changing vectors used by the two generated OSQP problems.
	 *
	 * Output:
	 *   selected actuator setpoint in _actuator_sp.
	 * ===================================================================== */

	if (!_matrix_consistent || _num_actuators != FA_NP) {
		_actuator_sp = _prev_actuator_sp;
		_last_valid_result = false;
        PX4_ERR("QP: Control allocation matrix inconsistent");
		return;
	}

	OSQPFloat control_des[FA_WRENCH_DIM] = {0};
	OSQPFloat control_scaled[FA_WRENCH_DIM] = {0};
	OSQPFloat control_achieved[FA_WRENCH_DIM] = {0};
	OSQPFloat mu_projected[FA_NP] = {0};
	OSQPFloat closest_q[FA_NP] = {0};

	for (int axis = 0; axis < FA_WRENCH_DIM; ++axis) {
		control_des[axis] = static_cast<OSQPFloat>(_control_sp(axis));
		control_scaled[axis] = FA_SW[axis] * control_des[axis];
	}

	/* =====================================================================
	 * BLOCK 3: BOUNDED CLOSEST-CONTROL PROJECTION
	 *
	 * Problem:
	 *       minimize 0.5 ||Sw(C mu - c_des)||_2^2
	 *   subject to
	 *       mu_min <= mu <= mu_max
	 *
	 * Runtime update:
	 *       q = -C' Sw' Sw c_des
	 *
	 * Outputs:
	 *   bounded projected allocation and scaled projection error.
	 * ===================================================================== */

    bool wrench_is_corrupted = false;
    for (int axis = 0; axis < FA_WRENCH_DIM; ++axis) {
        if (!PX4_ISFINITE(control_des[axis])) {
            wrench_is_corrupted = true;
            PX4_ERR("QP: control_des[%d] is NaN", axis);
        }
    }

    if (wrench_is_corrupted) {
        // zero out the demands so the drone doesn't spin out of control
        for (int axis = 0; axis < FA_WRENCH_DIM; ++axis) {
            control_des[axis] = 0.0f; 
        }
    }

	for (int actuator = 0; actuator < FA_NP; ++actuator) {
		for (int axis = 0; axis < FA_WRENCH_DIM; ++axis) {
			closest_q[actuator] -= FA_C[axis][actuator]
						       * FA_SW[axis]
						       * FA_SW[axis]
						       * control_des[axis];
            //PX4_INFO("closest_q: axis(%d), actuator(%d) = %.2f", axis, actuator, static_cast<double>(closest_q[actuator]));
		}
	}

	OSQPInt exitflag = osqp_update_data_vec(
		&fa_closest_solver,
		closest_q,
		nullptr,
		nullptr);

	if (exitflag != 0) {
		_actuator_sp = _prev_actuator_sp;
		_last_valid_result = false;
		return;
	}

	exitflag = osqp_solve(&fa_closest_solver);

	if (exitflag != 0
	    || (fa_closest_solver.info->status_val != OSQP_SOLVED
		&& fa_closest_solver.info->status_val != OSQP_SOLVED_INACCURATE)) {
		_actuator_sp = _prev_actuator_sp;
		_last_valid_result = false;

        // logs
        PX4_ERR("QP: Optimization was unable to be solved. OSQP error: %d", fa_closest_solver.info->status_val);

        PX4_INFO("  Iters: %d | Time: %.2f ms | Obj: %.4f (Gap: %.4f)",
                static_cast<int>(fa_closest_solver.info->iter),
                static_cast<double>(fa_closest_solver.info->solve_time * 1000.0f), // Convert seconds to ms
                static_cast<double>(fa_closest_solver.info->obj_val),
                static_cast<double>(fa_closest_solver.info->duality_gap));
                
        PX4_INFO("  Residuals: Primal=%.6f, Dual=%.6f | KKT Err: %.6f",
                static_cast<double>(fa_closest_solver.info->prim_res),
                static_cast<double>(fa_closest_solver.info->dual_res),
                static_cast<double>(fa_closest_solver.info->rel_kkt_error));
                
        PX4_INFO("  Rho: Estimate=%.4f, Updates=%d",
                static_cast<double>(fa_closest_solver.info->rho_estimate),
                static_cast<int>(fa_closest_solver.info->rho_updates));
		return;
	}

	for (int actuator = 0; actuator < FA_NP; ++actuator) {
		mu_projected[actuator] = fa_closest_solver.solution->x[actuator];
		_actuator_sp(actuator) = static_cast<float>(mu_projected[actuator]);
	}

	OSQPFloat scaled_error_squared = 0.0f;

	for (int axis = 0; axis < FA_WRENCH_DIM; ++axis) {
		for (int actuator = 0; actuator < FA_NP; ++actuator) {
			control_achieved[axis] += FA_C[axis][actuator] * mu_projected[actuator];
		}

		const OSQPFloat error = FA_SW[axis] * (control_achieved[axis] - control_des[axis]);
		scaled_error_squared += error * error;
	}

	OSQPFloat scaled_error = sqrtf(scaled_error_squared);
	bool feasible = scaled_error <= FA_FEASIBILITY_TOL;
	OSQPFloat saturation_margin = std::numeric_limits<OSQPFloat>::quiet_NaN();

	/* =====================================================================
	 * BLOCK 4: FEASIBLE REQUEST -> MAXIMUM SATURATION-MARGIN ALLOCATION
	 *
	 * Entered only when the projected control error is within the configured
	 * numerical membership tolerance.
	 *
	 * Problem:
	 *       maximize m
	 *   subject to
	 *       Sw C mu = Sw c_des
	 *       mu_i >= mu_min_i + m(mu_max_i - mu_min_i)
	 *       mu_i <= mu_max_i - m(mu_max_i - mu_min_i)
	 *       0 <= m <= 0.5
	 *
	 * Output:
	 *   exact max-min-margin allocation when the margin solve succeeds.
	 * ===================================================================== */

	if (feasible) {
		OSQPFloat margin_l[FA_MARGIN_CONSTRAINTS] = {0};
		OSQPFloat margin_u[FA_MARGIN_CONSTRAINTS] = {0};

		for (int axis = 0; axis < FA_WRENCH_DIM; ++axis) {
			margin_l[axis] = control_scaled[axis];
			margin_u[axis] = control_scaled[axis];
		}

		for (int actuator = 0; actuator < FA_NP; ++actuator) {
			margin_l[FA_WRENCH_DIM + actuator] = -OSQP_INFTY;
			margin_u[FA_WRENCH_DIM + actuator] = -FA_MU_MIN[actuator];
		}

		for (int actuator = 0; actuator < FA_NP; ++actuator) {
			margin_l[FA_WRENCH_DIM + FA_NP + actuator] = -OSQP_INFTY;
			margin_u[FA_WRENCH_DIM + FA_NP + actuator] = FA_MU_MAX[actuator];
		}

		margin_l[FA_MARGIN_CONSTRAINTS - 1] = 0.0f;
		margin_u[FA_MARGIN_CONSTRAINTS - 1] = 0.5f;

		exitflag = osqp_update_data_vec(
			&fa_margin_solver,
			nullptr,
			margin_l,
			margin_u);
        
		if (exitflag == 0) {
			exitflag = osqp_solve(&fa_margin_solver);
		}

		if (exitflag == 0
		    && (fa_margin_solver.info->status_val == OSQP_SOLVED
			|| fa_margin_solver.info->status_val == OSQP_SOLVED_INACCURATE)) {
            OSQPFloat margin_error_squared = 0.0f;

			for (int axis = 0; axis < FA_WRENCH_DIM; ++axis) {
				control_achieved[axis] = 0.0f;
			}

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

			const OSQPFloat margin_error = sqrtf(margin_error_squared);

			if (margin_error <= FA_FEASIBILITY_TOL) {
				scaled_error = margin_error;
				saturation_margin = fa_margin_solver.solution->x[FA_NP];

			} else {
				for (int actuator = 0; actuator < FA_NP; ++actuator) {
					_actuator_sp(actuator) = static_cast<float>(mu_projected[actuator]);
				}
			}
		}
	}

	/* =====================================================================
	 * BLOCK 5: FINAL NUMERICAL CHECKS AND DIAGNOSTICS
	 *
	 * Checks:
	 *   selected actuator bounds.
	 *
	 * Outputs:
	 *   _actuator_sp and last-solve diagnostics.
	 * ===================================================================== */

	bool bounds_valid = true;

	for (int actuator = 0; actuator < FA_NP; ++actuator) {
		if (_actuator_sp(actuator) < FA_MU_MIN[actuator] - FA_FEASIBILITY_TOL
		    || _actuator_sp(actuator) > FA_MU_MAX[actuator] + FA_FEASIBILITY_TOL) {
			bounds_valid = false;
		}
	}

	if (!bounds_valid) {
		_actuator_sp = _prev_actuator_sp;
		_last_valid_result = false;
		return;
	}

	_last_valid_result = true;
	_last_feasible = feasible;
	_last_scaled_error = static_cast<float>(scaled_error);
	_last_saturation_margin = static_cast<float>(saturation_margin);
}
