#include <gtest/gtest.h>
#include <matrix/math.hpp>
#include <cmath>

#include "ControlAllocation.hpp"
#include "ControlAllocationQP.hpp"

#include <px4_platform_common/log.h>

#define MODULE_NAME "control_allcoation"

using namespace matrix;

class ControlAllocationQPTest : public ::testing::Test {
protected:
    using ActuatorVector = ControlAllocation::ActuatorVector;

    ControlAllocationQP allocator;
    ActuatorVector trim;
    ActuatorVector linearization_point;
    
    void SetUp() override {
        // initialize standard trim and linearization vectors to zero
        trim.setZero();
        linearization_point.setZero();
    }

    // helper function to build a perfect effectiveness matrix matching the python generator
    Matrix<float, ControlAllocation::NUM_AXES, ControlAllocation::NUM_ACTUATORS> createPerfectEffectivenessMatrix() {
        
        Matrix<float, ControlAllocation::NUM_AXES, ControlAllocation::NUM_ACTUATORS> effectiveness;
        effectiveness.setZero(); 

        for (int axis = 0; axis < FA_WRENCH_DIM; ++axis) {
            for (int actuator = 0; actuator < FA_NP; ++actuator) {
                effectiveness(axis, actuator) = static_cast<float>(FA_B[axis][actuator]);
            }
        }
        
        return effectiveness;
    }

    // set of verticies from the zero-moment FWS in N
    Vector3f zero_moment_force_set[8] = {
        Vector3f(  0.0000f,   0.0000f,   0.0000f),
        Vector3f( -7.4626f, -12.9256f, -32.0072f),
        Vector3f( 14.9252f,   0.0000f, -32.0072f),
        Vector3f(  7.4626f, -12.9256f, -64.0143f),
        Vector3f( -7.4626f,  12.9256f, -32.0072f),
        Vector3f(-14.9252f,   0.0000f, -64.0143f),
        Vector3f(  7.4626f,  12.9256f, -64.0143f),
        Vector3f(  0.0000f,   0.0000f, -96.0215f)
    };


    void TearDown() override {
        // prevent OSQP state leakage between tests
        osqp_cold_start(&fa_closest_solver);
        osqp_cold_start(&fa_margin_solver);
    }
};



// ==============================================================================
// Test 1: Matrix Consistency - Perfect Match
// Verifies that the allocator successfully initializes and accepts a perfectly 
// formatted effectiveness matrix without throwing configuration errors.
// ==============================================================================
TEST_F(ControlAllocationQPTest, MatrixConsistencyPerfectMatch) {
    auto effectiveness = createPerfectEffectivenessMatrix();

    allocator.setEffectivenessMatrix(effectiveness, trim, linearization_point, FA_NP, false);

    Vector<float, FA_WRENCH_DIM> control_sp;
    control_sp.setZero();
    allocator.setControlSetpoint(control_sp);
    
    allocator.allocate();
    
    EXPECT_TRUE(allocator.isLastResultValid()) 
        << "Allocator should accept an exact match of the FA_B matrix.";
}

// ==============================================================================
// Test 2: Early Exit on Inconsistent Matrix
// Ensures the allocator has defensive checks against malformed configurations, 
// specifically rejecting matrices where the number of actuators doesn't match 
// the pre-compiled OSQP problem size (FA_NP).
// ==============================================================================
TEST_F(ControlAllocationQPTest, RejectsInconsistentMatrixSize) {
    auto effectiveness = createPerfectEffectivenessMatrix();

    // Pass an incorrect number of actuators (e.g., 4 instead of FA_NP)
    allocator.setEffectivenessMatrix(effectiveness, trim, linearization_point, 4, false);

    Vector<float, FA_WRENCH_DIM> control_sp;
    control_sp.setZero();
    allocator.setControlSetpoint(control_sp);
    
    allocator.allocate();

    EXPECT_FALSE(allocator.isLastResultValid()) 
        << "Allocator must reject allocation if num_actuators != FA_NP.";
}

// ==============================================================================
// Test 3: Feasible Hover Allocation
// Tests a standard flight condition (pure vertical thrust). It validates that a 
// symmetric drone geometry results in perfectly symmetric actuator outputs, 
// and that the outputs exceed the minimum spin requirements.
// ==============================================================================
TEST_F(ControlAllocationQPTest, FeasibleHoverAllocation) {
    auto effectiveness = createPerfectEffectivenessMatrix();
    allocator.setEffectivenessMatrix(effectiveness, trim, linearization_point, FA_NP, false);

    // Request pure Z thrust (index 5)
    Vector<float, FA_WRENCH_DIM> control_sp;
    control_sp.setZero();
    
    // Request a reasonable total thrust in Newtons
    control_sp(5) = -40.0f; 
    
    allocator.setControlSetpoint(control_sp);
    allocator.allocate();

    EXPECT_TRUE(allocator.isLastResultValid());
    
    const ActuatorVector& output = allocator.getActuatorSetpoint();
    
    // Check that all actuators are firing equally and are within normalized bounds [0, 1]
    float first_actuator_output = output(0);
    EXPECT_GT(first_actuator_output, FA_MU_MIN[0]) << "Hover thrust should be greater than MU_MIN.";
    EXPECT_LE(first_actuator_output, FA_MU_MAX[0]) << "Hover thrust exceeds FA_MU_MAX.";

    for (int i = 1; i < FA_NP; ++i) {
        // for a symmetric fixed-tilt hexarotor, this is true
        EXPECT_NEAR(output(i), first_actuator_output, 1e-3f) 
            << "Actuators are not evenly sharing pure hover thrust.";
    }
}

// ==============================================================================
// Test 4: Actuator Output Bounds Check
// Forces the solver into deep saturation by requesting an impossible torque. 
// Validates that the QP constraints hold and no actuator outputs a normalized 
// value outside the physical [0, 1] bounds (accounting for solver tolerance).
// ==============================================================================
TEST_F(ControlAllocationQPTest, OutputStrictlyBounded) {
    auto effectiveness = createPerfectEffectivenessMatrix();
    allocator.setEffectivenessMatrix(effectiveness, trim, linearization_point, FA_NP, false);

    // request an impossibly high torque to force saturation
    Vector<float, FA_WRENCH_DIM> control_sp;
    control_sp.setZero();
    control_sp(0) = 9999.0f; // absurd roll torque
    
    allocator.setControlSetpoint(control_sp);
    allocator.allocate();

    const ActuatorVector& output = allocator.getActuatorSetpoint();
    
    EXPECT_TRUE(allocator.isLastResultValid());
    // ensure the final SI to normalized mapping prevents numbers > 1.0 or < 0.0
    for (int i = 0; i < FA_NP; ++i) {
        EXPECT_GE(output(i), 0.0f) << "Actuator " << i << " is less than 0.";
        EXPECT_LE(output(i), 1.0f) << "Actuator " << i << " is greater than 1.";
    }
}

// ==============================================================================
// Test 5: FWS Exact Vertex
// Probes the exact mathematical boundaries of the zero-moment force workspace.
// The solver should find these points perfectly feasible with minimal error.
// ==============================================================================
TEST_F(ControlAllocationQPTest, FWSExactVertex) {
    auto effectiveness = createPerfectEffectivenessMatrix();
    allocator.setEffectivenessMatrix(effectiveness, trim, linearization_point, FA_NP, false);

    Vector<float, FA_WRENCH_DIM> control_sp;
    control_sp.setZero();

    for (int i = 0; i < 8; ++i) {
        // assign each vertex value (incl 0 vector) to control_sp
        for (int j = 0; j < 3; ++j) {
            // torques remain zero
            control_sp(3 + j) = zero_moment_force_set[i](j); // control_sp = concat(torque[3] + force[3])
        }

        allocator.setControlSetpoint(control_sp);
        allocator.allocate();

        EXPECT_TRUE(allocator.isLastResultValid()) << "Vertex " << i << " failed to solve.";

        // error should be small
        EXPECT_LT(allocator.getLastClosestError(), FA_FEASIBILITY_TOL);
    }
}

// ==============================================================================
// Test 6: FWS Just Inside Boundary
// Verifies that scaling the boundary inward guarantees feasibility. Tests 
// the solver's standard interior-point solving capabilities.
// ==============================================================================
TEST_F(ControlAllocationQPTest, FWSJustInsideBoundary) {
    auto effectiveness = createPerfectEffectivenessMatrix();
    allocator.setEffectivenessMatrix(effectiveness, trim, linearization_point, FA_NP, false);

    Vector<float, FA_WRENCH_DIM> control_sp;
    control_sp.setZero();

    for (int i = 0; i < 8; ++i) {
        // assign each vertex value (incl 0 vector) to control_sp
        for (int j = 0; j < 3; ++j) {
            // torques remain zero, verticies pushed in a very small amount (1%)
            control_sp(3 + j) = 0.9f * zero_moment_force_set[i](j); // control_sp = concat(torque[3] + force[3])
        }

        allocator.setControlSetpoint(control_sp);
        allocator.allocate();

        EXPECT_TRUE(allocator.isLastResultValid()) << "Vertex " << i << " failed to solve.";

        // error should be small
        EXPECT_LT(allocator.getLastClosestError(), FA_FEASIBILITY_TOL);
    }
}

// ==============================================================================
// Test 7: FWS Just Outside Boundary
// Verifies the margin/closest solver fallback mechanism. When requesting an 
// unreachable setpoint, the solver must flag an error greater than the tolerance.
// ==============================================================================
TEST_F(ControlAllocationQPTest, FWSJustOutsideBoundary) {
    auto effectiveness = createPerfectEffectivenessMatrix();
    allocator.setEffectivenessMatrix(effectiveness, trim, linearization_point, FA_NP, false);

    Vector<float, FA_WRENCH_DIM> control_sp;
    control_sp.setZero();

    // not including zero vector, as it should be INSIDE FWS
    for (int i = 1; i < 8; ++i) {
        // assign each vertex value (incl 0 vector) to control_sp
        for (int j = 0; j < 3; ++j) {
            // torques remain zero, verticies pushed in a very small amount (1%)
            control_sp(3 + j) = 1.1f * zero_moment_force_set[i](j); // control_sp = concat(torque[3] + force[3])
        }

        allocator.setControlSetpoint(control_sp);
        allocator.allocate();

        EXPECT_TRUE(allocator.isLastResultValid()) << "Vertex " << i << " failed to solve.";

        // error should be large
        EXPECT_GT(allocator.getLastClosestError(), FA_FEASIBILITY_TOL);
    }
}

// ==============================================================================
// Test 8: Coupled Wrench Prioritization (Saturation Handling)
// Verifies the solver's behavior when requested to produce simultaneous 
// high torques and high thrusts that exceed the vehicle's physical limits.
// It ensures the solver returns a valid bounded solution, correctly flags the 
// state as infeasible, and applies QP prioritization to the achieved wrench.
// ==============================================================================
TEST_F(ControlAllocationQPTest, CoupledWrenchPrioritization) {
    auto effectiveness = createPerfectEffectivenessMatrix();
    allocator.setEffectivenessMatrix(effectiveness, trim, linearization_point, FA_NP, false);

    const float extreme_thrust = -150.0f;   // exceeds total max thrust
    const float extreme_torque = 50.0f; // exceeds single-axis moment
    
    // wrench format: {Roll, Pitch, Yaw, Fx, Fy, Fz}
    const float test_points_data[4][6] = {
        {extreme_torque, 0.0f, 0.0f, 0.0f, 0.0f, extreme_thrust},           // extreme roll + thrust
        {0.0f, extreme_torque, 0.0f, 0.0f, 0.0f, extreme_thrust},           // extreme pitch + thrust
        {0.0f, 0.0f, extreme_torque, 0.0f, 0.0f, extreme_thrust},           // extreme yaw + thrust
        {extreme_torque, extreme_torque, extreme_torque, 0.0f, 0.0f, extreme_thrust} // extreme everything
    };

    const float solver_tolerance = 1e-4f;

    for (size_t i = 0; i < 4; ++i) {
        // Instantiate the matrix::Vector from the raw array row
        Vector<float, FA_WRENCH_DIM> test_point(test_points_data[i]);
        
        allocator.setControlSetpoint(test_point);
        allocator.allocate();

        EXPECT_TRUE(allocator.isLastResultValid()) 
            << "Solver failed to find a valid fallback solution for test point " << i;

        const ActuatorVector& output = allocator.getActuatorSetpoint();

        // outputs must remain strictly bounded despite the impossible request
        for (int j = 0; j < FA_NP; ++j) {
            EXPECT_GE(output(j), 0.0f - solver_tolerance) << "Actuator strictly below 0 at point " << i;
            EXPECT_LE(output(j), 1.0f + solver_tolerance) << "Actuator strictly above 1 at point " << i;
        }

        // because the requested wrench is physically impossible, feasibility error must trigger
        EXPECT_GT(allocator.getLastClosestError(), FA_FEASIBILITY_TOL) 
            << "Expected an infeasible allocation error for over-saturated test point " << i;

        ActuatorVector unnormalized_output;
        for (int j = 0; j < FA_NP; ++j) {
            unnormalized_output(j) = output(j) * FA_MU_MAX[j];
        }

        // calculate what the motors actually achieved in SI units
        Vector<float, FA_WRENCH_DIM> achieved = effectiveness * unnormalized_output;
       
        // check Z-thrust: should be significantly pushing upwards (negative in FRD)
        EXPECT_LT(achieved(5), -40.0f) 
            << "Solver failed to apply significant vertical thrust under saturation at point " << i;

        // check roll torque
        if (std::abs(test_point(0)) > 0.1f) {
            // maximum physical roll torque for this frame is roughly ~12 Nm
            EXPECT_GT(achieved(0), 5.0f) 
                << "Solver failed to apply significant Roll torque under saturation at point " << i;
        }
    }
}

extern "C" {
    void px4_log_modulename(int level, const char *moduleName, const char *fmt, ...) {
        printf("[%s] ", moduleName);
        va_list args;
        va_start(args, fmt);
        vprintf(fmt, args);
        va_end(args);
        printf("\n");
    }
}