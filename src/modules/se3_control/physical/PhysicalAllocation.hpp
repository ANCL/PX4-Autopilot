// Physical rotor allocation for the onboard SE3 PHYSICAL controller.
//
// The rotor model is PX4's own: control_allocator builds its effectiveness
// matrix from CA_ROTOR*_PX/PY/PZ, CA_ROTOR*_AX/AY/AZ, CA_ROTOR*_CT and
// CA_ROTOR*_KM. Column i is the wrench of rotor i per unit motor command u_i,
//
//   thrust CT_i n_i [N],  moment CT_i (r_i x n_i - KM_i n_i) [N m],
//
// so with CT in newtons the matrix is already SI: rotor thrust is CT_i u_i,
// where u_i is the thrust-linear actuator_motors command. The output driver
// maps u_i to the motor signal with THR_MDL_FAC exactly as for native
// allocation, so the thrust-curve compensation is applied once, natively.
//
// The wrench W = [f, Mx, My, Mz] (collective thrust along -body z [N], body
// FRD moment [N m]) is W = B u with
//
//   B_0i = -E_(z thrust),i,  B_(1..3)i = E_(moment),i.
//
// For four rotors B is square and u = B^-1 W. The motor limits [u_min, u_max]
// form a box in which the map to W is exactly linear, so feasibility is
// enforced exactly.
//
// Desaturation priorities, following PX4's MC_AIRMODE = 0 policy:
//   1. Roll/pitch moment: scaled down as a vector (direction preserved) only
//      when no collective at or below the request makes it feasible.
//   2. Collective thrust: reduced as needed to fit roll/pitch; never raised,
//      except to the all-motors-minimum value.
//   3. Yaw moment: scaled down to fit, without changing 1 or 2.
// The achieved wrench and the scale factors are always reported; nothing is
// clipped silently. SE3 uses them for integrator anti-windup.

#pragma once

#include <matrix/matrix/math.hpp>

#include <cstdint>

namespace se3_physical
{

static constexpr int kRotorCount = 4;

// PX4 effectiveness rows: roll, pitch, yaw moment, then x, y, z thrust.
using Effectiveness = matrix::Matrix<float, 6, kRotorCount>;

struct Wrench {
	float thrust{0.f};            // Collective thrust along -body z [N].
	matrix::Vector3f moment{};    // Body FRD moment [N m].
};

struct AllocationResult {
	float command[kRotorCount] {};       // Thrust-linear actuator_motors command.
	float rotor_thrust[kRotorCount] {};  // [N]
	Wrench achieved{};                   // B u for the commands above.
	float roll_pitch_scale{1.f};         // Allocated fraction of the roll/pitch moment.
	float yaw_scale{1.f};                // Allocated fraction of the yaw moment.
	bool thrust_limited{false};          // Collective differs from the request.
};

class Allocation
{
public:
	// Precompute B, B^-1 and the command limits from PX4's effectiveness of
	// the four motors. Returns false (and stays unconfigured) for lateral or
	// downward rotor thrust, invalid limits, or a singular or ill-conditioned
	// rotor geometry.
	bool configure(const Effectiveness &effectiveness, const float minimum[kRotorCount],
		       const float maximum[kRotorCount]);

	bool configured() const { return _configured; }

	// Returns false for an unconfigured allocation or a non-finite request.
	bool allocate(const Wrench &request, AllocationResult &result) const;

	// Wrench produced by the given motor commands.
	Wrench wrench(const float command[kRotorCount]) const;

	// Thrust of rotor i for a motor command [N].
	float rotor_thrust(int rotor, float command) const { return _matrix(0, rotor) * command; }

	const matrix::SquareMatrix<float, kRotorCount> &effectiveness() const { return _matrix; }
	const matrix::SquareMatrix<float, kRotorCount> &inverse() const { return _inverse; }

private:
	matrix::SquareMatrix<float, kRotorCount> _matrix{};
	matrix::SquareMatrix<float, kRotorCount> _inverse{};
	float _minimum[kRotorCount] {};
	float _maximum[kRotorCount] {};
	bool _configured{false};
};

// control_allocator ownership of the motors.
//
// The physical path owns allocation exactly while the vehicle is armed in the
// navigation state named by the SE3 wrench, with PX4 allocation enabled and the
// PX4 rate controller disabled. In that configuration the native normalized
// path is never used, so the two never command the motors together, and
// nothing else (stale native setpoints included) can drive them.
//
// Within it, a physical motor command is published only while the wrench it
// was allocated from is at most kWrenchTimeout old. After that the allocator
// stops publishing; recovery is the SE3 fault response, which re-enables the
// PX4 rate controller (ending this configuration), or Commander's failsafe.
static constexpr uint64_t kWrenchTimeout = 20000; // [us]

struct OwnershipState {
	bool input_available{false};   // se3_wrench_setpoint callback registered
	bool setpoint_received{false};
	uint8_t setpoint_nav_state{0};
	uint8_t nav_state{0};
	bool armed{false};
	bool allocation_enabled{false};
	bool rates_enabled{true};
};

inline bool physical_owns_allocation(const OwnershipState &state)
{
	return state.input_available && state.setpoint_received && state.armed &&
	       state.allocation_enabled && !state.rates_enabled && state.setpoint_nav_state == state.nav_state;
}

// True if a command allocated from a wrench with this timestamp may still be
// (re)published at time now.
inline bool physical_command_fresh(uint64_t now, uint64_t wrench_timestamp)
{
	return wrench_timestamp > 0 && now >= wrench_timestamp && now - wrench_timestamp <= kWrenchTimeout;
}

} // namespace se3_physical
