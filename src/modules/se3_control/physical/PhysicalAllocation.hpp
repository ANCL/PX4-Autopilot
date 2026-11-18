// Physical rotor-thrust allocation for the onboard SE3 PHYSICAL controller.
//
// The wrench W = [f, Mx, My, Mz] (collective thrust along -body z [N], body
// FRD moment [N m]) maps linearly to the rotor thrusts T [N]:
//
//   W = B T,  B_0i = -n_i,z,  B_(1..3)i = r_i x n_i - k_i n_i
//
// with rotor position r_i, thrust axis n_i and reaction coefficient k_i. For
// four rotors B is square and T = B^-1 W. Feasibility is enforced in thrust
// coordinates, where the rotor limits [T(u_min), T(u_max)] form a box and the
// map to W is exactly linear. Each feasible thrust is then converted with the
// inverse thrust curve, so no linearization of the motor model is involved.
//
// Desaturation priorities, following PX4's MC_AIRMODE = 0 policy:
//   1. Roll/pitch moment: scaled down as a vector (direction preserved) only
//      when no collective at or below the request makes it feasible.
//   2. Collective thrust: reduced as needed to fit roll/pitch; never raised,
//      except to the all-rotors-idle minimum.
//   3. Yaw moment: scaled down to fit, without changing 1 or 2.
// The achieved wrench is always reported; nothing is clipped silently.

#pragma once

#include "PhysicalModel.hpp"

namespace se3_physical
{

struct Wrench {
	float thrust{0.f};            // Collective thrust along -body z [N].
	matrix::Vector3f moment{};    // Body FRD moment [N m].
};

struct AllocationResult {
	float rotor_thrust[kRotorCount] {};  // [N]
	float command[kRotorCount] {};       // Normalized actuator_motors command.
	Wrench achieved{};                   // B T for the rotor thrusts above.
	float roll_pitch_scale{1.f};         // Allocated fraction of the roll/pitch moment.
	float yaw_scale{1.f};                // Allocated fraction of the yaw moment.
	bool thrust_limited{false};          // Collective differs from the request.
};

class Allocation
{
public:
	// Precompute B, B^-1 and rotor limits. Returns false (and stays
	// unconfigured) for an invalid model or a singular rotor geometry.
	bool configure(const Model &model);

	bool configured() const { return _configured; }

	// Returns false for an unconfigured allocation or a non-finite request.
	bool allocate(const Wrench &request, AllocationResult &result) const;

	// Wrench produced by the given rotor thrusts.
	Wrench wrench(const float thrust[kRotorCount]) const;

	// Rotor thrusts produced by normalized motor commands (no range check).
	void thrust_from_commands(const float command[kRotorCount], float thrust[kRotorCount]) const;

	const matrix::SquareMatrix<float, kRotorCount> &effectiveness() const { return _effectiveness; }
	const matrix::SquareMatrix<float, kRotorCount> &inverse() const { return _inverse; }

private:
	Model _model{};
	matrix::SquareMatrix<float, kRotorCount> _effectiveness{};
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
