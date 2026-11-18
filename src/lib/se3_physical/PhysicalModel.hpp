// Physical multirotor models for the onboard SE3 PHYSICAL controller.
//
// All quantities are SI and expressed in the body FRD frame about the
// centre of mass. A model is plain data; it is selected with SE3_MODEL and
// identified by the same id in se3_wrench_setpoint, so the controller (mass,
// inertia) and control_allocator (rotor geometry, thrust curves) always use
// the same vehicle description.
//
// These are physical propulsion coefficients. They are unrelated to PX4's
// normalized effectiveness parameters (CA_ROTOR*_PX/PY/KM), which only shape
// the native normalized allocation.

#pragma once

#include <matrix/matrix/math.hpp>

#include <cstdint>

namespace se3_physical
{

static constexpr int kRotorCount = 4;

enum class ModelId : uint8_t {
	GazeboF450 = 0,
};

// Rotor thrust as a function of the normalized actuator_motors command u:
//
//   T(u) = a0 + a1 u + a2 u^2 [N],  u in [u_min, u_max]
//
// The curve must be convex (a2 >= 0) and strictly increasing over the range,
// so it has a unique inverse there. THR_MDL_FAC must be 0, otherwise PX4
// applies its own thrust curve to the same command a second time.
struct ThrustCurve {
	float a0{0.f};
	float a1{0.f};
	float a2{0.f};
	float u_min{0.f};
	float u_max{1.f};
};

struct Rotor {
	matrix::Vector3f position{};  // Rotor hub relative to the centre of mass [m].
	matrix::Vector3f axis{};      // Unit thrust direction; upward thrust is (0, 0, -1).

	// Reaction-torque coefficient [m]: the rotor applies -reaction * T * axis.
	// Positive for a rotor turning counter-clockwise seen from above.
	float reaction{0.f};

	ThrustCurve thrust{};
};

// Integer parameter value that the model coefficients depend on (airframe,
// output mapping). PHYSICAL control refuses to run if any value differs.
struct RequiredParameter {
	const char *name;
	int32_t value;
};

struct Model {
	const char *name{nullptr};
	float mass{0.f};              // [kg]
	matrix::Matrix3f inertia{};   // About the centre of mass [kg m^2].
	Rotor rotors[kRotorCount] {}; // In actuator_motors index order.
	const RequiredParameter *required{nullptr};
	int required_count{0};
};

// Returns false for an unknown model id.
bool get_model(uint8_t id, Model &model);

// Physical consistency of mass, inertia, rotor data and thrust curves.
// Allocation-matrix rank is checked by Allocation::configure().
bool valid(const Model &model);

// T(u) [N]. The command is not range-checked.
float rotor_thrust(const ThrustCurve &curve, float command);

// Inverse of T(u) on [u_min, u_max]. Returns false for a non-finite thrust
// or one outside [T(u_min), T(u_max)].
bool rotor_command(const ThrustCurve &curve, float thrust, float &command);

} // namespace se3_physical
