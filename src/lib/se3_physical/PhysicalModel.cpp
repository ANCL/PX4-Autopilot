#include "PhysicalModel.hpp"

#include <cmath>

namespace se3_physical
{
namespace
{

// Gazebo F450: Tools/simulation/gz/models/f450/model.sdf with airframe
// 4022_gz_f450. The SDF uses FLU; values below are converted to FRD.
//
// Mass: base_link 2.0232 kg + 4 rotor links of 0.0126 kg = 2.0736 kg.
//
// Inertia about the composite centre of mass (0.28 mm above the base_link
// origin; the offset is neglected): base_link diag(0.0206535, 0.0206535,
// 0.040464) plus each rotor's parallel-axis term at (+-0.16263, +-0.16263,
// 0.0115) m and its own inertia, with the transverse part averaged over the
// spin angle. No products of inertia.
//
// Thrust: the Gazebo motor model applies T = k_f w^2 with motorConstant
// k_f = 1.2e-5 N s^2. PX4 commands w = 150 + 850 u rad/s (SIM_GZ_EC_MIN/MAX,
// THR_MDL_FAC = 0), so T(u) = k_f (150 + 850 u)^2 = 0.27 + 3.06 u + 8.67 u^2.
// Measured Position-mode hover in SITL: u = 0.5889; model: 0.5893.
//
// Reaction torque: momentConstant 0.0137 m (Q = 0.0137 T about the rotor
// axis). Rotors 0 and 1 turn counter-clockwise seen from above (SDF "ccw").
//
// Not modelled: rotor drag and rolling moment, the 12.5 ms motor time
// constant, the motor model's airspeed thrust reduction, and the 1 rad/s
// integer quantization of the Gazebo velocity command.
constexpr float kF450Arm = 0.1626345597f;      // [m]
constexpr float kF450RotorZ = -0.0112f;        // Rotor plane above the CoM [m].
constexpr float kF450Reaction = 0.0137f;       // [m]
constexpr ThrustCurve kF450Thrust{0.27f, 3.06f, 8.67f, 0.f, 1.f};

// The thrust curve is valid only for this airframe and output mapping.
constexpr RequiredParameter kF450Required[] {
	{"SYS_AUTOSTART", 4022},
	{"CA_AIRFRAME", 0},
	{"CA_ROTOR_COUNT", 4},
	{"SIM_GZ_EC_FUNC1", 101}, {"SIM_GZ_EC_FUNC2", 102}, {"SIM_GZ_EC_FUNC3", 103}, {"SIM_GZ_EC_FUNC4", 104},
	{"SIM_GZ_EC_MIN1", 150}, {"SIM_GZ_EC_MIN2", 150}, {"SIM_GZ_EC_MIN3", 150}, {"SIM_GZ_EC_MIN4", 150},
	{"SIM_GZ_EC_MAX1", 1000}, {"SIM_GZ_EC_MAX2", 1000}, {"SIM_GZ_EC_MAX3", 1000}, {"SIM_GZ_EC_MAX4", 1000},
};

Model gazebo_f450()
{
	Model model{};
	model.name = "Gazebo F450";
	model.mass = 2.0736f;

	model.inertia.setZero();
	model.inertia(0, 0) = 0.022078f;
	model.inertia(1, 1) = 0.022078f;
	model.inertia(2, 2) = 0.043301f;

	const matrix::Vector3f up{0.f, 0.f, -1.f};

	// actuator_motors index -> position (FRD) and reaction sign.
	// 0 front right CCW, 1 back left CCW, 2 front left CW, 3 back right CW.
	const float x[kRotorCount] {kF450Arm, -kF450Arm, kF450Arm, -kF450Arm};
	const float y[kRotorCount] {kF450Arm, -kF450Arm, -kF450Arm, kF450Arm};
	const float reaction[kRotorCount] {kF450Reaction, kF450Reaction, -kF450Reaction, -kF450Reaction};

	for (int i = 0; i < kRotorCount; ++i) {
		model.rotors[i].position = matrix::Vector3f{x[i], y[i], kF450RotorZ};
		model.rotors[i].axis = up;
		model.rotors[i].reaction = reaction[i];
		model.rotors[i].thrust = kF450Thrust;
	}

	model.required = kF450Required;
	model.required_count = sizeof(kF450Required) / sizeof(kF450Required[0]);
	return model;
}

bool valid_curve(const ThrustCurve &curve)
{
	// T'(u) = a1 + 2 a2 u is smallest at u_min for a convex curve.
	return std::isfinite(curve.a0) && std::isfinite(curve.a1) && std::isfinite(curve.a2) &&
	       std::isfinite(curve.u_min) && std::isfinite(curve.u_max) &&
	       curve.u_min >= 0.f && curve.u_max <= 1.f && curve.u_min < curve.u_max &&
	       curve.a2 >= 0.f && curve.a1 + 2.f * curve.a2 * curve.u_min > 0.f &&
	       rotor_thrust(curve, curve.u_min) >= 0.f;
}

} // namespace

bool get_model(uint8_t id, Model &model)
{
	switch (static_cast<ModelId>(id)) {
	case ModelId::GazeboF450:
		model = gazebo_f450();
		return true;
	}

	return false;
}

bool valid(const Model &model)
{
	if (!std::isfinite(model.mass) || model.mass <= 0.f || !model.inertia.isAllFinite()) {
		return false;
	}

	// Symmetric positive definite inertia (Sylvester's criterion).
	const matrix::Matrix3f &J = model.inertia;

	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			if (fabsf(J(i, j) - J(j, i)) > 1e-6f) {
				return false;
			}
		}
	}

	const float minor2 = J(0, 0) * J(1, 1) - J(0, 1) * J(1, 0);
	const float minor3 = J(0, 0) * (J(1, 1) * J(2, 2) - J(1, 2) * J(2, 1)) -
			     J(0, 1) * (J(1, 0) * J(2, 2) - J(1, 2) * J(2, 0)) +
			     J(0, 2) * (J(1, 0) * J(2, 1) - J(1, 1) * J(2, 0));

	if (J(0, 0) <= 0.f || minor2 <= 0.f || minor3 <= 0.f) {
		return false;
	}

	for (const Rotor &rotor : model.rotors) {
		// Lateral thrust is not allocated, so rotor axes must be vertical.
		if (!rotor.position.isAllFinite() || !rotor.axis.isAllFinite() || !std::isfinite(rotor.reaction) ||
		    fabsf(rotor.axis.norm() - 1.f) > 1e-4f || fabsf(rotor.axis(0)) > 1e-3f || fabsf(rotor.axis(1)) > 1e-3f ||
		    rotor.axis(2) >= 0.f || !valid_curve(rotor.thrust)) {
			return false;
		}
	}

	return model.required_count >= 0 && (model.required_count == 0 || model.required != nullptr);
}

float rotor_thrust(const ThrustCurve &curve, float command)
{
	return curve.a0 + command * (curve.a1 + command * curve.a2);
}

bool rotor_command(const ThrustCurve &curve, float thrust, float &command)
{
	const float minimum = rotor_thrust(curve, curve.u_min);
	const float maximum = rotor_thrust(curve, curve.u_max);

	if (!std::isfinite(thrust) || thrust < minimum || thrust > maximum) {
		return false;
	}

	// Larger root of a2 u^2 + a1 u + (a0 - T) = 0, in a form without
	// cancellation for either sign of a1 (and valid for a2 = 0 when a1 > 0).
	const float excess = thrust - curve.a0;
	const float root = sqrtf(fmaxf(curve.a1 * curve.a1 + 4.f * curve.a2 * excess, 0.f));
	const float candidate = curve.a1 >= 0.f ?
				2.f * excess / (curve.a1 + root) :
				(root - curve.a1) / (2.f * curve.a2);

	if (!std::isfinite(candidate)) {
		return false;
	}

	// Absorb single-precision rounding at the range limits.
	command = fminf(fmaxf(candidate, curve.u_min), curve.u_max);
	return true;
}

} // namespace se3_physical
