// Physical model, thrust-curve inversion, rotor allocation and ownership.

#include <gtest/gtest.h>

#include "PhysicalAllocation.hpp"

#include <cmath>

using namespace se3_physical;

namespace
{

constexpr float kGravity = 9.80665f;

Model f450()
{
	Model model{};
	EXPECT_TRUE(get_model(static_cast<uint8_t>(ModelId::GazeboF450), model));
	return model;
}

Allocation configured(const Model &model)
{
	Allocation allocation;
	EXPECT_TRUE(allocation.configure(model));
	return allocation;
}

AllocationResult allocate(const Allocation &allocation, float thrust, const matrix::Vector3f &moment)
{
	AllocationResult result{};
	EXPECT_TRUE(allocation.allocate(Wrench{thrust, moment}, result));
	return result;
}

void expect_within_limits(const Model &model, const AllocationResult &result)
{
	for (int i = 0; i < kRotorCount; ++i) {
		const ThrustCurve &curve = model.rotors[i].thrust;
		EXPECT_GE(result.rotor_thrust[i], rotor_thrust(curve, curve.u_min) - 1e-5f) << "rotor " << i;
		EXPECT_LE(result.rotor_thrust[i], rotor_thrust(curve, curve.u_max) + 1e-5f) << "rotor " << i;
		EXPECT_GE(result.command[i], curve.u_min) << "rotor " << i;
		EXPECT_LE(result.command[i], curve.u_max) << "rotor " << i;
		EXPECT_NEAR(rotor_thrust(curve, result.command[i]), result.rotor_thrust[i], 2e-4f) << "rotor " << i;
	}
}

void expect_wrench(const Wrench &actual, float thrust, const matrix::Vector3f &moment, float tolerance)
{
	EXPECT_NEAR(actual.thrust, thrust, tolerance);

	for (int axis = 0; axis < 3; ++axis) {
		EXPECT_NEAR(actual.moment(axis), moment(axis), tolerance) << "axis " << axis;
	}
}

float weight()
{
	return f450().mass * kGravity;
}

} // namespace

TEST(PhysicalAllocation, GazeboF450ModelMatchesSimulation)
{
	const Model model = f450();
	EXPECT_TRUE(valid(model));
	EXPECT_FLOAT_EQ(model.mass, 2.0736f);
	EXPECT_NEAR(model.inertia(0, 0), 0.022078f, 1e-6f);
	EXPECT_NEAR(model.inertia(2, 2), 0.043301f, 1e-6f);

	// T(u) = 1.2e-5 (150 + 850 u)^2 at both ends of the command range.
	EXPECT_NEAR(rotor_thrust(model.rotors[0].thrust, 0.f), 0.27f, 1e-6f);
	EXPECT_NEAR(rotor_thrust(model.rotors[0].thrust, 1.f), 12.f, 1e-5f);

	// Hover command; SITL Position-mode hover measured 0.5889.
	float command = 0.f;
	EXPECT_TRUE(rotor_command(model.rotors[0].thrust, weight() / kRotorCount, command));
	EXPECT_NEAR(command, 0.5893f, 2e-4f);

	Model unknown{};
	EXPECT_FALSE(get_model(200, unknown));
}

TEST(PhysicalAllocation, ThrustCurveInversion)
{
	const ThrustCurve curves[] {
		{0.27f, 3.06f, 8.67f, 0.f, 1.f},  // Gazebo F450
		{0.5f, -0.8f, 12.f, 0.1f, 0.95f}, // Negative linear term, increasing from u_min
		{0.f, 10.f, 0.f, 0.f, 1.f},       // Linear
	};

	for (const ThrustCurve &curve : curves) {
		for (int k = 0; k <= 100; ++k) {
			const float u = curve.u_min + (curve.u_max - curve.u_min) * static_cast<float>(k) / 100.f;
			float inverse = -1.f;
			ASSERT_TRUE(rotor_command(curve, rotor_thrust(curve, u), inverse)) << "u " << u;
			EXPECT_NEAR(inverse, u, 2e-5f) << "u " << u;
		}

		float command = 0.f;
		EXPECT_FALSE(rotor_command(curve, rotor_thrust(curve, curve.u_min) - 1e-3f, command));
		EXPECT_FALSE(rotor_command(curve, rotor_thrust(curve, curve.u_max) + 1e-3f, command));
		EXPECT_FALSE(rotor_command(curve, NAN, command));
	}
}

TEST(PhysicalAllocation, InvalidModelsAreRejected)
{
	Allocation allocation;

	Model model = f450();
	model.mass = 0.f;
	EXPECT_FALSE(valid(model));
	EXPECT_FALSE(allocation.configure(model));
	EXPECT_FALSE(allocation.configured());

	model = f450();
	model.inertia(0, 1) = 0.01f;    // Asymmetric
	EXPECT_FALSE(valid(model));

	model = f450();
	model.inertia(0, 1) = model.inertia(1, 0) = 0.03f; // Symmetric, not positive definite
	EXPECT_FALSE(valid(model));

	model = f450();
	model.rotors[2].thrust.a1 = -1.f;  // Decreasing at u_min = 0
	EXPECT_FALSE(valid(model));

	model = f450();
	model.rotors[1].thrust.a2 = -1.f;  // Concave
	EXPECT_FALSE(valid(model));

	model = f450();
	model.rotors[3].axis = matrix::Vector3f{0.1f, 0.f, -1.f}.normalized(); // Lateral thrust
	EXPECT_FALSE(valid(model));

	model = f450();
	model.rotors[0].thrust.u_max = 1.2f;  // Beyond the actuator_motors range
	EXPECT_FALSE(valid(model));

	// Valid data, singular geometry: all rotors on one side cannot balance roll.
	model = f450();

	for (Rotor &rotor : model.rotors) {
		rotor.position(1) = 0.16f;
	}

	EXPECT_TRUE(valid(model));
	EXPECT_FALSE(allocation.configure(model));

	model = f450();
	model.rotors[0].position(0) = NAN;
	EXPECT_FALSE(valid(model));
}

TEST(PhysicalAllocation, EffectivenessGeometryAndSigns)
{
	const Model model = f450();
	const Allocation allocation = configured(model);
	const matrix::SquareMatrix<float, kRotorCount> &B = allocation.effectiveness();

	for (int i = 0; i < kRotorCount; ++i) {
		const Rotor &rotor = model.rotors[i];
		// Upward thrust T at (x, y): roll moment -y T, pitch moment x T,
		// reaction yaw moment +k T for a counter-clockwise rotor.
		EXPECT_FLOAT_EQ(B(0, i), 1.f);
		EXPECT_NEAR(B(1, i), -rotor.position(1), 1e-6f);
		EXPECT_NEAR(B(2, i), rotor.position(0), 1e-6f);
		EXPECT_NEAR(B(3, i), rotor.reaction, 1e-6f);
	}

	// Rotor order of 4022_gz_f450 (CA_ROTOR*): FR, BL, FL, BR; 0 and 1 CCW.
	EXPECT_GT(B(2, 0), 0.f); EXPECT_LT(B(1, 0), 0.f); EXPECT_GT(B(3, 0), 0.f);
	EXPECT_LT(B(2, 1), 0.f); EXPECT_GT(B(1, 1), 0.f); EXPECT_GT(B(3, 1), 0.f);
	EXPECT_GT(B(2, 2), 0.f); EXPECT_GT(B(1, 2), 0.f); EXPECT_LT(B(3, 2), 0.f);
	EXPECT_LT(B(2, 3), 0.f); EXPECT_LT(B(1, 3), 0.f); EXPECT_LT(B(3, 3), 0.f);

	const matrix::SquareMatrix<float, kRotorCount> identity = B * allocation.inverse();

	for (int i = 0; i < kRotorCount; ++i) {
		for (int j = 0; j < kRotorCount; ++j) {
			EXPECT_NEAR(identity(i, j), i == j ? 1.f : 0.f, 1e-5f);
		}
	}
}

TEST(PhysicalAllocation, HoverAndMomentDirections)
{
	const Model model = f450();
	const Allocation allocation = configured(model);

	const AllocationResult hover = allocate(allocation, weight(), {});

	for (int i = 0; i < kRotorCount; ++i) {
		EXPECT_NEAR(hover.rotor_thrust[i], weight() / kRotorCount, 1e-4f);
		EXPECT_NEAR(hover.command[i], 0.5893f, 2e-4f);
	}

	// Positive roll raises the left rotors (1, 2), positive pitch the front
	// rotors (0, 2), positive yaw the counter-clockwise rotors (0, 1).
	const AllocationResult roll = allocate(allocation, weight(), {0.2f, 0.f, 0.f});
	EXPECT_GT(roll.rotor_thrust[1], hover.rotor_thrust[1]);
	EXPECT_GT(roll.rotor_thrust[2], hover.rotor_thrust[2]);
	EXPECT_LT(roll.rotor_thrust[0], hover.rotor_thrust[0]);
	EXPECT_LT(roll.rotor_thrust[3], hover.rotor_thrust[3]);

	const AllocationResult pitch = allocate(allocation, weight(), {0.f, 0.2f, 0.f});
	EXPECT_GT(pitch.rotor_thrust[0], hover.rotor_thrust[0]);
	EXPECT_GT(pitch.rotor_thrust[2], hover.rotor_thrust[2]);
	EXPECT_LT(pitch.rotor_thrust[1], hover.rotor_thrust[1]);

	const AllocationResult yaw = allocate(allocation, weight(), {0.f, 0.f, 0.05f});
	EXPECT_GT(yaw.rotor_thrust[0], hover.rotor_thrust[0]);
	EXPECT_GT(yaw.rotor_thrust[1], hover.rotor_thrust[1]);
	EXPECT_LT(yaw.rotor_thrust[2], hover.rotor_thrust[2]);
}

TEST(PhysicalAllocation, FeasibleWrenchesAreReproducedExactly)
{
	const Model model = f450();
	const Allocation allocation = configured(model);

	for (int k = 0; k < 64; ++k) {
		const float phase = 0.37f * static_cast<float>(k);
		const float thrust = weight() * (0.7f + 0.5f * (0.5f + 0.5f * sinf(1.3f * phase)));
		const matrix::Vector3f moment{0.6f * sinf(phase), 0.6f * cosf(0.7f * phase), 0.08f * sinf(2.1f * phase)};

		const AllocationResult result = allocate(allocation, thrust, moment);
		expect_within_limits(model, result);
		expect_wrench(result.achieved, thrust, moment, 2e-4f);
		EXPECT_FLOAT_EQ(result.roll_pitch_scale, 1.f);
		EXPECT_FLOAT_EQ(result.yaw_scale, 1.f);
		EXPECT_FALSE(result.thrust_limited);

		// The reported wrench is the one produced by the motor commands.
		float rotor[kRotorCount];
		allocation.thrust_from_commands(result.command, rotor);
		expect_wrench(allocation.wrench(rotor), thrust, moment, 2e-3f);
	}
}

TEST(PhysicalAllocation, LowThrustScalesRollPitchWithoutRaisingThrust)
{
	const Model model = f450();
	const Allocation allocation = configured(model);
	const float thrust = 6.f;
	const matrix::Vector3f moment{2.f, -1.f, 0.f};

	const AllocationResult result = allocate(allocation, thrust, moment);
	expect_within_limits(model, result);

	EXPECT_LT(result.roll_pitch_scale, 1.f);
	EXPECT_GT(result.roll_pitch_scale, 0.f);
	EXPECT_LE(result.achieved.thrust, thrust + 1e-4f);
	EXPECT_NEAR(result.achieved.thrust, thrust, 1e-3f);

	// Direction kept, magnitude reduced by exactly the reported scale.
	EXPECT_NEAR(result.achieved.moment(0), result.roll_pitch_scale * moment(0), 1e-4f);
	EXPECT_NEAR(result.achieved.moment(1), result.roll_pitch_scale * moment(1), 1e-4f);

	// The scale is maximal: some rotor sits at its idle limit.
	float minimum = INFINITY;

	for (const float rotor : result.rotor_thrust) { minimum = fminf(minimum, rotor); }

	EXPECT_NEAR(minimum, 0.27f, 1e-4f);
}

TEST(PhysicalAllocation, HighThrustReducesCollectiveBeforeRollPitch)
{
	const Model model = f450();
	const Allocation allocation = configured(model);
	const float thrust = 46.f;
	const matrix::Vector3f moment{1.f, 0.5f, 0.f};

	const AllocationResult result = allocate(allocation, thrust, moment);
	expect_within_limits(model, result);

	EXPECT_FLOAT_EQ(result.roll_pitch_scale, 1.f);
	EXPECT_TRUE(result.thrust_limited);
	EXPECT_LT(result.achieved.thrust, thrust);
	EXPECT_NEAR(result.achieved.moment(0), moment(0), 1e-4f);
	EXPECT_NEAR(result.achieved.moment(1), moment(1), 1e-4f);

	float maximum = 0.f;

	for (const float rotor : result.rotor_thrust) { maximum = fmaxf(maximum, rotor); }

	EXPECT_NEAR(maximum, 12.f, 1e-4f);
}

TEST(PhysicalAllocation, YawHasLowestPriority)
{
	const Model model = f450();
	const Allocation allocation = configured(model);
	const matrix::Vector3f moment{0.3f, 0.2f, 2.f};

	const AllocationResult result = allocate(allocation, weight(), moment);
	expect_within_limits(model, result);

	EXPECT_FLOAT_EQ(result.roll_pitch_scale, 1.f);
	EXPECT_LT(result.yaw_scale, 1.f);
	EXPECT_GT(result.yaw_scale, 0.f);
	EXPECT_NEAR(result.achieved.thrust, weight(), 1e-3f);
	EXPECT_NEAR(result.achieved.moment(0), moment(0), 1e-4f);
	EXPECT_NEAR(result.achieved.moment(1), moment(1), 1e-4f);
	EXPECT_NEAR(result.achieved.moment(2), result.yaw_scale * moment(2), 1e-4f);
}

TEST(PhysicalAllocation, CollectiveLimitsAndInvalidRequests)
{
	const Model model = f450();
	const Allocation allocation = configured(model);

	const AllocationResult high = allocate(allocation, 100.f, {});
	EXPECT_NEAR(high.achieved.thrust, 48.f, 1e-4f);
	EXPECT_TRUE(high.thrust_limited);

	const AllocationResult low = allocate(allocation, -5.f, {});
	EXPECT_NEAR(low.achieved.thrust, 4.f * 0.27f, 1e-4f);
	EXPECT_TRUE(low.thrust_limited);

	for (const float command : low.command) { EXPECT_FLOAT_EQ(command, 0.f); }

	AllocationResult result{};
	EXPECT_FALSE(allocation.allocate(Wrench{NAN, {}}, result));
	EXPECT_FALSE(allocation.allocate(Wrench{weight(), {0.f, INFINITY, 0.f}}, result));

	const Allocation unconfigured{};
	EXPECT_FALSE(unconfigured.allocate(Wrench{weight(), {}}, result));
}

TEST(PhysicalAllocation, OwnershipRequiresCurrentModeWithoutRateControl)
{
	OwnershipState state{};
	state.now = 1000000;
	state.setpoint_timestamp = state.now - 4000;
	state.setpoint_nav_state = 23;
	state.nav_state = 23;
	state.armed = true;
	state.allocation_enabled = true;
	state.rates_enabled = false;
	EXPECT_TRUE(physical_owns_allocation(state));

	OwnershipState changed = state;
	changed.rates_enabled = true;       // PX4 rate controller active
	EXPECT_FALSE(physical_owns_allocation(changed));

	changed = state;
	changed.nav_state = 2;              // Another mode (Position) selected
	EXPECT_FALSE(physical_owns_allocation(changed));

	changed = state;
	changed.setpoint_timestamp = state.now - kWrenchTimeout - 1; // Stale
	EXPECT_FALSE(physical_owns_allocation(changed));

	changed = state;
	changed.setpoint_timestamp = state.now + 1; // From the future
	EXPECT_FALSE(physical_owns_allocation(changed));

	changed = state;
	changed.armed = false;
	EXPECT_FALSE(physical_owns_allocation(changed));

	changed = state;
	changed.allocation_enabled = false;
	EXPECT_FALSE(physical_owns_allocation(changed));

	changed = state;
	changed.setpoint_timestamp = 0;     // Never received
	EXPECT_FALSE(physical_owns_allocation(changed));
}
