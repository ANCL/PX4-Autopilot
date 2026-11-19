// Physical allocation from PX4's rotor effectiveness, desaturation and ownership.

#include <gtest/gtest.h>

#include "PhysicalAllocation.hpp"

#include <cmath>

using namespace se3_physical;

namespace
{

constexpr float kGravity = 9.80665f;
constexpr float kMass = 2.0736f;      // SE3_MASS default (Gazebo F450).
constexpr float kArm = 0.16263456f;   // 4022_gz_f450 CA_ROTOR*_PX/PY
constexpr float kRotorZ = -0.0112f;   // CA_ROTOR*_PZ
constexpr float kCt = 12.f;           // CA_ROTOR*_CT [N]
constexpr float kKm = 0.0137f;        // |CA_ROTOR*_KM| [m]

struct Rotor {
	matrix::Vector3f position;
	matrix::Vector3f axis;
	float ct;
	float km;
};

// 4022_gz_f450 rotors in actuator_motors order: FR, BL, FL, BR; 0 and 1 CCW.
void f450_rotors(Rotor rotors[kRotorCount])
{
	const float x[kRotorCount] {kArm, -kArm, kArm, -kArm};
	const float y[kRotorCount] {kArm, -kArm, -kArm, kArm};
	const float km[kRotorCount] {kKm, kKm, -kKm, -kKm};

	for (int i = 0; i < kRotorCount; ++i) {
		rotors[i] = Rotor{matrix::Vector3f{x[i], y[i], kRotorZ}, matrix::Vector3f{0.f, 0.f, -1.f}, kCt, km[i]};
	}
}

// Same columns as ActuatorEffectivenessRotors::computeEffectivenessMatrix():
// moment ct (r x n) - ct km n, thrust ct n.
Effectiveness effectiveness(const Rotor rotors[kRotorCount])
{
	Effectiveness E;

	for (int i = 0; i < kRotorCount; ++i) {
		const matrix::Vector3f moment = rotors[i].position.cross(rotors[i].axis) * rotors[i].ct
						- rotors[i].axis * (rotors[i].ct * rotors[i].km);
		const matrix::Vector3f thrust = rotors[i].axis * rotors[i].ct;

		for (int j = 0; j < 3; ++j) {
			E(j, i) = moment(j);
			E(j + 3, i) = thrust(j);
		}
	}

	return E;
}

Effectiveness f450()
{
	Rotor rotors[kRotorCount];
	f450_rotors(rotors);
	return effectiveness(rotors);
}

const float kMinimum[kRotorCount] {0.f, 0.f, 0.f, 0.f};
const float kMaximum[kRotorCount] {1.f, 1.f, 1.f, 1.f};

Allocation configured(const Effectiveness &E = f450())
{
	Allocation allocation;
	EXPECT_TRUE(allocation.configure(E, kMinimum, kMaximum));
	return allocation;
}

AllocationResult allocate(const Allocation &allocation, float thrust, const matrix::Vector3f &moment)
{
	AllocationResult result{};
	EXPECT_TRUE(allocation.allocate(Wrench{thrust, moment}, result));
	return result;
}

void expect_within_limits(const AllocationResult &result)
{
	for (int i = 0; i < kRotorCount; ++i) {
		EXPECT_GE(result.command[i], 0.f) << "rotor " << i;
		EXPECT_LE(result.command[i], 1.f) << "rotor " << i;
		EXPECT_NEAR(result.rotor_thrust[i], kCt * result.command[i], 1e-5f) << "rotor " << i;
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
	return kMass * kGravity;
}

} // namespace

TEST(PhysicalAllocation, EffectivenessGeometryAndSigns)
{
	Rotor rotors[kRotorCount];
	f450_rotors(rotors);
	const Allocation allocation = configured();
	const matrix::SquareMatrix<float, kRotorCount> &B = allocation.effectiveness();

	for (int i = 0; i < kRotorCount; ++i) {
		// Upward thrust CT u at (x, y): roll moment -y CT u, pitch moment
		// x CT u, reaction yaw moment +KM CT u (PX4 sign: KM > 0 is CCW).
		EXPECT_FLOAT_EQ(B(0, i), kCt);
		EXPECT_NEAR(B(1, i), -rotors[i].position(1) * kCt, 1e-5f);
		EXPECT_NEAR(B(2, i), rotors[i].position(0) * kCt, 1e-5f);
		EXPECT_NEAR(B(3, i), rotors[i].km * kCt, 1e-6f);
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

TEST(PhysicalAllocation, RotorThrustIsLinearInTheMotorCommand)
{
	// No thrust curve here: THR_MDL_FAC is applied once by the output driver.
	const Allocation allocation = configured();

	for (const float command : {0.f, 0.25f, 0.5f, 1.f}) {
		for (int i = 0; i < kRotorCount; ++i) {
			EXPECT_FLOAT_EQ(allocation.rotor_thrust(i, command), kCt * command);
		}
	}

	// Doubling CT halves the commands for the same wrench.
	Rotor rotors[kRotorCount];
	f450_rotors(rotors);

	for (Rotor &rotor : rotors) { rotor.ct *= 2.f; }

	const AllocationResult nominal = allocate(allocation, weight(), {0.1f, -0.05f, 0.02f});
	const AllocationResult doubled = allocate(configured(effectiveness(rotors)), weight(), {0.1f, -0.05f, 0.02f});

	for (int i = 0; i < kRotorCount; ++i) {
		EXPECT_NEAR(doubled.command[i], 0.5f * nominal.command[i], 1e-6f);
		EXPECT_NEAR(doubled.rotor_thrust[i], nominal.rotor_thrust[i], 1e-5f);
	}
}

TEST(PhysicalAllocation, InvalidEffectivenessAndLimitsAreRejected)
{
	Rotor rotors[kRotorCount];
	Allocation allocation;

	// Lateral thrust (tilted rotor) is not allocated.
	f450_rotors(rotors);
	rotors[2].axis = matrix::Vector3f{0.1f, 0.f, -1.f}.normalized();
	EXPECT_FALSE(allocation.configure(effectiveness(rotors), kMinimum, kMaximum));

	// Downward thrust.
	f450_rotors(rotors);
	rotors[1].axis = matrix::Vector3f{0.f, 0.f, 1.f};
	EXPECT_FALSE(allocation.configure(effectiveness(rotors), kMinimum, kMaximum));

	// A failed motor's zeroed column (control_allocator) is singular.
	Effectiveness failed = f450();

	for (int row = 0; row < 6; ++row) { failed(row, 3) = 0.f; }

	EXPECT_FALSE(allocation.configure(failed, kMinimum, kMaximum));

	// Collinear rotors cannot produce independent moments.
	f450_rotors(rotors);
	rotors[2].position = rotors[0].position;
	rotors[2].km = rotors[0].km;
	EXPECT_FALSE(allocation.configure(effectiveness(rotors), kMinimum, kMaximum));

	// Non-finite entries, reversible or empty command ranges.
	Effectiveness invalid = f450();
	invalid(0, 0) = NAN;
	EXPECT_FALSE(allocation.configure(invalid, kMinimum, kMaximum));

	const float reversible[kRotorCount] {-1.f, 0.f, 0.f, 0.f};
	EXPECT_FALSE(allocation.configure(f450(), reversible, kMaximum));

	const float empty[kRotorCount] {1.f, 1.f, 1.f, 1.f};
	EXPECT_FALSE(allocation.configure(f450(), empty, kMaximum));
	EXPECT_FALSE(allocation.configured());

	// A later valid configuration succeeds.
	EXPECT_TRUE(allocation.configure(f450(), kMinimum, kMaximum));
	EXPECT_TRUE(allocation.configured());
}

TEST(PhysicalAllocation, HoverAndMomentDirections)
{
	const Allocation allocation = configured();

	const AllocationResult hover = allocate(allocation, weight(), {});

	for (int i = 0; i < kRotorCount; ++i) {
		EXPECT_NEAR(hover.rotor_thrust[i], weight() / kRotorCount, 1e-4f);
		EXPECT_NEAR(hover.command[i], weight() / (kRotorCount * kCt), 1e-5f);
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
	const Allocation allocation = configured();

	for (int k = 0; k < 64; ++k) {
		const float phase = 0.37f * static_cast<float>(k);
		const float thrust = weight() * (0.7f + 0.5f * (0.5f + 0.5f * sinf(1.3f * phase)));
		const matrix::Vector3f moment{0.6f * sinf(phase), 0.6f * cosf(0.7f * phase), 0.08f * sinf(2.1f * phase)};

		const AllocationResult result = allocate(allocation, thrust, moment);
		expect_within_limits(result);
		expect_wrench(result.achieved, thrust, moment, 2e-4f);
		EXPECT_FLOAT_EQ(result.roll_pitch_scale, 1.f);
		EXPECT_FLOAT_EQ(result.yaw_scale, 1.f);
		EXPECT_FALSE(result.thrust_limited);

		// The reported wrench is the one produced by the motor commands.
		expect_wrench(allocation.wrench(result.command), thrust, moment, 2e-4f);
	}
}

TEST(PhysicalAllocation, LowThrustScalesRollPitchWithoutRaisingThrust)
{
	const Allocation allocation = configured();
	const float thrust = 6.f;
	const matrix::Vector3f moment{2.f, -1.f, 0.f};

	const AllocationResult result = allocate(allocation, thrust, moment);
	expect_within_limits(result);

	EXPECT_LT(result.roll_pitch_scale, 1.f);
	EXPECT_GT(result.roll_pitch_scale, 0.f);
	EXPECT_LE(result.achieved.thrust, thrust + 1e-4f);
	EXPECT_NEAR(result.achieved.thrust, thrust, 1e-3f);

	// Direction kept, magnitude reduced by exactly the reported scale.
	EXPECT_NEAR(result.achieved.moment(0), result.roll_pitch_scale * moment(0), 1e-4f);
	EXPECT_NEAR(result.achieved.moment(1), result.roll_pitch_scale * moment(1), 1e-4f);

	// The scale is maximal: some motor sits at its minimum command.
	float minimum = INFINITY;

	for (const float command : result.command) { minimum = fminf(minimum, command); }

	EXPECT_NEAR(minimum, 0.f, 1e-5f);
}

TEST(PhysicalAllocation, HighThrustReducesCollectiveBeforeRollPitch)
{
	const Allocation allocation = configured();
	const float thrust = 46.f;
	const matrix::Vector3f moment{1.f, 0.5f, 0.f};

	const AllocationResult result = allocate(allocation, thrust, moment);
	expect_within_limits(result);

	EXPECT_FLOAT_EQ(result.roll_pitch_scale, 1.f);
	EXPECT_TRUE(result.thrust_limited);
	EXPECT_LT(result.achieved.thrust, thrust);
	EXPECT_NEAR(result.achieved.moment(0), moment(0), 1e-4f);
	EXPECT_NEAR(result.achieved.moment(1), moment(1), 1e-4f);

	float maximum = 0.f;

	for (const float command : result.command) { maximum = fmaxf(maximum, command); }

	EXPECT_NEAR(maximum, 1.f, 1e-5f);
}

TEST(PhysicalAllocation, YawHasLowestPriority)
{
	const Allocation allocation = configured();
	const matrix::Vector3f moment{0.3f, 0.2f, 2.f};

	const AllocationResult result = allocate(allocation, weight(), moment);
	expect_within_limits(result);

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
	const Allocation allocation = configured();

	const AllocationResult high = allocate(allocation, 100.f, {});
	EXPECT_NEAR(high.achieved.thrust, kRotorCount * kCt, 1e-4f);
	EXPECT_TRUE(high.thrust_limited);

	const AllocationResult low = allocate(allocation, -5.f, {});
	EXPECT_NEAR(low.achieved.thrust, 0.f, 1e-5f);
	EXPECT_TRUE(low.thrust_limited);

	for (const float command : low.command) { EXPECT_FLOAT_EQ(command, 0.f); }

	// Non-zero minimum commands (e.g. idle) are honoured.
	Allocation idle;
	const float minimum[kRotorCount] {0.1f, 0.1f, 0.1f, 0.1f};
	ASSERT_TRUE(idle.configure(f450(), minimum, kMaximum));
	EXPECT_NEAR(allocate(idle, 0.f, {}).achieved.thrust, kRotorCount * kCt * 0.1f, 1e-4f);

	AllocationResult result{};
	EXPECT_FALSE(allocation.allocate(Wrench{NAN, {}}, result));
	EXPECT_FALSE(allocation.allocate(Wrench{weight(), {0.f, INFINITY, 0.f}}, result));

	const Allocation unconfigured{};
	EXPECT_FALSE(unconfigured.allocate(Wrench{weight(), {}}, result));
}

TEST(PhysicalAllocation, OwnershipRequiresCurrentModeWithoutRateControl)
{
	OwnershipState state{};
	state.input_available = true;
	state.setpoint_received = true;
	state.setpoint_nav_state = 23;
	state.nav_state = 23;
	state.armed = true;
	state.allocation_enabled = true;
	state.rates_enabled = false;
	EXPECT_TRUE(physical_owns_allocation(state));

	OwnershipState changed = state;
	changed.rates_enabled = true;       // PX4 rate controller active (incl. SE3 fallback)
	EXPECT_FALSE(physical_owns_allocation(changed));

	changed = state;
	changed.nav_state = 2;              // Another mode (Position) selected
	EXPECT_FALSE(physical_owns_allocation(changed));

	changed = state;
	changed.armed = false;
	EXPECT_FALSE(physical_owns_allocation(changed));

	changed = state;
	changed.allocation_enabled = false;
	EXPECT_FALSE(physical_owns_allocation(changed));

	changed = state;
	changed.setpoint_received = false;  // Never received
	EXPECT_FALSE(physical_owns_allocation(changed));
}

TEST(PhysicalAllocation, UnavailableWrenchInputNeverOwnsAllocation)
{
	// A failed se3_wrench_setpoint callback registration leaves native
	// allocation running and never hands the motors to the physical path.
	OwnershipState state{};
	state.input_available = false;
	state.setpoint_received = true;
	state.setpoint_nav_state = 23;
	state.nav_state = 23;
	state.armed = true;
	state.allocation_enabled = true;
	state.rates_enabled = false;
	EXPECT_FALSE(physical_owns_allocation(state));
}

TEST(PhysicalAllocation, StalePhysicalCommandsAreNotPublished)
{
	const uint64_t now = 1000000;
	EXPECT_TRUE(physical_command_fresh(now, now));
	EXPECT_TRUE(physical_command_fresh(now, now - kWrenchTimeout));
	EXPECT_FALSE(physical_command_fresh(now, now - kWrenchTimeout - 1)); // Updates stopped
	EXPECT_FALSE(physical_command_fresh(now, now + 1));                  // From the future
	EXPECT_FALSE(physical_command_fresh(now, 0));                        // Never allocated
}
