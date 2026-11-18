// Consistency checks for the native SE3 mathematics.
//
// The tests exercise only the pure functions (no uORB, parameters or
// scheduling). Analytic derivatives are compared with central finite
// differences of the next-lower analytic quantity, so errors do not compound
// across derivative orders. Tolerances reflect single-precision arithmetic.

#include <gtest/gtest.h>

#include "AttitudeLaws.hpp"
#include "AttitudeReference.hpp"
#include "ControlPath.hpp"
#include "Controller.hpp"
#include "physical/PhysicalAllocation.hpp"
#include "trajectory/Trajectory.hpp"

#include <cmath>

using namespace se3_control;
using trajectory_generator::Reference;
using trajectory_generator::Segment;
using trajectory_generator::Sequence;

namespace
{

constexpr float kGravity = 9.80665f;
constexpr float kStep = 1.0e-3f;

Vector3 vec(const trajectory_generator::Vector3 &v)
{
	return Vector3{v[0], v[1], v[2]};
}

Sequence single_segment(Segment::Type type, float duration)
{
	Sequence sequence{};
	sequence.count = 1;
	sequence.segments[0].type = type;
	sequence.segments[0].duration_s = duration;
	return sequence;
}

Sequence circle(float radius, float turns, float duration)
{
	Sequence sequence = single_segment(Segment::Type::Circle, duration);
	sequence.segments[0].radius_m = radius;
	sequence.segments[0].turns = turns;
	return sequence;
}

Sequence figure_eight(float x_amplitude, float y_amplitude, float turns, float duration)
{
	Sequence sequence = single_segment(Segment::Type::FigureEight, duration);
	sequence.segments[0].x_amplitude_m = x_amplitude;
	sequence.segments[0].y_amplitude_m = y_amplitude;
	sequence.segments[0].turns = turns;
	return sequence;
}

Reference origin()
{
	Reference reference{};
	reference.position = {{0.0f, 0.0f, -0.75f}};
	reference.yaw = 0.3f;
	return reference;
}

Reference sample(const Sequence &sequence, float time_s)
{
	Reference reference{};
	EXPECT_TRUE(trajectory_generator::sample(sequence, time_s, origin(), reference));
	return reference;
}

void expect_near(const Vector3 &actual, const Vector3 &expected, float tolerance)
{
	for (int axis = 0; axis < 3; ++axis) {
		EXPECT_NEAR(actual(axis), expected(axis), tolerance) << "axis " << axis;
	}
}

// Finite-difference comparison. Single-precision central differences with a
// 1 ms step have errors of order 1e-4 relative to the derivative magnitude,
// so the tolerance combines an absolute floor with a relative term. Sign or
// missing-term errors produce discrepancies of order one and remain visible.
void expect_close(const Vector3 &actual, const Vector3 &expected, float absolute, float relative)
{
	for (int axis = 0; axis < 3; ++axis) {
		EXPECT_NEAR(actual(axis), expected(axis), absolute + relative * std::fabs(expected(axis)))
				<< "axis " << axis;
	}
}

// Each analytic derivative must equal the central difference of the one below.
void expect_consistent_derivatives(const Sequence &sequence, float duration)
{
	for (int k = 0; k < 10; ++k) {
		const float t = (0.05f + 0.1f * k) * duration; // Interior samples only.
		const Reference before = sample(sequence, t - kStep);
		const Reference after = sample(sequence, t + kStep);
		const Reference now = sample(sequence, t);
		const float inverse = 0.5f / kStep;

		expect_close((vec(after.position) - vec(before.position)) * inverse, vec(now.velocity), 2e-3f, 5e-4f);
		expect_close((vec(after.velocity) - vec(before.velocity)) * inverse, vec(now.acceleration), 2e-3f, 5e-4f);
		expect_close((vec(after.acceleration) - vec(before.acceleration)) * inverse, vec(now.jerk), 2e-3f, 5e-4f);
		expect_close((vec(after.jerk) - vec(before.jerk)) * inverse, vec(now.snap), 2e-3f, 5e-4f);
	}
}

// Perfect-tracking state: the vehicle is exactly on the reference with the
// desired attitude and angular velocity.
State tracking_state(const Reference &reference, const DesiredAttitudeDynamics &desired)
{
	State state{};
	state.position = vec(reference.position);
	state.velocity = vec(reference.velocity);
	state.attitude = desired.attitude;
	state.angular_velocity = desired.angular_velocity;
	return state;
}

DesiredAttitudeDynamics desired_attitude(const Reference &reference)
{
	const Vector3 gravity{0.0f, 0.0f, kGravity};
	DesiredAttitudeDynamics desired{};
	EXPECT_TRUE(AttitudeReference::compute(
			    vec(reference.acceleration) - gravity, vec(reference.jerk), vec(reference.snap),
			    reference.yaw, reference.yaw_rate, reference.yaw_acceleration, desired));
	return desired;
}

Vector3 vee(const matrix::SquareMatrix<float, 3> &skew)
{
	return Vector3{0.5f * (skew(2, 1) - skew(1, 2)),
		       0.5f * (skew(0, 2) - skew(2, 0)),
		       0.5f * (skew(1, 0) - skew(0, 1))};
}

}  // namespace

TEST(Se3Trajectory, CircleDerivativesAreConsistent)
{
	expect_consistent_derivatives(circle(0.5f, 1.0f, 8.0f), 8.0f);
}

TEST(Se3Trajectory, FigureEightDerivativesAreConsistent)
{
	expect_consistent_derivatives(figure_eight(0.5f, 0.5f, 1.0f, 8.0f), 8.0f);
}

TEST(Se3Trajectory, SmoothSegmentsStartAndEndAtRestAndClose)
{
	for (const Sequence &sequence : {circle(0.5f, 2.0f, 10.0f), figure_eight(0.5f, 0.4f, 2.0f, 10.0f)}) {
		for (float t : {0.0f, 10.0f}) {
			const Reference reference = sample(sequence, t);
			expect_near(vec(reference.velocity), Vector3{}, 1e-5f);
			expect_near(vec(reference.acceleration), Vector3{}, 1e-5f);
			expect_near(vec(reference.jerk), Vector3{}, 1e-5f);
			expect_near(vec(reference.snap), Vector3{}, 1e-5f);
			expect_near(vec(reference.position), vec(origin().position), 1e-5f);
		}
	}
}

TEST(Se3Trajectory, StepIsImmediateAndFinalReferenceIsHeld)
{
	Sequence sequence = single_segment(Segment::Type::Hold, 1.0f);
	sequence.count = 2;
	sequence.segments[1].type = Segment::Type::Step;
	sequence.segments[1].duration_s = 1.0f;
	sequence.segments[1].offset = {{0.25f, 0.0f, 0.0f}};

	EXPECT_FLOAT_EQ(sample(sequence, 0.999f).position[0], 0.0f);
	EXPECT_FLOAT_EQ(sample(sequence, 1.001f).position[0], 0.25f);
	EXPECT_FLOAT_EQ(sample(sequence, 50.0f).position[0], 0.25f);
}

TEST(Se3AttitudeReference, HoverMatchesHeading)
{
	Reference reference{};
	reference.yaw = 0.7f;
	const DesiredAttitudeDynamics desired = desired_attitude(reference);
	const matrix::Eulerf euler{desired.attitude};

	EXPECT_NEAR(euler.phi(), 0.0f, 1e-6f);
	EXPECT_NEAR(euler.theta(), 0.0f, 1e-6f);
	EXPECT_NEAR(euler.psi(), 0.7f, 1e-6f);
}

// Omega_d and dOmega_d are compared with finite differences of R_d and Omega_d.
TEST(Se3AttitudeReference, AngularVelocityAndAccelerationMatchFiniteDifferences)
{
	const Sequence sequence = figure_eight(1.0f, 0.8f, 1.0f, 6.0f);

	for (int k = 1; k < 10; ++k) {
		const float t = 0.6f * k; // Interior samples of the 6 s segment.
		const DesiredAttitudeDynamics before = desired_attitude(sample(sequence, t - kStep));
		const DesiredAttitudeDynamics after = desired_attitude(sample(sequence, t + kStep));
		const DesiredAttitudeDynamics now = desired_attitude(sample(sequence, t));

		const matrix::SquareMatrix<float, 3> rotation_rate =
			(matrix::SquareMatrix<float, 3>(after.attitude) - matrix::SquareMatrix<float, 3>(before.attitude))
			* (0.5f / kStep);
		const Vector3 omega = vee(now.attitude.transpose() * rotation_rate);

		expect_close(omega, now.angular_velocity, 2e-3f, 1e-3f);
		expect_close((after.angular_velocity - before.angular_velocity) * (0.5f / kStep),
			     now.angular_acceleration, 5e-3f, 2e-3f);

		// The desired rotation remains orthonormal and right-handed.
		const matrix::SquareMatrix<float, 3> identity = now.attitude.transpose() * now.attitude;

		for (int row = 0; row < 3; ++row) {
			for (int column = 0; column < 3; ++column) {
				EXPECT_NEAR(identity(row, column), row == column ? 1.0f : 0.0f, 1e-5f);
			}
		}

		const Vector3 b1 = now.attitude.col(0);
		const Vector3 b2 = now.attitude.col(1);
		expect_near(b1.cross(b2), Vector3(now.attitude.col(2)), 1e-5f);
	}
}

// With zero tracking error, the model acceleration equals a_d and the model
// jerk equals j_d, so the specific-force derivatives reduce to j_d and s_d.
TEST(Se3Controller, PerfectTrackingForceDerivativesEqualReferenceJerkAndSnap)
{
	const Sequence sequence = circle(0.8f, 1.0f, 5.0f);
	const TranslationGains gains{4.0f, 3.0f};

	for (int k = 1; k < 10; ++k) {
		const float t = 0.5f * k;
		const Reference reference = sample(sequence, t);
		const State state = tracking_state(reference, desired_attitude(reference));

		GeometricOutput output{};
		ASSERT_TRUE(Controller::compute_geometry(state, reference, gains, output));

		expect_near(output.translation.acceleration, vec(reference.acceleration), 1e-5f);
		expect_near(output.force_derivatives.first, vec(reference.jerk), 1e-4f);
		expect_near(output.force_derivatives.second, vec(reference.snap), 1e-3f);

		const Vector3 gravity{0.0f, 0.0f, kGravity};
		EXPECT_NEAR(output.collective_acceleration, (vec(reference.acceleration) - gravity).norm(), 1e-4f);
	}
}

TEST(Se3Controller, RejectsNonPositiveCollectiveThrust)
{
	// A downward acceleration demand larger than gravity (20 m position error
	// with the reference below the vehicle) cannot be produced by an upright
	// body: u = -B . b3 < 0. The controller must reject it.
	Reference reference{};
	reference.position = {{0.0f, 0.0f, 0.0f}};

	State state{};
	state.position = Vector3{0.0f, 0.0f, -20.0f};
	state.attitude = matrix::Dcmf{};

	GeometricOutput output{};
	EXPECT_FALSE(Controller::compute_geometry(state, reference, TranslationGains{4.0f, 3.0f}, output));
}

TEST(Se3AttitudeLaws, BodyRateFeedbackOpposesAttitudeError)
{
	DesiredAttitudeDynamics desired{};
	desired.attitude = matrix::Dcmf{matrix::Eulerf{0.0f, 0.0f, 0.4f}};
	desired.angular_velocity = Vector3{0.0f, 0.0f, 0.2f};

	State state{};
	state.attitude = desired.attitude;

	const Vector3 gains{5.0f, 5.0f, 2.0f};
	Vector3 rate{};

	// Zero attitude error: the setpoint is the desired angular velocity.
	ASSERT_TRUE(AttitudeLaws::body_rate(state, desired, gains, rate));
	expect_near(rate, desired.angular_velocity, 1e-6f);

	// A positive roll error about the body x axis commands negative roll rate.
	const float roll_error = 0.1f;
	state.attitude = desired.attitude * matrix::Dcmf{matrix::Eulerf{roll_error, 0.0f, 0.0f}};
	ASSERT_TRUE(AttitudeLaws::body_rate(state, desired, gains, rate));
	EXPECT_NEAR(rate(0), -gains(0) * std::sin(roll_error), 1e-5f);
}

namespace
{

matrix::Matrix3f test_inertia()
{
	// Full symmetric positive-definite tensor so every product is exercised.
	matrix::Matrix3f inertia;
	inertia(0, 0) = 0.022f; inertia(0, 1) = 0.001f; inertia(0, 2) = -0.0005f;
	inertia(1, 0) = 0.001f; inertia(1, 1) = 0.025f; inertia(1, 2) = 0.0008f;
	inertia(2, 0) = -0.0005f; inertia(2, 1) = 0.0008f; inertia(2, 2) = 0.043f;
	return inertia;
}

PhysicalAttitudeGains physical_gains()
{
	PhysicalAttitudeGains gains{};
	gains.attitude = Vector3{5.0f, 4.0f, 1.1f};
	gains.angular_velocity = Vector3{0.53f, 0.5f, 0.35f};
	return gains;
}

}  // namespace

TEST(Se3AttitudeLaws, PhysicalMomentIsTheLeeEquation)
{
	DesiredAttitudeDynamics desired{};
	desired.attitude = matrix::Dcmf{matrix::Eulerf{0.1f, -0.2f, 0.7f}};
	desired.angular_velocity = Vector3{0.3f, -0.2f, 0.5f};
	desired.angular_acceleration = Vector3{1.0f, 0.4f, -0.6f};

	State state{};
	state.attitude = matrix::Dcmf{matrix::Eulerf{0.25f, -0.05f, 0.6f}};
	state.angular_velocity = Vector3{-0.4f, 0.9f, 0.2f};

	const matrix::Matrix3f J = test_inertia();
	const PhysicalAttitudeGains gains = physical_gains();
	PhysicalMomentOutput output{};
	ASSERT_TRUE(AttitudeLaws::physical(state, desired, J, gains, output));

	// Independent evaluation of Lee et al. (2010), eq. (11)-(12) and (16).
	const matrix::Dcmf &R = state.attitude;
	const matrix::Dcmf &Rd = desired.attitude;
	const Vector3 &W = state.angular_velocity;
	// e_R = 0.5 (Rd^T R - R^T Rd)^vee; vee() above already halves its argument.
	const Vector3 eR = vee(matrix::SquareMatrix<float, 3>(Rd.transpose() * R - R.transpose() * Rd)) * 0.5f;
	const Vector3 Wd = R.transpose() * (Rd * desired.angular_velocity);
	const Vector3 eW = W - Wd;
	const Vector3 Wd_dot = R.transpose() * (Rd * desired.angular_acceleration);
	const Vector3 expected = -gains.attitude.emult(eR) - gains.angular_velocity.emult(eW) +
				 W.cross(J * W) - J * (W.cross(Wd) - Wd_dot);

	expect_near(output.attitude_feedback, -gains.attitude.emult(eR), 1e-6f);
	expect_near(output.angular_velocity_feedback, -gains.angular_velocity.emult(eW), 1e-6f);
	expect_near(output.gyroscopic, W.cross(J * W), 1e-7f);
	expect_near(output.inertial_feedforward, -J * (W.cross(Wd) - Wd_dot), 1e-6f);
	expect_near(output.moment, expected, 1e-5f);
}

TEST(Se3AttitudeLaws, PhysicalInertiaAndGyroscopicTerms)
{
	matrix::Matrix3f J;
	J.setZero();
	J(0, 0) = 0.02f;
	J(1, 1) = 0.03f;
	J(2, 2) = 0.05f;

	// Perfect tracking (e_R = e_Omega = 0): the moment is the rigid-body
	// requirement Omega_d x J Omega_d + J dOmega_d/dt.
	DesiredAttitudeDynamics desired{};
	desired.attitude = matrix::Dcmf{matrix::Eulerf{0.2f, 0.1f, -0.3f}};
	desired.angular_velocity = Vector3{1.0f, 2.0f, 3.0f};
	desired.angular_acceleration = Vector3{0.5f, -1.0f, 2.0f};

	State state{};
	state.attitude = desired.attitude;
	state.angular_velocity = desired.angular_velocity;

	PhysicalMomentOutput output{};
	ASSERT_TRUE(AttitudeLaws::physical(state, desired, J, physical_gains(), output));
	expect_near(output.attitude_feedback, Vector3{}, 1e-6f);
	expect_near(output.angular_velocity_feedback, Vector3{}, 1e-6f);
	expect_near(output.gyroscopic, Vector3{0.12f, -0.09f, 0.02f}, 1e-6f); // Hand-evaluated
	expect_near(output.inertial_feedforward, Vector3{0.01f, -0.03f, 0.10f}, 1e-6f);
	expect_near(output.moment, Vector3{0.13f, -0.12f, 0.12f}, 1e-6f);

	// Doubling the inertia doubles both model terms; feedback is unchanged.
	PhysicalMomentOutput doubled{};
	ASSERT_TRUE(AttitudeLaws::physical(state, desired, J * 2.0f, physical_gains(), doubled));
	expect_near(doubled.moment, output.moment * 2.0f, 1e-6f);

	// Neutral command (R_d = R, Omega_d = 0): rate damping plus gyroscopic term.
	DesiredAttitudeDynamics current{};
	current.attitude = state.attitude;
	ASSERT_TRUE(AttitudeLaws::physical(state, current, J, physical_gains(), output));
	expect_near(output.moment,
		    -physical_gains().angular_velocity.emult(state.angular_velocity) +
		    state.angular_velocity.cross(J * state.angular_velocity), 1e-6f);
}

TEST(Se3AttitudeLaws, PhysicalMomentFeedbackSignsAndInvalidInputs)
{
	DesiredAttitudeDynamics desired{};
	State state{};
	const matrix::Matrix3f J = test_inertia();
	PhysicalMomentOutput output{};

	// Positive roll error and positive roll rate both command negative roll moment.
	state.attitude = matrix::Dcmf{matrix::Eulerf{0.1f, 0.0f, 0.0f}};
	ASSERT_TRUE(AttitudeLaws::physical(state, desired, J, physical_gains(), output));
	EXPECT_NEAR(output.moment(0), -physical_gains().attitude(0) * std::sin(0.1f), 1e-5f);

	state.attitude = matrix::Dcmf{};
	state.angular_velocity = Vector3{0.0f, 0.4f, 0.0f};
	ASSERT_TRUE(AttitudeLaws::physical(state, desired, J, physical_gains(), output));
	EXPECT_LT(output.angular_velocity_feedback(1), 0.0f);

	// Invalid gains, inertia or state are rejected and leave the output unchanged.
	const PhysicalMomentOutput previous = output;
	PhysicalAttitudeGains negative = physical_gains();
	negative.angular_velocity(2) = -0.1f;
	EXPECT_FALSE(AttitudeLaws::physical(state, desired, J, negative, output));

	matrix::Matrix3f invalid = J;
	invalid(1, 1) = NAN;
	EXPECT_FALSE(AttitudeLaws::physical(state, desired, invalid, physical_gains(), output));

	state.angular_velocity(0) = INFINITY;
	EXPECT_FALSE(AttitudeLaws::physical(state, desired, J, physical_gains(), output));
	expect_near(output.moment, previous.moment, 0.0f);
}

TEST(Se3ControlPath, ParameterValuesSelectTheThreePaths)
{
  ControlPath path = ControlPath::Physical;
  ASSERT_TRUE(control_path_from_parameter(0, path));
  EXPECT_EQ(path, ControlPath::AttitudePx4);
  ASSERT_TRUE(control_path_from_parameter(1, path));
  EXPECT_EQ(path, ControlPath::RatePx4);
  ASSERT_TRUE(control_path_from_parameter(2, path));
  EXPECT_EQ(path, ControlPath::Physical);

  // Unknown values select nothing (the module then refuses to start).
  path = ControlPath::RatePx4;
  EXPECT_FALSE(control_path_from_parameter(-1, path));
  EXPECT_FALSE(control_path_from_parameter(3, path));
  EXPECT_EQ(path, ControlPath::RatePx4);

  EXPECT_STREQ(control_path_name(ControlPath::AttitudePx4), "ATTITUDE_PX4");
  EXPECT_STREQ(control_path_name(ControlPath::RatePx4), "RATE_PX4");
  EXPECT_STREQ(control_path_name(ControlPath::Physical), "PHYSICAL");
}

TEST(Se3ControlPath, ModeEnablesOnlyTheRequiredPx4Controllers)
{
  const ModeControllers attitude = mode_controllers(ControlPath::AttitudePx4, false);
  EXPECT_TRUE(attitude.attitude && attitude.rates && attitude.allocation);

  // RATE_PX4 registers exactly as before: rate loop and allocator.
  const ModeControllers rate = mode_controllers(ControlPath::RatePx4, false);
  EXPECT_TRUE(!rate.attitude && rate.rates && rate.allocation);

  const ModeControllers physical = mode_controllers(ControlPath::Physical, false);
  EXPECT_TRUE(!physical.attitude && !physical.rates && physical.allocation);

  // PHYSICAL fault: torque production returns to the PX4 rate controller.
  const ModeControllers fallback = mode_controllers(ControlPath::Physical, true);
  EXPECT_TRUE(!fallback.attitude && fallback.rates && fallback.allocation);

  // The fallback flag affects only PHYSICAL.
  EXPECT_TRUE(mode_controllers(ControlPath::RatePx4, true).rates);
  EXPECT_TRUE(mode_controllers(ControlPath::AttitudePx4, true).attitude);
}

TEST(Se3ControlPath, RatePx4SelectionIsTheOriginalPredicate)
{
  for (int bits = 0; bits < 16; ++bits) {
    ControlModeFlags flags{};
    flags.attitude = bits & 1;
    flags.rates = bits & 2;
    flags.multicopter_position = bits & 4;
    flags.allocation = bits & 8;

    // Original RATE_PX4 condition (independent of the fault state).
    const bool original = flags.rates && !flags.attitude && !flags.multicopter_position && flags.allocation;
    EXPECT_EQ(controllers_match(ControlPath::RatePx4, false, flags), original) << bits;
    EXPECT_EQ(controllers_match(ControlPath::RatePx4, true, flags), original) << bits;
  }
}

TEST(Se3ControlPath, EachPathPublishesOnlyWithItsOwnControllers)
{
  auto flags_of = [](const ModeControllers & controllers) {
    ControlModeFlags flags{};
    flags.attitude = controllers.attitude;
    flags.rates = controllers.rates;
    flags.allocation = controllers.allocation;
    return flags;
  };

  const ControlPath paths[] {ControlPath::AttitudePx4, ControlPath::RatePx4, ControlPath::Physical};

  // A path matches the configuration it registers and no other path's.
  for (ControlPath path : paths) {
    for (ControlPath other : paths) {
      const ControlModeFlags flags = flags_of(mode_controllers(other, false));
      EXPECT_EQ(controllers_match(path, false, flags), path == other);
    }
  }

  // Never while PX4 position control or no allocation is active (e.g. a
  // native mode, or termination).
  ControlModeFlags position = flags_of(mode_controllers(ControlPath::AttitudePx4, false));
  position.multicopter_position = true;
  ControlModeFlags no_allocation = flags_of(mode_controllers(ControlPath::Physical, false));
  no_allocation.allocation = false;

  for (ControlPath path : paths) {
    EXPECT_FALSE(controllers_match(path, false, position));
    EXPECT_FALSE(controllers_match(path, true, position));
    EXPECT_FALSE(controllers_match(path, false, no_allocation));
  }

  // Faulted PHYSICAL keeps handling its fault before and after Commander
  // applies the rate-control fallback; unfaulted it never runs with it.
  const ControlModeFlags fallback = flags_of(mode_controllers(ControlPath::Physical, true));
  EXPECT_FALSE(controllers_match(ControlPath::Physical, false, fallback));
  EXPECT_TRUE(controllers_match(ControlPath::Physical, true, fallback));
  EXPECT_TRUE(controllers_match(ControlPath::Physical, true,
                                flags_of(mode_controllers(ControlPath::Physical, false))));
}

TEST(Se3ControlPath, OnlyUnfaultedPhysicalOwnsTheAllocator)
{
  // control_allocator's ownership rule against each registered configuration.
  auto owns = [](const ModeControllers & controllers) {
    se3_physical::OwnershipState state{};
    state.input_available = true;
    state.setpoint_received = true;
    state.setpoint_nav_state = 23;
    state.nav_state = 23;
    state.armed = true;
    state.allocation_enabled = controllers.allocation;
    state.rates_enabled = controllers.rates;
    return se3_physical::physical_owns_allocation(state);
  };

  EXPECT_TRUE(owns(mode_controllers(ControlPath::Physical, false)));
  EXPECT_FALSE(owns(mode_controllers(ControlPath::Physical, true)));   // Fault handoff
  EXPECT_FALSE(owns(mode_controllers(ControlPath::RatePx4, false)));
  EXPECT_FALSE(owns(mode_controllers(ControlPath::AttitudePx4, false)));
}
