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
#include "Integral.hpp"
#include "physical/PhysicalAllocation.hpp"
#include "trajectory/Trajectory.hpp"

#include <cmath>

using namespace se3_control;
using trajectory_generator::Reference;
using trajectory_generator::Segment;
using trajectory_generator::Settings;
using trajectory_generator::Trajectory;
using trajectory_generator::TrajectoryId;

namespace
{

constexpr float kGravity = 9.80665f;
constexpr float kPi = 3.14159265358979323846f;
// Central-difference step. Patterns reach phases of 60 rad, where float sin()
// has errors of order 1e-6; 2.5 ms keeps that noise well below tolerance.
constexpr float kStep = 2.5e-3f;

Vector3 vec(const trajectory_generator::Vector3 &v)
{
	return Vector3{v[0], v[1], v[2]};
}

// The agreed TRJ_* defaults (module.yaml).
Settings default_settings()
{
	Settings settings{};
	settings.sp_x = 0.0f;
	settings.sp_y = 0.0f;
	settings.sp_z = -1.0f;
	settings.sp_yaw = 0.0f;
	settings.step_hold = 5.0f;
	settings.home_x = 0.0f;
	settings.home_y = 0.0f;
	settings.home_z = -1.2f;
	settings.home_yaw = 0.0f;
	settings.circle_radius = 1.0f;
	settings.circle_duration = 60.0f;
	settings.circle_turns = 5;
	settings.figure_x = 0.8f;
	settings.figure_y = 1.2f;
	settings.figure_duration = 90.0f;
	settings.figure_turns = 5;
	return settings;
}

Reference stationary(float x, float y, float z, float yaw = 0.0f)
{
	Reference reference{};
	reference.position = {{x, y, z}};
	reference.yaw = yaw;
	return reference;
}

Reference home()
{
	return stationary(0.0f, 0.0f, -1.2f);
}

Trajectory build(TrajectoryId id, const Reference &start, const Settings &settings = default_settings())
{
	Trajectory plan{};
	EXPECT_TRUE(trajectory_generator::build_trajectory(static_cast<int>(id), settings, start, plan));
	return plan;
}

Reference sample(const Trajectory &plan, float time_s)
{
	Reference reference{};
	EXPECT_TRUE(trajectory_generator::sample(plan.sequence, time_s, reference));
	return reference;
}

// Start time of segment index within the plan.
float segment_start(const Trajectory &plan, std::size_t index)
{
	float start = 0.0f;

	for (std::size_t i = 0; i < index; ++i) { start += plan.sequence.segments[i].duration_s; }

	return start;
}

// The constant-rate pattern segment: the longest shaped one.
std::size_t pattern_segment(const Trajectory &plan)
{
	std::size_t best = plan.sequence.count;

	for (std::size_t i = 0; i < plan.sequence.count; ++i) {
		const Segment &segment = plan.sequence.segments[i];

		if (segment.shape != Segment::Shape::None &&
		    (best == plan.sequence.count || segment.duration_s > plan.sequence.segments[best].duration_s)) {
			best = i;
		}
	}

	return best;
}

void expect_near(const Vector3 &actual, const Vector3 &expected, float tolerance)
{
	for (int axis = 0; axis < 3; ++axis) {
		EXPECT_NEAR(actual(axis), expected(axis), tolerance) << "axis " << axis;
	}
}

// Finite-difference comparison. Single-precision central differences with a
// millisecond step have errors of order 1e-4 relative to the derivative magnitude,
// so the tolerance combines an absolute floor with a relative term. Sign or
// missing-term errors produce discrepancies of order one and remain visible.
void expect_close(const Vector3 &actual, const Vector3 &expected, float absolute, float relative)
{
	for (int axis = 0; axis < 3; ++axis) {
		EXPECT_NEAR(actual(axis), expected(axis), absolute + relative * std::fabs(expected(axis)))
				<< "axis " << axis;
	}
}

// Full reference state equality (position through snap, yaw through yaw acceleration).
void expect_same_state(const Reference &actual, const Reference &expected, float tolerance)
{
	expect_near(vec(actual.position), vec(expected.position), tolerance);
	expect_near(vec(actual.velocity), vec(expected.velocity), tolerance);
	expect_near(vec(actual.acceleration), vec(expected.acceleration), tolerance);
	expect_near(vec(actual.jerk), vec(expected.jerk), 10.0f * tolerance);
	expect_near(vec(actual.snap), vec(expected.snap), 10.0f * tolerance);
	EXPECT_NEAR(std::remainder(actual.yaw - expected.yaw, 2.0f * kPi), 0.0f, tolerance);
	EXPECT_NEAR(actual.yaw_rate, expected.yaw_rate, tolerance);
	EXPECT_NEAR(actual.yaw_acceleration, expected.yaw_acceleration, tolerance);
}

void expect_at_rest(const Reference &reference, const Reference &pose, float tolerance)
{
	expect_same_state(reference, pose, tolerance);
}

// Each analytic derivative must equal the central difference of the one
// below, at n points spread over [begin, end] (joins included).
void expect_consistent_derivatives(const Trajectory &plan, float begin, float end, int n)
{
	for (int k = 0; k < n; ++k) {
		const float t = begin + (end - begin) * (static_cast<float>(k) + 0.5f) / static_cast<float>(n);
		const Reference before = sample(plan, t - kStep);
		const Reference after = sample(plan, t + kStep);
		const Reference now = sample(plan, t);
		const float inverse = 0.5f / kStep;

		expect_close((vec(after.position) - vec(before.position)) * inverse, vec(now.velocity), 3e-3f, 1e-3f);
		expect_close((vec(after.velocity) - vec(before.velocity)) * inverse, vec(now.acceleration), 3e-3f, 1e-3f);
		expect_close((vec(after.acceleration) - vec(before.acceleration)) * inverse, vec(now.jerk), 3e-3f, 1e-3f);
		expect_close((vec(after.jerk) - vec(before.jerk)) * inverse, vec(now.snap), 3e-3f, 1e-3f);
		EXPECT_NEAR((after.yaw - before.yaw) * inverse, now.yaw_rate, 3e-3f);
		EXPECT_NEAR((after.yaw_rate - before.yaw_rate) * inverse, now.yaw_acceleration, 3e-3f);
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

// Mid-pattern time of a continuous plan.
float pattern_time(const Trajectory &plan, float fraction)
{
	const std::size_t index = pattern_segment(plan);
	return segment_start(plan, index) + fraction * plan.sequence.segments[index].duration_s;
}

}  // namespace

TEST(Se3Trajectory, CatalogHasExactlyTheFourTrajectories)
{
	ASSERT_EQ(trajectory_generator::trajectory_count(), 4u);
	EXPECT_STREQ(trajectory_generator::trajectory_name(0), "setpoint");
	EXPECT_STREQ(trajectory_generator::trajectory_name(1), "steps5");
	EXPECT_STREQ(trajectory_generator::trajectory_name(2), "circle");
	EXPECT_STREQ(trajectory_generator::trajectory_name(3), "figure8");
	EXPECT_EQ(trajectory_generator::trajectory_index("figure8"), 3);
	EXPECT_EQ(trajectory_generator::trajectory_index("helix"), -1);
	EXPECT_EQ(trajectory_generator::trajectory_name(4), nullptr);

	Trajectory plan{};
	EXPECT_FALSE(trajectory_generator::build_trajectory(4, default_settings(), home(), plan));
	EXPECT_FALSE(trajectory_generator::build_trajectory(-1, default_settings(), home(), plan));
}

TEST(Se3Trajectory, SetpointIsAnImmediateStep)
{
	const Reference start = stationary(0.4f, -0.3f, -1.5f, 0.6f);
	const Trajectory plan = build(TrajectoryId::Setpoint, start);

	// TRJ_SP_* at once, then held at rest.
	for (float t : {0.0f, 0.5f, 100.0f}) {
		expect_at_rest(sample(plan, t), stationary(0.0f, 0.0f, -1.0f, 0.0f), 1e-6f);
	}
}

TEST(Se3Trajectory, Steps5AreTrueStepsThenASmoothReturnHome)
{
	constexpr float steps[5][3] {
		{0.0f, 0.0f, -1.2f}, {1.0f, 0.0f, -1.2f}, {1.0f, 1.0f, -1.2f}, {0.0f, 1.0f, -1.2f}, {0.0f, 0.0f, -0.9f}
	};
	Reference start = stationary(0.3f, -0.2f, -1.0f, 0.2f);
	start.velocity = {{0.4f, 0.1f, 0.0f}};
	const Trajectory plan = build(TrajectoryId::Steps5, start);
	const float hold = 5.0f;

	ASSERT_EQ(plan.sequence.count, 6u); // Five holds and the return.

	// Step 1 is taken immediately, without interpolating from the start.
	expect_at_rest(sample(plan, 0.0f), stationary(steps[0][0], steps[0][1], steps[0][2]), 1e-6f);

	for (int k = 0; k < 5; ++k) {
		const Reference target = stationary(steps[k][0], steps[k][1], steps[k][2]);
		expect_at_rest(sample(plan, k * hold + 1.0e-3f), target, 1e-6f);
		expect_at_rest(sample(plan, (k + 1) * hold - 1.0e-3f), target, 1e-6f);

		if (k > 0) {
			// Instantaneous change: the previous reference until the boundary.
			expect_at_rest(sample(plan, k * hold - 1.0e-3f), stationary(steps[k - 1][0], steps[k - 1][1], steps[k - 1][2]),
				       1e-6f);
		}
	}

	// After step 5: a C4 return to home, then home is held.
	const float return_start = 5.0f * hold;
	const float return_end = plan.duration_s;
	EXPECT_GT(return_end, return_start + 1.0f);
	Reference begin{};
	ASSERT_TRUE(trajectory_generator::sample_segment(plan.sequence.segments[5], 0.0f, begin));
	expect_at_rest(begin, stationary(0.0f, 0.0f, -0.9f), 1e-5f);
	expect_at_rest(sample(plan, return_end), home(), 1e-5f);
	expect_at_rest(sample(plan, return_end + 1000.0f), home(), 1e-6f);
	expect_consistent_derivatives(plan, return_start + 0.01f, return_end - 0.01f, 20);

	EXPECT_EQ(trajectory_generator::segment_index(plan.sequence, 2.0f), 0u);
	EXPECT_EQ(trajectory_generator::segment_index(plan.sequence, return_start + 0.5f), 5u);
	EXPECT_EQ(trajectory_generator::segment_index(plan.sequence, return_end + 1.0f), 6u);
}

TEST(Se3Trajectory, CircleHasTheConfiguredTurnsDurationAndRadius)
{
	const Trajectory plan = build(TrajectoryId::Circle, home());
	const std::size_t index = pattern_segment(plan);
	ASSERT_LT(index, plan.sequence.count);
	const Segment &pattern = plan.sequence.segments[index];
	EXPECT_EQ(pattern.shape, Segment::Shape::Circle);
	EXPECT_FLOAT_EQ(pattern.duration_s, 60.0f);
	EXPECT_FLOAT_EQ(pattern.x_amplitude_m, 1.0f);

	// Five turns at the constant rate 2 pi 5 / 60 around home: speed r w and
	// radius r throughout the pattern, which starts at phase 0 (+x).
	const float begin = segment_start(plan, index);
	const float rate = 2.0f * kPi * 5.0f / 60.0f;
	expect_near(vec(sample(plan, begin).position), Vector3{1.0f, 0.0f, -1.2f}, 1e-4f);
	expect_near(vec(sample(plan, begin + 60.0f).position), Vector3{1.0f, 0.0f, -1.2f}, 2e-3f);

	for (int k = 0; k <= 60; ++k) {
		const Reference reference = sample(plan, begin + static_cast<float>(k));
		const Vector3 offset = vec(reference.position) - Vector3{0.0f, 0.0f, -1.2f};
		EXPECT_NEAR(offset.norm(), 1.0f, 1e-4f);
		EXPECT_NEAR(vec(reference.velocity).norm(), rate, 1e-4f);
		EXPECT_NEAR(vec(reference.acceleration).norm(), rate * rate, 1e-4f);
		EXPECT_FLOAT_EQ(reference.yaw, 0.0f);
	}

	// Entry and return are outside the 60 s; the plan starts at the current
	// reference and ends at rest at home.
	EXPECT_GT(plan.duration_s, 60.0f + 4.0f);
	expect_same_state(sample(plan, 0.0f), home(), 1e-5f);
	expect_at_rest(sample(plan, plan.duration_s), home(), 1e-5f);
	expect_at_rest(sample(plan, plan.duration_s + 10.0f), home(), 1e-6f);
	expect_consistent_derivatives(plan, 0.0f, plan.duration_s, 300);
}

TEST(Se3Trajectory, FigureEightHasTheConfiguredCyclesDurationAndAmplitudes)
{
	const Trajectory plan = build(TrajectoryId::Figure8, home());
	const std::size_t index = pattern_segment(plan);
	ASSERT_LT(index, plan.sequence.count);
	const Segment &pattern = plan.sequence.segments[index];
	EXPECT_EQ(pattern.shape, Segment::Shape::FigureEight);
	EXPECT_FLOAT_EQ(pattern.duration_s, 90.0f);

	// x = 0.8 sin(phi), y = 1.2 sin(2 phi), phi = 2 pi 5 t / 90, about home.
	const float begin = segment_start(plan, index);
	const float rate = 2.0f * kPi * 5.0f / 90.0f;
	float max_x = 0.0f;
	float max_y = 0.0f;

	for (int k = 0; k <= 900; ++k) {
		const float t = 0.1f * static_cast<float>(k);
		const float phase = rate * t;
		const Reference reference = sample(plan, begin + t);
		expect_near(vec(reference.position),
			    Vector3{0.8f * std::sin(phase), 1.2f * std::sin(2.0f * phase), -1.2f}, 2e-3f);
		max_x = std::fmax(max_x, std::fabs(reference.position[0]));
		max_y = std::fmax(max_y, std::fabs(reference.position[1]));
	}

	EXPECT_NEAR(max_x, 0.8f, 1e-3f);
	EXPECT_NEAR(max_y, 1.2f, 1e-3f);

	EXPECT_GT(plan.duration_s, 90.0f + 4.0f);
	expect_same_state(sample(plan, 0.0f), home(), 1e-5f);
	expect_at_rest(sample(plan, plan.duration_s), home(), 1e-5f);
	expect_consistent_derivatives(plan, 0.0f, plan.duration_s, 300);
}

// Every command may follow every other, at any time, without leaving SE3.
// Circle and figure-eight continue the current reference exactly (position
// through snap, yaw through yaw acceleration); setpoint and steps5 step.
TEST(Se3Trajectory, AnyTrajectoryPreemptsAnyOtherAtAnyTime)
{
	const Reference hold = stationary(0.35f, -0.4f, -1.4f, 2.9f);
	const TrajectoryId ids[] {TrajectoryId::Setpoint, TrajectoryId::Steps5, TrajectoryId::Circle, TrajectoryId::Figure8};

	for (TrajectoryId from : ids) {
		const Trajectory active = build(from, hold);
		const float times[] {0.0f, 0.73f, 3.2f, 7.5f, pattern_time(active, 0.37f), active.duration_s - 1.3f,
				     active.duration_s + 5.0f};

		for (float t : times) {
			const Reference current = sample(active, std::fmax(t, 0.0f));

			for (TrajectoryId to : ids) {
				const Trajectory next = build(to, current);
				const Reference first = sample(next, 0.0f);

				if (to == TrajectoryId::Setpoint) {
					expect_at_rest(first, stationary(0.0f, 0.0f, -1.0f), 1e-6f);

				} else if (to == TrajectoryId::Steps5) {
					expect_at_rest(first, stationary(0.0f, 0.0f, -1.2f), 1e-6f);

				} else {
					expect_same_state(first, current, 2e-4f);
					const float entry = segment_start(next, pattern_segment(next));
					expect_consistent_derivatives(next, 0.0f, entry + 1.0f, 24);
				}

				// Finite plans end at rest at home.
				if (to != TrajectoryId::Setpoint) {
					expect_at_rest(sample(next, next.duration_s + 1.0f), home(), 1e-4f);
				}
			}
		}
	}
}

// Entry, ramp and return transitions stay within 0.6 m/s and 0.6 m/s^2, or
// within 110 % of the start values when those are larger, and last 2..30 s.
TEST(Se3Trajectory, TransitionsStayWithinTheirLimits)
{
	const Reference starts[] {home(), stationary(2.0f, -1.5f, -2.5f, -2.0f),
				  sample(build(TrajectoryId::Figure8, home()), 30.0f)};

	for (const Reference &start : starts) {
		for (TrajectoryId id : {TrajectoryId::Circle, TrajectoryId::Figure8}) {
			const Trajectory plan = build(id, start);
			const std::size_t pattern = pattern_segment(plan);
			const float speed_limit = std::fmax(0.6f, 1.1f * vec(start.velocity).norm());
			const float acceleration_limit = std::fmax(0.6f, 1.1f * vec(start.acceleration).norm());

			for (std::size_t i = 0; i < plan.sequence.count; ++i) {
				const Segment &segment = plan.sequence.segments[i];

				if (segment.shape != Segment::Shape::None) {
					continue;
				}

				EXPECT_GE(segment.duration_s, 2.0f - 1e-4f);
				EXPECT_LE(segment.duration_s, 30.0f + 1e-4f);
				const float begin = segment_start(plan, i);

				for (int k = 0; k <= 100; ++k) {
					const Reference r = sample(plan, begin + segment.duration_s * static_cast<float>(k) / 100.0f);
					// 32-point duration search: allow a few percent between samples.
					EXPECT_LE(vec(r.velocity).norm(), 1.05f * (i < pattern ? speed_limit : 0.6f));
					EXPECT_LE(vec(r.acceleration).norm(), 1.05f * (i < pattern ? acceleration_limit : 0.6f));
					EXPECT_LE(std::fabs(r.yaw_rate), 1.05f * 0.5f);
				}
			}
		}
	}
}

TEST(Se3Trajectory, HoldIsStationaryAtTheMeasuredPose)
{
	Trajectory plan{};
	ASSERT_TRUE(trajectory_generator::build_hold({{0.2f, -0.7f, -1.1f}}, 1.3f, plan));
	EXPECT_EQ(plan.id, trajectory_generator::kHoldId);

	for (float t : {0.0f, 0.5f, 3600.0f}) {
		expect_at_rest(sample(plan, t), stationary(0.2f, -0.7f, -1.1f, 1.3f), 1e-6f);
	}

	EXPECT_FALSE(trajectory_generator::build_hold({{NAN, 0.0f, -1.0f}}, 0.0f, plan));
}

TEST(Se3Trajectory, InvalidSettingsAreRejected)
{
	const Settings valid = default_settings();

	for (int index = 0; index < 4; ++index) {
		EXPECT_TRUE(trajectory_generator::valid_settings(index, valid)) << index;
	}

	Settings invalid = valid;
	invalid.step_hold = 0.0f;
	EXPECT_FALSE(trajectory_generator::valid_settings(static_cast<int>(TrajectoryId::Steps5), invalid));

	invalid = valid;
	invalid.circle_turns = 0;
	EXPECT_FALSE(trajectory_generator::valid_settings(static_cast<int>(TrajectoryId::Circle), invalid));

	invalid = valid;
	invalid.circle_radius = -1.0f;
	EXPECT_FALSE(trajectory_generator::valid_settings(static_cast<int>(TrajectoryId::Circle), invalid));

	invalid = valid;
	invalid.figure_y = 0.0f;
	EXPECT_FALSE(trajectory_generator::valid_settings(static_cast<int>(TrajectoryId::Figure8), invalid));

	invalid = valid;
	invalid.sp_z = NAN;
	EXPECT_FALSE(trajectory_generator::valid_settings(static_cast<int>(TrajectoryId::Setpoint), invalid));

	Trajectory plan{};
	Reference start = home();
	start.velocity[0] = INFINITY;
	EXPECT_FALSE(trajectory_generator::build_trajectory(static_cast<int>(TrajectoryId::Circle), valid, start, plan));
}

TEST(Se3Integral, BoundedFreezableIntegration)
{
	float value = 0.0f;

	// Plain integration inside the bound.
	EXPECT_TRUE(integrate_bounded(value, 0.5f, 0.1f, 1.0f, false));
	EXPECT_FLOAT_EQ(value, 0.05f);

	// Frozen: unchanged.
	EXPECT_FALSE(integrate_bounded(value, 0.5f, 0.1f, 1.0f, true));
	EXPECT_FLOAT_EQ(value, 0.05f);

	// Reaching the bound holds it; further outward input is discarded.
	EXPECT_FALSE(integrate_bounded(value, 20.0f, 0.1f, 1.0f, false));
	EXPECT_FLOAT_EQ(value, 1.0f);
	EXPECT_FALSE(integrate_bounded(value, 0.5f, 0.1f, 1.0f, false));
	EXPECT_FLOAT_EQ(value, 1.0f);

	// Input back inside acts immediately (no unwinding delay).
	EXPECT_TRUE(integrate_bounded(value, -0.5f, 0.1f, 1.0f, false));
	EXPECT_FLOAT_EQ(value, 0.95f);

	// A smaller bound (larger gain) clamps; a zero bound (zero gain) resets.
	EXPECT_FALSE(integrate_bounded(value, 0.0f, 0.1f, 0.5f, true));
	EXPECT_FLOAT_EQ(value, 0.5f);
	EXPECT_FALSE(integrate_bounded(value, 0.5f, 0.1f, 0.0f, false));
	EXPECT_FLOAT_EQ(value, 0.0f);

	// No time step or non-finite input: nothing integrated.
	EXPECT_FALSE(integrate_bounded(value, 0.5f, 0.0f, 1.0f, false));
	EXPECT_FALSE(integrate_bounded(value, NAN, 0.1f, 1.0f, false));
	EXPECT_FLOAT_EQ(value, 0.0f);
}

TEST(Se3Controller, PositionIntegralSignsBoundsAndReset)
{
	const Reference reference = home();
	TranslationGains gains{4.0f, 3.0f, Vector3{0.3f, 0.3f, 0.6f}};
	State state{};
	state.attitude = matrix::Dcmf{};

	// Vehicle 0.1 m below (NED +z) and 0.2 m north of the reference.
	state.position = Vector3{0.2f, 0.0f, -1.1f};
	PositionIntegral integral{};
	ASSERT_TRUE(Controller::update_integral(state, reference, gains, 0.01f, false, integral));
	expect_near(integral.value, Vector3{0.002f, 0.0f, 0.001f}, 1e-7f);
	expect_near(integral.rate, Vector3{0.2f, 0.0f, 0.1f}, 1e-6f);

	// The integral pushes the acceleration command up and south, against the error.
	TranslationOutput translation{};
	ASSERT_TRUE(Controller::compute_translation(state, reference, gains, integral, translation));
	EXPECT_LT(translation.integral(0), 0.0f);
	EXPECT_LT(translation.integral(2), 0.0f);
	expect_near(translation.integral, Vector3{-0.3f * 0.002f, 0.0f, -0.6f * 0.001f}, 1e-7f);
	expect_near(translation.acceleration,
		    Vector3{-4.0f * 0.2f, 0.0f, -4.0f * 0.1f} + translation.integral, 1e-6f);

	// The integrated error saturates at 0.5 m and its rate derivative vanishes.
	state.position = Vector3{3.0f, 0.0f, -1.2f};
	state.velocity = Vector3{0.4f, 0.0f, 0.1f};
	ASSERT_TRUE(Controller::update_integral(state, reference, gains, 0.01f, false, integral));
	EXPECT_FLOAT_EQ(integral.rate(0), 0.5f);
	EXPECT_FLOAT_EQ(integral.rate_derivative(0), 0.0f);
	EXPECT_FLOAT_EQ(integral.rate_derivative(2), 0.1f); // Inside: d/dt e_x = e_v.

	// Bounded: |k_i I| <= 1 m/s^2 horizontally, 2 m/s^2 vertically.
	state.position = Vector3{3.0f, -3.0f, 2.0f};

	for (int k = 0; k < 10000; ++k) {
		ASSERT_TRUE(Controller::update_integral(state, reference, gains, 0.01f, false, integral));
	}

	expect_near(integral.value, Vector3{1.0f / 0.3f, -1.0f / 0.3f, 2.0f / 0.6f}, 1e-4f);
	expect_near(integral.rate, Vector3{}, 0.0f);
	ASSERT_TRUE(Controller::compute_translation(state, reference, gains, integral, translation));
	expect_near(translation.integral, Vector3{-1.0f, 1.0f, -2.0f}, 1e-5f);

	// Frozen: unchanged, with zero rates.
	state.position = Vector3{-0.1f, 0.1f, -1.2f};
	const Vector3 held = integral.value;
	ASSERT_TRUE(Controller::update_integral(state, reference, gains, 0.01f, true, integral));
	expect_near(integral.value, held, 0.0f);
	expect_near(integral.rate, Vector3{}, 0.0f);
	expect_near(integral.rate_derivative, Vector3{}, 0.0f);

	// Integrating back inside is immediate.
	ASSERT_TRUE(Controller::update_integral(state, reference, gains, 0.01f, false, integral));
	EXPECT_LT(integral.value(0), held(0));

	// A zero gain clears its axes; invalid inputs leave the integral unchanged.
	gains.integral = Vector3{0.0f, 0.0f, 0.6f};
	ASSERT_TRUE(Controller::update_integral(state, reference, gains, 0.01f, false, integral));
	EXPECT_FLOAT_EQ(integral.value(0), 0.0f);
	EXPECT_FLOAT_EQ(integral.value(1), 0.0f);
	const PositionIntegral previous = integral;
	gains.integral(2) = -0.1f;
	EXPECT_FALSE(Controller::update_integral(state, reference, gains, 0.01f, false, integral));
	gains.integral(2) = 0.6f;
	EXPECT_FALSE(Controller::update_integral(state, reference, gains, NAN, false, integral));
	state.velocity(1) = NAN;
	EXPECT_FALSE(Controller::update_integral(state, reference, gains, 0.01f, false, integral));
	expect_near(integral.value, previous.value, 0.0f);
}

namespace
{

struct ForceSample {
	Vector3 force;
	Vector3 first;
	Vector3 second;
};

// Geometry for a vehicle that tracks its commanded attitude exactly: R = R_d
// and Omega = Omega_d, so the model acceleration g e3 - u b3 is the actual
// one and the analytic force derivatives must match finite differences.
ForceSample geometry(State &state, const Reference &reference, const TranslationGains &gains,
		     const PositionIntegral &integral)
{
	TranslationOutput translation{};
	EXPECT_TRUE(Controller::compute_translation(state, reference, gains, integral, translation));
	DesiredAttitudeDynamics attitude{};
	EXPECT_TRUE(AttitudeReference::compute(translation.specific_force, Vector3{}, Vector3{}, reference.yaw, 0.0f, 0.0f,
					       attitude));
	state.attitude = attitude.attitude;

	GeometricOutput output{};
	EXPECT_TRUE(Controller::compute_geometry(state, reference, gains, integral, output));
	state.angular_velocity = output.desired_attitude.angular_velocity;
	EXPECT_TRUE(Controller::compute_geometry(state, reference, gains, integral, output));
	return ForceSample{output.translation.specific_force, output.force_derivatives.first,
			   output.force_derivatives.second};
}

// Closed-loop simulation of x'' = g e3 + B with the integral updated before
// each translation evaluation, as in the runtime. Checks the analytic force
// derivatives against central differences of B along the trajectory.
void expect_force_derivatives_consistent(const TranslationGains &gains, const Vector3 &initial_error,
		float duration, bool freeze)
{
	const Trajectory plan = build(TrajectoryId::Figure8, home());
	const float begin = pattern_time(plan, 0.2f);
	constexpr float dt = 1.0e-4f;
	constexpr int h = 10; // Central-difference half step: 1 ms.
	const int steps = static_cast<int>(duration / dt);

	State state{};
	const Reference first = sample(plan, begin);
	state.position = vec(first.position) + initial_error;
	state.velocity = vec(first.velocity) + Vector3{0.05f, -0.05f, 0.02f};
	PositionIntegral integral{};
	ForceSample window[2 * h + 1] {};

	for (int k = 0; k <= steps; ++k) {
		const Reference reference = sample(plan, begin + static_cast<float>(k) * dt);
		ASSERT_TRUE(Controller::update_integral(state, reference, gains, dt, freeze, integral));
		State evaluated = state;
		const ForceSample current = geometry(evaluated, reference, gains, integral);

		// Keep the last 2h + 1 samples.
		for (int i = 0; i < 2 * h; ++i) { window[i] = window[i + 1]; }

		window[2 * h] = current;

		if (k >= 2 * h && (k % 2000) == 0) {
			const ForceSample &middle = window[h];
			const float inverse = 0.5f / (h * dt);
			expect_close((window[2 * h].force - window[0].force) * inverse, middle.first, 5e-3f, 5e-3f);
			expect_close((window[2 * h].first - window[0].first) * inverse, middle.second, 2e-2f, 1e-2f);
		}

		state.position += state.velocity * dt;
		state.velocity += (current.force + Vector3{0.0f, 0.0f, kGravity}) * dt;
	}
}

}  // namespace

TEST(Se3Controller, ForceDerivativesIncludeTheActualIntegralUpdate)
{
	// Active integral inside the error saturation (d^2I/dt^2 = e_v).
	expect_force_derivatives_consistent(TranslationGains{4.0f, 3.0f, Vector3{0.3f, 0.3f, 0.6f}},
					    Vector3{0.3f, -0.2f, 0.25f}, 1.0f, false);
	// Error beyond the 0.5 m saturation on x (dI/dt constant, d^2I/dt^2 = 0).
	expect_force_derivatives_consistent(TranslationGains{4.0f, 3.0f, Vector3{0.3f, 0.3f, 0.6f}},
					    Vector3{1.5f, -0.2f, 0.25f}, 0.2f, false);
	// Frozen (dI/dt = 0).
	expect_force_derivatives_consistent(TranslationGains{4.0f, 3.0f, Vector3{0.3f, 0.3f, 0.6f}},
					    Vector3{0.3f, -0.2f, 0.25f}, 1.0f, true);
	// Large gains: the integral reaches its bound and holds it (dI/dt = 0).
	expect_force_derivatives_consistent(TranslationGains{4.0f, 3.0f, Vector3{5.0f, 5.0f, 5.0f}},
					    Vector3{0.45f, -0.45f, 0.45f}, 1.0f, false);
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
	Settings fast = default_settings();
	fast.figure_duration = 12.0f;
	fast.figure_turns = 1;
	const Trajectory plan = build(TrajectoryId::Figure8, stationary(0.0f, 0.0f, -1.2f, 0.4f), fast);

	for (int k = 1; k < 40; ++k) {
		const float t = plan.duration_s * k / 40.0f; // Entry, ramps, pattern and return.
		const DesiredAttitudeDynamics before = desired_attitude(sample(plan, t - kStep));
		const DesiredAttitudeDynamics after = desired_attitude(sample(plan, t + kStep));
		const DesiredAttitudeDynamics now = desired_attitude(sample(plan, t));

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
	const Trajectory plan = build(TrajectoryId::Circle, home());
	const TranslationGains gains{4.0f, 3.0f, Vector3{0.3f, 0.3f, 0.6f}};

	for (int k = 1; k < 20; ++k) {
		const float t = plan.duration_s * k / 20.0f;
		const Reference reference = sample(plan, t);
		const State state = tracking_state(reference, desired_attitude(reference));

		// Zero error: the integral does not move (dI/dt = 0) and is zero.
		PositionIntegral integral{};
		ASSERT_TRUE(Controller::update_integral(state, reference, gains, 0.01f, false, integral));
		GeometricOutput output{};
		ASSERT_TRUE(Controller::compute_geometry(state, reference, gains, integral, output));

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
	EXPECT_FALSE(Controller::compute_geometry(state, reference, TranslationGains{4.0f, 3.0f}, PositionIntegral{}, output));
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
	gains.integral = Vector3{0.5f, 0.5f, 0.1f};
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
	ASSERT_TRUE(AttitudeLaws::physical(state, desired, J, gains, Vector3{}, output));

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
	ASSERT_TRUE(AttitudeLaws::physical(state, desired, J, physical_gains(), Vector3{}, output));
	expect_near(output.attitude_feedback, Vector3{}, 1e-6f);
	expect_near(output.angular_velocity_feedback, Vector3{}, 1e-6f);
	expect_near(output.gyroscopic, Vector3{0.12f, -0.09f, 0.02f}, 1e-6f); // Hand-evaluated
	expect_near(output.inertial_feedforward, Vector3{0.01f, -0.03f, 0.10f}, 1e-6f);
	expect_near(output.moment, Vector3{0.13f, -0.12f, 0.12f}, 1e-6f);

	// Doubling the inertia doubles both model terms; feedback is unchanged.
	PhysicalMomentOutput doubled{};
	ASSERT_TRUE(AttitudeLaws::physical(state, desired, J * 2.0f, physical_gains(), Vector3{}, doubled));
	expect_near(doubled.moment, output.moment * 2.0f, 1e-6f);

	// Neutral command (R_d = R, Omega_d = 0): rate damping plus gyroscopic term.
	DesiredAttitudeDynamics current{};
	current.attitude = state.attitude;
	ASSERT_TRUE(AttitudeLaws::physical(state, current, J, physical_gains(), Vector3{}, output));
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
	ASSERT_TRUE(AttitudeLaws::physical(state, desired, J, physical_gains(), Vector3{}, output));
	EXPECT_NEAR(output.moment(0), -physical_gains().attitude(0) * std::sin(0.1f), 1e-5f);

	state.attitude = matrix::Dcmf{};
	state.angular_velocity = Vector3{0.0f, 0.4f, 0.0f};
	ASSERT_TRUE(AttitudeLaws::physical(state, desired, J, physical_gains(), Vector3{}, output));
	EXPECT_LT(output.angular_velocity_feedback(1), 0.0f);

	// Invalid gains, inertia or state are rejected and leave the output unchanged.
	const PhysicalMomentOutput previous = output;
	PhysicalAttitudeGains negative = physical_gains();
	negative.angular_velocity(2) = -0.1f;
	EXPECT_FALSE(AttitudeLaws::physical(state, desired, J, negative, Vector3{}, output));

	matrix::Matrix3f invalid = J;
	invalid(1, 1) = NAN;
	EXPECT_FALSE(AttitudeLaws::physical(state, desired, invalid, physical_gains(), Vector3{}, output));

	state.angular_velocity(0) = INFINITY;
	EXPECT_FALSE(AttitudeLaws::physical(state, desired, J, physical_gains(), Vector3{}, output));
	expect_near(output.moment, previous.moment, 0.0f);
}

TEST(Se3AttitudeLaws, PhysicalIntegralIsAddedToTheLeeMoment)
{
	DesiredAttitudeDynamics desired{};
	desired.attitude = matrix::Dcmf{matrix::Eulerf{0.0f, 0.0f, 0.3f}};

	State state{};
	state.attitude = desired.attitude * matrix::Dcmf{matrix::Eulerf{0.05f, -0.03f, 0.02f}};
	state.angular_velocity = Vector3{0.1f, -0.2f, 0.05f};

	const matrix::Matrix3f J = test_inertia();
	const PhysicalAttitudeGains gains = physical_gains();
	PhysicalMomentOutput without{};
	ASSERT_TRUE(AttitudeLaws::physical(state, desired, J, gains, Vector3{}, without));
	expect_near(without.integral_feedback, Vector3{}, 0.0f);

	// -K_I e_I is added unchanged; every other term is unaffected.
	const Vector3 integral{0.02f, -0.01f, 0.03f};
	PhysicalMomentOutput with{};
	ASSERT_TRUE(AttitudeLaws::physical(state, desired, J, gains, integral, with));
	expect_near(with.integral_feedback, -gains.integral.emult(integral), 1e-7f);
	expect_near(with.moment - without.moment, -gains.integral.emult(integral), 1e-6f);
	expect_near(with.attitude_feedback, without.attitude_feedback, 0.0f);

	// The integrator input is e_R (positive roll error -> positive e_R(0),
	// so a growing integral commands a negative roll moment).
	EXPECT_GT(with.attitude_error(0), 0.0f);
	expect_near(with.attitude_feedback, -gains.attitude.emult(with.attitude_error), 1e-7f);

	// Negative integral gains and non-finite integral states are rejected.
	PhysicalAttitudeGains negative = gains;
	negative.integral(0) = -0.1f;
	EXPECT_FALSE(AttitudeLaws::physical(state, desired, J, negative, integral, with));
	EXPECT_FALSE(AttitudeLaws::physical(state, desired, J, gains, Vector3{NAN, 0.0f, 0.0f}, with));
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
