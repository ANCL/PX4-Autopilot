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
	EXPECT_TRUE(trajectory_generator::sample(plan.sequence, time_s, plan.origin, reference));
	return reference;
}

// Start time of segment index within the plan.
float segment_start(const Trajectory &plan, std::size_t index)
{
	float start = 0.0f;

	for (std::size_t i = 0; i < index; ++i) { start += plan.sequence.segments[i].duration_s; }

	return start;
}

// The constant-rate circle or figure-eight segment (count if none).
std::size_t pattern_segment(const Trajectory &plan)
{
	for (std::size_t i = 0; i < plan.sequence.count; ++i) {
		const Segment::Type type = plan.sequence.segments[i].type;

		if (type == Segment::Type::Circle || type == Segment::Type::FigureEight) {
			return i;
		}
	}

	return plan.sequence.count;
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

TEST(Se3Trajectory, DefaultPlansMatchTheExperimentDefinitions)
{
	// Steps5: a hold and four steps of TRJ_STEP_HOLD = 5 s, then a 0.3 m Line
	// home (2 s minimum).
	const Trajectory steps = build(TrajectoryId::Steps5, home());
	ASSERT_EQ(steps.sequence.count, 6u);
	EXPECT_EQ(steps.sequence.segments[0].type, Segment::Type::Hold);

	for (int k = 0; k < 5; ++k) {
		EXPECT_FLOAT_EQ(steps.sequence.segments[k].duration_s, 5.0f) << k;

		if (k > 0) { EXPECT_EQ(steps.sequence.segments[k].type, Segment::Type::Step) << k; }
	}

	EXPECT_EQ(steps.sequence.segments[5].type, Segment::Type::Line);
	EXPECT_FLOAT_EQ(steps.duration_s, 27.0f);

	// Circle from home: a 1 m Line to the +x edge (4 s), five turns of radius
	// 1 m in 60 s, and the 1 m Line back.
	const Trajectory circle = build(TrajectoryId::Circle, home());
	ASSERT_EQ(circle.sequence.count, 3u);
	EXPECT_EQ(circle.sequence.segments[0].type, Segment::Type::Line);
	EXPECT_EQ(circle.sequence.segments[1].type, Segment::Type::Circle);
	EXPECT_FLOAT_EQ(circle.sequence.segments[1].duration_s, 60.0f);
	EXPECT_FLOAT_EQ(circle.sequence.segments[1].radius_m, 1.0f);
	EXPECT_FLOAT_EQ(circle.sequence.segments[1].turns, 5.0f);
	EXPECT_EQ(circle.sequence.segments[2].type, Segment::Type::Line);
	EXPECT_FLOAT_EQ(circle.duration_s, 68.0f);

	// Figure-eight from home: no entry move; five cycles of 0.8 x 1.2 m in 90 s.
	const Trajectory figure = build(TrajectoryId::Figure8, home());
	ASSERT_EQ(figure.sequence.count, 1u);
	EXPECT_EQ(figure.sequence.segments[0].type, Segment::Type::FigureEight);
	EXPECT_FLOAT_EQ(figure.sequence.segments[0].duration_s, 90.0f);
	EXPECT_FLOAT_EQ(figure.sequence.segments[0].x_amplitude_m, 0.8f);
	EXPECT_FLOAT_EQ(figure.sequence.segments[0].y_amplitude_m, 1.2f);
	EXPECT_FLOAT_EQ(figure.sequence.segments[0].turns, 5.0f);
	EXPECT_FLOAT_EQ(figure.duration_s, 90.0f);
}

TEST(Se3Trajectory, SetpointIsAnImmediateStep)
{
	const Reference start = stationary(0.4f, -0.3f, -1.5f, 0.6f);
	const Trajectory plan = build(TrajectoryId::Setpoint, start);

	// TRJ_SP_* at once, then held at rest (the runtime replaces it with live
	// TRJ_SP_* edits and never completes it).
	for (float t : {0.0f, 0.5f, 100.0f}) {
		expect_at_rest(sample(plan, t), stationary(0.0f, 0.0f, -1.0f, 0.0f), 1e-6f);
	}

	// A different snapshot is a different immediate step.
	Settings moved = default_settings();
	moved.sp_x = 0.5f;
	moved.sp_yaw = 1.0f;
	expect_at_rest(sample(build(TrajectoryId::Setpoint, start, moved), 0.0f), stationary(0.5f, 0.0f, -1.0f, 1.0f),
		       1e-6f);
}

TEST(Se3Trajectory, Steps5AreTrueStepsThenALineHome)
{
	constexpr float steps[5][3] {
		{0.0f, 0.0f, -1.2f}, {1.0f, 0.0f, -1.2f}, {1.0f, 1.0f, -1.2f}, {0.0f, 1.0f, -1.2f}, {0.0f, 0.0f, -0.9f}
	};
	Reference start = stationary(0.3f, -0.2f, -1.0f, 0.2f);
	start.velocity = {{0.4f, 0.1f, 0.0f}};
	const Trajectory plan = build(TrajectoryId::Steps5, start);
	const float hold = 5.0f;

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

	// After step 5: a Line from rest to rest at home, then home is held.
	const float return_start = 5.0f * hold;
	expect_at_rest(sample(plan, return_start), stationary(0.0f, 0.0f, -0.9f), 1e-6f);
	expect_at_rest(sample(plan, plan.duration_s), home(), 1e-5f);
	expect_at_rest(sample(plan, plan.duration_s + 1000.0f), home(), 1e-6f);
	expect_consistent_derivatives(plan, return_start + 0.01f, plan.duration_s - 0.01f, 20);

	EXPECT_EQ(trajectory_generator::segment_index(plan.sequence, 2.0f), 0u);
	EXPECT_EQ(trajectory_generator::segment_index(plan.sequence, return_start + 0.5f), 5u);
	EXPECT_EQ(trajectory_generator::segment_index(plan.sequence, plan.duration_s + 1.0f), 6u);
}

TEST(Se3Trajectory, CircleRunsExactlyFiveTurnsAtConstantRate)
{
	const Trajectory plan = build(TrajectoryId::Circle, home());
	const std::size_t index = pattern_segment(plan);
	ASSERT_LT(index, plan.sequence.count);
	const float begin = segment_start(plan, index);
	const float end = begin + 60.0f;
	const float r = 1.0f;
	const float w = 2.0f * kPi * 5.0f / 60.0f;
	const Vector3 centre{0.0f, 0.0f, -1.2f};

	// Constant angular rate w over the whole 60 s, including its first and
	// last instants: radius, speed and every derivative magnitude are constant.
	float swept = 0.0f;
	Vector3 previous = vec(sample(plan, begin).position) - centre;

	for (int k = 1; k <= 600; ++k) {
		const Reference reference = sample(plan, begin + 0.1f * static_cast<float>(k));
		const Vector3 offset = vec(reference.position) - centre;
		swept += std::atan2(previous(0) * offset(1) - previous(1) * offset(0), previous.dot(offset));
		previous = offset;
		EXPECT_NEAR(offset.norm(), r, 1e-4f);
		EXPECT_NEAR(vec(reference.velocity).norm(), r * w, 1e-5f);
		EXPECT_NEAR(vec(reference.acceleration).norm(), r * w * w, 1e-5f);
		EXPECT_NEAR(vec(reference.jerk).norm(), r * w * w * w, 1e-5f);
		EXPECT_NEAR(vec(reference.snap).norm(), r * w * w * w * w, 1e-5f);
		EXPECT_FLOAT_EQ(reference.yaw, 0.0f);
	}

	EXPECT_NEAR(swept, 2.0f * kPi * 5.0f, 1e-3f); // Exactly five revolutions.

	// Analytic derivatives at phase 0 (+x edge) and their consistency inside
	// the pattern.
	const Reference first = sample(plan, begin + 1.0e-6f);
	expect_near(vec(first.position), Vector3{1.0f, 0.0f, -1.2f}, 1e-4f);
	expect_near(vec(first.velocity), Vector3{0.0f, r * w, 0.0f}, 1e-4f);
	expect_near(vec(first.acceleration), Vector3{-r * w * w, 0.0f, 0.0f}, 1e-4f);
	expect_near(vec(first.jerk), Vector3{0.0f, -r * w * w * w, 0.0f}, 1e-4f);
	expect_near(vec(first.snap), Vector3{r * w * w * w * w, 0.0f, 0.0f}, 1e-4f);
	expect_consistent_derivatives(plan, begin + 0.01f, end - 0.01f, 200);

	// Intentional discontinuity: the entry Line ends at rest and the circle
	// starts at speed r w (and the reverse at its end). Position is continuous.
	expect_at_rest(sample(plan, begin), stationary(1.0f, 0.0f, -1.2f), 1e-5f);
	expect_near(vec(sample(plan, end).position), Vector3{1.0f, 0.0f, -1.2f}, 2e-3f);
	EXPECT_NEAR(vec(sample(plan, end).velocity).norm(), r * w, 1e-4f);
	EXPECT_NEAR(vec(sample(plan, end + 1.0e-3f).velocity).norm(), 0.0f, 1e-3f);

	// The entry from home and the return home are smooth Lines.
	expect_at_rest(sample(plan, 0.0f), home(), 1e-6f);
	expect_consistent_derivatives(plan, 0.01f, begin - 0.01f, 20);
	expect_consistent_derivatives(plan, end + 0.01f, plan.duration_s - 0.01f, 20);
	expect_at_rest(sample(plan, plan.duration_s), home(), 1e-4f);
	expect_at_rest(sample(plan, plan.duration_s + 10.0f), home(), 1e-4f);
}

TEST(Se3Trajectory, FigureEightRunsExactlyFiveCyclesAtConstantRate)
{
	const Trajectory plan = build(TrajectoryId::Figure8, home());
	const std::size_t index = pattern_segment(plan);
	ASSERT_LT(index, plan.sequence.count);
	const float begin = segment_start(plan, index);
	const float w = 2.0f * kPi * 5.0f / 90.0f;

	// x = 0.8 sin(w t), y = 1.2 sin(2 w t) about home for all 90 s: phase 10 pi
	// (five cycles) at the end, with the analytic derivatives of that phase.
	for (int k = 0; k <= 900; ++k) {
		const float t = 0.1f * static_cast<float>(k);
		const float phase = w * t;
		const Reference reference = sample(plan, begin + t);
		const float s1 = std::sin(phase);
		const float c1 = std::cos(phase);
		const float s2 = std::sin(2.0f * phase);
		const float c2 = std::cos(2.0f * phase);
		expect_near(vec(reference.position), Vector3{0.8f * s1, 1.2f * s2, -1.2f}, 2e-3f);
		expect_near(vec(reference.velocity), Vector3{0.8f * w * c1, 2.4f * w * c2, 0.0f}, 2e-3f);
		expect_near(vec(reference.acceleration), Vector3{-0.8f * w * w * s1, -4.8f * w * w * s2, 0.0f}, 2e-3f);
		expect_near(vec(reference.jerk), Vector3{-0.8f * w * w * w * c1, -9.6f * w * w * w * c2, 0.0f}, 2e-3f);
		expect_near(vec(reference.snap), Vector3{0.8f * w * w * w * w * s1, 19.2f * w * w * w * w * s2, 0.0f}, 2e-3f);
		EXPECT_FLOAT_EQ(reference.yaw, 0.0f);
	}

	expect_consistent_derivatives(plan, begin + 0.01f, begin + 90.0f - 0.01f, 200);

	// Starts and ends at home; the velocity steps from and to rest there.
	expect_near(vec(sample(plan, begin).position), Vector3{0.0f, 0.0f, -1.2f}, 1e-5f);
	EXPECT_GT(vec(sample(plan, begin).velocity).norm(), 0.5f);
	expect_at_rest(sample(plan, plan.duration_s + 1.0e-3f), home(), 2e-3f);
}

TEST(Se3Trajectory, PatternEntryTurnsThenMovesFromRest)
{
	// From a distant pose: a 2 rad Yaw rotation in place (8 s), then a Line
	// to the circle's +x edge, both from rest to rest within about 0.6 m/s
	// and 0.6 rad/s.
	const Reference start = stationary(2.0f, -1.5f, -2.5f, -2.0f);
	const Trajectory plan = build(TrajectoryId::Circle, start);
	ASSERT_EQ(plan.sequence.count, 4u);
	EXPECT_EQ(plan.sequence.segments[0].type, Segment::Type::Yaw);
	EXPECT_FLOAT_EQ(plan.sequence.segments[0].duration_s, 8.0f);
	EXPECT_EQ(plan.sequence.segments[1].type, Segment::Type::Line);

	expect_at_rest(sample(plan, 0.0f), start, 1e-6f);
	expect_at_rest(sample(plan, 8.0f), stationary(2.0f, -1.5f, -2.5f, 0.0f), 1e-5f);
	const float begin = segment_start(plan, 2);
	expect_at_rest(sample(plan, begin), stationary(1.0f, 0.0f, -1.2f, 0.0f), 1e-4f);
	expect_consistent_derivatives(plan, 0.01f, begin - 0.01f, 40);

	for (int k = 0; k <= 200; ++k) {
		const Reference reference = sample(plan, begin * static_cast<float>(k) / 200.0f);
		EXPECT_LE(vec(reference.velocity).norm(), 0.62f);
		EXPECT_LE(std::fabs(reference.yaw_rate), 0.62f);
	}
}

// Every command replaces every other directly, at any time. setpoint and
// steps5 step to their first point; circle and figure8 continue from the
// current position and yaw at rest, so a moving reference changes velocity
// discontinuously (by design: derivatives are not matched).
TEST(Se3Trajectory, AnyTrajectoryReplacesAnyOtherDirectly)
{
	const Reference hold = stationary(0.35f, -0.4f, -1.4f, 2.9f);
	const TrajectoryId ids[] {TrajectoryId::Setpoint, TrajectoryId::Steps5, TrajectoryId::Circle, TrajectoryId::Figure8};

	for (TrajectoryId from : ids) {
		const Trajectory active = build(from, hold);

		for (float t : {0.0f, 0.73f, 7.5f, 0.5f * active.duration_s, active.duration_s + 5.0f}) {
			const Reference current = sample(active, t);

			for (TrajectoryId to : ids) {
				const Trajectory next = build(to, current);
				const Reference first = sample(next, 0.0f);

				if (to == TrajectoryId::Setpoint) {
					expect_at_rest(first, stationary(0.0f, 0.0f, -1.0f), 1e-6f);

				} else if (to == TrajectoryId::Steps5) {
					expect_at_rest(first, stationary(0.0f, 0.0f, -1.2f), 1e-6f);

				} else {
					// Same position and yaw. At rest, unless the reference is
					// already at the pattern start, which then begins at once
					// (velocity step).
					expect_near(vec(first.position), vec(current.position), 1e-5f);
					EXPECT_NEAR(first.yaw, current.yaw, 1e-6f);

					if (pattern_segment(next) != 0) {
						expect_near(vec(first.velocity), Vector3{}, 0.0f);
						EXPECT_FLOAT_EQ(first.yaw_rate, 0.0f);
					}
				}

				// Finite plans end at rest at home.
				if (to != TrajectoryId::Setpoint) {
					expect_at_rest(sample(next, next.duration_s + 1.0f), home(), 2e-3f);
				}
			}
		}
	}

	// Replacing a moving circle keeps the position but not the velocity.
	const Trajectory circle = build(TrajectoryId::Circle, home());
	const Reference moving = sample(circle, pattern_time(circle, 0.3f));
	ASSERT_GT(vec(moving.velocity).norm(), 0.5f);
	const Reference replaced = sample(build(TrajectoryId::Figure8, moving), 0.0f);
	expect_near(vec(replaced.position), vec(moving.position), 1e-6f);
	expect_near(vec(replaced.velocity), Vector3{}, 0.0f);
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
	EXPECT_FALSE(trajectory_generator::build_hold({{0.0f, 0.0f, -1.0f}}, NAN, plan));
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
	start.position[0] = INFINITY;
	EXPECT_FALSE(trajectory_generator::build_trajectory(static_cast<int>(TrajectoryId::Circle), valid, start, plan));
}

TEST(Se3Integral, BoundedFreezableIntegration)
{
	float value = 0.0f;

	// Plain integration inside the bound: the applied rate is the input.
	EXPECT_FLOAT_EQ(integrate_bounded(value, 0.5f, 0.1f, 1.0f, false), 0.5f);
	EXPECT_FLOAT_EQ(value, 0.05f);

	// Frozen: unchanged, rate zero.
	EXPECT_FLOAT_EQ(integrate_bounded(value, 0.5f, 0.1f, 1.0f, true), 0.0f);
	EXPECT_FLOAT_EQ(value, 0.05f);

	// The step that reaches the bound applies only the part up to it, and the
	// reported rate is exactly that change; further outward input is discarded.
	float before = value;
	float rate = integrate_bounded(value, 20.0f, 0.1f, 1.0f, false);
	EXPECT_FLOAT_EQ(value, 1.0f);
	EXPECT_NEAR(rate * 0.1f, value - before, 1e-6f);
	EXPECT_NEAR(rate, 9.5f, 1e-5f);
	EXPECT_FLOAT_EQ(integrate_bounded(value, 0.5f, 0.1f, 1.0f, false), 0.0f);
	EXPECT_FLOAT_EQ(value, 1.0f);

	// The same at the negative bound.
	float negative = -0.9f;
	before = negative;
	rate = integrate_bounded(negative, -2.0f, 0.1f, 1.0f, false);
	EXPECT_FLOAT_EQ(negative, -1.0f);
	EXPECT_NEAR(rate * 0.1f, negative - before, 1e-6f);

	// Input back inside acts immediately (no unwinding delay).
	EXPECT_FLOAT_EQ(integrate_bounded(value, -0.5f, 0.1f, 1.0f, false), -0.5f);
	EXPECT_FLOAT_EQ(value, 0.95f);

	// Any sequence: the value stays bounded and moves exactly by rate * dt.
	float walk = 0.0f;

	for (int k = 0; k < 200; ++k) {
		const float input = 3.0f * sinf(0.1f * static_cast<float>(k)) + 1.0f;
		const float previous = walk;
		const float applied = integrate_bounded(walk, input, 0.05f, 0.4f, (k % 17) == 0);
		ASSERT_LE(fabsf(walk), 0.4f);
		ASSERT_NEAR(applied * 0.05f, walk - previous, 1e-6f) << k;
	}

	// A smaller bound (larger gain) clamps; a zero bound (zero gain) resets.
	EXPECT_FLOAT_EQ(integrate_bounded(value, 0.0f, 0.1f, 0.5f, true), 0.0f);
	EXPECT_FLOAT_EQ(value, 0.5f);
	EXPECT_FLOAT_EQ(integrate_bounded(value, 0.5f, 0.1f, 0.0f, false), 0.0f);
	EXPECT_FLOAT_EQ(value, 0.0f);

	// No time step or non-finite input: nothing integrated.
	EXPECT_FLOAT_EQ(integrate_bounded(value, 0.5f, 0.0f, 1.0f, false), 0.0f);
	EXPECT_FLOAT_EQ(integrate_bounded(value, NAN, 0.1f, 1.0f, false), 0.0f);
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

TEST(Se3Controller, PositionIntegralBoundaryStepReportsTheAppliedRate)
{
	const Reference reference = home();
	const TranslationGains gains{4.0f, 3.0f, Vector3{0.3f, 0.3f, 0.6f}};
	const float limit_xy = 1.0f / 0.3f;
	State state{};
	state.attitude = matrix::Dcmf{};
	state.position = Vector3{0.3f, -0.2f, -1.2f};   // e_x = (0.3, -0.2, 0) inside the 0.5 m saturation
	state.velocity = Vector3{0.1f, 0.2f, 0.0f};

	// One step short of the bound on x and y; the next step crosses it.
	PositionIntegral integral{};
	integral.value = Vector3{limit_xy - 0.001f, -limit_xy + 0.0005f, 0.0f};
	const PositionIntegral before = integral;
	TranslationOutput previous{};
	ASSERT_TRUE(Controller::compute_translation(state, reference, gains, before, previous));

	const float dt = 0.01f;
	ASSERT_TRUE(Controller::update_integral(state, reference, gains, dt, false, integral));
	EXPECT_FLOAT_EQ(integral.value(0), limit_xy);
	EXPECT_FLOAT_EQ(integral.value(1), -limit_xy);

	// The reported rate is the change actually applied (not the input 0.3,
	// and not zero), and no second derivative is reported at the bound.
	for (int axis = 0; axis < 3; ++axis) {
		EXPECT_NEAR(integral.rate(axis) * dt, integral.value(axis) - before.value(axis), 1e-6f) << axis;
	}

	EXPECT_NEAR(integral.rate(0), 0.1f, 1e-3f);
	EXPECT_NEAR(integral.rate(1), -0.05f, 1e-3f);
	EXPECT_FLOAT_EQ(integral.rate(2), 0.0f);           // e_x(2) = 0
	EXPECT_FLOAT_EQ(integral.rate_derivative(0), 0.0f);
	EXPECT_FLOAT_EQ(integral.rate_derivative(1), 0.0f);

	// The integral acceleration changed by exactly -k_i dI/dt dt, the integral
	// term of the analytic first force derivative.
	TranslationOutput current{};
	ASSERT_TRUE(Controller::compute_translation(state, reference, gains, integral, current));
	expect_near(current.integral - previous.integral, -gains.integral.emult(integral.rate) * dt, 1e-6f);

	ForceDerivatives with{};
	ForceDerivatives without{};
	PositionIntegral no_rates = integral;
	no_rates.rate.setZero();
	no_rates.rate_derivative.setZero();
	ASSERT_TRUE(Controller::compute_force_derivatives(state, reference, gains, integral, current, with));
	ASSERT_TRUE(Controller::compute_force_derivatives(state, reference, gains, no_rates, current, without));
	expect_near(with.first - without.first, -gains.integral.emult(integral.rate), 1e-6f);

	// Held at the bound against further outward error: no change, zero rates.
	const PositionIntegral held = integral;
	ASSERT_TRUE(Controller::update_integral(state, reference, gains, dt, false, integral));
	expect_near(integral.value, held.value, 0.0f);
	EXPECT_FLOAT_EQ(integral.rate(0), 0.0f);
	EXPECT_FLOAT_EQ(integral.rate(1), 0.0f);

	// Freeze, then resume: frozen steps change nothing; resuming with an error
	// back inside integrates at sat(e_x) with d^2I/dt^2 = e_v again.
	state.position = Vector3{-0.2f, 0.1f, -1.2f};
	ASSERT_TRUE(Controller::update_integral(state, reference, gains, dt, true, integral));
	expect_near(integral.value, held.value, 0.0f);
	expect_near(integral.rate, Vector3{}, 0.0f);
	ASSERT_TRUE(Controller::update_integral(state, reference, gains, dt, false, integral));
	EXPECT_FLOAT_EQ(integral.rate(0), -0.2f);
	EXPECT_FLOAT_EQ(integral.rate(1), 0.1f);
	EXPECT_FLOAT_EQ(integral.rate_derivative(0), 0.1f);
	EXPECT_FLOAT_EQ(integral.rate_derivative(1), 0.2f);
	EXPECT_LT(fabsf(integral.value(0)), limit_xy);
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

TEST(Se3Controller, ZeroIntegralGainsEqualTheNonIntegralController)
{
	const Trajectory plan = build(TrajectoryId::Figure8, home());
	const Reference reference = sample(plan, pattern_time(plan, 0.3f));
	const TranslationGains plain{4.0f, 3.0f, Vector3{}};
	State state{};
	state.position = vec(reference.position) + Vector3{0.4f, -0.3f, 0.2f};
	state.velocity = vec(reference.velocity) + Vector3{0.1f, 0.05f, -0.1f};
	state.attitude = matrix::Dcmf{matrix::Eulerf{0.05f, -0.04f, 0.2f}};
	state.angular_velocity = Vector3{0.1f, -0.05f, 0.02f};

	// A previously accumulated integral is cleared by zero gains, so the
	// geometry is exactly the controller without the integral term.
	PositionIntegral integral{};
	integral.value = Vector3{0.5f, -0.4f, 0.3f};
	integral.rate = Vector3{0.1f, 0.1f, 0.1f};
	integral.rate_derivative = Vector3{0.2f, 0.2f, 0.2f};
	ASSERT_TRUE(Controller::update_integral(state, reference, plain, 0.01f, false, integral));
	expect_near(integral.value, Vector3{}, 0.0f);
	expect_near(integral.rate, Vector3{}, 0.0f);
	expect_near(integral.rate_derivative, Vector3{}, 0.0f);

	GeometricOutput with{};
	GeometricOutput without{};
	ASSERT_TRUE(Controller::compute_geometry(state, reference, plain, integral, with));
	ASSERT_TRUE(Controller::compute_geometry(state, reference, plain, PositionIntegral{}, without));
	expect_near(with.translation.integral, Vector3{}, 0.0f);
	expect_near(with.translation.specific_force, without.translation.specific_force, 0.0f);
	expect_near(with.force_derivatives.first, without.force_derivatives.first, 0.0f);
	expect_near(with.force_derivatives.second, without.force_derivatives.second, 0.0f);
	expect_near(with.desired_attitude.angular_velocity, without.desired_attitude.angular_velocity, 0.0f);
	expect_near(with.desired_attitude.angular_acceleration, without.desired_attitude.angular_acceleration, 0.0f);
	EXPECT_FLOAT_EQ(with.collective_acceleration, without.collective_acceleration);

	// The PHYSICAL law with K_I = 0 is the pure Lee moment for any e_I.
	PhysicalAttitudeGains gains = physical_gains();
	gains.integral.setZero();
	PhysicalMomentOutput lee{};
	PhysicalMomentOutput zero_gain{};
	ASSERT_TRUE(AttitudeLaws::physical(state, with.desired_attitude, test_inertia(), gains, Vector3{}, lee));
	ASSERT_TRUE(AttitudeLaws::physical(state, with.desired_attitude, test_inertia(), gains,
					   Vector3{0.3f, -0.2f, 0.1f}, zero_gain));
	expect_near(zero_gain.moment, lee.moment, 0.0f);
	expect_near(zero_gain.integral_feedback, Vector3{}, 0.0f);
}
