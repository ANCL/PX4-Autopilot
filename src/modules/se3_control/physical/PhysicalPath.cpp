#include "PhysicalPath.hpp"

#include "../AttitudeLaws.hpp"
#include "../Integral.hpp"

#include <px4_platform_common/log.h>

#include <cmath>

using namespace time_literals;

namespace se3_control
{
namespace
{
constexpr float kGravity = 9.80665f;

// The 100 Hz reference may miss at most two updates, and the attitude must
// come from the current estimator output (>= 200 Hz).
constexpr hrt_abstime kReferenceTimeout = 30_ms;
constexpr hrt_abstime kAttitudeTimeout = 20_ms;

// control_allocator answers every wrench at the gyro rate and stops publishing
// motor commands 20 ms after the last one (se3_physical::kWrenchTimeout), so
// silence beyond this means it is not allocating them.
constexpr hrt_abstime kAllocationTimeout = 30_ms;

// Attitude-integral bound |K_I e_I| <= J_ii alpha_max per body axis: the
// integral can correct at most these angular accelerations [rad/s^2].
constexpr float kIntegralAngularAccelerationXY = 8.0f;
constexpr float kIntegralAngularAccelerationZ = 2.0f;

// Longest gyro interval integrated; a longer gap integrates nothing.
constexpr hrt_abstime kIntegralMaxInterval = 20_ms;

// An allocation scale below this is a saturation, not rounding.
constexpr float kScaleSaturated = 1.0f - 1.0e-4f;

bool fresh(hrt_abstime now, hrt_abstime stamp, hrt_abstime limit)
{
	return stamp > 0 && now >= stamp && now - stamp <= limit;
}
} // namespace

PhysicalPath::~PhysicalPath()
{
	perf_free(_inner_perf);
	perf_free(_inner_interval_perf);
}

void PhysicalPath::init()
{
	if (_inner_perf == nullptr) {
		_inner_perf = perf_alloc(PC_ELAPSED, "se3_control: physical inner");
		_inner_interval_perf = perf_alloc(PC_INTERVAL, "se3_control: physical inner interval");
	}
}

void PhysicalPath::set_vehicle(float mass, const Vector3 &inertia)
{
	_mass = mass;
	_inertia.setZero();

	for (int axis = 0; axis < 3; ++axis) {
		_inertia(axis, axis) = inertia(axis);
	}
}

bool PhysicalPath::configured(const PhysicalAttitudeGains &gains) const
{
	if (!std::isfinite(_mass) || _mass <= 0.0f) {
		return false;
	}

	for (int axis = 0; axis < 3; ++axis) {
		if (!std::isfinite(_inertia(axis, axis)) || _inertia(axis, axis) <= 0.0f ||
		    !std::isfinite(gains.attitude(axis)) || gains.attitude(axis) <= 0.0f ||
		    !std::isfinite(gains.angular_velocity(axis)) || gains.angular_velocity(axis) <= 0.0f ||
		    !std::isfinite(gains.integral(axis)) || gains.integral(axis) < 0.0f) {
			return false;
		}
	}

	return true;
}

void PhysicalPath::set_reference(const Vector3 &specific_force, const DesiredAttitudeDynamics &desired,
				 hrt_abstime timestamp)
{
	_reference.specific_force = specific_force;
	_reference.desired = desired;
	_reference.timestamp = timestamp;
	_reference.valid = true;
}

void PhysicalPath::clear_reference()
{
	_reference.valid = false;
	reset_integral();
}

void PhysicalPath::start_monitoring(hrt_abstime active_time)
{
	_active_time = active_time;
	reset_integral();
}

void PhysicalPath::reset_integral()
{
	_attitude_integral.setZero();
	_integral_sample = 0;
}

void PhysicalPath::refresh_allocation_status()
{
	se3_allocation_status_s status{};

	if (_allocation_status_sub.update(&status)) {
		_allocation_status = status;
	}
}

const char *PhysicalPath::allocation_problem()
{
	refresh_allocation_status();

	failure_detector_status_s failure{};

	if (_failure_detector_sub.update(&failure)) {
		_motor_failure = failure.fd_motor;
	}

	if (_motor_failure) {
		return "motor failure detected";
	}

	// Only allocator replies since this activation count.
	if (_allocation_status.timestamp >= _active_time) {
		if (!_allocation_status.compatible) {
			return "physical allocation rejected (motor outputs or rotor effectiveness)";
		}

		if (!_allocation_status.valid) {
			return "physical allocation without all rotors";
		}
	}

	const hrt_abstime now = hrt_absolute_time();

	if (now - _active_time > kAllocationTimeout &&
	    !fresh(now, _allocation_status.timestamp, kAllocationTimeout)) {
		return "control_allocator not allocating the SE3 wrench";
	}

	return nullptr;
}

bool PhysicalPath::force_saturated() const
{
	return _allocation_status.timestamp >= _active_time && _allocation_status.compatible &&
	       (_allocation_status.thrust_limited || _allocation_status.roll_pitch_scale < kScaleSaturated);
}

PhysicalPath::Fault PhysicalPath::lee_wrench(const State &state, hrt_abstime attitude_timestamp,
		hrt_abstime sample_time, const PhysicalAttitudeGains &gains, float &thrust, Vector3 &moment,
		Vector3 &moment_integral, const char *&reason)
{
	const hrt_abstime now = hrt_absolute_time();

	if (!fresh(now, attitude_timestamp, kAttitudeTimeout) ||
	    !state.attitude.isAllFinite() || !state.angular_velocity.isAllFinite()) {
		reason = "attitude or angular velocity not valid for SE3";
		return Fault::State;
	}

	if (!_reference.valid || !fresh(now, _reference.timestamp, kReferenceTimeout)) {
		reason = "SE3 reference not updated";
		return Fault::Controller;
	}

	// Lee collective thrust f = -m (A/m) . (R e3) with the current attitude.
	const Vector3 body_z{state.attitude(0, 2), state.attitude(1, 2), state.attitude(2, 2)};
	const float collective = -(_reference.specific_force * body_z);
	PhysicalMomentOutput output{};

	if (!std::isfinite(collective) || collective <= 0.0f ||
	    !AttitudeLaws::physical(state, _reference.desired, _inertia, gains, _attitude_integral, output)) {
		reason = "physical wrench computation failed";
		return Fault::Controller;
	}

	thrust = _mass * collective;
	moment = output.moment;
	moment_integral = output.integral_feedback;

	// Integrate e_R over this gyro interval for the next sample. The reply to
	// the previous wrench says whether the allocator had to scale roll/pitch
	// or yaw; those axes are frozen while it does (anti-windup).
	refresh_allocation_status();
	const bool status_current = _allocation_status.timestamp >= _active_time && _allocation_status.compatible;
	const bool freeze_xy = status_current && _allocation_status.roll_pitch_scale < kScaleSaturated;
	const bool freeze_z = status_current && _allocation_status.yaw_scale < kScaleSaturated;
	const float dt = _integral_sample > 0 && sample_time > _integral_sample &&
			 sample_time - _integral_sample <= kIntegralMaxInterval ?
			 static_cast<float>(sample_time - _integral_sample) * 1.0e-6f : 0.0f;
	_integral_sample = sample_time;

	for (int axis = 0; axis < 3; ++axis) {
		const float acceleration = axis < 2 ? kIntegralAngularAccelerationXY : kIntegralAngularAccelerationZ;
		const float limit = gains.integral(axis) > 0.0f ?
				    _inertia(axis, axis) * acceleration / gains.integral(axis) : 0.0f;
		integrate_bounded(_attitude_integral(axis), output.attitude_error(axis), dt, limit,
				  axis < 2 ? freeze_xy : freeze_z);
	}

	return Fault::None;
}

bool PhysicalPath::neutral_wrench(State state, bool landed, const PhysicalAttitudeGains &gains,
				  float &thrust, Vector3 &moment)
{
	reset_integral();

	// Without a gyro sample nothing is published.
	if (!state.angular_velocity.isAllFinite()) {
		return false;
	}

	if (!state.attitude.isAllFinite()) {
		state.attitude.setIdentity(); // R_d = R: e_R = 0 for any R.
	}

	DesiredAttitudeDynamics current{};
	current.attitude = state.attitude;
	PhysicalMomentOutput output{};

	if (!AttitudeLaws::physical(state, current, _inertia, gains, Vector3{}, output)) {
		return false;
	}

	moment = output.moment;
	thrust = landed ? 0.0f : _mass * kGravity;
	return true;
}

void PhysicalPath::publish(const vehicle_angular_velocity_s &gyro, uint8_t nav_state, bool neutral,
			   float thrust, const Vector3 &moment, const Vector3 &moment_integral, float hover_thrust)
{
	se3_wrench_setpoint_s setpoint{};
	setpoint.timestamp_sample = gyro.timestamp_sample;
	setpoint.nav_state = nav_state;
	setpoint.neutral = neutral;
	setpoint.thrust = thrust;

	for (int axis = 0; axis < 3; ++axis) {
		setpoint.moment[axis] = moment(axis);
		setpoint.moment_integral[axis] = moment_integral(axis);
	}

#if defined(CONFIG_ARCH_BOARD_PX4_SITL)

	if (_inject_invalid_wrench) {
		setpoint.thrust = NAN;
	}

#endif

	setpoint.timestamp = hrt_absolute_time();
	_wrench_pub.publish(setpoint);

	vehicle_thrust_setpoint_s observation{};
	observation.timestamp_sample = gyro.timestamp_sample;
	observation.xyz[2] = -fminf(fmaxf(hover_thrust * thrust / (_mass * kGravity), 0.0f), 1.0f);
	observation.timestamp = setpoint.timestamp;
	_thrust_observation_pub.publish(observation);
}

void PhysicalPath::print_status() const
{
	PX4_INFO("physical vehicle: mass %.4f kg, inertia diag(%.6f, %.6f, %.6f) kg m^2", (double)_mass,
		 (double)_inertia(0, 0), (double)_inertia(1, 1), (double)_inertia(2, 2));
	PX4_INFO("attitude integral e_I: [%.4f, %.4f, %.4f] rad s", (double)_attitude_integral(0),
		 (double)_attitude_integral(1), (double)_attitude_integral(2));
	perf_print_counter(_inner_perf);
	perf_print_counter(_inner_interval_perf);
}

} // namespace se3_control
