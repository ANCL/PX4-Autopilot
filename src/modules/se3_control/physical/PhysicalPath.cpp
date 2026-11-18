#include "PhysicalPath.hpp"

#include "../AttitudeLaws.hpp"

#include <parameters/param.h>
#include <px4_platform_common/log.h>

#include <float.h>
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

bool PhysicalPath::init(int32_t model_parameter)
{
	_model_id = static_cast<uint8_t>(model_parameter);

	if (model_parameter < 0 || model_parameter > UINT8_MAX ||
	    !se3_physical::get_model(_model_id, _model) || !se3_physical::valid(_model)) {
		PX4_ERR("SE3_MODEL %d is not a valid physical model", (int)model_parameter);
		return false;
	}

	update_parameters();

	if (_inner_perf == nullptr) {
		_inner_perf = perf_alloc(PC_ELAPSED, "se3_control: physical inner");
		_inner_interval_perf = perf_alloc(PC_INTERVAL, "se3_control: physical inner interval");
	}

	return true;
}

// The model's thrust curve and geometry hold only for its airframe and output
// mapping, and only if PX4 applies no thrust curve of its own.
bool PhysicalPath::parameters_match() const
{
	float thrust_model_factor = NAN;

	// FunctionMotors applies its curve only for a factor above FLT_EPSILON.
	if (param_get(param_find("THR_MDL_FAC"), &thrust_model_factor) != PX4_OK ||
	    !(fabsf(thrust_model_factor) <= FLT_EPSILON)) {
		return false;
	}

	for (int i = 0; i < _model.required_count; ++i) {
		const param_t handle = param_find(_model.required[i].name);
		int32_t value = 0;

		if (handle == PARAM_INVALID || param_get(handle, &value) != PX4_OK || value != _model.required[i].value) {
			return false;
		}
	}

	return true;
}

bool PhysicalPath::configured(const PhysicalAttitudeGains &gains) const
{
	for (int axis = 0; axis < 3; ++axis) {
		if (!std::isfinite(gains.attitude(axis)) || gains.attitude(axis) <= 0.0f ||
		    !std::isfinite(gains.angular_velocity(axis)) || gains.angular_velocity(axis) <= 0.0f) {
			return false;
		}
	}

	return _parameters_ok;
}

void PhysicalPath::set_reference(const Vector3 &specific_force, const DesiredAttitudeDynamics &desired,
				 hrt_abstime timestamp)
{
	_reference.specific_force = specific_force;
	_reference.desired = desired;
	_reference.timestamp = timestamp;
	_reference.valid = true;
}

const char *PhysicalPath::allocation_problem()
{
	se3_allocation_status_s status{};

	if (_allocation_status_sub.update(&status)) {
		_allocation_status = status;
	}

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
			return "physical allocation incompatible with model";
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

PhysicalPath::Fault PhysicalPath::lee_wrench(const State &state, hrt_abstime attitude_timestamp,
		const PhysicalAttitudeGains &gains, float &thrust, Vector3 &moment, const char *&reason) const
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
	    !AttitudeLaws::physical(state, _reference.desired, _model.inertia, gains, output)) {
		reason = "physical wrench computation failed";
		return Fault::Controller;
	}

	thrust = _model.mass * collective;
	moment = output.moment;
	return Fault::None;
}

bool PhysicalPath::neutral_wrench(State state, bool landed, const PhysicalAttitudeGains &gains,
				  float &thrust, Vector3 &moment) const
{
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

	if (!AttitudeLaws::physical(state, current, _model.inertia, gains, output)) {
		return false;
	}

	moment = output.moment;
	thrust = landed ? 0.0f : _model.mass * kGravity;
	return true;
}

void PhysicalPath::publish(const vehicle_angular_velocity_s &gyro, uint8_t nav_state, bool neutral,
			   float thrust, const Vector3 &moment, float hover_thrust)
{
	se3_wrench_setpoint_s setpoint{};
	setpoint.timestamp_sample = gyro.timestamp_sample;
	setpoint.nav_state = nav_state;
	setpoint.model = _model_id;
#if defined(CONFIG_ARCH_BOARD_PX4_SITL)

	if (_inject_unknown_model) {
		setpoint.model = 0xff;
	}

#endif
	setpoint.neutral = neutral;
	setpoint.thrust = thrust;

	for (int axis = 0; axis < 3; ++axis) {
		setpoint.moment[axis] = moment(axis);
	}

	setpoint.timestamp = hrt_absolute_time();
	_wrench_pub.publish(setpoint);

	vehicle_thrust_setpoint_s observation{};
	observation.timestamp_sample = gyro.timestamp_sample;
	observation.xyz[2] = -fminf(fmaxf(hover_thrust * thrust / (_model.mass * kGravity), 0.0f), 1.0f);
	observation.timestamp = setpoint.timestamp;
	_thrust_observation_pub.publish(observation);
}

void PhysicalPath::print_status() const
{
	PX4_INFO("physical model %s, parameters %s", _model.name,
		 _parameters_ok ? "match model" : "DO NOT match model (THR_MDL_FAC, airframe or outputs)");
	perf_print_counter(_inner_perf);
	perf_print_counter(_inner_interval_perf);
}

} // namespace se3_control
