#include "PhysicalAllocation.hpp"

#include <cmath>

namespace se3_physical
{
namespace
{

using Vector4 = matrix::Vector<float, kRotorCount>;

Vector4 column(const matrix::SquareMatrix<float, kRotorCount> &matrix, int index)
{
	Vector4 result;

	for (int i = 0; i < kRotorCount; ++i) {
		result(i) = matrix(i, index);
	}

	return result;
}

} // namespace

bool Allocation::configure(const Model &model)
{
	_configured = false;

	if (!valid(model)) {
		return false;
	}

	matrix::SquareMatrix<float, kRotorCount> effectiveness;

	for (int i = 0; i < kRotorCount; ++i) {
		const Rotor &rotor = model.rotors[i];
		const matrix::Vector3f moment = rotor.position.cross(rotor.axis) - rotor.axis * rotor.reaction;
		effectiveness(0, i) = -rotor.axis(2);
		effectiveness(1, i) = moment(0);
		effectiveness(2, i) = moment(1);
		effectiveness(3, i) = moment(2);
		_minimum[i] = rotor_thrust(rotor.thrust, rotor.thrust.u_min);
		_maximum[i] = rotor_thrust(rotor.thrust, rotor.thrust.u_max);
	}

	matrix::SquareMatrix<float, kRotorCount> inverse;

	if (!matrix::inv(effectiveness, inverse) || !inverse.isAllFinite()) {
		return false;
	}

	// Reject an ill-conditioned inverse and any rotor that does not add to the
	// collective (the desaturation below divides by these shares).
	const matrix::SquareMatrix<float, kRotorCount> identity = effectiveness * inverse;

	for (int i = 0; i < kRotorCount; ++i) {
		if (inverse(i, 0) <= 0.f) {
			return false;
		}

		for (int j = 0; j < kRotorCount; ++j) {
			if (fabsf(identity(i, j) - (i == j ? 1.f : 0.f)) > 1e-4f) {
				return false;
			}
		}
	}

	_model = model;
	_effectiveness = effectiveness;
	_inverse = inverse;
	_configured = true;
	return true;
}

bool Allocation::allocate(const Wrench &request, AllocationResult &result) const
{
	if (!_configured || !std::isfinite(request.thrust) || !request.moment.isAllFinite()) {
		return false;
	}

	// Rotor-thrust change per unit of each wrench axis (columns of B^-1).
	const Vector4 share = column(_inverse, 0);
	const Vector4 tilt = column(_inverse, 1) * request.moment(0) + column(_inverse, 2) * request.moment(1);
	const Vector4 yaw = column(_inverse, 3) * request.moment(2);

	// Express each rotor's limits as collective values: rotor i stays within
	// its limits for collective s and roll/pitch fraction c when
	//   low(i) - c p(i) <= s <= high(i) - c p(i).
	float low[kRotorCount];
	float high[kRotorCount];
	float p[kRotorCount];
	float collective_min = -INFINITY;
	float collective_max = INFINITY;

	for (int i = 0; i < kRotorCount; ++i) {
		low[i] = _minimum[i] / share(i);
		high[i] = _maximum[i] / share(i);
		p[i] = tilt(i) / share(i);
		collective_min = fmaxf(collective_min, low[i]);
		collective_max = fminf(collective_max, high[i]);
	}

	// 1. Collective without moments; only the rotor limits can change it.
	const float collective = fminf(fmaxf(request.thrust, collective_min), collective_max);

	// 2. Largest roll/pitch fraction c in [0, 1] for which some collective in
	// [max_i(low - c p), min(collective, min_j(high - c p))] exists. Each
	// constraint is linear in c and holds at c = 0, so c is exact.
	float tilt_scale = 1.f;

	for (int i = 0; i < kRotorCount; ++i) {
		for (int j = 0; j < kRotorCount; ++j) {
			const float rate = p[j] - p[i];

			if (rate > 0.f) {
				tilt_scale = fminf(tilt_scale, fmaxf(high[j] - low[i], 0.f) / rate);
			}
		}

		if (-p[i] > 0.f) {
			tilt_scale = fminf(tilt_scale, fmaxf(collective - low[i], 0.f) / -p[i]);
		}
	}

	float allocated_collective = collective;

	for (int i = 0; i < kRotorCount; ++i) {
		allocated_collective = fminf(allocated_collective, high[i] - tilt_scale * p[i]);
	}

	Vector4 thrust = share * allocated_collective + tilt * tilt_scale;

	// 3. Largest yaw fraction that keeps every rotor within its limits.
	float yaw_scale = 1.f;

	for (int i = 0; i < kRotorCount; ++i) {
		if (yaw(i) > 0.f) {
			yaw_scale = fminf(yaw_scale, fmaxf(_maximum[i] - thrust(i), 0.f) / yaw(i));

		} else if (yaw(i) < 0.f) {
			yaw_scale = fminf(yaw_scale, fmaxf(thrust(i) - _minimum[i], 0.f) / -yaw(i));
		}
	}

	thrust += yaw * yaw_scale;

	AllocationResult candidate{};

	for (int i = 0; i < kRotorCount; ++i) {
		// Only single-precision rounding remains outside the limits here.
		candidate.rotor_thrust[i] = fminf(fmaxf(thrust(i), _minimum[i]), _maximum[i]);

		if (!rotor_command(_model.rotors[i].thrust, candidate.rotor_thrust[i], candidate.command[i])) {
			return false;
		}
	}

	candidate.achieved = wrench(candidate.rotor_thrust);
	candidate.roll_pitch_scale = tilt_scale;
	candidate.yaw_scale = yaw_scale;
	candidate.thrust_limited = fabsf(candidate.achieved.thrust - request.thrust) >
				   1e-4f * fmaxf(1.f, fabsf(request.thrust));

	result = candidate;
	return true;
}

Wrench Allocation::wrench(const float thrust[kRotorCount]) const
{
	Vector4 rotor;

	for (int i = 0; i < kRotorCount; ++i) {
		rotor(i) = thrust[i];
	}

	const Vector4 wrench = _effectiveness * rotor;
	return Wrench{wrench(0), matrix::Vector3f{wrench(1), wrench(2), wrench(3)}};
}

void Allocation::thrust_from_commands(const float command[kRotorCount], float thrust[kRotorCount]) const
{
	for (int i = 0; i < kRotorCount; ++i) {
		thrust[i] = rotor_thrust(_model.rotors[i].thrust, command[i]);
	}
}

} // namespace se3_physical
