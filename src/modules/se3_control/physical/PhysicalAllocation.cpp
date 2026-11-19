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

bool thrust_differs(const Wrench &achieved, const Wrench &request)
{
	return fabsf(achieved.thrust - request.thrust) > 1e-4f * fmaxf(1.f, fabsf(request.thrust));
}

// Fraction of a requested moment (roll/pitch pair or yaw) that the achieved
// moment delivers, in [0, 1]: 1 - |achieved - requested| / |requested|. For
// achieved = c requested with c in [0, 1] this is exactly c, the allocation
// scale; an off-axis error or an overshoot lowers it, as does any achieved
// moment for a zero request. Errors within 1e-4 max(1 N m, |requested|), the
// tolerance thrust_differs uses for the collective, are single-precision
// rounding and count as fully achieved.
float achieved_fraction(float error, float requested)
{
	if (error <= 1e-4f * fmaxf(1.f, requested)) {
		return 1.f;
	}

	return requested > 0.f ? fminf(fmaxf(1.f - error / requested, 0.f), 1.f) : 0.f;
}

} // namespace

bool Allocation::configure(const Effectiveness &effectiveness, const float minimum[kRotorCount],
			   const float maximum[kRotorCount])
{
	_configured = false;

	if (!effectiveness.isAllFinite()) {
		return false;
	}

	matrix::SquareMatrix<float, kRotorCount> B;

	for (int i = 0; i < kRotorCount; ++i) {
		// Upward thrust (negative body z) only: lateral thrust is not allocated.
		const float thrust = -effectiveness(5, i);

		if (!(thrust > 0.f) || fabsf(effectiveness(3, i)) > 1e-3f * thrust || fabsf(effectiveness(4, i)) > 1e-3f * thrust
		    || !std::isfinite(minimum[i]) || !std::isfinite(maximum[i]) || minimum[i] < 0.f || minimum[i] >= maximum[i]) {
			return false;
		}

		B(0, i) = thrust;
		B(1, i) = effectiveness(0, i);
		B(2, i) = effectiveness(1, i);
		B(3, i) = effectiveness(2, i);
	}

	matrix::SquareMatrix<float, kRotorCount> inverse;

	if (!matrix::inv(B, inverse) || !inverse.isAllFinite()) {
		return false;
	}

	// Reject an ill-conditioned inverse and any rotor that does not add to the
	// collective (the desaturation below divides by these shares).
	const matrix::SquareMatrix<float, kRotorCount> identity = B * inverse;

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

	_matrix = B;
	_inverse = inverse;

	for (int i = 0; i < kRotorCount; ++i) {
		_minimum[i] = minimum[i];
		_maximum[i] = maximum[i];
	}

	_configured = true;
	return true;
}

bool Allocation::allocate(const Wrench &request, AllocationResult &result) const
{
	if (!_configured || !std::isfinite(request.thrust) || !request.moment.isAllFinite()) {
		return false;
	}

	// Command change per unit of each wrench axis (columns of B^-1).
	const Vector4 share = column(_inverse, 0);
	const Vector4 tilt = column(_inverse, 1) * request.moment(0) + column(_inverse, 2) * request.moment(1);
	const Vector4 yaw = column(_inverse, 3) * request.moment(2);

	// Express each motor's limits as collective values: motor i stays within
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

	// 1. Collective without moments; only the motor limits can change it.
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

	Vector4 command = share * allocated_collective + tilt * tilt_scale;

	// 3. Largest yaw fraction that keeps every motor within its limits.
	float yaw_scale = 1.f;

	for (int i = 0; i < kRotorCount; ++i) {
		if (yaw(i) > 0.f) {
			yaw_scale = fminf(yaw_scale, fmaxf(_maximum[i] - command(i), 0.f) / yaw(i));

		} else if (yaw(i) < 0.f) {
			yaw_scale = fminf(yaw_scale, fmaxf(command(i) - _minimum[i], 0.f) / -yaw(i));
		}
	}

	command += yaw * yaw_scale;

	AllocationResult candidate{};

	for (int i = 0; i < kRotorCount; ++i) {
		// Only single-precision rounding remains outside the limits here.
		candidate.command[i] = fminf(fmaxf(command(i), _minimum[i]), _maximum[i]);
		candidate.rotor_thrust[i] = rotor_thrust(i, candidate.command[i]);
	}

	candidate.achieved = wrench(candidate.command);
	candidate.roll_pitch_scale = tilt_scale;
	candidate.yaw_scale = yaw_scale;
	candidate.thrust_limited = thrust_differs(candidate.achieved, request);

	result = candidate;
	return true;
}

void Allocation::apply_commands(const Wrench &request, const float command[kRotorCount],
				AllocationResult &result) const
{
	for (int i = 0; i < kRotorCount; ++i) {
		result.command[i] = command[i];
		result.rotor_thrust[i] = rotor_thrust(i, command[i]);
	}

	result.achieved = wrench(command);
	result.thrust_limited = thrust_differs(result.achieved, request);

	const matrix::Vector3f &requested = request.moment;
	const matrix::Vector3f error = result.achieved.moment - requested;

	result.roll_pitch_scale = fminf(result.roll_pitch_scale,
					achieved_fraction(matrix::Vector2f{error(0), error(1)}.norm(),
							matrix::Vector2f{requested(0), requested(1)}.norm()));
	result.yaw_scale = fminf(result.yaw_scale, achieved_fraction(fabsf(error(2)), fabsf(requested(2))));
}

Wrench Allocation::wrench(const float command[kRotorCount]) const
{
	Vector4 u;

	for (int i = 0; i < kRotorCount; ++i) {
		u(i) = command[i];
	}

	const Vector4 w = _matrix * u;
	return Wrench{w(0), matrix::Vector3f{w(1), w(2), w(3)}};
}

} // namespace se3_physical
