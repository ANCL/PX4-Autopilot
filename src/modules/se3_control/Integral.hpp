// One axis of a bounded, freezable integral, shared by the translational
// (position-error) and the PHYSICAL attitude (e_R) integrators.

#pragma once

#include <cmath>

namespace se3_control
{

// Advance value by input * dt and keep |value| <= limit. Returns the rate
// dvalue/dt actually applied, so the stored value and the reported rate always
// agree: input inside the bound, (bound - value) / dt for the step that
// reaches it, and zero at the bound against an outward input.
//
// Anti-windup: nothing is integrated while frozen, without a time step or for
// a non-finite input (rate zero). A zero limit (zero gain) resets the axis. A
// value beyond a reduced limit (gain increase) is clamped as a discrete
// reconfiguration, not as integration.
inline float integrate_bounded(float &value, float input, float dt, float limit, bool freeze)
{
	if (!(limit > 0.0f) || !std::isfinite(value)) {
		value = 0.0f;
		return 0.0f;
	}

	value = fminf(fmaxf(value, -limit), limit);

	if (freeze || !(dt > 0.0f) || !std::isfinite(input)) {
		return 0.0f;
	}

	const float next = value + input * dt;

	if (fabsf(next) <= limit) {
		value = next;
		return input;
	}

	// The step reaches the bound: only the part up to it is applied.
	const float bounded = next > 0.0f ? limit : -limit;
	const float rate = (bounded - value) / dt;
	value = bounded;
	return rate;
}

} // namespace se3_control
