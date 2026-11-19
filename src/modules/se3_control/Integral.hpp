// One axis of a bounded, freezable integral, shared by the translational
// (position-error) and the PHYSICAL attitude (e_R) integrators.

#pragma once

#include <cmath>

namespace se3_control
{

// Advance value by input * dt and keep |value| <= limit.
//
// Anti-windup: nothing is integrated while frozen, and an input that would
// push the value further beyond the limit is discarded. A zero limit (zero
// gain) resets the axis. Returns true if the input was integrated, so the
// applied rate dvalue/dt is input; otherwise it is zero.
inline bool integrate_bounded(float &value, float input, float dt, float limit, bool freeze)
{
	if (!(limit > 0.0f) || !std::isfinite(value)) {
		value = 0.0f;
		return false;
	}

	if (freeze || !(dt > 0.0f) || !std::isfinite(input)) {
		value = fminf(fmaxf(value, -limit), limit);
		return false;
	}

	const float next = value + input * dt;

	if (fabsf(next) >= limit) {
		// At the limit the integral holds; only an input back inside acts.
		value = next > 0.0f ? limit : -limit;
		return false;
	}

	value = next;
	return true;
}

} // namespace se3_control
