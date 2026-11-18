// SE3_CTRL control paths and the PX4 controllers each one requires.
//
// Pure functions of Commander flags, so mode ownership is unit-tested
// independently of uORB and scheduling.

#pragma once

#include <cstdint>

namespace se3_control
{

// SE3_CTRL values, fixed until reboot. The experimental branch before this
// numbering used 0 = RATE_PX4 and 1 = PHYSICAL. PX4 never stores a parameter
// at its default, so the only value that branch could have persisted is 1,
// which now selects the default RATE_PX4. The runtime announces its path at
// boot so that change is visible.
enum class ControlPath : int32_t {
	AttitudePx4 = 0, // SE3 attitude + thrust -> mc_att_control -> mc_rate_control -> allocator
	RatePx4 = 1,     // SE3 body rate + thrust -> mc_rate_control -> allocator
	Physical = 2,    // SE3 Lee wrench [N, N m] -> physical allocation in control_allocator
};

inline bool control_path_from_parameter(int32_t value, ControlPath &path)
{
	switch (value) {
	case static_cast<int32_t>(ControlPath::AttitudePx4):
	case static_cast<int32_t>(ControlPath::RatePx4):
	case static_cast<int32_t>(ControlPath::Physical):
		path = static_cast<ControlPath>(value);
		return true;
	}

	return false;
}

inline const char *control_path_name(ControlPath path)
{
	switch (path) {
	case ControlPath::AttitudePx4: return "ATTITUDE_PX4";

	case ControlPath::RatePx4: return "RATE_PX4";

	case ControlPath::Physical: return "PHYSICAL";
	}

	return "unknown";
}

// PX4 controllers enabled by the registered mode (config_control_setpoints).
// Position, velocity and altitude control always stay disabled, so Commander
// also keeps multicopter position control off while SE3 owns the reference.
struct ModeControllers {
	bool attitude{false};
	bool rates{false};
	bool allocation{false};
};

// After a PHYSICAL fault the mode hands torque production back to
// mc_rate_control (native_rate_fallback), which then tracks the SE3 neutral
// rate setpoint until Commander leaves the mode.
inline ModeControllers mode_controllers(ControlPath path, bool native_rate_fallback)
{
	switch (path) {
	case ControlPath::AttitudePx4: return ModeControllers{true, true, true};

	case ControlPath::RatePx4: return ModeControllers{false, true, true};

	case ControlPath::Physical: return ModeControllers{false, native_rate_fallback, true};
	}

	return ModeControllers{};
}

// vehicle_control_mode flags that decide who commands the vehicle.
struct ControlModeFlags {
	bool attitude{false};
	bool rates{false};
	bool multicopter_position{false};
	bool allocation{false};
};

// True while Commander's active controllers are those this path requires;
// SE3 publishes no setpoint otherwise. A faulted PHYSICAL path accepts either
// rate-loop state while Commander applies the native-rate fallback.
inline bool controllers_match(ControlPath path, bool faulted, const ControlModeFlags &flags)
{
	if (flags.multicopter_position || !flags.allocation) {
		return false;
	}

	switch (path) {
	case ControlPath::AttitudePx4: return flags.attitude && flags.rates;

	case ControlPath::RatePx4: return !flags.attitude && flags.rates;

	case ControlPath::Physical: return !flags.attitude && (!flags.rates || faulted);
	}

	return false;
}

} // namespace se3_control
