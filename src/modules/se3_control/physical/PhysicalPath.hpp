// SE3_CTRL = PHYSICAL runtime support for se3_control.
//
// Holds the vehicle model, the reference shared by the 100 Hz SE3 loop and
// the gyro-triggered inner loop, the Lee wrench computation and publication,
// and the monitoring of control_allocator's replies. All calls are made from
// the se3_control work queue, so no locking is needed.
//
// The flight-mode lifecycle, fault handling and ownership checks stay in
// Runtime.cpp, shared with the other control paths.

#pragma once

#include "PhysicalModel.hpp"
#include "../Types.hpp"

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/failure_detector_status.h>
#include <uORB/topics/se3_allocation_status.h>
#include <uORB/topics/se3_wrench_setpoint.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>

namespace se3_control
{

class PhysicalPath
{
public:
	PhysicalPath() = default;
	~PhysicalPath();
	PhysicalPath(const PhysicalPath &) = delete;
	PhysicalPath &operator=(const PhysicalPath &) = delete;

	// Load and validate the SE3_MODEL model. False for an unknown or invalid one.
	bool init(int32_t model_parameter);

	const se3_physical::Model &model() const { return _model; }

	// THR_MDL_FAC = 0 and the model's airframe/output parameters. Re-read on
	// every parameter update; never changed here.
	void update_parameters() { _parameters_ok = parameters_match(); }
	bool parameters_ok() const { return _parameters_ok; }

	// Positive finite gains and matching parameters.
	bool configured(const PhysicalAttitudeGains &gains) const;

	// Reference from the 100 Hz loop, used by the inner loop until the next.
	void set_reference(const Vector3 &specific_force, const DesiredAttitudeDynamics &desired, hrt_abstime timestamp);
	void clear_reference() { _reference.valid = false; }

	// Called on SE3 mode entry; only allocator replies after it count.
	void start_monitoring(hrt_abstime active_time) { _active_time = active_time; }

	// Checked every 100 Hz cycle while active; the fault reason, or nullptr
	// while control_allocator is allocating the wrench with all rotors.
	const char *allocation_problem();

	enum class Fault { None, State, Controller };

	// Lee wrench from the current reference and the newest attitude and gyro
	// sample. On a fault, reason names it and thrust/moment are unchanged.
	Fault lee_wrench(const State &state, hrt_abstime attitude_timestamp, const PhysicalAttitudeGains &gains,
			 float &thrust, Vector3 &moment, const char *&reason) const;

	// Neutral wrench without a valid Lee reference: R_d = R, Omega_d = 0, so
	// the moment is rate damping plus Omega x J Omega (rotation is stopped,
	// not levelled). Weight-compensating thrust in the air, idle on the ground.
	bool neutral_wrench(State state, bool landed, const PhysicalAttitudeGains &gains,
			    float &thrust, Vector3 &moment) const;

	// se3_wrench_setpoint for control_allocator, plus the hover-scaled
	// normalized collective on vehicle_thrust_setpoint for observers (land
	// detector, logs); the allocator ignores the latter in this path.
	void publish(const vehicle_angular_velocity_s &gyro, uint8_t nav_state, bool neutral,
		     float thrust, const Vector3 &moment, float hover_thrust);

	void count_inner_start() { perf_count(_inner_interval_perf); perf_begin(_inner_perf); }
	void count_inner_end() { perf_end(_inner_perf); }
	void print_status() const;

#if defined(CONFIG_ARCH_BOARD_PX4_SITL)
	// SITL allocation-fault test: wrenches name an unknown model.
	void inject_unknown_model(bool inject) { _inject_unknown_model = inject; }
#endif

private:
	bool parameters_match() const;

	struct Reference {
		Vector3 specific_force{}; // A/m in NED [m/s^2].
		DesiredAttitudeDynamics desired{};
		hrt_abstime timestamp{0};
		bool valid{false};
	};

	uint8_t _model_id{0};
	se3_physical::Model _model{};
	bool _parameters_ok{false};
	Reference _reference{};
	hrt_abstime _active_time{0};

	uORB::Publication<se3_wrench_setpoint_s> _wrench_pub{ORB_ID(se3_wrench_setpoint)};
	uORB::Publication<vehicle_thrust_setpoint_s> _thrust_observation_pub{ORB_ID(vehicle_thrust_setpoint)};
	uORB::Subscription _allocation_status_sub{ORB_ID(se3_allocation_status)};
	uORB::Subscription _failure_detector_sub{ORB_ID(failure_detector_status)};
	se3_allocation_status_s _allocation_status{};
	bool _motor_failure{false};

	perf_counter_t _inner_perf{nullptr};
	perf_counter_t _inner_interval_perf{nullptr};

#if defined(CONFIG_ARCH_BOARD_PX4_SITL)
	bool _inject_unknown_model {false};
#endif
};

} // namespace se3_control
