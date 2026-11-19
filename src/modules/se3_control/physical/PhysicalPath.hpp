// SE3_CTRL = PHYSICAL runtime support for se3_control.
//
// Holds the vehicle mass and inertia (SE3_MASS, SE3_J*), the reference shared
// by the 100 Hz SE3 loop and the gyro-triggered inner loop, the Lee wrench
// with its geometric attitude integral, and the monitoring of
// control_allocator's replies. All calls are made from the se3_control work
// queue, so no locking is needed.
//
// The flight-mode lifecycle, fault handling and ownership checks stay in
// Runtime.cpp, shared with the other control paths.

#pragma once

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

	void init();

	// SE3_MASS [kg] and the principal inertia SE3_JXX/JYY/JZZ [kg m^2].
	void set_vehicle(float mass, const Vector3 &inertia);

	// Positive mass and inertia, positive K_R and K_Omega, non-negative K_I.
	bool configured(const PhysicalAttitudeGains &gains) const;

	// Reference from the 100 Hz loop, used by the inner loop until the next.
	void set_reference(const Vector3 &specific_force, const DesiredAttitudeDynamics &desired, hrt_abstime timestamp);

	// Also resets the attitude integral (mode exit, fault).
	void clear_reference();

	// Called on SE3 mode entry; only allocator replies after it count.
	void start_monitoring(hrt_abstime active_time);

	// Checked every 100 Hz cycle while active; the fault reason, or nullptr
	// while control_allocator is allocating the wrench with all rotors.
	const char *allocation_problem();

	// The latest allocation since mode entry limited the collective or the
	// roll/pitch moment: the translational integral must not grow.
	bool force_saturated() const;

	enum class Fault { None, State, Controller };

	// Lee wrench from the current reference and the newest attitude and gyro
	// sample, then the attitude-integral update for that sample. On a fault,
	// reason names it and the outputs are unchanged.
	Fault lee_wrench(const State &state, hrt_abstime attitude_timestamp, hrt_abstime sample_time,
			 const PhysicalAttitudeGains &gains, float &thrust, Vector3 &moment, Vector3 &moment_integral,
			 const char *&reason);

	// Neutral wrench without a valid Lee reference: R_d = R, Omega_d = 0 and
	// no integral, so the moment is rate damping plus Omega x J Omega
	// (rotation is stopped, not levelled). Weight-compensating thrust in the
	// air, zero on the ground. Resets the attitude integral.
	bool neutral_wrench(State state, bool landed, const PhysicalAttitudeGains &gains,
			    float &thrust, Vector3 &moment);

	// se3_wrench_setpoint for control_allocator, plus the hover-scaled
	// normalized collective on vehicle_thrust_setpoint for observers (land
	// detector, logs); the allocator ignores the latter in this path.
	void publish(const vehicle_angular_velocity_s &gyro, uint8_t nav_state, bool neutral,
		     float thrust, const Vector3 &moment, const Vector3 &moment_integral, float hover_thrust);

	void count_inner_start() { perf_count(_inner_interval_perf); perf_begin(_inner_perf); }
	void count_inner_end() { perf_end(_inner_perf); }
	void print_status() const;

#if defined(CONFIG_ARCH_BOARD_PX4_SITL)
	// SITL allocation-fault test: published wrenches are non-finite, which
	// control_allocator rejects.
	void inject_invalid_wrench(bool inject) { _inject_invalid_wrench = inject; }
#endif

private:
	void refresh_allocation_status();
	void reset_integral();

	struct Reference {
		Vector3 specific_force{}; // A/m in NED [m/s^2].
		DesiredAttitudeDynamics desired{};
		hrt_abstime timestamp{0};
		bool valid{false};
	};

	float _mass{0.f};
	matrix::Matrix3f _inertia{};
	Reference _reference{};
	hrt_abstime _active_time{0};

	// Geometric attitude integral e_I = int e_R dt [rad s] and its last sample.
	Vector3 _attitude_integral{};
	hrt_abstime _integral_sample{0};

	uORB::Publication<se3_wrench_setpoint_s> _wrench_pub{ORB_ID(se3_wrench_setpoint)};
	uORB::Publication<vehicle_thrust_setpoint_s> _thrust_observation_pub{ORB_ID(vehicle_thrust_setpoint)};
	uORB::Subscription _allocation_status_sub{ORB_ID(se3_allocation_status)};
	uORB::Subscription _failure_detector_sub{ORB_ID(failure_detector_status)};
	se3_allocation_status_s _allocation_status{};
	bool _motor_failure{false};

	perf_counter_t _inner_perf{nullptr};
	perf_counter_t _inner_interval_perf{nullptr};

#if defined(CONFIG_ARCH_BOARD_PX4_SITL)
	bool _inject_invalid_wrench {false};
#endif
};

} // namespace se3_control
