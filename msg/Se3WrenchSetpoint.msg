# Physical body wrench from the onboard SE3 PHYSICAL controller (SI units).
# control_allocator allocates it in rotor-thrust space only while the SE3 mode
# in nav_state is the current navigation state and the PX4 rate controller is
# disabled; otherwise it is ignored. Normalized vehicle_torque_setpoint and
# vehicle_thrust_setpoint keep their meaning and are not used for this path.
uint64 timestamp                  # time since system start (microseconds)
uint64 timestamp_sample           # vehicle_angular_velocity sample used for the moment [us]
uint8 nav_state                   # SE3 navigation state that owns this setpoint
uint8 model                       # Physical model id (SE3_MODEL)
bool neutral                      # Rate-damping wrench while SE3 has no valid Lee reference
float32 thrust                    # Collective thrust along -body z [N]
float32[3] moment                 # Body FRD moment [N m]
