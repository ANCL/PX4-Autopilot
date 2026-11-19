# control_allocator result for each accepted se3_wrench_setpoint.
uint64 timestamp                  # time since system start (microseconds)
uint64 timestamp_sample           # timestamp_sample of the allocated wrench [us]
bool valid                        # Allocated with all rotors running
bool compatible                   # Motor outputs and CA_ROTOR* effectiveness support physical allocation and the wrench was allocated; otherwise nothing is published for it
float32 thrust_requested          # [N]
float32[3] moment_requested       # [N m]
float32 thrust_achieved           # Wrench of the published motor commands [N]
float32[3] moment_achieved        # [N m]
float32 roll_pitch_scale          # Allocated fraction of the roll/pitch moment [0, 1]
float32 yaw_scale                 # Allocated fraction of the yaw moment [0, 1]
bool thrust_limited               # Collective changed by motor limits
float32[4] rotor_thrust           # CA_ROTOR*_CT times the published command [N]
float32[4] motor_command          # Published thrust-linear actuator_motors command (THR_MDL_FAC is applied by the output driver)
