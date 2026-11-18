# control_allocator result for each accepted se3_wrench_setpoint.
uint64 timestamp                  # time since system start (microseconds)
uint64 timestamp_sample           # timestamp_sample of the allocated wrench [us]
uint8 model                       # Physical model id
bool valid                        # Allocated with the model and all of its rotors running
bool compatible                   # Model known and motor outputs match it; otherwise outputs are held
float32 thrust_requested          # [N]
float32[3] moment_requested       # [N m]
float32 thrust_achieved           # Wrench of the published motor commands [N]
float32[3] moment_achieved        # [N m]
float32 roll_pitch_scale          # Allocated fraction of the roll/pitch moment [0, 1]
float32 yaw_scale                 # Allocated fraction of the yaw moment [0, 1]
bool thrust_limited               # Collective changed by rotor limits
float32[4] rotor_thrust           # [N]
float32[4] motor_command          # Published actuator_motors command
