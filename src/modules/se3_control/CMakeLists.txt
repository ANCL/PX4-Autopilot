# Native geometric outer loop; PID, allocation and outputs remain PX4-owned.
px4_add_module(
    MODULE modules__se3_control
    MAIN se3_control
    SRCS
        Runtime.cpp
        Controller.cpp
        AttitudeReference.cpp
        AttitudeLaws.cpp
    MODULE_CONFIG
        module.yaml
    DEPENDS
        trajectory_generator
        px4_work_queue
)
