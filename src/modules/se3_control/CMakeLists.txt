# Geometric SE(3) control module: ATTITUDE_PX4, RATE_PX4 or PHYSICAL (SE3_CTRL).

# Physical rotor allocation (se3_physical), shared with control_allocator,
# which links it when this module is enabled.
add_subdirectory(physical)

px4_add_module(
    MODULE modules__se3_control
    MAIN se3_control
    SRCS
        Runtime.cpp
        Controller.cpp
        AttitudeReference.cpp
        AttitudeLaws.cpp
        physical/PhysicalPath.cpp
        trajectory/Catalog.cpp
        trajectory/Parameters.cpp
        trajectory/Sampler.cpp
    MODULE_CONFIG
        module.yaml
    DEPENDS
        px4_work_queue
        se3_physical
)

# Pure-math consistency checks (make tests TESTFILTER=Se3Math).
px4_add_unit_gtest(SRC Se3MathTest.cpp
    EXTRA_SRCS
        Controller.cpp
        AttitudeReference.cpp
        AttitudeLaws.cpp
        trajectory/Catalog.cpp
        trajectory/Sampler.cpp
)
