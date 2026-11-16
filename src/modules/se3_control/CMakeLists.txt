# Geometric SE(3) body-rate control module.
px4_add_module(
    MODULE modules__se3_control
    MAIN se3_control
    SRCS
        Runtime.cpp
        Controller.cpp
        AttitudeReference.cpp
        AttitudeLaws.cpp
        trajectory/Catalog.cpp
        trajectory/Sampler.cpp
    MODULE_CONFIG
        module.yaml
    DEPENDS
        px4_work_queue
)
