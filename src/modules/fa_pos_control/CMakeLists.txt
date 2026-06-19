px4_add_module(
    MODULE modules__fa_pos_control
    MAIN fa_pos_control
    SRCS
        FAPositionControl.cpp
    DEPENDS
        px4_work_queue
)