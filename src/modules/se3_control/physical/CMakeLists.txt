# Physical rotor allocation for SE3 PHYSICAL control, built from PX4's rotor
# effectiveness. Used by control_allocator when se3_control is enabled
# (#include <modules/se3_control/physical/PhysicalAllocation.hpp>).
# PhysicalPath.cpp is se3_control runtime code and is built with that module.
px4_add_library(se3_physical
	PhysicalAllocation.cpp
)

# make tests TESTFILTER=PhysicalAllocation
px4_add_unit_gtest(SRC PhysicalAllocationTest.cpp LINKLIBS se3_physical)
