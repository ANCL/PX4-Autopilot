# Physical vehicle models and rotor-thrust allocation for SE3 PHYSICAL control.
# Used by se3_control and, when se3_control is enabled, by control_allocator
# (#include <modules/se3_control/physical/PhysicalAllocation.hpp>).
# PhysicalPath.cpp is se3_control runtime code and is built with that module.
px4_add_library(se3_physical
	PhysicalModel.cpp
	PhysicalAllocation.cpp
)

# make tests TESTFILTER=PhysicalAllocation
px4_add_unit_gtest(SRC PhysicalAllocationTest.cpp LINKLIBS se3_physical)
