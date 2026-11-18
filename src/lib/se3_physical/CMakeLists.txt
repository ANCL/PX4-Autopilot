# Physical vehicle models and rotor-thrust allocation for onboard SE3 PHYSICAL
# control (used by se3_control and, when it is enabled, control_allocator).
px4_add_library(se3_physical
	PhysicalModel.cpp
	PhysicalAllocation.cpp
)
target_include_directories(se3_physical PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# make tests TESTFILTER=PhysicalAllocation
px4_add_unit_gtest(SRC PhysicalAllocationTest.cpp LINKLIBS se3_physical)
