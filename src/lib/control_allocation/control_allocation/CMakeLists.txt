############################################################################
#
#   Copyright (c) 2019 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

px4_add_library(ControlAllocation
	ControlAllocation.cpp
	ControlAllocation.hpp
	ControlAllocationPseudoInverse.cpp
	ControlAllocationPseudoInverse.hpp
	ControlAllocationSequentialDesaturation.cpp
	ControlAllocationSequentialDesaturation.hpp
	ControlAllocationQP.cpp
	ControlAllocationQP.hpp
	generated/fa_closest_workspace.c
	generated/fa_margin_workspace.c
	generated/src/algebra_libs.c
	generated/src/auxil.c
	generated/src/csc_math.c
	generated/src/csc_utils.c
	generated/src/error.c
	generated/src/kkt.c
	generated/src/matrix.c
	generated/src/osqp_api.c
	generated/src/qdldl.c
	generated/src/qdldl_interface.c
	generated/src/scaling.c
	generated/src/util.c
	generated/src/vector.c
	#control_allocation_qp_params.c
)

# Relax compiler flags for generated and OSQP C files
set_source_files_properties(
    generated/fa_closest_workspace.c
    generated/fa_margin_workspace.c
    generated/src/algebra_libs.c
    generated/src/auxil.c
    generated/src/csc_math.c
    generated/src/csc_utils.c
    generated/src/error.c
    generated/src/kkt.c
    generated/src/matrix.c
    generated/src/osqp_api.c
    generated/src/qdldl.c
    generated/src/qdldl_interface.c
    generated/src/scaling.c
    generated/src/util.c
    generated/src/vector.c
    PROPERTIES COMPILE_FLAGS "-Wno-error"
)

target_compile_options(ControlAllocation PRIVATE ${MAX_CUSTOM_OPT_LEVEL})

target_include_directories(ControlAllocation
	PUBLIC
		${CMAKE_CURRENT_SOURCE_DIR}
		${CMAKE_CURRENT_SOURCE_DIR}/generated
		${CMAKE_CURRENT_SOURCE_DIR}/generated/inc/public
	PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}/generated/inc/private
)

target_link_libraries(ControlAllocation PRIVATE mathlib)

px4_add_unit_gtest(SRC ControlAllocationPseudoInverseTest.cpp LINKLIBS ControlAllocation)
px4_add_functional_gtest(SRC ControlAllocationSequentialDesaturationTest.cpp LINKLIBS ControlAllocation ActuatorEffectiveness)
px4_add_unit_gtest(SRC ControlAllocationQPTest.cpp LINKLIBS ControlAllocation)