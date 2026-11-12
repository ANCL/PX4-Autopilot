# Shared analytic trajectory generator for native PX4 controllers.

px4_add_library(trajectory_generator
  Sampler.cpp
  Catalog.cpp
)

target_include_directories(trajectory_generator PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/..
)
