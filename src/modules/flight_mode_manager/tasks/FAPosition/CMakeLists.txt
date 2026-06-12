px4_add_library(FlightTaskFAPosition
    FlightTaskFAPosition.cpp
)

target_link_libraries(FlightTaskFAPosition PRIVATE
    FlightTaskManualPosition
)

target_include_directories(FlightTaskFAPosition PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})