# --- mission_core library ---

add_library(mission_core)

file(GLOB_RECURSE MISSION_CORE_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/mission_core/*.hpp
)

target_sources(mission_core PRIVATE
    ${MISSION_CORE_SOURCES}
)

target_include_directories(mission_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(mission_core
    rclcpp
    geometry_msgs
    sensor_msgs
    px4_msgs
)

install(TARGETS
    mission_core
    DESTINATION ${INSTALL_DIR}
)
