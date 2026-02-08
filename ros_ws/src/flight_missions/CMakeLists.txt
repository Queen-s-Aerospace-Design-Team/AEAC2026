cmake_minimum_required(VERSION 3.22)
project(flight_missions)

# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Targets - FOLLOW THIS PATTERN for adding cmake executables (ROS Nodes)

# --- Mission Nodes ---

add_executable(return_to_origin 
    src/mission/return_to_origin.cpp
)

ament_target_dependencies(return_to_origin 
    rclcpp 
    geometry_msgs 
    sensor_msgs 
    px4_msgs
)

target_include_directories(return_to_origin PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(TARGETS return_to_origin DESTINATION lib/${PROJECT_NAME})

# Testing
if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)

    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)

    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
