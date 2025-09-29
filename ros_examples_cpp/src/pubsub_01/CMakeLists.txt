cmake_minimum_required(VERSION 3.8)
project(pubsub_01)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default to C++ 20
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 20)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

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

# ----- Necessary CMake Calls to add your ROS2 Node -----

# Make executable and add your libraries
add_executable(talker src/publisher_node.cpp)
add_executable(listener src/subscriber_node.cpp)

target_link_libraries(talker PUBLIC 
    rclcpp::rclcpp 
    ${std_msgs_TARGETS}
)

target_link_libraries(listener PUBLIC 
    rclcpp::rclcpp 
    ${std_msgs_TARGETS}
)

# Specify install function so ROS2 can find your node.
install(TARGETS 
    talker 
    listener 
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
