# --- return_to_origin node ---

add_executable(return_to_origin
    src/return_to_origin.cpp
)

target_link_libraries(return_to_origin PRIVATE
    mission_core
)

# ament_target_dependencies(return_to_origin
#     rclcpp
#     geometry_msgs
#     sensor_msgs
#     px4_msgs
# )

# --- return_to_origin_v2 node ---

add_executable(return_to_origin_v2
    src/return_to_origin_v2.cpp
)

target_link_libraries(return_to_origin_v2 PRIVATE
    mission_core
)

# ament_target_dependencies(return_to_origin_v2
#     rclcpp
#     geometry_msgs
#     sensor_msgs
#     px4_msgs
# )

# --- orbit_location node ---

add_executable(orbit_location
    src/orbit_location.cpp
)

target_link_libraries(orbit_location PRIVATE
    mission_core
)

# ament_target_dependencies(orbit_location
#     rclcpp
#     geometry_msgs
#     sensor_msgs
#     px4_msgs
# )

install(TARGETS
    return_to_origin
    return_to_origin_v2
    orbit_location
    DESTINATION ${INSTALL_DIR}
)
