# --- return_to_origin node ---

add_executable(return_to_origin
    src/return_to_origin.cpp
)
target_include_directories(return_to_origin PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(return_to_origin PRIVATE
    mission_core
)

# --- return_to_origin_v2 node ---

add_executable(return_to_origin_v2
    src/return_to_origin_v2.cpp
)
target_include_directories(return_to_origin_v2 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(return_to_origin_v2 PRIVATE
    mission_core
)

# --- orbit_location node ---

add_executable(orbit_location
    src/orbit_location.cpp
)
target_include_directories(orbit_location PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(orbit_location PRIVATE
    mission_core
)

install(TARGETS
    return_to_origin
    return_to_origin_v2
    orbit_location
    DESTINATION ${INSTALL_DIR}
)
