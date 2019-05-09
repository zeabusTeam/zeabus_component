add_library( ClientSingleThread SHARED
    get_sensor_imu.cpp
    get_depth_command.cpp
)

add_dependencies( ClientSingleThread ${catkin_EXPORTED_TARGETS} )
target_link_libraries( ClientSingleThread
    ${catkin_LIBRARIES}
)
