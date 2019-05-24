add_library( ClientSingleThread SHARED
    get_sensor_imu.cpp
    get_depth_command.cpp
    get_geometry_vector3_stamped.cpp
)

add_dependencies( ClientSingleThread 
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries( ClientSingleThread
    ${catkin_LIBRARIES}
)
