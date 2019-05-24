add_library( ServiceGetData SHARED
    sensor_imu.cpp
    geometry_vector3_stamped.cpp
    header_float64.cpp
    auv_state.cpp
)
add_dependencies( ServiceGetData ${catkin_EXPORTED_TARGETS})
target_link_libraries( ServiceGetData 
    ${catkin_LIBRARIES} 
)
