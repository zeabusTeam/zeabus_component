add_library( ServiceGetData SHARED
    sensor_imu.cpp
    geometry_vector3_stamped.cpp
    header_float64.cpp
)
add_dependencies( ServiceGetSingleData ${catkin_EXPORTED_TARGETS})
target_link_libraries( ServiceGetSingleData 
    ${catkin_LIBRARIES} 
)
