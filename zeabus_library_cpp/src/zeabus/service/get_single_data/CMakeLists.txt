add_library( ServiceGetSingleData SHARED
    sensor_imu.cpp
)
add_dependencies( ServiceGetSingleData ${catkin_EXPORTED_TARGETS})
target_link_libraries( ServiceGetSingleData 
    ${catkin_LIBRARIES} 
)
