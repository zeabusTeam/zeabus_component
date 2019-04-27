add_library( ServiceGetSingleData
    sensor_imu.cpp
)
target_link_libraries( ServiceGetSingleData ${catkin_LIBRARIES} )
