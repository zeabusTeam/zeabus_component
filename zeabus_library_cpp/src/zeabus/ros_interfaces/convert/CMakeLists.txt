add_library( ROSConvert SHARED
    geometry_msgs.cpp
)

target_link_libraries( ROSConvert ${catkin_LIBRARIES})
