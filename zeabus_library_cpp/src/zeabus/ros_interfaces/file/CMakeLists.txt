add_library( ROSFile SHARED
    path_file.cpp
)

target_link_libraries( ROSFile ${catkin_LIBRARIES})
