add_library( ROSFile SHARED
    path_file.cpp
    single_filter.cpp
    vector3_filter.cpp
)

target_link_libraries( ROSFile ${catkin_LIBRARIES})
