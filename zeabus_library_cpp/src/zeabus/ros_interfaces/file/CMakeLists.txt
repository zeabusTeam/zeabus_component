add_library( ROSSingleFilter SHARED
    path_file.cpp
    single_filter.cpp
)
target_link_libraries( ROSSingleFilter ${catkin_LIBRARIES})

add_library( ROSVector3Filter SHARED
    path_file.cpp
    vector3_filter.cpp
)
target_link_libraries( ROSVector3Filter ${catkin_LIBRARIES})
