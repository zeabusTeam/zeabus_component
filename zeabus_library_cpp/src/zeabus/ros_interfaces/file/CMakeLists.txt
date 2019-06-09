add_library( ROSSingleFilter SHARED
    path_file.cpp
    single_filter.cpp
)
target_link_libraries( ROSSingleFilter ${catkin_LIBRARIES})

add_library( ROSVector3Filter SHARED
    path_file.cpp
    vector3_filter.cpp
)
target_link_libraries( ROSVector3Filter ${catkin_LIBRARIES} Array ROSConvert)

add_library( ROSQuaternionFilter SHARED
    path_file.cpp
    quaternion_filter.cpp
)
target_link_libraries( ROSQuaternionFilter ${catkin_LIBRARIES} Array)

add_library( ROSControlCommand SHARED
    path_file.cpp
    control_command.cpp
)
target_link_libraries( ROSControlCommand ${catkin_LIBRARIES} Array)

add_library( ROSRawFusion SHARED
    path_file.cpp
    raw_fusion.cpp 
)
add_dependencies( ROSRawFusion 
    ${catkin_EXPORTED_TARGETS}
)
target_link_libraries( ROSRawFusion ${catkin_LIBRARIES} )
