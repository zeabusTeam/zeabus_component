add_subdirectory( single_thread )

add_library( ClientFusion SHARED
    fusion_3_thread
)
target_link_libraries( ClientFusion ${catkin_LIBRARIES})
