add_subdirectory( single_thread )

add_library( ClientFusion SHARED
    fusion_3_thread
)

add_dependencies( ClientFusion 
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries( ClientFusion 
    ${catkin_LIBRARIES}
)
