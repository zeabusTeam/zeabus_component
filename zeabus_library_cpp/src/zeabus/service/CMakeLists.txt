add_subdirectory( get_data )

add_library( GeneralService SHARED
    base_class.cpp
    pololu.cpp
    control_command.cpp 
)

add_dependencies( GeneralService 
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries( GeneralService 
    ${catkin_LIBRARIES} 
)
