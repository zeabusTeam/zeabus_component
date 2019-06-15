add_library( ControlFuzzy SHARED
    control_error_3dimension.cpp
)
add_dependencies( ControlFuzzy ${catkin_EXPORTED_TARGETS} )
target_link_libraries( ControlFuzzy ${catkin_LIBRARIES} )

add_library( Fuzzy  SHARED 
    control_fuzzy.cpp
)
