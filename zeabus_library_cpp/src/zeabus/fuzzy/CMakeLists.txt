add_library( ControlFuzzy SHARED
    control_error_3dimension.cpp
)
add_dependencies( ControlFuzzy ${catkin_EXPORTED_TARGETS} )
target_link_libraries( ControlFuzzy ${catkin_LIBRARIES} )

add_library( Fuzzy  SHARED 
    control_fuzzy.cpp
<<<<<<< HEAD
    control_by_force.cpp
=======
    #control_velocity.cpp
    control_velocity_output.cpp
>>>>>>> 6ff860c33f1f9a1f6c049c896f2220b814c6b50b
)


