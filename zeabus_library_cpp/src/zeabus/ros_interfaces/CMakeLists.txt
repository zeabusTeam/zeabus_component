add_subdirectory( file )

add_subdirectory( convert )

add_library( ROSInterfaces SHARED
    single_thread.cpp
)
target_link_libraries( ROSInterfaces ${catkin_LIBRARIES})
