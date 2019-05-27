add_library( ROSInterfaces SHARED
    single_thread.cpp
    time.cpp
)
target_link_libraries( ROSInterfaces ${catkin_LIBRARIES})

add_subdirectory( convert )

add_subdirectory( file )
