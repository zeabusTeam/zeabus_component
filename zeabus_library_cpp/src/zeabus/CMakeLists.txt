# This file add directory from main CMakeLists.txt of package
# directory from this file is ./../../CMakeLists.txt
# directory from main CMakeLists to this is src/zeabus/CMakeLists.rxr

# This library is general purpose don't have any subdirectory CMakeLists.txt
add_library( General SHARED
    escape_code.cpp
    print_array.cpp
)

# Add library about sort algorithm
add_subdirectory( sort )

# Add library about convert
add_subdirectory( convert )

# Add library about serial
add_subdirectory( serial )

# Add library about apply from serial library for sensor part
add_subdirectory( sensor )

# Add library about apply from serial library for hardware part
add_subdirectory( hardware )

# Add library about interface of ros1 system
add_subdirectory( ros_interfaces )

# Add library about servier
add_subdirectory( service )

# Add library about client
add_subdirectory( client )
