# This file add directory from main CMakeLists.txt of package
# directory from this file is ./../../CMakeLists.txt
# directory from main CMakeLists to this is src/zeabus/CMakeLists.rxr

# This library is general purpose don't have any subdirectory CMakeLists.txt
add_library( General SHARED
    escape_code.cpp
)

# Add library about convert
add_subdirectory( convert )

# Add library about serial
add_subdirectory( serial )

# Add library about apply from serial library
add_subdirectory( sensor )
