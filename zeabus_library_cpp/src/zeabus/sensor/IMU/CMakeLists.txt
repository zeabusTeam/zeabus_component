add_library( IMUConnector SHARED 
    packet.cpp
    interface.cpp
)
target_link_libraries( IMUConnector 
    SynchronousPort 
    ${Boost_LIBRARIES}
)
