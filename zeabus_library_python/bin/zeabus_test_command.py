#!/usr/bin/env python2
# FILE			: zeabus_test_command.py
# AUTHOR		: K.Supasan
# CREATE ON		: 2019, June 27 (UTC+0)
# MAINTAINER	: K.Supasan

# README

# REFERENCE

import rospy
from zeabus.control.command_interfaces import CommandInterfaces

if( __name__=="__main__"):
    rospy.init_node( "test_lib" )
    
    control = CommandInterfaces( "testing" )
    print( "Command to depth of water" )

    rate = rospy.Rate( 5 )

    control.absolute_z( -1.5 )
    print( "Waiting depth")
    while( not control.check_z( 0.15 ) ):
        rate.sleep()
        
    print( "Waiting yaw")
    while( not control.check_yaw( 0.15 ) ):
        rate.sleep()
    
    control.deactivate( ["x" , "y"] )
    control.force_xy( 1 , 0 , True )

    while( not rospy.is_shutdown() ):
        rate.sleep()
        distance = control.force_xy( 1 , 0 )
        print( "Now distance is " + str( distance ) )
        if( distance > 5 ):
            break

    control.activate( ["x" , "y"] ) 
    print( "Finish" )
