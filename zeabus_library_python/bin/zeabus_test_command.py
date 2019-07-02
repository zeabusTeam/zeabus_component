#!/usr/bin/env python2
# FILE			: zeabus_test_command.py
# AUTHOR		: K.Supasan
# CREATE ON		: 2019, June 27 (UTC+0)
# MAINTAINER	: K.Supasan

# README

# REFERENCE
#   ref01   : https://www.geeksforgeeks.org/vulnerability-input-function-python-2-x/

from __future__ import print_function

import rospy

from zeabus.control.command_interfaces import CommandInterfaces

if( __name__=="__main__"):
    rospy.init_node( "test_lib" )
    
    control = CommandInterfaces( "testing" )

    type_list = {
        "1" : "Test moving by force forward"
        ,"2" : "Test get current state"
    }

    print( "Please choose you tye" )
    for run_key in type_list.keys():
        print( "\tNumber  " + run_key + " for " + type_list[ run_key ]   ) 

    type_test = input( "Enter type do you want to test : " )

    rate = rospy.Rate( 5 )

    if( type_test == 1 ):
        print( "Welcome to test moving direct by force" )

        print( "Command to depth of water" )
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
    elif( type_test == 2 ):
        print( "Weclome to test get current state" )
        while( not rospy.is_shutdown() ):
            rate.sleep()
            control.get_state()
            control.echo_data()
            print( "=======================================================================")
    else:
        print( "Don't have this type")
