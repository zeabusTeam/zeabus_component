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
    while( not rospy.is_shutdown() ):
        rospy.sleep( 0.5 )
        control.update_target()
        print( repr( control.target_pose ) ) 
        print( "=========================================================")
