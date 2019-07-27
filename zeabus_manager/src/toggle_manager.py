#!/usr/bin/env python2
# FILE			: toggle_manager.py
# AUTHOR		: K.Supasan
# CREATE ON		: 2019, July 26 (UTC+0)
# MAINTAINER	: K.Supasan

# README
#   This file purpose to can't assign start yaw by switch that will help you
#   To can do mission by flip coin don't worry but you must use this find when IMU is Good
#   Design mode 
#   ===> mode 0 is mode shutdown control have loop command false
#   ===> mode 1 stil shutdown control but receive yaw to set target and force 0.5 in z axis
#   ===> mode 2 stil shutdown control and waiting command to run mission no loop command false
#   ===> mode 3 This mode will start control and start depth and rotation yaw

# REFERENCE

from __future__ import print_function

import rospy
import thread
from zeabus.control.command_interfaces import CommandInterfaces
from zeabus_utility.msg import MessagePlannerSwitch
from zeabus_utility.srv import SendBool
from std_msgs.msg import Header

class ToggleManager:

    def __init__( self ):

        # For subscriber
        topic_subscribe = rospy.get_param( '~topic_planer_switch' , "/planner_switch" )
        self.message_subscribe = MessagePlannerSwitch()

        limit_toggle = rospy.get_param( "~limit_toggle" , 50 )

        start_mode = rospy.get_param( "~mode_start" , 0 )

         

        # set up ros system connection
        rospy.Subscriber( topic_subscrbe , MessagePlannerSwitch , self.listen_switch )

    def loop_toggle( self , limit_toggle , start_mode ):
        None

    def listen_switch( self , message ):
        None

    # Function __enter__ and __exit__ I have create for only remeber syntax
    def __enter__( self ):
        pass

    def __exit__( self , type , value , tb ):
        pass
