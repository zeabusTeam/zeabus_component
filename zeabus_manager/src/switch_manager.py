#!/usr/bin/env python2
# FILE			: switch_manager.py
# AUTHOR		: K.Supasan
# CREATE ON		: 2019, June 27 (UTC+0)
# MAINTAINER	: K.Supasan

# README

# REFERENCE

from __future__ import print_function

import rospy
from zeabus.control.command_interfaces import CommandInterfaces
from zeabus_utility.msg import MessagePlannerSwitch
from zeabus_utility.srv import SendBool
from std_msgs.msg import Header

_SWITCH_MANAGER_PRINT_RECEIVED_ = False
_SWITCH_MANAGER_SHOW_COUNT_ = False

class SwitchManager:

    def __init__( self ):

        topic_subscribe_switch = rospy.get_param( '~topic_planer_switch' , "/planner_switch")

        self.header = Header()
        self.header.frame_id = "mission"

        self.limit_process = rospy.get_param( '~limit_process' , 200 )

        self.start_time = rospy.get_rostime()
        self.limit_time = 5

        self.control_status = False # Close control now
        self.control = CommandInterfaces( "manager" )
        self.control.publish_data( "manager have deactive all axies of control" )
        self.control.deactivate( ("x" , "y" , "z" , "roll" , "pitch" , "yaw") )

        # We have to setup client doing task
        self.client_strategy = rospy.ServiceProxy( '/mission/strategy' , SendBool() )

        # First time we must to deactive all freedom of control
        self.count_switch = 0
        rospy.Subscriber( topic_subscribe_switch , MessagePlannerSwitch , self.listen_switch 
            , queue_size = 1)

        self.control.publish_data( "manager now spin node")
        rospy.spin()

    def listen_switch( self, message ):
        if( _SWITCH_MANAGER_PRINT_RECEIVED_ ):
            self.control.publish_data( "listen_switch is " + repr( message ) )

        if( message.planner_switch_state ):
            self.count_switch += 1
        else:
            self.count_switch += (-1)

        if( self.count_switch < 0 ):
            self.count_switch = 0
        elif( self.count_switch > self.limit_process ):
            self.count_switch = self.limit_process
        else:
            None

        if( _SWITCH_MANAGER_SHOW_COUNT_ ):
            self.control.publish_data( "Count switch is " 
                + str( self.count_switch) 
                + " and status of control is "
                + str( self.control_status ) )

        if( self.control_status ): # Now control already open or normal activate
            if( self.count_switch == 0 ):
                self.control_status = False
                self.control.publish_data( "manage now switch tell me time to shutdown" )
                self.control.deactivate( ("x" , "y" , "z" , "roll" , "pitch" , "yaw" ) )
                self.header.stamp = rospy.get_rostime()
                self.client_strategy( self.header , False )
            else:
                None
        else: # Now control already status close
            if( self.count_switch == self.limit_process ):
                self.control_status = True
                self.control.publish_data( "manage now switch tell me to open all" )
                self.control.reset_state()
                self.control.activate( ("x", "y" , "z" , "roll" , "pitch" , "yaw" ) )
                self.control.sleep()
                self.header.stamp = rospy.get_rostime()
                self.client_strategy( self.header , True )
            else:
                if( ( rospy.get_rostime() - self.start_time ).to_sec() > self.limit_time ):
                    self.control.deactivate( ("x" , "y" , "z" , "roll" , "pitch" , "yaw" ) )
                    self.start_time = rospy.get_rostime()

if __name__=="__main__":
    rospy.init_node( "manager_switch")

    manager = SwitchManager()
