#!/usr/bin/env python2
# FILE			: emergency_manager.py
# AUTHOR		: K.Supasan
# CREATE ON		: 2019, July 30 (UTC+0)
# MAINTAINER	: K.Supasan

# README

# REFERENCE
#   This file will always ues to kill thruster_mapper_node.py

import os

import rospy

from zeabus_utility.msg import MessagePlannerSwitch
from std_msgs.msg import Header

class EmergencyManager:

    def __init__( self ):

        rospy.init_node( "emergency_node")
    
        topic_subscribe_switch = rospy.get_param( '~topic_subscribe_switch' , "/planner_switch" )

        self.target_node = rospy.get_param( '~target_node_name' , 'control_thruster' )

        rospy.Subscriber( topic_subscribe_switch , MessagePlannerSwitch , self.listen_switch 
            , queue_size = 1)

        self.count = 0 
        self.limit = 10

    def listen_switch( self , message ):

        if message.planner_switch_state :
            self.count = 0
        else:
            self.count += 1

        if self.count == self.limit :
            print( "EmergencyManager Try to kill thruster node")
            os.system( "rosnode kill " + self.target_node + " &")
            rospy.sleep(1)
            print( "EmergencyManager wakeup")
            self.count = 0

if __name__=="__main__" :
    emergency_node = EmergencyManager()
    rospy.spin()
    
