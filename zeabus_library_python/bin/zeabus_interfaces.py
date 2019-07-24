#!/usr/bin/env python2
# FILE			: zeabus_interfaces.py
# AUTHOR		: K.Supasan
# CREATE ON		: 2019, July 24 (UTC+0)
# MAINTAINER	: K.Supasan

# README

# REFERENCE

import rospy
from zeabus_utility.srv import SendFloat , SendBool
from zeabus_utility.srv import SendFloatResponse , SendBoolResponse
from zeabus.control.command_interfaces import CommandInterfaces

class NodeInterfaces:

    def __init__( self ):

        self.control = CommandInterfaces( "interface" )

        self.service_relative_x = rospy.Service( "/interfaces/relative_x" 
            , SendFloat() , self.relative_x )
        self.service_relative_y = rospy.Service( "/interfaces/relative_y" 
            , SendFloat() , self.relative_y )
        self.service_relative_yaw = rospy.Service( "/interfaces/relative_yaw"
            , SendFloat() , self.relative_yaw )

        self.service_absolute_z = rospy.Service( "/interfaces/absolute_z"
            , SendFloat() , self.absolute_z )
        self.service_absolute_yaw = rospy.Service( "/interfaces/absolute_yaw"
            , SendFloat() , self.absolute_yaw )

        self.service_gripper = rospy.Service( "/interfaces/gripper"
            , SendBool() , self.command_gripper )
        self.service_torpido = rospy.Service( "/interfaces/torpido" 
            , SendBool() , self.command_torpido )

    def relative_x( self , message ):
        self.control.relative_xy( message.data , 0 )
        self.control.publish_data( "Interfaces command forward " + str( message.data ) )
        return SendFloatResponse()

    def relative_y( self , message ):
        self.control.relative_xy( 0 , message.data )
        self.control.publish_data( "Interfaces command survey " + str( message.data ) )
        return SendFloatResponse()

    def relative_yaw( self , message ):
        self.control.relative_yaw( message.data )
        self.control.publish_data( "Interfaces command relative yaw " + str( message.data ) )
        return SendFloatResponse()

    def absolute_yaw( self , message ):
        self.control.absolute_yaw( message.data)
        self.control.publish_data( "Interfaces command absolute yaw " + str( message.data ) )
        return SendFloatResponse()

    def absolute_z( self , message ):
        self.control.absolute_z( message.data )
        self.control.publish_data( "Interfaces command absolute z " + str( message.data ) )
        return SendFloatResponse()

    def command_gripper( self , message ):
        if message.data :
            self.control.publish_data( "Interfaces command to release garlic") 
        else:
            self.control.publish_data( "Interfaces command to pick garlic")
        self.control.command_gripper( message.data )
        return SendBoolResponse()

    def command_torpido( self , message ):
        if message.data :
            self.control.publish_data( "Interfaces command to fire torpido")
        else:
            self.control.publish_data( "Interfaces command to reload torpido")
        self.control.command_torpido( message.data )
        return SendBoolResponse()

if __name__=="__main__":
    rospy.init_node( 'interfaces' )
    interfaces = NodeInterfaces()
    print("Node Interfaces waiting command by user")
    rospy.spin()
