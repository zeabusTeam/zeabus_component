#!/usr/bin/env python2
# FILE			: command_interfaces.py
# AUTHOR		: K.Supasan
# CREATE ON		: 2019, June 15 (UTC+0)
# MAINTAINER	: K.Supasan

# README

# REFERENCE
#   ref1 : http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29

import math
import rospy
from ..math.quaternion import Quaternion
from ..math import general as zeabus_math
from std_msgs.msg import Header
from zeabus_utility.msg import AUVState, ControlCommand
from zeabus_utility.srv import SendControlCommand, GetAUVState

class CommandInterfaces:

    def __init__( self , your_name ):

        rospy.loginfo( "Waiting service of auv state")
        rospy.wait_for_service( "/fusion/auv_state")
        self.command_to_fusion = rospy.ServiceProxy( '/fusion/auv_state', GetAUVState )

        rospy.loginfo( "Waiting service of command to control")
        rospy.wait_for_service( "/control/interfaces")
        self.command_to_control = rospy.ServiceProxy( '/control/interfaces', SendControlCommand )

        self.current_state = AUVState() # Use to collect current state msg

        self.current_quaternion = Quaternion()

        self.control_command = ControlCommand() # Use to collect control command msg

        self.control_command.header.frame_id = your_name
        self.control_command.header.seq = 0

        self.current_pose = [0 , 0 , 0 , 0 , 0 , 0]
        self.target_pose = [0 , 0 , 0 , 0 , 0 , 0]

        self.tuple_true = (True, True, True, True, True, True)

    def get_state( self ):
        try:
            self.current_state = self.command_to_fusion()
            self.current_pose[0] = self.current_state.auv_state.data.pose.pose.position.x
            self.current_pose[1] = self.current_state.auv_state.data.pose.pose.position.y
            self.current_pose[2] = self.current_state.auv_state.data.pose.pose.position.z
            self.current_quaternion.set_quaternion( (
                self.current_state.auv_state.data.pose.pose.orientation.x
                , self.current_state.auv_state.data.pose.pose.orientation.y
                , self.current_state.auv_state.data.pose.pose.orientation.z
                , self.current_state.auv_state.data.pose.pose.orientation.w )
            )

            ( self.current_pose[5] 
                , self.current_pose[4] 
                , self.current_pose[3] ) = self.current_quaternion.get_euler()
        except rospy.ServiceException , e:
            rospy.logfatal( "Service call AUVState Failed : %s" , e )

    def send_command( self ):
        try:
            # you must stamp time before send data
            self.control_command.target = tuple( self.target_pose )
            # self.control_command.header.stamp = rospy.get_rostime()
            self.control_command.header = Header()
            self.control_command.header.stamp = rospy.Time.now()
            self.command_to_control( self.control_command )
            self.control_command.header.seq += 1
        except rospy.ServiceException , e:
            rospy.logfatal( "Service call control interfaces Failed : %s" , e )

    def reset_state( self , roll = None, pitch = None):
        self.get_state()
        self.control_command.target = tuple( self.current_pose )
        self.target_pose[:] = self.current_pose[:] # make sure will not copy pointer
        if( roll != None ):
            self.target_pose[3] = roll
        if( pitch != None ):
            self.target_pose[4] = pitch
        self.control_command.mask = self.tuple_true
        self.send_command()

    # argument 3 yaw will mean you want to rotation will target yaw if that is true
    def relative_xy( self , x , y , target_yaw = True ):
        movement_x = 0
        movement_y = 0

        if( target_yaw ):
            movement_x = ( (x * math.cos( self.target_pose[5] ) ) 
                + ( y * math.cos( self.target_pose[5] + math.pi ) ) )
            movement_y = ( (y * math.sin( self.target_pose[5] ) ) 
                + ( y * math.sin( self.target_pose[5] + math.pi ) ) )
        else:
            self.get_state()
            movement_x = ( (x * math.cos( self.current_pose[5] ) ) 
                + ( y * math.cos( self.current_pose[5] + math.pi ) ) )
            movement_y = ( (y * math.sin( self.current_pose[5] ) ) 
                + ( y * math.sin( self.current_pose[5] + math.pi ) ) )

        self.target_pose[ 0 ] = self.current_pose[ 0 ] + movement_x
        self.target_pose[ 1 ] = self.current_pose[ 1 ] + movement_y

        self.control_command.mask = ( True , True , False , False , False , False )
        self.send_command()

    def absolute_xy( self , x , y ):
        self.target_pose[ 0 ] = x
        self.target_pose[ 1 ] = y
        self.control_command.mask = ( True , True , False , False , False , False )
        self.send_command()

    def relative_yaw( self, yaw , target_yaw = True ):

        if( target_yaw ):
            self.target_pose[ 5 ] = zeabus_math.bound_radian( self.target_pose[5] + yaw )
        else:
            self.get_state()
            self.target_pose[ 5 ] = zeabus_math.bound_radian( self.current_pose[5] + yaw )

        self.control_command.mask = ( False , False , False , False , False ,True )
        self.send_command()

    def absolute_yaw( self , yaw ):
        self.target_pose[ 5 ] = zeabus_math.bound_radian( yaw )

        self.control_command.mask = ( False , False , False , False , False ,True )
        self.send_command()

    def relative_z( self , z ):
        self.get_state()
        self.target_pose[ 2 ] =  self.current_pose[2] + z
        self.control_command.mask = ( False , False , True , False , False , False )
        self.send_command()

    def absolute_z( self , z ):
        self.target_pose[ 2 ] = z
        self.control_command.mask = ( False , False , True , False , False , False )
        self.send_command()

    def echo_data( self ):
        print( "current_pose is {:6.2f} {:6.2f} {:6.2f} {:6.2f} {:6.2f} {:6.2f}".format(
             self.current_pose[0] , self.current_pose[1] , self.current_pose[2]
            , self.current_pose[3] , self.current_pose[4] , self.current_pose[5] ) )
        print( "target_pose is {:6.2f} {:6.2f} {:6.2f} {:6.2f} {:6.2f} {:6.2f}".format(
             self.target_pose[0] , self.target_pose[1] , self.target_pose[2]
            , self.target_pose[3] , self.target_pose[4] , self.target_pose[5] ) )
        print( "Mask data are " , self.control_command.mask)
