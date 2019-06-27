#!/usr/bin/env python2
# FILE			: command_interfaces.py
# AUTHOR		: K.Supasan
# CREATE ON		: 2019, June 15 (UTC+0)
# MAINTAINER	: K.Supasan

# README
#   2019 06 16 follow control interfaces version 2 master command use part of mask only
#   2019 06 26 I add check about status of node for you

# REFERENCE
#   ref1 : http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29

import tf
import math
import rospy
from ..math.quaternion import Quaternion
from ..math import general as zeabus_math
from std_msgs.msg import Header, String
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

        rospy.loginfo( "Waiting service of master to control")
        rospy.wait_for_service( "/control/master")
        self.command_master_control = rospy.ServiceProxy( '/control/master' ,SendControlCommand )
        self.master_command = ControlCommand() # This use only master command
        self.master_command.target = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.master_command.header.seq = 0
        self.set_name( your_name )

        self.pub_message = rospy.Publisher( "/mission/command", String, queue_size = 10 )

        self.current_state = AUVState() # Use to collect current state msg

        self.current_quaternion = Quaternion()
        self.target_quaternion = Quaternion()

        self.control_command = ControlCommand() # Use to collect control command msg

        self.control_command.header.frame_id = your_name
        self.control_command.header.seq = 0

        self.current_pose = [0 , 0 , 0 , 0 , 0 , 0]
        self.target_pose = [0 , 0 , 0 , 0 , 0 , 0]

        self.tuple_true = (True, True, True, True, True, True)

        self.tf_listener = tf.TransformListener()

    def update_target( self ):
        temp = self.tf_listener.lookupTransform( "odom" , "flag_target" , rospy.Time(0) )
        self.target_pose[0] = temp[0][0]
        self.target_pose[1] = temp[0][1]
        self.target_pose[2] = temp[0][2]
        self.target_quaternion.set_quaternion( temp[1] )
        ( self.target_pose[3]
            , self.target_pose[4]
            , self.target_pose[5] ) = self.target_quaternion.get_euler()

    def set_name( self, your_name ):
        self.master_command.header.frame_id = your_name
        self.my_name = your_name

    def publish_data( self, message ):
        print( "=====>" + message )
        self.pub_message.publish(  String( self.my_name + message )  )

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
    def relative_xy( self , x , y , target_yaw = True , target_xy = True ):
        movement_x = 0
        movement_y = 0

        self.get_state()

        if( target_yaw ):
            movement_x = ( (x * math.cos( self.target_pose[5] ) ) 
                + ( y * math.cos( self.target_pose[5] + ( math.pi / 2 ) ) ) )
            movement_y = ( (x * math.sin( self.target_pose[5] ) ) 
                + ( y * math.sin( self.target_pose[5] + ( math.pi / 2 ) ) ) )
        else:
            movement_x = ( (x * math.cos( self.current_pose[5] ) ) 
                + ( y * math.cos( self.current_pose[5] + math.pi ) ) )
            movement_y = ( (x * math.sin( self.current_pose[5] ) ) 
                + ( y * math.sin( self.current_pose[5] + math.pi ) ) )

        print( "Adding x distance is {:6.3f}".format( movement_x ) )
        print( "Adding y distance is {:6.3f}".format( movement_y ) )
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

        print( "Active relative yaw is {:6.3f}".format( yaw ) ) 

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

    def relative_z( self , z , target_z = True ):
        self.get_state()
        if( target_z ):
            self.target_pose[ 2 ] += z
        else:
            self.target_pose[ 2 ] = self.current_pose[2] + z
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
        print( "Mask data are " , self.control_command.mask )

    def check_xy( self , error_x , error_y ):
        result = False
        self.get_state()
        temp_x = abs( self.target_pose[0] - self.current_pose[0] ) 
        temp_y = abs( self.target_pose[1] - self.current_pose[1] ) 
        if( temp_x < error_x and temp_y < error_y ):
            result = True
        print( "Check xy temp ({:6.2f} , {:6.2f}) : error ok ( {:6.2f} , {:6.2f})".format(
            temp_x , temp_y , error_x , error_y ) )
        if( rospy.is_shutdown() ):
            result = True
        return result
    
    def check_z( self , error_z ):
        result = False
        self.get_state()
        if( abs( self.target_pose[2] - self.current_pose[2] ) < error_z ):
            result = True
        if( rospy.is_shutdown() ):
            result = True
        return result 

    def check_yaw( self , error_yaw ):
        result = False
        self.get_state()
        temp = abs( zeabus_math.bound_radian( self.target_pose[5] - self.current_pose[5] ) ) 
        print( "error_yaw : temp {:5.2f} : {:5.2f}".format( error_yaw , temp ) )
        if( temp < error_yaw ):
            result = True
        if( rospy.is_shutdown() ):
            result = True
        return result

    def check_distance( self, distance ):
        self.get_state()
        result = False
        if( abs( 
                math.sqrt( pow( self.current_pose[0], 2 ) 
                    + pow( self.current_pose[1], 2 ) 
                    + pow ( self.current_pose[2] , 2 )  ) 
                - math.sqrt( pow( self.target_pose[0] , 2 ) 
                    + pow( self.target_pose[1] , 2 )
                    + pow( self.target_pose[2] , 2) ) ) 
            < distance ):
            result = True
        return result

## Warning below function will have direct effect of control interface to manage about error

    def master_call( self , master_mask ):
        print( "WARNING you are call master_mask " , master_mask )
        self.master_command.header.stamp = rospy.get_rostime()
        self.master_command.mask = master_mask
        try:
            self.command_master_control( self.master_command ) 
        except rospy.ServiceException , e:
            rospy.logfatal( "Service call control master Failed : %s" , e )

    # Please send in array that will help you to manage about data collect
    def deactivate( self , data ):
        temp_data = []
        for run_command in ( "x" , "y" , "z" , "roll" , "pitch" , "yaw" ):
            temp_data.append( not ( run_command in data ) )
        self.master_call( tuple( temp_data ) )

    # This function will help you reset data in axis you have command to activate
    def activate( self , data ):
        temp_data = []
        self.get_state()
        for run_command in ( "x" , "y" , "z" , "roll" , "pitch" , "yaw" ):
            temp_data.append( ( run_command in data ) )

        # control_command will help you to reset all pose that you command to activate
        self.control_command.mask = tuple( temp_data )
        self.send_command()

        self.master_call( self.control_command.mask ) 

if( __name__=="__main__"):
    rospy.init_node( "test_lib" )
    
    control = CommandInterfaces( "testing" )
    while( not rospy.is_shutdown() ):
        rospy.sleep( 0.5 )
        control.update_target()
        print( repr( control.target_pose ) )
        print( "=========================================================")
