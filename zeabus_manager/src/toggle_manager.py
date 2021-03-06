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

        # For section of receive parameter
        topic_subscribe = rospy.get_param( '~topic_planer_switch' , "/planner_switch" )

        limit_toggle = rospy.get_param( "~limit_toggle" , 50 )

        start_mode = rospy.get_param( "~mode_start" , 0 )

        self.message_subscribe = MessagePlannerSwitch()
        
        self.control = CommandInterfaces( "manager" )
        self.control.publish_data( "manager have been start by deactivate all axies of control")
        self.control.deactivate( ['x' , 'y' , 'z' , 'roll' , 'pitch' , 'yaw' ] )

        # lock handle use to protect message when want to use because spin of python
        #   din't protect you about problem cach inherent
        self.message_locker = thread.allocate_lock()

        # We have to setup client doing task
        self.client_strategy = rospy.ServiceProxy( "/mission/strategy" , SendBool() )

        # set up ros system connection
        rospy.Subscriber( topic_subscribe, MessagePlannerSwitch, self.listen_switch, queue_size = 1 )

        self.loop_toggle( limit_toggle , start_mode )

    def loop_toggle( self , limit_toggle , start_mode ):

        count = 0   # Use to cound data received

        received_message = MessagePlannerSwitch()
        save_time = rospy.get_rostime()

        rate = rospy.Rate( 50 )

        mode = start_mode

        toggle_time = rospy.get_rostime()

        target_yaw = 0.0

        header = Header()
        header.frame_id = "odom"

        self.control.publish_data( "ToggleManager start loop manager")

        while not rospy.is_shutdown() :
            rate.sleep()

            with self.message_locker :
                received_message = self.message_subscribe

            if save_time < received_message.header.stamp :
                save_time = received_message.header.stamp
            else: # That mean your data in old data
                continue

            # process to manage about variable to use count swtich
            if received_message.planner_switch_state :
                if count == limit_toggle :
                    pass
                else:
                    count += 1
            else:
                if count == 0 :
                    pass
                else:
                    count -= 1

            if mode == 0 :  # Mode deactivate control

                if count == limit_toggle :
                    self.control.publish_data( "MODE 0 save current yaw and command force z" )
                    mode = 1 
                    self.control.get_state()

                if ( rospy.get_rostime() - toggle_time).to_sec() > 5 :
                    self.control.deactivate( ('x' , 'y' , 'z' , 'roll' , 'pitch' , 'yaw' ) )
                    toggle_time = rospy.get_rostime()

            elif mode == 1 : # Mode save yaw

                if count == 0 :
                    self.control.publish_data( "MODE 1 change to mode 2" )
                    mode = 2
                    self.control.deactivate( ('x' , 'y' , 'z' , 'roll' , 'pitch' , 'yaw' ) )
                    self.control.force_false( )
                    self.control.publish_data( "MODE 1 exit and target yaw at " + str(target_yaw) )
                else:
                    self.control.force_xyz( 0.0 , 0.0 , 0.5 )
                    self.control.get_state()
                    target_yaw = self.control.current_pose[5]

            elif mode == 2 : # Mode deactivate control waiting to open contrl

                if count == limit_toggle :
                    mode = 3
                    self.control.publish_data( "MODE 2 change to mode 3 and active mission" )
                    self.control.activate( ('x', 'y', 'z', 'yaw') )
                    self.control.absolute_z( -0.4 )
                    self.control.publish_data( "MODE 2 command absolute yaw is " + str(target_yaw) )
                    self.control.absolute_yaw( target_yaw )
                    self.control.sleep()
                    while not self.control.check_yaw( 0.15 ):
                        rate.sleep()
                    header.stamp = rospy.get_rostime()
                    self.client_strategy(  header , True )

                if ( rospy.get_rostime() - toggle_time ).to_sec() > 5 :
                    self.control.deactivate( ('x' , 'y' , 'z' , 'roll' , 'pitch' , 'yaw' ) )
                    toggle_time = rospy.get_rostime()

            else: # Mode to start control,by rotation and depth before call switch start mission

                if count == 0 :
                    self.control.publish_data( "MODE 3 change to mode 0 shutdown process" )
                    self.control.deactivate( ('x' , 'y' , 'z' , 'roll' , 'pitch' , 'yaw' ) )
                    mode = 0
                    toggle_time = rospy.get_rostime()

    def listen_switch( self , message ):

        with self.message_locker :
            self.message_subscribe = message
            self.message_subscribe.header.stamp = rospy.get_rostime()
        
    # Function __enter__ and __exit__ I have create for only remeber syntax
    def __enter__( self ):
        pass

    def __exit__( self , type , value , tb ):
        pass

if __name__=="__main__" :
    rospy.init_node( "toggle_switch")

    manager = ToggleManager()
