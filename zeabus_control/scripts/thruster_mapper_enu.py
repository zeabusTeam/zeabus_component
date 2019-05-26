#!/usr/bin/env python2
# FILE			: thruster_mapper_enu.py
# AUTHOR		: K.Supasan
# CREATE ON	    : 2019, May 26 (UTC+0)
# MAINTAINER	: K.Supasan
# ORIGINAL FILE : K.Supakit 2018/11/14

# README
#   This code have response about command to use matrix distribute for from world frame to 
#       robot frame.
#   In the future, I think this node should have process to detect about frame id to checkout
#       about piority to use so that is the future time.
#   If above line have I think we must to use thread manage about that process
#   About tf library. In different between python and cpp style coding. 
#       Python language have purpose about to interesting process and how to
#       CPP language have stlye to interesting about theory and ristric
#       All that make python don't have quaternion struct please ref02
#       You will know quaternion have been representing in mode numpy.dyarray for parameter
#           you must to pass by use tuple to pass parameter
#   In noted of ref02 you will know this tf library not optimized for speed!
#   Axes 4-str : will have affect about output and input. Only quaternion don't affect from this
#       first str will be 's'tatic or 'r'otation 
#           - 's' will return euler from that quaternion to origin
#           - 'r' will return euler form origin to that quaternion
#   Axes 4-tuple : 
#       Array or tuple will representing to ix+jy+kz+w to [ x , y , z , w ]

# REFERENCE
#   ref01 : http://wiki.ros.org/ROS/Tutorials/WritingServiceClient(python)
#   ref02 : http://docs.ros.org/melodic/api/tf/html/python/transformations.html#module-tf.transformations

import math
import rospy
import numpy
import lookup_pwm_force import lookup_pwm_force
from zeabus_utility.srv import SendThrottle ,  SendControlCommand

class ThrusterMapper:
    
    # function init that is function to constructor class
    # I will use to assign about const data that mean that data will not change
    def __init__( self ):

        rospy.init_node( 'control_thruster' )
        
        self.client_throttle = rospy.ServiceProxy( '/hardware/thruster_throttle' )

        self.client_state = rospy.ServiceProxy( '')

        self.server = rospy.Service( '/control/thruster' , SendControlCommand , self.callback )

        self.lookup_handle = lookup_pwm_force( "zeabus_control" 
                , "scripts" 
                , "4th_t200_16.txt" )

        cos_45 = math.cos( math.radians(45) )
        sin_45 = math.sin( math.radians(45) )

        # this variable will show about position to calculate force
        self.direction_linear = numpy.array( [ [ 0 , 0 , 1 ]    # thruster id 0
                ,[0 , 0 , 1 ]                                   # thruster id 1
                ,[0 , 0 , 1 ]                                   # thruster id 2
                ,[0 , 0 , 1 ]                                   # thruster id 3
                ,[-cos_45 , sin_45 , 0 ]                        # thruster id 4
                ,[-cos_45, -sin_45, 0 ]                         # thruster id 5 
                ,[cos_45 , sin_45 , 0 ]                         # thruster id 6 
                ,[cons_45 , -sin_45, 0 ] ] )                    # thruster id 7
        
        self.distance = numpy.array( [ [0.332 , 0.2202 , -0.023 ]   # thruster id 0
                ,[0.332 , -0.2202 , -0.023 ]                        # thruster id 1
                ,[-0.332 , 0.2202 , -0.023 ]                        # thruster id 2
                ,[-0.332 , -0.2202 , -0.023 ]                       # thruster id 3
                ,[0.3536 , 0.3536 , -0.023]                         # thruster id 4
                ,[0.3536 , -0.3536 , -0.023 ]                       # thruster id 5
                ,[-0.3536 , 0.3536, -0.023]                         # thruster id 6
                ,[-0.3536, -0.3536 , -0.023] ] )                    # thruster id 7

        # this variable will show about momentum to calculate about rotatio by euler
        self.direction_angular = numpy.array( [ 
                 numpy.cross( self.distance[0] , self.direction_linear[0])  # thruster id 0
                ,numpy.cross( self.distance[0] , self.direction_linear[1])      # thruster id 1
                ,numpy.cross( self.distance[0] , self.direction_linear[1])      # thruster id 2
                ,numpy.cross( self.distance[0] , self.direction_linear[1])      # thruster id 3
                ,numpy.cross( self.distance[0] , self.direction_linear[1])      # thruster id 4
                ,numpy.cross( self.distance[0] , self.direction_linear[1])      # thruster id 5 
                ,numpy.cross( self.distance[0] , self.direction_linear[1])      # thruster id 6 
                ,numpy.cross( self.distance[0] , self.direction_linear[1]) ] )  # thruster id 7

        self.direction = numpy.concatenate( 
            ( self.direction_linear , self.direction_angular ) 
            , axis = 1)

        self.direction_inverse = numpy.linalg.pinv( self.direction )

        self.current_quaternion = ( 0 , 0 , 0 , 1 ) 
        # At ref02 you can use tuple instead numpy.array
        
        self.target_quaternion = ( 0 , 0 , 0 , 1 )
        # for above quaternion or tuple variable will use to convert

    def update_state( self ):
        current_auv_state = self.client_state()
        

    def callback( self , request ):

        target_force = numpy.array( [ request.target(0) , request.target(1) , request.target(2)
                , request.target(3) , request.target(4) , request.target(5) ] )

        torque = numpy.matmul( self.direction_inverse.T , force.T )


