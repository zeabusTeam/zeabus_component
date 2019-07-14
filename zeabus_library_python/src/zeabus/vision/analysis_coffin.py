#!/usr/bin/env python2
# FILE			: analysis_coffin.py
# AUTHOR		: K.Supasan
# CREATE ON		: 2019, July 14 (UTC+0)
# MAINTAINER	: K.Supasan

# README

# REFERENCE

from __future__ import print_function

import math
import rospy

from ..transformation.broadcaster import Broadcaster

from .analysis_constant import *

from std_msgs.msg import String

from zeabus_utility.srv import VisionGate

class AnalysisCoffin:

    def __init__( self, child_frame_id = "base_coffin" ):

        rospy.loginfo( "Waiting service of /vision/coffin" )
        rospy.wait_for_service( "/vision/gate" )
        self.call_vision_data = rospy.ServiceProxy( "/vision/coffin" , VisionGate )

        self.result = {
            'num_object' : 0
            ,'center_x' : 0.0 # Avaliable when you found num_object == 2
            ,'center_y' : 0.0 # Avaliable when you found num_object == 2
            ,'object_1' : { 
                'center_x' : 0.0 
                ,'center_y' : 0.0
                ,'area' : 0.0
                ,'rotation' : 0.0
                }
            ,'object_2' : {
                
            }
        }

        self.center_y = 0

        self.broadcaster = Broadcaster( "bottom_camera_optical" , child_frame_id )

    def call_data( self ):
        result = False

        try:
            raw_data = self.call_vision_data()
            result = True
        except rospy.ServiceException , e :
            rospy.logfatal( "Service call gate failed : %s" , e )

        if( result ):
            if( raw_data.found == 1 ):
                self.result['found'] = True
                self.result['left_x'] = raw_data.x_left * 100
                self.result['right_x'] = raw_data.x_right * 100
                self.result['center_x'] = raw_data.cx1 * 100
                self.result['center_y'] = raw_data.cy1 * 100
                self.center_y = raw_data.cy1 * 100
                self.analysis_picture()
            else:
                self.result['found'] = False
        else:
            self.result['found'] = False

        return result

    def echo_data( self ):
        if( self.result['found'] ):
            print( "x_point ( left , center , right ) : ({:6.3f} , {:6.3f} , {:6.3f})" .format( 
                self.result['left_x']
                , self.result['center_x'] 
                , self.result['right_x'] ) )
            print( "length data " + str( self.result['length_x'] ) )
            print( "distance " + str( self.result[ 'distance' ] ) )
        else:
            print( "Don't found target")

    def analysis_picture( self ):
        if( self.result['found'] ):
            self.result['length_x'] = self.result['right_x'] - self.result[ 'left_x' ]
            self.result['distance'] = ( ( ( GATE_LENGTH - self.result['length_x'] ) 
                * GATE_RATIO ) + GATE_NEAR )
            self.broadcaster.euler( ( ( self.result['center_x'] / 100 ) * 1.5 
                    , ( self.center_y / 100  ) * 1.5 
                    , -1.0 * self.result['distance'] ) 
                , 0 , 0 , 0 )
        
