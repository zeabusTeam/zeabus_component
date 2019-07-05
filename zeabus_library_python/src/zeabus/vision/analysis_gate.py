#!/usr/bin/env python2
# FILE			: analysis_gate.py
# AUTHOR		: K.Supasan
# CREATE ON		: 2019, June 29 (UTC+0)
# MAINTAINER	: K.Supasan

# README
# Data from vision have
#   int8 found (0 = not found , 1 = founded)
#   cx range -1 to 1
#   cy range -1 to 1
#   x_left position of left gate -1 to 1
#   x_right position of right gate -1 to 1
#   area 0 to 1

# REFERENCE

from __future__ import print_function

import math
import rospy

from ..transformation.broadcaster import Broadcaster

from .analysis_constant import *

from std_msgs.msg import String

from zeabus_utility.srv import VisionGate

class AnalysisGate:

    def __init__( self, child_frame_id = "base_gate" ):

        rospy.loginfo( "Waiting service of /vision/gate" )
        rospy.wait_for_service( "/vision/gate" )
        self.call_vision_data = rospy.ServiceProxy( "/vision/gate" , VisionGate )

        self.result = {
            'found' : 0
            ,'left_x' : 0
            ,'right_x' : 0
            ,'center_x' : 0
            ,'length_x' : 0 # 0 - 200
            ,'distance' : 0 # distance is calculate estimate for length of x
        }

    def call_data( self ):
        result = False

        try:
            raw_data = self.call_vision_data( String("") , String("") )
            result = True
        except rospy.ServiceException , e :
            rospy.logfatal( "Service call gate failed : %s" , e )

        if( result ):
            if( raw_data.found == 1 ):
                self.result['found'] = True
                self.result['left_x'] = raw_data.x_left * 100
                self.result['right'] = raw_data.x_right * 100
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
        
