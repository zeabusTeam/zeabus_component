#!/usr/bin/env python2
# FILE			: analysis_buoy.py
# AUTHOR		: K.Supasan
# CREATE ON		: 2019, June 30 (UTC+0)
# MAINTAINER	: K.Supasan

# README
# Data from vision have
#   int8 found 
#   float64 cx <center of x range -1 to 1> 
#   float64 cy <center of y range -1 to 1>
#   float64 score < range 0 to 1 is 1 confident that is picture>
#   float64 area <range 0 to 1> convert to percent <range 0 - 100>

# REFERENCE
#   ref01   : https://en.wikipedia.org/wiki/Camera_matrix

from __future__ import print_function

import math
import rospy

from ..transformation.broadcaster import Broadcaster

from ..math.quaternion import Quaternion

from .analysis_constant import *

from std_msgs.msg import String

from zeabus_utility.srv import VisionBuoy

class AnalysisBuoy:

    def __init__( self , child_frame_id = "base_buoy" ):

        print( "CONSTANT : ( AREA , SCORE ) : ( {:8.5f} , {:8.5f})".format( BUOY_AREA 
            , BUOY_SCORE) )

        rospy.loginfo( "Waiting service of /vision/buoy" )
        rospy.wait_for_service( "/vision/buoy" )
        self.call_vision_data = rospy.ServiceProxy( "/vision/buoy" , VisionBuoy )
        self.rate = rospy.Rate( 6 )

        self.result = {
            'found' : False
            ,'center_x' : 0.0
            ,'center_y' : 0.0
            ,'area' : 0.0
            ,'distance' : 0.0
        }

    def call_data( self ):
        # This variable use to return result from call_data
        self.buffer_center_x = []
        self.buffer_center_y = []
        self.buffer_area = []
        self.buffer_score = []

        self.found = False
        self.can_call = False
        count = 0
        count_found = 0 
        while( count < 5 ):
            count += 1
            temp_result = self.individual_call()
            if( temp_result ):
                count_found += 1 
                self.buffer_center_x.append( self.center_x )
                self.buffer_center_y.append( self.center_y )
                self.buffer_score.append( self.score )
                self.buffer_area.append( self.area )
                if( count_found == BUOY_MINIMUM_FOUND ):
                    self.found = True
                    break

        self.analysis_picture()

        return self.can_call
        
    def individual_call( self ):
        result = False
        found = False

        try:
            raw_data = self.call_vision_data( String('buoy') , String("") )
            result = True
            self.can_call = True
        except rospy.ServiceException , e :
            rospy.logfatal( "Service call buoy failed : %s" , e )

        if( result ):
            if( raw_data.found != 1 ):
                found = False
            elif( ( raw_data.area < BUOY_AREA ) or  ( raw_data.score * 100  < BUOY_SCORE ) ):
                found = False
            else:
                found = True

            if( found ):
                self.center_x = raw_data.cx * 100
                self.center_y = raw_data.cy * 100
                self.score = raw_data.score * 100
                self.area = raw_data.area * 100
            else:
                self.center_x = 0 
                self.center_y = 0
                self.score = 0 
                self.area = 0

        return found 

    def echo_data( self ):
        if( self.found ):
            print( "BUFFER_CENTER_X " + repr( self.buffer_center_x ) )
            print( "BUFFER_CENTER_y " + repr( self.buffer_center_y ) )
            print( "BUFFER_AREA" + repr( self.buffer_area ) )
            print( "RESULT " + repr( self.result ) )
        else:
            print( "Don't found picture")

    def analysis_picture( self ):
        if( self.found ):
            self.result['found'] = True
            self.result['area'] = sum( self.buffer_area ) / len( self.buffer_area )
            self.result['center_x'] = sum( self.buffer_center_x ) / len( self.buffer_center_x )
            self.result['center_y'] = sum( self.buffer_center_y ) / len( self.buffer_center_y )
            self.result['distance'] = BUOY_MIN_DISTANCE - ( ( self.result['area'] - BUOY_MIN_AREA )
                * BUOY_RATIO_DISTANCE )
        else:
            self.result['found'] = False
            self.result['area'] = 0
            self.result['center_x'] = 0
            self.result['center_y'] = 0
            
