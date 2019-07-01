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

from std_msgs.msg import String

from zeabus_utility.srv import VisionBuoy

# Picutre have calibration before vision use them
# I have design paramer is
#       p1------------p2   <-- line_2
#         \           |
#          \          |
#           \         |
#            \        |
#            p3       p4   <-- line_1

class AnalysisBuoy:

    def __init__( self , child_frame_id = "base_buoy" );
        rospy.loginfo( "Waiting service of /vision/buoy" )
        rospy.wait_for_service( "/vision/buoy" )
        self.call_vision_data = rospy.ServiceProxy( "/vision/buoy" , VisionBuoy )

        # size is area of picture
        # error_x : is distance estimate from size
        # ratio : value of centimeter per pixels of picture and coordinate optical frame

        detail_line_1 = {
            'size' : 60
            , 'error_x' : 3
            , 'ratio_x' : 0.2
            , 'ratio_y' : 0.1 
        }

        detail_line_2 = {
            'size' : 20
            , 'error_x' : 7
            , 'ratio_x' : 0.6
            , 'ratio_y' : 0.3
        }

        analysis_parameter = {
            'x' : { 'ratio' : ( ( detail_line_2['error_x'] - detail_line_1['error_x'] ) 
                                / ( detail_line_2['size'] - detail_line_1['size'] ) ) # cm / size
                    ,'offset': detail_line_1['error_x'] }
            ,'y': { 'ratio' : ( -1 * ( detail_line_2['ratio_x'] - detail_line_1['ratio_x'] ) 
                                / ( detail_line_2['size'] - detail_line_1['size'] ) ) # cm / size
                    ,'offset' : 0 }
            ,'z': { 'ratio' : ( ( detail_line_2['ratio_y'] - detail_line_1['ratio_y'] ) 
                                / ( detail_line_2['size'] - detail_line_1['size'] ) ) # cm / size
                    ,'offset' : 0 }
        }

        # Part of parameter from vision
        self.found = 0
        self.score = 0
        self.center_x = 0
        self.center_y = 0
        self.area = 0

        self.broadcaster = Broadcaster( "front_camera_optical" , child_frame_id )

        self.analysis = {
            'found' : 0
            ,'x' : 0
            ,'y' : 0
            ,'z' : 0
        }

    def call_data( self ):
        # This variable use to return result from call_data
        result = False

        try:
            raw_data = self.call_vision_data( String('buoy') , String("") )
            result = True
        except rospy.ServiceException , e :
            rospy.logfatal( "Service call buoy failed : %s" , e )

        if( result ):
            self.found = raw_data.found
            self.center_x = raw_data.cx * 100
            self.center_y = raw_data.cy * 100
            self.score = raw_data.score
            self.area = raw_data.area
            self.analysis_picture()

        return result

    def echo_data( self ):
        print( "status of found : {:2d}".format( self.found ) )
        if( self.found == 0 ):
            print( "RAW_DATA : {:6.3f} {:6.3f} {:6.3f} {6.3f}".format( self.center_x 
                , self.center_y 
                , self.score 
                , self.area ) )
            print( "ANALYSIS : {:6.3f} {:6.3f} {:6.3f}".format( self.analysis['x'] 
                , self.analysis['y']
                , self.analysis['z'] ) )

    def analysis_picture( self ):

        if( self.found != 0 ):
            self.analysis[ 'found' ] = 0
            self.analysis[ 'x' ] = 0
            self.analysis[ 'y' ] = 0
            self.analysis[ 'z' ] = 0
        else:
            self.analysis[ 'found' ] = 1
            for run_axis in [ 'x' , 'y' , 'z' ]:
                self.analysis[ run_axis ] = ( ( self.area 
                        * self.analysis_parameter[ run_axis ][ 'ratio' ] )
                    + self.analysis_parameter[ run_axis ][ 'offset' ] )
            # Use in tf broadcaster
            self.broadcaster.quaternion( ( -1 * self.analysis['y'] 
                    , self.analysis['z'] 
                    , -1.0 * self.analysis['x'] ) 
                , ( 0 , 0 , 0 , 1 ) )
