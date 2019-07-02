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

    def __init__( self , child_frame_id = "base_buoy" ):
        rospy.loginfo( "Waiting service of /vision/buoy" )
        rospy.wait_for_service( "/vision/buoy" )
        self.call_vision_data = rospy.ServiceProxy( "/vision/buoy" , VisionBuoy )

        # size is area of picture
        # error_x : is distance estimate from size
        # ratio : value of centimeter per pixels of picture and coordinate optical frame

        detail_line_1 = {
            'size' : 10
            , 'error_x' : 300.0 # cm
            , 'ratio_x' : 60.0 # cm / pixels <percent>
            , 'ratio_y' : 20.0 # cm / pixels <percent>
        }

        detail_line_2 = {
            'size' : 2
            , 'error_x' : 520.0 # cm
            , 'ratio_x' : 200.0 # cm / pixels <percent>
            , 'ratio_y' : 60.0 # cm / pixels <percent>
        }

        self.rotation = Quaternion()
        self.rotation.set_euler( -math.pi/2 , 0 , math.pi/2 )
        self.optical = [0 , 0 , 0] # Data in optical frame
        
        self.analysis = { # Data in robot frame
            'found' : False
            ,'x' : 0
            ,'y' : 0
            ,'z' : 0
        }


        # for y and z unit of ratio is cm per ( pixels * size )
        self.analysis_parameter = {
            'z' : { 'ratio' : ( ( detail_line_2['error_x'] - detail_line_1['error_x'] ) 
                                / ( detail_line_2['size'] - detail_line_1['size'] ) ) # cm / size
                    ,'offset' : detail_line_1['error_x'] }
            ,'y': { 'ratio' : ( ( detail_line_2['ratio_y'] - detail_line_1['ratio_y'] ) 
                                / ( detail_line_2['size'] - detail_line_1['size'] ) ) 
                    ,'offset' : 0 }
            ,'x': { 'ratio' : ( ( detail_line_2['ratio_x'] - detail_line_1['ratio_x'] ) 
                                / ( detail_line_2['size'] - detail_line_1['size'] ) ) 
                    ,'offset' : 0 }
        }

        self.analysis_parameter[ 'z' ][ 'offset' ] -= (
                self.analysis_parameter[ 'z' ][ 'ratio' ] 
                * self.analysis_parameter[ 'z' ][ 'offset' ] )

        # Part of parameter from vision
        self.found = False
        self.score = 0
        self.center_x = 0
        self.center_y = 0
        self.area = 0

        self.broadcaster = Broadcaster( "front_camera_optical" , child_frame_id )

    def call_data( self ):
        # This variable use to return result from call_data
        result = False

        try:
            raw_data = self.call_vision_data( String('buoy') , String("") )
            result = True
        except rospy.ServiceException , e :
            rospy.logfatal( "Service call buoy failed : %s" , e )

        if( result ):
            if( raw_data.found != 1 ):
                self.found = False
            else:
                self.found = True
            self.center_x = raw_data.cx * 100
            self.center_y = raw_data.cy * 100
            self.score = raw_data.score * 100
            self.area = raw_data.area * 100
            self.analysis_picture()

        return result

    def echo_data( self ):
        print( "status of found : {:2d}".format( self.found ) )
        if( self.found ):
            print( "RAW_DATA : {:6.3f} {:6.3f} {:6.3f} {:6.3f}".format( self.center_x 
                , self.center_y 
                , self.score 
                , self.area ) )
            print( "OPTICAL  : {:6.3f} {:6.3f} {:6.3f}".format( self.optical[0] 
                , self.optical[1]
                , self.optical[2] ) )
            print( "ROBOT    : {:6.3f} {:6.3f} {:6.3f}".format( self.analysis['x'] 
                , self.analysis['y']
                , self.analysis['z'] ) )

    def analysis_picture( self ):

        if( ( not self.found ) or ( self.area < 0.001 ) ):
            self.analysis[ 'found' ] = False
            self.analysis[ 'x' ] = 0
            self.analysis[ 'y' ] = 0
            self.analysis[ 'z' ] = 0
        else:
            self.analysis[ 'found' ] = True 

#            print( repr( self.analysis_parameter ) )

#            self.optical[ 0 ] = ( ( self.center_x * self.area 
#                    * self.analysis_parameter['x']['ratio'] )
#                + self.analysis_parameter['x']['offset'] )
#            self.optical[ 1 ] = ( ( self.center_y * self.area 
#                    * self.analysis_parameter['y']['ratio'] )
#                + self.analysis_parameter['y']['offset'] )
#            self.optical[ 2 ] = ( ( self.area * self.analysis_parameter['z']['ratio'] )
#                + self.analysis_parameter['z']['offset'] )

            self.optical[ 0 ] = self.center_x * 5
            self.optical[ 1 ] = self.center_y * 3
            self.optical[ 2 ] = ( self.area - 12 ) * 50

            ( self.analysis['x'] , self.analysis['y'] , self.analysis['z'] ) = ( 
                self.rotation.rotation( 
                    ( self.optical[0] , self.optical[1] , self.optical[2] , 0 ) ) ).vector[:3]
            # Use in tf broadcaster
            self.broadcaster.quaternion( tuple( self.optical ) , ( 0 , 0 , 0 , 1 ) )
