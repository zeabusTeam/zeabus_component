#!/usr/bin/env python2
# FILE			: analysis_drop.py
# AUTHOR		: K.Supasan
# CREATE ON		: 2019, July 13 (UTC+0)
# MAINTAINER	: K.Supasan

# README
# Data from vision have 3 case can consider on state variable
#   state == 0
#       Vision don't found target in picture
#   state == 1
#       Vision can recognize object to around 4 point don't have target in out of frame
#   state == 2
#       Vision can recognize 4 point around object because have target out of frame
#   In case vision can recognise rectangle to over target will use 4 point for you to send 
#       in order bl , br , tr , tl (b = Bottom , t = Top , l = Left , r = Right )
#   In case vision can't rectangle will send center toin varaible name point_01
#   range is -100 to 100

# REFERENCE

from __future__ import print_function

import math
import rospy

from ..transformation.broadcaster import Broadcaster

from .analysis_constant import *

from std_msgs.msg import String

from zeabus_utility.srv import VisionSrvDropGarlic

from zeabus_utility.msg import VisionBox

class AnalysisDrop:

    def __init__( self, child_frame_id = "base_drop" , default_request = DROP_FIND_TARGET ):

        rospy.loginfo( "Waiting service of /vision/drop_garlic" )
        rospy.wait_for_service( "/vision/drop_garlic" )
        self.call_vision_data = rospy.ServiceProxy( "/vision/drop_garlic" , VisionSrvDropGarlic )

        self.result = {
            'found' : False
            , 'type' : False # True is 4 point or state == 1 and false is state == 2
            , 'center_x' : 0.0
            , 'center_y' : 0.0
            , 'area' : 0.0
            , 'rotation' : 0.0 # This variable can use only in case vision state == 1
            , 'bottom_y' : 0.0 # This variable can use only in case vision state == 1
        }

        self.default_request = default_request

    def echo_data( self ):
        if( self.result['found'] ):
            if( self.result['type'] ):
                print("ANALYSIS_DROP see 4 point")
                print("____center point : ({:6.3f} , {:6.3f})".format( self.result['center_x'] 
                    , self.result['center_y'] ) )
                print("____( area , bottom , rotation ) : ({:6.2f} , {:6.2f} , {:6.3f})".format(
                    self.result['area'] , self.result['bottom_y'] , self.result['rotation'] ) )
            else:
                print("ANALYSIS_DROP only see center")
                print("____center point : ({:6.3f} , {:6.3f})".format( self.result['center_x'] 
                    , self.result['center_y'] ) )
                print("____area is " + str( self.result['area'] ) )
        else:
            print("ANALYSIS_DROP don't found target") 

    def call_data( self , request = None ):

        result = False

        if( request == None ):
            request = self.default_request

        try:
            raw_data = self.call_vision_data( String("drop_garlic") , String( request ) ).data
            result = True
        except rospy.ServiceException , e :
            rospy.logfatal( "Service call vision drop garlic failure : %s" , e )

        # For use less memory this method will calculate in this function
        if( result ):
            if( raw_data.state == 1 ):
                self.result['found'] = True
                self.result['type'] = True
                self.result['center_x'] = sum( ( raw_data.point_1[0] , raw_data.point_2[0] 
                    , raw_data.point_3[0] , raw_data.point_4[0] ) ) / 4
                self.result['center_y'] = sum( ( raw_data.point_1[1] , raw_data.point_2[1] 
                    , raw_data.point_3[1] , raw_data.point_4[1] ) ) / 4
                self.result['area'] = raw_data.area
                self.result['bottom_y'] = ( raw_data.point_1[ 1 ] + raw_data.point_2[1] ) / 2
                self.result['rotation'] = math.atan2( raw_data.point_2[1] - raw_data.point_1[1] 
                    , raw_data.point_2[0] - raw_data.point_1[0] )
            elif( raw_data.state == 2 ):
                self.result['found'] = True
                self.result['type'] = False
                self.result['center_x'] = raw_data.point_1[0]
                self.result['center_y'] = raw_data.point_1[1]
            else:
                self.result['found'] = False
        else:
            self.result['found'] = False

        return result
