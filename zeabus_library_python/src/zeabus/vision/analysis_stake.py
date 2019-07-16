#!/usr/bin/env python2
# FILE			: analysis_stake.py
# AUTHOR		: K.Supasan
# CREATE ON		: 2019, July 15 (UTC+0)
# MAINTAINER	: K.Supasan

# README

# REFERENCE

from __future__ import print_function

import math
import rospy

from ..transformation.broadcaster import Broadcaster

from .analysis_constant import *

from std_msgs.msg import String

from zeabus_utility.srv import VisionSrvStake.srv

class AnalysisStake:

    def __init__( self , child_frame_id = "base_stake" , default = STAKE_FIND_TARGET ):

        rospy.loginfo( "Waiting service of /vision/stake" )
        rospy.wait_for_service( "/vision/stake" , VisionSrvStake )

        self.result = {
            'found' : 0
            , 'center' : [ 0.0 , 0.0 ] # range -100 to 100
            , 'top' : [ 0.0 ]
            , 'rotation' : 0.0
            , 'area' : 0.0 # range 0 to 100
            , 'distance' : 0 # distance is calculate estimate for length of x
        }

        self.broadcaster = Broadcaster( "front_camera_optical" , child_frame_id )
        self.rate = rospy.Rate( 10 )

        self.default = default

    def call_data( self , request = None ): 
        buffer_center_x : []
        buffer_center_y : []
        buffer_rotation : []
        buffer_area : []
        buffer_distance = []
        buffer_top_y = []

        if( request == None ):
            request = self.default

        self.result['found'] = False
        self.can_call = False
        count = 0 
        count_found = 0
        while( count < 3 ):
            self.rate.sleep()
            count += 1
            temp_result = self.individual_call( request )
            if( temp_result[0] ):
                count_found += 1
                buffer_center_x.append( temp_result[1][0] )
                buffer_center_y.append( temp_result[1][1] ) 
                buffer_rotation.append( temp_result[2] )
                buffer_area.append( temp_result[3] )
                buffer_distance.append( temp_result[4] )
                buffer_top_y.append( temp_result[5] )
                if( count_found == STAKE_MINIMUM_FOUND ):
                    self.result['found'] = True

        if( self.result['found'] ):
            self.result['center'][0] = sum( buffer_center_x ) / count_found
            self.result['center'][1] = sum( buffer_center_y ) / count_found
            self.result['rotation'] = sum( buffer_rotation ) / count_found
            self.result['area'] = sum( buffer_area ) / count_found
            self.result['distance'] = sum( buffer_distance ) / count_found
            self.result['top'] = sum( buffer_top_y ) / count_found

        return self.can_call

    def echo_data( self , request ):
        if( self.result['found'] ):
            print( "RESULT center : {:7.2f} {:7.2f}".format( self.result['center'][0] 
                , self.result['center'][1] ) )
            print("(rotation area distance top ) : ( {:6.3f} {:7.3f} {:6.3f} {:7.3f} )".format(
                self.result['rotation'] , self.result['area'] , self.result['distance'] 
                , self.result['top'] ) )
        else:
            print( "Don't found target")

    def individual_call( self , request ):

        found = False
        result = [ False ]
        try:
            raw_data = self.call_vision_data( String("stake") , String( request ) ).data
            if( raw_data.state == 1 ):
                found = True
            result[ 0 ] = True
        except rospy.ServiceException , e :
            rospy.logfatal( "Service call stake failed : %s" , e )

        if( call_success ):
            if( found ):
                if( request == STAKE_FIND_HEART ):
                    result.append( raw_data.point_1 )
                    result.append( 0.0 )
                    result.append( raw_data.area )
                    result.append( 0.0 )
                    result.append( 0.0 )
                else:
                    result.append( ( ( ( raw_data.point_1[ 0 ] * 100, raw_data.point_2[ 0 ] * 100
                            , raw_data.point_3[ 0 ] * 100 , raw_data.point_4[ 0 ] * 100 ) / 4)
                        , ( ( raw_data.point_1[ 1 ] * 100 , raw_data.point_2[ 1 ] * 100
                            , raw_data.point_3[ 1 ] * 100 , raw_data.point_4[ 1 ] * 100 ) / 4)))
                    result.append( ( ( raw_data.point_4[1] * 100 - raw_data.point_1[1] * 100 ) 
                            - ( raw_data.point_3[1] * 100 - raw_data.point_4 * 100 ) ) 
                        * STAKE_RATIO_RADIAN )
                    result.append( raw_data.area )
                    result.apeend( 0.0 )
                    result.append( max( ( raw_data.point_1[0] * 100 , raw_data.point_2[0] * 100
                        , raw_data.point_3[0] * 100 , raw_data.point_4[ 0] * 100 ) ) )

        return result
