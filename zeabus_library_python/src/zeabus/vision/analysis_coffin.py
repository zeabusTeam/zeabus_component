#!/usr/bin/env python2
# FILE			: analysis_coffin.py
# AUTHOR		: K.Supasan
# CREATE ON		: 2019, July 14 (UTC+0)
# MAINTAINER	: K.Supasan

# README

# REFERENCE
#   ref01   : https://docs.scipy.org/doc/numpy/reference/generated/numpy.linalg.norm.html

from __future__ import print_function

import math
import rospy
import numpy 

from ..transformation.broadcaster import Broadcaster

from .analysis_constant import *

from std_msgs.msg import String

from zeabus_utility.srv import VisionSrvCoffin

class AnalysisCoffin:

    def __init__( self, child_frame_id = "base_coffin" ):

        rospy.loginfo( "Waiting service of /vision/coffin" )
        rospy.wait_for_service( "/vision/coffin" )
        self.call_vision_data = rospy.ServiceProxy( "/vision/coffin" , VisionSrvCoffin )

        self.result = {
            'num_object' : 0
            ,'center_x' : 0.0 # Avaliable when you found num_object == 2
            ,'center_y' : 0.0 # Avaliable when you found num_object == 2
            ,'object_1' : { 
                'center_x' : 0.0 
                ,'center_y' : 0.0
                ,'area' : 0.0
                ,'rotation' : 0.0 # Avaliable when type is true
                }
            ,'object_2' : {
                'center_x' : 0.0
                ,'center_y' : 0.0
                ,'area' : 0.0
                ,'rotation' : 0.0 # Avaliable when type is true
            }
        }

        self.broadcaster = Broadcaster( "bottom_camera_optical" , child_frame_id )

    def call_data( self ):
        result = False

        try:
            raw_data = self.call_vision_data( String("exposed") , String("coffin") )
            result = True
        except rospy.ServiceException , e :
            rospy.logfatal( "Service call gate failed : %s" , e )

        if( result ):
            self.result['num_object'] = raw_data.state
            if( raw_data.state > 0 ):
                if( raw_data.data[0].state == 1 ):
                    self.result['object_1']['center_x'] = sum( ( raw_data.data[0].point_1[0] 
                        , raw_data.data[0].point_2[0] , raw_data.data[0].point_3[0]
                        , raw_data.data[0].point_4[0] ) ) * 100 / 4
                    self.result['object_1']['center_y'] = sum( ( raw_data.data[0].point_1[1] 
                        , raw_data.data[0].point_2[1] , raw_data.data[0].point_3[1]
                        , raw_data.data[0].point_4[1] ) ) * 100 / 4
                    # calculate rotation
                    first_pair = ( raw_data.data[0].point_1 , raw_data.data[0].point_2 )
                    second_pair = ( raw_data.data[0].point_1 , raw_data.data[0].point_4 )
                    first_distance = ( first_pair[1][0] - first_pair[0][0] 
                        , first_pair[1][1] - first_pair[0][1] )
                    second_distance = ( second_pair[1][0] - second_pair[0][0] 
                        , second_pair[1][1] - second_pair[0][1] )
                    if numpy.linalg.norm(first_distance) > numpy.linalg.norm(second_distance):
                        self.result['object_1']['rotation'] = math.atan2( first_distance[1] 
                            , first_distance[0] )
                    else:
                        self.result['object_1']['rotation'] = math.atan2( second_distance[1] 
                            , second_distance[0] )
                        
                if( raw_data.data[1].state == 1 ):
                    self.result['object_2']['center_x'] = sum( raw_data.data[1].point_1[0] 
                        , raw_data.data[1].point_2[0] , raw_data.data[1].point_3[0]
                        , raw_data.data[1].point_4[0] ) * 100 / 4 
                    self.result['object_2']['center_y'] = sum( raw_data.data[1].point_1[1] 
                        , raw_data.data[1].point_2[1] , raw_data.data[1].point_3[1]
                        , raw_data.data[1].point_4[1] ) * 100 / 4
                    # calculate rotation
                    first_pair = ( raw_data.data[1].point_1 , raw_data.data[1].point_2 )
                    second_pair = ( raw_data.data[1].point_1 , raw_data.data[1].point_4 )
                    first_distance = ( first_pair[1][0] - first_pair[0][0] 
                        , first_pair[1][1] - first_pair[0][1] )
                    second_distance = ( second_pair[1][0] - second_pair[0][0] 
                        , second_pair[1][1] - second_pair[0][1] )
                    if numpy.linalg.norm(first_distance) > numpy.linalg.norm(second_distance):
                        self.result['object_2']['rotation'] = math.atan2( first_distance[1] 
                            , first_distance[0] )
                    else:
                        self.result['object_2']['rotation'] = math.atan2( second_distance[1] 
                            , second_distance[0] )

                self.analysis_data()    
        else:
            self.result['num_object'] = 0

        return result

    def echo_data( self ):
        if( self.result['num_object'] == 2 ):
            print( "number of object is 2 and center " + repr( (
                self.result['center_x'] , self.result['center_y' ] ) ) )
            print( "center of each object ({:6.3f},{:6.3f}) : ({:6.3f},{:6.3f})".format(
                self.result['object_1']['center_x'] , self.result['object_1']['center_y']
                , self.result['object_2']['center_x'] , self.result['object_2']['center_y'] ) )
        elif self.result['num_object'] == 1 :
            print( "Found only one object center is " + repr( (
                self.result['center_x'] , self.result['center_y'] ) ) + " and rotation is " 
                + str( self.result['object_1']['rotation'] ) )
        else:
            print( "Don't found target")

    def analysis_data( self ):
        if( self.result['num_object'] == 2 ):
            self.result['center_x'] = ( self.result['object_1']['center_x'] 
                + self.result['object_2']['center_x'] ) / 2.0
            self.result['center_y'] = ( self.result['object_1']['center_y'] 
                + self.result['object_2']['center_y'] ) / 2.0
        elif( self.result['num_object'] == 1 ):
            self.result['center_x'] = self.result['object_1']['center_x']
            self.result['center_y'] = self.result['object_1']['center_y']
        else:
            pass 
