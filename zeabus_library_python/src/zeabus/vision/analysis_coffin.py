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

from zeabus_utility.srv import VisionSrvCoffin

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
                'type' : False 
                ,'center_x' : 0.0 
                ,'center_y' : 0.0
                ,'area' : 0.0
                ,'rotation' : 0.0 # Avaliable when type is true
                }
            ,'object_2' : {
                'type' : False
                ,'center_x' : 0.0
                ,'center_y' : 0.0
                ,'area' : 0.0
                ,'rotation' : 0.0 # Avaliable when type is true
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
            self.result['num_object'] = raw_data.state
            if( raw_data.state > 0 ):


                if( raw_data.data[0].state == 1 ):
                    self.result['object_1']['center_x'] = sum( raw_data.data[0].point_1[0] 
                        , raw_data.data[0].point_2[0] , raw_data.data[0].point_3[0]
                        , raw_data.data[0].point_4[0] ) / 4
                    self.result['object_1']['center_y'] = sum( raw_data.data[0].point_1[1] 
                        , raw_data.data[0].point_2[1] , raw_data.data[0].point_3[1]
                        , raw_data.data[0].point_4[1] ) / 4
                    self.result['object_1']['type'] = True 
                else:
                    self.result['object_1']['center_x'] = raw_data.daa[0].point_1[0]
                    self.result['object_1']['center_y'] = raw_data.daa[0].point_1[1]
                    self.result['object_1']['type'] = False 

                if( raw_data.data[1].state == 1 ):
                    self.result['object_2']['center_x'] = sum( raw_data.data[1].point_1[0] 
                        , raw_data.data[1].point_2[0] , raw_data.data[1].point_3[0]
                        , raw_data.data[1].point_4[0] ) / 4
                    self.result['object_2']['center_y'] = sum( raw_data.data[1].point_1[1] 
                        , raw_data.data[1].point_2[1] , raw_data.data[1].point_3[1]
                        , raw_data.data[1].point_4[1] ) / 4
                    self.result['object_2']['type'] = True 
                elif( raw_data.data[1].state == 2 ):
                    self.result['object_2']['center_x'] = raw_data.daa[0].point_1[0]
                    self.result['object_2']['center_y'] = raw_data.daa[0].point_1[1]
                    self.result['object_2']['type'] = False 
                else:
                    self.result['object_2']['center_x'] = 0
                    self.result['object_2']['center_y'] = 0
                    self.result['object_2']['type'] = False 

                self.analysis_data()    
            else:
                pass
        else:
            self.result['num_object'] = 0

        return result

    def echo_data( self ):
        if( self.result['found'] ):
            pass
        else:
            print( "Don't found target")

    def analysis_data( self ):
        if( self.result['num_object'] == 2 ):
            self.result['']
