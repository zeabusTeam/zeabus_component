#!/usr/bin/env python2
# FILE			: analysis_path.py
# AUTHOR		: K.Supasan
# CREATE ON		: 2019, June 18 (UTC+0)
# MAINTAINER	: K.Supasan

# README

# REFERENCE

from __future__ import print_function

import math
import rospy
# math.atan2( y , x ) will use find radian for y/x

from ..transformation.broadcaster import Broadcaster

from std_msgs.msg import String

from zeabus_utility.srv import VisionSrvPath

class AnalysisPath:

    def __init__( self , child_frame_id = "base_path" ):

        rospy.loginfo( "Waiting service of /vision/path" )
        rospy.wait_for_service( "/vision/path" )
        self.call_vision_data = rospy.ServiceProxy( "/vision/path" , VisionSrvPath )

        # Part of Calculate when get data
        self.num_point = 0 # collect number point in picture
        self.x_point = ( 0 , 0 , 0 ) # use to collect point x
        self.y_point = ( 0 , 0 , 0 ) # use to collect point y
        self.rotation = ( 0 , 0 ) # use to collect angle will linear point x and y from y axis
        self.area = (0 , 0)

        self.broadcaster = Broadcaster( "bottom_camera_optical" , child_frame_id )

        # Part for filter data of path

    def call_data( self ):
        # call service of vision
        result = False

        try:
            temp_data = self.call_vision_data(String('path') , String('request' ))
            result = True
        except rospy.ServiceException , e :
            rospy.logfatal( "Sevice call vision path Failed : %s" , e )

        if( result ):
            self.x_point = ( temp_data.data.point_1[0] 
                , temp_data.data.point_2[0] 
                , temp_data.data.point_3[0] )
            self.y_point = ( temp_data.data.point_1[1] 
                , temp_data.data.point_2[1] 
                , temp_data.data.point_3[1] )
            self.rotation = ( 
                math.atan2(
                    bin_h( self.y_point[1] - self.y_point[0] )
                    , bin_w( self.x_point[1] - self.x_point[0] ) )
                , math.atan2(
                    bin_h( self.y_point[2] - self.y_point[1] )
                    , bin_w( self.x_point[2] - self.x_point[1] ) ) )

            self.area = ( temp_data.data.area[0] , temp_data.data.area[1] )

            self.num_point = temp_data.data.n_point

            temp_x = 0
            temp_y = 0

            # Now we can't estimate z but  I think we can estimate xy
            # I gave ratio y is 100 : 0.2 and x is 100 : 0.35
            if( self.num_point == 2 ):
                print("=======================broadcaster point 2")
                temp_y = 0.2 * self.y_point[0] / 100
                temp_x = 0.35 * self.x_point[0] / 100
                self.broadcaster.euler( ( temp_x , temp_y , -2 ) , 0 , 0 , self.rotation[0] )
            elif( self.num_point == 3 ):
                print("=======================broadcaster point 3")
                temp_y = ( 0.2 * self.y_point[0] / 100 ) + (-0.4 * math.sin( self.rotation[0] ))
                temp_x = ( 0.35 * self.x_point[0] / 100 ) + (-0.4 * math.cos( self.rotation[0] ))
                self.broadcaster.euler( ( temp_x , temp_y , -2 ) , 0 , 0 , self.rotation[0] )
            else:
                None
                

        return result

    def echo_data( self ):
        print( "Numpoint is {:4d}".format( self.num_point ) )
        if( self.num_point != 0 ):
            print("Point is ({:6.2f},{:6.2f}) : ({:6.2f},{:6.2f}) : ({:6.2f},{:6.2f}) : ".format(
                self.x_point[0] , self.y_point[0]
                , self.x_point[1] , self.y_point[1] 
                , self.x_point[2] , self.y_point[2] ) )
            print( "Rotation value {:6.3f} , {:6.3f}".format( self.rotation[0] 
                , self.rotation[1] ) )
            print( "Area is {:6.2f} : {:6.2f}".format( self.area[0] 
                , self.area[1] ) )


if __name__=="__main__":
    rospy.init_node( "analysis_path" )

    analysis_path = AnalysisPath()

    rate = rospy.Rate( 10 )

    while( not rospy.is_shutdown() ):
        if( analysis_path.call_data() ):
            print("================================SUCCESS===================================")
            analysis_path.echo_data()
        else:
            print("=============================FALIURE & WAIT==============================")
            rospy.wait_for_service( "/vision/path" )
