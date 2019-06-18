#!/usr/bin/env python2
# FILE			: analysis_path.py
# AUTHOR		: K.Supasan
# CREATE ON		: 2019, June 18 (UTC+0)
# MAINTAINER	: K.Supasan

# README

# REFERENCE

from __future__ import print_function

from zeabus_utility.srv import VisionSrvPath

import math
# math.atan2( y , x ) will use find radian for y/x

class AnalysisPath:

    def __init__( self ):

        rospy.loginfo( "Waiting service of /vision/path" )
        rospy.wait_for_service( "/vision/path" )
        self.call_vision_data = rospy.ServiceProxy( "/vision/path" , VisionSrvPath )

        # Part of Calculate when get data
        self.num_point = 0 # collect number point in picture
        self.x_point = ( 0 , 0 , 0 ) # use to collect point x
        self.y_point = ( 0 , 0 , 0 ) # use to collect point y
        self.rotation = ( 0 , 0 ) # use to collect angle will linear point x and y from y axis

        # Part for filter data of path

    def call_data( self ):
        # call service of vision
        temp_data = self.call_vision_data( "task" , "request" )
        self.x_point = ( temp_data.data.point_1[0] 
            , temp_data.data.point_2[0] 
            , temp_data.data.point_3[0] )
        self.y_point = ( temp_data.data.point_1[1] 
            , temp_data.data.point_2[1] 
            , temp_data.data.point_3[1] )
        self.rotation = ( 
            math.atan2(self.x_point[1] - self.x_point[0], self.y_point[1] - self.y_point[0] )
            , math.atan2(self.x_point[2] - self.y_point[1], self.y_point[2] - self.y_point[1] ) )
        self.num_point = temp_data.data.n_point

    def echo_data( self ):
        print( "Point is ({:4.2f},{:4.2f}) : ({:4.2f},{:4.2f}) : ({:4.2f}, {:4.2f}) : ".format(
            self.x_point[0] , self.y_point[0]
            , self.x_point[1] , self.y_point[1] 
            , self.x_point[2] , self.y_point[2] ) )
        print( "Numpoint is {:4d}".format( self.num_point ) )
        print( "Rotation value {:6.3f} , {6.3f}".format( self.rotation[0] , self.rotation[1] ) )


if __name__=="__main__":
    rospy.init_node( "analysis_path" )

    analysis_path = AnalysisPath()

    while( not rospy.is_shutdown() ):
        data = int( input("Enter command 0 <stop> other get data : " ) )
        if( data == 0 ):
            rospy.shutdown() 
        else:
            analysis_path.call_data()
            analysis_path.echo_data()
