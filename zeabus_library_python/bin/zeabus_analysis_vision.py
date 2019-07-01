#!/usr/bin/env python2
# FILE			: zeabus_analysis_vision.py
# AUTHOR		: K.Supasan
# CREATE ON		: 2019, June 28 (UTC+0)
# MAINTAINER	: K.Supasan

# README

# REFERENCE

import rospy
from zeabus.vision.analysis_path import AnalysisPath
from zeabus.vision.analysis_buoy import AnalysisBuoy

if __name__=="__main__":

    rospy.init_node( "zeabus_library_analysis" )

    mission_list = [ "path" , "buoy"]

    analysis_mode = rospy.get_param( '~mission' , "path" )
    frequency = rospy.get_param( '~frequency' , 10 )

    if( analysis_mode == "path" ):
            print( "You choose analysis path")
            analysis_vision = AnalysisPath( "base_path" )
    elif( analysis_mode == "buoy" ):
            print( "You choose analysis buoy")
            analysis_vision = AnalysisBuoy( "base_buoy" )
    else:
        print( "Don't have your mode plesase from them\n" , mission_list )
        rospy.signal_shutdown( "Don't have mode " + str( analysis_mode ) )

    rate = rospy.Rate( frequency )

    while( not rospy.is_shutdown() ):
        if( analysis_vision.call_data() ):
            print("================================SUCCESS===================================")
            analysis_vision.echo_data()
        else:
            print("=============================FALIURE & WAIT==============================")
            rospy.wait_for_service( "/vision/" + str( analysis_mode) )
