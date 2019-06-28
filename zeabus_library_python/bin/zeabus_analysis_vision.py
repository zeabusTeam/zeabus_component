#!/usr/bin/env python2
# FILE			: zeabus_analysis_vision.py
# AUTHOR		: K.Supasan
# CREATE ON		: 2019, June 28 (UTC+0)
# MAINTAINER	: K.Supasan

# README

# REFERENCE

import rospy
from zeabus.vision.analysis_path import AnalysisPath

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
