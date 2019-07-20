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
from zeabus.vision.analysis_gate import AnalysisGate
from zeabus.vision.analysis_drop import AnalysisDrop
from zeabus.vision.analysis_stake import AnalysisStake
from zeabus.vision.analysis_coffin import AnalysisCoffin
from zeabus.vision.analysis_constant import *

if __name__=="__main__":

    rospy.init_node( "zeabus_library_analysis" )

    mission_list = ( "path" , "buoy" , "gate" , "drop_find" , "drop_drop" , "drop_open"
        , "stake_find" , "stake_heart" ,  "coffin")

    analysis_mode = rospy.get_param( '~mission' , "path" )
    frequency = rospy.get_param( '~frequency' , 10 )

    if( analysis_mode == "path" ):
        print( "You choose analysis path" )
        analysis_vision = AnalysisPath( "base_path" )
    elif( analysis_mode == "buoy" ):
        print( "You choose analysis buoy" )
        analysis_vision = AnalysisBuoy( "base_buoy" )
    elif( analysis_mode == "gate" ):
        print( "You choose analysis gate" )
        analysis_vision = AnalysisGate( "base_gate" )
    elif( analysis_mode == "drop_find" ):
        print( "You choose analysis drop find")
        analysis_vision = AnalysisDrop( "base_drop" , DROP_FIND_TARGET )
        analysis_mode = "drop_garlic"
    elif( analysis_mode == "drop_drop" ):
        print( "You choose analysis drop drop")
        analysis_vision = AnalysisDrop( "base_drop" , DROP_FIND_DROP )
        analysis_mode = "drop_garlic"
    elif( analysis_mode == "drop_open" ):
        print( "You choose analysis drop open")
        analysis_vision = AnalysisDrop( "base_drop" , DROP_FIND_OPEN )
        analysis_mode = "drop_garlic"
    elif( analysis_mode == "stake_find" ):
        print("You choose analysis stake find")
        analysis_vision = AnalysisStake( "base_drop" , STAKE_FIND_TARGET )
        analysis_mode = "stake"
    elif( analysis_mode == "stake_heart" ):
        print("You choose analysis stake heart")
        analysis_vision = AnalysisStake( "base_drop" , STAKE_FIND_HEART )
        analysis_mode = "heart"
    elif( analysis_mode == "stake_left" ):
        print("You choose analysis stake left")
        analysis_vision = AnalysisStake( "base_drop" , STAKE_FIND_LEFT )
        analysis_mode = "heart"
    elif( analysis_mode == "stake_right" ):
        print("You choose analysis stake right")
        analysis_vision = AnalysisStake( "base_drop" , STAKE_FIND_RIGHT )
        analysis_mode = "heart"
    elif( analysis_mode == "coffin" ):
        print("You choose analysis coffin" )
        analysis_vision = AnalysisCoffin( "base_coffin" )
    else:
        print( "Don't have your mode plesase from them\n" , mission_list )
        rospy.signal_shutdown( "Don't have mode " + str( analysis_mode ) )

    rate = rospy.Rate( frequency )

    while( not rospy.is_shutdown() ):
        if( analysis_vision.call_data() ):
            analysis_vision.echo_data()
            print("================================SUCCESS===================================")
        else:
            print("=============================FALIURE & WAIT==============================")
            rospy.wait_for_service( "/vision/" + str( analysis_mode) )
