#!/usr/bin/env python2
# FILE			: analysis_handle.py
# AUTHOR		: K.Supasan
# CREATE ON		: 2019, July 20 (UTC+0)
# MAINTAINER	: K.Supasan

# README

# REFERENCE

from analysis_constant import *
import numpy as np
import math

def bin_w( data ):
    return data * BOTTOM_WIDTH / 2.0 

def bin_h( data ):
    return data * BOTTOM_HEIGHT / 2.0

def fin_w( data ):
    return data * FRONT_WIDTH / 2.0

def fin_h( data ):
    return data * FRONT_HEIGHT / 2.0

# Available for 3 point only and mode is True is high and False is Low
def bottom_find_radian( data , mode = True ): # Bottom find radian
    first_distance = ( bin_w( data[1][0] - data[0][0] ) , bin_h( data[1][1] - data[0][1]) )
    second_distance = ( bin_w( data[2][0] - data[0][0] ) , bin_h( data[2][1] - data[0][1]) )
    result = 0 
    if np.linalg.norm( first_distance ) > np.linalg.norm( second_distance ) :
        if mode :
            result = math.atan2( first_distance[1] , first_distance[0] )
        else:
            result = math.atan2( second_distance[1] , second_distance[0] )
    else:
        if mode :
            result = math.atan2( second_distance[1] , second_distance[0] )
        else:
            result = math.atan2( first_distance[1] , first_distance[0] )
    return result

def front_find_radian( data , mode = True ): # Front find radian
    first_distance = ( fin_w( data[1][0] - data[0][0] ) , fin_h( data[1][1] - data[0][1]) )
    second_distance = ( fin_w( data[2][0] - data[0][0] ) , fin_h( data[2][1] - data[0][1]) )
    result = 0 
    if np.linalg.norm( first_distance ) > np.linalg.norm( second_distance ) :
        if mode :
            result = math.atan2( first_distance[1] , first_distance[0] )
        else:
            result = math.atan2( second_distance[1] , second_distance[0] )
    else:
        if mode :
            result = math.atan2( second_distance[1] , second_distance[0] )
        else:
            result = math.atan2( first_distance[1] , first_distance[0] )
    return result

def bottom_distance( first , second , divide=False):
    answer = 0.
    if divide:
        answer = np.linalg.norm( 
            ( bin_w( first[0] - second[0] ) / 100 , bin_h( first[1] - second[1] ) / 100 ) )
    else:
        answer = np.linalg.norm( ( bin_w( first[0] - second[0] ) , bin_h( first[1] - second[1] ) ) )
    return answer
