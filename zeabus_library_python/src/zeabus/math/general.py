#!/usr/bin/env python2
# FILE			: general.py
# AUTHOR		: K.Supasan
# CREATE ON		: 2019, June 15 (UTC+0)
# MAINTAINER	: K.Supasan

# README

# REFERENCE

import math

    # Function for bound range of radian in range -pi to pi
def bound_radian( problem ):
    while( problem < -1.0*math.pi or problem > math.pi ):
        if( problem < 0 ):
            problem += (2.0*math.pi)
        else:
            problem += (-2.0*math.pi)
    return problem
