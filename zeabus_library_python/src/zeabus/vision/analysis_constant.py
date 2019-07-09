#!/usr/bin/env python2
# FILE			: analysis_constant.py
# AUTHOR		: K.Supasan
# CREATE ON		: 2019, July 02 (UTC+0)
# MAINTAINER	: K.Supasan

# README

# REFERENCE

# Constant for filter parameter of buoy mission
BUOY_SCORE = 50 # Range is 0 - 100
BUOY_AREA = 0.0001 # Range is 0 - 1 but this mission have area lowest
BUOY_MINIMUM_FOUND = 2

# Constant for estimate gate value
GATE_NEAR = 2.0 # Distannce between robot and lenth when length is max ( 200 )
GATE_RATIO = 2.0 / 100 # Ratio of distance per legth
GATE_LENGTH = 200.0 # Ditance of x length range 0 to 200
