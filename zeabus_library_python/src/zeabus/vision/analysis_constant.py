#!/usr/bin/env python2
# FILE			: analysis_constant.py
# AUTHOR		: K.Supasan
# CREATE ON		: 2019, July 02 (UTC+0)
# MAINTAINER	: K.Supasan

# README

# REFERENCE

# Constant for convert norm value to orinal value
FRONT_WIDTH = 581.0
FRONT_HEIGHT = 365.0
BOTTOM_WIDTH = 581.0
BOTTOM_HEIGHT = 365.0

# Constant for filter parameter of buoy mission
BUOY_SCORE = 10 # Range is 0 - 100
BUOY_AREA = 0.0001 # Range is 0 - 1 but this mission have area lowest
BUOY_MINIMUM_FOUND = 2

# Constant for estimate gate value
GATE_NEAR = 2.0 # Distannce between robot and lenth when length is max ( 200 )
GATE_RATIO = 2.0 / 100 # Ratio of distance per legth
GATE_LENGTH = 200.0 # Ditance of x length range 0 to 200

# Constant for vision of drop gralic mission
DROP_FIND_TARGET = "search"
DROP_FIND_DROP = "drop"
DROP_FIND_OPEN = "open"

STAKE_FIND_TARGET = "vampire"
STAKE_FIND_HEART = "heart"
STAKE_FIND_RIGHT = "right"
STAKE_FIND_LEFT = "left"
STAKE_RATIO_RADIAN = 0.02 
STAKE_MINIMUM_FOUND = 2
