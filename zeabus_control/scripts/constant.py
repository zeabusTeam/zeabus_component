#!/usr/bin/env python
# FILE			: constant.py
# AUTHOR		: K.Supasan
# CREATE ON		: 2019, May 26 (UTC+0)
# MAINTAINER	: K.Supasan

# README

# REFERENCE
#   ref01   : https://www.programiz.com/python-programming/variables-constants-literals#variable
#   ref02   : https://www.python.org/dev/peps/pep-0008/#constants

# part of file thruster_mapper_enu
# Below constant will about data when updated time
THRUSTER_MAPPER_UPDATED = True
# Below constant will about show all calculate process
THRUSTER_MAPPER_CALCULATE_PROCESS = True
# Below constant will show you a result
THRUSTER_MAPPER_RESULT = True
# Below constant will help you to see RPY instead quaternion
THRUSTER_MAPPER_EULER = True
# Below constant will know about can call service to auv_state or not
THRUSTER_MAPPER_AUV_STATE = True

if(THRUSTER_MAPPER_CALCULATE_PROCESS):
    THRUSTER_MAPPER_RESULT = True
# In the result is one part of PROCESS CALCULATE
