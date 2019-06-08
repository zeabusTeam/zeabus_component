#!/usr/bin/env python
"""
FILE			: constant.py
AUTHOR		    : K.Supasan
CREATE ON		: 2019, May 26 (UTC+0)
MAINTAINER	    : K.Supasan

README

REFERENCE
  ref01   : https://www.programiz.com/python-programming/variables-constants-literals#variable
  ref02   : https://www.python.org/dev/peps/pep-0008/#constants
"""

# part of file thruster_mapper_enu
# Below constant will about show process of data
THRUSTER_MAPPER_CHOOSE_PROCESS = False
# Below constant will about show all calculate process
THRUSTER_MAPPER_CALCULATE_PROCESS = False
# Below constant will show you a result
THRUSTER_MAPPER_RESULT = True
# Below constant will show about callback have been called
THRUSTER_MAPPER_CALLBACK_CALLED = False
# Below constant will consider about time out of data
THRUSTER_MAPPER_TIME_OUT = 1.0 
# Below constant will help you to send command don't worry about time stamp
THRUSTER_MAPPER_AUTO_TIME = False
# Below constant will help you to manage about loop to print value result
THRUSTER_MAPPER_COUNT = 5

if(THRUSTER_MAPPER_CALCULATE_PROCESS):
    THRUSTER_MAPPER_RESULT = True
# In the result is one part of PROCESS CALCULATE


# Below constant help to manage about tuple
FALSE_MASK = ( False , False , False , False , False , False )
