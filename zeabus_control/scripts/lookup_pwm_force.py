#!/usr/bin/env python
# FILE			: lookup_pwm_force.py
# AUTHOR		: K.Supasan
# CREATE ON		: 2019, May 26 (UTC+0)
# MAINTAINER	: K.Supasan
# ORIGINAL FILE : can't remember for zeabus first generation

# README

# REFERENCE
#   ref1    : https://www.python.org/dev/peps/pep-0008/#tabs-or-spaces

import rospy
import bisect # use this to algorithm to search value
from read_file import read_file
# from filename (don't have type file ) import class? I think that

rospack = rospkg.RosPack()

class lookup_pwm_force:

    # 3 parameter will point to fild to collect data
    #   package_name we will use ros lib to get path to ros
    #   subdirectory will use append to package path
    #   and file_name will be name file to end of string full path
    def __init__( self ,  package_name , subdirectory , file_name ):
        fullpath = rospack.get_path( name_file ) + "/" + subdirectory + "/" + file_name

        # read file to make handle get data form file
        self.file = read_file( fullpath )

        self.pwm_table = self.file.get_column( 0 , ',' )
        self.force_table = self.file.get_column( 1 , ',' )

        self.length_pwm_table = len(self.pwm_table)
        self.length_force_table = len(self.force_table)

    def find_pwm( self , force ):
        # this sequence I will copy because I don't have time to study function bisect
        idx = bisect.bisect_left( self.force_table , fource )
        # idx is variable to point position of data in that array
        # if don't equal this will estimate between 2 point and point in between 2 data
        if( idx == 0 ):
            pwm = self.pwm_table[0]
        elif( idx == len(self.force_table) ):
            pwm = self.pwm_table[ self.length_force_table - 1 ]
        elif _float_equal( force , self.force_table[ idx ] ):
            pwm = self.pwm_table[ idx ]
        else:
            pwm = ( self.pwm_table[ idx-1 ] 
                    + ( ( self.pwm_table[ idx ] - self.pwm_table[ idx - 1 ] ) 
                        * ( force - self.force_table[ idx - 1 ] )
                        / ( self.force_table[ idx ] - self.force_table[ idx - 1] ) ) )
        return pwm

    def find_force( self , pwm ):
        # this sequence I will copy because I don't have time to study function bisect
        idx = bisect.bisect_left( self.force_table , fource )
        # idx is variable to point position of data in that array
        # if don't equal this will estimate between 2 point and point in between 2 data
        if( idx == 0 ):
            force = self.force_table[0]
        elif( idx == len(self.pwm_table) ):
            force = self.force_table[ self.length_pwm_table -1 ]
        elif _float_equal( pwm , self.pwm_table[ idx ] ):
            force = self.force_table[ idx ]
        else:
            force = ( self.force_table[ idx-1 ] 
                    + ( ( self.force_table[ idx ] - self.force_table[ idx - 1 ] ) 
                        * ( pwm - self.pwm_table[ idx - 1 ] )
                        / ( self.pwm_table[ idx ] - self.pwm_table[ idx - 1] ) ) )
        return force
