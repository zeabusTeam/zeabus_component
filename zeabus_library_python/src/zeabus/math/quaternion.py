#!/usr/bin/env python
# FILE			: quaternion.py
# AUTHOR		: K.Supasan , W.Natthaphat
# CREATE ON		: 2019, June 05 (UTC+0)
# MAINTAINER	: K.Supasan

# README
#   This code have inspiration to create from use python quaternion to distribute data between
#       parent frame and child frame
#   In the order of quaternion we will use standard follow ref1
#   Tuple of quaternion will show by order is x y z w

# REFERENCE
#   ref01 : http://docs.ros.org/melodic/api/tf/html/python/transformations.html#module-tf.transform
#   ref02 : https://docs.python.org/2/tutorial/classes.html
#   ref03 : https://www.geeksforgeeks.org/operator-overloading-in-python/

import math
import numpy
from . import general
from tf import transformations as tf_handle

class Quaternion :

    # I use vector because quaternion is subset of vector but quaternion is vector in
    #   4 Dimension

    # We will init quaternion at roll pitch yaw is 0 radian
    def __init__ (self):
        self.vector = ( 0, 0, 0, 1 )
       
    # function set_quaternion use to set by input tuple of quaternion
    def set_quaternion (self , quaternion):
        self.vector = quaternion

    # We follow order rotation is yaw pitch roll.
    # This order will apply use in system of zeabus team
    def set_euler (self , yaw , pitch , roll):
        temp = tf_handle.quaternion_from_euler( yaw, pitch, roll, axes='rzyx')
        self.set_quaternion( temp )

    # function get_euler use to find euler from quaternion 
    def get_euler (self):
        temp = tf_handle.euler_from_quaternion( self.vector , axes='rzyx')
        result = ( general.bound_radian( temp[0] )
            , general.bound_radian( temp[1])
            , general.bound_radian( temp[2]) )
        return temp

    def print_quaternion( self , name ):
        print( "{:s} is {:6.3f} {:6.3f} {:6.3f} {:6.3f}".format( name
            , self.vector[0] 
            , self.vector[1] 
            , self.vector[2]
            , self.vector[3] ) )

    # function inverse of quaternion inverse of quaternion is conjugate all imginary part
    def inverse(self):
        q = Quaternion()
        q.set_quaternion( ( -self.vector[0], -self.vector[1], -self.vector[2], self.vector[3] ) )
        return q

    def normalize(self):
        temp = tf_handle.unit_vector( self.vector )
        self.set_quaternion( temp )

    # Function this use to rotation data
    def rotation(self, quaternion ):
        temp = Quaternion()
        temp.set_quaternion( quaternion )
        result = self * temp * self.inverse()
        return result

    # This use in pattern of multiple between object quaternion
    def __mul__(self, other_quatanion):
        temp = tf_handle.quaternion_multiply( self.vector, other_quatanion.vector )
        q = Quaternion()
        q.set_quaternion(temp)
        return q

