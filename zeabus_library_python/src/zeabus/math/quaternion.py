#!/usr/bin/env python
# FILE			: quaternion.py
# AUTHOR		: K.Supasan , W.Natthaphat
# CREATE ON		: 2019, June 05 (UTC+0)
# MAINTAINER	: K.Supasan

# README
#   This code have inspiration to create from use python quaternion to distribute data between
#       parent frame and child frame

# REFERENCE
#   ref01 : http://docs.ros.org/melodic/api/tf/html/python/transformations.html#module-tf.transform
#   ref02 : https://docs.python.org/2/tutorial/classes.html
#   ref03 : https://www.geeksforgeeks.org/operator-overloading-in-python/

import math
import numpy
from tf import transformations as tf_handle

class Quaternion :
    def __init__ (self):
        self.x = 0
        self.y = 0
        self.z = 0
        self.w = 0

    def set_quaternion (self,quaternion):
        self.x = quaternion[0]
        self.y = quaternion[1]
        self.z = quaternion[2]
        self.w = quaternion[3]

    def set_euler (self , yaw , pitch , roll):
        self = tf_handle.quaternion_from_euler(yaw, pitch, roll, axes='sxyz')

    def print_quaternion(self):
        print self.x , self.y , self.z , self.w

    def convert_to_tuple(self):
        return (self.x,self.y,self.z,self.w)

    def __mul__(self,other_quatanion):
        temp = tf_handle.quaternion_multiply(self.convert_to_tuple()
                , other_quatanion.convert_to_tuple())
        q = Quaternion()
        q.set_quaternion(temp)
        return q

    def inverse(self):
        temp = tf_handle.quaternion_inverse(self.convert_to_tuple())
        self.set_quaternion(temp)

    def nomalize(self):
        temp = tf_handle.unit_vector(self.convert_to_tuple())
        self.set_quaternion(temp)

    def __sub__(self,other_quatanion):
        temp = numpy.subtract(self.convert_to_tuple(),other_quatanion.convert_to_tuple())
        q = Quaternion()
        q.set_quaternion(temp)
        return q

    def rotation(self,angle,direction):
        q = Quaternion()
        temp = tf_handle.rotation_matrix(angle, direction)
        q.set_quaternion(temp)
        return q
