#!/usr/bin/env python2
# FILE			: broadcaster.py
# AUTHOR		: K.Supasan
# CREATE ON		: 2019, June 27 (UTC+0)
# MAINTAINER	: K.Supasan

# README

# REFERENCE
# ref01 : http://docs.ros.org/melodic/api/tf/html/python/tf_python.html#transformbroadcaster

import tf
import rospy

from ..math.quaternion import Quaternion

class Broadcaster:

    def __init__( self , parent_frame , child_frame ):

        self.parent_frame = parent_frame
        self.child_frame = child_frame

        self.broadcaster = tf.TransformBroadcaster()

    # you will give us a linear translation and quaternion rotation
    def quaternion( self , linear , quaternion ):
        self.broadcaster( linear 
            , quaternion 
            , rospy.Time() 
            , self.child_frame 
            , self.parent_frame )

    # you will give us a linear translation and euler rotation
    def euler( self , linear , roll , pitch , yaw ):
        temp_quaternion = Quaternion()
        temp_quaternion.set_euler( yaw , pitch , roll )
        self.quaternion( linear , temp_quaternion.vector )
