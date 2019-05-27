#!/usr/bin/env python2
"""
FILE			: thruster_mapper_enu.py
AUTHOR		    : K.Supasan
CREATE ON	    : 2019, May 26 (UTC+0)
MAINTAINER	    : K.Supasan
ORIGINAL FILE   : K.Supakit 2018/11/14

README
  This code have response about command to use matrix distribute for from world frame to
      robot frame.
  In the future, I think this node should have process to detect about frame id to checkout
      about piority to use so that is the future time.
  If above line have I think we must to use thread manage about that process
  About tf library. In different between python and cpp style coding.
      Python language have purpose about to interesting process and how to
      CPP language have stlye to interesting about theory and ristric
      All that make python don't have quaternion struct please ref02
      You will know quaternion have been representing in mode numpy.dyarray for parameter
          you must to pass by use tuple to pass parameter
  In noted of ref02 you will know this tf library not optimized for speed!
  Axes 4-str : will have affect about output and input. Only quaternion don't affect from this
      first str will be 's'tatic or 'r'otation
          - 's' will return euler from that quaternion to origin
          - 'r' will return euler form origin to that quaternion
  Axes 4-tuple :
      Array or tuple will representing to ix+jy+kz+w to [ x , y , z , w ]

REFERENCE
  ref01 : http://wiki.ros.org/ROS/Tutorials/WritingServiceClient(python)
  ref02 : http://docs.ros.org/melodic/api/tf/html/python/transformations.html#module-tf.transformations
  ref03 : https://pyformat.info/
  ref04 : https://www.w3schools.com/python/ref_func_print.asp
  ref05 : https://docs.python.org/3/whatsnew/3.0.html
"""

from __future__ import print_function

import math
import rospy
import numpy
from lookup_pwm_force import lookup_pwm_force
from zeabus_utility.srv import SendThrottle,  SendControlCommand, GetAUVState
from zeabus_utility.msg import ControlCommand
from std_msgs.msg import Header
from tf import transformations as tf_handle
from constant import *


class ThrusterMapper:
    """
    function init that is function to constructor class
    I will use to assign about const data that mean that data will not change
    """

    def __init__(self):

        rospy.init_node('control_thruster')

        self.client_throttle = rospy.ServiceProxy(
            '/hardware/thruster_throttle', SendThrottle)

        self.client_state = rospy.ServiceProxy(
            '/fusion/auv_state', GetAUVState)

        self.server = rospy.Service(
            '/control/thruster', SendControlCommand, self.callback)

        self.lookup_handle = lookup_pwm_force(
            "zeabus_control", "scripts", "4th_t200_16.txt")

        cos_45 = math.cos(math.radians(45))
        sin_45 = math.sin(math.radians(45))

        # this variable will show about position to calculate force
        self.direction_linear = numpy.array([
            [0, 0, 1],              # thruster id 0
            [0, 0, 1],              # thruster id 1
            [0, 0, 1],              # thruster id 2
            [0, 0, 1],              # thruster id 3
            [-cos_45, sin_45, 0],   # thruster id 4
            [-cos_45, -sin_45, 0],  # thruster id 5
            [cos_45, sin_45, 0],    # thruster id 6
            [cos_45, -sin_45, 0]    # thruster id 7
        ])

        self.distance = numpy.array([
            [0.332, 0.2202, -0.023],   # thruster id 0
            [0.332, -0.2202, -0.023],  # thruster id 1
            [-0.332, 0.2202, -0.023],  # thruster id 2
            [-0.332, -0.2202, -0.023],  # thruster id 3
            [0.3536, 0.3536, -0.023],  # thruster id 4
            [0.3536, -0.3536, -0.023],  # thruster id 5
            [-0.3536, 0.3536, -0.023],  # thruster id 6
            [-0.3536, -0.3536, -0.023]  # thruster id 7
        ])

        # this variable will show about momentum to calculate about rotatio by euler
        self.direction_angular = numpy.array([
            # thruster id 0
            # thruster id 1
            # thruster id 2
            # thruster id 3
            # thruster id 4
            # thruster id 5
            # thruster id 6
            numpy.cross(self.distance[0], self.direction_linear[0]), numpy.cross(self.distance[0], self.direction_linear[1]), numpy.cross(self.distance[0], self.direction_linear[1]), numpy.cross(self.distance[0], self.direction_linear[1]), numpy.cross(self.distance[0], self.direction_linear[1]), numpy.cross(self.distance[0], self.direction_linear[1]), numpy.cross(self.distance[0], self.direction_linear[1]), numpy.cross(self.distance[0], self.direction_linear[1])])  # thruster id 7

        self.direction = numpy.concatenate(
            (self.direction_linear, self.direction_angular), axis=1)

        self.direction_inverse = numpy.linalg.pinv(self.direction)

        self.current_quaternion = (0, 0, 0, 1)
        # At ref02 you can use tuple instead numpy.array
        # we have to use current quaternion because input from control will always world frame
        #   that make us have convert world frame to robot frame

        self.header = Header()
        self.header.frame_id = "robot"

    def update_state(self):
        current_auv_state = self.client_state()
        self.current_quaternion = current_auv_state.auv_state.pose.pose.orieantation
        if(THRUSTER_MAPPER_UPDATED):
            if(THRUSTER_MAPPER_EULER):
                print("UPDATED CURRENT EULER : ",
                      tf_handle.euler_from_quaternion(self.current_quaternion))
            else:
                print("UPDATED CURRENT QUATERNION : ", self.current_quaternion)

    def callback(self, request):

        if(THRUSTER_MAPPER_AUV_STATE):
            self.update_state()
            robot_linear_force = (tf_handle.quaternion_multiply(
                tf_handle.quaternion_multiply(
                    self.current_quaternion, (request.target)[0:3]), tf_handle.quaternion_inverse(self.current_quaternion)))[0:3]

        if(THRUSTER_MAPPER_CALCULATE_PROCESS):
            print("Force linear target : ", (request.command)[0:3])
            if(THRUSTER_MAPPER_EULER):
                print("Current euler : ", tf_handle.euler_from_quaternion(
                    self.current_quaternion))
            else:
                print("Current quaternion  : ", self.current_quaternion)
            print("Force linear robot : ", robot_linear_force[0:3])

        force = numpy.array([
            robot_linear_force[0], robot_linear_force[1], robot_linear_force[2], (request.command)[3], (request.command)[4], (request.command[5])])

        if(THRUSTER_MAPPER_CALCULATE_PROCESS):
            print("Force result robot frame : ", force)

        torque = numpy.matmul(self.direction_inverse.T, force.T)

        cmd = []
        for run in range(0, 8):
            cmd.append(self.lookup_handle.find_pwm(torque[run]))

        self.header.stamp = rospy.get_rostime()
        pwm = tuple(cmd)

        if(THRUSTER_MAPPER_RESULT):
            print("{:6d} {:6d} {:6d} {:6d} {:6d} {:6d} {:6d} {:6d}".format(
                  pwm[0], pwm[1], pwm[2], pwm[3], pwm[4], pwm[5], pwm[6], pwm[7]))

        self.client_throttle(self.header, pwm)


if __name__ == '__main__':
    thruster_mapper = ThrusterMapper
    rospy.spin()
