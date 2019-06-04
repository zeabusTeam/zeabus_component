#!/usr/bin/env python2

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

#Reference
#   ref1    : http://wiki.ros.org/rospy_tutorials/Tutorials/Makefile?fbclid=IwAR2A9UVzczVQoIe31XKCcw6TY180kRGY0WTpahvKoLr72S99Lrg5FKCUa9A

setup_args = generate_distutils_setup(
    packages = ['zeabus']
    , package_dif = { '' : 'src' }
)

setup( **setup_args )
