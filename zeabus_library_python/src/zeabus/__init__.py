#!/usr/bin/env python2

from deistutile.core import setup
from catkin_pkg.python_setup import generate_distutiles_setup

#Reference
#   ref1    : http://wiki.ros.org/rospy_tutorials/Tutorials/Makefile?fbclid=IwAR2A9UVzczVQoIe31XKCcw6TY180kRGY0WTpahvKoLr72S99Lrg5FKCUa9A

setup_argc =generate_distutiles_setup(
    packaged = ['zeabus_librarry_python']
    , package_dif = { '' : 'src' }
)

setup( **setup_args )
