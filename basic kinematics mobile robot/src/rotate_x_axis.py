#! /usr/bin/env python

import math, rospy
from utilities import set_model_state, get_model_state, \
                      spawn_coke_can, spawn_table, \
                      pause_physics, unpause_physics
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_about_axis

position = Point(x=0.5, y=0, z=0.5)

for angle in range(0,360,10):
    theta = math.radians(angle)
    q = quaternion_about_axis(theta, (1,0,0))
    orientation = Quaternion(*q)
    set_model_state('coke_can', Pose(position, orientation))
    rospy.sleep(0.1)