#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

rospy.init_node('moving_robot_publisher')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
rate = rospy.Rate(2)
var = Twist()
var.linear.x = 0.5
var.angular.z = 0.5

while not rospy.is_shutdown():
    pub.publish(var)
    rate.sleep()