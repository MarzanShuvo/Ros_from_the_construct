#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

rospy.init_node("publish_vels_node")
pub = rospy.Publisher('/twist_vels', Twist, queue_size=1)
vels = Twist()
vels.linear.x = 0.1
vels.angular.z = 0.1
rate = rospy.Rate(1)
while not rospy.is_shutdown():
    pub.publish(vels)
    rate.sleep()