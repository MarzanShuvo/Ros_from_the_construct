#! /usr/bin/env python
import rospy
from std_msgs.msg import Empty


pub = rospy.Publisher('/drone/takeoff', Empty, queue_size=1)
rospy.init_node('taking_off', anonymous=True)

i=0
takeoff_msg = Empty()
while not (i==3):
    rospy.loginfo("Taking off....... ")
    pub.publish(takeoff_msg)
    rospy.sleep(1)
    i +=1

