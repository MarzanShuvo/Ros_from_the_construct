#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time
pub = rospy.Publisher('/cmd_vel',Twist, queue_size=1)

def callback(msg):
    data = msg.ranges
    middle = data[int(len(data)/2)]
    left = data[0]
    right = data[len(data)-1]
    move = Twist()
    print("middel: ", middle)
    print("left: ",left)
    print("right: ",right)

    
    if(middle<30):

        if(middle<1.0 or left<1.0 or (middle==float('inf')and left<1.0) or (middle<1.0 and left<1.0) or (middle>1 and left<1.0)):
            move.linear.x = 0.0
            move.angular.z = 5
            pub.publish(move)

        else:
            move.linear.x = 0.5
            move.angular.z = 0.0
            pub.publish(move)
    
    if(middle>30 and left<1):
        
        move.linear.x = 0.0
        move.angular.z = 5
        pub.publish(move)

    if(middle>30):
        move.linear.x = 0.5
        move.angular.z = 0.0
        pub.publish(move)
    

    

rospy.init_node('topics_quiz_node')
sub = rospy.Subscriber('/kobuki/laser/scan', LaserScan, callback)
rospy.spin()