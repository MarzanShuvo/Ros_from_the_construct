#! /usr/bin/env python

import rospy
from std_srvs.srv import Empty as Empty_srv
from std_srvs.srv import EmptyResponse
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

def my_callback(request):
    rospy.loginfo('Starting my service...')
    #We takeoff the drone during the first 3 seconds
    i=0
    while not i == 3:
        takeoff.publish(takeoff_msg)
        rospy.loginfo('Taking off...')
        time.sleep(1)
        i += 1

    i=0
    #We move the drone forward
    while not i==5:
        move_msg.linear.x = 1
        move_msg.angular.z = 0
        move.publish(move_msg)
        rospy.loginfo('moving forward...')
        time.sleep(1)
        i +=1

    move_msg.linear.x = 0
    move.publish(move_msg)
    time.sleep(1)

    i=0
    while not i == 3:
        move_msg.linear.x = 0
        move_msg.angular.z = 0
        move.publish(move_msg)
        land.publish(land_msg)
        rospy.loginfo('Landing...')
        time.sleep(1)
        i += 1
    response = EmptyResponse()
    return response


rospy.init_node('Motion_service') 
my_service = rospy.Service('/my_service', Empty_srv , my_callback) # create the Service called my_service with the defined callback
move = rospy.Publisher('/cmd_vel', Twist, queue_size=1) #Create a Publisher to move the drone
move_msg = Twist() #Create the message to move the drone
takeoff = rospy.Publisher('/drone/takeoff', Empty, queue_size=1) #Create a Publisher to takeoff the drone
takeoff_msg = Empty() #Create the message to takeoff the drone
land = rospy.Publisher('/drone/land', Empty, queue_size=1) #Create a Publisher to land the drone
land_msg = Empty() #Create the message to land the drone
rospy.spin() # maintain the service open.
