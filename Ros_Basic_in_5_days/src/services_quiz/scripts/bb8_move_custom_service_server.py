#! /usr/bin/env python

import rospy
from services_quiz.srv import BB8CustomServiceMessage, BB8CustomServiceMessageResponse
from geometry_msgs.msg import Twist


def my_callback(request):
    print "My_callback has been called"
    for i in range(request.repetitions):
        for j in range(4):
            move.linear.x = 0.0
            move.angular.z = 0.0
            pub.publish(move)
            #forward
            move.linear.x = 0.5
            move.angular.z = 0.0
            pub.publish(move)
            rospy.sleep(request.side)
            #stop
            move.linear.x = 0.0
            move.angular.z = 0.0
            pub.publish(move)
            rospy.sleep(1)
            #turn
            move.linear.x = 0.0
            move.angular.z = 1.57/2
            pub.publish(move)
            rospy.sleep(2)
            #stop
            move.linear.x = 0.0
            move.angular.z = 0.0
            pub.publish(move)
            rospy.sleep(1)

    response = BB8CustomServiceMessageResponse()
    response.success = True
    return response # the service Response class, in this case EmptyResponse
    #return MyServiceResponse(len(request.words.split())) 

rospy.init_node('custom_service_server_bb8_square') 
my_service = rospy.Service('/move_bb8_in_square_custom', BB8CustomServiceMessage , my_callback) # create the Service called my_service with the defined callback
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
move = Twist()
rospy.spin() # maintain the service open.