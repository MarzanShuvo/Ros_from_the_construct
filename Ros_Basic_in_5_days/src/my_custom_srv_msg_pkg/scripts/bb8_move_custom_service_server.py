#! /usr/bin/env python

import rospy
from my_custom_srv_msg_pkg.srv import MyCustomServiceMessage, MyCustomServiceMessageResponse
from geometry_msgs.msg import Twist


def my_callback(request):
    print "My_callback has been called"
    for i in range(request.duration):
        move.linear.x = 0.5
        move.angular.z = 0.5
        pub.publish(move)
        rospy.sleep(2)
    move.linear.x = 0.0
    move.angular.z = 0.0
    pub.publish(move)
    response = MyCustomServiceMessageResponse()
    response.success = True
    return response # the service Response class, in this case EmptyResponse
    #return MyServiceResponse(len(request.words.split())) 

rospy.init_node('custom_service_server_bb8_circle') 
my_service = rospy.Service('/move_bb8_in_circle_custom', MyCustomServiceMessage , my_callback) # create the Service called my_service with the defined callback
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
move = Twist()
rospy.spin() # maintain the service open.