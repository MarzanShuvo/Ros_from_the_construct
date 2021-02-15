#! /usr/bin/env python

import rospy
from std_srvs.srv import Trigger, TriggerRequest

rospy.init_node('Motion_service_client')
rospy.wait_for_service("/my_service")
direction_service = rospy.ServiceProxy("/my_service", Trigger)
request_object = TriggerRequest()

rate = rospy.Rate(5)

ctrl_c = False
def shutdownhook():
    global ctrl_c
    print "shutdown time"
    ctrl_c = True

rospy.on_shutdown(shutdownhook)

while not ctrl_c:
    result = direction_service(request_object) # send through the connection the name of the object to be deleted by the service
    """
    ---                             
    bool success   # indicate succes
    string message # informational, 
    """
    if result.success:
        rospy.loginfo("Success =="+str(result.success)) # print the result given by the service called
        rospy.loginfo("Direction To Go=="+str(result.message)) # print the result given by the service called
        break