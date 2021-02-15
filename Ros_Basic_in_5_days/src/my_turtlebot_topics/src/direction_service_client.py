#! /usr/bin/env python

import rospy
from std_srvs.srv import Trigger, TriggerRequest

rospy.init_node('crash_direction_service_client')
rospy.wait_for_service("/crash_direction_service")
direction_service = rospy.ServiceProxy("/crash_direction_service", Trigger)
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
        rospy.logwarn("Success =="+str(result.success)) # print the result given by the service called
        rospy.logwarn("Direction To Go=="+str(result.message)) # print the result given by the service called
    else:
        rospy.loginfo("Success =="+str(result.success)) # print the result given by the service called
        rospy.loginfo("Direction To Go=="+str(result.message)) # print the result given by the service called
    rate.sleep()