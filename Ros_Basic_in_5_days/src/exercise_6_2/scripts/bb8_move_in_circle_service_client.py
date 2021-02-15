#! /usr/bin/env python

import rospy
from std_srvs.srv import Empty, EmptyRequest

# Initialise a ROS node with the name service_client
rospy.init_node('service_client_bb8_circle')
# Wait for the service client /trajectory_by_name to be running
rospy.wait_for_service('/my_service_bb8_circle')
# Create the connection to the service
bb8_service = rospy.ServiceProxy('/my_service_bb8_circle', Empty)
# Create an object of type TrajByNameRequest
bb8_service_object = EmptyRequest()
result = bb8_service(bb8_service_object)
# Print the result given by the service called
print result