#! /usr/bin/env python
import rospy
from nav_msgs.srv import GetMap, GetMapRequest
import sys

rospy.init_node('service_client')
rospy.wait_for_service('/static_map')
get_map_service = rospy.ServiceProxy('/static_map', GetMap)
get_map = GetMapRequest()
result = get_map_service(get_map)
print(result)