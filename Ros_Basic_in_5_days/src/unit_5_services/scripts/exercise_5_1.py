#! /usr/bin/env python

import rospy
# Import the service message used by the service /trajectory_by_name
from iri_wam_reproduce_trajectory.srv import ExecTraj, ExecTrajRequest
import sys
import rospkg

# Initialise a ROS node with the name service_client
rospy.init_node('service_client_execute')
# Wait for the service client /trajectory_by_name to be running
rospy.wait_for_service('/execute_trajectory')
# Create the connection to the service
execute_service = rospy.ServiceProxy('/execute_trajectory', ExecTraj)
# Create an object of type TrajByNameRequest
execute_object = ExecTrajRequest()
# Fill the variable traj_name of this object with the desired value
rospack = rospkg.RosPack()
# This rospack.get_path() works in the same way as $(find name_of_package) in the launch files.
trajectory_file_path = rospack.get_path('iri_wam_reproduce_trajectory') + "/config/get_food.txt"

execute_object.file = trajectory_file_path
# Send through the connection the name of the trajectory to be executed by the robot
result = execute_service(execute_object)
# Print the result given by the service called
print result