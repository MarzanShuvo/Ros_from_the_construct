#! /usr/bin/env python
import rospkg
import rospy
from services_quiz.srv import BB8CustomServiceMessage, BB8CustomServiceMessageRequest


rospy.init_node('service_move_bb8_in_square_custom_client') # Initialise a ROS node with the name service_client
rospy.wait_for_service('/move_bb8_in_square_custom') # Wait for the service client /move_bb8_in_circle_custom to be running
move_bb8_in_square_service_client = rospy.ServiceProxy('/move_bb8_in_square_custom', BB8CustomServiceMessage) # Create the connection to the service
move_bb8_in_square_request_object = BB8CustomServiceMessageRequest() # Create an object of type EmptyRequest

move_bb8_in_square_request_object.repetitions = 2
move_bb8_in_square_request_object.side = 4.0
result = move_bb8_in_square_service_client(move_bb8_in_square_request_object)
rospy.loginfo(str(result))

move_bb8_in_square_request_object.repetitions = 1
move_bb8_in_square_request_object.side = 8.0
result = move_bb8_in_square_service_client(move_bb8_in_square_request_object)
rospy.loginfo(str(result))


rospy.loginfo("END of Service call...")

