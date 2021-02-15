#! /usr/bin/env python

import rospy
import time
import actionlib
from path_exam.msg import RecordOdomGoal, RecordOdomFeedback, RecordOdomResult, RecordOdomAction

def feedback_callback(feedback):
    rospy.loginfo("Rec Odom Feedback feedback ==>"+str(feedback))

rospy.init_node('record_pose_action_client_node')

# create the connection to the action server
client = actionlib.SimpleActionClient('/rec_pose_as', RecordOdomAction)
rate = rospy.Rate(1)

# waits until the action server is up and running
rospy.loginfo('Waiting for action Server')
client.wait_for_server()
rospy.loginfo('Action Server Found...')

# creates a goal to send to the action server
goal = RecordOdomGoal()

# sends the goal to the action server, specifying which feedback function
# to call when feedback received
client.send_goal(goal, feedback_cb=feedback_callback)

state_result = client.get_state()

rospy.loginfo("state_result: "+str(state_result))

while state_result < 2:
    rospy.loginfo("Waiting to finish: ")
    rate.sleep()
    state_result = client.get_state()
    rospy.loginfo("state_result: "+str(state_result))
    

state_result = client.get_state()
rospy.loginfo("[Result] State: "+str(state_result))
if state_result == 4:
    rospy.logerr("Something went wrong in the Server Side")
if state_result == 3:
    rospy.logwarn("There is a warning in the Server Side")
result = client.get_result()
rospy.loginfo("Last Pose: "+str(result.result_pose_array[-1]))
