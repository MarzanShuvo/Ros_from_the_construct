#! /usr/bin/env python
import rospy
import time
import actionlib
from geometry_msgs.msg import Twist
from ardrone_as.msg import ArdroneAction, ArdroneGoal, ArdroneResult, ArdroneFeedback
from std_msgs.msg import Empty

nImage = 1
PENDING = 0
ACTIVE = 1
DONE = 2
WARN = 3
ERROR = 4

# definition of the feedback callback. This will be called when feedback
# is received from the action server
# it just prints a message indicating a new message has been received
def feedback_callback(feedback):
    global nImage
    print('[Feedback] image n.%d received'%nImage)
    nImage += 1

# initializes the action client node
rospy.init_node('drone_action_client')

# create the connection to the action server
client = actionlib.SimpleActionClient('/ardrone_action_server', ArdroneAction)
pub_take_off = rospy.Publisher('/drone/takeoff', Empty, queue_size=1)
pub_land = rospy.Publisher('/drone/land', Empty, queue_size=1)
pub_move = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
move_msg = Twist()
take_land_msg = Empty()
# waits until the action server is up and running
client.wait_for_server()


# creates a goal to send to the action server
goal = ArdroneGoal()
goal.nseconds = 10 # indicates, take pictures along 10 seconds

# sends the goal to the action server, specifying which feedback function
# to call when feedback received
client.send_goal(goal, feedback_cb=feedback_callback)

# Uncomment these lines to test goal preemption:
#time.sleep(3.0)
#client.cancel_goal()  # would cancel the goal 3 seconds after starting

# wait until the result is obtained
# you can do other stuff here instead of waiting
# and check for status from time to time 
# status = client.get_state()
# check the client API link below for more info

state_result = client.get_state()

rospy.loginfo("Robot is taking off for 3 second")
#We takeoff the drone during the first 3 seconds
i=0
while not i == 3:
    pub_take_off.publish(take_land_msg)
    rospy.loginfo('Taking off...')
    time.sleep(1)
    i += 1

'''pub_take_off.publish(take_land_msg)
rospy.loginfo('Taking off...')
time.sleep(1)

pub_take_off.publish(take_land_msg)
rospy.loginfo('Taking off...')
time.sleep(1)'''


rospy.loginfo("Take off completed")


rate = rospy.Rate(1)

rospy.loginfo("state_result: "+str(state_result))

rospy.loginfo("starting to move for taking various type of picture")
while state_result < DONE:
    move_msg.linear.x = 1
    move_msg.angular.z = 1
    pub_move.publish(move_msg)
    rospy.sleep(2)
    state_result = client.get_state()
    rospy.loginfo("Moving......")

rospy.loginfo("[Result] State: "+str(state_result))
if state_result == ERROR:
    rospy.logerr("Something went wrong in the Server Side")
if state_result == WARN:
    rospy.logwarn("There is a warning in the Server Side")

#rospy.loginfo("[Result] State: "+str(client.get_result()))


rospy.loginfo("Robot is landing within 3 second")
i=0
while not i == 3:
    move_msg.linear.x = 0.0
    move_msg.angular.z = 0.0
    pub_move.publish(move_msg)
    pub_land.publish(take_land_msg)
    rospy.loginfo('Landing...')
    time.sleep(1)
    i += 1
rospy.loginfo("Landing is completed")