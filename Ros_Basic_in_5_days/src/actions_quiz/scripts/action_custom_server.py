#!/usr/bin/env python

import rospy
import actionlib
from std_msgs.msg import Empty
from actions_quiz.msg import CustomActionMsgFeedback, CustomActionMsgResult, CustomActionMsgAction

class CustomActionDrone(object):

    _feedback = CustomActionMsgFeedback()
    _result   = CustomActionMsgResult()
    
    def __init__(self):
        # creates the action server
        self._as = actionlib.SimpleActionServer("action_custom_msg_as", CustomActionMsgAction, self.goal_callback, False)
        self._as.start()
        
    
  
    def publish_once_in_cmd_vel(self, cmd):
        """
        This is because publishing in topics sometimes fails teh first time you publish.
        In continuos publishing systems there is no big deal but in systems that publish only
        once it IS very important.
        """
        while not self.ctrl_c:
            connections = self._pub_cmd_vel.get_num_connections()
            if connections > 0:
                self._pub_cmd_vel.publish(cmd)
                rospy.loginfo("Publish in cmd_vel...")
                break
            else:
                self.rate.sleep()

    def goal_callback(self, goal):

        success = True

        self._pub_takeoff = rospy.Publisher('/drone/takeoff', Empty, queue_size=1)
        self._takeoff_msg = Empty()
        self._pub_land = rospy.Publisher('/drone/land', Empty, queue_size=1)
        self._land_msg = Empty()

        goal_string = goal.goal
        rospy.loginfo("Goal is to: " + str(goal_string))

        '''if(goal_string=="TAKEOFF"):
            i = 0
            while not (i==3):
                rospy.loginfo("Taking off....... ")
                self._feedback.feedback = "Taking Off"
                self._as.publish_feedback(self._feedback)
                self._pub_takeoff.publish(self._takeoff_msg)
                rospy.sleep(1)
                i +=1
        

        if(goal_string=="LAND"):
            i = 0
            while not (i==3):
                rospy.loginfo("Landing....... ")
                self._feedback.feedback = "Landing"
                self._as.publish_feedback(self._feedback)
                self._pub_takeoff.publish(self._land_msg)
                rospy.sleep(1)
                i +=1'''

        i=0
        while not (i==5):

            if self._as.is_preempt_requested():
                rospy.loginfo('The goal has been cancelled/preempted')
                # the following line, sets the client in preempted state (goal cancelled)
                self._as.set_preempted()
                success = False
                break

            if(goal_string=="TAKEOFF"):
                rospy.loginfo("Taking off....... ")
                self._feedback.feedback = "Taking Off"
                self._as.publish_feedback(self._feedback)
                self._pub_takeoff.publish(self._takeoff_msg)

            if(goal_string=="LAND"):
                rospy.loginfo("Landing....... ")
                self._feedback.feedback = "Landing"
                self._as.publish_feedback(self._feedback)
                self._pub_land.publish(self._land_msg)

            rospy.sleep(1)
            i+=1




        if(success):
            self._result = Empty()       
            self._as.set_succeeded(self._result)


if __name__ == '__main__':
  rospy.init_node('ardone_action_custom_msg')
  CustomActionDrone()
  rospy.spin()