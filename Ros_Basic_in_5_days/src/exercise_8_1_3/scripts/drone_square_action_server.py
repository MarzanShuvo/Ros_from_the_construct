#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import actionlib
from std_msgs.msg import Empty
from actionlib.msg import TestFeedback, TestResult, TestAction

class MoveDroneSquare(object):

    _feedback = TestFeedback()
    _result = TestResult()

    def __init__(self):
        self._as = actionlib.SimpleActionServer("Ardrone_square_action_server", TestAction, self.goal_callback, False)
        self._as.start()
        self.ctrl_c = False
        self.rate = rospy.Rate(10)

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

    def drone_stop(self):
        rospy.loginfo("Stoping....")
        self._move_msg.linear.x = 0.0
        self._move_msg.angular.z = 0.0
        self.publish_once_in_cmd_vel(self._move_msg)

    def drone_forward(self):
        rospy.loginfo("Moving Forward....")
        self._move_msg.linear.x = 1.0
        self._move_msg.angular.z = 0.0
        self.publish_once_in_cmd_vel(self._move_msg)

    def drone_turn(self):
        rospy.loginfo("Turning....")
        self._move_msg.linear.x = 0.0
        self._move_msg.angular.z = 1.0
        self.publish_once_in_cmd_vel(self._move_msg)



    def goal_callback(self, goal):

        side_square = goal.goal
        turning_time = 1.8

        # helper variables
        r = rospy.Rate(1)
        success = True
        
        # define the different publishers and messages that will be used
        self._pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self._move_msg = Twist()
        self._pub_takeoff = rospy.Publisher('/drone/takeoff', Empty, queue_size=1)
        self._takeoff_msg = Empty()
        self._pub_land = rospy.Publisher('/drone/land', Empty, queue_size=1)
        self._land_msg = Empty()

        i = 0
        while not i ==3:
            self._pub_takeoff.publish(self._takeoff_msg)
            rospy.sleep(1)
            rospy.loginfo("Taking off")
            i +=1
        
        for i in range(4):
            # check that preempt (cancelation) has not been requested by the action client
            if self._as.is_preempt_requested():
                rospy.loginfo('The goal has been cancelled/preempted')
                # the following line, sets the client in preempted state (goal cancelled)
                self._as.set_preempted()
                success = False
                # we end the calculation of the Fibonacci sequence
                break

            self.drone_stop()
            rospy.sleep(1)
            self.drone_forward()
            rospy.sleep(side_square)
            self.drone_stop()
            rospy.sleep(1)
            self.drone_turn()
            rospy.sleep(turning_time)
            self.drone_stop()
            rospy.sleep(1)

            # build and publish the feedback message
            self._feedback.feedback = i
            self._as.publish_feedback(self._feedback)
            # the sequence is computed at 1 Hz frequency
            r.sleep()

        if success:
            self._result.result = (side_square*4) + (turning_time*4)+3*4
            rospy.loginfo('The total seconds it took the drone to perform the square was %i' % self._result.result )
            self._as.set_succeeded(self._result)
                
            # make the drone stop and land
            self.drone_stop()
            i=0
            while not i == 3:
                self._pub_land.publish(self._land_msg)
                rospy.loginfo('Landing...')
                rospy.sleep(1)
                i += 1

if __name__ == '__main__':
  rospy.init_node('ardone_move_square')
  MoveDroneSquare()
  rospy.spin()
