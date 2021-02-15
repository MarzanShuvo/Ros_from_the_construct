#! /usr/bin/env python

import rospy
import actionlib
from path_exam.msg import RecordOdomFeedback, RecordOdomResult, RecordOdomAction
from pose_topic_subscriber import PoseTopicReader

class RecordPoseClass(object):
    
    def __init__(self):
        """
        It starts an action Server. To test it was created correctly, just rostopic the list and search for /rec_odom_as/...
        When launching, bear in mind that you should have:
        $catkin_make
        $source devel/setup.bash
        """
        # creates the action server
        self._as = actionlib.SimpleActionServer("/rec_pose_as", RecordOdomAction, self.goal_callback, False)
        self._as.start()
        
        # Create an object that reads from the topic Odom
        self._pose_reader_object = PoseTopicReader()
        
        # create messages that are used to publish result
        self._result   = RecordOdomResult()
        
        self._seconds_recording = 20
    
    
    def goal_callback(self, goal):
    
        success = True
        rate = rospy.Rate(1)
        
        for i in range(self._seconds_recording):
            rospy.loginfo("Recording Pose index="+str(i))
            # check that the preempt (cancelation) has not been requested by the action client
            if self._as.is_preempt_requested():
                rospy.logdebug('The goal has been cancelled/preempted')
                # the following line sets the client in a preempted state (goal cancelled)
                self._as.set_preempted()
                success = False
                # we end the action loop
                break
            
            else:# builds the next feedback msg to be sent
                
                rospy.logdebug('Reading Pose...')
                self._result.result_pose_array.append(self._pose_reader_object.get_posedata())
                
            rate.sleep()
        
        # at this point, either the goal has been achieved (success==true)
        # or the client preempted the goal (success==false)
        # If successful, then we publish the final result
        # If not successful, we do not publish anything in the result
        if success:
            self._as.set_succeeded(self._result)
            # Clean the Result Variable
        
        self.clean_variables()
    
    def clean_variables(self):
        """
        Cleans variables for the next call
        """
        self._result   = RecordOdomResult()
    

    

if __name__ == '__main__':
  rospy.init_node('record_odom_action_server_node')
  RecordPoseClass()
  rospy.spin()  