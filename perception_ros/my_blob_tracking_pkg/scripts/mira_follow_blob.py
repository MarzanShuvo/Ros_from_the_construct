#!/usr/bin/env python
import time
import rospy
from math import pi, sin, cos, acos
import random
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point

"""
Topics To Write on:
type: std_msgs/Float64
/mira/pitch_joint_position_controller/command
/mira/roll_joint_position_controller/command
/mira/yaw_joint_position_controller/command
"""

class MiraBlobFollower(object):

    def __init__(self, is_2D = True):
        
        rospy.loginfo("Mira Initialising Blob Follower...")

        self.move_rate = rospy.Rate(10)

        self._is_2D = is_2D
        self.acceptable_error = 0.2

        self.current_yaw = 0.0
        self.twist_obj = Twist()
        self.pub_mira_move = rospy.Publisher('/mira/commands/velocity',  Twist, queue_size=1)
       
        self.point_blob_topic = "/blob/point_blob"
        self._check_cv_image_ready()
        rospy.Subscriber(self.point_blob_topic, Point, self.point_blob_callback)

        rospy.loginfo("Mira Initialising Blob Follower...")
    

    def _check_cv_image_ready(self):
        self.point_blob = None
        while self.point_blob is None and not rospy.is_shutdown():
            try:
                self.point_blob = rospy.wait_for_message(self.point_blob_topic, Point, timeout=1.0)
                rospy.logdebug("Current "+self.point_blob_topic+" READY=>")

            except:
                rospy.logerr("Current "+self.point_blob_topic+" not ready yet, retrying for getting "+self.point_blob_topic+"")
        return self.point_blob
    
        
    def point_blob_callback(self, msg):
                
        if msg.x > self.acceptable_error:
            self.twist_obj.angular.z = -1.0
        elif msg.x < -1*self.acceptable_error:
            self.twist_obj.angular.z = 1.0
        else:
            self.twist_obj.angular.z = 0.0

        if msg.y > self.acceptable_error:
            self.twist_obj.angular.x = - 1.0
        elif msg.y < -1*self.acceptable_error:
            self.twist_obj.angular.x = 1.0
        else:
            self.twist_obj.angular.x = 0.0
        
        


    def loop(self):

        while not rospy.is_shutdown():

            self.pub_mira_move.publish(self.twist_obj)
            self.move_rate.sleep()


if __name__ == "__main__":
    rospy.init_node('mira_follow_blob_node', anonymous=True, log_level=rospy.DEBUG)
    mira_jointmover_object = MiraBlobFollower()
    mira_jointmover_object.loop()