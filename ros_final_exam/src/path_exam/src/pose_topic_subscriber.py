#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Pose

class PoseTopicReader(object):
    def __init__(self, topic_name = '/drone/gt_pose'):
        self._topic_name = topic_name
        self._sub = rospy.Subscriber(self._topic_name, Pose, self.topic_callback)
        self._posedata = Pose()
    
    def topic_callback(self, msg):
        self._posedata = msg
        rospy.logdebug(self._posedata)
    
    def get_posedata(self):
        
        return self._posedata
    
if __name__ == "__main__":
    rospy.init_node('pose_topic_subscriber', log_level=rospy.INFO)
    pose_reader_object = PoseTopicReader()
    rospy.loginfo(pose_reader_object.get_posedata())
    rate = rospy.Rate(0.5)
    
    ctrl_c = False
    def shutdownhook():
        # works better than the rospy.is_shut_down()
        global ctrl_c
        print "shutdown time!"
        ctrl_c = True

    rospy.on_shutdown(shutdownhook)
    
    while not ctrl_c:
        data = pose_reader_object.get_posedata()
        rospy.loginfo(data)
        rate.sleep()