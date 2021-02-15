#! /usr/bin/env python

import rospy
import time
from sensor_msgs.msg import LaserScan

class LaserTopicReader(object):
    def __init__(self, topic_name = '/kobuki/laser/scan'):
        self._topic_name = topic_name
        self._sub = rospy.Subscriber(self._topic_name, LaserScan, self.topic_callback)
        self._laserdata = LaserScan()
        self._front = 0.0
        self._right = 0.0
        self._left = 0.0
    
    def topic_callback(self, msg):
        self._laserdata = msg
        rospy.logdebug(self._laserdata)
    
    def get_laserdata(self):
        """
        Returns the newest odom data

        std_msgs/Header header
          uint32 seq
          time stamp
          string frame_id
        float32 angle_min
        float32 angle_max
        float32 angle_increment
        float32 time_increment
        float32 scan_time
        float32 range_min
        float32 range_max
        float32[] ranges
        float32[] intensities                                                                                                               
        
        """
        return self._laserdata
        
    def crash_detector(self):
        
        self._front = self._laserdata.ranges[360]
        self._right = self._laserdata.ranges[0]
        self._left = self._laserdata.ranges[719]
        rospy.loginfo("Front Distance == "+str(self._front))
        rospy.loginfo("Left Distance == "+str(self._left))
        rospy.loginfo("Right Distance == "+str(self._right))
        
        
        return self.convert_to_dict()
        
        
    def convert_to_dict(self):
        """
        Converts the fiven message to a dictionary telling in which direction there is a detection
        """
        detect_dict = {}
        # We consider that when there is a big Z axis component there has been a very big front crash
        detection_dict = {"front":self._front,
                          "left":self._left,
                          "right":self._right}
        return detection_dict
        
    
if __name__ == "__main__":
    rospy.init_node('laser_topic_subscriber', log_level=rospy.INFO)
    laser_reader_object = LaserTopicReader()
    time.sleep(2)
    rate = rospy.Rate(0.5)
    
    ctrl_c = False
    def shutdownhook():
        # works better than the rospy.is_shut_down()
        global ctrl_c
        print "shutdown time!"
        ctrl_c = True

    rospy.on_shutdown(shutdownhook)
    
    while not ctrl_c:
        data = laser_reader_object.get_laserdata()
        laser_reader_object.crash_detector()
        rate.sleep()