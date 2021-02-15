#! /usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan

class LaserSubscribe(object):

    def __init__(self):

        self._laser_subs = rospy.Subscriber('/kobuki/laser/scan', LaserScan, self.topic_callback)
        self._laser_data = LaserScan()

    def topic_callback(self, msg):
        self._laser_data = msg
        rospy.logdebug(self._laser_data)

    def get_laser_data(self):
        return self._laser_data

if __name__=="__main__":
    rospy.init_node('laser_topic_subscriber', log_level=rospy.INFO)
    laser_reader_object = LaserSubscribe()
    rospy.sleep(2)
    rate = rospy.Rate(0.5)
    
    ctrl_c = False
    def shutdownhook():
        # works better than the rospy.is_shut_down()
        global ctrl_c
        print "shutdown time!"
        ctrl_c = True

    rospy.on_shutdown(shutdownhook)
    
    while not ctrl_c:
        data = laser_reader_object.get_laser_data()
        rospy.loginfo(data)
        rate.sleep()
