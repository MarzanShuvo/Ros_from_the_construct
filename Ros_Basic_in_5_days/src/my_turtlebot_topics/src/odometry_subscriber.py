#! /usr/bin/env python
import rospy
from nav_msgs.msg import Odometry

class OdometrySubcribe(object):
    def __init__(self):
        self._odometry_subs = rospy.Subscriber("/odom", Odometry, self.topic_callback)
        self._odometry_data = Odometry()

    def topic_callback(self, msg):
        self._odometry_data = msg
        rospy.logdebug(self._odometry_data)

    def get_odometry_data(self):
        return self._odometry_data

if __name__ == "__main__":
    rospy.init_node('odom_topic_subscriber', log_level=rospy.INFO)
    odom_reader_object = OdometrySubcribe()
    rospy.loginfo(odom_reader_object.get_odometry_data())
    rate = rospy.Rate(0.5)
    
    ctrl_c = False
    def shutdownhook():
        # works better than the rospy.is_shut_down()
        global ctrl_c
        print "shutdown time!"
        ctrl_c = True

    rospy.on_shutdown(shutdownhook)
    
    while not ctrl_c:
        data = odom_reader_object.get_odometry_data()
        rospy.loginfo(data)
        rate.sleep()