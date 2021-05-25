import rospy, math, numpy as np
from std_msgs.msg import Float32MultiArray
from utilities import reset_world

rospy.init_node('holonomic_controller', anonymous=True)
pub = rospy.Publisher('wheel_speed', Float32MultiArray, queue_size=10)

forward = [1, 1, 1, 1]
msg = Float32MultiArray(data=forward)
pub.publish(msg)

rospy.sleep(5.0)

stop = [0, 0, 0, 0]
msg = Float32MultiArray(data=stop)
pub.publish(msg)