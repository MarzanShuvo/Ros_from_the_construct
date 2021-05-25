import math, rospy
from utilities import set_model_state, get_model_state
from geometry_msgs.msg import Pose, Point, Quaternion

x, y, z = 1, 0, 0.22
for angle in range(0,360,10):
    theta = math.radians(angle)
    xp = x * math.cos(theta) - y * math.sin(theta)
    yp = x * math.sin(theta) + y * math.cos(theta)
    set_model_state('coke_can', Pose(position=Point(xp,yp,z)))
    rospy.sleep(0.1)