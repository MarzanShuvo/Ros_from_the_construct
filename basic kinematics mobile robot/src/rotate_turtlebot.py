import math, rospy
from utilities import set_model_state, get_model_state
from geometry_msgs.msg import Pose, Point, Quaternion

position = Point(x=0, y=0, z=0)
for angle in range(0,360,10):
    theta = math.radians(angle)
    orientation = Quaternion(x=0, y=0, z=math.sin(theta/2), w=math.cos(theta/2))
    set_model_state('mobile_base', Pose(position, orientation))
    rospy.sleep(0.1)