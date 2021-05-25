import math, rospy
from utilities import set_model_state, get_model_state, \
                      pause_physics, unpause_physics
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_about_axis

position = Point(x=0.5, y=0, z=0.5)
for angle in range(0,90,3):
    q_y = quaternion_about_axis(math.radians(angle), (0,1,0))
    orientation = Quaternion(*q_y)
    set_model_state('coke_can', Pose(position, orientation))
    rospy.sleep(0.1)