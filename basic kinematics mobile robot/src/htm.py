import math, rospy
from utilities import set_model_state, get_model_state
from geometry_msgs.msg import Pose, Point, Quaternion
import numpy as np
from tf.transformations import quaternion_matrix

model_state = get_model_state('coke_can')
pose = model_state.pose
q = np.array([pose.orientation.x, pose.orientation.y, 
              pose.orientation.z, pose.orientation.w])
H = quaternion_matrix(q)
H[0][3] = pose.position.x
H[1][3] = pose.position.y
H[2][3] = pose.position.z

print(pose)
print(H)