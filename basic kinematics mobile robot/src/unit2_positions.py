import math, rospy
from utilities import set_model_state, get_model_state, \
                      spawn_coke_can
from geometry_msgs.msg import Pose, Point, Quaternion


if get_model_state('coke_can').success == False:
    spawn_coke_can('coke_can', Pose(position=Point(0,1,0.22)))

model_state = get_model_state('coke_can')
print (model_state.pose.position)

new_pose = Pose(position=Point(1,0,0.22))
set_model_state('coke_can', new_pose)

