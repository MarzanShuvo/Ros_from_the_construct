import math, rospy
from utilities import set_model_state, get_model_state, \
                      pause_physics, unpause_physics
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_about_axis
from tf.transformations import quaternion_multiply
from tf.transformations import quaternion_from_euler
# RPY to convert: 90deg, 0, -90deg
q = quaternion_from_euler(math.radians(90), 0, math.radians(-90))
Quaternion(*q)

position = Point(x=0.5, y=0, z=0.5)
roll, pitch, yaw = 90, 0, -90

q_rpy = quaternion_from_euler(math.radians(roll), math.radians(pitch), math.radians(yaw))
orientation = Quaternion(*q_rpy)
set_model_state('coke_can', Pose(position, orientation))
print (orientation)
q_x = quaternion_about_axis(0, (1,0,0))
if roll>=0:
    roll_range = range(0,roll,1)
    offset = 1
else:
    roll_range = range(0,roll,-1)
    offset = -1
for roll_angle in roll_range:
    q_x = quaternion_about_axis(math.radians(roll_angle+offset), (1,0,0))
    orientation = Quaternion(*q_x)
    set_model_state('coke_can', Pose(position, orientation))
    rospy.sleep(0.01)
q_y = quaternion_about_axis(0, (0,1,0))
q_xy = quaternion_multiply(q_y, q_x)
if pitch>=0:
    pitch_range = range(0,pitch,1)
    offset = 1
else:
    pitch_range = range(0,pitch,-1)
    offset = -1
for pitch_angle in pitch_range:
    q_y = quaternion_about_axis(math.radians(pitch_angle+offset), (0,1,0))
    q_xy = quaternion_multiply(q_y, q_x)
    orientation = Quaternion(*q_xy)
    set_model_state('coke_can', Pose(position, orientation))
    rospy.sleep(0.01)
q_z = quaternion_about_axis(0, (0,0,1))
q_xyz = quaternion_multiply(q_z, q_xy)
if yaw>=0:
    yaw_range = range(0,yaw,1)
    offset = 1
else:
    yaw_range = range(0,yaw,-1)
    offset = -1
for yaw_angle in yaw_range:
    q_z = quaternion_about_axis(math.radians(yaw_angle+offset), (0,0,1))
    q_xyz = quaternion_multiply(q_z, q_xy)
    orientation = Quaternion(*q_xyz)
    set_model_state('coke_can', Pose(position, orientation))
    rospy.sleep(0.01)
print (orientation)


