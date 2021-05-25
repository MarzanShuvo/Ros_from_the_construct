#! /usr/bin/env python

import rospy, numpy as np
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

def odom_callback(msg):
    global phi   
    position = msg.pose.pose.position
    (_, _, phi) = euler_from_quaternion([msg.pose.pose.orientation.x, 
                                         msg.pose.pose.orientation.y, 
                                         msg.pose.pose.orientation.z, 
                                         msg.pose.pose.orientation.w])
    
rospy.init_node('absolute_motion', anonymous=True)
pub = rospy.Publisher('wheel_speed', Float32MultiArray, queue_size=10)
position_sub = rospy.Subscriber("/odom", Odometry, odom_callback)
rospy.sleep(1)

def velocity2twist(dphi, dx, dy):
    R = np.array([[1, 0, 0],
                  [0,  np.cos(phi), np.sin(phi)],
                  [0, -np.sin(phi), np.cos(phi)]])
    v = np.array([dphi, dx, dy])
    v.shape = (3,1)
    twist = np.dot(R, v)
    wz, vx, vy = twist.flatten().tolist()
    return wz, vx, vy

def twist2wheels(wz, vx, vy):
    l = 0.500/2
    r = 0.254/2
    w = 0.548/2
    H = np.array([[-l-w, 1, -1],
                  [ l+w, 1,  1],
                  [ l+w, 1, -1],
                  [-l-w, 1,  1]]) / r
    twist = np.array([wz, vx, vy])
    twist.shape = (3,1)
    u = np.dot(H, twist)
    return u.flatten().tolist()

for _ in range(100):
    wz, vx, vy = velocity2twist(dphi=1.5708, dx=1, dy=0)
    u = twist2wheels(wz, vx, vy)
    msg = Float32MultiArray(data=u)
    pub.publish(msg)
    rospy.sleep(0.01)
    
stop = [0,0,0,0]
msg = Float32MultiArray(data=stop)
pub.publish(msg)