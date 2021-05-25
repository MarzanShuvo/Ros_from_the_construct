#! /usr/bin/env python

import rospy, math, numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

rospy.init_node('kinematic_controller', anonymous=True)

class VelocityController():
    def __init__(self, topic):
        self.cmd_vel = rospy.Publisher(topic, Twist, queue_size=10)
        rospy.sleep(0.1)

    def move(self, linear_velocity=0.0, angular_velocity=0.0):
        msg = Twist()
        msg.linear.x = linear_velocity
        msg.angular.z = angular_velocity
        self.cmd_vel.publish(msg)

class OdometryReader():
    def __init__(self, topic):
        self.pose = {}
        self.trajectory = []
        self.topic = topic
        self.subscribe()

    def callback(self, msg):
        self.pose['x'] = msg.pose.pose.position.x
        self.pose['y'] = msg.pose.pose.position.y
        self.trajectory.append((self.pose['x'], self.pose['y']))
        (_, _, self.pose['theta']) = euler_from_quaternion([msg.pose.pose.orientation.x, 
                                                            msg.pose.pose.orientation.y, 
                                                            msg.pose.pose.orientation.z, 
                                                            msg.pose.pose.orientation.w])
    def subscribe(self):
        self.subscriber = rospy.Subscriber(self.topic, Odometry, self.callback)
        rospy.sleep(0.1)

    def unregister(self):
        np.save('trajectory',self.trajectory)
        self.subscriber.unregister()

def normalize(angle):
    return np.arctan2(np.sin(angle),np.cos(angle))

def go_to(xg, yg, thetag_degrees, constant_vel = None):
    rho = float("inf")
    thetag = math.radians(thetag_degrees)
    while rho>0.01:
        dx = xg - odometry.pose['x']
        dy = yg - odometry.pose['y']
        rho = np.sqrt(dx**2 + dy**2)
        theta = odometry.pose['theta']
        alpha = normalize(np.arctan2(dy, dx) - theta)
        beta = normalize(thetag - np.arctan2(dy, dx))
        v = k_rho * rho
        w = k_alpha * alpha + k_beta * beta
        if constant_vel:
            abs_v = abs(v)
            v = v / abs_v * constant_vel
            w = w / abs_v * constant_vel
        velocity.move(v, w)
        rospy.sleep(0.01)

k_rho = 0.3
k_alpha = 0.8
k_beta = -0.15

from utilities import set_position
set_position(0,0,0)

velocity = VelocityController('/cmd_vel')
odometry = OdometryReader('/odom')

waypoints = [(1,-1,-90),(2,-2,0),(3,-2,0),(4,-1,90),(3.5,-0.5,180),
             (3,0,90),(3,1,90),(2,1,-90),(1,0,180),(0,0,180)]

for xg, yg, thetag in waypoints:
    go_to(xg, yg, thetag, constant_vel=0.3)
    
velocity.move(0,0)
odometry.unregister()
error = math.hypot(odometry.pose['x'], odometry.pose['y'])
print('Final positioning error is %.2fm' % error)