#! /usr/bin/env python

import rospy, math, numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

rospy.init_node('vacuum_cleaner', anonymous=True)

class VelocityController():
    def __init__(self, topic):
        self.cmd_vel = rospy.Publisher(topic, Twist, queue_size=10)
        self.max_vel = 0.65
        rospy.sleep(0.1)

    def move(self, linear_velocity=0.0, angular_velocity=0.0):
        abs_v = abs(linear_velocity)
        if abs_v <= self.max_vel:
            vx = linear_velocity
            wz = angular_velocity
        else:
            vx = linear_velocity / abs_v * self.max_vel
            wz = angular_velocity / abs_v * self.max_vel
        msg = Twist()
        msg.linear.x = vx
        msg.angular.z = wz
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
    return math.atan2(math.sin(angle),math.cos(angle))

def go_to(xg, yg, thetag_degrees):
    k_rho = 0.3
    k_alpha = 0.8
    k_beta = -0.15
    rho = float("inf")
    thetag = math.radians(thetag_degrees)
    while rho>0.01:
        dx = xg - odometry.pose['x']
        dy = yg - odometry.pose['y']
        rho = math.sqrt(dx**2 + dy**2)
        theta = odometry.pose['theta']
        alpha = normalize(math.atan2(dy, dx) - theta)
        beta = normalize(thetag - math.atan2(dy, dx))
        v = k_rho * rho
        w = k_alpha * alpha + k_beta * beta
        velocity.move(v, w)
        rospy.sleep(0.01)

def spiral(t):
    v = 0.65
    r = 0.1
    t0 = rospy.get_time()
    while rospy.get_time() - t0 < t:
        w = v / r
        velocity.move(v, w)
        r += 0.0015/r
        rospy.sleep(0.1)

from utilities import set_position, cleaned_area
set_position(0,0,0)
velocity = VelocityController('/cmd_vel')
odometry = OdometryReader('/odom')
start_time = rospy.get_time()

points = [(-2,0,-90),(2,0,0),(1.5,-1.5,-90),
          (1.5,-4.5,-90),(2,-8,-90),(-2,-8,180)]
times = [30, 30, 30, 30, 150, 150]
try:
    spiral(30)
    for point, time in zip(points,times):
        x, y, th = point
        go_to(x,y,th)
        spiral(time)
except KeyboardInterrupt:
    pass

end_time = rospy.get_time()    
velocity.move(0,0)
odometry.unregister()

t = end_time-start_time
m = int(t/60)
s = t - m*60
print("You cleaned %.2f m2 in %d minutes and %d seconds." % 
      (cleaned_area(odometry.trajectory), m, s))