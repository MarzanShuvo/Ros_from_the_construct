#!/usr/bin/env python
import sys
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from move_robot import MoveKobuki

class StarFollower(object):

    def __init__(self, star_color):
    
        self._star_color = star_color

        # name : {"upper":[H,S,V], "lower":[H,S,V]}
        self.star_HSV_ranges_colour_db = {
                            "green":{"upper":[64,255,255],"lower":[45,142,0]},
                            "red":{"upper":[0,255,255],"lower":[0,185,0]},
                            "blue":{"upper":[111,255,255],"lower":[104,134,0]},
                            "yellow":{"upper":[34,255,255],"lower":[23,98,0]},
                                }

        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback)
        self.movekobuki_object = MoveKobuki()

    def get_range_hsv_color(self, color_name):
        if color_name in self.star_HSV_ranges_colour_db:
            ranges_dict = self.star_HSV_ranges_colour_db[color_name]
            upper = ranges_dict["upper"]
            lower = ranges_dict["lower"]

            upper_color = np.array([upper[0],upper[1],upper[2]])
            lower_color = np.array([lower[0],lower[1],lower[2]])
            
            return upper_color, lower_color
        else:
            return None, None

    def update_star_color(self, new_color):
        self._star_color = new_color

    def camera_callback(self,data):
        
        try:
            # We select bgr8 because its the OpneCV encoding by default
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
            
        height, width, channels = cv_image.shape
        # Noetic integer conversion
        height = int(height)
        width = int(width)
        channels = int(channels)
        descentre = 0
        rows_to_watch = 200

        aux1 = int(((height)/2)+descentre)
        aux2 = int((height)/2+(descentre+rows_to_watch))

        crop_img = cv_image[aux1:aux2][1:width]
        
        # Convert from RGB to HSV
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        
        upper_hsv_color , lower_hsv_color = self.get_range_hsv_color(self._star_color)
        if upper_hsv_color is not None and lower_hsv_color is not None:
            # Threshold the HSV image to get only yellow colors
            mask = cv2.inRange(hsv, lower_hsv_color, upper_hsv_color)
            
            # Calculate centroid of the blob of binary image using ImageMoments
            m = cv2.moments(mask, False)
            try:
                cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
            except ZeroDivisionError:
                cy, cx = height/2, width/2
            
            
            # Bitwise-AND mask and original image
            res = cv2.bitwise_and(crop_img,crop_img, mask= mask)
            
            # Draw the centroid in the resultut image
            # cv2.circle(img, center, radius, color[, thickness[, lineType[, shift]]]) 
            cv2.circle(res,(int(cx), int(cy)), 10,(0,0,255),-1)

            cv2.imshow("Original", cv_image)
            cv2.imshow("HSV", hsv)
            cv2.imshow("MASK", mask)
            cv2.imshow("RES", res)
            
            cv2.waitKey(1)
            
            
            error_x = cx - width / 2
            twist_object = Twist()
            twist_object.linear.x = 0.2
            twist_object.angular.z = -error_x / 100
            rospy.loginfo("ANGULAR VALUE SENT===>"+str(twist_object.angular.z))
            # Make it start turning
            self.movekobuki_object.move_robot(twist_object)
        else:
            print("Colour not in database=="+str(self._star_color))
        
    def clean_up(self):
        self.movekobuki_object.clean_class()
        cv2.destroyAllWindows()
        
        

def main():
    rospy.init_node('star_following_node', anonymous=True)
    
    arguments = sys.argv

    if len(arguments) > 1:
        color_name = arguments[1]
    else:
        print("Use: python follow_star_step_hsv color_name[red,green,blue or yellow]")
        color_name = "yellow"
    
    star_follower_object = StarFollower(star_color=color_name)
   
    rate = rospy.Rate(5)
    ctrl_c = False
    def shutdownhook():
        # works better than the rospy.is_shut_down()
        star_follower_object.clean_up()
        rospy.loginfo("shutdown time!")
        ctrl_c = True
    
    rospy.on_shutdown(shutdownhook)
    
    while not ctrl_c:
        rate.sleep()

    
    
if __name__ == '__main__':
    main()