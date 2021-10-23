#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image


class LineFollower(object):

    def __init__(self, star_color="yellow", go_right=True):

        self._star_color = star_color
        self._go_right = go_right
        self.detection_state_on = False

        # name : {"upper":[H,S,V], "lower":[H,S,V]}
        self.star_HSV_ranges_colour_db = {
                            "green":{"upper":[64,255,255],"lower":[45,142,0]},
                            "red":{"upper":[0,255,255],"lower":[0,185,0]},
                            "blue":{"upper":[111,255,255],"lower":[104,134,0]},
                            "yellow":{"upper":[34,255,255],"lower":[23,98,0]},
                                }

        self.current_twist = Twist()
    
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback)
        

    def get_current_twist(self):
        return self.current_twist

    def update_turning_tendency(self,go_right):
        self._go_right = go_right

    def get_range_hsv_color(self, color_name):
        #rospy.loginfo("Getting HSV for Color=="+str(color_name))

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
    
    def set_detection_state(self,is_on):
        self.detection_state_on = is_on

    def camera_callback(self,data):
        
        if self.detection_state_on:
            try:
                # We select bgr8 because its the OpneCV encoding by default
                cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
            except CvBridgeError as e:
                print(e)
                
            # We get image dimensions and crop the parts of the image we dont need
            # Bear in mind that because the first value of the image matrix is start and second value is down limit.
            # Select the limits so that it gets the line not too close and not too far, and the minimum portion possible
            # To make the process faster.
            height, width, channels = cv_image.shape
            # Noetic integer conversion
            height = int(height)
            width = int(width)
            channels = int(channels)
            descentre = 160
            rows_to_watch = 20

            aux1 = int(((height)/2)+descentre)
            aux2 = int((height)/2+(descentre+rows_to_watch))

            crop_img = cv_image[aux1:aux2][1:width]
            
            # Convert from RGB to HSV
            hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
            
            upper_hsv_color , lower_hsv_color = self.get_range_hsv_color(self._star_color)
            if upper_hsv_color is not None and lower_hsv_color is not None:
            
                # Threshold the HSV image to get only yellow colors
                mask = cv2.inRange(hsv, lower_hsv_color, upper_hsv_color)
                
                # Bitwise-AND mask and original image
                res = cv2.bitwise_and(crop_img,crop_img, mask= mask)
                
                contours, _, = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_TC89_L1)
                #rospy.loginfo("Number of centroids==>"+str(len(contours)))
                centres = []
                for i in range(len(contours)):
                    moments = cv2.moments(contours[i])
                    try:
                        centres.append((int(moments['m10']/moments['m00']), int(moments['m01']/moments['m00'])))
                        cv2.circle(res, centres[-1], 10, (0, 255, 0), -1)
                    except ZeroDivisionError:
                        pass
                    
                
                # rospy.loginfo(str(centres))
                #Select the right centroid
                # [(542, 39), (136, 46)], (x, y)
                selected_centroid_index = 0
                index = 0
                max_x_value = 0
                min_x_value = width
                for candidate in centres:
                    # Retrieve the cx value
                    cx = candidate[0]
                    if self._go_right:
                        # Get the Cx more to the right
                        if cx >= max_x_value:
                            max_x_value = cx
                            selected_centroid_index = index
                    else:
                        # Get the Cx more to the left
                        if cx <= min_x_value:
                            min_x_value = cx
                            selected_centroid_index = index
                    index += 1
                
                
                try:
                    cx = centres[selected_centroid_index][0]
                    cy = centres[selected_centroid_index][1]
                    #rospy.logwarn("Winner =="+str(cx)+","+str(cy)+"")

                    # Draw the centroid in the resulting image
                    # cv2.circle(img, center, radius, color[, thickness[, lineType[, shift]]]) 
                    cv2.circle(res,(int(cx), int(cy)), 5,(0,0,255),-1)
                    
                    

                    cv2.imshow("Original_"+str(self._star_color), cv_image)
                    #cv2.imshow("HSV", hsv)
                    #cv2.imshow("MASK", mask)
                    cv2.imshow("RES_"+str(self._star_color), res)
                    
                    cv2.waitKey(1)
                    
                    
                    error_x = cx - width / 2
                    twist_object = Twist()
                    twist_object.linear.x = 0.2
                    twist_object.angular.z = -error_x / 100
                    # rospy.loginfo("ANGULAR VALUE SENT===>"+str(twist_object.angular.z))
                    self.current_twist = twist_object

                except:
                    #cy, cx = height/2, width/2

                    cv2.imshow("Original", cv_image)
                    #cv2.imshow("HSV", hsv)
                    #cv2.imshow("MASK", mask)
                    cv2.imshow("RES", res)
                    
                    cv2.waitKey(1)

                    twist_object = Twist()
                    rospy.logwarn("NO BLOB Found===>")
                    self.current_twist = twist_object
                
                
            
            else:
                rospy.logerr("Color To follow not supported, stopping Robot="+str(self._star_color))
                twist_object = Twist()
                self.current_twist = twist_object
            
        else:
            twist_object = Twist()
            self.current_twist = twist_object
        
        
        
        
    def clean_up(self):
        cv2.destroyAllWindows()
        
        

def main():
    rospy.init_node('line_following_node', anonymous=True)
    
    
    line_follower_object = LineFollower()

    rate = rospy.Rate(5)
    ctrl_c = False
    def shutdownhook():
        # works better than the rospy.is_shut_down()
        line_follower_object.clean_up()
        rospy.loginfo("shutdown time!")
        ctrl_c = True
    
    rospy.on_shutdown(shutdownhook)
    
    while not ctrl_c:
        rate.sleep()

    
    
if __name__ == '__main__':
    main()