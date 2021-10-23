#!/usr/bin/env python
# -*- coding: utf-8 -*-


import cv2
import argparse
from operator import xor
from rgb_camera_robot_sensor import RobotRGBSensors
import rospy

def callback(value):
    pass


def setup_trackbars(range_filter):
    cv2.namedWindow("Trackbars", 0)

    for i in ["MIN", "MAX"]:
        v = 0 if i == "MIN" else 255

        for j in range_filter:
            cv2.createTrackbar("%s_%s" % (j, i), "Trackbars", v, 255, callback)

def get_trackbar_values(range_filter):
    values = []

    for i in ["MIN", "MAX"]:
        for j in range_filter:
            v = cv2.getTrackbarPos("%s_%s" % (j, i), "Trackbars")
            values.append(v)

    return values


def main():
    rospy.init_node("hsv_detector", log_level=rospy.DEBUG)
    rospy.logwarn("Starting....")

    sensors_obj = RobotRGBSensors("/camera/rgb/image_raw")
    cv_image = sensors_obj.get_image()

    range_filter = "HSV"

    setup_trackbars(range_filter)

    while not rospy.is_shutdown():
        image = sensors_obj.get_image()

        frame_to_thresh = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        v1_min, v2_min, v3_min, v1_max, v2_max, v3_max = get_trackbar_values(range_filter)

        thresh = cv2.inRange(frame_to_thresh, (v1_min, v2_min, v3_min), (v1_max, v2_max, v3_max))

        preview = cv2.bitwise_and(image, image, mask=thresh)
        cv2.imshow("Preview", preview)

        if cv2.waitKey(1) & 0xFF is ord('q'):
            break

    rospy.logwarn("Shutting down")
    
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

