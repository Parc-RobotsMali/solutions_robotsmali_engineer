#! /usr/bin/env python

import rospy
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
#from shape import forme

cv_b = CvBridge()
mn = np.array([0,0,120])
ma = np.array([0,0,130])

def ImageProcessing(image):

    try:
        cv_image = cv_b.imgmsg_to_cv2(image, "bgr8")
    except CvBridgeError as e:
        print(e)
    #img = forme(cv_image)
    cv2.imshow('forme', cv_image)
    cv2.waitKey(1)
    """hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, mn, ma)
    cv2.imshow('original',cv_image)
    cv2.imshow('mask', mask)
    cv2.waitKey(1)"""

if __name__ == '__main__':
    rospy.init_node('image_process')

    rospy.Subscriber('/camera/color/image_raw', Image, ImageProcessing)
    
    rospy.spin()

