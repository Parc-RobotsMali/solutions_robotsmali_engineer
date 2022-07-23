#! /usr/bin/env python
#coding=utf-8
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge,CvBridgeError
from sensor_msgs.msg import Image
import matplotlib.pyplot as plt
red_green_range_lower = np.array([0, 200, 215])
red_green_range_upper = np.array([100, 255,255])

green_lower_range = np.array([50,100,100])
green_upper_range = np.array([70,255,255])

#couleur rouge et verte en même temps
red_lower_range = np.array([0,100,200])
red_upper_range = np.array([0,255,255])
cv_b = CvBridge()
def IMAGE(msg) :
    try :
        image = cv2.imread("/home/panga/catkin_ws/src/detection_objet_avec_camera/script/dt.jpg")
        image1 = cv2.imread("/home/panga/catkin_ws/src/detection_objet_avec_camera/script/dv.jpg")
        cv_image_x = image1.shape[1]
        cv_image_y = image1.shape[0]
        cv_image = cv_b.imgmsg_to_cv2(msg,"bgr8")
        cv_image_ros = cv2.resize(cv_image,(cv_image_x,cv_image_y))
    except CvBridgeError as e :
        print(e)
    def masked(ima) :
        mask = np.zeros(ima.shape[:2],dtype="uint8")
        cv2.rectangle(mask,(334,211),(392,260),(255,255,255),-5)
        masked_image = cv2.bitwise_and(ima,ima,mask=mask)
        return masked_image

    def get_image_mask(image, mask_range_min, mask_range_max) :
        hls = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
        return cv2.inRange(hls, mask_range_min, mask_range_max)
    def simi(cv_ima,ima1,ima2,mask1,mask2,cv_mask) :
        cv_image_bins = 40
        image1_bins = 60
        image2_bins = 50
        histSize = [cv_image_bins,image1_bins, image2_bins]
        cv_image_ranges = [0,128]
        image1_ranges = [0, 256]
        image2_ranges = [0, 256]
        ranges =  cv_image_ranges + image1_ranges + image2_ranges
        channels = [0,1]
        cv_hist = cv2.calcHist([cv_ima],channels,cv_mask,histSize, ranges, accumulate=False)
        cv2.normalize(cv_hist, cv_hist, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX)
        hist_image1 = cv2.calcHist([ima1], channels,mask1,histSize, ranges, accumulate=False)
        cv2.normalize(hist_image1, hist_image1, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX)
        hist_image2 = cv2.calcHist([ima2], channels, mask2, histSize, ranges, accumulate=False)
        cv2.normalize(hist_image2, hist_image2, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX)

        compare_method = cv2.HISTCMP_CORREL
        cv_image_simi = cv2.compareHist(cv_hist, cv_hist, compare_method)
        image1_simi = cv2.compareHist(cv_hist, hist_image1, compare_method)
        image2_simi = cv2.compareHist(cv_hist, hist_image2, compare_method)

        print('cv_image_simi Similarity = ', cv_image_simi)
        print('image1_simi Similarity = ', image1_simi)
        print('image2_simi Similarity = ', image2_simi)


    sim1 = masked(image)
    sim2 = masked(image1)
    ros_ = masked(cv_image_ros)
    green = get_image_mask(sim1,green_lower_range,green_upper_range)
    red = get_image_mask(sim2,red_lower_range,red_upper_range)
    green_or_red = get_image_mask(ros_,red_green_range_lower,red_green_range_upper)
    sim1_1 = cv2.cvtColor(image,cv2.COLOR_RGB2HSV)
    sim2_2 = cv2.cvtColor(image1,cv2.COLOR_RGB2HSV)
    sim3_3 = cv2.cvtColor(cv_image_ros,cv2.COLOR_RGB2HSV)
    cv2.imshow("ros",green_or_red)
    cv2.waitKey(0)
    simi(sim3_3,sim1_1,sim2_2,mask1=green,mask2=red,cv_mask=green_or_red)
if __name__=="__main__" :
    rospy.init_node("image_process")
    rospy.Subscriber("camera/color/image_raw",Image,IMAGE)
    rospy.spin()