import numpy as np
import cv2
import matplotlib.pyplot as plt
green_lower_range = np.array([50,100,100])
green_upper_range = np.array([70,255,255])

red_lower_range = np.array([0,100,200])
red_upper_range = np.array([0,255,255])

image = cv2.imread("rouge.png")
image1 = cv2.imread("verte.png")

def simi(ima1,ima2,mask1,mask2) :
    image1_bins = 60
    image2_bins = 50
    histSize = [image1_bins, image2_bins]
    image1_ranges = [0, 128]
    image2_ranges = [0, 256]
    ranges = image1_ranges + image2_ranges
    channels = [0,1]
    hist_image1 = cv2.calcHist([ima1], channels,mask1,histSize, ranges, accumulate=False)
    #cv2.imshow("ss",hist_image1)
    cv2.normalize(hist_image1, hist_image1, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX)
    hist_image2 = cv2.calcHist([ima2], channels, mask2, histSize, ranges, accumulate=False)
    cv2.normalize(hist_image2, hist_image2, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX)

    compare_method = cv2.HISTCMP_CORREL

    image1_simi = cv2.compareHist(hist_image1, hist_image1, compare_method)
    image2_simi = cv2.compareHist(hist_image1, hist_image2, compare_method)

    print('image1_simi Similarity = ', image1_simi)
    print('image2_simi Similarity = ', image2_simi)

def masked(ima) :
    mask = np.zeros(ima.shape[:2],dtype="uint8")
    cv2.rectangle(mask,(334,211),(392,260),(255,255,255),-5)
    masked_image = cv2.bitwise_and(ima,ima,mask=mask)
    return masked_image

def get_image_mask(image, mask_range_min, mask_range_max) :
    hls = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    return cv2.inRange(hls, mask_range_min, mask_range_max)

sim1 = masked(image)
sim2 = masked(image1)
cv2.imshow("cc", sim1)
cv2.imshow("4c", sim2)
#cv2.waitKey(0)
green = get_image_mask(sim1,green_lower_range,green_upper_range)
red = get_image_mask(sim2,red_lower_range,red_upper_range)
cv2.imshow("i",green)
cv2.imshow("a",red)
sim1_1 = cv2.cvtColor(image,cv2.COLOR_RGB2HSV)
sim2_2 = cv2.cvtColor(image1,cv2.COLOR_RGB2HSV)
simi(sim1,sim2,mask1=green,mask2=red)
cv2.waitKey(0)