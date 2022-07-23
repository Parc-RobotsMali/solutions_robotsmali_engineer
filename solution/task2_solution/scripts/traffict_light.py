import rospy
import cv2 
import numpy as np 

green_lower_range = np.array([5,40,200])
green_upper_range = np.array([75,255,255])

red_lower_range = np.array([0,100,200])
red_upper_range = np.array([0,255,255])

kernel = np.ones((3, 3), np.uint8)

def get_image_mask(image, mask_range_min, mask_range_max) :

    hls = cv2.cvtColor(image,cv2.COLOR_BGR2HLS)
  
    return cv2.inRange(hls, mask_range_min, mask_range_max)


def min_y(image, x, y):
	#print(x, y)
	for i in range(y -1):
		for t in range(x -1):
			#print(image.shape)
			if image[t, i] == 255:
				return i
 
def detect_traffic_pose(image_red, image_green):
	green_shape = image_green.shape
	red_shape = image_red.shape
	green = min_y(image_green, green_shape[0], green_shape[1])
	red = min_y(image_red, red_shape[0], red_shape[1])
	
	if green is None:
		return 0
	return 1
def detect_traffic(image):


	green_mask = get_image_mask(image, 
					green_lower_range, 
						green_upper_range )

	red_mask = get_image_mask(image, 
					red_lower_range, 
						red_upper_range )

	green_mask = cv2.erode(green_mask, kernel)
	red_mask = cv2.erode(red_mask, kernel)
	
	etat = detect_traffic_pose(red_mask, green_mask)

	return etat


"""cv2.imshow("image", image)

cv2.imshow("green", green_mask)
cv2.imshow("red", red_mask)

result = detect_traffic_pose(red_mask, green_mask)
print(result)
cv2.waitKey(0)"""

