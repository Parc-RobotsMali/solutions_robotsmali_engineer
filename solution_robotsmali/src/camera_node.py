#! /usr/bin/env python

# RobotsMali team Enginneer 

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

import cv2

class Camera(object):
    def __init__(self, 
                camera_topic="/zed_multi/front/zed_nodelet_front/left/image_rect_color",
                pub_topic="/filtre_camera",
                hz=10):
        
        rospy.Subscriber(camera_topic, Image, self.camera)

        self.cvBrideObect = CvBridge()

        self._image = None
        self._rate = rospy.Rate(hz)
        
        # _inc increment chaque lecture de l'image
        # pour n'est pas lire deux fois le meme image
        self._inc = 0

    def camera(self, camera_msg):
        """
        Description:
        ------------
            cette fonction recupurer les images de la camera
            pour les

        """
        
        self._image = camera_msg
    
    def affiche(self):
        
        while not rospy.is_shutdown():
            if self._image is not None:
                if self._image.header.seq != self._inc:
                    self._inc = self._image.header.seq
                
                    try:
                        image = self.cvBrideObect.imgmsg_to_cv2(self._image, "bgr8")
                        print(self._image.header)
                    except CvBridgeError as e:
                        print(e)
                    cv2.imshow("fenetre", image)
                    cv2.waitKey(1)
                    #self._rate.sleep()
    
    def get_image(self):
        if self._image is not None:
                if self._image.header.seq != self._inc:
                    self._inc = self._image.header.seq
                    return self.cvBrideObect.imgmsg_to_cv2(self._image, "bgr8")
        return None

    def mask(min_range, max_range):
        pass


if __name__ == '__main__':
    rospy.init_node("test")
    can = Camera()
    can.affiche()
    rospy.spin()
