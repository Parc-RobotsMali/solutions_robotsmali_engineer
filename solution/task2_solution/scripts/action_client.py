#! /usr/bin/env python2

import rospy
import actionlib
from action_robot.msg import trajetGoal, trajetAction
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from traffict_light import detect_traffic
import cv2
cv_b = CvBridge()

etat = False
def feedback_cb(msg):
    print("trajet en cour %s"%msg)
    
def call_server():
    client = actionlib.SimpleActionClient('trajet', trajetAction)
    client.wait_for_server()

    goal = trajetGoal()
    
    route_x = [-2.138]
    route_y = [10.861]
    
    goal.traject_x, goal.traject_y = route_x, route_y

    client.send_goal(goal, feedback_cb=feedback_cb)

    client.wait_for_result()

    result = client.get_result()
    
    return result
def main(image):
    #print('start')
    global etat
    
    try:
        cv_image = cv_b.imgmsg_to_cv2(image, "bgr8")
        cv_image = cv2.resize(cv_image, (150, 200))
    except CvBridgeError as e:
        print(e)
    detect_traffic_ligth = detect_traffic(cv_image)
    if detect_traffic_ligth and not etat:
        print('verte detecte')
        call_server()
        etat = True
    else:
        print('route detection')


    

if __name__ == '__main__':
    try:
        rospy.init_node('action_client')
        rospy.Subscriber('/camera/color/image_raw', Image, main)
        rospy.spin()
        #result = call_server()

        #print(result)
    except rospy.ROSInterruptException as e:
        print(e)
