#! /usr/bin/env python2

import rospy
import actionlib
from action_robot.msg import trajetGoal, trajetAction

def feedback_cb(msg):
    print("trajet en cour %s"%msg)
    
def call_server():
    client = actionlib.SimpleActionClient('trajet', trajetAction)
    client.wait_for_server()

    goal = trajetGoal()
    
    #route2
    route_deux_x = [-20.942, -20.365, -12.523, -12.445]
    route_deux_y = [-10.975, -11.157, -10.920, 10.913]
    
    #route1
    route_un_x = [-30.716, -30.1325, -20.887, -12.3418]
    route_un_y = [6.09, 10.905 ,10.87 , 10.939]
    
    goal.traject_x, goal.traject_y = route_un_x, route_un_y

    client.send_goal(goal, feedback_cb=feedback_cb)

    client.wait_for_result()

    result = client.get_result()
    
    return result

if __name__ == '__main__':
    try:
        rospy.init_node('action_client')
        
        result = call_server()

        print(result)
    except rospy.ROSInterruptException as e:
        print(e)
