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
    
    #task3
    route_x = [4.340, 2.917, 2.646, 7.207]
    route_y = [6.819, 3.745, -2.744, -6.945]
    
    goal.traject_x, goal.traject_y = route_x, route_y

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
