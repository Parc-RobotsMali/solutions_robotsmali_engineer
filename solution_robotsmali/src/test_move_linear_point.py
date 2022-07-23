#! /usr/bin/env python

import rospy
from odometrie.msg import MoveLinearPointGoal, MoveLinearPointAction
import actionlib

def affiche(msg):
    print(msg)
def main():
    client = actionlib.SimpleActionClient("move_linear_point", MoveLinearPointAction)
    client.wait_for_server()
    go = MoveLinearPointGoal()
    go.position_but_x , go.position_but_y = 1, 1
    #goal.goal.position_but_x = 0.0
    #goal.goal.position_but_y = 0.0
    client.send_goal(go, feedback_cb=affiche)
    client.wait_for_result()

    return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node("test", anonymous=True)
        print(main())
    except rospy.ROSInterruptException as e:
        print(e) 
    #rospy.spin()
