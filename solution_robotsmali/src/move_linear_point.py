#! /usr/bin/env python
# RobotsMali team Enginneer

"""
Description:
------------
    cet module est un server specialiser dans
    le mouvement du robot d'un point a un autre point
    sous forme de ligne droite.
Remarque:
---------
    il ne devrais pas avoir un object entre les deux point.
    le robot joint les deux points sous forme de ligne droite

"""
from solution_robotsmali.msg import (
                MoveLinearPointAction, MoveLinearPointFeedback,
                 MoveLinearPointResult, PoseRobot
                )

import rospy

import actionlib

from geometry_msgs.msg import Twist

from odom import Odom

class MoveLinearPoint(Odom):
    """
    // 
        La Classe MoveLinearPoint fournie un service
        d'action permettant de deplacer le robot d'un point et un autre
        et de publier la position du robot
    //
    
    """

    def __init__(self, name="MoveLinearPoint", goal_x=0, goal_y=0,
                vitesse=0.45, hz=10,
                topic="/odom", 
                velocite_topic="/cmd_vel",
                pose_robot="/pose_robot"):
        
        Odom.__init__(self, goal_x, goal_y, vitesse, topic )

        self._server = actionlib.SimpleActionServer(
                                                name, 
                                                MoveLinearPointAction, 
                                                execute_cb=self.run, auto_start=False
                                                )

        self._pub_cmd = rospy.Publisher(velocite_topic, Twist, queue_size=hz)

        self._pose_robot = rospy.Publisher(pose_robot, PoseRobot, queue_size=hz)

        self._command = Twist()

        self._rate = rospy.Rate(hz)
        self._retoure = MoveLinearPointFeedback()
        self._resultat = MoveLinearPointResult()
        self._server.start()

        

        rospy.loginfo("Demarage du service {}".format(name))


    def _move_robot(self, init=False):
        """
        Description:
        ------------
            cette fonction permet bouger un robot selon
            le planificateur de l'object Odom
        

        """

        planificateur = self.plaming()
        
        #print(planificateur)
        if planificateur is not None and planificateur is not True:

            # si planificateur non egal a None:
            # index 0 : acceleration
            # index 1 : rotation
            
            self._command.angular.z = planificateur[1]
            self._command.linear.x = planificateur[0]
        
        # si planificateur est vrai alors arrete le robot
        elif planificateur is True or init:

            self._command.linear.x = 0.0
            self._command.angular.z = 0.0
            self._pub_cmd.publish(self._command)
            return True
        # 
        else:
            self._command.angular.z = 0.0
            self._command.linear.x = 0.0
        
        self._pub_cmd.publish(self._command)
    
    def pose(self,):
        """
        Description:
        ------------
            cette fonction publier la position du robot
            sur le noeud /pose_robot sous forme:
            pose_robot:
                position_x : float
                    il represente la position du robot sur l'axe x
                position_y : float
                    il represente la position du robot sur l'axe y
                orientation: float
                    il represente l'orientation du robot sous
                    forme cartesian

        """
        robot_pub = PoseRobot()

        while not rospy.is_shutdown():
            try:
                robot_pose_data = self.get_data()

                robot_pub.position_x = robot_pose_data["pose"].x
                robot_pub.position_y = robot_pose_data["pose"].y

                robot_pub.orientation = robot_pose_data["orientation"]

                self._pose_robot.publish(robot_pub)
                
            except:
                pass

    def run(self, goal):
        """
        Description:
        ------------
            cette fonction permet de bouger le robots
            a partir de la position du robot jusqu'au
            but fournie comme parametre
        
        Parametre:
        ---------
            goal: 
        """
        succee = True

        self.set_goal(goal.position_but_x, goal.position_but_y)
        rospy.loginfo("Derramage du robot")

        while not rospy.is_shutdown():
            try:
                etat = self._move_robot()
                
                if etat is True:
                    break
                
                #if etat is  True: break
                get_robot = self.get_data()

                self._retoure.position_actuel_x = get_robot["pose"].x
                
                self._retoure.position_actuel_y = get_robot["pose"].y

                self._retoure.orientation_robot = get_robot['orientation']

                self._resultat.point_final_x.append(self._retoure.position_actuel_x)
                self._resultat.point_final_y.append(self._retoure.position_actuel_y)
                
                self._resultat.orientation_robot.append(self._retoure.orientation_robot)

                self._server.publish_feedback(self._retoure)
    
                self._rate.sleep()
            except:
                pass

        
        if succee:
            
            self._server.set_succeeded(self._resultat)

            self._move_robot(init=True)

            print("but atteind")
        
if __name__ == '__main__':
    rospy.init_node("move")

    move_server = MoveLinearPoint()

    move_server.pose()

    rospy.spin()