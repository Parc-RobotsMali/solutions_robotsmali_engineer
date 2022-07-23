#! /usr/bin/env python

# RobotsMali team Enginneer

import cmath
import math
import rospy
import time

from solution_robotsmali.srv import (
    Vitesse, Rotation
)


from solution_robotsmali.msg import (
    PoseRobot, MoveLinearPointAction,
)

import actionlib

class ClientMoveRobot(object):
    def __init__(self,
                name_rotation="/rotation_bot",
                name_acceleration="/acceleration_bot",
                pose_topic="/pose_robot",
                move_linear="MoveLinearPoint",
                hz=10,
                ):

        rospy.wait_for_service(name_rotation)

        rospy.wait_for_service(name_acceleration)



        #rospy.wait_for_service(move_linear)

        self._server_rotation = rospy.ServiceProxy(name_rotation, Rotation)

        self._server_acceleration = \
            rospy.ServiceProxy(name_acceleration, Vitesse)
        
        self._move_linear = actionlib.SimpleActionClient(move_linear, MoveLinearPointAction)
        
        
        rospy.Subscriber(pose_topic, PoseRobot, self.pose_robot)
        
        # self._pose contient la position et l'orientation
        # du robot, None au demarrage
        self._pose = None

        # cette variable contient les intervale des angles disponible
        

    def rotation_angle(self, angle=cmath.pi/4):
        """
        Description:
        ------------
            cette fonction permet de touner le robot d'un angle
            speficique fournie en parametre
        
        Parametre:
        ----------
            angle: un float
                il represente l'angle dont le robot doit tourner
        """

        if self._pose is not None:

            self.rotation(self._pose.orientation + angle)
        
    def laser_filtre(self, laser_filtre_msg):
        """
        Descriprion:
        -------------
            cette foncion permet donne les angles disponible

        """

        self._laser_filtre_data = laser_filtre_msg

    def pose_robot(self, pose_msg):
        """
        DEscription:
        ------------
            cette fonction permet de fournir les coordonnes du
            robot ( sa position et son orientation)
        
        Parametre:
        ----------
            pose_msg
        """
        self._pose = pose_msg

    def rotation(self, angle):
        """
        Description:
        ------------
            cette fonction envoie la commande de rotation a
            move_goal
        
        Parametre:
        ----------
            angle : un float
                il represente l'angle a tournee

        """
        data_angle = self._server_rotation(angle)

        return data_angle.reponse

    

    def acceleration(self, longueur):
        """
        Description:
        ------------
            cette fonction envoie le commande
            a move_goal
        
        Parametre:
        ----------
            longueur : un float
                il represente la longueur a parcourir
        """

        data_longueur = self._server_acceleration(longueur)

        return data_longueur.reponse
    
    def main(self, trajets, flags=False):
        """
        Description:
        ------------
            cette fonction bouge le robot d'un point de depart
            a un point d'arriver en considerant le trajet comme
            des series de distance a parcourir et de rotation a
            faire

        Parametre:
        ----------
            trajets : une liste
                il represente la distance a parcourir et de rotation
                a faire.
                index 0 : la distance
                index 1 : la rotations en radians
            nbre :
        """
        time.sleep(2)
        

        if flags is not False :

            print("Test n: ", flags, )

            self.trajet_goal(trajets[flags][0], trajets[flags][1])
            
        
        else:
            for index, trajet in enumerate(trajets):

                print("trajet n: ", index )

                self.trajet_goal(trajet[0], trajet[1])
            
    def trajet_goal(self, longueur, angle=False, ):
        """
        """
        if angle is not False:
            angle_re = angle
            print(angle)
            if angle_re > cmath.pi *2:
                angle_re = angle_re -  cmath.pi
            
            elif angle_re < -cmath.pi * 2:
                angle_re = angle_re + cmath.pi 
            else:
                self.acceleration(longueur)
                self.rotation_angle(angle_re)


        else:
            self.acceleration(longueur)
            

    
if __name__ == '__main__':
    rospy.init_node("client_move_robot", anonymous=True)
    
    client = ClientMoveRobot()

    #definition des trajets a parcourir
    # index 0: la distance
    # index 1: la rotation
    trajets = [ 
        [4.15, -math.radians(88)],
        #[5.3, -math.radians(88)],
        #[2.72, -math.radians(88) ],
        #[0.5, -math.radians(45)],
        #[0.9, math.radians(90)],
        #[ 1, False],
        ]
        
    client.main(trajets)
    

    
