#! /usr/bin/env python

import rospy 
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import cmath

class Odom(object):

    def __init__(self, goal_x=0, goal_y=0,
                vitesse=0.45,
                topic="/odom"):
        
        rospy.Subscriber(topic, Odometry, self.odom)
        
        self._vitesse = vitesse
        # parametre de rotation
        self._yaw, self._roll, self._pitch = 0, 0, 0
        
        # coordonne du but 
        self._point = complex(goal_x, goal_y)
        
        # position du robot en cours
        self._pose = None

        self._kp = 0.3
        
    def odom(self, odom_msg):
        self.get_rotation(odom_msg)
    
    def get_data(self):

        return {
            "pose": self._pose,
            "orientation": self._yaw,
            "pose_goal": self._point,
        }

    def angle_regulator(self, target_rad):
        """
        Description:
        ------------
            cette fonction regule la rotation du robot en fonction
            de l'angle demander
        """
        return self._kp * (target_rad - self._yaw)
    
    def set_goal(self, x_cor, y_cor):
        """
        Description:
        ------------
            cette fonction permet de reactualiser la position
            du but

        Parametre:
        ----------
            x_cor : un entier
                il represente le coordonne x du nouveau but
            y_cor : un entier
                il represente le coordonne y du nouveau but
        """
        self._point = complex(x_cor, y_cor)

    def plaming(self, precision_rotation=0.01, precision_but=0.1 ):
        """
        Description:
        ------------
            cette fonction permet de planifier le mvt en fonction du but

        """
        if self._pose is not None:

            goal_pose = complex(
                                -(self._pose.x - self._point.real),\
                                -(self._pose.y - self._point.imag)
                                )
            
            # angle qui represente l'orientation du robot 
            # par rapport au but

            angle_but = cmath.phase(goal_pose)

            valeur_angle_regulateur = self.angle_regulator(angle_but)

            if abs(valeur_angle_regulateur) < precision_rotation:

                return [self._vitesse, 0]

            elif abs(goal_pose.real) < precision_but and abs(goal_pose.imag) < precision_but:
                return True

            else:
                return [0, valeur_angle_regulateur]
        
        else:
            return None

    
    def get_rotation (self, odom_msg):
        """
        Description:
        ------------
            cette fonction fournie la rotation et l'acceleration
            du robot dans le plan cartesian
        
        Parametre:
        ----------
            odom_msg : un object Odometry
                il represente le frange de Odometrie du robot

        """
        orientation_q = odom_msg.pose.pose.orientation
        self._pose = odom_msg.pose.pose.position

        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self._roll, self._pitch, self._yaw) = euler_from_quaternion (orientation_list)

if __name__ == '_main_':

    rospy.init_node("Odom_robot")
    
    odom = Odom()

    rospy.spin()