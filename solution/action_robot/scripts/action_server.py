#! /usr/bin/env python

import rospy
import actionlib
from action_robot.msg import trajetResult, trajetAction, trajetFeedback

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from tf.transformations import euler_from_quaternion

import cmath


# les regulateur de la rotation

roll = pitch = yaw = 0.0
kp=0.3

# les constante de la vitesse et de rotation
vitesse = 0.45

pose = None

def get_rotation (msg):

    global roll, pitch, yaw, pose
    orientation_q = msg.pose.pose.orientation
    pose = msg.pose.pose.position

    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)


def angle_regulator(target_rad):
    """
    Description:
    ------------
        cette fonction regule la rotation du robot en fonction
        de l'angle demander
    """

    return kp * (target_rad-yaw)

def planing_min(goal_pose, preci=0.01, preci_but=0.1):
    """
    Description:
    ------------
        cette fonction calcul la vitesse ou la
        rotation necessaire pour que le robot soit
        droite au but
    
    Parametre:
    ---------
        goal_pose : un nombre complexe
            il represente le point a atteindre
        preci : un float
            il represente la precision d'inclinaision
            necessaire pour que le robot soit droite au but
    
    Retoure:
    --------
        une liste :
            indexe : 0 -> la vitesse
            indexe : 1 -> la rotation
    """

    #calcul d'angle antre le robot et le but
    angle = cmath.phase(goal_pose)
    
    # calcul du taux de rotation necessaire pour etre droite au but
    regulator = angle_regulator(angle)
   
    # si la precision du but a est plus petit que 0.1
    # arrete le robot
    
    if abs(goal_pose.real) < preci_but and abs(goal_pose.imag) < preci_but:
        
        return [0.0, 0.0]

    elif abs(regulator) < preci:

        return [vitesse, 0.0]

    else:
        return [0.0, regulator]
    

class ActionServer():
    def __init__(self):

        self.a_server = actionlib.SimpleActionServer("trajet", trajetAction, execute_cb=self.execute, auto_start=False)
        self.a_server.start()

        self.pub_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/odom', Odometry, get_rotation)

        self.msg_vel = Twist()

    def execute(self, goal):

        rate = rospy.Rate(10)

        success = True
        
        feedback = trajetFeedback()
        
        result = trajetResult()

        for x_cor, y_cor in zip(goal.traject_x, goal.traject_y):

            if self.a_server.is_preempt_requested():
                self.a_server.set_preempted()
                success = False
                break

            pt = complex(x_cor, y_cor)
                
            while True:
                # translation entre le point du but et du robot de
                # sorte que le point d'origine soit le robot a chaque fois
                if pose is not None:
                    goal_pose = complex(-(pose.x - pt.real), -(pose.y - pt.imag))
                    
                    cm_vel = planing_min(goal_pose)

                    rospy.loginfo("commande_velocite : %s" % cm_vel)
                    #rospy.loginfo('position : %s' % pose)

                    if cm_vel[0] != 0.0 or cm_vel[1] != 0.0:

                        self.msg_vel.angular.z = cm_vel[1]
                        self.msg_vel.linear.x = cm_vel[0]
                        self.pub_cmd.publish(self.msg_vel)
                        rate.sleep()

                        continue
                    #print(cm_vel)
                    break
            
            last_trajet = 'trajet x %s y =%s' %(str(x_cor), str(y_cor))

            feedback.point = last_trajet

            result.atteint.append(last_trajet)

            self.a_server.publish_feedback(feedback)
            
            rate.sleep()
        
        if success:

            self.a_server.set_succeeded(result)
            self.msg_vel.angular.z = 0
            self.msg_vel.linear.x = 0

            self.pub_cmd.publish(self.msg_vel)
            
            rospy.loginfo("But atteind ")

if __name__== '__main__':
    
    rospy.init_node('action_trajet')
    
    s = ActionServer()
    
    rospy.spin()
