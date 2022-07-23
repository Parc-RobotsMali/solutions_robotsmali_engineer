#! /usr/bin/env python

# RobotsMali team Enginneer

"""
Description:
------------
    cet module est specialiser dans mouvement du robot
    en parcourant une certaine distance, comme nous avons la
    posibiliter de dire au robot de parcourir que 2m puis s'arrete
    ou bien de faire une rotation d'un angle precis en radians

"""
import cmath
import rospy
import actionlib
from solution_robotsmali.msg import (
    MoveLinearPointAction, MoveLinearPointGoal, PoseRobot
)
from solution_robotsmali.srv import Vitesse, Rotation, RotationResponse


from odom import Odom
from geometry_msgs.msg import Twist

class MoveRobot(Odom):

    def __init__(
                self,
                move_service="MoveLinearPoint",
                name_rotation="rotation_bot",
                name_acceleration="acceleration_bot",
                pose_robot="/pose_robot",
                cmd_topic="/cmd_vel",
                hz=10,
                ):
        
        Odom.__init__(self, )

        self._server_rotation = rospy.Service(name_rotation, Rotation, self.rotation )

        self._server_acceleration = rospy.Service(name_acceleration, Vitesse, self.acceleration)

        self._pose_robot = rospy.Subscriber(pose_robot, PoseRobot, self.pose)

        self._pub_cmd = rospy.Publisher(cmd_topic, Twist, queue_size=hz)

        self._pose = None
        
        self._client = actionlib.SimpleActionClient(
                move_service,
                MoveLinearPointAction,
                
        )

        self._client.wait_for_server()

        rospy.loginfo("Demarage du service en cours...")

    def rotation(self, angle_goal, precision=0.01):
        """
        Description:
        ------------
            cette fonction de tourner le robot
            a un angle precis
        
        Parametre:
        ----------
            angle_goal: un float
                il represente l'angle de rotation
                demander
        """
        twist = Twist()
        #print(angle_goal)

        while not rospy.is_shutdown():
            angle_regulateur = self.angle_regulator(angle_goal.angle)
            if abs( angle_regulateur) < precision:
                twist.angular.z = 0.0
                twist.linear.x = 0.0

                break
            else:
                twist.angular.z = angle_regulateur
                twist.linear.x =0.0
                self._pub_cmd.publish(twist)
        
        self._pub_cmd.publish(twist)
     
        return RotationResponse(angle_goal.angle)
    
    def pose(self, pose_msg):

        self._pose = pose_msg


    def acceleration(self, distance_goal):
        """
        Description:
        ------------
            cette fonction represente permet de faire bouger le robot
            a une distance precis
        
        Parametre:
        ----------
            distance_goal : un float
                il represente la distance a parcourir
        
        """
        goal = MoveLinearPointGoal()

        if self._pose is not None:
            print(self._pose)
            position_distance = cmath.rect(distance_goal.longueur, self._pose.orientation)
            goal.position_but_x = self._pose.position_x + position_distance.real
            goal.position_but_y = self._pose.position_y + position_distance.imag

            self._client.send_goal(goal)
            return self._client.wait_for_result()
            
        else:
            return self._client.wait_for_result()
        

if __name__ == '__main__':
    rospy.init_node("MoveRobot")
    move_robot = MoveRobot()

    rospy.spin()