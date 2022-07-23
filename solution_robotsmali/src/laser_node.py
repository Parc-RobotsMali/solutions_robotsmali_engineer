#! /usr/bin/env python

import rospy
import cmath

from solution_robotsmali.srv import AngleVoid, AngleVoidResponse

from sensor_msgs.msg import LaserScan

from solution_robotsmali.msg import ObjectVoid

class Laser(object):
    def __init__(
                self, topic="/scan", 
                hz=10,
                filtre_topic="/laser_filtre",
                angle_void_topic="/angle_void"
                ):
        
        self._pub_laser_filtre = rospy.Publisher(filtre_topic, ObjectVoid, queue_size=hz)
        # subcription au noeud du scanner laser
        rospy.Subscriber(topic, LaserScan, self.laser)

        self._server = rospy.Service(angle_void_topic, AngleVoid, self.angle_filtre)
        
        

        self._laser_msg = None

        
        self._rate = rospy.Rate(10)

    def angle_filtre(self, angle_range):
        """
        Description:
        ------------
            cette fonction permet de founir le module
            le plus petit pour une diffrence d'angle
        
        """
        if self._laser_msg is not None and angle_range.angle_max > angle_range.angle_min:

            data_angle_range = min(self._laser_msg.ranges[int(angle_range.angle_min): \
                int(angle_range.angle_max)])
            
            return AngleVoidResponse(data_angle_range)
        
        else:

            return AngleVoidResponse(0.0)

    def laser(self, msg_laser):

        self._laser_msg = msg_laser



    def run_filtre(self):
        """
        
        """
        while not rospy.is_shutdown():
            try:
                data_filtre = self.laser_filter_2()

                range_angle_laser = self.fusion_all(data_filtre)

                self._pub_laser_filtre.publish(range_angle_laser)
                self._rate.sleep()
            except:
                pass


    def laser_filter(self, precision=float('inf')):
        """
        Description:
        ------------
            cette fonction detecte les angles disponible
            pour un module superieur ou egale a la precision
        
        Parametre:
        ----------
            laser_msg : un object LaserScan
                il represente le bayalage du frame laser
            precsion: un float 
                il represente la precision avec la quelle 
                l'angle sera conserver ou rejeter
        
        Retoure:
        --------
            une liste: contenant tout les angles que le
            robot est capable de traverser

        """
        angle_void = []

        for index, module in enumerate(self._laser_msg.ranges):

            angle = self._laser_msg.angle_increment * (index + 1)
            if module >= precision:
                angle_void.append(angle)
        
        return angle_void
    
    def fusion_all(self, laser_angle):

        angles = laser_angle
        angleVoid = ObjectVoid()

        while True:
            get_fusion_angle = self.fusion_angle(angles)

            if get_fusion_angle['ancien'] and len(get_fusion_angle['ancien']) > 1:

                min_angle = min(get_fusion_angle["nouveau"])
                max_angle = max(get_fusion_angle["nouveau"])

                angleVoid.angle_void_max.append(max_angle * 200 / cmath.pi)

                angleVoid.angle_void_min.append(min_angle * 200 / cmath.pi)

                angles = get_fusion_angle["ancien"]

            else:
                #print(get_fusion_angle)
                min_angle = min(get_fusion_angle["nouveau"])
                max_angle = max(get_fusion_angle["nouveau"])

                angleVoid.angle_void_max.append(max_angle * 200 / cmath.pi)
                angleVoid.angle_void_min.append(min_angle * 200 / cmath.pi )

                break

        return angleVoid

    def fusion_angle(self, angle_disponible, sep=cmath.pi/32):
        """
        Description:
        ------------
            cette fonction permet de fusionnner les angles
            proches
        """

        nouveau_angle = []
        ancien_angle = []

        angles = angle_disponible

        angles.sort()
        
        taille = len(angle_disponible)

        for index, angle in enumerate(angles):

            index_suivant = index + 1

            if index_suivant < taille:
                
                if abs(angle_disponible[index_suivant]) - abs(angle) > sep:

                    nouveau_angle = angle_disponible[:index_suivant]
                    ancien_angle = angle_disponible[index_suivant:]

                    return {
                        "nouveau": nouveau_angle,
                        "ancien": ancien_angle,
                    }

        return {
            "nouveau": angle_disponible,
            "ancien": ancien_angle,
        }
    
if __name__ == '__main__':
    
    rospy.init_node("laser_node")
    
    laser = Laser()
    
    laser.run_filtre()
    
    rospy.spin()