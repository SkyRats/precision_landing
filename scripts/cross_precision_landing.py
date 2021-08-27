#!/usr/bin/env python3
import rospy
import math
import time
import numpy as np

from MRS_MAV import MRS_MAV
from precision_landing.msg import H_info
import mrs_msgs
from mrs_msgs import srv
from mrs_msgs.msg import PositionCommand, Reference
from mrs_msgs.srv import TrajectoryReferenceSrv, ReferenceStampedSrv
import std_srvs
from std_srvs import srv
from std_srvs.srv import Trigger
from std_msgs.msg import Bool


class PrecisionLanding():
    def __init__(self,MRS_MAV):
        # ROS setup
        self.rate = rospy.Rate(60)
        self.MAV = MRS_MAV

        self.cv_control_publisher = rospy.Publisher("/precision_landing/set_running_state", Bool, queue_size=10)

        self.detection_sub = rospy.Subscriber('/precision_landing/detection', H_info, self.detection_callback)
        self.tol = 0.05
        #Cam Params
        self.image_pixel_width = 752.0
        self.image_pixel_height = 480.0
        self.FOV = 0.8576 # Horizontal Field of View in rad 49.13434

        # Attributes
        self.detection = H_info()
        self.last_time = time.time()


    def running_callback(self, bool):
        #Variavel alterada em run_h_mission para iniciar a deteccao do H
        #rospy.logwarn("Recebendo info do topicode cv")
        self.running_state = bool.data

    def detection_callback(self, vector_data):
        #Dados enviados pelo H.cpp -> Centro do H e Proximidade do H (Area ratio)
        self.detection = vector_data
        self.last_time = time.time()

    def calculate_h_position(self):
        
        

        
        if self.detection:
            '''
                            |\
                            | \
                            |  \
                            |   \
                            |    \
                     Altura |-----\
                            |FOV/2 \
                            |       \
                            |        \
                            |         \
                            |__________\
                        Distancia cobrida pela camera
            SAbendo a distancia cobrida pela imagem e a qualtidade de pixeis na horizontal,
            conseguimos saber a distancia que cada pixel cobre (dist_of_one_pixel)
            '''
            rospy.logwarn(self.MAV.controller_data.position.z )
            dist_of_one_pixel = (self.MAV.controller_data.position.z * math.tan(self.FOV/2))/(self.image_pixel_width/2)
            rospy.logwarn("Distancia de um pixel")
            rospy.logwarn(dist_of_one_pixel)
            x_dif = self.detection.center_y - (self.image_pixel_width/2)
            y_dif = self.detection.center_x - (self.image_pixel_height/2)

            self.H_pos_rel_x = x_dif * dist_of_one_pixel
            self.H_pos_rel_y = y_dif * dist_of_one_pixel
            self.goal_x = self.MAV.controller_data.position.x - self.H_pos_rel_x
            self.goal_y = self.MAV.controller_data.position.y - self.H_pos_rel_y 
            # DEBBUG
                     
            rospy.loginfo("detection center x")
            rospy.loginfo(self.detection.center_x)
            rospy.loginfo("detection center y")
            rospy.loginfo(self.detection.center_y)  
            
            rospy.loginfo("x dif")
            rospy.loginfo(x_dif)
            rospy.loginfo("y dif")
            rospy.loginfo(y_dif)
            
            rospy.loginfo("x ")
            rospy.loginfo(self.H_pos_rel_x)
            rospy.loginfo("y")
            rospy.loginfo(self.H_pos_rel_y)

            rospy.loginfo("Goal x")
            rospy.loginfo(self.goal_x)
            rospy.loginfo("Goal y")
            rospy.loginfo(self.goal_y)  
            




    def run(self):
        for i in range (10):
            self.cv_control_publisher.publish(Bool(True))
            print("Ligando codigo de deteccao da cruz")
            self.rate.sleep()
        time.sleep(3)
        self.calculate_h_position() 
        self.MAV.set_position(self.goal_x, self.goal_y, 2)
        print("Supostamente em cima da cruz")
        rospy.spin()
        self.MAV.land()

if __name__ == "__main__":
    rospy.init_node('cross_precision_landing')
    drone = MRS_MAV("uav1")
    c = PrecisionLanding(drone)
    c.run()
