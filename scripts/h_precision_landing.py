#!/usr/bin/env python
import rospy
import math
import time
import numpy as np

from mavbase.MAV import MAV
from precision_landing.msg import H_info
from geometry_msgs.msg import TwistStamped, PoseStamped
from std_msgs.msg import Bool
from dynamic_reconfigure.server import Server
from precision_landing.cfg import ControllerConfig


class PrecisionLanding():
    def __init__(self,MAV):
        # ROS setup
        self.rate = rospy.Rate(60)
        self.MAV = MAV

        self.cv_control_publisher = rospy.Publisher("/precision_landing/set_running_state", Bool, queue_size=10)

        self.detection_sub = rospy.Subscriber('/precision_landing/detection', H_info, self.detection_callback)
        self.tol = 0.05
        #Cam Params
        self.image_pixel_width = 320.0
        self.image_pixel_height = 240.0
        self.FOV = 1.047 # Horizontal Field of View in rad
         

        # Attributes
        self.detection = H_info()
        


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
     
            dist_of_one_pixel = (self.MAV.drone_pose.pose.position.z * math.tan(self.FOV/2))/(self.image_pixel_width/2)
            rospy.logwarn("Distancia de um pixel")
            rospy.logwarn(dist_of_one_pixel)
            x_dif = self.detection.center_y - (self.image_pixel_width/2)
            y_dif = self.detection.center_x - (self.image_pixel_height/2)

            self.H_pos_rel_x = x_dif * dist_of_one_pixel
            self.H_pos_rel_y = y_dif * dist_of_one_pixel
            self.goal_x = self.MAV.drone_pose.pose.position.x - self.H_pos_rel_x
            self.goal_y = self.MAV.drone_pose.pose.position.y - self.H_pos_rel_y 
            # DEBBUG
                       
            rospy.loginfo("detection center x")
            rospy.loginfo(self.detection.center_x)
            rospy.loginfo("detection center y")
            rospy.loginfo(self.detection.center_y)  
            '''
            rospy.loginfo("x dif")
            rospy.loginfo(x_dif)
            rospy.loginfo("y dif")
            rospy.loginfo(y_dif)
            '''
            rospy.loginfo("x ")
            rospy.loginfo(self.H_pos_rel_x)
            rospy.loginfo("y")
            rospy.loginfo(self.H_pos_rel_y)

            rospy.loginfo("Goal x")
            rospy.loginfo(self.goal_x)
            rospy.loginfo("Goal y")
            rospy.loginfo(self.goal_y)  





    def run(self,initial_height):
        for i in range (10):
            self.cv_control_publisher.publish(Bool(True))
            self.MAV.rate.sleep()
        self.MAV.hold(3)
        self.calculate_h_position() 
        self.MAV.set_position(self.goal_x - 0.2, self.goal_y + 0.5, 0.1)
        while not self.MAV.chegou():
            self.MAV.set_position(self.goal_x - 0.2, self.goal_y + 0.5 , 0.1)
            self.MAV.rate.sleep()  
    
        '''
        self.calculate_h_position()
        for i in range(100):
            self.MAV.set_position(self.goal_x, self.goal_y, 0.5)
            self.MAV.rate.sleep()  '''        
        self.MAV.land()

if __name__ == "__main__":
    rospy.init_node('precision_landing')
    drone = MAV("Robinho")
    c = PrecisionLanding(drone)
    rospy.loginfo("X")
    rospy.loginfo(c.MAV.drone_pose.pose.position.x)
    rospy.loginfo("Y")
    rospy.loginfo(c.MAV.drone_pose.pose.position.y)
    rospy.loginfo("Z")
    rospy.loginfo(c.MAV.drone_pose.pose.position.z)
    c.MAV.takeoff(3)
    initial_height = 3
    c.run(initial_height)
