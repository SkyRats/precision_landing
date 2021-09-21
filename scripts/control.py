#!/usr/bin/env python
import rospy
import math
import time
import numpy as np

from MRS_MAV import MRS_MAV
from precision_landing.msg import H_info
from geometry_msgs.msg import TwistStamped, PoseStamped
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool
from simple_pid import PID


class PrecisionLanding():
    def __init__(self, MAV):
        # ROS setup
        self.rate = rospy.Rate(60)

        self.MAV = MAV

        self.cv_control_publisher = rospy.Publisher(
            "/precision_landing/set_running_state", Bool, queue_size=10)

        self.detection_sub = rospy.Subscriber(
            '/precision_landing/detection', H_info, self.detection_callback)
        self.last_time = time.time()

        self.vel_publisher = rospy.Publisher("/vel", Vector3, queue_size=10)

        # Cam Params
        self.image_pixel_width = 752
        self.image_pixel_height = -480

        # Attributes
        self.delay = 0
        self.is_lost = True
        self.flag = 0
        self.done = 0
        self.first = True
        self.velocity = Vector3()
        self.first_detection = 0


        # PIDs
        # Parametros Proporcional,Integrativo e Derivativo
        self.pid_x = PID(-0.002, -0, -0)
        self.pid_y = PID(0.002, 0, 0)
        # Negative parameters (CV's -y -> Frame's +z)
        self.pid_z = PID(-0.7, 0, 0)
        self.pid_w = PID(0, 0, 0)  # Orientation

        self.pid_x.setpoint = self.image_pixel_height/2 + 27 # y size
        self.pid_y.setpoint = self.image_pixel_width/2  # x
        self.pid_z.setpoint = 0.25 #Podemos mudar para um lidar (fazer um filtro)
        self.pid_w.setpoint = 0  # orientation

        # Limitacao da saida
        self.pid_x.output_limits = self.pid_y.output_limits = (-0.15, 0.15)
        self.pid_z.output_limits = (-0.4, 0.4)


    def detection_callback(self, vector_data):
        # Dados enviados pelo H.cpp -> Centro do H e Proximidade do H (Area ratio)
        self.detection = vector_data
        self.last_time = time.time()
        self.first_detection = 1

    def run(self, initial_height):
        # Inicializacao das variaveis usadas caso o drone perca o H
        r = 0  # Inicia valor para o raio da espiral
        teta = 0  # Inicia valor para o teta da espiral
        last_x = self.MAV.controller_data.position.x 
        last_y = self.MAV.controller_data.position.y
        last_z = self.MAV.controller_data.position.z
        print(last_x)
        print(last_y)
        print(last_z)

        for i in range(10):
            self.cv_control_publisher.publish(Bool(True))
            self.MAV.rate.sleep()

        while not rospy.is_shutdown():
            self.delay = time.time() - self.last_time
            if self.first:
                self.is_lost = True
            else:
                self.is_lost = self.delay > 3
            if not self.is_lost and self.first_detection == 1:
                if self.detection.area_ratio < 0.12:  # Drone ainda esta longe do H
                    r = 0
                    teta = 0
                    
                    rospy.loginfo("Controle PID")
                    self.velocity.x= self.pid_x(-self.detection.center_y)
                    self.velocity.y = self.pid_y(self.detection.center_x)
                    # PID z must have negative parameters
                    self.velocity.z = self.pid_z(self.detection.area_ratio)

                    print("Vel x")
                    print(self.velocity.x)
                    print("Vel y")
                    print(self.velocity.y)
                    print("Vel z")
                    print(self.velocity.z)
                    # Armazena ultima posicao em que o drone nao estava perdido
                    last_x = self.MAV.controller_data.position.x
                    last_y = self.MAV.controller_data.position.y
                    last_z = self.MAV.controller_data.position.z

                else:
                    # Caso o drone esteja suficientemente perto do H
                    rospy.loginfo("H encontrado!")
                    rospy.logwarn("Descendo...")
                    self.MAV.land()
                    for i in range(10):
                        self.cv_control_publisher.publish(Bool(False))
                        self.MAV.rate.sleep()
                    self.done = 1

                for i in range(10):
                    self.vel_publisher.publish(self.velocity)
                    self.rate.sleep()

            elif self.first:
                rospy.loginfo("Iniciando...")
                x = last_x
                y = last_y
                self.first = False
                self.MAV.set_position(x, y, initial_height)
            elif self.done != 1:  # Drone perdeu o H
                ### Fazer espiral ###
               
                rospy.loginfo("Fazendo espiral")
                self.flag = 0
                # Funcao de espiral e volta para dois metros de altura
                r += 0.002
                teta += 0.03
                x = last_x - r * math.cos(teta)
                y = last_y - r * math.sin(teta)
                self.MAV.set_position(x, y, initial_height)
                if (r > 1.5):  # limita o tamanho da espiral
                    r = 0

            self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node('control')
    drone = MRS_MAV("uav1")
    c = PrecisionLanding(drone)
    initial_height = 4
    c.run(initial_height)