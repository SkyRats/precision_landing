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
from simple_pid import PID

# TODO
# DESLIGAR O CV CONTROL NO FINAL DO CODIGO
# MUDAR OS PRINTS DA MAVBASE 
class PrecisionLanding():
    def __init__(self,MAV):
        # ROS setup
        self.rate = rospy.Rate(60)

        self.MAV = MAV

        self.cv_control_publisher = rospy.Publisher("/precision_landing/set_running_state", Bool, queue_size=10)
        
        self.detection_sub = rospy.Subscriber('/precision_landing/detection', H_info, self.detection_callback)
        self.last_time = time.time()

        # Attributes
        self.vel_x = self.vel_y = self.vel_z = 0
        self.delay = 0
        self.is_losted = True
        self.last_time = time.time()
        
        # PIDs 
        # Parametros Proporcional,Integrativo e Derivativo 
        self.pid_x = PID(-0.2, -0.01, -0.1)         
        self.pid_y = PID(0.2, 0.01, 0.1)
        self.pid_z = PID(-0.2, -0.005, -0.001)# Negative parameters (CV's -y -> Frame's +z)
        self.pid_w = PID(0, 0, 0) # Orientation

        self.pid_x.setpoint = -240.0/2 # y size
        self.pid_y.setpoint = 320.0/2 # x
        self.pid_z.setpoint = 0.2 # 
        self.pid_w.setpoint = 0 # orientation

        #Limitacao da saida        
        self.pid_x.output_limits = self.pid_y.output_limits = (-0.2, 0.2) # output value will be between -0.3 and 0.3
        self.pid_z.output_limits = (-0.15, 0.15)  # output value will be between -0.8 and 0.8

    def running_callback(self, bool):
        #Variavel alterada em run_h_mission para iniciar a deteccao do H
        #rospy.logwarn("Recebendo info do topicode cv")
        self.running_state = bool.data

    def detection_callback(self, vector_data):
        #Dados enviados pelo H.cpp -> Centro do H e Proximidade do H (Area ratio)
        self.detection = vector_data
        self.last_time = time.time()
        rospy.loginfo(self.detection.area_ratio)

    def run(self):
        #Inicializacao das variaveis usadas caso o drone perca o H
        r = 0                                        #Inicia valor para o raio da espiral 
        teta = 0                                     #Inicia valor para o teta da espiral 
        last_x = self.MAV.drone_pose.pose.position.x    
        last_y = self.MAV.drone_pose.pose.position.y
        last_z = self.MAV.drone_pose.pose.position.z
        for i in range (10):
            self.cv_control_publisher.publish(Bool(True))
            self.MAV.rate.sleep()
        self.MAV.takeoff(2)
        

        while not rospy.is_shutdown():
            flag = 0
            self.delay = time.time() - self.last_time
            self.is_losted = self.delay > 5 
            if not self.is_losted:  
                if self.detection.area_ratio < 0.1: #Drone ainda esta longe do H
                    r = 0
                    teta = 0
                    if(flag == 0):
                        rospy.loginfo("Controle PID para pouso de precisao")
                    flag = 1
                    
                    self.vel_x = self.pid_x(-self.detection.center_y)
                    self.vel_y = self.pid_y(self.detection.center_x)
                    self.vel_z = self.pid_z(-self.detection.area_ratio) # PID z must have negative parameters
                    
                    #Armazena ultima posicao em que o drone nao estava perdido
                    last_x = self.MAV.drone_pose.pose.position.x
                    last_y = self.MAV.drone_pose.pose.position.y
                    last_z = self.MAV.drone_pose.pose.position.z
            
                else:
                    #Caso o drone esteja suficientemente perto do H
                    rospy.loginfo("H encontrado!")
                    rospy.logwarn("Descendo...")
                    self.MAV.land()
                    for i in range (10):
                        self.cv_control_publisher.publish(Bool(False))
                        self.MAV.rate.sleep()

                self.MAV.set_vel(self.vel_x,self.vel_y,self.vel_z,0,0,0)

            else:  #Drone perdeu o H
                ### Fazer espiral ###
                if(flag == 1):
                    rospy.loginfo("Fazendo espiral para procurar o H")
                flag = 0
                #Funcao de espiral e volta para dois metros de altura
                r += 0.002  
                teta += 0.03
                x = last_x - r * math.cos( teta ) 
                y = last_y - r * math.sin( teta )
                self.MAV.set_position(x,y,2)
                if (r > 1.5):   #limita o tamanho da espiral
                    r = 0

            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node('precision_landing')
    drone = MAV("Robinho")
    c = PrecisionLanding(drone)
    c.run()