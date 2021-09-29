#!/usr/bin/env python3
from pickle import FALSE
import rospy
import time
import numpy as np

from mrs_mavbase.MRS_MAV import MRS_MAV
from precision_landing.msg import H_info
from geometry_msgs.msg import TwistStamped, PoseStamped
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool
from simple_pid import PID
from mrs_msgs.msg import PositionCommand
VEL_CERTO = 0.3

class PrecisionLanding():
    def __init__(self, MAV):
        # ROS setup
        self.rate = rospy.Rate(60)
        self.MAV = MAV

        self.detection_sub = rospy.Subscriber('/precision_landing/detection', H_info, self.detection_callback)
        self.last_time = time.time()
        self.vel_publisher = rospy.Publisher("/vel", Vector3, queue_size=10)
        self.base_publisher = rospy.Publisher("/base_position", Vector3, queue_size=10)
        self.stop_publisher = rospy.Publisher("/stop_trajectory", Bool, queue_size=10)

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
        talz = 15 #segundos
        kpz = 2
        # PIDs
        # Parametros Proporcional,Integrativo e Derivativo
        self.pid_x = PID(-0.005, 0, -0)
        self.pid_y = PID(0.005, 0, 0)
        # Negative parameters (CV's -y -> Frame's +z)
        self.pid_z = PID(-kpz, -kpz/talz, 0)
        self.pid_w = PID(0, 0, 0)  # Orientation

        self.pid_x.setpoint = self.image_pixel_height/2  # y size
        self.pid_y.setpoint = self.image_pixel_width/2  # x
        self.pid_z.setpoint = 0.12 #Podemos mudar para um lidar (fazer um filtro)
        self.pid_w.setpoint = 0  # orientation

        # Limitacao da saida
        self.pid_x.output_limits = self.pid_y.output_limits = (-1, 1)
        self.pid_z.output_limits = (-1, 1)


    def detection_callback(self, vector_data):
        # Dados enviados pelo H.cpp -> Centro do H e Proximidade do H (Area ratio)
        self.detection = vector_data
        self.last_time = time.time()
        self.first_detection = 1

    def precision_land(self, initial_height):
        rospy.wait_for_message("/uav1/control_manager/position_cmd", PositionCommand)
        # Inicializacao das variaveis usadas caso o drone perca o H
        last_x = return_to_trajectory_x = self.MAV.controller_data.position.x 
        last_y = return_to_trajectory_y = self.MAV.controller_data.position.y
        last_z = return_to_trajectory_z = self.MAV.controller_data.position.z
        flag =0 
        print(last_x)
        print(last_y)
        print(last_z)
        self.done = 0
        
        while not rospy.is_shutdown() and self.done == 0:
            self.delay = time.time() - self.last_time
            if self.first:
                self.is_lost = True
            else:
                self.is_lost = self.delay > 3
            if not self.is_lost and self.first_detection == 1:
                if self.detection.area_ratio < 0.1:  # Drone ainda esta longe do H
                    if(flag == 0):
                        rospy.loginfo("Controle PID")
                        print("Stop trajectory")
                        for i in range(10):
                            self.stop_publisher.publish(1)
                            self.rate.sleep()
                        flag = 1
                    return_to_trajectory_x = self.MAV.controller_data.position.x
                    return_to_trajectory_y = self.MAV.controller_data.position.y
                    return_to_trajectory_z = self.MAV.controller_data.position.z
                
                    
                    for i in range(10):
                        self.stop_publisher.publish(Bool(True))
                    
                    self.velocity.x= self.pid_x(-self.detection.center_y)
                    self.velocity.y = self.pid_y(self.detection.center_x)
                    # PID z must have negative parameters
                    if(abs(self.velocity.x) < VEL_CERTO and abs(self.velocity.y) < VEL_CERTO):
                        self.velocity.z = self.pid_z(self.detection.area_ratio)
                    else:
                        self.velocity.z = 0

                    # Armazena ultima posicao em que o drone nao estava perdido
                    last_x = self.MAV.controller_data.position.x
                    last_y = self.MAV.controller_data.position.y
                    last_z = self.MAV.controller_data.position.z

                else:
                    flag = 0
                    # Caso o drone esteja suficientemente perto do H
                    rospy.loginfo("Cruz encontrada!")
                    rospy.logwarn("Descendo...")
                    
                    self.MAV.set_position(0,-0.2 ,0,relative_to_drone=True)
                    now = rospy.get_rostime()
                    while not rospy.get_rostime() - now > rospy.Duration(secs=2):
                        self.rate.sleep()
                    self.MAV.land()
                    now = rospy.get_rostime()
                    while not rospy.get_rostime() - now > rospy.Duration(secs=4):
                        self.rate.sleep()
                    self.MAV.disarm()
                    base_pos_x = self.MAV.controller_data.position.x
                    base_pos_y = self.MAV.controller_data.position.y

                    rospy.loginfo("Localizacao da base:")
                    rospy.loginfo(base_pos_x)
                    rospy.loginfo(base_pos_y)
                    for i in range(10):
                        self.base_publisher.publish(base_pos_x,base_pos_y,0)
                        print("publicando")

                    now = rospy.get_rostime()
                    while not rospy.get_rostime() - now > rospy.Duration(secs=2):
                        self.rate.sleep()
                    self.MAV.arm()
                    now = rospy.get_rostime()
                    while not rospy.get_rostime() - now > rospy.Duration(secs=2):
                        self.rate.sleep()
                    self.MAV.takeoff()
                    now = rospy.get_rostime()
                    while not rospy.get_rostime() - now > rospy.Duration(secs=2):
                        self.rate.sleep()
                    self.MAV.set_position(return_to_trajectory_x, return_to_trajectory_y,return_to_trajectory_z, 0, False)
                    print("Voltou para posicao e ligou trajetoria")
                    for i in range(10):
                            self.stop_publisher.publish(Bool(False))
    
                for i in range(10):
                    self.vel_publisher.publish(self.velocity)
                    self.rate.sleep()

            elif self.first:
                rospy.loginfo("Iniciando...")
                x = last_x
                y = last_y
                self.first = False

                self.MAV.set_position(x, y, last_z, 0, False)
            elif self.done != 1:  # Drone perdeu o H 
                if flag == 1:              
                    rospy.loginfo("Ainda nao achou a cruz")
                    for i in range(10):
                        self.stop_publisher.publish(Bool(False))
                flag = 0

            self.rate.sleep()
    
    '''def run():
        #Depois de ter a tajetoria, criar um subscriber para receber se deve ou nao usar o precision_land
        pass'''

if __name__ == "__main__":
    rospy.init_node('control')
    drone = MRS_MAV("uav1")
    c = PrecisionLanding(drone)
    initial_height = 4
    c.precision_land(initial_height)