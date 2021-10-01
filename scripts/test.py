#!/usr/bin/env python3
#Trajetoria de 5 em 5 metros
#A cada 5 segundos

from re import X
import rospy
import numpy as np
from mrs_mavbase.MRS_MAV import MRS_MAV
from geometry_msgs.msg import Vector3
from rospy.core import is_shutdown
from std_msgs.msg import Bool

TOL = 1

class TrajectoryTest():
    def __init__(self, MAV):
        self.rate = rospy.Rate(60)
        self.MAV = MAV
        self.altura = 4    #Altura da trajetoria
        self.vel = 4
        self.parte_missao = 0
        self.cv_control_publisher = rospy.Publisher("/precision_landing/set_running_state", Bool, queue_size=10)
        self.stop_sub = rospy.Subscriber("/stop_trajectory", Bool, self.stop_callback)
        self.stop = 0
        self.base_sub = rospy.Subscriber("/base_position", Vector3, self.base_pos_callback)
        self.Lista_das_bases = []

    def base_pos_callback(self, data):  #Recebe posicao das bases encontradas
        self.base_pos_x = data.x
        self.base_pos_y = data.y
        if [data.x,data.y] not in self.Lista_das_bases: 
            self.Lista_das_bases.append([self.base_pos_x,self.base_pos_y])


    def stop_callback(self, data):      #Recebe ordem para parar a trajetoria
        self.stop = data.data


    def run(self):
        initial_x = 30
        initial_y = 40
        largura = 15
        curva = 0
        goal_x = initial_x
        cont = 0
        loop = 0
        goal_y = 10
        self.MAV.set_position(46,9,self.altura)
        while(not rospy.is_shutdown()):
            for i in range(10):
                self.cv_control_publisher.publish(Bool(self.verificar_area()))   #Liga a deteccao da cruz
                #self.cv_control_publisher.publish(Bool(True))   #Liga a deteccao da cruz

                #print("CV: " + str(self.verificar_area()))
                self.rate.sleep()
            print("STOP: " + str(self.stop))
            if(self.stop == False ):
                if(self.parte_missao == 0):
                    self.MAV.set_position(46,9,-4)
                    self.parte_missao = 1
                    
                elif(self.parte_missao == 1):
                    self.MAV.set_position(46,9,-6)
                    self.MAV.set_position(30,10,self.altura)
                    self.parte_missao = 2

                elif(self.parte_missao == 2):
                    if (curva == 0):
                        initial_y = 10
                        goal_y = -30 #-20
                        if(loop%2 == 1):
                            print("volta")
                            backup_y = goal_y
                            goal_y = initial_y 
                            initial_y = backup_y
                        
                            self.MAV.set_position(goal_x, initial_y - cont, self.altura)
                            cont -= self.vel
                            #self.MAV.set_position(0,self.vel,0, relative_to_drone = True)

                            if(self.MAV.controller_data.position.y - goal_y > TOL):
                                loop = loop + 1
                                cont = 0
                                goal_x -= largura
                                curva = 1

                        else:
                            print("ida")
                            self.MAV.set_position(goal_x, initial_y - cont, self.altura)
                            cont += self.vel
                            #self.MAV.set_position(0,-self.vel,0, relative_to_drone = True)
                            if(self.MAV.controller_data.position.y - goal_y < TOL):
                                loop += 1
                                cont = 0
                                goal_x -= largura
                                curva = 1

                    if(curva == 1):
                        print("curva")
                        self.MAV.set_position(initial_x -(loop * largura) - cont, goal_y, self.altura)
                        cont += self.vel
                        #self.MAV.set_position(-self.vel,0,0, relative_to_drone = True)
                        if(self.MAV.controller_data.position.x - goal_x < TOL):
                            cont = 0
                            curva = 0

                    if(loop == 3):
                        self.parte_missao = 3
                        print("Fim zigzag")


                elif (self.parte_missao == 3):
                    self.MAV.set_position(-19,-21,self.altura)
                    self.parte_missao = 4

                elif self.parte_missao == 4:
                    self.MAV.set_position(-50,-21,self.altura + 3)
                    self.parte_missao = 5

                elif self.parte_missao == 5:
                    self.MAV.set_position(-19,-21,self.altura)
                    self.parte_missao = 6
                
                elif self.parte_missao == 6:
                    self.MAV.RTL()
                    self.parte_missao = 7
            self.rate.sleep()

    def verificar_area(self): 
        for [x,y] in self.Lista_das_bases:
            if( np.power((self.MAV.controller_data.position.x - x),2) + np.power((self.MAV.controller_data.position.y - y),2) <= 144):
                return False
        return True

if __name__ == "__main__":
    rospy.init_node('test')
    drone = MRS_MAV("uav1")
    c = TrajectoryTest(drone)
    c.run()