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

class Trajectory():
    def __init__(self, MAV):
        self.rate = rospy.Rate(0.5)
        self.MAV = MAV
        self.altura = 4    #Altura da trajetoria
        self.stop = False
        self.vel = 4
        self.Lista_das_bases = []


        ##############Publishers##################
        self.cv_control_publisher = rospy.Publisher("/precision_landing/set_running_state", Bool, queue_size=10)

        ##############Subscribers##################
        self.stop_sub = rospy.Subscriber("/stop_trajectory", Bool, self.stop_callback)
        self.base_sub = rospy.Subscriber("/base_position", Vector3, self.base_pos_callback)
        

    def base_pos_callback(self, data):  #Recebe posicao das bases encontradas
        self.base_pos_x = data.x
        self.base_pos_y = data.y
        print(data.x)
        print(data.y)
        if [data.x,data.y] not in self.Lista_das_bases: 
            self.Lista_das_bases.append([self.base_pos_x,self.base_pos_y])


    def stop_callback(self, data):      #Recebe ordem para parar a trajetoria
        self.stop = data
        
    def run(self):
        flag = 0
        parte_missao = 0
        initial_x = 30
        initial_y = 40
        largura = 15
        curva = 0
        goal_x = initial_x
        cont = 0
        loop = 0
        self.MAV.set_position(initial_x,initial_y,self.altura, hdg = 0, relative_to_drone = False)  #Ponto de partida
        print("Ponto inicial")
        while(not rospy.is_shutdown() and parte_missao < 9):
            for i in range(30):
                    self.cv_control_publisher.publish(Bool(self.verificar_area()))   #Liga a deteccao da cruz
                    print("CV: " + str(self.verificar_area()))
            if(self.stop == 0 ):
                flag = 0
                if(parte_missao == 0):
                    goal_y = 10
                    parte_missao = 1

                elif(parte_missao == 1):
                    self.MAV.set_position(initial_x, initial_y - cont, self.altura)
                    cont+= self.vel
                    if (self.MAV.controller_data.position.y - goal_y < TOL):
                        parte_missao = 2
                        print("Chegou perto da base")


                elif(parte_missao == 2):
                    self.MAV.set_position(46,9,-2)
                    parte_missao = 3
                    print("Chegou na base")

                elif(parte_missao == 3):
                    self.MAV.rate.sleep()
                    self.MAV.set_position(30,10,self.altura, hdg = 0, relative_to_drone = False)
                    parte_missao = 4
                    print(loop)

                elif(parte_missao == 4):
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
                            if(self.MAV.controller_data.position.y - goal_y > TOL):
                                loop = loop + 1
                                cont = 0
                                goal_x -= largura
                                curva = 1

                        else:
                            print("ida")
                            self.MAV.set_position(goal_x, initial_y - cont, self.altura)
                            cont += self.vel
                            if(self.MAV.controller_data.position.y - goal_y < TOL):
                                loop += 1
                                cont = 0
                                goal_x -= largura
                                curva = 1

                    if(curva == 1):
                        print("curva")
                        self.MAV.set_position(initial_x -(loop * largura) - cont, goal_y, self.altura)
                        cont += self.vel
                        if(self.MAV.controller_data.position.x - goal_x < TOL):
                            cont = 0
                            curva = 0

                    if(loop == 3):
                        parte_missao = 5
                        print("Fim zigzag")


                elif (parte_missao == 5):
                    self.MAV.set_position(-19,-21,self.altura)
                    parte_missao = 6

                elif parte_missao == 6:
                    self.MAV.set_position(-50,-21,self.altura + 3)
                    parte_missao = 7

                elif parte_missao == 7:
                    self.MAV.set_position(-19,-21,self.altura)
                    parte_missao = 8
                
                elif parte_missao == 8:
                    self.MAV.RTL()
                    parte_missao = 9
            else:
                if(flag == 0):
                    self.MAV.set_position(0,0,0,0,relative_to_drone = True)
                    flag = 1

            self.rate.sleep()

    def verificar_area(self): # Ainda nao ta dando certo
        print("VERIFICAR AREA")
        print(self.Lista_das_bases)
        for [x,y] in self.Lista_das_bases:
            print("x: ")
            print(x)
            print("y: ")
            print(y)
            if( np.power((self.MAV.controller_data.position.x - x),2) + np.power((self.MAV.controller_data.position.y - y),2) <= 144):
                print("FALSE")
                return False
        print("TRUE")
        return True
      
if __name__ == "__main__":
    rospy.init_node('trajectory')
    drone = MRS_MAV("uav1")
    c = Trajectory(drone)
    c.run()





'''self.quantidade_idas_atual = 0
    initial_x = 30
    initial_y = 40
    goal_x = initial_x - 50/(self.quantidade_idas_meta-1)

    self.MAV.set_position(initial_x,initial_y,self.altura)
    now = rospy.get_rostime()
    self.MAV.set_position(self.MAV.controller_data.position.x,self.MAV.controller_data.position.y,self.altura,hdg = 0, relative_to_drone = False)
    while not rospy.get_rostime() - now > rospy.Duration(secs=2):
        self.rate.sleep()
    while abs(self.quantidade_idas_atual < self.quantidade_idas_meta) and not rospy.is_shutdown(): 
        if self.quantidade_idas_atual % 2 == 0:
            goal_y = -20
            dist_y = abs(goal_y - self.MAV.controller_data.position.y)
            print("ino")
            while dist_y > TOL :
                self.velocity.y = -self.vel
                self.velocity.x = 0
                for i in range(10):
                    self.vel_publisher.publish(self.velocity)
                    self.rate.sleep()
                dist_y = abs(goal_y - self.MAV.controller_data.position.y)
                self.rate.sleep()
        

        else:
            print("vortano")

            goal_y =  initial_y
            dist_y = abs(goal_y - self.MAV.controller_data.position.y)

            while dist_y > TOL:
                self.velocity.y = self.vel
                self.velocity.x = 0
                for i in range(10):
                    self.vel_publisher.publish(self.velocity)
                    self.rate.sleep()
                dist_y = abs(goal_y - self.MAV.controller_data.position.y)
                self.rate.sleep()
        self.quantidade_idas_atual += 1
        self.velocity.y = 0
        self.velocity.x = 0
        self.MAV.set_position(self.MAV.controller_data.position.x,self.MAV.controller_data.position.y,self.altura,hdg = 0, relative_to_drone = False)
        now = rospy.get_rostime()
        print("Pos x:")
        print(self.MAV.controller_data.position.x)
        print("Pos y:")
        print(self.MAV.controller_data.position.y)
        while not rospy.get_rostime() - now > rospy.Duration(secs=2):
            self.vel_publisher.publish(self.velocity)
            self.rate.sleep()

        if(self.quantidade_idas_atual < self.quantidade_idas_meta):
            print("curva")
            dist_x = abs(goal_x - self.MAV.controller_data.position.x)
            while dist_x > TOL : 
                self.velocity.y = 0
                self.velocity.x = -self.vel
                for i in range(10):
                    self.vel_publisher.publish(self.velocity)
                    self.rate.sleep()
                dist_x = abs(goal_x - self.MAV.controller_data.position.x)
                self.rate.sleep()
            goal_x -= 50/(self.quantidade_idas_meta-1)
        self.velocity.y = 0
        self.velocity.x = 0
        now = rospy.get_rostime()
        self.MAV.set_position(self.MAV.controller_data.position.x,self.MAV.controller_data.position.y,self.altura,hdg = 0, relative_to_drone = False)
        while not rospy.get_rostime() - now > rospy.Duration(secs=2):
            self.vel_publisher.publish(self.velocity)
            self.rate.sleep()
        print("Pos x:")
        print(self.MAV.controller_data.position.x)
        print("Pos y:")
        print(self.MAV.controller_data.position.y)'''
