#!/usr/bin/env python3
#Trajetoria de 5 em 5 metros
#A cada 5 segundos

import rospy
import numpy as np
from mrs_mavbase.MRS_MAV import MRS_MAV
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool

TOL = 1

class Trajectory():
    def __init__(self, MAV):
        self.rate = rospy.Rate(0.5)
        self.MAV = MAV
        self.altura = 6
        self.cv_control_publisher = rospy.Publisher("/precision_landing/set_running_state", Bool, queue_size=10)
        self.stop_sub = rospy.Subscriber("/stop_trajectory", Bool, self.stop_callback)


    def stop_callback(self, data):
        self.stop = data
        
    def run(self):
        initial_x = 30
        initial_y = 40
        cont = 0
        self.MAV.set_position(initial_x,initial_y,self.altura, hdg = 0, relative_to_drone = False)
        
        self.cv_control_publisher.publish(Bool(True))
        self.MAV.rate.sleep()
        goal_y = 10
        while(abs(self.MAV.controller_data.position.y - goal_y) > TOL):
                if(self.stop == 0):
                    print("y")
                    print(initial_y - cont)
                    self.MAV.set_position(initial_x, initial_y - cont, self.altura)
                    cont+= 5
                self.rate.sleep()

        self.MAV.set_position(46,9,6)
        self.MAV.rate.sleep()

        self.MAV.set_position(30,10,self.altura, hdg = 0, relative_to_drone = False)

        largura = 15
        goal_x = initial_x

        for i in range(4):
            initial_x = 30
            initial_y = 10
            goal_y = -20 #-20
            if(i%2 == 1):
                backup_y = goal_y
                goal_y = initial_y 
                initial_y = backup_y

            while(abs(self.MAV.controller_data.position.y - goal_y) > TOL):
                if(self.stop == 0):
                    print("y")
                    print(initial_y - cont)
                    self.MAV.set_position(goal_x, initial_y - cont, self.altura)
                    if(i%2 == 1):
                        cont -= 5
                    else:
                        cont+= 5
                self.rate.sleep()
            self.rate.sleep()
            cont = 0
            goal_x -= largura#16
            if(i < 3):
                while(abs(self.MAV.controller_data.position.x - goal_x) > TOL):
                    if(self.stop == 0):
                        print("x")
                        print(initial_x -(i * largura) - cont)
                        self.MAV.set_position(initial_x -(i * largura) - cont, goal_y, self.altura)
                        cont += 5
                    self.rate.sleep()
            cont = 0

    def verificar_area():
        #Fazer uma funcao para se uma base ja pousada estiver na area, nao ligar a deteccao da cruz

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
