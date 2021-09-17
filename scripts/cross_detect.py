#!/usr/bin/env python3
import cv2
import rospy
import math
import time
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError
from precision_landing.msg import H_info
from std_msgs.msg import Bool

class CrossDetect():
    def __init__(self):
        self.rate = rospy.Rate(60)
        self.cv_control_sub = rospy.Subscriber("/precision_landing/set_running_state", Bool, self.running_callback)
        self.image_sub = rospy.Subscriber("/uav1/bluefox_optflow/image_raw", Image, self.camera_callback)
        self.detection_pub = rospy.Publisher('/precision_landing/detection', H_info, queue_size = 10)
        self.bridge_object = CvBridge()
        self.image_pixel_width = 752.0
        self.image_pixel_height = 480.0
        self.running = False

    def camera_callback(self,data):
        try:
            self.cv_image = self.bridge_object.imgmsg_to_cv2(data,desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

    def running_callback(self, data):
        self.running = data


    def detect(self):
        frame = self.cv_image
        cv2.imshow("HSV", frame)
        cv2.waitKey(3)
        cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        frame_threshold = cv2.inRange(frame, (0, 240, 240), (20, 255, 255))
        contours, hierarchy = cv2.findContours(image=frame_threshold, mode=cv2.RETR_LIST, method=cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area>1000:
                peri = cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, 0.15 * peri, True)

                if len(approx) == 4:
                    screen = approx
                    cv2.drawContours(frame, [screen], -1, (0,255,0), 3)
                    M = cv2.moments(cnt)

                    mensagem = H_info()
                    mensagem.detected = True
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    mensagem.center_x = cx
                    mensagem.center_y = cy
                    print(cx)
                    print(cy)
                    areaImagem = self.image_pixel_width * self.image_pixel_height
                    areaRatio = area / areaImagem
                    mensagem.area_ratio = areaRatio
                    self.detection_pub.publish(mensagem)
                    mensagem.detected = False
        cv2.imshow("HSV", frame)
        cv2.waitKey(3)
        cv2.imshow("InRange", frame_threshold)
        cv2.waitKey(3)


    def run(self):
        while(not rospy.is_shutdown()):
            if self.running:
                self.detect()
        self.rate.sleep() 


if __name__ == '__main__':
    rospy.init_node('cross_detect')
    c = CrossDetect()
    c.run()
