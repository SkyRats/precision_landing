
#!/usr/bin/env python3
import rospy
import math
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError
import numpy as np

class Resolution():
    def __init__(self):
        self.image_sub = rospy.Subscriber("/uav1/bluefox_optflow/image_raw", Image, self.camera_callback)
        self.bridge_object = CvBridge()
        self.rate = rospy.Rate(60)

    def camera_callback(self,data):
            try:
                self.cv_image = self.bridge_object.imgmsg_to_cv2(data,desired_encoding="bgr8")
            except CvBridgeError as e:
                print(e)
            
            h, w, c = self.cv_image.shape
            print(h)
            print(w)

    def run(self):
       rospy.spin()




if __name__ == '__main__':
    rospy.init_node('resolution')
    c = Resolution()
    c.run()