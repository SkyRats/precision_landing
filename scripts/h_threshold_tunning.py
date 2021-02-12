#!/usr/bin/env python
import rospy
from dynamic_reconfigure.client import Client
from precision_landing.msg import H_info
from std_msgs.msg import Bool

class HAutotune:
    
    DETECTION_THRESH = 100
    CLOSENESS_THRESH = 5

    # def __init__(self, mav):
    def __init__(self):
        self.running_state_pub = rospy.Publisher("/precision_landing/set_running_state", Bool, queue_size=10)
        self.detection_sub = rospy.Subscriber('/precision_landing/detection', H_info, self.detection_callback)
        # self.mav = mav
        self.config_client = None
        self.detection = None
        self.rate = rospy.Rate(1)

    def run(self):
        self.start_h_node()
        self.tune_threshold()

    def tune_threshold(self):
        new_threshold = 0
        while self.detection is None:
            self.config_client.update_configuration({"cv_threshold" : new_threshold})
            new_threshold += 3
            self.rate.sleep()
        
        first = True
        average_center_x = 0
        average_center_y = 0
        detection_counter = 1
        while not rospy.is_shutdown() and detection_counter < self.DETECTION_THRESH:

            # self.mav.hold(5)
            self.rate.sleep()

            if self.detection.detected:

                if first:
                    average_center_x = self.detection.center_x
                    average_center_y = self.detection.center_y
                    first = False
                else:
                    average_center_x = (average_center_x*detection_counter + self.detection.center_x) / (detection_counter+1)
                    average_center_y = (average_center_y*detection_counter + self.detection.center_y) / (detection_counter+1)

                if (self.are_close(average_center_x, self.detection.center_x)
                    and self.are_close(average_center_y, self.detection.center_y)):
                    detection_counter += 1
                else:
                    new_threshold -= 1

            else:
                new_threshold += 1

            self.config_client.update_configuration({"cv_threshold" : new_threshold})
            self.rate.sleep()
        rospy.loginfo('IDEAL THRESHOLD: ' + str(new_threshold))
        # self.mav.land()

    def start_h_node(self):
        for i in range (100):
            self.running_state_pub.publish(Bool(True))
            # self.mav.rate.sleep()
            self.rate.sleep()

    def are_close(self, x, y):
        if abs(x-y) < self.CLOSENESS_THRESH:
            return True
        else:
            return False

    def detection_callback(self, vector_data):
        self.detection = vector_data

if __name__ == '__main__':
    # from mavbase.MAV import MAV
    # mav = MAV('1')
    rospy.init_node('h_tunning')
    height = rospy.get_param('/height')
    # autotune = HAutotune(mav)
    autotune = HAutotune()
    autotune.config_client = Client('h_node', timeout=30)
    autotune.config_client.update_configuration({"cv_threshold" : 0})
    # mav.takeoff(height)
    autotune.run()
