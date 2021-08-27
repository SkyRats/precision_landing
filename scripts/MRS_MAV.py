#!/usr/bin/env python3

import rospy
import mrs_msgs
from mrs_msgs import srv
from mrs_msgs.msg import PositionCommand, Reference, TrajectoryReference
from mrs_msgs.srv import TrajectoryReferenceSrv, ReferenceStampedSrv
import std_srvs
from std_srvs import srv
from std_srvs.srv import Trigger
import numpy as np
import math
import time

TOL = 0.1

class MRS_MAV:
    def __init__(self, mav_name):
        self.rate = rospy.Rate(60)
        self.controller_data = PositionCommand()
        self.position_controller = Reference()
        self.trajectory = TrajectoryReference()
        self.mav_name = mav_name
        
        ############# Services #############
        self.reference = rospy.ServiceProxy("/" + mav_name + "/control_manager/reference", ReferenceStampedSrv)

        self.trajectory_reference_srv = rospy.ServiceProxy("/" + mav_name + "/control_manager/trajectory_reference", TrajectoryReferenceSrv)
        self.goto_trajectory_start = rospy.ServiceProxy("/" + mav_name + "/control_manager/goto_trajectory_start", Trigger)
        self.start_trajectory_tracking = rospy.ServiceProxy("/" + mav_name + "/control_manager/start_trajectory_tracking", Trigger)
        self.stop_trajectory_tracking = rospy.ServiceProxy("/" + mav_name + "/control_manager/stop_trajectory_tracking", Trigger)
        self.resume_trajectory_tracking = rospy.ServiceProxy("/" + mav_name + "/control_manager/resume_trajectory_tracking", Trigger)

        self.takeoff_srv = rospy.ServiceProxy("/" + mav_name +"/uav_manager/takeoff", Trigger)
        self.land_srv = rospy.ServiceProxy("/" + mav_name +"/uav_manager/land", Trigger)

        ########## Subscribers #############
        controller_sub = rospy.Subscriber("/" + mav_name + "/control_manager/position_cmd", PositionCommand, self.controller_callback)
        rospy.wait_for_message("/" + mav_name + "/control_manager/position_cmd", PositionCommand)

    ############# Callback Functions #############
    def controller_callback(self, data):
        self.controller_data = data

    def set_position(self, x, y, z=None, hdg=None):
        if z == None:
            z = self.controller_data.position.z
        if hdg == None:
            hdg = self.controller_data.heading

        self.position_controller.position.x = x
        self.position_controller.position.y = y
        self.position_controller.position.z = z
        self.position_controller.heading = hdg
        
        
        rospy.wait_for_service("/" + self.mav_name + "/control_manager/reference")
        while abs(self.controller_data.position.x - x) > TOL or abs(self.controller_data.position.y - y) > TOL:
            self.reference(reference = self.position_controller)
    

    def land(self):
        rospy.wait_for_service("/" + self.mav_name +"/uav_manager/land")
        land_output = self.land_srv()
        if land_output.success:   
            rospy.loginfo_once(land_output.message)
        else:
            rospy.logerr_once(land_output.message)
        
    def takeoff(self):
        rospy.wait_for_service("/" + self.mav_name +"/uav_manager/takeoff")
        takeoff_output = self.takeoff_srv()
        if takeoff_output.success:
            rospy.loginfo_once(takeoff_output.message)
        else:
            rospy.logerr_once(takeoff_output.message)


if __name__ == '__main__':
    rospy.init_node("MRS_MAV")
    mav = MRS_MAV("uav1")
    mav.set_position(10,10)
    rospy.spin()
        