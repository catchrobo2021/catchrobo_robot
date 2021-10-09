#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from sensor_msgs.msg import JointState

from catchrobo_manager.servo import Servo, Laser

class GripperNumber:
    GRIPPER1 = 0
    GRIPPER2 = 1

class GripperManager():
    def __init__(self):
        self._grippers = [Servo("gripper1"), Servo("gripper2")]
        self._lasers = [Laser("laser1"), Laser("laser2")]

    def laser(self, laser_on):
        self._lasers[0].output(laser_on)
        self._lasers[1].output(laser_on)

    def graspBisco(self, target_gripper, grip_way,wait):
        dist = grip_way
        self._grippers[target_gripper].move(dist,wait)
        temp = "gripper {}: {} cm".format(target_gripper, dist)
        rospy.loginfo(temp)


    def releaseBisco(self, target_gripper):
        self._grippers[target_gripper].move(0)
        
        rospy.loginfo("release")

