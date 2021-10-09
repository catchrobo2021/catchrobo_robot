#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from sensor_msgs.msg import JointState

from catchrobo_manager.servo import Servo

class GripperNumber:
    GRIPPER1 = 0
    GRIPPER2 = 1

class GripWay:
    LONG_MOVE = 0.03
    SMALL_MOVE = 0.01


class GripperManager():
    def __init__(self):
        self._grippers = [Servo("gripper1"), Servo("gripper2")]

    def graspBisco(self, target_gripper, grip_way):
        dist = grip_way
        self._grippers[target_gripper].move(dist)
        temp = "gripper {}: {} cm".format(target_gripper, dist)
        rospy.loginfo(temp)


    def releaseBisco(self, target_gripper):
        self._grippers[target_gripper].move(0)
        
        rospy.loginfo("release")

