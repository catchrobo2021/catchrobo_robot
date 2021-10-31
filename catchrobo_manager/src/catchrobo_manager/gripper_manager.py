#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from sensor_msgs.msg import JointState

from catchrobo_manager.servo import Servo, Laser



class GripperID:
    NEAR = 0
    FAR = 1

class GripWay:
    LONG_GRIP = 0
    SMALL_GRIP = 1

class GripperManager():
    def __init__(self, color):
        self._grippers = [Servo("gripper1"), Servo("gripper2")]
        self._lasers = [Laser("laser1"), Laser("laser2")]

        id_map = [0,1]
        if color == "red":
            id_map[GripperID.NEAR] = 0
            id_map[GripperID.FAR] = 1
        if color == "blue":
            id_map[GripperID.NEAR] = 1
            id_map[GripperID.FAR] = 0
        self._id_map = id_map

        self._grip_dist = rospy.get_param("grip_dist")

    def getGripDist(self, target_gripper, grip_way):
        if grip_way == GripWay.LONG_GRIP:
            ret = self._grip_dist["long"][target_gripper]
        else:
            ret = self._grip_dist["small"][target_gripper]
        return ret

    def laser(self, laser_on):
        self._lasers[0].output(laser_on)
        self._lasers[1].output(laser_on)

    def graspBisco(self, target_gripper, grip_way,wait):
        target_gripper_id = self._id_map[target_gripper]
        dist = self.getGripDist(target_gripper, grip_way)
        self._grippers[target_gripper_id].move(dist,wait)
        temp = "gripper {}: {} cm".format(target_gripper_id, dist)
        rospy.loginfo(temp)


    def releaseBisco(self, target_gripper):
        target_gripper_id = self._id_map[target_gripper]
        dist = self._grip_dist["max"]
        self._grippers[target_gripper_id].move(dist)
        
        rospy.loginfo("release")

