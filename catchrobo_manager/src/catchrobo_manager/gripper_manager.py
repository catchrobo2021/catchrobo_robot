#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from sensor_msgs.msg import JointState

class GripperNumber:
    GRIPPER1 = 0
    GRIPPER2 = 1

class GripWay:
    LONG_MOVE = 0.03
    SMALL_MOVE = 0.01


class Gripper:
    def __init__(self, name):
        self._name = name
        self._pub = rospy.Publisher("arduino_command", JointState, queue_size=10)
        self._state = JointState()
        self._state.name = [name]
        
    def move(self, dist):
        self._state.position = [dist]
        self._pub.publish(self._state)
        rospy.sleep(0.5)
    

        

class GripperManager():
    def __init__(self):
        self._grippers = [Gripper("gripper1"), Gripper("gripper2")]

    def graspBisco(self, target_gripper, grip_way):
        dist = grip_way
        self._grippers[target_gripper].move(dist)
        temp = "gripper {}: {} cm".format(target_gripper, dist)
        rospy.loginfo(temp)


    def releaseBisco(self, target_gripper):
        self._grippers[target_gripper].move(0)
        
        rospy.loginfo("release")

