#!/usr/bin/env python
# -*- coding: utf-8 -*-

import pandas as pd
import numpy as np

import rospy

from catchrobo_manager.brain import Brain, ActionType
from catchrobo_manager.arm import Arm




class GripperManager():
    def graspBisco(self, target_gripper, wait, dist):
        temp = "gripper {}: {} cm".format(target_gripper, dist)
        rospy.loginfo(temp)
    def releaseBisco(self, target_gripper):
        rospy.loginfo("release")

class MyRobot():
    def __init__(self):
        self._brain = Brain()
        self._arm = Arm()
        self._gripper = GripperManager()

        self._arm.goHome()

    def doAction(self):
        action = self._brain.popAction()
        command_type = action[0]
        ActionType.show_action(command_type)
        if command_type == ActionType.MOVE:
            self._arm.move(action[1])
        elif command_type == ActionType.ABOVE:
            self._arm.above(action[1])
        elif command_type == ActionType.GRIP:
            self._gripper.graspBisco(*action[2:])
        elif command_type == ActionType.RELEASE:
            self._gripper.releaseBisco(action[1])
        return action
    
    def calcBiscoAction(self, targets, is_twin):
        self._brain.calcBiscoAction(targets, is_twin)

    def calcShootAction(self, targets, is_twin):
        self._brain.calcShootAction(targets, is_twin)
