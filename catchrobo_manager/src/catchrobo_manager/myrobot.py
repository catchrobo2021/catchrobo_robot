#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from catchrobo_manager.brain import Brain
from catchrobo_manager.arm import Arm
from catchrobo_manager.my_robot_result import MyRobotResultMaker
from catchrobo_manager.gripper_manager import GripperManager
from catchrobo_manager.shooter_manager import ShooterManager

class MyRobot():
    def __init__(self):
        self._brain = Brain()
        self._arm = Arm()
        self._gripper = GripperManager()
        self._shooter = ShooterManager()

        self._arm.goHome()

    def doAction(self):
        action = self._brain.popAction()
        
        action.show_action()
        params = action.getParams()
        result = MyRobotResultMaker.empty()
        if action.isMove():
            target_pose = params[0]
            self._arm.move(target_pose)

        elif action.isAbove():
            z = params[0]
            self._arm.above(z)

        elif action.isGrip():
            target_gripper, bisco, grip_way = params
            self._gripper.graspBisco(target_gripper, grip_way)
            result = MyRobotResultMaker.grip(bisco.name)

        elif action.isShoot():
            target_gripper, bisco, shooting_box = params
            self._gripper.releaseBisco(target_gripper)
            self._shooter.close(shooting_box)
            result = MyRobotResultMaker.shoot(bisco.name, shooting_box.name)

        elif action.isFinish():
            result = MyRobotResultMaker.finish()
            
        elif action.isOpenShooter():
            shooting_box = params[0]
            self._shooter.open(shooting_box)

        return result
    
    def calcBiscoAction(self, targets, is_twin):
        self._brain.calcBiscoAction(targets, is_twin)

    def calcShootAction(self, targets, is_twin):
        self._brain.calcShootAction(targets, is_twin)

    def end(self):
        self._arm.goHome()