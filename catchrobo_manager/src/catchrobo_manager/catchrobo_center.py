#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from catchrobo_manager.bisco_manager import BiscoManager
from catchrobo_manager.shooting_box_manager import ShootingBoxManager
from catchrobo_manager.myrobot import MyRobot, ActionType

class ActionResult():
    DOING = 0
    FINISH = 1
    GAME_END = 2

class CatchroboCenter():
    def __init__(self):
        self._color = rospy.get_param("/color")
        self._biscos = BiscoManager(self._color)   
        self._shooting_box = ShootingBoxManager(self._color)   
        self._robot = MyRobot() 
        
    def doAction(self):
        action = self._robot.doAction()
        command_type = action[0]
        if command_type == ActionType.FINISH:
            return ActionResult.FINISH
        elif command_type == ActionType.GRIP:
            self._biscos.attach(action[1])
            self._biscos.delete(action[1])
        elif command_type == ActionType.RELEASE:
            self._biscos.release(action[3])
            self._shooting_box.delete(action[2])
        return ActionResult.DOING
    
    def calcBiscoAction(self):
        self._biscos.calcTargetTwin()
        targets, is_twin = self._biscos.getTargetTwin()
        if targets[0] is None and targets[1] is None:
            return ActionResult.GAME_END
        self._robot.calcBiscoAction(targets, is_twin)
        return ActionResult.DOING
    
    def doBiscoAction(self):
        return self.doAction()

    def calcShootAction(self):
        self._shooting_box.calcTargetTwin()
        targets, is_twin = self._shooting_box.getTargetTwin()
        biscos, _ = self._biscos.getTargetTwin()
        if targets[0] is None and targets[1] is None:
            return ActionResult.GAME_END
        self._robot.calcShootAction(targets, biscos)
        return ActionResult.DOING
    
    def doShootAction(self):
        return self.doAction()

