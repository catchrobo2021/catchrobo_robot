#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from catchrobo_manager.bisco_manager import BiscoManager
from catchrobo_manager.shooting_box_manager import ShootingBoxManager
from catchrobo_manager.myrobot import MyRobot

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
        result = self._robot.doAction()
        params = result.getParams()
        
        if result.isFinish():
            return ActionResult.FINISH
        elif result.isGrip():
            bisco_id = params[0]
            self._biscos.attach(bisco_id)
            self._biscos.delete(bisco_id)

        elif result.isShoot():
            bisco_id, shooting_box_id = params
            self._biscos.release(bisco_id)
            self._shooting_box.delete(shooting_box_id)
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

    def end(self):
        self._robot.end()