#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from catchrobo_manager.bisco.bisco_manager import BiscoManager
from catchrobo_manager.shooting_box_manager import ShootingBoxManager
from catchrobo_manager.myrobot import MyRobot
from catchrobo_manager.obstacle import Obstacle


class ActionResult():
    DOING = 0
    FINISH = 1
    GAME_END = 2

        

class CatchroboCenter():
    def __init__(self):
        self._color = rospy.get_param("/color")
        self._robot = MyRobot(self._color) 
        
    
        self._biscos = BiscoManager(self._color)   
        self._shooting_box = ShootingBoxManager(self._color)   
        self._obstacle = Obstacle(self._color)

        self._can_go_common = False
        
    def doAction(self):
        result = self._robot.doAction()
        params = result.getParams()
        
        if result.isFinish():
            return ActionResult.FINISH
        elif result.isGrip():
            bisco_id = params[0]
            self._biscos.pick(bisco_id)

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

        ### obstacle to prevent robot intrude common area after picking common biscos
        if not self._biscos.isCommonExist():
            self._obstacle.makeCommonAreaMiddleObstacle()
        else:
            self._obstacle.deleteCommonAreaMiddleObstacle()
        return ActionResult.DOING
    
    def doBiscoAction(self):
        return self.doAction()

    def calcShootAction(self):
        self._shooting_box.calcTargetTwin()
        targets, _ = self._shooting_box.getTargetTwin()
        biscos, _ = self._biscos.getTargetTwin()
        if targets[0] is None and targets[1] is None:
            return ActionResult.GAME_END
        self._robot.calcShootAction(targets, biscos)

        return ActionResult.DOING
    
    def doShootAction(self):

        ### obstacle to prevent robot intrude common area before filling all shooting box
        if not self._can_go_common:
            self._can_go_common = self._shooting_box.canGoCommon()
            if self._can_go_common:
                self._obstacle.deleteCommonAreaObstacle()
                self._biscos.setCanGoCommon(True)

        return self.doAction()

    def end(self):
        self._robot.end()