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
    PERMISSION = 3
    SHOOT_PERMISSION = 4


class CatchroboCenter():
    def __init__(self):
        self._color = rospy.get_param("/color")
        self._robot = MyRobot(self._color)

        self._biscos = BiscoManager(self._color)
        self._shooting_box = ShootingBoxManager(self._color)
        self._obstacle = Obstacle(self._color)

        self._can_go_common = False
    
        self._next_state = self.calcBiscoAction
        self.END_BISCO_ID = 23
        self.makeEndPose()

    def makeEndPose(self):
        target_bisco = self._biscos._database.getObj(self.END_BISCO_ID)
        self._robot.makeEndPose(target_bisco)
        
    def init(self):
        self._robot.init()
        self._obstacle.makeCommonAreaObstacle()

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
            self._shooting_box.shoot(shooting_box_id)
        elif result.isPersmission():
            return ActionResult.PERMISSION


        return ActionResult.DOING

    def calcBiscoAction(self):
        self._biscos.calcTargetTwin()
        targets, is_twin = self._biscos.getTargetTwin()
        temp = "target bisco : {}".format(targets)
        # rospy.loginfo(temp)
        if targets[0] is None and targets[1] is None:
            return ActionResult.GAME_END
        self._robot.calcBiscoAction(targets, is_twin)

        # obstacle to prevent robot intrude common area after picking common biscos
        if not self._biscos.isCommonExist():
            self._obstacle.makeCommonAreaMiddleObstacle()
        else:
            self._obstacle.deleteCommonAreaMiddleObstacle()
        return self.doBiscoAction

    def doBiscoAction(self):
        result = self.doAction()
        if result==ActionResult.DOING:
            ret = self.doBiscoAction
        elif result == ActionResult.PERMISSION:
            ret = ActionResult.PERMISSION
        else:
            ret = self.calcShootAction
        
        return ret

    def calcShootAction(self):
        self._shooting_box.calcTargetTwin()
        targets, _ = self._shooting_box.getTargetTwin()
        temp = "target shoot : {}".format(targets)
        # rospy.loginfo(temp)
        biscos, _ = self._biscos.getTargetTwin()
        if targets[0] is None and targets[1] is None:
            return ActionResult.GAME_END
        self._robot.calcShootAction(targets, biscos)

        return self.doShootAction

    def doShootAction(self):

        # obstacle to prevent robot intrude common area before filling all shooting box
        can_go_common = self._shooting_box.canGoCommon()
        if not self._can_go_common:
            # self._can_go_common = self._shooting_box.canGoCommon()
            if can_go_common:
                self._obstacle.deleteCommonAreaObstacle()
                self._biscos.setCanGoCommon(True)
                self._robot._guide.canGoCommon()
                ret = ActionResult.SHOOT_PERMISSION
                self._can_go_common = can_go_common
                return ret
        self._can_go_common = can_go_common
        result = self.doAction()
        if result==ActionResult.DOING:
            ret = self.doShootAction
        else:
            ret = self.calcBiscoAction
        return ret

    def main(self):
        ret = self._next_state()
        if ret == ActionResult.GAME_END:
            self._next_state = self.calcBiscoAction
        elif ret == ActionResult.PERMISSION:
            self._next_state = self.doBiscoAction
        elif ret == ActionResult.SHOOT_PERMISSION:
            self._next_state = self.doShootAction
        else:
            self._next_state = ret
        return ret
    
    def end(self):
        self._robot.end()
    
    def mainStart(self):
        self._robot.mainStart()
    
    def emergencyStop(self):
        self._robot.emergencyStop()