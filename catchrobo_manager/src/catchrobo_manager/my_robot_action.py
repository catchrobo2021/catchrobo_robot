#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

class MyRobotActionEnum:
    MOVE = 0
    ABOVE = 1
    GRIP = 2
    SHOOT = 3
    FINISH = 4
    OPEN_SHOOTER = 5


class MyRobotAction:
    def __init__(self, type, params):
        self._type = type
        self._params =params
    
    def isMove(self):
        return self._type == MyRobotActionEnum.MOVE

    def isAbove(self):
        return self._type == MyRobotActionEnum.ABOVE

    def isGrip(self):
        return self._type == MyRobotActionEnum.GRIP
    
    def isShoot(self):
        return self._type == MyRobotActionEnum.SHOOT
    
    def isFinish(self):
        return self._type == MyRobotActionEnum.FINISH
    
    def isOpenShooter(self):
        return self._type == MyRobotActionEnum.OPEN_SHOOTER

    def show_action(self):
        if self.isMove():
            temp = "MOVE"
        elif self.isAbove():
            temp = "ABOVE"
        elif self.isGrip():
            temp = "GRIP"
        elif self.isShoot():
            temp = "SHOOT"
        elif self.isFinish():
            temp = "FINISH"
        elif self.isOpenShooter():
            temp = "OPEN_SHOOTER"
        rospy.loginfo("ActionType : " + temp)

    def getParams(self):
        return self._params

class MyRobotActionMaker:
    @classmethod
    def move(cls, target_pose):
        return MyRobotAction(MyRobotActionEnum.MOVE, [target_pose])
    @classmethod
    def above(cls, z):
        return MyRobotAction(MyRobotActionEnum.ABOVE, [z])
    @classmethod
    def grip(cls, target_gripper, bisco, grip_way):
        return MyRobotAction(MyRobotActionEnum.GRIP, [target_gripper, bisco, grip_way])
    
    @classmethod
    def shoot(cls,  target_gripper, bisco, shooting_box):
        return MyRobotAction(MyRobotActionEnum.SHOOT, [target_gripper, bisco, shooting_box])

    @classmethod
    def finish(cls):
        return MyRobotAction(MyRobotActionEnum.FINISH, None)

    @classmethod
    def openShooter(cls, shooting_box):
        return MyRobotAction(MyRobotActionEnum.OPEN_SHOOTER, [shooting_box])