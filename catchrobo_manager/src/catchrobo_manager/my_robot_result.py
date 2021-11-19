#!/usr/bin/env python
# -*- coding: utf-8 -*-


class MyRobotResultEnum:
    GRIP = 0
    SHOOT = 1
    FINISH = 2
    PERMISSION = 3

class MyRobotResult:
    def __init__(self, type, params):
        self._type = type
        self._params =params
        
    
    # def grip(self, bisco_id):
    #     self._type = MyRobotResultEnum.GRIP
    #     self._params = [bisco_id]

    # def shoot(self, bisco_id, shooting_box_id):
    #     self._type = MyRobotResultEnum.SHOOT
    #     self._params = [bisco_id, shooting_box_id]
    
    def finish(self):
        self._type = MyRobotResultEnum.FINISH
    
    def isGrip(self):
        return self._type == MyRobotResultEnum.GRIP
    
    def isShoot(self):
        return self._type == MyRobotResultEnum.SHOOT
    
    def isFinish(self):
        return self._type == MyRobotResultEnum.FINISH

    def isPersmission(self):
        return self._type == MyRobotResultEnum.PERMISSION

    def getParams(self):
        return self._params

class MyRobotResultMaker:
    @classmethod
    def grip(cls, bisco_id):
         return MyRobotResult(MyRobotResultEnum.GRIP,[bisco_id] )

    @classmethod
    def shoot(cls, bisco_id, shooting_box_id):
        return MyRobotResult(MyRobotResultEnum.SHOOT, [bisco_id, shooting_box_id])
    @classmethod
    def finish(cls):
        return MyRobotResult(MyRobotResultEnum.FINISH, None)
    @classmethod
    def empty(cls):
        return MyRobotResult(None, None)

    @classmethod 
    def permission(cls):
        return MyRobotResult(MyRobotResultEnum.PERMISSION, None)
