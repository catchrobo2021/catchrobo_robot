#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander

from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped

from catchrobo_manager.bisco_manager import BiscoManager
from catchrobo_manager.shooting_box_manager import ShootingBoxManager
from catchrobo_manager.myrobot import MyRobot


class ActionResult():
    DOING = 0
    FINISH = 1
    GAME_END = 2

class Obstacle:
    def __init__(self, color):
        rospy.wait_for_service("/get_planning_scene", timeout=10.0)
        rospy.wait_for_service("/apply_planning_scene", timeout=10.0)
        self._scene = moveit_commander.PlanningSceneInterface(synchronous=True)

        p = PoseStamped()
        p.header.frame_id = "world"  

        size = 0.15, 0.2, 0.5
        p.pose.position.x = 0.850
        p.pose.position.y = 0.100
        p.pose.position.z = size[2]/2 + 0.05

        # if color == "blue":
        #     p.pose.position.x = -p.pose.position.x 
        # p.pose.orientation.w = 1.0
        # rospy.sleep(0.1)
        # self._scene.add_box("avoid_back", p, size)

        # size = 0.01, 1.350, 0.5
        # p.pose.position.x = -0.153
        # if color == "blue":
        #     p.pose.position.x = -p.pose.position.x 
        # p.pose.position.y = size[1] /2
        # p.pose.position.z = size[2]/2 + 0.05        
        # p.pose.orientation.w = 1.0
        # rospy.sleep(0.1)
        # self._scene.add_box("avoid_opponent", p, size)


        

class CatchroboCenter():
    def __init__(self):
        self._color = rospy.get_param("/color")
        self._robot = MyRobot(self._color) 
        
    
        self._biscos = BiscoManager(self._color)   
        self._shooting_box = ShootingBoxManager(self._color)   
        self._obstacle = Obstacle(self._color)
        
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