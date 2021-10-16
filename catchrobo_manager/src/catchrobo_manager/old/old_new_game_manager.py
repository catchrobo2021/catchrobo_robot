#!/usr/bin/env python
# -*- coding: utf-8 -*-

from os import wait
from re import I
import pandas as pd
import numpy as np

import rospy
import rospkg
import tf
import moveit_commander

from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.msg import RobotTrajectory, PositionIKRequest
from moveit_msgs.srv import GetPositionIK


from catchrobo_manager.object_database import BiscoDatabase, ObjectDatabase
from catchrobo_manager.my_moveit_robot import MyMoveitRobot
# from catchrobo_manager.actions import Actions
from catchrobo_manager.action_planner import ActionPlanner, getName

from catchrobo_manager.bisco_manager import BiscoManager
from catchrobo_manager.brain import Brain, ActionType
from catchrobo_manager.arm import Arm
from catchrobo_manager.shooting_box_manager import ShootingBoxManager


class GripperManager():
    def graspBisco(self, param):
        pass
    def releaseBisco(self, param):
        pass

class MyRobot():
    def __init__(self):
        self._brain = Brain()
        self._arm = Arm()
        self._gripper = GripperManager()

    def doAction(self):
        action = self._brain.popAction()
        command_type = action[0]
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
            return True
        elif command_type == ActionType.GRIP:
            self._biscos.attach(action[1])
            self._biscos.delete([action[1]])
        elif command_type == ActionType.RELEASE:
            self._biscos.release(action[3])
            self._shooting_box.delete(action[2])
        return False
    
    def calcBiscoAction(self):
        self._biscos.calcTargetTwin()
        targets, is_twin = self._biscos.getTargetTwin()
        if targets[0] is None and targets[1] is None:
            return False
        self._robot.calcBiscoAction(targets, is_twin)
    
    def doBiscoAction(self):
        self.doAction()

    def calcShootAction(self):
        self._shooting_box.calcTargetTwin()
        targets, is_twin = self._shooting_box.getTargetTwin()
        if targets[0] is None and targets[1] is None:
            return False
        self._robot.calcShootAction(targets, is_twin)
    
    def doShootAction(self):
        self.doAction()



class GamePlayer():
    def __init__(self):
        
        self._catchrobo = CatchroboCenter()
        self._next_state = self.calcBiscoAction
        
    def main(self):
        self._next_state = self._next_state()
        
    def restart(self):
        pass

    def pause(self):
        pass

    def finish(self):
        pass


    def calcBiscoAction(self):
        if self._catchrobo.calcBiscoAction():
            next_state =  self.doBiscoAction
        else:
            next_state = self.manual
        return next_state
    
    def doBiscoAction(self):
        if self._catchrobo.doBiscoAction():
            next_state =  self.doBiscoAction
        else:
            next_state = self.calcShootAction
        return next_state
    
    def calcShootAction(self):
        if self._catchrobo.calcShootAction():
            next_state =  self.doShootAction
        else:
            next_state = self.manual
        return next_state

    def doShootAction(self):
        if self._catchrobo.doShootAction():
            next_state =  self.doShootAction
        else:
            next_state = self.calcBiscoAction
        return next_state

    def manual(self):
        return self.manual

class GameStatus():
    SETUP_TIME = 0
    GAME = 1
    RESTART = 2 
    MANUAL = 3
    FINISH = 4








class ActionState(object):
    BISCO = 1
    SHOOT = 3
    FINISH = 4

class GameManager(object):
    def __init__(self):
        rospy.init_node("GameManager")

        self._can_go_common = False
        self._next_action = ActionState.BISCO
        self._mymoveit = MyMoveitRobot()
        
        self._action_planner = ActionPlanner()
        self._target_biscos = [None, None]
        self._color = rospy.get_param("/color")

    def init(self):
        self.readCsvs()
        self.addBox2Scene()
        self._mymoveit.goHome()
        # self._action_planner.init(self._biscos)
        # rospy.sleep(30)
    
    def addBox2Scene(self):
        BISCO_SIZE = self._action_planner.BISCO_SIZE
        for i in range(self._action_planner.BISCO_NUM):
            if not self._biscos.isExist(i):
                continue
            p = PoseStamped()
            p.header.frame_id = "world"
            p.pose.position = self._biscos.getPosi(i)
            p.pose.position.z += BISCO_SIZE[2] / 2 + 0.0005
            p.pose.orientation.w = 1.0
            size = BISCO_SIZE[0], BISCO_SIZE[1], BISCO_SIZE[2] - 0.001
            self._mymoveit.addBox2Scene(getName(i), p, size)

    # [TODO] load from the retry point
    def readCsvs(self):
        rospack = rospkg.RosPack()
        # get the file path for rospy_tutorials
        pkg_path = rospack.get_path("catchrobo_manager")

        config_path = pkg_path + "/config/"

        bisco_csv = config_path + self._color + "_bisco.csv"
        self._biscos = BiscoDatabase(bisco_csv, "exist")
        shoot_csv = config_path + self._color + "_shoot.csv"
        self._shoots = ObjectDatabase(shoot_csv, "open")
    
    def calcTarget(self, obj, name_str):
        obj.calcTargetTwin()
        target_ids, is_twin = obj.getTargetTwin()
        rospy.loginfo("target {}: {}".format(name_str,target_ids))
        ##### grip bisco
        targets = [obj.getObj(id) for id in target_ids]
        return target_ids, is_twin, targets 

    def main(self):
        start_time = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            if self._next_action == ActionState.BISCO:
                ##### calc next bisco
                target_ids, is_twin, targets = self.calcTarget(self._biscos, "box")
                ##### if no bisco, finish
                if target_ids[0] is None and target_ids[1] is None:
                    self._next_action = ActionState.FINISH
                    continue
                ##### grip bisco
                actions = self._action_planner.biscoAction(targets, is_twin)
                self._mymoveit.setActions(actions)
                self._mymoveit.doActions()
                self._biscos.delete(target_ids)
                self._target_biscos = targets
                self._next_action = ActionState.SHOOT
                    
            elif self._next_action == ActionState.SHOOT:
                target_ids, is_twin, targets = self.calcTarget(self._shoots, "shoot")
                ##### if no bisco, finish
                if target_ids[0] is None and target_ids[1] is None:
                    self._next_action = ActionState.FINISH
                    continue

                actions = self._action_planner.shootAction(targets, self._target_biscos)
                self._mymoveit.setActions(actions)
                self._mymoveit.doActions()
                self._shoots.delete(target_ids)
                self._next_action = ActionState.BISCO
                
            elif self._next_action == ActionState.FINISH:
                break

        end_time = rospy.Time.now().to_sec()
        print("time: ", end_time - start_time)
        # rate.sleep()

    # [TODO]
    def checkGoCommon(self):
        if not self._can_go_common:
            self._can_go_common = (
                self._box[self._box["open"]]["priority"].max() <= -3
            )  # 0,1,2が埋まればcommonに入ってよい
        return self._can_go_common



if __name__ == "__main__":
    game_manager = GameManager()
    game_manager.init()
    game_manager.main()
